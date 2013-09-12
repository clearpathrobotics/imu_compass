#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"

#define PI 3.14159

class UM6Compass {

private:
	ros::NodeHandle node_;
	ros::Subscriber imu_sub_;
	ros::Subscriber mag_sub_;
	ros::Publisher imu_pub_;
	ros::Publisher compass_pub_;
	ros::Publisher raw_compass_pub_;

	tf::TransformListener listener;
	ros::Timer debug_timer;

	void imuCallback(const sensor_msgs::ImuConstPtr& data);
	void magCallback(const geometry_msgs::Vector3StampedConstPtr& data);
	void debugCallback(const ros::TimerEvent&);
	void repackageImuPublish(tf::StampedTransform);

	//Heading Filter functions
	void initFilter(double heading_meas); //initialize heading fiter
	bool first_mag_reading; //signifies receiving the first magnetometer message
	bool first_gyro_reading; //signifies receiving the first gyroscope message
	bool filter_initialized; //after receiving the first measurement, make sure the filter is initialized
	bool gyro_update_complete; //sigfnifies that a gyro update (motion model update) has gone through

	double mag_zero_x, mag_zero_y, mag_zero_z;

	sensor_msgs::Imu curr_imu_reading;

	//Heading Filter Variables

	//State and Variance
	double curr_heading;
	double curr_heading_variance;
	double sensor_timeout;

	//Motion Update Variables
	double heading_prediction;
	double heading_variance_prediction;
	double heading_prediction_variance;
	double last_motion_update_time;
	double last_measurement_update_time;

	//Measurement Update Variables
	double yaw_meas_variance;

public:
	UM6Compass(ros::NodeHandle &n);
	~UM6Compass() {
	 }
};

UM6Compass::UM6Compass(ros::NodeHandle &n):node_(n) 
{
	//Acquire Parameters
	mag_zero_x = 0.0;
	mag_zero_y = 0.0;
	mag_zero_z = 0.0;
	node_.getParam("mag_zero_x",mag_zero_x); 
	node_.getParam("mag_zero_y",mag_zero_y);
	node_.getParam("mag_zero_z",mag_zero_z);	

	sensor_timeout = 0.5;
	yaw_meas_variance = 0.5; //TODO: Make this tunable as a parameter, not a priority
	heading_prediction_variance = 0.01;

	//Setup Subscribers
	imu_sub_ = node_.subscribe("/imu/data",1000, &UM6Compass::imuCallback, this);
	mag_sub_ = node_.subscribe("/imu/mag",1000, &UM6Compass::magCallback, this);
	imu_pub_ = node_.advertise<sensor_msgs::Imu>("/imu/data_compass",1);
	compass_pub_ = node_.advertise<std_msgs::Float32>("/imu/compass_heading",1);
	raw_compass_pub_ = node_.advertise<std_msgs::Float32>("/imu/raw_compass_heading",1);

	first_mag_reading = false;
	first_gyro_reading = false;
	gyro_update_complete = false;
	last_motion_update_time = ros::Time::now().toSec();
	debug_timer = node_.createTimer(ros::Duration(1), &UM6Compass::debugCallback,this);

	ROS_INFO("Compass Estimator Started");
}

void UM6Compass::debugCallback(const ros::TimerEvent&) { 
	if (!first_gyro_reading)
		ROS_WARN("Waiting for IMU data, no gyroscope data available)");
	if (!first_mag_reading)
 		ROS_WARN("Waiting for mag data, no magnetometer data available, Filter not initialized");

	if ((ros::Time::now().toSec() - last_motion_update_time > sensor_timeout) && first_gyro_reading) { //gyro data is coming in too slowly
		ROS_WARN("Gyroscope data being receieved too slow or not at all");
		first_gyro_reading = false;
	}

	if ((ros::Time::now().toSec() - last_measurement_update_time > sensor_timeout) && first_mag_reading) { //gyro data is coming in too slowly
		ROS_WARN("Magnetometer data being receieved too slow or not at all");
		filter_initialized = false;
		first_mag_reading = false;
	}
}

void UM6Compass::imuCallback(const sensor_msgs::ImuConstPtr& data) { 

	//Transform Data and get the yaw direction
	geometry_msgs::Vector3 gyro_vector;
	geometry_msgs::Vector3 gyro_vector_transformed;
	gyro_vector = data->angular_velocity;
//	gyro_vector.header = data->header;

	if(!first_gyro_reading) 
		first_gyro_reading = true;

	double dt = ros::Time::now().toSec() - last_motion_update_time;
	last_motion_update_time = ros::Time::now().toSec();
	tf::StampedTransform transform;

	try {
		listener.lookupTransform("base_link",data->header.frame_id,ros::Time(0),transform);
	} catch (tf::TransformException &ex) {
		ROS_WARN("Missed transform between base_link and %s",data->header.frame_id.c_str());
		return;
	}

	tf::Vector3 orig_bt;
	tf::Matrix3x3 transform_mat(transform.getRotation());
	tf::vector3MsgToTF(gyro_vector, orig_bt);
	tf::vector3TFToMsg(orig_bt*transform_mat, gyro_vector_transformed);
	double yaw_gyro_reading =  gyro_vector_transformed.z;

	//Run Motion Update
	if (filter_initialized) {
		heading_prediction = curr_heading + yaw_gyro_reading * dt;	//xp = A*x + B*u
		heading_variance_prediction = curr_heading_variance + heading_prediction_variance; //Sp = A*S*A' + R

		if (heading_prediction > 3.14159) 
			heading_prediction-=2*3.14159;
		else if(heading_prediction < -3.14159)
			heading_prediction+=2*3.14159;
		gyro_update_complete = true;
	}
	curr_imu_reading = (*data);
}


void UM6Compass::magCallback(const geometry_msgs::Vector3StampedConstPtr& data) {

	geometry_msgs::Vector3 imu_mag = data->vector; 
	geometry_msgs::Vector3 imu_mag_transformed;

	imu_mag.x = data->vector.x;
	imu_mag.y = data->vector.y;
	imu_mag.z = data->vector.z;

	last_measurement_update_time = ros::Time::now().toSec();
	tf::StampedTransform transform;
	try {
		listener.lookupTransform("base_link",data->header.frame_id, ros::Time(0),transform);

	} catch (tf::TransformException &ex) {
		ROS_WARN("Missed transform between base_link and %s",data->header.frame_id.c_str());
		return;
	}
	
	tf::Vector3 orig_bt;
	tf::Matrix3x3 transform_mat(transform.getRotation());
	tf::vector3MsgToTF(imu_mag, orig_bt);
	tf::vector3TFToMsg(orig_bt*transform_mat, imu_mag_transformed);

	//Compensate for hard iron
	double mag_x = imu_mag_transformed.x - mag_zero_x;
	double mag_y = imu_mag_transformed.y - mag_zero_y;
	double mag_z = imu_mag_transformed.z - mag_zero_z;

	tf::Quaternion q;
	tf::quaternionMsgToTF(curr_imu_reading.orientation,q);
	tf::Transform curr_imu_meas;
	curr_imu_meas = tf::Transform(q,tf::Vector3(0,0,0));
	curr_imu_meas = curr_imu_meas * transform;


	//Till Compensation
	tf::Matrix3x3 temp(curr_imu_meas.getRotation());
	double c_r, c_p, c_y;
	temp.getRPY(c_r, c_p, c_y);	
	double cos_pitch = cos(c_p);
	double sin_pitch = sin(c_p); 
	double cos_roll = cos(c_r);
	double sin_roll = sin(c_r);
	double t_mag_x = mag_x*cos_pitch + mag_z*sin_pitch;
	double t_mag_y = mag_x*sin_roll*sin_pitch + mag_y*cos_roll - mag_z*sin_roll*cos_pitch;
	double head_x = t_mag_x;
	double head_y = t_mag_y;


	//Retrieve magnetometer heading 
	double heading_meas = atan2(head_x, head_y);
	//If this is the first magnetometer reading, initialize filter
	if (!first_mag_reading) {
		//Initialize filter
		initFilter(heading_meas);		
		first_mag_reading = true;
		return;
	}
	//If gyro update (motion update) is complete, run measurement update and publish imu data
	if (gyro_update_complete) {
		double kalman_gain = heading_variance_prediction* (1/(heading_variance_prediction + yaw_meas_variance)); //K = Sp*C'*inv(C*Sp*C' + Q)
		double innovation = heading_meas - heading_prediction;
		if (abs(innovation) > PI)  //large change, signifies a wraparound. kalman filters don't like discontinuities like wraparounds, handle seperately.
			curr_heading = heading_meas;
		else
			curr_heading = 	heading_prediction + kalman_gain*(innovation); //mu = mup + K*(y-C*mup)

		curr_heading_variance = (1-kalman_gain)*heading_variance_prediction;// S = (1-K*C)*Sp

		std_msgs::Float32 raw_heading_float;
		raw_heading_float.data = heading_meas;
		raw_compass_pub_.publish(raw_heading_float);
	
		repackageImuPublish(transform);
		gyro_update_complete = false;
	}

}


void UM6Compass::repackageImuPublish(tf::StampedTransform transform)
{	

	//Get Current IMU reading and Compass heading
	tf::Quaternion imu_reading;
	tf::quaternionMsgToTF(curr_imu_reading.orientation, imu_reading);
	double compass_heading = curr_heading;

	//Transform curr_imu_reading to base_link
	tf::Transform o_imu_reading;
    o_imu_reading = tf::Transform(imu_reading, tf::Vector3(0,0,0));
    o_imu_reading = o_imu_reading * transform;
	imu_reading = o_imu_reading.getRotation();


	//Acquire Quaternion that is the difference between the two readings	
	tf::Quaternion compass_yaw = tf::createQuaternionFromRPY(0.0,0.0,compass_heading);
	tf::Quaternion diff_yaw = tf::createQuaternionFromRPY(0.0,0.0, compass_heading - tf::getYaw(imu_reading));

	//Transform the imu reading by the difference
	tf::Quaternion new_quaternion = diff_yaw * imu_reading;

	//Transform the imu reading back into imu_link
	o_imu_reading = tf::Transform(new_quaternion, tf::Vector3(0,0,0));
	o_imu_reading = o_imu_reading*(transform.inverse());
	tf::quaternionTFToMsg(o_imu_reading.getRotation(), curr_imu_reading.orientation);

	//Publish all data
	std_msgs::Float32 curr_heading_float;
	curr_heading_float.data = curr_heading;
	compass_pub_.publish(curr_heading_float);	
	imu_pub_.publish(curr_imu_reading);	
}

void UM6Compass::initFilter(double heading_meas) {
	curr_heading = heading_meas;
	curr_heading_variance = 1; //not very sure
	filter_initialized = true;
	ROS_INFO("Magnetometer data received. Compass estimator initialized");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "um6_compass");
	ros::NodeHandle node;	
	UM6Compass um6_heading_estimator(node);
	ros::spin();
	return 0;
}


