#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"

#define PI 3.14159

class IMUCompass {

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
	IMUCompass(ros::NodeHandle &n);
	~IMUCompass() {
	 }
};

