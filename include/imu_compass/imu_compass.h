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

	tf::TransformListener listener_;
	ros::Timer debug_timer_;

	void imuCallback(const sensor_msgs::ImuConstPtr& data);
	void magCallback(const geometry_msgs::Vector3StampedConstPtr& data);
	void debugCallback(const ros::TimerEvent&);
	void repackageImuPublish(tf::StampedTransform);

	//Heading Filter functions
	void initFilter(double heading_meas); //initialize heading fiter
	bool first_mag_reading_; //signifies receiving the first magnetometer message
	bool first_gyro_reading_; //signifies receiving the first gyroscope message
	bool filter_initialized_; //after receiving the first measurement, make sure the filter is initialized
	bool gyro_update_complete_; //sigfnifies that a gyro update (motion model update) has gone through

	double mag_zero_x_, mag_zero_y_, mag_zero_z_;

	sensor_msgs::Imu curr_imu_reading_;

	//Heading Filter Variables

	//State and Variance
	double curr_heading_;
	double curr_heading_variance_;
	double sensor_timeout_;

	//Motion Update Variables
	double heading_prediction_;
	double heading_variance_prediction_;
	double heading_prediction_variance_;
	double last_motion_update_time_;
	double last_measurement_update_time_;

	//Measurement Update Variables
	double yaw_meas_variance_;

public:
	IMUCompass(ros::NodeHandle &n);
	~IMUCompass() {
	 }
};

