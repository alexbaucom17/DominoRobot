#include "ros/ros.h"
#include "std_msgs/String.h"
#include "marvelmind_nav/hedge_pos.h"
#include "marvelmind_nav/hedge_pos_a.h"
#include "marvelmind_nav/hedge_pos_ang.h"
#include "marvelmind_nav/beacon_pos_a.h"
#include "marvelmind_nav/hedge_imu_raw.h"
#include "marvelmind_nav/hedge_imu_fusion.h"
#include "marvelmind_nav/beacon_distance.h"
#include <visualization_msgs/Marker.h>

#define ROS_NODE_NAME "subscriber_test"
#define HEDGE_POSITION_TOPIC_NAME "/hedge_pos"
#define HEDGE_POSITION_ADDRESSED_TOPIC_NAME "/hedge_pos_a"
#define HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME "/hedge_pos_ang"

#define BEACONS_POSITION_ADDRESSED_TOPIC_NAME "/beacons_pos_a"

#define HEDGE_IMU_RAW_TOPIC_NAME "/hedge_imu_raw"
#define HEDGE_IMU_FUSION_TOPIC_NAME "/hedge_imu_fusion"

#define BEACON_RAW_DISTANCE_TOPIC_NAME "/beacon_raw_distance"

#define SCALE_HEDGE 3.0

ros::Publisher rviz_marker_pub;

uint32_t rviz_shape;

float orientation_qx= 0.0;
float orientation_qy= 0.0;
float orientation_qz= 0.0;
float orientation_qw= 1.0;

typedef enum {objHedge, objBeacon} DrawObjectType;

void showRvizObject(uint8_t address, float x, float y, float z, DrawObjectType obj) 
{uint8_t lifeTime, i;
 
	if (rviz_marker_pub.getNumSubscribers() < 1) return;
	
	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = address;

    // Set the marker type
    marker.type = rviz_shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    if (obj == objHedge)
      {
        marker.pose.orientation.x = orientation_qx;
        marker.pose.orientation.y = orientation_qy;
        marker.pose.orientation.z = orientation_qz;
        marker.pose.orientation.w = orientation_qw;
      }
     else
      {
		marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
      }   

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.05*SCALE_HEDGE;
    marker.scale.y = 0.05*SCALE_HEDGE;
    marker.scale.z = 0.02*SCALE_HEDGE;
    // Set the color -- be sure to set alpha to something non-zero!
    if (obj == objHedge)
      {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
      }
     else
      {
		marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
      }   
    marker.color.a = 1.0;

    if (obj == objHedge) lifeTime= 5;
                    else lifeTime= 25;
    marker.lifetime = ros::Duration(lifeTime);
     
    rviz_marker_pub.publish(marker);
}

void hedgePosAngCallback(const marvelmind_nav::hedge_pos_ang& hedge_pos_msg)
{
  ROS_INFO("Hedgehog data: Address= %d, timestamp= %d, X=%.3f  Y= %.3f  Z=%.3f  Angle: %.1f  flags=%d", 	
				(int) hedge_pos_msg.address, 
				(int) hedge_pos_msg.timestamp_ms, 
				(float) hedge_pos_msg.x_m, (float) hedge_pos_msg.y_m, (float) hedge_pos_msg.z_m,
				(float) hedge_pos_msg.angle,  
				(int) hedge_pos_msg.flags);
				
  if ((hedge_pos_msg.flags&(1<<0))==0)
    {				
      showRvizObject(hedge_pos_msg.address,hedge_pos_msg.x_m, hedge_pos_msg.y_m, hedge_pos_msg.z_m, objHedge);
    }  
}

/*
void hedgePosCallback(const marvelmind_nav::hedge_pos_a& hedge_pos_msg)
{
  ROS_INFO("Hedgehog data: Address= %d, timestamp= %d, X=%.3f  Y= %.3f  Z=%.3f  flags=%d", 	
				(int) hedge_pos_msg.address, 
				(int) hedge_pos_msg.timestamp_ms, 
				(float) hedge_pos_msg.x_m, (float) hedge_pos_msg.y_m, (float) hedge_pos_msg.z_m,  
				(int) hedge_pos_msg.flags);
				
  if ((hedge_pos_msg.flags&(1<<0))==0)
    {				
      showRvizObject(hedge_pos_msg.address,hedge_pos_msg.x_m, hedge_pos_msg.y_m, hedge_pos_msg.z_m, objHedge);
    }  
}

void hedgePos_noaddressCallback(const marvelmind_nav::hedge_pos& hedge_pos_msg)
{
  ROS_INFO("Hedgehog data: Timestamp= %d, X=%.3f  Y= %.3f  Z=%.3f  flags=%d", 	
				(int) hedge_pos_msg.timestamp_ms, 
				(float) hedge_pos_msg.x_m, (float) hedge_pos_msg.y_m, (float) hedge_pos_msg.z_m,  
				(int) hedge_pos_msg.flags);
				
  if ((hedge_pos_msg.flags&(1<<0))==0)
    {				
      showRvizObject(0,hedge_pos_msg.x_m, hedge_pos_msg.y_m, hedge_pos_msg.z_m, objHedge);
    }  
}
* */

void beaconsPosCallback(const marvelmind_nav::beacon_pos_a& beacon_pos_msg)
{
  ROS_INFO("Stationary beacon data: Address= %d, X=%.3f  Y= %.3f  Z=%.3f", 	
				(int) beacon_pos_msg.address, 
				(float) beacon_pos_msg.x_m, (float) beacon_pos_msg.y_m, (float) beacon_pos_msg.z_m);
				
  showRvizObject(beacon_pos_msg.address, beacon_pos_msg.x_m, beacon_pos_msg.y_m, beacon_pos_msg.z_m, objBeacon);
}


void IMURawCallback(const marvelmind_nav::hedge_imu_raw& hedge_imu_raw_msg)
{
	ROS_INFO("Raw IMU: Timestamp: %08d, aX=%05d aY=%05d aZ=%05d  gX=%05d gY=%05d gZ=%05d  cX=%05d cY=%05d cZ=%05d", 	
				(int) hedge_imu_raw_msg.timestamp_ms,
				(int) hedge_imu_raw_msg.acc_x, (int) hedge_imu_raw_msg.acc_y, (int) hedge_imu_raw_msg.acc_z,
				(int) hedge_imu_raw_msg.gyro_x, (int) hedge_imu_raw_msg.gyro_y, (int) hedge_imu_raw_msg.gyro_z,
				(int) hedge_imu_raw_msg.compass_x, (int) hedge_imu_raw_msg.compass_y, (int) hedge_imu_raw_msg.compass_z);
}

void IMUFusionCallback(const marvelmind_nav::hedge_imu_fusion& hedge_imu_fusion_msg)
{
	ROS_INFO("IMU fusion: Timestamp: %08d, X=%.3f  Y= %.3f  Z=%.3f  q=%.3f,%.3f,%.3f,%.3f v=%.3f,%.3f,%.3f  a=%.3f,%.3f,%.3f", 	
				(int) hedge_imu_fusion_msg.timestamp_ms,
				(float) hedge_imu_fusion_msg.x_m, (float) hedge_imu_fusion_msg.y_m, (float) hedge_imu_fusion_msg.z_m,
				(float) hedge_imu_fusion_msg.qw, (float) hedge_imu_fusion_msg.qx, (float) hedge_imu_fusion_msg.qy, (float) hedge_imu_fusion_msg.qz,
				(float) hedge_imu_fusion_msg.vx, (float) hedge_imu_fusion_msg.vy, (float) hedge_imu_fusion_msg.vz,
				(float) hedge_imu_fusion_msg.ax, (float) hedge_imu_fusion_msg.ay, (float) hedge_imu_fusion_msg.az);
				
   orientation_qx= hedge_imu_fusion_msg.qx;
   orientation_qy= hedge_imu_fusion_msg.qy;
   orientation_qz= hedge_imu_fusion_msg.qz;
   orientation_qw= hedge_imu_fusion_msg.qw;
}

void RawDistanceCallback(const marvelmind_nav::beacon_distance& beacon_raw_distance_msg)
{
	ROS_INFO("Raw distance: %02d ==> %02d,  Distance= %.3f ", 	
				(int) beacon_raw_distance_msg.address_hedge,
				(int) beacon_raw_distance_msg.address_beacon,
				(float) beacon_raw_distance_msg.distance_m);
}


/**
 * Test subscriber node for getting data from Marvelmind publishers nodes
 */
int main(int argc, char **argv)
{
	
  // initialize ROS node
  ros::init(argc, argv, ROS_NODE_NAME);
  
  // ROS node reference 
  ros::NodeHandle rosNode;

  // Declare need to subscribe data from topic
  ros::Subscriber subHedgeWithAngle = rosNode.subscribe(HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME, 1000, hedgePosAngCallback);
  //ros::Subscriber subHedge = rosNode.subscribe(HEDGE_POSITION_ADDRESSED_TOPIC_NAME, 1000, hedgePosCallback);
  //ros::Subscriber subHedge_noaddress = rosNode.subscribe(HEDGE_POSITION_TOPIC_NAME, 1000, hedgePos_noaddressCallback);
  ros::Subscriber subBeacons = rosNode.subscribe(BEACONS_POSITION_ADDRESSED_TOPIC_NAME, 1000, beaconsPosCallback);
  ros::Subscriber subIMURaw = rosNode.subscribe(HEDGE_IMU_RAW_TOPIC_NAME, 1000, IMURawCallback);
  ros::Subscriber subIMUFusion = rosNode.subscribe(HEDGE_IMU_FUSION_TOPIC_NAME, 1000, IMUFusionCallback);
  ros::Subscriber subRawDistance = rosNode.subscribe(BEACON_RAW_DISTANCE_TOPIC_NAME, 1000, RawDistanceCallback);
  
  // Declare publisher for rviz visualization
  rviz_marker_pub = rosNode.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Set our initial shape type to be a cube
  rviz_shape = visualization_msgs::Marker::CUBE;

  ros::spin();

  return 0;
}
