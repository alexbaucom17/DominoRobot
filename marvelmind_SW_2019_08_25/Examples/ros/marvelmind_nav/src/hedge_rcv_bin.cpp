#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "marvelmind_nav/hedge_pos.h"
#include "marvelmind_nav/hedge_pos_a.h"
#include "marvelmind_nav/hedge_pos_ang.h"
#include "marvelmind_nav/beacon_pos_a.h"
#include "marvelmind_nav/hedge_imu_raw.h"
#include "marvelmind_nav/hedge_imu_fusion.h"
#include "marvelmind_nav/beacon_distance.h"
extern "C" 
{
#include "marvelmind_nav/marvelmind_hedge.h"
}

#include <sstream>

#define ROS_NODE_NAME "hedge_rcv_bin"
#define HEDGE_POSITION_TOPIC_NAME "/hedge_pos"
#define HEDGE_POSITION_ADDRESSED_TOPIC_NAME "/hedge_pos_a"
#define HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME "/hedge_pos_ang"
#define BEACONS_POSITION_ADDRESSED_TOPIC_NAME "/beacons_pos_a"
#define HEDGE_IMU_RAW_TOPIC_NAME "/hedge_imu_raw"
#define HEDGE_IMU_FUSION_TOPIC_NAME "/hedge_imu_fusion"
#define BEACON_RAW_DISTANCE_TOPIC_NAME "/beacon_raw_distance"

struct MarvelmindHedge * hedge= NULL;

static uint32_t hedge_timestamp_prev= 0;
marvelmind_nav::hedge_pos hedge_pos_noaddress_msg;// hedge coordinates message (old version without address) for publishing to ROS topic
marvelmind_nav::hedge_pos_a hedge_pos_msg;// hedge coordinates message for publishing to ROS topic
marvelmind_nav::hedge_pos_ang hedge_pos_ang_msg;// hedge coordinates and angle message for publishing to ROS topic
marvelmind_nav::beacon_pos_a beacon_pos_msg;// stationary beacon coordinates message for publishing to ROS topic
marvelmind_nav::hedge_imu_raw hedge_imu_raw_msg;// raw IMU data message for publishing to ROS topic
marvelmind_nav::hedge_imu_fusion hedge_imu_fusion_msg;// IMU fusion data message for publishing to ROS topic
marvelmind_nav::beacon_distance beacon_raw_distance_msg;// Raw distance message for publishing to ROS topic

static sem_t *sem;
struct timespec ts;

////////////////////////////////////////////////////////////////////////

void semCallback()
{
	sem_post(sem);
}

static int hedgeReceivePrepare(int argc, char **argv)
{
	 // get port name from command line arguments (if specified)
    const char * ttyFileName;
    if (argc==2) ttyFileName=argv[1];
    else ttyFileName=DEFAULT_TTY_FILENAME;
    
    // Init
    hedge=createMarvelmindHedge ();
    if (hedge==NULL)
    {
        ROS_INFO ("Error: Unable to create MarvelmindHedge");
        return -1;
    }
    hedge->ttyFileName=ttyFileName;
    hedge->verbose=true; // show errors and warnings
    hedge->anyInputPacketCallback= semCallback;
    startMarvelmindHedge (hedge);
}

static bool hedgeReceiveCheck(void)
{
  if (hedge->haveNewValues_)
    {
        struct PositionValue position;
        getPositionFromMarvelmindHedge (hedge, &position);
        
        hedge_pos_msg.address= position.address;
        hedge_pos_ang_msg.address= position.address;
        
        hedge_pos_msg.flags= position.flags;
        hedge_pos_noaddress_msg.flags= position.flags;
        hedge_pos_ang_msg.flags= position.flags;
        if (hedge_pos_msg.flags&(1<<1))// flag of timestamp format 
          {
			hedge_pos_msg.timestamp_ms= position.timestamp;// msec
			hedge_pos_noaddress_msg.timestamp_ms= position.timestamp;
		  }	
	     else 
	      {
            hedge_pos_msg.timestamp_ms= position.timestamp*15.625;// alpha-cycles ==> msec
            hedge_pos_noaddress_msg.timestamp_ms= position.timestamp*15.625;
          } 
        hedge_pos_ang_msg.timestamp_ms= position.timestamp;
          
        hedge_pos_msg.x_m= position.x/1000.0; 
        hedge_pos_msg.y_m= position.y/1000.0; 
        hedge_pos_msg.z_m= position.z/1000.0; 
        
        hedge_pos_noaddress_msg.x_m= position.x/1000.0; 
        hedge_pos_noaddress_msg.y_m= position.y/1000.0; 
        hedge_pos_noaddress_msg.z_m= position.z/1000.0;
        
        hedge_pos_ang_msg.x_m= position.x/1000.0; 
        hedge_pos_ang_msg.y_m= position.y/1000.0; 
        hedge_pos_ang_msg.z_m= position.z/1000.0;
        
        hedge_pos_ang_msg.angle= position.angle;
        
        hedge->haveNewValues_=false;
        
        return true;
    }
   return false;
}

static bool beaconReceiveCheck(void)
{
  uint8_t i;
  struct StationaryBeaconsPositions positions;
  struct StationaryBeaconPosition *bp= NULL;
  bool foundUpd= false;
  uint8_t n;
	
  getStationaryBeaconsPositionsFromMarvelmindHedge (hedge, &positions);
  n= positions.numBeacons;
  if (n == 0) 
	return false;
  
  for(i=0;i<n;i++)
  {
	  bp= &positions.beacons[i];
	  if (bp->updatedForMsg)
	  {
		  clearStationaryBeaconUpdatedFlag(hedge, bp->address);
		  foundUpd= true;
		  break;
	  } 
  }
  if (!foundUpd)
	return false;
  if (bp == NULL) 
	return false;
  	      
  beacon_pos_msg.address= bp->address;
  beacon_pos_msg.x_m= bp->x/1000.0; 
  beacon_pos_msg.y_m= bp->y/1000.0; 
  beacon_pos_msg.z_m= bp->z/1000.0; 
  
  return true;
}

static bool hedgeIMURawReceiveCheck(void)
{
  if (!hedge->rawIMU.updated)
     return false;
     
  hedge_imu_raw_msg.acc_x= hedge->rawIMU.acc_x;
  hedge_imu_raw_msg.acc_y= hedge->rawIMU.acc_y;
  hedge_imu_raw_msg.acc_z= hedge->rawIMU.acc_z;
  
  hedge_imu_raw_msg.gyro_x= hedge->rawIMU.gyro_x;
  hedge_imu_raw_msg.gyro_y= hedge->rawIMU.gyro_y;
  hedge_imu_raw_msg.gyro_z= hedge->rawIMU.gyro_z;
  
  hedge_imu_raw_msg.compass_x= hedge->rawIMU.compass_x;
  hedge_imu_raw_msg.compass_y= hedge->rawIMU.compass_y;
  hedge_imu_raw_msg.compass_z= hedge->rawIMU.compass_z;
  
  hedge_imu_raw_msg.timestamp_ms= hedge->rawIMU.timestamp;
  
  hedge->rawIMU.updated= false;
  
  return true;
}

static bool hedgeIMUFusionReceiveCheck(void)
{
  if (!hedge->fusionIMU.updated)
     return false;
     
  hedge_imu_fusion_msg.x_m= hedge->fusionIMU.x/1000.0;
  hedge_imu_fusion_msg.y_m= hedge->fusionIMU.y/1000.0;
  hedge_imu_fusion_msg.z_m= hedge->fusionIMU.z/1000.0;
  
  hedge_imu_fusion_msg.qw= hedge->fusionIMU.qw/10000.0;
  hedge_imu_fusion_msg.qx= hedge->fusionIMU.qx/10000.0;
  hedge_imu_fusion_msg.qy= hedge->fusionIMU.qy/10000.0;
  hedge_imu_fusion_msg.qz= hedge->fusionIMU.qz/10000.0;
  
  hedge_imu_fusion_msg.vx= hedge->fusionIMU.vx/1000.0;
  hedge_imu_fusion_msg.vy= hedge->fusionIMU.vy/1000.0;
  hedge_imu_fusion_msg.vz= hedge->fusionIMU.vz/1000.0;
  
  hedge_imu_fusion_msg.ax= hedge->fusionIMU.ax/1000.0;
  hedge_imu_fusion_msg.ay= hedge->fusionIMU.ay/1000.0;
  hedge_imu_fusion_msg.az= hedge->fusionIMU.az/1000.0;
  
  hedge_imu_fusion_msg.timestamp_ms= hedge->fusionIMU.timestamp;
  
  hedge->fusionIMU.updated= false;
  
  return true;
}

static void getRawDistance(uint8_t index)
{   
  beacon_raw_distance_msg.address_hedge= hedge->rawDistances.address_hedge;
  beacon_raw_distance_msg.address_beacon= hedge->rawDistances.distances[index].address_beacon;
  beacon_raw_distance_msg.distance_m= hedge->rawDistances.distances[index].distance/1000.0;
}


/**
 * Node for Marvelmind hedgehog binary streaming data processing
 */
int main(int argc, char **argv)
{uint8_t beaconReadIterations;
 uint8_t rawDistanceReadIterations;	
  // initialize ROS node
  ros::init(argc, argv, ROS_NODE_NAME);
  
  sem = sem_open(DATA_INPUT_SEMAPHORE, O_CREAT, 0777, 0);
  
  // prepare hedgehog data receiver module
  hedgeReceivePrepare(argc, argv);

  // ROS node reference 
  ros::NodeHandle n;

  // Register topics for puplishing messages
  ros::Publisher hedge_pos_ang_publisher = n.advertise<marvelmind_nav::hedge_pos_ang>(HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME, 1000);
  ros::Publisher hedge_pos_publisher = n.advertise<marvelmind_nav::hedge_pos_a>(HEDGE_POSITION_ADDRESSED_TOPIC_NAME, 1000);
  ros::Publisher hedge_pos_noaddress_publisher = n.advertise<marvelmind_nav::hedge_pos>(HEDGE_POSITION_TOPIC_NAME, 1000);
  
  ros::Publisher beacons_pos_publisher = n.advertise<marvelmind_nav::beacon_pos_a>(BEACONS_POSITION_ADDRESSED_TOPIC_NAME, 1000);
  
  ros::Publisher hedge_imu_raw_publisher = n.advertise<marvelmind_nav::hedge_imu_raw>(HEDGE_IMU_RAW_TOPIC_NAME, 1000);
  ros::Publisher hedge_imu_fusion_publisher = n.advertise<marvelmind_nav::hedge_imu_fusion>(HEDGE_IMU_FUSION_TOPIC_NAME, 1000);
  
  ros::Publisher beacon_distance_publisher = n.advertise<marvelmind_nav::beacon_distance>(BEACON_RAW_DISTANCE_TOPIC_NAME, 1000);


  // 200 Hz 
  ros::Rate loop_rate(200);

  // default values for position message
  hedge_pos_ang_msg.address= 0;
  hedge_pos_ang_msg.timestamp_ms = 0;
  hedge_pos_ang_msg.x_m = 0.0;
  hedge_pos_ang_msg.y_m = 0.0;
  hedge_pos_ang_msg.z_m = 0.0;
  hedge_pos_ang_msg.flags = (1<<0);// 'data not available' flag
  hedge_pos_ang_msg.angle= 0.0;
  
  hedge_pos_msg.address= 0;
  hedge_pos_msg.timestamp_ms = 0;
  hedge_pos_msg.x_m = 0.0;
  hedge_pos_msg.y_m = 0.0;
  hedge_pos_msg.z_m = 0.0;
  hedge_pos_msg.flags = (1<<0);// 'data not available' flag
  
  hedge_pos_noaddress_msg.timestamp_ms = 0;
  hedge_pos_noaddress_msg.x_m = 0.0;
  hedge_pos_noaddress_msg.y_m = 0.0;
  hedge_pos_noaddress_msg.z_m = 0.0;
  hedge_pos_noaddress_msg.flags = (1<<0);// 'data not available' flag
  
  beacon_pos_msg.address= 0;
  beacon_pos_msg.x_m = 0.0;
  beacon_pos_msg.y_m = 0.0;
  beacon_pos_msg.z_m = 0.0;

  
  while (ros::ok())
  {
    if (hedge->terminationRequired)
      {
		  break;
      }	
       
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
     {
        ROS_INFO("clock_gettime");
        return -1;
	 }
    ts.tv_sec += 2;
    sem_timedwait(sem,&ts);  
	  
    if (hedgeReceiveCheck())
     {// hedgehog data received
		ROS_INFO("Address: %d, timestamp: %d, %d, X=%.3f  Y= %.3f  Z=%.3f  Angle: %.1f  flags=%d", 	
				(int) hedge_pos_ang_msg.address,
				(int) hedge_pos_ang_msg.timestamp_ms, 
				(int) (hedge_pos_ang_msg.timestamp_ms - hedge_timestamp_prev),
				(float) hedge_pos_ang_msg.x_m, (float) hedge_pos_ang_msg.y_m, (float) hedge_pos_ang_msg.z_m, 
				(float) hedge_pos_ang_msg.angle,
				(int) hedge_pos_msg.flags);
		hedge_pos_ang_publisher.publish(hedge_pos_ang_msg);
        hedge_pos_publisher.publish(hedge_pos_msg);
        hedge_pos_noaddress_publisher.publish(hedge_pos_noaddress_msg);
        
        hedge_timestamp_prev= hedge_pos_ang_msg.timestamp_ms;
     }   
     
    beaconReadIterations= 0; 
    while(beaconReceiveCheck())
     {// stationary beacons data received
		ROS_INFO("Stationary beacon: Address: %d, X=%.3f  Y= %.3f  Z=%.3f", 	
				(int) beacon_pos_msg.address,
				(float) beacon_pos_msg.x_m, (float) beacon_pos_msg.y_m, (float) beacon_pos_msg.z_m);
        beacons_pos_publisher.publish(beacon_pos_msg);
        
        if ((beaconReadIterations++)>4)
          break;
     }
     
    if (hedgeIMURawReceiveCheck())
    {
		ROS_INFO("Raw IMU: Timestamp: %08d, aX=%05d aY=%05d aZ=%05d  gX=%05d gY=%05d gZ=%05d  cX=%05d cY=%05d cZ=%05d", 	
				(int) hedge_imu_raw_msg.timestamp_ms,
				(int) hedge_imu_raw_msg.acc_x, (int) hedge_imu_raw_msg.acc_y, (int) hedge_imu_raw_msg.acc_z,
				(int) hedge_imu_raw_msg.gyro_x, (int) hedge_imu_raw_msg.gyro_y, (int) hedge_imu_raw_msg.gyro_z,
				(int) hedge_imu_raw_msg.compass_x, (int) hedge_imu_raw_msg.compass_y, (int) hedge_imu_raw_msg.compass_z);
		hedge_imu_raw_publisher.publish(hedge_imu_raw_msg);
    } 
    
    if (hedgeIMUFusionReceiveCheck())
    {
		ROS_INFO("IMU fusion: Timestamp: %08d, X=%.3f  Y= %.3f  Z=%.3f  q=%.3f,%.3f,%.3f,%.3f v=%.3f,%.3f,%.3f  a=%.3f,%.3f,%.3f", 	
				(int) hedge_imu_fusion_msg.timestamp_ms,
				(float) hedge_imu_fusion_msg.x_m, (float) hedge_imu_fusion_msg.y_m, (float) hedge_imu_fusion_msg.z_m,
				(float) hedge_imu_fusion_msg.qw, (float) hedge_imu_fusion_msg.qx, (float) hedge_imu_fusion_msg.qy, (float) hedge_imu_fusion_msg.qz,
				(float) hedge_imu_fusion_msg.vx, (float) hedge_imu_fusion_msg.vy, (float) hedge_imu_fusion_msg.vz,
				(float) hedge_imu_fusion_msg.ax, (float) hedge_imu_fusion_msg.ay, (float) hedge_imu_fusion_msg.az);
		hedge_imu_fusion_publisher.publish(hedge_imu_fusion_msg);
    } 
    
    rawDistanceReadIterations= 0;
    if (hedge->rawDistances.updated)
     {
		for(uint8_t i=0;i<4;i++)
		{
		  getRawDistance(i);
		  if (beacon_raw_distance_msg.address_beacon != 0)
		  {
			  ROS_INFO("Raw distance: %02d ==> %02d,  Distance= %.3f ", 	
				(int) beacon_raw_distance_msg.address_hedge,
				(int) beacon_raw_distance_msg.address_beacon,
				(float) beacon_raw_distance_msg.distance_m);
			  beacon_distance_publisher.publish(beacon_raw_distance_msg);
		  }	
		} 
		hedge->rawDistances.updated= false;
	 } 

    ros::spinOnce();

    loop_rate.sleep();
  }

  // Exit
  if (hedge != NULL) 
    {
      stopMarvelmindHedge (hedge);
      destroyMarvelmindHedge (hedge);
    }
    
   sem_close(sem);

  return 0;
}
