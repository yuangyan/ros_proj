#include "ros/ros.h"
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/robot_get_device_list.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <signal.h>
#include <stdio.h>
#include <sensor_msgs/NavSatFix.h>

#define TIME_STEP 32
#define NMOTORS 4
#define MAX_SPEED 6.4
ros::NodeHandle *n;
static double speeds[NMOTORS] = {0.0};
static const char *motorNames[NMOTORS] = {"front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"};
static double GPSValues[3] = {0, 0, 0};
static double inertialUnitValues[4] = {0, 0, 0, 0};

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

ros::ServiceClient enableKeyboardClient;
webots_ros::set_int enableKeyboardSrv;

void quit(int sig) {
  enableKeyboardSrv.request.value = 0;
  enableKeyboardClient.call(enableKeyboardSrv);
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ROS_INFO("User stopped the 'keyboard_teleop' node.");
  ros::shutdown();
  exit(0);
}

void broadcastTransform() {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.frame_id = "odom";
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.child_frame_id = "master_position";
  transformStamped.transform.translation.x = -GPSValues[2];
  transformStamped.transform.translation.y = GPSValues[0];
  transformStamped.transform.translation.z = GPSValues[1];

  tf2::Quaternion q(
    inertialUnitValues[0], inertialUnitValues[1], inertialUnitValues[2], inertialUnitValues[3]);
  q = q.inverse();
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  // ROS_INFO("RPY: %f, %f, %f", roll, pitch, yaw);


  geometry_msgs::Quaternion quat_msg = tf2::toMsg(q);
  transformStamped.transform.rotation = quat_msg;

  br.sendTransform(transformStamped);

}


void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr &values) {
  GPSValues[0] = values->latitude;
  GPSValues[1] = values->altitude;
  GPSValues[2] = values->longitude;
//   ROS_INFO("%f, %f, %f", GPSValues[0], GPSValues[1], GPSValues[2]);
  broadcastTransform();
}

void inertialUnitCallback(const sensor_msgs::Imu::ConstPtr &values) {
  inertialUnitValues[0] = values->orientation.x;
  inertialUnitValues[1] = values->orientation.y;
  inertialUnitValues[2] = values->orientation.z;
  inertialUnitValues[3] = values->orientation.w;
  broadcastTransform();
}


void Forward() {

  for(int i = 0; i < NMOTORS; i++) {
      speeds[i] = MAX_SPEED;
      ros::ServiceClient set_velocity_client;
      webots_ros::set_float set_velocity_srv;
      set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("master/") + std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));
      set_velocity_srv.request.value = speeds[i];
      set_velocity_client.call(set_velocity_srv);
  }
}

void Backward() {

  for(int i = 0; i < NMOTORS; i++) {
      speeds[i] = - MAX_SPEED;
      ros::ServiceClient set_velocity_client;
      webots_ros::set_float set_velocity_srv;
      set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("master/") + std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));
      set_velocity_srv.request.value = speeds[i];
      set_velocity_client.call(set_velocity_srv);
  }
}

void Brake() {

  for(int i = 0; i < NMOTORS; i++) {
      speeds[i] = 0;
      ros::ServiceClient set_velocity_client;
      webots_ros::set_float set_velocity_srv;
      set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("master/") + std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));
      set_velocity_srv.request.value = speeds[i];
      set_velocity_client.call(set_velocity_srv);
  }
}

void Turnleft() {

  for(int i = 0; i < NMOTORS; i++) {
      if(i == 0 || i == 2) {
          speeds[i] = 0;
          speeds[i + 1] = MAX_SPEED;
      }

      ros::ServiceClient set_velocity_client;
      webots_ros::set_float set_velocity_srv;
      set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("master/") + std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));
      set_velocity_srv.request.value = speeds[i];
      set_velocity_client.call(set_velocity_srv);
  }
}

void Turnright() {

  for(int i = 0; i < NMOTORS; i++) {
      if(i == 1 || i == 3) {
          speeds[i] = 0;
          speeds[i - 1] = MAX_SPEED;
      }

      ros::ServiceClient set_velocity_client;
      webots_ros::set_float set_velocity_srv;
      set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("master/") + std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));
      set_velocity_srv.request.value = speeds[i];
      set_velocity_client.call(set_velocity_srv);

      if (!set_velocity_client.call(set_velocity_srv)  || !set_velocity_srv.response.success)
      ROS_ERROR("Failed to send new position commands to the robot.");
  }
}

void keyboardCallback(const webots_ros::Int32Stamped::ConstPtr &value) {
  int key = value->data;

  switch (key) {
    case 314:
      Turnleft();
      break;
    case 316:
      Turnright();
      break;
    case 315:
      Forward();

      break;
    case 317:
      Backward();
      break;

    case 32:
      Brake();
      break;

    case 312:
      ROS_INFO("END.");
      quit(-1);
      break;
      
    default:

      break;
  }
  
}

void Setposition() {
  for (int i = 0; i < NMOTORS; ++i) {
        // position
        ros::ServiceClient set_position_client;
        webots_ros::set_float set_position_srv;
        set_position_client = n->serviceClient<webots_ros::set_float>(std::string("master/") + std::string(motorNames[i]) +
                                                                    std::string("/set_position"));

        set_position_srv.request.value = INFINITY;
        if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
        ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);
        else
        ROS_ERROR(": Failed to call service set_position on motor %s.", motorNames[i]);

        // speed
        ros::ServiceClient set_velocity_client;
        webots_ros::set_float set_velocity_srv;
        set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("master/") + std::string(motorNames[i]) +
                                                                    std::string("/set_velocity"));

        set_velocity_srv.request.value = 0;
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
        ROS_INFO("Velocity set to 0 for motor %s.", motorNames[i]);
        else
        ROS_ERROR(" Failed to call service set_velocity on motor %s.", motorNames[i]);
      }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard", ros::init_options::NoSigintHandler);
    n = new ros::NodeHandle;
    

    signal(SIGINT, quit);
    Setposition();
    // enable gps
    ros::ServiceClient set_GPS_client;
    webots_ros::set_int GPS_srv;
    ros::Subscriber sub_GPS;
    set_GPS_client = n->serviceClient<webots_ros::set_int>("master/gps/enable");
    GPS_srv.request.value = 32;
    if (set_GPS_client.call(GPS_srv) && GPS_srv.response.success) {
      sub_GPS = n->subscribe("master/gps/values", 1, GPSCallback);
      while (sub_GPS.getNumPublishers() == 0) {
      }
      ROS_INFO("GPS enabled.");
    } else {
      if (!GPS_srv.response.success)
        ROS_ERROR("Sampling period is not valid.");
      ROS_ERROR("Failed to enable GPS.");
      return 1;
    }

    // enable inertial unit
    ros::ServiceClient set_inertial_unit_client;
    webots_ros::set_int inertial_unit_srv;
    ros::Subscriber sub_inertial_unit;
    set_inertial_unit_client = n->serviceClient<webots_ros::set_int>("master/inertial_unit/enable");
    inertial_unit_srv.request.value = 32;
    if (set_inertial_unit_client.call(inertial_unit_srv) && inertial_unit_srv.response.success) {
      sub_inertial_unit = n->subscribe("master/inertial_unit/roll_pitch_yaw", 1, inertialUnitCallback);
      while (sub_inertial_unit.getNumPublishers() == 0) {
      }
      ROS_INFO("Inertial unit enabled.");
    } else {
      if (!inertial_unit_srv.response.success)
        ROS_ERROR("Sampling period is not valid.");
      ROS_ERROR("Failed to enable inertial unit.");
      return 1;
    }


    timeStepSrv.request.value = TIME_STEP;

    timeStepClient = n->serviceClient<webots_ros::set_int>("master/robot/time_step");
  timeStepSrv.request.value = TIME_STEP;

  enableKeyboardClient = n->serviceClient<webots_ros::set_int>("master/keyboard/enable");
  enableKeyboardSrv.request.value = TIME_STEP;
  if (enableKeyboardClient.call(enableKeyboardSrv) && enableKeyboardSrv.response.success) {
    ros::Subscriber sub_keyboard;
    sub_keyboard = n->subscribe("master/keyboard/key", 1, keyboardCallback);
    while (sub_keyboard.getNumPublishers() == 0) {
    }
    ROS_INFO("Keyboard enabled.");
    ROS_INFO("Use the arrows in Webots window to move the robot.");
    ROS_INFO("Press the End key to stop the node.");

    // main loop
    while (ros::ok()) {
      ros::spinOnce();
      if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
        ROS_ERROR("Failed to call service time_step for next step.");
        break;
      }
      ros::spinOnce();
        
    }
  } else
    ROS_ERROR("Could not enable keyboard, success = %d.", enableKeyboardSrv.response.success);

  enableKeyboardSrv.request.value = 0;
  if (!enableKeyboardClient.call(enableKeyboardSrv) || !enableKeyboardSrv.response.success)
    ROS_ERROR("Could not disable keyboard, success = %d.", enableKeyboardSrv.response.success);
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  return (0);
    


  }




  
  
  


