#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/TransformStamped.h>

#define TIME_STEP 32
#define NMOTORS 4
#define NSLAVES 2
#define MAX_SPEED 6.4

ros::NodeHandle *nodehandles[NSLAVES];
ros::CallbackQueue callbackQueues[NSLAVES];
ros::ServiceClient timeStepClient[NSLAVES];
webots_ros::set_int timeStepSrv[NSLAVES];
ros::ServiceClient set_GPS_client[NSLAVES];
webots_ros::set_int GPS_srv[NSLAVES];
ros::Subscriber sub_GPS[NSLAVES];
ros::ServiceClient set_inertial_unit_client[NSLAVES];
webots_ros::set_int inertial_unit_srv[NSLAVES];
ros::Subscriber sub_inertial_unit[NSLAVES];
ros::ServiceClient set_velocity_client;
webots_ros::set_float set_velocity_srv;

tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;

static const char *motorNames[NMOTORS] = {"front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"};
static std::string slaveNames[NSLAVES] = {"yya", "ranger"};
static double GPSValues[NSLAVES][3] = {0, 0, 0};
static double inertialUnitValues[NSLAVES][4] = {0, 0, 0, 0};
static int currentSlave = 0;

static double speeds[NMOTORS]{0.0};

void quit(int sig) {
  ROS_INFO("User stopped the 'stupidtest4' node.");
  for (int i = 0; i < NSLAVES; ++i) {
    timeStepSrv[i].request.value = 0;
    timeStepClient[i].call(timeStepSrv[i]);
  }
  ros::shutdown();
  exit(0);
}

double sigmoid(double x) {
    return (1 / (1 + exp(-x)) - 0.5) * 2;
}

void updateSpeed(double x, double y) {
    double kp1;
    double kp2;
    double l = 0.5;
    double we = atan2(y, x);
    ROS_INFO("angle: %f", we * 180 / 3.14);
    double de = sqrt(x * x + y * y);

    kp1 = MAX_SPEED * sigmoid(de);
    kp2 = 3.14 * sigmoid(we) * 5;

    double vl = kp1 + kp2 * l / 2;
    double vr = kp1 - kp2 * l / 2;
    ROS_INFO("x: %f, y % f", x, y);

    ROS_INFO("left: %f, right % f", vl, vr);
    
    if(vr > MAX_SPEED) vr = MAX_SPEED;
    if(vr < 0) vr = 0;
    if(vl > MAX_SPEED) vl = MAX_SPEED;
    if(vl < 0) vl = 0;

    if(x*x + y*y <= 3){
        vr = 0;
        vl = 0;
    }

    ROS_INFO("left: %f, right % f", vl, vr);

    for (int m = 0; m < NMOTORS; ++m){
        if(m == 0 || m == 2) {
            set_velocity_client = nodehandles[currentSlave]->serviceClient<webots_ros::set_float>(slaveNames[currentSlave] + std::string("/") + std::string(motorNames[m]) +
                                                                      std::string("/set_velocity"));
            set_velocity_srv.request.value = vl;
            if (!(set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1))
                ROS_INFO("speed Update Fail");
        }

        if(m == 1 || m == 3) {
            set_velocity_client = nodehandles[currentSlave]->serviceClient<webots_ros::set_float>(slaveNames[currentSlave] + std::string("/") + std::string(motorNames[m]) +
                                                                      std::string("/set_velocity"));
            set_velocity_srv.request.value = vr;
            if (!(set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1))
                ROS_INFO("speed Update Fail");
        }
    }
}

void broadcastTransform() {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.frame_id = "odom";
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.child_frame_id = slaveNames[currentSlave];
  transformStamped.transform.translation.x = - GPSValues[currentSlave][2];
  transformStamped.transform.translation.y = GPSValues[currentSlave][0];
  transformStamped.transform.translation.z = GPSValues[currentSlave][1];

  tf2::Quaternion q(
    inertialUnitValues[currentSlave][0], inertialUnitValues[currentSlave][1], inertialUnitValues[currentSlave][2], inertialUnitValues[currentSlave][3]);
  q = q.inverse();
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(q);
  transformStamped.transform.rotation = quat_msg;
  
  br.sendTransform(transformStamped);
  geometry_msgs::TransformStamped transformStamped1;
    
  try{
    transformStamped1 = tfBuffer->lookupTransform(slaveNames[currentSlave], "master_position", ros::Time(0));
    ROS_INFO("broadcast: No.%i, x = %f, y = %f, z = %f", currentSlave, transformStamped1.transform.translation.x, transformStamped1.transform.translation.y, transformStamped1.transform.translation.z);   
  
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what()); 
  }

  // Update speed
  double xDiff = transformStamped1.transform.translation.x;
  double yDiff = transformStamped1.transform.translation.y;

  updateSpeed(xDiff, yDiff);
  
 

}
void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr &values) {
  GPSValues[currentSlave][0] = values->latitude;
  GPSValues[currentSlave][1] = values->altitude;
  GPSValues[currentSlave][2] = values->longitude;


  broadcastTransform();

}
void inertialUnitCallback(const sensor_msgs::Imu::ConstPtr &values) {
  inertialUnitValues[currentSlave][0] = values->orientation.x;
  inertialUnitValues[currentSlave][1] = values->orientation.y;
  inertialUnitValues[currentSlave][2] = values->orientation.z;
  inertialUnitValues[currentSlave][3] = values->orientation.w;
  // ROS_INFO("%d,%d,%d,%d",inertialUnitValues[currentSlave][0],inertialUnitValues[currentSlave][1],inertialUnitValues[currentSlave][2],inertialUnitValues[currentSlave][3]);
  broadcastTransform();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "copycat4", ros::init_options::NoSigintHandler);

    for(int i = 0; i < NSLAVES; i++) {
        nodehandles[i] = new ros::NodeHandle;
        nodehandles[i]->setCallbackQueue(&callbackQueues[i]);
    }

    tfBuffer = new tf2_ros::Buffer;
    tfListener = new tf2_ros::TransformListener(*tfBuffer);
    
    ROS_INFO("Main(): I am fine");

    signal(SIGINT, quit);

    for(int i = 0; i < NSLAVES; i++) {
      currentSlave = i;

      // enable gps
      set_GPS_client[i] = nodehandles[i]->serviceClient<webots_ros::set_int>(slaveNames[i]+std::string("/gps/enable"));
      GPS_srv[i].request.value = 32;
      if (set_GPS_client[i].call(GPS_srv[i]) && GPS_srv[i].response.success) {
        sub_GPS[i] = nodehandles[i]->subscribe(slaveNames[i]+std::string("/gps/values"), 1, GPSCallback);
        while (sub_GPS[i].getNumPublishers() == 0){
        }
        ROS_INFO("GPS enabled.");
      } else {
          if (!GPS_srv[i].response.success)
            ROS_ERROR("Sampling period is not valid.");
            ROS_ERROR("Failed to enable GPS.");
          return 1;
      }

      //enable inertial unit
      set_inertial_unit_client[i] = nodehandles[i]->serviceClient<webots_ros::set_int>(slaveNames[i]+std::string("/inertial_unit/enable"));
      inertial_unit_srv[i].request.value = 32;
      if (set_inertial_unit_client[i].call(inertial_unit_srv[i]) && inertial_unit_srv[i].response.success) {
        sub_inertial_unit[i] = nodehandles[i]->subscribe(slaveNames[i]+std::string("/inertial_unit/roll_pitch_yaw"), 1, inertialUnitCallback);
        while (sub_inertial_unit[i].getNumPublishers() == 0) {
        }
        ROS_INFO("Inertial unit enabled.");
      } else {
        if (!inertial_unit_srv[i].response.success)
          ROS_ERROR("Sampling period is not valid.");
          ROS_ERROR("Failed to enable inertial unit.");
        return 1;
      }

      // position & speed
      for (int m = 0; m < NMOTORS; ++m){
          ros::ServiceClient set_position_client;
          webots_ros::set_float set_position_srv;
          set_position_client = nodehandles[i]->serviceClient<webots_ros::set_float>(slaveNames[i] + std::string("/") + std::string(motorNames[m]) +
                                                                      std::string("/set_position"));

          set_position_srv.request.value = INFINITY;
          if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
            ROS_INFO("%s: Position set to INFINITY for motor %s.", slaveNames[i].c_str(), motorNames[m]);
          else
            ROS_ERROR("%s: Failed to call service set_position on motor %s.", slaveNames[i].c_str(), motorNames[m]);


          
          set_velocity_client = nodehandles[i]->serviceClient<webots_ros::set_float>(slaveNames[i] + std::string("/") + std::string(motorNames[m]) +
                                                                      std::string("/set_velocity"));

          set_velocity_srv.request.value = 0;
          if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
          ROS_INFO("%s: Velocity set to 0 for motor %s.", slaveNames[i].c_str(), motorNames[m]);
          else
          ROS_ERROR("%s: Failed to call service set_velocity on motor %s.", slaveNames[i].c_str(), motorNames[m]);
      }

        timeStepClient[i] = nodehandles[i]->serviceClient<webots_ros::set_int>(slaveNames[i] + "/robot/time_step");
        timeStepSrv[i].request.value = TIME_STEP;
    }


    // main loop
    while (ros::ok()) {
      bool normal = true;
      
      for(int i = 0; i < NSLAVES; i++) {
          if (!timeStepClient[i].call(timeStepSrv[i]) || !timeStepSrv[i].response.success) {
            ROS_ERROR("Failed to call service time_step for next step.");
            normal = false;
            break;
          }
          
          timeStepSrv[i].request.value = 0;
          timeStepClient[i].call(timeStepSrv[i]);
      }

      if(!normal) break;
      for(int i = 0; i < NSLAVES; i++) {
        currentSlave = i;
        callbackQueues[i].callOne(ros::WallDuration());
      }
    }
    for(int i = 0; i < NSLAVES; i++) {
      timeStepSrv[i].request.value = 0;
      timeStepClient[i].call(timeStepSrv[i]);
    }
    
    ros::shutdown();
    return 0;


}