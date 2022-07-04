ros文档：http://wiki.ros.org/

webots文档：https://cyberbotics.com/doc/reference/index

https://cyberbotics.com/doc/guide/index

相关文件在https://github.com/yuangyan/ros_proj上



1. 安装webots_ros package

   详见 http://wiki.ros.org/webots_ros

   ```shell
   cd /path/to/catkin_ws/src
   # retrieve the sources
   git clone https://github.com/cyberbotics/webots_ros.git
   # if you are not using the latest version of Webots, you need to checkout the tag corresponding to the Webots version: https://github.com/cyberbotics/webots_ros/releases
   # if you are using the development version of Webots, you need to checkout the develop branch.
   
   cd /path/to/catkin_ws
   
   # checking dependencies
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro melodic
   
   # building
   catkin_make
    
   # source this workspace (careful when also sourcing others)
   source /path/to/catkin_ws/devel/setup.bash
   ```

​	catkin_ws是你当前开发的工作目录



2. 把源代码文件拷贝到catkin_ws/src/webots_ros/src下（在https://github.com/yuangyan/ros_proj上）

   键盘控制小车节点magicKeyboard2.cpp

   ```c++
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
   ```
   
   空格键为刹车，上下左右控制前进和转向
   
   
   
   另外两个小车跟随的节点whymagicworks.cpp（在https://github.com/yuangyan/ros_proj上）
   
   ```c++
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
   ```

​		编译前在catkin_ws/src/webots_ros/CMakelists.txt添加依赖（原来内容不要动）（在https://github.com/yuangyan/ros_proj上）：

```cmake
add_executable(whymagicworks src/whymagicworks.cpp)

add_dependencies(whymagicworks webots_ros_generate_messages_cpp)

target_link_libraries(whymagicworks
	${catkin_LIBRARIES}
)

add_executable(magicKeyboard2 src/magicKeyboard2.cpp)

add_dependencies(magicKeyboard2 webots_ros_generate_messages_cpp)

target_link_libraries(magicKeyboard2
	${catkin_LIBRARIES}
)
```

编译代码

```bash
cd catkin_ws
catkin_make
```

catkin_ws是工作目录



3. 把world文件拷贝到catkin_ws/src/webots_ros/world下（在https://github.com/yuangyan/ros_proj上）

   stupidtest.wbt

   ```
   #VRML_SIM R2020b utf8
   WorldInfo {
     coordinateSystem "NUE"
   }
   Viewpoint {
     orientation -0.993061727059197 -0.11307532256968249 -0.032285874248145933 0.5599370941877719
     position -4.882206347441038 29.626504106577134 58.442549516690406
   }
   TexturedBackground {
   }
   TexturedBackgroundLight {
   }
   RectangleArena {
     floorSize 50 50
     floorTileSize 10 10
     floorAppearance Parquetry {
       type "light strip"
     }
     wallHeight 0.5
   }
   DEF PIONEER3AT Pioneer3at {
     translation 10 0.11 10
     name "Pioneer 3-AT(1)"
     controller "ros"
     controllerArgs [
       "--name=ranger"
     ]
     extensionSlot [
       InertialUnit {
       }
       GPS {
       }
     ]
   }
   DEF PIONEER3AT Pioneer3at {
     translation -10 0 -10
     controller "ros"
     controllerArgs [
       "--name=yya"
     ]
     extensionSlot [
       InertialUnit {
       }
       GPS {
       }
     ]
   }
   DEF PIONEER3AT Pioneer3at {
     name "Pioneer 3-AT(2)"
     controller "ros"
     controllerArgs [
       "--name=master"
     ]
     extensionSlot [
       InertialUnit {
       }
       GPS {
       }
     ]
   }
   
   ```
   
4. 把launch文件拷贝到catkin_ws/src/webots_ros/launch下（在https://github.com/yuangyan/ros_proj上）

   stupidtest.launch

   ```xml
   <?xml version="1.0"?>
   <launch>
     <!-- start Webots -->
     <arg name="no-gui" default="false," doc="Start Webots with minimal GUI"/>
     <include file="$(find webots_ros)/launch/webots.launch">
       <arg name="mode" value="realtime"/>
       <arg name="no-gui" value="$(arg no-gui)"/>
       <arg name="world" value="$(find webots_ros)/worlds/stupidtest.wbt"/>
     </include>
   
     <!-- <arg name="auto-close" default="false" doc="Startup mode"/>
     <node name="pioneer3at_controller" pkg="webots_ros" type="pioneer3at" required="$(arg auto-close)"/> -->
   </launch>
   
   ```

5. 启动launch文件

   ```bash
   cd catkin_ws
   source devel/setup.bash
   roslaunch webots_ros stupidtest.launch
   ```

6. 在新teminal启动编译好的节点

   启动键盘控制

   ```bash
   cd catkin_ws
   source devel/setup.bash
   rosrun webots_ros magicKeyboard2
   ```

   启动跟随

   ```bash
   cd catkin_ws
   source devel/setup.bash
   rosrun webots_ros whymagicworks
   ```

   

   

   严宇昂 2022.7.4

