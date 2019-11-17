/*!
 *  drrobotI90_player
 *  Copyright (c) 2011, Dr Robot Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!

@mainpage
  drrobot_I90_player is a driver for motion control system on I90/Sentinel3/Hawk/H20/X80SV/Jaguar series mobile robot, available from
<a href="http://www.drrobot.com">Dr Robot </a>.
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
@verbatim
$ drrobotI90_player
@endverbatim

<hr>
@section topic ROS topics

Subscribes to (name/type):
- @b "cmd_vel"/Twist : velocity commands to differentially drive the robot.
- @b will develop other command subscribles in future, such as servo control.

Publishes to (name / type):
-@b drrobot_motor: will publish MotionInfoArray Message. Please referee the message file.
-@b drrobot_powerinfo: will publish PowerInfo Message. Please referee the message file.
-@b drrobot_ir: will publish RangeArray Message for IR sensor, and transform AD value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_sonar: will publish RangeArray Message for ultrasonic sensor, and transform value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_standardsensor: will publish StandardardSensor Message. Please referee the message file.
-@b drrobot_customsensor: will publish CustomSensor Message. Please referee the message file. Not available for standard I90/Sentinel3/Hawk/H20/X80SV robot

<hr>

@section parameters ROS parameters, please read yaml file

- @b RobotCommMethod (string) : Robot communication method, normally is "Network".
- @b RobotID (string) : specify the robot ID
- @b RobotBaseIP (string) : robot main WiFi module IP address in dot format, default is "192.168.0.201".
- @b RobotPortNum (string) : socket port number first serial port, and as default the value increased by one will be second port number.
- @b RobotSerialPort (int) : specify the serial port name if you choose serial communication in RobotCommMethod, default /dev/ttyS0"
- @b RobotType (string) : specify the robot type, now should in list: I90, Sentinel3, Hawk_H20, Jaguar, X80SV
- @b MotorDir (int) : specify the motor control direction
- @b WheelRadius (double) : wheel radius
- @b WheelDistance (double) : the distance between two driving wheels
- @b EncoderCircleCnt (int) : one circle encoder count
- @b MinSpeed (double) : minimum speed, unit is m/s.
- @b MaxSpeed (double) : maximum speed, unit is m/s.
- @b enable_ir (bool)  : Whether to enable sonar range sensors. Default: true.
- @b enable_sonar (bool)  : Whether to enable IR range sensors. Default: true.
 */

#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <drrobot_I90_player/MotorInfo.h>
#include <drrobot_I90_player/MotorInfoArray.h>
#include <drrobot_I90_player/RangeArray.h>
#include <drrobot_I90_player/Range.h>
#include <drrobot_I90_player/PowerInfo.h>
#include <drrobot_I90_player/StandardSensor.h>
#include <drrobot_I90_player/CustomSensor.h>
#include <DrRobotMotionSensorDriver.hpp>

#define MOTOR_NUM       6
#define IR_NUM          10
#define US_NUM          6
using namespace std;
using namespace DrRobot_MotionSensorDriver;

class DrRobotPlayerNode
{
public:

    ros::NodeHandle node_;

    tf::TransformBroadcaster tf_;

    ros::Publisher motorInfo_pub_;
    ros::Publisher powerInfo_pub_;
    ros::Publisher ir_pub_;
    ros::Publisher sonar_pub_;
    ros::Publisher standardSensor_pub_;
    ros::Publisher customSensor_pub_;

    ros::Subscriber cmd_vel_sub_;
    std::string robot_prefix_;

    DrRobotPlayerNode()
    {
        ros::NodeHandle private_nh("~");

        robotID_ = "drobot1";
        private_nh.getParam("RobotID",robotID_);
        ROS_INFO("I get ROBOT_ID: [%s]", robotID_.c_str());

        robotType_ = "I90";		//now fixed as I90 robot
        private_nh.getParam("RobotType",robotType_);
        ROS_INFO("I get ROBOT_Type: [%s]", robotType_.c_str());

        robotCommMethod_ = "Network";
        private_nh.getParam("RobotCommMethod",robotCommMethod_);
        ROS_INFO("I get ROBOT_CommMethod: [%s]", robotCommMethod_.c_str());

        robotIP_ = "192.168.0.201";		// you could modify the IP as your robot IP
        private_nh.getParam("RobotBaseIP",robotIP_);
        ROS_INFO("I get ROBOT_IP: [%s]", robotIP_.c_str());

        commPortNum_ = 10001;
        private_nh.getParam("RobotPortNum",commPortNum_);
        ROS_INFO("I get ROBOT_PortNum: [%d]", commPortNum_);

        robotSerialPort_ = "/dev/ttyS0";
        private_nh.getParam("RobotSerialPort",robotSerialPort_);
        ROS_INFO("I get ROBOT_SerialPort: [%s]", robotSerialPort_.c_str());

        enable_ir_ = true;
	/*
        private_nh.getParam("Enable_IR", enable_ir_);
        if (enable_ir_)
          ROS_INFO("I get Enable_IR: true");
        else
          ROS_INFO("I get Enable_IR: false");
	*/

        enable_sonar_ = true;
/*
        private_nh.getParam("Enable_US", enable_sonar_);
        if (enable_sonar_)
          ROS_INFO("I get Enable_US: true");
        else
          ROS_INFO("I get Enable_US: false");
*/
        motorDir_ = 1;
        private_nh.getParam("MotorDir", motorDir_);
        ROS_INFO("I get MotorDir: [%d]", motorDir_);

        wheelRadius_ = 0.0835;
        private_nh.getParam("WheelRadius", wheelRadius_);
        ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

        wheelDis_ = 0.32;
        private_nh.getParam("WheelDistance", wheelDis_);
        ROS_INFO("I get Wheel Distance: [%f]", wheelDis_);

        minSpeed_ = 0.1;
        private_nh.getParam("MinSpeed", minSpeed_);
        ROS_INFO("I get Min Speed: [%f]", minSpeed_);

        maxSpeed_ = 2.0;
        private_nh.getParam("MaxSpeed", maxSpeed_);
        ROS_INFO("I get Max Speed: [%f]", maxSpeed_);

        encoderOneCircleCnt_ = 1862;	// you need modify these values with your robot hardware
        private_nh.getParam("EncoderCircleCnt", encoderOneCircleCnt_);
        ROS_INFO("I get Encoder One Circle Count: [%d]", encoderOneCircleCnt_);

        if (robotCommMethod_ == "Network")
        {
          robotConfig1_.commMethod = Network;
          robotConfig2_.commMethod = Network;
        }
        else
        {
          robotConfig1_.commMethod = Serial;
          robotConfig2_.commMethod = Serial;
        }

          robotConfig1_.boardType = I90_Power;
          robotConfig2_.boardType = I90_Motion;

        robotConfig1_.portNum = commPortNum_;
        robotConfig2_.portNum = commPortNum_ + 1;

      //  strcat(robotConfig1_.robotIP,robotIP_.c_str());
	  strcpy(robotConfig1_.robotIP,robotIP_.c_str());
      //  strcat(robotConfig2_.robotIP,robotIP_.c_str());
	  strcpy(robotConfig2_.robotIP,robotIP_.c_str());

      //  strcat(robotConfig1_.serialPortName,robotSerialPort_.c_str());
	  strcpy(robotConfig1_.serialPortName,robotSerialPort_.c_str());
      //  strcat(robotConfig2_.serialPortName,robotSerialPort_.c_str());
	  strcpy(robotConfig2_.serialPortName,robotSerialPort_.c_str());
        //create publishers for sensor data information
        motorInfo_pub_ = node_.advertise<drrobot_I90_player::MotorInfoArray>("drrobot_motor", 1);
        powerInfo_pub_ = node_.advertise<drrobot_I90_player::PowerInfo>("drrobot_powerinfo", 1);
        if (enable_ir_) { ir_pub_ = node_.advertise<drrobot_I90_player::RangeArray>("drrobot_ir", 1); }
        if (enable_sonar_) { sonar_pub_ = node_.advertise<drrobot_I90_player::RangeArray>("drrobot_sonar",1); }
        standardSensor_pub_ = node_.advertise<drrobot_I90_player::StandardSensor>("drrobot_standardsensor", 1);
        customSensor_pub_ = node_.advertise<drrobot_I90_player::CustomSensor>("drrobot_customsensor", 1);


        drrobotPowerDriver_ = new DrRobotMotionSensorDriver();
        drrobotMotionDriver_ = new DrRobotMotionSensorDriver();
	drrobotPowerDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
	drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig2_);

        cntNum_ = 0;
    }

    ~DrRobotPlayerNode()
    {
    }

    int start()
    {

      int res = -1;
 
        drrobotMotionDriver_->openNetwork(robotConfig2_.robotIP,robotConfig2_.portNum);
        drrobotPowerDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);
     cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("drrobot_cmd_vel", 1, boost::bind(&DrRobotPlayerNode::cmdVelReceived, this, _1));
        return(0);
    }

    int stop()
    {
        int status = 0;
         drrobotMotionDriver_->close();
        drrobotPowerDriver_->close();
        usleep(1000000);
        return(status);
    }

    void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
      double g_vel = cmd_vel->linear.x;
      double t_vel = cmd_vel->angular.z;
      if (robotConfig1_.boardType != Jaguar)
      {
        double leftWheel = (2 * g_vel - t_vel* wheelDis_) / (2 * wheelRadius_);
        double rightWheel = (t_vel* wheelDis_ + 2 * g_vel) / (2 * wheelRadius_);

        int leftWheelCmd = motorDir_ * leftWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        int rightWheelCmd = - motorDir_ * rightWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        ROS_INFO("Received control command: [%d, %d]", leftWheelCmd,rightWheelCmd);
        drrobotMotionDriver_->sendMotorCtrlAllCmd(Velocity,leftWheelCmd, rightWheelCmd,NOCONTROL,NOCONTROL, NOCONTROL,NOCONTROL);
      }
 
    }

    void doUpdate()
    {

      if ( (robotConfig1_.boardType == I90_Power) || (robotConfig1_.boardType == Sentinel3_Power)
          || (robotConfig1_.boardType == Hawk_H20_Power) )
      {
        if (drrobotPowerDriver_->portOpen())
        {
          drrobotPowerDriver_->readPowerSensorData(&powerSensorData_);
          drrobot_I90_player::PowerInfo powerInfo;
          powerInfo.ref_vol = 1.5 * 4095 /(double)powerSensorData_.refVol;

          powerInfo.bat1_vol = (double)powerSensorData_.battery1Vol  * 8 / 4095 * powerInfo.ref_vol;
          powerInfo.bat2_vol = (double) powerSensorData_.battery2Vol * 8 / 4095 * powerInfo.ref_vol;

          powerInfo.bat1_temp = powerSensorData_.battery1Thermo;
          powerInfo.bat2_temp = powerSensorData_.battery2Thermo;

          powerInfo.dcin_vol = (double)powerSensorData_.dcINVol * 8 / 4095 * powerInfo.ref_vol;
          powerInfo.charge_path = powerSensorData_.powerChargePath;
          powerInfo.power_path = powerSensorData_.powerPath;
          powerInfo.power_status = powerSensorData_.powerStatus;

          powerInfo_pub_.publish(powerInfo);
        }
      }
      if (drrobotMotionDriver_->portOpen())
      {
        drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
        drrobotMotionDriver_->readRangeSensorData(&rangeSensorData_);
        drrobotMotionDriver_->readStandardSensorData(&standardSensorData_);

        drrobotMotionDriver_->readCustomSensorData(&customSensorData_);
              // Translate from driver data to ROS data
            cntNum_++;
              drrobot_I90_player::MotorInfoArray motorInfoArray;
              motorInfoArray.motorInfos.resize(MOTOR_NUM);
              for (uint32_t i = 0 ; i < MOTOR_NUM; ++i)
              {
                  motorInfoArray.motorInfos[i].header.stamp = ros::Time::now();
                  motorInfoArray.motorInfos[i].header.frame_id = string("drrobot_motor_");
                  motorInfoArray.motorInfos[i].header.frame_id += boost::lexical_cast<std::string>(i);
                  motorInfoArray.motorInfos[i].robot_type = robotConfig1_.boardType;
                  motorInfoArray.motorInfos[i].encoder_pos = motorSensorData_.motorSensorEncoderPos[i];
                  motorInfoArray.motorInfos[i].encoder_vel = motorSensorData_.motorSensorEncoderVel[i];
                  motorInfoArray.motorInfos[i].encoder_dir = motorSensorData_.motorSensorEncoderDir[i];
                  if (robotConfig1_.boardType == Hawk_H20_Motion)
                  {
                    motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] * 3 /4096;;
                  }
                  else if(robotConfig1_.boardType != Jaguar)
                  {
                    motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] / 728;
                  }
                  else
                  {
                    motorInfoArray.motorInfos[i].motor_current = 0.0;
                  }
                  motorInfoArray.motorInfos[i].motor_pwm = motorSensorData_.motorSensorPWM[i];
              }

              //ROS_INFO("publish motor info array");
              motorInfo_pub_.publish(motorInfoArray);


              drrobot_I90_player::RangeArray rangerArray;
              rangerArray.ranges.resize(US_NUM);
	      if(enable_sonar_)
	      {
		      for (uint32_t i = 0 ; i < US_NUM; ++i)
		      {

		          rangerArray.ranges[i].header.stamp = ros::Time::now();
		          rangerArray.ranges[i].header.frame_id = string("drrobot_sonar_");
		          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
		          rangerArray.ranges[i].range = (float)rangeSensorData_.usRangeSensor[i]/100;     //to meters

		          // around 30 degrees
		          rangerArray.ranges[i].field_of_view = 0.5236085;
		          rangerArray.ranges[i].max_range = 2.55;
		          rangerArray.ranges[i].min_range = 0;
		          rangerArray.ranges[i].radiation_type = drrobot_I90_player::Range::ULTRASOUND;
		      }

		      sonar_pub_.publish(rangerArray);
		}


	      if(enable_ir_)
	      {
		      rangerArray.ranges.resize(IR_NUM);
		      for (uint32_t i = 0 ; i < IR_NUM; ++i)
		      {
		          rangerArray.ranges[i].header.stamp = ros::Time::now();
		          rangerArray.ranges[i].header.frame_id = string("drrobot_ir_");
		          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
		          rangerArray.ranges[i].range = ad2Dis(rangeSensorData_.irRangeSensor[i]);
		          rangerArray.ranges[i].radiation_type = drrobot_I90_player::Range::INFRARED;
		      }

		      ir_pub_.publish(rangerArray);
	     }

              drrobot_I90_player::StandardSensor standardSensor;
              standardSensor.humanSensorData.resize(4);
              standardSensor.tiltingSensorData.resize(2);
              standardSensor.overHeatSensorData.resize(2);
              standardSensor.header.stamp = ros::Time::now();
              standardSensor.header.frame_id = string("drrobot_standardsensor");
              for (uint32_t i = 0; i < 4; i++)
                standardSensor.humanSensorData[i] = standardSensorData_.humanSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.tiltingSensorData[i] = standardSensorData_.tiltingSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.overHeatSensorData[i] = standardSensorData_.overHeatSensorData[i];

              standardSensor.thermoSensorData = standardSensorData_.thermoSensorData;

              standardSensor.boardPowerVol = (double)standardSensorData_.boardPowerVol * 9 /4095;
              standardSensor.servoPowerVol = (double)standardSensorData_.servoPowerVol * 9 /4095;

              if (robotConfig1_.boardType != Jaguar)
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 24 /4095;
              }
              else
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 34.498 /4095;
              }
              standardSensor.refVol = (double)standardSensorData_.refVol / 4095 * 6;
              standardSensor.potVol = (double)standardSensorData_.potVol / 4095 * 6;
              standardSensor_pub_.publish(standardSensor);

              drrobot_I90_player::CustomSensor customSensor;
              customSensor.customADData.resize(8);
              customSensor.header.stamp = ros::Time::now();
              customSensor.header.frame_id = string("drrobot_customsensor");

              for (uint32_t i = 0; i < 8; i ++)
              {
                customSensor.customADData[i] = customSensorData_.customADData[i];
              }
              customSensor.customIO = (uint8_t)(customSensorData_.customIO & 0xff);
              customSensor_pub_.publish(customSensor);
      }
    }

private:

    DrRobotMotionSensorDriver* drrobotMotionDriver_;
    DrRobotMotionSensorDriver* drrobotPowerDriver_;
    struct DrRobotMotionConfig robotConfig1_;
    struct DrRobotMotionConfig robotConfig2_;

    std::string odom_frame_id_;
    struct MotorSensorData motorSensorData_;
    struct RangeSensorData rangeSensorData_;
    struct PowerSensorData powerSensorData_;
    struct StandardSensorData standardSensorData_;
    struct CustomSensorData customSensorData_;


    std::string robotType_;
    std::string robotID_;
    std::string robotIP_;
    std::string robotCommMethod_;
    std::string robotSerialPort_;
    bool enable_ir_;
    bool enable_sonar_;
    int  commPortNum_;
    int  encoderOneCircleCnt_;
    double wheelDis_;
    double wheelRadius_;
    int motorDir_;
    double minSpeed_;
    double maxSpeed_;

    int cntNum_;
    double ad2Dis(int adValue)
    {
      double temp = 0;
      double irad2Dis = 0;

      if (adValue <= 0)
        temp = -1;
      else
        temp = 21.6 /((double)adValue * 3 /4096 - 0.17);

      if ( (temp > 80) || (temp < 0))
      {
        irad2Dis = 0.81;
      }
      else if( (temp < 10) && (temp > 0))
      {
        irad2Dis = 0.09;
      }
      else
        irad2Dis = temp /100;
      return irad2Dis;
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "drrobot_I90_player");

    DrRobotPlayerNode drrobotPlayer;
    ros::NodeHandle n;
    // Start up the robot
    if (drrobotPlayer.start() != 0)
    {
        exit(-1);
    }
    /////////////////////////////////////////////////////////////////

    ros::Rate loop_rate(10);      //10Hz

    while (n.ok())
    {
      drrobotPlayer.doUpdate();
      ros::spinOnce();
     loop_rate.sleep();
    }
    /////////////////////////////////////////////////////////////////

    // Stop the robot
    drrobotPlayer.stop();

    return(0);
}

