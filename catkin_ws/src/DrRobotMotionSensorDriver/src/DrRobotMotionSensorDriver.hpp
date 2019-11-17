/*! @mainpage
 *  DrRobotMotionSensorDriver
 *  Copyright (C) 2010-2011  Dr Robot Inc
 *
 *  This library is software driver for motion/power control system
 *  on I90 series robot, Sentinel3 robot, Hawk/H20, X80SV
 *  series robot and Jaguar outdoor robot from Dr Robot Inc.
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * DrRobotMotionSensorDriver.hpp
 *
 *  Created on: Mar 11, 2011
 *      Author: dri
 */

#ifndef DRROBOTMOTIONSENSORDRIVER_H_
#define DRROBOTMOTIONSENSORDRIVER_H_
#include <stdexcept>
#include <termios.h>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <poll.h>


//! A namespace containing the DrRobot Motion/Sensor driver
namespace DrRobot_MotionSensorDriver
{
  typedef unsigned char BYTE;

 /*! This definition limits all the string variables length < 255
  * such as robotID, robotIP and serial port name
 */
  const int CHAR_BUF_LEN  =  255;

  /*! This definition limits the communication receiving buffer length
   */
  const int MAXBUFLEN = 512;

  /*! This definition is no control command for motion control system.
   *
   */
  const int  NOCONTROL = -32768;

  const int ULTRASONICSENSOR_NUM = 6;	//!< on motion control system up to 6 channel ultrasonic sensor,
					//!< I90, Sentinel3 [0,1,2], Hawk [0~5], H20[0~4], X80SV, X80SVP [0~5]
  const int CUSTOMSENSOR_NUM = 8;	//!< on motion control system up to 8 AD channel
					//!< not available on standard I90,Sentinel3, Hawk/H20 robot
  const int MOTORSENSOR_NUM = 6;	//!< on motion control system up to 6 motor channel
					//!< channel 0,1 for I90,Sentinel3,Hawk/H20 left/right motor
					//!< channel 0,1 for Jaguar Arm motor, channel 3,4 for Jaguar forward/Turn power,
  const int IRRANGESENSOR_NUM = 10;	//!< up to 10 IR range sensor on standard I90/Sentinel3/Hawk/H20 Robot
					//!< mounting position/orientation information please referee robot manual

  /*! \enum CommMethod
   * Normally for standard robot, driver will use Network to communicate with robot
   * If you use RS232 module connect robot with your PC, you need set as Serial
   */
  enum CommMethod {Network, Serial};
  /*! \enum CommState
   *  Driver communication status
   */
  enum CommState { Disconnected, Connected};
  /*! \enum CommMethod
   *  specify the control system on the robot
   */
  enum BoardType {I90_Motion,I90_Power,Sentinel3_Motion, Sentinel3_Power,Hawk_H20_Motion,Hawk_H20_Power,Jaguar,X80SV};
  /*! \enum CtrlMethod
   *  specify control method of the motor control command
   */
  enum CtrlMethod {PWM,Velocity,Position};

  /*! \struct  DrRobotMotionConfig
   *  to configure the driver
   */
  struct DrRobotMotionConfig
  {
    char robotID[CHAR_BUF_LEN];        //!< robotID, you could specify your own name for your robot
    char robotIP[CHAR_BUF_LEN];        //!< robot main WiFi module IP address, you could get it by manual
    int portNum;                       //!< robot main WiFi module port number, default is power system on 10001 port, motion system on 10002
    CommMethod commMethod;             //!< communication method enum CommMethod
    char serialPortName[CHAR_BUF_LEN]; //!< serial port name if you use serial communication
    BoardType boardType;               //!< specify the control system on the robot, enum BoardType
  };

  /*! \struct  MotorSensorData
   *  for motor sensor data
   */
  struct MotorSensorData
  {
    int motorSensorEncoderPos[MOTORSENSOR_NUM];         //!< encoder count reading
    int motorSensorEncoderVel[MOTORSENSOR_NUM];         //!< encoder velocity reading
    int motorSensorEncoderDir[MOTORSENSOR_NUM];         //!< encoder move direction reading
    int motorSensorCurrent[MOTORSENSOR_NUM];            //!< motor current AD value reading, only channel 0,1 available on I90/Sentinel3/H20/Hawk robot
    int motorSensorPot[MOTORSENSOR_NUM];                //!< motor potentiometer sensor reading, not available now
    int motorSensorPWM[MOTORSENSOR_NUM];                //!< motion control system output PWM value, only channel[0,1,3,4] available on Jaguar robot
  };

  /*! \struct  PowerSensorData
   *  only available on I90/Sentinel3/Hawk/H20 power control system
   */
  struct PowerSensorData
  {
    int battery1Vol;            //!< battery 1 voltage AD value reading
    int battery1Thermo;         //!< battery 1 temperature AD value reading
    int battery2Vol;            //!< battery 2 voltage AD value reading
    int battery2Thermo;         //!< battery 2 temperature AD value reading
    int dcINVol;                //!< DCIN voltage AD value reading
    int refVol;                 //!< AD reference power AD value reading
    BYTE powerStatus;           //!< power control system status, please referee the Dr Robot protocol reference manual
                                //!< bit 0 --- reserved
                                //!< bit 1 --- reserved
                                //!< bit 2 --- charge status, '1' in charging, '0' no charging
                                //!< bit 3 --- power fail
                                //!< bit 4 --- DCDIV comparator output
                                //!< bit 5 --- lower power
                                //!< bit 6 --- Fault(no reset, short circuit, shutdown)
                                //!< bit 7 --- reserved
    BYTE powerPath;             //!< power path control, please referee the Dr Robot protocol reference mannaul
                                //!< bit 0 --- reserved
                                //!< bit 1 --- reserved
                                //!< bit 2 --- reserved
                                //!< bit 3 --- reserved
                                //!< bit 4 --- reserved
                                //!< bit 5 --- '1' powered by DCIN, '0' no
                                //!< bit 6 --- '1' powered by Battery2, '0' no
                                //!< bit 7 --- '1' powered by Battery1, '0' no
    BYTE powerChargePath;       //!< charge path control, please referee the Dr Robot protocol reference mannual
                                //!< bit 0 --- reserved
                                //!< bit 1 --- reserved
                                //!< bit 2 --- reserved
                                //!< bit 3 --- reserved
                                //!< bit 4 --- reserved
                                //!< bit 5 --- reserved
                                //!< bit 6 --- '1' Battery2 in charging, '0' no
                                //!< bit 7 --- '1' Battery1 in charging, '0' no
  };

  /*! \struct  RangeSensorData
   *  the driver will collector all the IR range sensors(AD value) and ultrasonic sensors reading
   *  into this structure, user could use on reading function to get all the range sensor information
   */
  struct RangeSensorData
  {
    int irRangeSensor[IRRANGESENSOR_NUM];       //!< IR range sensor AD value reading
    int usRangeSensor[ULTRASONICSENSOR_NUM];    //!< ultrasonic range sensor reading, unit is cm, 0~ 255cm
  };
  /*! \struct  CustomSensorData
   *  8 expanded AD channel not available on standard robot
   *  8bit expanded IO channel available
   */
  struct CustomSensorData
  {
    int customADData[CUSTOMSENSOR_NUM];         //!< expanded AD channel not available
    int customIO;                               //!< expanded IO channel, the lower 8bit to 8 input channel
  };
  /*! \struct  StandardSensorData
   *  Here are the sensor reading which could be connected with motion control system.
   *
   */
  struct StandardSensorData
  {
    int humanSensorData[4];             //!< 2 human sensor, each with alarm/motion 2 channel output, on standard robot,
                                        //!< left human sensor channel[0,1], right human sensor channel[2,3]
    int tiltingSensorData[2];           //!< only available on X80SVP robot, channel[0] - X axis, channel[1] -- Y axis
    int overHeatSensorData[2];          //!< temperature sensor on motion control board
    int thermoSensorData;               //!< temperature sensor only available on X80SVP robot
    int boardPowerVol;                  //!< motion control board main power voltage AD reading
    int motorPowerVol;                  //!< motor power voltage AD reading
    int servoPowerVol;                  //!< servo power voltage AD reading, only available on X80 series robot
    int refVol;                         //!< motion control board AD reference power voltage reading, should be around 2048
    int potVol;                         //!< potentiometer power voltage reading, should be around 2048, not used now
  };

/*! \class DrRobotMotionSensorDriver
 *      This is the main class declare
 *      When using this driver library, user need call construct function DrRobotMotionSensorDriver() first, it will initialize
 *      all the internal variables to default value, then call setDrRobotMotionDriverConfig() function to initialize all the
 *      setting, such as robot ID, IP address and port number.user could call openNetwork() or openSerial() function to start
 *      the driver. After that user could use reading functions to poll the sensor reading, please notice the sensor update rate
 *       is around 10Hz, it is determined by firmware on the robot, so faster than this rate is not necessary and should be avoid.
 *
 */
  class DrRobotMotionSensorDriver
  {
  public:
    /*! @brief
     * Constructor function
     *
     */
    DrRobotMotionSensorDriver();

    /*! @brief
     *  Destructor function
     *
     */
    ~DrRobotMotionSensorDriver();

    /*! @brief
     *  This function is used for detecting the communication status,
     * @param[in]   none
     * @return false -- communication is lost
     *         true  -- communication is OK
     */
    bool portOpen();

    /*! @brief
     *  This function is used for closing the communication
     * @param[in]   none
     * @return 0 -- communication is closed
     *         others  -- something wrong there
     */
    void close();

    /*! @brief
     *  If the driver is configured as using serial communication, this function could open serial port and starting communication
     * @param[in]   serialPort serial port, on linux system, should as /dev/ttyS0
     * @param[in]   BAUD serial port baud rate, should be 115200 on standard robot
     * @return 0  port opened and starting communication
     *         others  something wrong there
     */
    int openSerial(const char* serialPort, const long BAUD);

    /*! @brief
     *  If the driver is configured as using network communication, this function could open UDP port to connect with robot
     *  and start communication
     * @param[in]   robotIP, should be as dot format, such as "192.168.0.201"
     * @param[in]   portNum, port number, 10001 or 10002
     * @return 0  port opened and starting communication
     *         others  something wrong there
     */
    int openNetwork(const char*  robotIP, const int portNum );

    /*! @brief
     *  This function will use struct DrRobotMotionConfig to configure the driver
     * @param[in]   driverConfig struct DrRobotMotionConfig
     * @return null
     */
    void setDrRobotMotionDriverConfig(DrRobotMotionConfig* driverConfig);

    /*! @brief
    *  This function will return the configuration of the driver
    * @param[in]   driverConfig struct DrRobotMotionConfig, will contain the driver configuration
    * @return null
    */
    void getDrRobotMotionDriverConfig(DrRobotMotionConfig* driverConfig);

    /*! @brief
     *  This function is used for reading motor sensor back from motion controller
     * @param[in]   motorSensorData this struct MotorSensorData will contain all the raw motor sensor data when return
     * @return 0 means success, other fail
     */


    int readMotorSensorData(MotorSensorData* motorSensorData );

    /*! @brief
     *  This function is used for reading power sensor back from power control system
     *  It is only for I90 series/Sentinel3/Hawk/H20 system
     * @param[in]   powerSensorData this struct PowerSensorData will contain all the raw power sensor data when returning
     * @return 0 means success, other fail
     */
    int readPowerSensorData(PowerSensorData* powerSensorData );

    /*! @brief
     *  This function is used for reading custom sensor back from motion controller
     *  On standard I90/Sentinel3/Hawk/H20 Robot, custom expanded AD channel not available
     * @param[in]   customSensorData this struct CustomSensorData will contain all the raw custom sensor data when returning
     * @return 0 means success, other fail
     */
    int readCustomSensorData(CustomSensorData* customSensorData);

    /*! @brief
     *  This function is used for reading all the range sensor back from motion controller
     *  It will include 6 ultrasonic sensors and 10 IR range sensor raw data.
     *  What is available on robot and mounting information please referee the robot manual
     * @param[in]   rangeSensorData this struct RangeSensorData will contain all the raw range sensor data when returning
     * @return 0 means success, other fail
     */
    int readRangeSensorData(RangeSensorData* rangeSensorData);

    /*! @brief
     *  This function is used for reading standard sensor back from motion controller
     * @param[in]   standardSensorData this struct StandardSensorData will contain all the raw standard sensor data when returning
     * @return 0 means success, other fail
     */
    int readStandardSensorData(StandardSensorData* standardSensorData);


    /*! @brief
     *  This function is used for sending 6 channel motors time control command to motion controller
     *  The first input parameter will tell controller what control method will be used for these command.
     *  PWM command value is in range (0 ~ 32767), 16384 will stop robot.
     *  Velocity/Position command value is for encoder value.
     *  If the value is -32768, it means this channel will not be controlled by this command.
     * @param[in]   ctrlMethod enum CtrlMethod{PWM,Velocity,Position}
     * @param[in]   cmd1 motor channel 0 command value
     * @param[in]   cmd2 motor channel 1 command value
     * @param[in]   cmd3 motor channel 2 command value
     * @param[in]   cmd4 motor channel 3 command value
     * @param[in]   cmd5 motor channel 4 command value
     * @param[in]   cmd6 motor channel 5 command value
     * @param[in]   time execute time, unit is ms, it must > 0, otherwise in function it will be forced as "1"
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int sendMotorCtrlAllCmd(CtrlMethod ctrlMethod, const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6, const int time);

    /*! @brief
     *  This function is used for sending 6 channel motors control command to motion controller
     *  The first input parameter will tell controller what control method will use for these command.
     *  PWM command value is in range (0 ~ 32767), 16384 will stop robot.
     *  Velocity/Position command value is for encoder value.
     *  If the value is -32768, it means this channel will not be controlled by this command.
     * @param[in]   ctrlMethod enum CtrlMethod{PWM,Velocity,Position}
     * @param[in]   cmd1 motor channel 0 command value
     * @param[in]   cmd2 motor channel 1 command value
     * @param[in]   cmd3 motor channel 2 command value
     * @param[in]   cmd4 motor channel 3 command value
     * @param[in]   cmd5 motor channel 4 command value
     * @param[in]   cmd6 motor channel 5 command value
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int sendMotorCtrlAllCmd(CtrlMethod ctrlMethod, const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6);

    /*! @brief
     *  This function is used for sending one channel motor time control command to motion controller
     *  The first input parameter will tell controller what control method will be used for the command.
     *  PWM command value is in range (0 ~ 32767), 16384 will stop robot.
     *  Velocity/Position command value is for encoder value.
     *  If the value is -32768, it means this channel will not be controlled by this command.
     * @param[in]   ctrlMethod enum CtrlMethod{PWM,Velocity,Position}
     * @param[in]   channel specify the motor channel, 0 ~ 5
     * @param[in]   cmd motor command value
     * @param[in]   time execute time, unit is ms, it must > 0, otherwise in function it will be forced as "1"
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int sendMotorCtrlCmd(CtrlMethod ctrlMethod, const int channel, const int cmd, const int time);

    /*! @brief
     *  This function is used for sending one channel motor control command to motion controller
     *  The first input parameter will tell controller what control method will be used for the command.
     *  PWM command value is in range (0 ~ 32767), 16384 will stop robot.
     *  Velocity/Position command value is for encoder value.
     *  If the value is -32768, it means this channel will not be controlled by this command.
     * @param[in]   ctrlMethod enum CtrlMethod{PWM,Velocity,Position}
     * @param[in]   channel specify the motor channel, 0 ~ 5
     * @param[in]   cmd motor command value
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int sendMotorCtrlCmd(CtrlMethod ctrlMethod, const int channel, const int cmd);

    /*! @brief
     *  This function is used for sending 6 channel servo time control command to motion controller
     *  command value should be in the servo command range to prevent the servo from stuck
     *  If the value is -32768, it means this channel will not be controlled by this command.
     * @param[in]   cmd1 servo channel 0 command value
     * @param[in]   cmd2 servo channel 1 command value
     * @param[in]   cmd3 servo channel 2 command value
     * @param[in]   cmd4 servo channel 3 command value
     * @param[in]   cmd5 servo channel 4 command value
     * @param[in]   cmd6 servo channel 5 command value
     * @param[in]   time execute time, unit is ms, it must > 0, otherwise in function it will be forced as "1"
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int sendServoCtrlAllCmd(const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6, const int time);

    /*! @brief
     *  This function is used for sending 6 channel servo control command to motion controller
     *  command value should be in the servo command range to prevent the servo from stuck
     *  If the value is -32768, it means this channel will not be controlled by this command.
     * @param[in]   cmd1 servo channel 0 command value
     * @param[in]   cmd2 servo channel 1 command value
     * @param[in]   cmd3 servo channel 2 command value
     * @param[in]   cmd4 servo channel 3 command value
     * @param[in]   cmd5 servo channel 4 command value
     * @param[in]   cmd6 servo channel 5 command value
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int sendServoCtrlAllCmd(const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6);

    /*! @brief
     *  This function is used for sending one channel servo time control command to motion controller
     *  command value should be in the servo command range to prevent the servo from stuck
     *  If the value is -32768, it means this channel will not be controlled by this command.
     * @param[in]   channel servo channel, 0 ~ 5
     * @param[in]   cmd servo command value
     * @param[in]   time execute time, unit is ms, it must > 0, otherwise in function it will be forced as "1"
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int sendServoCtrlCmd(const int channel, const int cmd, const int time);

    /*! @brief
     *  This function is used for sending one channel servo control command to motion controller
     *  command value should be in the servo command range to prevent the servo from stuck
     *  If the value is -32768, it means this channel will not be controlled by this command.
     * @param[in]   channel servo channel 0 ~ 5
     * @param[in]   cmd servo command value
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int sendServoCtrlCmd(const int channel, const int cmd);

    /*! @brief
     *  This function is used for disable one channel motor command to motion controller
     * @param[in]   channel motor channel
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int disableMotorCmd(const int channel);

    /*! @brief
     *  This function is used for disable one channel servo command to motion controller
     * @param[in]   channel servo channel, 0 ~ 5
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int disableServoCmd(const int channel);

    /*! @brief
     *  This function is used for sending one channel motor setting position PID control parameter command to motion controller
     * @param[in]   channel motor channel
     * @param[in]   kp PID control Kp value
     * @param[in]   kd PID control Kd value
     * @param[in]   ki PID control Ki value, this value will be divided by 10 in the firmware
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int setMotorPositionCtrlPID(const int channel, const int kp, const int kd, const int ki);

    /*! @brief
     *  This function is used for sending one channel motor setting velocity PID control parameter command to motion controller
     * @param[in]   channel motor channel
     * @param[in]   kp PID control Kp value
     * @param[in]   kd PID control Kd value
     * @param[in]   ki PID control Ki value, this value will be divided by 10 in the firmware
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int setMotorVelocityCtrlPID(const int channel, const int kp, const int kd, const int ki);

    /*! @brief
     *  This function is used for sending friction compensation value for all 6 motors to motion controller.
     *  Normaly you don't need to change these value in the firmware.
     * @param[in]   cmd1 motor channel 0 friction compensation value
     * @param[in]   cmd2 motor channel 1 friction compensation value
     * @param[in]   cmd3 motor channel 2 friction compensation value
     * @param[in]   cmd4 motor channel 3 friction compensation value
     * @param[in]   cmd5 motor channel 4 friction compensation value
     * @param[in]   cmd6 motor channel 5 friction compensation value
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int setMotorFricCompensation(const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6);

    /*! @brief
     *  This function is used for sending setting expanded IO command to motion controller
     * @param[in]   cmd setting IO command, one bit for one channel
     * @return If return value is negative, it means there is something wrong with sending command to robot.
     *        If successfully sending command, it will return the byte numbers of sending message.
     */
    int setCustomIO(const int cmd);

    /*! @brief
     *  This function is used for sending power control command to power control system.
     *  This command only available for I90/Sentinel3/Hawk/H20 robot.
     *  You could referee the robot manual to get what system on each power channel
     * @param[in]   cmd setting power control system
     *              bit 0 --- '1' turn on the channel 1, '0' turn off the channel 1
     *              bit 1 --- '1' turn on the channel 2, '0' turn off the channel 2
     *              bit 2 --- '1' turn on the channel 3, '0' turn off the channel 3
     *              bit 3 --- choose DCIN as power source
     *              bit 4 --- choose Battery 2 as power source
     *              bit 5 --- choose Battery 1 as power source
     *              bit 6 --- '1' start charging Battery2, '0' stop charging Battery2
     *              bit 7 --  '1' start charging Battery1, '0' stop charging Battery1
     * @return  If return value is negative, it means there is something wrong with sending command to robot.
     *          If successfully sending command, it will return the byte numbers of sending message.
     */
    int sendPowerCtrlCmd(const int cmd);

  private:
    BYTE _recBuf[MAXBUFLEN];
    BYTE _dataBuf[MAXBUFLEN];
    int _nMsgLen;
    int _sockfd;
    int _serialfd;
    struct sockaddr_in _addr;
    socklen_t _addr_len;
    char _sAddr[INET6_ADDRSTRLEN];
    int _numbytes;
    struct timeval _tv;
    fd_set _readfds;
    int _comCnt;
    pthread_mutex_t _mutex_Data_Buf;
    DrRobotMotionConfig *_robotConfig;
    boost::shared_ptr<boost::thread> _pCommThread;
    //private functions here
    void debug_ouput(const char* errorstr);
    int vali_ip(const char* ip_str);
    int sendAck();
    unsigned char CalculateCRC( const unsigned char *lpBuffer, const int nSize);
    void commWorkingThread();
    void DealWithPacket(const unsigned char *lpComData, const int nLen);
    void handleComData(const unsigned char *data, const int nLen);
    bool _stopComm;
    CommState _eCommState;
    void debugCommMessage(std::string msg);
    int sendCommand(const unsigned char* msg, const int nLen);
    //sensor data here
    struct RangeSensorData  _rangeSensorData;
    struct CustomSensorData _customSensorData;
    struct MotorSensorData _motorSensorData;
    struct PowerSensorData _powerSensorData;
    struct StandardSensorData _standardSensorData;
    unsigned char _desID;
    unsigned char _pcID;
  };

}

#endif /* DRROBOTMOTIONSENSORDRIVER_H_ */
