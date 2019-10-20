/*
 * DrRobotCommConst.hpp
 *
 *  Created on: Mar 24, 2011
 *      Author: dri
 */

#ifndef DRROBOTCOMMCONST_HPP_
#define DRROBOTCOMMCONST_HPP_

//here is some parameters in communication protocol, please consult the Dr Robot API document
  const uint32_t MAX_AD_READING = 4095;

  const unsigned char COM_STX0 = 0x5E;
  const unsigned char COM_STX1 = 0x02;
  const unsigned char COM_TYPE_PC = 0x00;
  const unsigned char COM_TYPE_PC_PLUS = 0xb8;     // for Sentinel3 Hawk, and H20

  const unsigned char COM_TYPE_MOT = 0x01;
  const unsigned char COM_TYPE_MOT_PLUS = 0x8b;         // for Sentinel3 Hawk, and H20
  const unsigned char COM_ETX0 = 0x5E;
  const unsigned char COM_ETX1 = 0x0D;

  const unsigned char TIMEFLAG = 28;
  const unsigned char MOTORPOSITIONCTRL = 3;
  const unsigned char MOTORPOSITIONCTRLALL = 4;

  const unsigned char MOTORPWMCTRL = 5;
  const unsigned char MOTORPWMCTRLALL = 6;

  const unsigned char MOTORPARAMETERSETTING = 7;
  const unsigned char POSITIONPID = 7;
  const unsigned char VELOCITYPID = 8;
  const unsigned char MOTORVELOCITYCTRL = 26;
  const unsigned char MOTORVELOCITYCTRLALL = 27;

  const unsigned char SERVOCTRL = 28;
  const unsigned char SERVOCTRLALL = 29;
  const unsigned char MOTORENABLE = 0x1e;

  const unsigned char MOTORFRICCOMP = 31;
  const unsigned char CUSTOMIO = 22;
  const unsigned char POWERCTRL = 22;

  const unsigned char KP_ID = 1;
  const unsigned char KD_ID = 2;
  const unsigned char KI_ID = 3;

  //actually lost communication detect time is 200*10ms = 2s;
  const int  COMM_LOST_TH  = 200;
  const int  COMM_SLEEP_TM =  10000;   //10ms

  const int SDK_COM_DATA_MIN_LENGTH = 9;


  #define DEFAULT_PORT 10001
  #define INDEX_STX0              0
  #define INDEX_STX1              1
  #define INDEX_DES               2
  #define INDEX_SN                3
  #define INDEX_TYPE              4
  #define INDEX_LENGTH            5
  #define INDEX_DATA              6

  #define COMTYPE_SYSTEM          0xFF
  #define COMTYPE_SENSOR          0x7F
  #define COMTYPE_MOTOR           40
  #define COMTYPE_MOTOR_SENSOR     0x7B
  #define COMTYPE_CUSTOM_SENSOR    0x7C
  #define COMTYPE_STANDARD_SENSOR   0x7D

  #define DATA_ACK                        0x01
  #define DATA_PING                       0x00
  #define DATA_URGENT_DATA        0x02
  #define DATA_SKIPPACKET         0x03

#endif /* DRROBOTCOMMCONST_HPP_ */
