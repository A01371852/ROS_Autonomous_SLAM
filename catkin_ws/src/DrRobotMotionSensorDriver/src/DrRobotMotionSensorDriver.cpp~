/*
 * DrRobotMotionSensorDriver.cpp
 *
 *  Created on: Mar 14, 2011
 *      Author: dri
 */

#include "DrRobotMotionSensorDriver.hpp"
#include "DrRobotCommConst.hpp"


//#define DEBUG_ERROR           //set printf out error message
#undef DEBUG_ERROR


using namespace std;
using namespace DrRobot_MotionSensorDriver;

/*! this function is construct function for DrRobotMotionSensorDriver Class
   It will initialize all the internal variables
*/

DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::DrRobotMotionSensorDriver()
{

  _robotConfig = new DrRobotMotionConfig();
  _robotConfig->commMethod = Network;
  sprintf(_robotConfig->serialPortName, "tty0");

  _robotConfig->portNum = DEFAULT_PORT;
  sprintf(_robotConfig->robotID, "DrRobot");
  sprintf(_robotConfig->robotIP, "192.168.0.201");

  _robotConfig->boardType = I90_Power;

  _nMsgLen = 0;

  bzero(&_addr, sizeof(_addr));
  _addr.sin_family = AF_INET;
  _addr.sin_port = htons(_robotConfig->portNum);
  _addr_len = sizeof _addr;
  _numbytes = 0;

  _sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  _serialfd = -1;

  _tv.tv_sec = 0;
  _tv.tv_usec = 200;             //200us ?

  _stopComm = true;
  _comCnt = 0;
  _mutex_Data_Buf = PTHREAD_MUTEX_INITIALIZER;
  _eCommState = Disconnected;
  _desID = COM_TYPE_MOT;
  _pcID = COM_TYPE_PC;

}

DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::~DrRobotMotionSensorDriver()
{
  if (portOpen())
    close();
}

bool DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::portOpen()
{
  if ( (_eCommState == Connected) && (!_stopComm))
    return true;
  else
    return false;
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::close()
{

  _stopComm = true;
  _pCommThread->join();
  _eCommState = Disconnected;

  //for UDP , do we need close socket?

  if (_robotConfig->commMethod == Network)
  {
    if (_sockfd > 0)
    {
      ::close(_sockfd);
      _sockfd = -1;
    }
  }
  else if(_robotConfig->commMethod == Serial)
  {
    if (_serialfd > 0)
      {
        :: close(_serialfd);
        _serialfd = -1;
      }
  }
}


int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::openSerial(const char* serialPort, const long BAUD)
{

  if (portOpen())
    close();

  _robotConfig->commMethod = Serial;

  sprintf(_robotConfig->serialPortName, "%s",serialPort);

  _serialfd = ::open(_robotConfig->serialPortName, O_RDWR | O_NONBLOCK | O_NOCTTY);
  //_serialfd = ::open("/dev/ttyS0", O_RDWR | O_NONBLOCK | O_NOCTTY);
  if (_serialfd > 0)
  {
    struct termios newtio;

    tcgetattr(_serialfd, &newtio);
    memset(&newtio.c_cc, 0,sizeof(newtio.c_cc));
    newtio.c_cflag = BAUD|CS8|CLOCAL|CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN] = 0;              //VMIN = 0, VTIME = 0, read will return immediately
    newtio.c_cc[VTIME] = 0;
    tcflush(_serialfd,TCIFLUSH);
    tcsetattr(_serialfd,TCSANOW, &newtio);

    printf("listener: waiting for robot server, starting receiving...\n");
   _eCommState = Connected;
   _stopComm = false;
   _pCommThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DrRobotMotionSensorDriver::commWorkingThread, this)));
     return 0;
 }

  else
  {
    const char *extra_msg = "";
    switch(errno)
    {
      case EACCES:
          extra_msg = "You probably don't have permission to open the port for reading and writing.\n";
          debug_ouput(extra_msg);
          break;
      case ENOENT:
        extra_msg = "The request port does not exit. Was the port name misspelled?\n";
        debug_ouput(extra_msg);
        break;
    }
   _stopComm = true;
   _eCommState = Disconnected;
    return errno;
  }


}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::debug_ouput(const char* errorstr)
{
#ifdef DEBUG_ERROR
    printf("DrRobot Motion Sensor: %s", errorstr);
#endif
}
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::vali_ip(const char*  ip_str)
{
  unsigned int n1,n2,n3,n4;
  if ( sscanf(ip_str, "%u.%u.%u.%u", &n1,&n2,&n3,&n4) != 4 ) return 1;
  if ((n1 != 0) && (n1 <= 255) && (n2 <= 255) && (n3 <= 255) && (n4 <= 255) )
  {
    char buf[64];
    sprintf(buf,"%u.%u.%u.%u", n1,n2,n3,n4);
    if (strcmp(buf,ip_str)) return 1;
    return 0;
  }

  return 1;
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::openNetwork(const char* robotIP, const int portNum )
{
  char temp[512] ;
  //check the parameter first
  if (portNum <= 0)
  {

    debug_ouput(temp);
    return -1;
  }

  if (vali_ip(robotIP) == 1)
  {
    sprintf(temp, "DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", robotIP);
    debug_ouput(temp);
    return -2;
  }
  _robotConfig->commMethod = Network;
  _robotConfig->portNum = portNum;

  sprintf(_robotConfig->robotIP, "%s",robotIP);
  bzero(&_addr, sizeof(_addr));
  _addr.sin_family = AF_INET;
  _addr.sin_port = htons(_robotConfig->portNum);

  if ( inet_aton(_robotConfig->robotIP, &_addr.sin_addr) == 0)
  {
    sprintf(temp, "DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", _robotConfig->robotIP);
    debug_ouput(temp);
    return -3;
  }

  _stopComm = false;
 _numbytes = sendAck();
 if (_numbytes < 0)
 {
   _stopComm = true;
   perror("sendto");
   return -4;
 }

 printf("listener: waiting for robot server, starting receiving...\n");
 _eCommState = Connected;


 _pCommThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DrRobotMotionSensorDriver::commWorkingThread, this)));
 return 0;
}

//communication thread here
void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::commWorkingThread(){
  while(!_stopComm)
  {
     if (_robotConfig->commMethod == Network)
     {

      FD_ZERO(&_readfds);
      FD_SET(_sockfd, &_readfds);
      select(_sockfd + 1, &_readfds, NULL, NULL, &_tv);
      if (FD_ISSET(_sockfd,&_readfds))
      {
        if ((_numbytes = recvfrom(_sockfd, _recBuf, MAXBUFLEN-1 , 0,(struct sockaddr *)&_addr, &_addr_len)) == -1)
        {
                perror("recvfrom");
                return;
        }
  #ifdef DEBUG_ERROR
        printf("listener: packet is %d bytes long\n", _numbytes);
  #endif
        _comCnt = 0;
        handleComData(_recBuf,_numbytes);
      }
      else
      {
        _comCnt++;

        usleep(10000);              //10ms
        if (_comCnt > COMM_LOST_TH)
        {
          printf("Communication is lost, need close all. IP address %s, Port: %d \n", _robotConfig->robotIP, _robotConfig->portNum);
          _stopComm = true;
          return;
        }
      }
     }
    else if(_robotConfig->commMethod == Serial)
    {

      _numbytes = read(_serialfd,_recBuf, sizeof(_recBuf));
      if (_numbytes <= 0 )   //( (_numbytes == -1) && (errno != EAGAIN) && (errno != EWOULDBLOCK) )
      {
        //read erro,
        _comCnt ++;
        //printf ("Serial time out\n");
        usleep(10000);
        if (_comCnt > COMM_LOST_TH)
        {
          printf("Communication is lost, need close all. Serial Port is %s \n", _robotConfig->serialPortName);
          _stopComm = true;
          _eCommState = Disconnected;
          ::close(_serialfd);
          _serialfd = -1;
        }

      }
      else
      {
    #ifdef DEBUG_ERROR
       printf("listener: packet is %d bytes long\n", _numbytes);
    #endif
       _comCnt = 0;
       handleComData(_recBuf,_numbytes);
      }



       /*
      struct pollfd ufd[1];
      int retval;
      int timeout = 0;
      ufd[0].fd = _serialfd;
      ufd[0].events = POLLIN;
      //below will block to wait event

      //if (timeout == 0)
      //  timeout = -1;



      retval = poll(ufd, 1, timeout);
      if (retval < 0)
      {
        //poll fialed -- error
        printf("DrRobot Serial Communication error, eroor no is %d: %s", errno, strerror(errno));
       _stopComm = true;
       _eCommState = Disconnected;
       ::close(_sockfd);
       _sockfd = -1;
       return;
      }
      else if (retval  == 0)
      {
        //timeout,
        _comCnt ++;
        printf ("Serial time out\n");
        usleep(10000);
        if (_comCnt > COMM_LOST_TH)
        {
          printf("communication is lost, need close all\n");
          _stopComm = true;
          _eCommState = Disconnected;
          ::close(_serialfd);
          _serialfd = -1;
        }
      }
      else
      {
        _numbytes = read(_serialfd,_recBuf, sizeof(_recBuf));
        if ( (_numbytes == -1) && (errno != EAGAIN) && (errno != EWOULDBLOCK) )
        {
          //read erro,
          _comCnt ++;
        }
        else
        {
#ifdef DEBUG_ERROR
       printf("listener: packet is %d bytes long\n", _numbytes);
 #endif
       _comCnt = 0;
       handleComData(_recBuf,_numbytes);
        }
      }
      */

    }
  }
  return;
}


unsigned char DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::CalculateCRC(const unsigned char *lpBuffer, const int nSize)
{
  unsigned char shift_reg, sr_lsb, data_bit, v;
  int i,j;
  unsigned char fb_bit;

  shift_reg = 0; // initialize the shift register

  for(i=0;i<nSize;i++)
  {
    v = (unsigned char)(lpBuffer[i]&0x0000FFFF);
    for(j=0;j<8;j++) // for each bit
    {
            data_bit = v & 0x01; // isolate least sign bit
            sr_lsb = shift_reg & 0x01;
            fb_bit = (data_bit^sr_lsb) & 0x01; // calculate the feed back bit
            shift_reg = shift_reg >> 1;
            if (fb_bit == 1)
                    shift_reg = shift_reg ^ 0x8C;
            v = v >> 1;
    }
  }
  return shift_reg;

}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendAck()
{
  unsigned char msg[] = {COM_STX0,COM_STX1,COM_TYPE_MOT,0,0xff,1,1,0,COM_ETX0,COM_ETX1};
  msg[2] = _desID;
  msg[7] = CalculateCRC(&msg[2],msg[5] + 4);
  return sendCommand(msg,10);
}


void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::DealWithPacket(const unsigned char *lpComData, const int nLen)
{
  char debugtemp[512] ;
  int temp;
  if ( (lpComData[INDEX_STX0] != COM_STX0)||
                   (lpComData[INDEX_STX1] != COM_STX1)
             )
          {

                  debugCommMessage("invalid packet header ID, discard it!\n");

                  return;
          }
          else if ( lpComData[INDEX_DES] != _pcID )
          {
            debugCommMessage("invalid packet destination id PC, discard it!\n");
            return;
          }

          BYTE ucLength = (BYTE)lpComData[INDEX_LENGTH];

          if ( (ucLength + INDEX_DATA + 3) != nLen )
          {
                  sprintf(debugtemp, "invalid packet size, discard it! whole length: %d, the data length: %d\n", nLen, ucLength);
                  std::string temp(debugtemp);
                  debugCommMessage(temp);

                  return;
          }

          if ( (lpComData[nLen-1]!=COM_ETX1)||
                   (lpComData[nLen-2]!=COM_ETX0)
             ) // check ETX indicator
          {
            debugCommMessage("invalid packet ETX indicator, discard it!\n");
            return;
          }

          // verify CRC correction
          if ( CalculateCRC(lpComData+INDEX_DES, ucLength+4)
                                                  != (BYTE)lpComData[ucLength+INDEX_DATA] )
          {
            debugCommMessage("invalid CRC, discard it\n");
            return;
          }

          // all right, until this point, all format data have been checked okay!
          //
          switch ( (unsigned char)lpComData[INDEX_TYPE] )
          {
            case COMTYPE_SYSTEM:
              {
                switch ( lpComData[INDEX_DATA] )
                {
                  case DATA_ACK:
                    debugCommMessage("receive acknowledgment packet!\n");
                    sendAck();
                    debugCommMessage("send acknowledge!\n");

                    break;
                  case DATA_PING:
                    debugCommMessage("receive ping packet!\n");


                    break;
                  case DATA_URGENT_DATA:

                    debugCommMessage("receive urgent packet!\n");
                    sendAck();
                    break;
                  case DATA_SKIPPACKET:
                    debugCommMessage("receive skip packet!\n");
                    sendAck();
                    break;
                  default:
                    debugCommMessage("invalid packet data in system packet, discard it!\n");
                    return;
                }
              }
              break ;
            case COMTYPE_SENSOR:
              debugCommMessage("receive Sensor data packet!\n");
              break ;
            case COMTYPE_MOTOR_SENSOR:
              debugCommMessage("receive motor sensor data packet!\n");
	      sendAck();
              pthread_mutex_lock(&_mutex_Data_Buf);
              for (int i = 0; i < MOTORSENSOR_NUM; i ++)
              {
                _motorSensorData.motorSensorPot[i] = lpComData[INDEX_DATA + 2 * i] + lpComData[INDEX_DATA + 2 * i + 1] * 256;
              }
              if (_robotConfig->boardType == Jaguar)
              {
                for (int i = 0; i < MOTORSENSOR_NUM; i++)
                {
                  _motorSensorData.motorSensorPWM[i] = lpComData[INDEX_DATA + 2 * i] + lpComData[INDEX_DATA + 2 * i + 1] * 256;
                }
              }

              for (int i = 0; i < MOTORSENSOR_NUM; i ++)
              {
                _motorSensorData.motorSensorCurrent[i] = lpComData[INDEX_DATA + 12 + 2 * i] + lpComData[INDEX_DATA + 12 + 2 * i + 1] * 256;
              }
              _motorSensorData.motorSensorEncoderPos[0] = lpComData[INDEX_DATA + 24] + lpComData[INDEX_DATA + 25] * 256;
              _motorSensorData.motorSensorEncoderVel[0] = lpComData[INDEX_DATA + 26] + lpComData[INDEX_DATA + 27] * 256;
              _motorSensorData.motorSensorEncoderPos[1] = lpComData[INDEX_DATA + 28] + lpComData[INDEX_DATA + 29] * 256;
              _motorSensorData.motorSensorEncoderVel[1] = lpComData[INDEX_DATA + 30] + lpComData[INDEX_DATA + 31] * 256;
              if (lpComData[INDEX_DATA + 32] & 0x01)
              {
                _motorSensorData.motorSensorEncoderDir[0] = 1;
              }
              else
              {
                _motorSensorData.motorSensorEncoderDir[0] = -1;
              }
              if (lpComData[INDEX_DATA + 32] & 0x02)
              {
                _motorSensorData.motorSensorEncoderDir[1] = 1;
              }
              else
              {
                _motorSensorData.motorSensorEncoderDir[1] = -1;
              }
              pthread_mutex_unlock(&_mutex_Data_Buf);
              break;
            case COMTYPE_CUSTOM_SENSOR:
              pthread_mutex_lock(&_mutex_Data_Buf);
              for (int i = 0; i < CUSTOMSENSOR_NUM ; i ++)
              {
                _customSensorData.customADData [i]  = lpComData[INDEX_DATA + 2 * i] + lpComData[INDEX_DATA + 2 * i + 1] * 256;
              }

              _customSensorData.customIO = lpComData[INDEX_DATA + 16];

              if (_robotConfig->boardType != Jaguar)
              {
                for (int i = 0; i < 6 ; i ++)
                {
                  _rangeSensorData.irRangeSensor[1 + i] = _customSensorData.customADData[2 + i];
                }

              }

              if ((_robotConfig->boardType == I90_Power) || (_robotConfig->boardType == Sentinel3_Power) ||(_robotConfig->boardType == Hawk_H20_Power))
              {
                _powerSensorData.battery1Vol = lpComData[INDEX_DATA + 0] + lpComData[INDEX_DATA + 1] * 256;
                _powerSensorData.battery1Thermo = lpComData[INDEX_DATA + 2] + lpComData[INDEX_DATA + 3] * 256;
                _powerSensorData.battery2Vol = lpComData[INDEX_DATA + 4] + lpComData[INDEX_DATA + 5] * 256;
                _powerSensorData.battery2Thermo = lpComData[INDEX_DATA + 6] + lpComData[INDEX_DATA + 7] * 256;
                _powerSensorData.dcINVol = lpComData[INDEX_DATA + 8] + lpComData[INDEX_DATA + 9] * 256;
                _powerSensorData.refVol = lpComData[INDEX_DATA + 19] + lpComData[INDEX_DATA + 20] * 256;
                _powerSensorData.powerChargePath = lpComData[INDEX_DATA + 29];
                _powerSensorData.powerPath = lpComData[INDEX_DATA + 27];
                _powerSensorData.powerStatus = lpComData[INDEX_DATA + 16];
              }
              if (_robotConfig->boardType == Jaguar)
              {
                _motorSensorData.motorSensorEncoderPos[3] = lpComData[INDEX_DATA + 17] + lpComData[INDEX_DATA + 18] * 256;
                _motorSensorData.motorSensorEncoderVel[3] = lpComData[INDEX_DATA + 19] + lpComData[INDEX_DATA + 20] * 256;

		temp = lpComData[INDEX_DATA + 21] + lpComData[INDEX_DATA + 22] * 256;
                if (temp & 0x01)
                {
                  _motorSensorData.motorSensorEncoderDir[3] = 1;
                }
                else
                {
                  _motorSensorData.motorSensorEncoderDir[3] = 0;
                }
                _motorSensorData.motorSensorEncoderPos[4] = lpComData[INDEX_DATA + 23] + lpComData[INDEX_DATA + 24] * 256;
                _motorSensorData.motorSensorEncoderVel[4] = lpComData[INDEX_DATA + 25] + lpComData[INDEX_DATA + 26] * 256;

		temp = lpComData[INDEX_DATA + 27] + lpComData[INDEX_DATA + 28] * 256;
                if (temp & 0x01)
                {
                  _motorSensorData.motorSensorEncoderDir[4] = 1;
                }
                else
                {
                  _motorSensorData.motorSensorEncoderDir[4] = 0;
                }
              }

              pthread_mutex_unlock(&_mutex_Data_Buf);
              debugCommMessage("receive custom sensor data packet!\n");
              break;
            case COMTYPE_STANDARD_SENSOR:
              pthread_mutex_lock(&_mutex_Data_Buf);
              for (int i = 0; i < ULTRASONICSENSOR_NUM; i ++)
              {
                _rangeSensorData.usRangeSensor[i] = lpComData[INDEX_DATA + i];
              }
              for (int i = 0; i < 4 ; i ++)
              {
                _standardSensorData.humanSensorData[i] = lpComData[INDEX_DATA + 6 + 2*i] + lpComData[INDEX_DATA + 6 + 2 * i + 1] * 256;
              }
              _standardSensorData.tiltingSensorData[0] = lpComData[INDEX_DATA + 14] + lpComData[INDEX_DATA + 15] * 256;
              _standardSensorData.tiltingSensorData[1] = lpComData[INDEX_DATA + 16] + lpComData[INDEX_DATA + 17] * 256;
              _standardSensorData.overHeatSensorData[0] = lpComData[INDEX_DATA + 18] + lpComData[INDEX_DATA + 19] * 256;
              _standardSensorData.overHeatSensorData[1] = lpComData[INDEX_DATA + 20] + lpComData[INDEX_DATA + 21] * 256;
              _standardSensorData.thermoSensorData = lpComData[INDEX_DATA + 22] + lpComData[INDEX_DATA + 23] * 256;
              _rangeSensorData.irRangeSensor[0] = lpComData[INDEX_DATA + 24] + lpComData[INDEX_DATA + 25] * 256;
              _standardSensorData.boardPowerVol = lpComData[INDEX_DATA + 30] + lpComData[INDEX_DATA + 31] * 256;
              _standardSensorData.motorPowerVol = lpComData[INDEX_DATA + 32] + lpComData[INDEX_DATA + 33] * 256;
              _standardSensorData.servoPowerVol = lpComData[INDEX_DATA + 34] + lpComData[INDEX_DATA + 35] * 256;
              _standardSensorData.refVol = lpComData[INDEX_DATA + 36] + lpComData[INDEX_DATA + 37] * 256;
              _standardSensorData.potVol = lpComData[INDEX_DATA + 38] + lpComData[INDEX_DATA + 39] * 256;

              if (_robotConfig->boardType != Jaguar) 
              {
                _rangeSensorData.irRangeSensor[7] = _standardSensorData.thermoSensorData;
                _rangeSensorData.irRangeSensor[8] = _standardSensorData.tiltingSensorData[0];
                _rangeSensorData.irRangeSensor[9] = _standardSensorData.tiltingSensorData[1];
              }

              pthread_mutex_unlock(&_mutex_Data_Buf);
              debugCommMessage("receive standard sensor data packet!\n");
              break;
            default:

               sprintf(debugtemp, "invalid packet data type(%#2X), discard it!\n", (unsigned char)lpComData[INDEX_TYPE] );
              debugCommMessage(debugtemp);
              return;

        }


        return;
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::handleComData(const unsigned char *data, const int nLen)
{
    unsigned char msgHeader[] = {COM_STX0,COM_STX1};
    unsigned char msgTail[] = {COM_ETX0,COM_ETX1};
    int nStartIndex, nUnProcessedPacketLen, nPacketIndex, i;
    unsigned char* unProcessedPacket = NULL;


    nStartIndex = 0;
    nPacketIndex = 2;


    if (_nMsgLen + nLen < MAXBUFLEN){
      memcpy(_dataBuf + _nMsgLen, data, nLen);
      _nMsgLen += nLen;
    }
    else{
      //clear the whole internal buffer, just keep the latest packet
      memcpy(_dataBuf,data,nLen);
      _nMsgLen = nLen;
    }


    //suppose the first 2 bytes are MG_HEAD1,2
    //otherwise bytes will removed from the beginning until MSG_HEAD occurs
    for(i = 0; i < _nMsgLen -1; i++){
      if ( (_dataBuf[i] != msgHeader[0]) || ( (_dataBuf[i + 1] != msgHeader[1]) && (_nMsgLen - i >= 2 )) ){
        //do nothing
      }
      else if( (_nMsgLen - i == 1) || (_nMsgLen - i == 2)){
        return;  // just got a header!!!
      }
      else{
        //here got the header
        nPacketIndex = i + 2;
        nStartIndex = i;
        break;
      }
   }

    if ( i == _nMsgLen - 1){
      //no MSG_HEAD find so far, then check if the last byte is HEAD1, if yes, just keep this byte, otherwise clear the whole received buffer
      if (_dataBuf[_nMsgLen - 1] == msgHeader[0]){
        _dataBuf[0] = msgHeader[0];
        _nMsgLen = 1;
        return;
      }
      else{
        _nMsgLen = 0;
        return;
      }

    }
    while(nPacketIndex + 1 < _nMsgLen){
      // look for the MSG_TAIL
      if ( (_dataBuf[nPacketIndex] == msgTail[0]) && (nPacketIndex + 1 < _nMsgLen) && (_dataBuf[nPacketIndex + 1] == msgTail[1])){
        // find a data package
        unProcessedPacket = _dataBuf + nStartIndex;
        nUnProcessedPacketLen = nPacketIndex + 2 - nStartIndex;
        nPacketIndex += 2;
        DealWithPacket(unProcessedPacket,nUnProcessedPacketLen);
        if (nPacketIndex == _nMsgLen){
          // the while data msg is processed, empty the buffer
          _nMsgLen = 0;
          return;
        }
        else{
          // in this case, packetIndex should be less than nMsgLen
          //look for start of packet indicator MSGHead
          while(nPacketIndex < _nMsgLen){
            if ( (_dataBuf[nPacketIndex] == msgHeader[0]) && (_dataBuf[nPacketIndex + 1] == msgHeader[1]) && (nPacketIndex + 1 < _nMsgLen)){
              nStartIndex = nPacketIndex;
              break;
            }
            else{
              nPacketIndex += 2;
            }
          }
          if (nPacketIndex < _nMsgLen){
            // find another header
          }
          else if(_dataBuf[_nMsgLen - 1] == msgHeader[0]){
            _dataBuf[0] = msgHeader[0];
            _nMsgLen = 1;
            return;
          }
          else{
            _nMsgLen = 0;
            return;
          }
        }
      }
      else{
        nPacketIndex++;
      }

    }
    if (nPacketIndex >= _nMsgLen -1){
      // did not find a tail
      if (nStartIndex != 0){
        memcpy(_dataBuf,_dataBuf + nStartIndex,_nMsgLen - nStartIndex);
      }
      _nMsgLen = _nMsgLen - nStartIndex;
    }

    return;
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::debugCommMessage(std::string msg)
{
#ifdef DEBUG_ERROR
  printf("DrRobot Motion Sensor Driver: %s",msg.c_str());
#endif

}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: getDrRobotMotionDriverConfig(DrRobotMotionConfig* driverConfig)
{

  strcpy(driverConfig->robotID,_robotConfig->robotID);
  driverConfig->boardType = _robotConfig->boardType;
  driverConfig->commMethod = _robotConfig->commMethod;
  driverConfig->portNum = _robotConfig->portNum;
  strcpy(driverConfig->robotIP, _robotConfig->robotIP);
  strcpy(driverConfig->serialPortName, _robotConfig->serialPortName);

  return;
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: setDrRobotMotionDriverConfig(DrRobotMotionConfig* driverConfig)
{

  strcpy(_robotConfig->robotID,driverConfig->robotID);
  _robotConfig->boardType = driverConfig->boardType;
  _robotConfig->commMethod = driverConfig->commMethod;
  _robotConfig->portNum = driverConfig->portNum;
  strcpy(_robotConfig->robotIP,driverConfig->robotIP);
  strcpy(_robotConfig->serialPortName,driverConfig->serialPortName);

  if ((_robotConfig->boardType == I90_Motion) || (_robotConfig->boardType == I90_Power) || (_robotConfig->boardType == Jaguar))
  {
    _pcID = COM_TYPE_PC;
    _desID = COM_TYPE_MOT;
  }
  else
  {
    _pcID = COM_TYPE_PC_PLUS;
    _desID = COM_TYPE_MOT_PLUS;
  }

  return;
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::readMotorSensorData(MotorSensorData* motorSensorData)
{
  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(motorSensorData,&_motorSensorData,sizeof(MotorSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
}
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: readPowerSensorData(PowerSensorData* powerSensorData)
{
  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(powerSensorData,&_powerSensorData,sizeof(PowerSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
}


int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: readCustomSensorData(CustomSensorData* customSensorData)
{

  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(customSensorData, &_customSensorData, sizeof(CustomSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
 }

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: readRangeSensorData(RangeSensorData* rangeSensorData)
{
  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(rangeSensorData, &_rangeSensorData, sizeof(RangeSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: readStandardSensorData(StandardSensorData* standardSensorData)
{
  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(standardSensorData,&_standardSensorData,sizeof(StandardSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
}


int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: sendMotorCtrlAllCmd(CtrlMethod ctrlMethod, const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6, const int time)
{
  unsigned char msg[255];
  short tempCmd = 0;
  short tempTime = time;
  if (tempTime <= 0) tempTime = 1;
  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;
  if (ctrlMethod == PWM)
  {
    msg[4] = MOTORPWMCTRLALL;
  }
  else if(ctrlMethod == Velocity)
  {
    msg[4] = MOTORVELOCITYCTRLALL;
  }
  else if(ctrlMethod == Position)
  {
    msg[4] = MOTORPOSITIONCTRLALL;
  }

    msg[5] = 14;
  for (int i = 0; i < 6; i ++)
  {
    if (i == 0)
    {
      tempCmd = cmd1;
    }
    if (i == 1)
    {
      tempCmd = cmd2;
    }
    if (i == 2)
    {
      tempCmd = cmd3;
    }
    if (i == 3)
    {
      tempCmd = cmd4;
    }
    if (i == 4)
    {
      tempCmd = cmd5;
    }
    if (i == 5)
    {
      tempCmd = cmd6;
    }
    memcpy(&msg[ 6 + i *2], &tempCmd, sizeof(short));
  }

  memcpy(&msg[18], &tempTime, sizeof(short));
  msg[20] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[21] = COM_ETX0; msg[22] = COM_ETX1;


  return sendCommand(msg, 23);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendMotorCtrlAllCmd(CtrlMethod ctrlMethod, const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6)
{
  unsigned char msg[255];
   short tempCmd = 0;
   msg[0] = COM_STX0; msg[1] = COM_STX1;
   msg[2] = _desID; msg[3] = 0;
   if (ctrlMethod == PWM)
    {
      msg[4] = MOTORPWMCTRLALL;
    }
    else if(ctrlMethod == Velocity)
    {
      msg[4] = MOTORVELOCITYCTRLALL;
    }
    else if(ctrlMethod == Position)
    {
      msg[4] = MOTORPOSITIONCTRLALL;
    }

    msg[5] = 12;
   for (int i = 0; i < 6; i ++)
   {
     if (i == 0)
     {
       tempCmd = cmd1;
     }
     if (i == 1)
     {
       tempCmd = cmd2;
     }
     if (i == 2)
     {
       tempCmd = cmd3;
     }
     if (i == 3)
     {
       tempCmd = cmd4;
     }
     if (i == 4)
     {
       tempCmd = cmd5;
     }
     if (i == 5)
     {
       tempCmd = cmd6;
     }
     memcpy(&msg[ 6 + i *2], &tempCmd, sizeof(short));
   }
   msg[18] = CalculateCRC(&msg[2], msg[5] + 4);
   msg[19] = COM_ETX0; msg[20] = COM_ETX1;
   return sendCommand(msg, 21);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendMotorCtrlCmd(CtrlMethod ctrlMethod, const int channel, const int cmd, const int time)
{
  unsigned char msg[255];
  short tempTime = (short)( time & 0xffff);
  if ((channel < 0) || (channel > 5))
    return -1;
  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;
  if (ctrlMethod == PWM)
  {
    msg[4] = MOTORPWMCTRL;
  }
  else if(ctrlMethod == Velocity)
  {
    msg[4] = MOTORVELOCITYCTRL;
  }
  else if(ctrlMethod == Position)
  {
    msg[4] = MOTORPOSITIONCTRL;
  }
  msg[5] = 6;
  msg[6] = (unsigned char)(channel & 0xff);
  msg[7] = (unsigned char)(cmd % 256);
  msg[8] = (unsigned char)(cmd /256);
  msg[9] = TIMEFLAG;
  memcpy(&msg[10], &tempTime, sizeof(short));
  msg[12] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[13] = COM_ETX0; msg[14] = COM_ETX1;
  return sendCommand(msg, 15);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendMotorCtrlCmd(CtrlMethod ctrlMethod,const int channel, const int cmd)
{
  unsigned char msg[255];
  if ((channel < 0) || (channel > 5))
     return -1;
  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;
  if (ctrlMethod == PWM)
  {
    msg[4] = MOTORPWMCTRL;
  }
  else if(ctrlMethod == Velocity)
  {
    msg[4] = MOTORVELOCITYCTRL;
  }
  else if(ctrlMethod == Position)
  {
    msg[4] = MOTORPOSITIONCTRL;
  }
  msg[5] = 3;
  msg[6] = (unsigned char)(channel & 0xff);
  msg[7] = (unsigned char)(cmd % 256);
  msg[8] = (unsigned char)(cmd /256);
  msg[9] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[10] = COM_ETX0; msg[11] = COM_ETX1;
  return sendCommand(msg, 12);
}


int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendServoCtrlAllCmd(const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6, const int time)
{
  unsigned char msg[255];
  short tempCmd = 0;
  short tempTime = time;
  if (tempTime <= 0) tempTime = 1;
  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;

  msg[4] = SERVOCTRLALL;
  msg[5] = 14;

  for (int i = 0; i < 6; i ++)
  {
    if (i == 0)
    {
      tempCmd = cmd1;
    }
    if (i == 1)
    {
      tempCmd = cmd2;
    }
    if (i == 2)
    {
      tempCmd = cmd3;
    }
    if (i == 3)
    {
      tempCmd = cmd4;
    }
    if (i == 4)
    {
      tempCmd = cmd5;
    }
    if (i == 5)
    {
      tempCmd = cmd6;
    }
    memcpy(&msg[ 6 + i *2], &tempCmd, sizeof(short));

  }

  
  memcpy(&msg[18], &tempTime, sizeof(short));
  msg[20] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[21] = COM_ETX0; msg[22] = COM_ETX1;
  return sendCommand(msg, 23);

}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendServoCtrlAllCmd(const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6)
{
  unsigned char msg[255];
  short tempCmd = 0;
  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;

  msg[4] = SERVOCTRLALL;
  msg[5] = 12;

  for (int i = 0; i < 6; i ++)
  {
   if (i == 0)
   {
     tempCmd = cmd1;
   }
   if (i == 1)
   {
     tempCmd = cmd2;
   }
   if (i == 2)
   {
     tempCmd = cmd3;
   }
   if (i == 3)
   {
     tempCmd = cmd4;
   }
   if (i == 4)
   {
     tempCmd = cmd5;
   }
   if (i == 5)
   {
     tempCmd = cmd6;
   }
   memcpy(&msg[ 6 + i *2], &tempCmd, sizeof(short));

  }
  msg[18] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[19] = COM_ETX0; msg[20] = COM_ETX1;
  return sendCommand(msg, 21);
}
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendServoCtrlCmd(const int channel, const int cmd, const int time)
{
  unsigned char msg[255];
  short tempTime = (short)( time & 0xffff);
  if ((channel < 0) || (channel > 5))
	return -1;
  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;

  msg[4] = SERVOCTRL;

  msg[5] = 6;
  msg[6] = (unsigned char)(channel & 0xff);
  msg[7] = (unsigned char)(cmd % 256);
  msg[8] = (unsigned char)(cmd /256);
  msg[9] = TIMEFLAG;
  memcpy(&msg[10], &tempTime, sizeof(short));
  msg[12] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[13] = COM_ETX0; msg[14] = COM_ETX1;
  return sendCommand(msg, 15);
}
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendServoCtrlCmd(const int channel, const int cmd)
{
  unsigned char msg[255];
  if ((channel < 0) || (channel > 5))
    return -1;
  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;
  msg[4] = SERVOCTRL;
  msg[5] = 3;
  msg[6] = (unsigned char)(channel & 0xff);
  msg[7] = (unsigned char)(cmd % 256);
  msg[8] = (unsigned char)(cmd /256);
  msg[9] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[10] = COM_ETX0; msg[11] = COM_ETX1;
  return sendCommand(msg, 12);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::disableMotorCmd(const int channel)
{
  unsigned char msg[255];
  if ((channel < 0) || (channel > 5))
    return -1;
  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;
  msg[4] = MOTORENABLE;
  msg[5] = 2;
  msg[6] = 0;
  msg[7] = (unsigned char)(channel & 0x0ff);

  msg[8] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[9] = COM_ETX0; msg[10] = COM_ETX1;
  return sendCommand(msg, 11);
}
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::disableServoCmd(const int channel)
{
  unsigned char msg[255];
  if ((channel < 0) || (channel > 5))
    return -1;

  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;
  msg[4] = MOTORENABLE;
  msg[5] = 2;
  msg[6] = 0;
  msg[7] = (unsigned char)( (channel & 0x0ff) + 6);

  msg[8] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[9] = COM_ETX0; msg[10] = COM_ETX1;
  return sendCommand(msg, 11);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::setMotorPositionCtrlPID(const int channel, const int kp, const int kd, const int ki)
{
  unsigned char msg[255];
  if ( (channel < 0) || (channel > 5) )
    return -1;
  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;
  msg[4] = MOTORPARAMETERSETTING;
  msg[5] = 11;
  msg[6] = POSITIONPID;
  msg[7] = (unsigned char)(channel & 0x0ff);
  msg[8] = KP_ID;
  memcpy(&msg[9],&kp, sizeof(short));
  msg[11] = KD_ID;
  memcpy(&msg[12],&kd, sizeof(short));
  msg[14] = KI_ID;
  memcpy(&msg[15],&ki, sizeof(short));
  msg[17] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[18] = COM_ETX0; msg[19] = COM_ETX1;
  return sendCommand(msg, 20);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::setMotorVelocityCtrlPID(const int channel, const int kp, const int kd, const int ki)
{
  unsigned char msg[255];
  if ( (channel < 0) || (channel > 5) )
  return -1;

  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;
  msg[4] = MOTORPARAMETERSETTING;
  msg[5] = 11;
  msg[6] = VELOCITYPID;
  msg[7] = (unsigned char)(channel & 0x0ff);
  msg[8] = KP_ID;
  memcpy(&msg[9],&kp, sizeof(short));
  msg[11] = KD_ID;
  memcpy(&msg[12],&kd, sizeof(short));
  msg[14] = KI_ID;
  memcpy(&msg[15],&ki, sizeof(short));
  msg[17] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[18] = COM_ETX0; msg[19] = COM_ETX1;
  return sendCommand(msg, 20);
}
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::setMotorFricCompensation(const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6)
{
  unsigned char msg[255];

  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;

  msg[4] = MOTORFRICCOMP;
  msg[5] = 12;

  memcpy(&msg[6], &cmd1, sizeof(short));
  memcpy(&msg[8], &cmd2, sizeof(short));
  memcpy(&msg[10], &cmd3, sizeof(short));
  memcpy(&msg[12], &cmd4, sizeof(short));
  memcpy(&msg[14], &cmd5, sizeof(short));
  memcpy(&msg[16], &cmd6, sizeof(short));


  msg[18] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[19] = COM_ETX0; msg[20] = COM_ETX1;
  return sendCommand(msg, 21);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::setCustomIO(const int cmd)
{
  unsigned char msg[255];

  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;
  msg[4] = CUSTOMIO;
  msg[5] = 1;
  msg[6] = (unsigned char)(cmd & 0x0000ff);
  msg[7] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[8] = COM_ETX0; msg[9] = COM_ETX1;
  return sendCommand(msg, 10);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendPowerCtrlCmd(const int cmd)
{
  unsigned char msg[255];

  msg[0] = COM_STX0; msg[1] = COM_STX1;
  msg[2] = _desID; msg[3] = 0;
  msg[4] = POWERCTRL;
  msg[5] = 1;
  msg[6] = (unsigned char)(cmd & 0x0000ff);
  msg[7] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[8] = COM_ETX0; msg[9] = COM_ETX1;
  return sendCommand(msg, 10);
}




int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendCommand(const unsigned char* msg, const int nLen)
{
  ssize_t retval = 0;
  if (!_stopComm)
  {

    if ( (_robotConfig->commMethod == Network) && ( _sockfd > 0))
    {
      int retval = sendto(_sockfd, msg, nLen, 0,(const struct sockaddr *)&_addr,sizeof(_addr));
      if (retval > 0)
      {
         return retval;
       }
       else
       {
         perror("sendto");
         return -1;
       }
    }
    else if( (_robotConfig->commMethod == Serial) && (_serialfd > 0))
    {
      int origflags = fcntl(_serialfd, F_GETFL,0);
      fcntl(_serialfd,F_SETFL,origflags & ~O_NONBLOCK);

      retval = write(_serialfd, msg, nLen);
      int fputserrno = errno;
      fcntl(_serialfd,F_SETFL,origflags | O_NONBLOCK);
      errno =fputserrno;
      if (retval != -1)
      {
        return retval;
      }
      else
      {
        return -1;
      }

    }
  }
  return -1;
}



