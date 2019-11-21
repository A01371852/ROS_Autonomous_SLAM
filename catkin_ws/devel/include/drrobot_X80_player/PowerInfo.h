// Generated by gencpp from file drrobot_X80_player/PowerInfo.msg
// DO NOT EDIT!


#ifndef DRROBOT_X80_PLAYER_MESSAGE_POWERINFO_H
#define DRROBOT_X80_PLAYER_MESSAGE_POWERINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace drrobot_X80_player
{
template <class ContainerAllocator>
struct PowerInfo_
{
  typedef PowerInfo_<ContainerAllocator> Type;

  PowerInfo_()
    : header()
    , robot_type()
    , bat1_vol(0.0)
    , bat2_vol(0.0)
    , bat1_temp(0.0)
    , bat2_temp(0.0)
    , dcin_vol(0.0)
    , ref_vol(0.0)
    , power_status(0)
    , power_path(0)
    , charge_path(0)  {
    }
  PowerInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , robot_type(_alloc)
    , bat1_vol(0.0)
    , bat2_vol(0.0)
    , bat1_temp(0.0)
    , bat2_temp(0.0)
    , dcin_vol(0.0)
    , ref_vol(0.0)
    , power_status(0)
    , power_path(0)
    , charge_path(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _robot_type_type;
  _robot_type_type robot_type;

   typedef float _bat1_vol_type;
  _bat1_vol_type bat1_vol;

   typedef float _bat2_vol_type;
  _bat2_vol_type bat2_vol;

   typedef float _bat1_temp_type;
  _bat1_temp_type bat1_temp;

   typedef float _bat2_temp_type;
  _bat2_temp_type bat2_temp;

   typedef float _dcin_vol_type;
  _dcin_vol_type dcin_vol;

   typedef float _ref_vol_type;
  _ref_vol_type ref_vol;

   typedef uint8_t _power_status_type;
  _power_status_type power_status;

   typedef uint8_t _power_path_type;
  _power_path_type power_path;

   typedef uint8_t _charge_path_type;
  _charge_path_type charge_path;





  typedef boost::shared_ptr< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> const> ConstPtr;

}; // struct PowerInfo_

typedef ::drrobot_X80_player::PowerInfo_<std::allocator<void> > PowerInfo;

typedef boost::shared_ptr< ::drrobot_X80_player::PowerInfo > PowerInfoPtr;
typedef boost::shared_ptr< ::drrobot_X80_player::PowerInfo const> PowerInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::drrobot_X80_player::PowerInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace drrobot_X80_player

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'drrobot_X80_player': ['/home/emmo/catkin_ws/src/drrobot_X80_player/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "13107d877ae888e7541f720d1432d852";
  }

  static const char* value(const ::drrobot_X80_player::PowerInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x13107d877ae888e7ULL;
  static const uint64_t static_value2 = 0x541f720d1432d852ULL;
};

template<class ContainerAllocator>
struct DataType< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "drrobot_X80_player/PowerInfo";
  }

  static const char* value(const ::drrobot_X80_player::PowerInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# battery /power information message from DrRobot Robot.\n\
\n\
Header header    	# timestamp in the header is the time the driver\n\
		 	# returned the battery/power reading\n\
string robot_type	# robot type, I90 series, sentinel3 or Hawk/H20 Power/Motion\n\
\n\
#below message is only I90 series with Power control system on robot, otherwise reserved\n\
float32 bat1_vol	# battery1 voltage\n\
float32 bat2_vol	# battery2 voltage\n\
float32 bat1_temp	# battery1 temperature reading, now only is the AD value\n\
float32 bat2_temp	# battery2 temperature reading, now only is the AD value\n\
float32 dcin_vol	# dcin power voltage reading\n\
float32 ref_vol		# board AD reference voltage reading\n\
uint8 power_status	# power status, referee document to get detailed info for every bit\n\
uint8 power_path	# power selected path, please referee DrRobot document\n\
uint8 charge_path	# charger selected path, please referee DrRobot document\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::drrobot_X80_player::PowerInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.robot_type);
      stream.next(m.bat1_vol);
      stream.next(m.bat2_vol);
      stream.next(m.bat1_temp);
      stream.next(m.bat2_temp);
      stream.next(m.dcin_vol);
      stream.next(m.ref_vol);
      stream.next(m.power_status);
      stream.next(m.power_path);
      stream.next(m.charge_path);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PowerInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::drrobot_X80_player::PowerInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::drrobot_X80_player::PowerInfo_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "robot_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.robot_type);
    s << indent << "bat1_vol: ";
    Printer<float>::stream(s, indent + "  ", v.bat1_vol);
    s << indent << "bat2_vol: ";
    Printer<float>::stream(s, indent + "  ", v.bat2_vol);
    s << indent << "bat1_temp: ";
    Printer<float>::stream(s, indent + "  ", v.bat1_temp);
    s << indent << "bat2_temp: ";
    Printer<float>::stream(s, indent + "  ", v.bat2_temp);
    s << indent << "dcin_vol: ";
    Printer<float>::stream(s, indent + "  ", v.dcin_vol);
    s << indent << "ref_vol: ";
    Printer<float>::stream(s, indent + "  ", v.ref_vol);
    s << indent << "power_status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.power_status);
    s << indent << "power_path: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.power_path);
    s << indent << "charge_path: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.charge_path);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DRROBOT_X80_PLAYER_MESSAGE_POWERINFO_H
