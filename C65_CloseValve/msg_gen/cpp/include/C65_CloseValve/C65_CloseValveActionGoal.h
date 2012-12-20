/* Auto-generated by genmsg_cpp for file /home/ariy/robil/C65_CloseValve/msg/C65_CloseValveActionGoal.msg */
#ifndef C65_CLOSEVALVE_MESSAGE_C65_CLOSEVALVEACTIONGOAL_H
#define C65_CLOSEVALVE_MESSAGE_C65_CLOSEVALVEACTIONGOAL_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "C65_CloseValve/C65_CloseValveGoal.h"

namespace C65_CloseValve
{
template <class ContainerAllocator>
struct C65_CloseValveActionGoal_ {
  typedef C65_CloseValveActionGoal_<ContainerAllocator> Type;

  C65_CloseValveActionGoal_()
  : header()
  , goal_id()
  , goal()
  {
  }

  C65_CloseValveActionGoal_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , goal_id(_alloc)
  , goal(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
   ::actionlib_msgs::GoalID_<ContainerAllocator>  goal_id;

  typedef  ::C65_CloseValve::C65_CloseValveGoal_<ContainerAllocator>  _goal_type;
   ::C65_CloseValve::C65_CloseValveGoal_<ContainerAllocator>  goal;


  typedef boost::shared_ptr< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct C65_CloseValveActionGoal
typedef  ::C65_CloseValve::C65_CloseValveActionGoal_<std::allocator<void> > C65_CloseValveActionGoal;

typedef boost::shared_ptr< ::C65_CloseValve::C65_CloseValveActionGoal> C65_CloseValveActionGoalPtr;
typedef boost::shared_ptr< ::C65_CloseValve::C65_CloseValveActionGoal const> C65_CloseValveActionGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace C65_CloseValve

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "da4cc7a37cf48a380afe9d5f2a30b123";
  }

  static const char* value(const  ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xda4cc7a37cf48a38ULL;
  static const uint64_t static_value2 = 0x0afe9d5f2a30b123ULL;
};

template<class ContainerAllocator>
struct DataType< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C65_CloseValve/C65_CloseValveActionGoal";
  }

  static const char* value(const  ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
C65_CloseValveGoal goal\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: C65_CloseValve/C65_CloseValveGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal definition\n\
float32[2] location\n\
int32 max\n\
\n\
";
  }

  static const char* value(const  ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.goal_id);
    stream.next(m.goal);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct C65_CloseValveActionGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::C65_CloseValve::C65_CloseValveActionGoal_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
s << std::endl;
    Printer< ::C65_CloseValve::C65_CloseValveGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};


} // namespace message_operations
} // namespace ros

#endif // C65_CLOSEVALVE_MESSAGE_C65_CLOSEVALVEACTIONGOAL_H

