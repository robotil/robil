/* Auto-generated by genmsg_cpp for file /home/ariy/robil/C51_CarOperation/msg/C0C51_ST.msg */
#ifndef C51_CAROPERATION_MESSAGE_C0C51_ST_H
#define C51_CAROPERATION_MESSAGE_C0C51_ST_H
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


namespace C51_CarOperation
{
template <class ContainerAllocator>
struct C0C51_ST_ {
  typedef C0C51_ST_<ContainerAllocator> Type;

  C0C51_ST_()
  : car_start_stop(0)
  {
  }

  C0C51_ST_(const ContainerAllocator& _alloc)
  : car_start_stop(0)
  {
  }

  typedef int32_t _car_start_stop_type;
  int32_t car_start_stop;

  enum { START_CAR = 1 };
  enum { STOP_CAR = 0 };

  typedef boost::shared_ptr< ::C51_CarOperation::C0C51_ST_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::C51_CarOperation::C0C51_ST_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct C0C51_ST
typedef  ::C51_CarOperation::C0C51_ST_<std::allocator<void> > C0C51_ST;

typedef boost::shared_ptr< ::C51_CarOperation::C0C51_ST> C0C51_STPtr;
typedef boost::shared_ptr< ::C51_CarOperation::C0C51_ST const> C0C51_STConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::C51_CarOperation::C0C51_ST_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::C51_CarOperation::C0C51_ST_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace C51_CarOperation

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::C51_CarOperation::C0C51_ST_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::C51_CarOperation::C0C51_ST_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::C51_CarOperation::C0C51_ST_<ContainerAllocator> > {
  static const char* value() 
  {
    return "026fc3434718efcfbf63a9a40356a01e";
  }

  static const char* value(const  ::C51_CarOperation::C0C51_ST_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x026fc3434718efcfULL;
  static const uint64_t static_value2 = 0xbf63a9a40356a01eULL;
};

template<class ContainerAllocator>
struct DataType< ::C51_CarOperation::C0C51_ST_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C51_CarOperation/C0C51_ST";
  }

  static const char* value(const  ::C51_CarOperation::C0C51_ST_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::C51_CarOperation::C0C51_ST_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 car_start_stop\n\
int32 START_CAR=1\n\
int32 STOP_CAR=0\n\
\n\
";
  }

  static const char* value(const  ::C51_CarOperation::C0C51_ST_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::C51_CarOperation::C0C51_ST_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::C51_CarOperation::C0C51_ST_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.car_start_stop);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct C0C51_ST_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::C51_CarOperation::C0C51_ST_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::C51_CarOperation::C0C51_ST_<ContainerAllocator> & v) 
  {
    s << indent << "car_start_stop: ";
    Printer<int32_t>::stream(s, indent + "  ", v.car_start_stop);
  }
};


} // namespace message_operations
} // namespace ros

#endif // C51_CAROPERATION_MESSAGE_C0C51_ST_H

