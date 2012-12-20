/* Auto-generated by genmsg_cpp for file /home/ariy/robil/C51_CarOperation/msg/C51C0_OPO.msg */
#ifndef C51_CAROPERATION_MESSAGE_C51C0_OPO_H
#define C51_CAROPERATION_MESSAGE_C51C0_OPO_H
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
struct C51C0_OPO_ {
  typedef C51C0_OPO_<ContainerAllocator> Type;

  C51C0_OPO_()
  : x(0)
  , y(0)
  , z(0)
  {
  }

  C51C0_OPO_(const ContainerAllocator& _alloc)
  : x(0)
  , y(0)
  , z(0)
  {
  }

  typedef int32_t _x_type;
  int32_t x;

  typedef int32_t _y_type;
  int32_t y;

  typedef int32_t _z_type;
  int32_t z;


  typedef boost::shared_ptr< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct C51C0_OPO
typedef  ::C51_CarOperation::C51C0_OPO_<std::allocator<void> > C51C0_OPO;

typedef boost::shared_ptr< ::C51_CarOperation::C51C0_OPO> C51C0_OPOPtr;
typedef boost::shared_ptr< ::C51_CarOperation::C51C0_OPO const> C51C0_OPOConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace C51_CarOperation

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3cb41a2c4416de195dbb95b7777a06fb";
  }

  static const char* value(const  ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3cb41a2c4416de19ULL;
  static const uint64_t static_value2 = 0x5dbb95b7777a06fbULL;
};

template<class ContainerAllocator>
struct DataType< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C51_CarOperation/C51C0_OPO";
  }

  static const char* value(const  ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 x\n\
int32 y\n\
int32 z\n\
\n\
";
  }

  static const char* value(const  ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct C51C0_OPO_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::C51_CarOperation::C51C0_OPO_<ContainerAllocator> & v) 
  {
    s << indent << "x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<int32_t>::stream(s, indent + "  ", v.z);
  }
};


} // namespace message_operations
} // namespace ros

#endif // C51_CAROPERATION_MESSAGE_C51C0_OPO_H

