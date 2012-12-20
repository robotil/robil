/* Auto-generated by genmsg_cpp for file /home/ariy/robil/C51_CarOperation/msg/C51C0_NOR.msg */
#ifndef C51_CAROPERATION_MESSAGE_C51C0_NOR_H
#define C51_CAROPERATION_MESSAGE_C51C0_NOR_H
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
struct C51C0_NOR_ {
  typedef C51C0_NOR_<ContainerAllocator> Type;

  C51C0_NOR_()
  : normal_abnormal_travel(0)
  {
  }

  C51C0_NOR_(const ContainerAllocator& _alloc)
  : normal_abnormal_travel(0)
  {
  }

  typedef int32_t _normal_abnormal_travel_type;
  int32_t normal_abnormal_travel;

  enum { NORMAL_TRAVEL = 0 };
  enum { ABNORMAL_TRAVEL = 1 };

  typedef boost::shared_ptr< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct C51C0_NOR
typedef  ::C51_CarOperation::C51C0_NOR_<std::allocator<void> > C51C0_NOR;

typedef boost::shared_ptr< ::C51_CarOperation::C51C0_NOR> C51C0_NORPtr;
typedef boost::shared_ptr< ::C51_CarOperation::C51C0_NOR const> C51C0_NORConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace C51_CarOperation

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> > {
  static const char* value() 
  {
    return "239a790af8bbde68db44417642b6eec8";
  }

  static const char* value(const  ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x239a790af8bbde68ULL;
  static const uint64_t static_value2 = 0xdb44417642b6eec8ULL;
};

template<class ContainerAllocator>
struct DataType< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C51_CarOperation/C51C0_NOR";
  }

  static const char* value(const  ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 normal_abnormal_travel\n\
int32 NORMAL_TRAVEL=0\n\
int32 ABNORMAL_TRAVEL=1\n\
\n\
";
  }

  static const char* value(const  ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.normal_abnormal_travel);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct C51C0_NOR_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::C51_CarOperation::C51C0_NOR_<ContainerAllocator> & v) 
  {
    s << indent << "normal_abnormal_travel: ";
    Printer<int32_t>::stream(s, indent + "  ", v.normal_abnormal_travel);
  }
};


} // namespace message_operations
} // namespace ros

#endif // C51_CAROPERATION_MESSAGE_C51C0_NOR_H

