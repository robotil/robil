/* Auto-generated by genmsg_cpp for file /home/darpa/Projects/Robil/C42_DynamicLocomotion/msg/C42_ZmpWlkFeedback.msg */
#ifndef C42_DYNAMICLOCOMOTION_MESSAGE_C42_ZMPWLKFEEDBACK_H
#define C42_DYNAMICLOCOMOTION_MESSAGE_C42_ZMPWLKFEEDBACK_H
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


namespace C42_DynamicLocomotion
{
template <class ContainerAllocator>
struct C42_ZmpWlkFeedback_ {
  typedef C42_ZmpWlkFeedback_<ContainerAllocator> Type;

  C42_ZmpWlkFeedback_()
  : dis_to_goal(0.0)
  {
  }

  C42_ZmpWlkFeedback_(const ContainerAllocator& _alloc)
  : dis_to_goal(0.0)
  {
  }

  typedef double _dis_to_goal_type;
  double dis_to_goal;


  typedef boost::shared_ptr< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct C42_ZmpWlkFeedback
typedef  ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<std::allocator<void> > C42_ZmpWlkFeedback;

typedef boost::shared_ptr< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback> C42_ZmpWlkFeedbackPtr;
typedef boost::shared_ptr< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback const> C42_ZmpWlkFeedbackConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace C42_DynamicLocomotion

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f680b2b504540b8e6a64539381e80bb3";
  }

  static const char* value(const  ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf680b2b504540b8eULL;
  static const uint64_t static_value2 = 0x6a64539381e80bb3ULL;
};

template<class ContainerAllocator>
struct DataType< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C42_DynamicLocomotion/C42_ZmpWlkFeedback";
  }

  static const char* value(const  ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback\n\
float64 dis_to_goal\n\
\n\
\n\
";
  }

  static const char* value(const  ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.dis_to_goal);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct C42_ZmpWlkFeedback_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::C42_DynamicLocomotion::C42_ZmpWlkFeedback_<ContainerAllocator> & v) 
  {
    s << indent << "dis_to_goal: ";
    Printer<double>::stream(s, indent + "  ", v.dis_to_goal);
  }
};


} // namespace message_operations
} // namespace ros

#endif // C42_DYNAMICLOCOMOTION_MESSAGE_C42_ZMPWLKFEEDBACK_H

