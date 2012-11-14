/* Auto-generated by genmsg_cpp for file /home/ariy/robil/C23_ObjectRecognition/msg/C23C0_OPO.msg */
#ifndef C23_OBJECTRECOGNITION_MESSAGE_C23C0_OPO_H
#define C23_OBJECTRECOGNITION_MESSAGE_C23C0_OPO_H
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

#include "C23_ObjectRecognition/TBD.h"

namespace C23_ObjectRecognition
{
template <class ContainerAllocator>
struct C23C0_OPO_ {
  typedef C23C0_OPO_<ContainerAllocator> Type;

  C23C0_OPO_()
  : position()
  {
  }

  C23C0_OPO_(const ContainerAllocator& _alloc)
  : position(_alloc)
  {
  }

  typedef  ::C23_ObjectRecognition::TBD_<ContainerAllocator>  _position_type;
   ::C23_ObjectRecognition::TBD_<ContainerAllocator>  position;


  typedef boost::shared_ptr< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct C23C0_OPO
typedef  ::C23_ObjectRecognition::C23C0_OPO_<std::allocator<void> > C23C0_OPO;

typedef boost::shared_ptr< ::C23_ObjectRecognition::C23C0_OPO> C23C0_OPOPtr;
typedef boost::shared_ptr< ::C23_ObjectRecognition::C23C0_OPO const> C23C0_OPOConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace C23_ObjectRecognition

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ba97431ecd8f967f52f49310867af44a";
  }

  static const char* value(const  ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xba97431ecd8f967fULL;
  static const uint64_t static_value2 = 0x52f49310867af44aULL;
};

template<class ContainerAllocator>
struct DataType< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C23_ObjectRecognition/C23C0_OPO";
  }

  static const char* value(const  ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C23_ObjectRecognition/TBD position\n\
\n\
================================================================================\n\
MSG: C23_ObjectRecognition/TBD\n\
int32 x\n\
int32 y\n\
int32 z\n\
\n\
";
  }

  static const char* value(const  ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.position);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct C23C0_OPO_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::C23_ObjectRecognition::C23C0_OPO_<ContainerAllocator> & v) 
  {
    s << indent << "position: ";
s << std::endl;
    Printer< ::C23_ObjectRecognition::TBD_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
  }
};


} // namespace message_operations
} // namespace ros

#endif // C23_OBJECTRECOGNITION_MESSAGE_C23C0_OPO_H

