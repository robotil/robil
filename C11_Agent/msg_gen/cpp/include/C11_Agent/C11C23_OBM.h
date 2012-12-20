/* Auto-generated by genmsg_cpp for file /home/ariy/robil/C11_Agent/msg/C11C23_OBM.msg */
#ifndef C11_AGENT_MESSAGE_C11C23_OBM_H
#define C11_AGENT_MESSAGE_C11C23_OBM_H
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


namespace C11_Agent
{
template <class ContainerAllocator>
struct C11C23_OBM_ {
  typedef C11C23_OBM_<ContainerAllocator> Type;

  C11C23_OBM_()
  : LAT(0.0)
  , SCN(0)
  , MOV(0)
  {
  }

  C11C23_OBM_(const ContainerAllocator& _alloc)
  : LAT(0.0)
  , SCN(0)
  , MOV(0)
  {
  }

  typedef double _LAT_type;
  double LAT;

  typedef int16_t _SCN_type;
  int16_t SCN;

  typedef int16_t _MOV_type;
  int16_t MOV;

  enum { SCN_SCAN = 0 };
  enum { SCN_CURRENT = 1 };
  enum { MOV_NONE = 0 };
  enum { MOV_HEAD = 1 };
  enum { MOV_POSTURE = 2 };
  enum { MOV_POSITION = 3 };

  typedef boost::shared_ptr< ::C11_Agent::C11C23_OBM_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::C11_Agent::C11C23_OBM_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct C11C23_OBM
typedef  ::C11_Agent::C11C23_OBM_<std::allocator<void> > C11C23_OBM;

typedef boost::shared_ptr< ::C11_Agent::C11C23_OBM> C11C23_OBMPtr;
typedef boost::shared_ptr< ::C11_Agent::C11C23_OBM const> C11C23_OBMConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::C11_Agent::C11C23_OBM_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::C11_Agent::C11C23_OBM_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace C11_Agent

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::C11_Agent::C11C23_OBM_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::C11_Agent::C11C23_OBM_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::C11_Agent::C11C23_OBM_<ContainerAllocator> > {
  static const char* value() 
  {
    return "021b2b252d90f9fb11b61af4d8bd8a44";
  }

  static const char* value(const  ::C11_Agent::C11C23_OBM_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x021b2b252d90f9fbULL;
  static const uint64_t static_value2 = 0x11b61af4d8bd8a44ULL;
};

template<class ContainerAllocator>
struct DataType< ::C11_Agent::C11C23_OBM_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C11_Agent/C11C23_OBM";
  }

  static const char* value(const  ::C11_Agent::C11C23_OBM_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::C11_Agent::C11C23_OBM_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 LAT\n\
int16 SCN\n\
int16 SCN_SCAN=0\n\
int16 SCN_CURRENT=1\n\
int16 MOV\n\
int16 MOV_NONE     = 0\n\
int16 MOV_HEAD     = 1\n\
int16 MOV_POSTURE  = 2\n\
int16 MOV_POSITION = 3\n\
";
  }

  static const char* value(const  ::C11_Agent::C11C23_OBM_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::C11_Agent::C11C23_OBM_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::C11_Agent::C11C23_OBM_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.LAT);
    stream.next(m.SCN);
    stream.next(m.MOV);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct C11C23_OBM_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::C11_Agent::C11C23_OBM_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::C11_Agent::C11C23_OBM_<ContainerAllocator> & v) 
  {
    s << indent << "LAT: ";
    Printer<double>::stream(s, indent + "  ", v.LAT);
    s << indent << "SCN: ";
    Printer<int16_t>::stream(s, indent + "  ", v.SCN);
    s << indent << "MOV: ";
    Printer<int16_t>::stream(s, indent + "  ", v.MOV);
  }
};


} // namespace message_operations
} // namespace ros

#endif // C11_AGENT_MESSAGE_C11C23_OBM_H

