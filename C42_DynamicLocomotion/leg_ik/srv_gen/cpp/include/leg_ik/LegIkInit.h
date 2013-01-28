/* Auto-generated by genmsg_cpp for file /home/darpa/rosworkspace/leg_ik/srv/LegIkInit.srv */
#ifndef LEG_IK_SERVICE_LEGIKINIT_H
#define LEG_IK_SERVICE_LEGIKINIT_H
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

#include "ros/service_traits.h"

#include "leg_ik/traj.h"


#include "leg_ik/LegAngle.h"

namespace leg_ik
{
template <class ContainerAllocator>
struct LegIkInitRequest_ {
  typedef LegIkInitRequest_<ContainerAllocator> Type;

  LegIkInitRequest_()
  : pos()
  {
  }

  LegIkInitRequest_(const ContainerAllocator& _alloc)
  : pos(_alloc)
  {
  }

  typedef  ::leg_ik::traj_<ContainerAllocator>  _pos_type;
   ::leg_ik::traj_<ContainerAllocator>  pos;


  typedef boost::shared_ptr< ::leg_ik::LegIkInitRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::leg_ik::LegIkInitRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LegIkInitRequest
typedef  ::leg_ik::LegIkInitRequest_<std::allocator<void> > LegIkInitRequest;

typedef boost::shared_ptr< ::leg_ik::LegIkInitRequest> LegIkInitRequestPtr;
typedef boost::shared_ptr< ::leg_ik::LegIkInitRequest const> LegIkInitRequestConstPtr;


template <class ContainerAllocator>
struct LegIkInitResponse_ {
  typedef LegIkInitResponse_<ContainerAllocator> Type;

  LegIkInitResponse_()
  : ang()
  {
  }

  LegIkInitResponse_(const ContainerAllocator& _alloc)
  : ang(_alloc)
  {
  }

  typedef  ::leg_ik::LegAngle_<ContainerAllocator>  _ang_type;
   ::leg_ik::LegAngle_<ContainerAllocator>  ang;


  typedef boost::shared_ptr< ::leg_ik::LegIkInitResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::leg_ik::LegIkInitResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LegIkInitResponse
typedef  ::leg_ik::LegIkInitResponse_<std::allocator<void> > LegIkInitResponse;

typedef boost::shared_ptr< ::leg_ik::LegIkInitResponse> LegIkInitResponsePtr;
typedef boost::shared_ptr< ::leg_ik::LegIkInitResponse const> LegIkInitResponseConstPtr;

struct LegIkInit
{

typedef LegIkInitRequest Request;
typedef LegIkInitResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct LegIkInit
} // namespace leg_ik

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::leg_ik::LegIkInitRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::leg_ik::LegIkInitRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::leg_ik::LegIkInitRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e199d522e509fc3ab2b108c764fa0e52";
  }

  static const char* value(const  ::leg_ik::LegIkInitRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe199d522e509fc3aULL;
  static const uint64_t static_value2 = 0xb2b108c764fa0e52ULL;
};

template<class ContainerAllocator>
struct DataType< ::leg_ik::LegIkInitRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "leg_ik/LegIkInitRequest";
  }

  static const char* value(const  ::leg_ik::LegIkInitRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::leg_ik::LegIkInitRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "leg_ik/traj pos\n\
\n\
================================================================================\n\
MSG: leg_ik/traj\n\
float64 COMx\n\
float64 COMy\n\
float64 COMz\n\
float64 Swing_x\n\
float64 Swing_y\n\
float64 Swing_z\n\
int32   leg\n\
\n\
";
  }

  static const char* value(const  ::leg_ik::LegIkInitRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::leg_ik::LegIkInitRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::leg_ik::LegIkInitResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::leg_ik::LegIkInitResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::leg_ik::LegIkInitResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "328a48585f80aa570f8bf4ab43f39a9f";
  }

  static const char* value(const  ::leg_ik::LegIkInitResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x328a48585f80aa57ULL;
  static const uint64_t static_value2 = 0x0f8bf4ab43f39a9fULL;
};

template<class ContainerAllocator>
struct DataType< ::leg_ik::LegIkInitResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "leg_ik/LegIkInitResponse";
  }

  static const char* value(const  ::leg_ik::LegIkInitResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::leg_ik::LegIkInitResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "leg_ik/LegAngle ang\n\
\n\
\n\
================================================================================\n\
MSG: leg_ik/LegAngle\n\
float64 mhx\n\
float64 lhy\n\
float64 uhz\n\
float64 kny\n\
float64 lax\n\
float64 uay\n\
float64 mby\n\
float64 ubx\n\
\n\
";
  }

  static const char* value(const  ::leg_ik::LegIkInitResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::leg_ik::LegIkInitResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::leg_ik::LegIkInitRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.pos);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LegIkInitRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::leg_ik::LegIkInitResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ang);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LegIkInitResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<leg_ik::LegIkInit> {
  static const char* value() 
  {
    return "b11527b1e6596cdbc96c7b94f75be9c7";
  }

  static const char* value(const leg_ik::LegIkInit&) { return value(); } 
};

template<>
struct DataType<leg_ik::LegIkInit> {
  static const char* value() 
  {
    return "leg_ik/LegIkInit";
  }

  static const char* value(const leg_ik::LegIkInit&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<leg_ik::LegIkInitRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b11527b1e6596cdbc96c7b94f75be9c7";
  }

  static const char* value(const leg_ik::LegIkInitRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<leg_ik::LegIkInitRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "leg_ik/LegIkInit";
  }

  static const char* value(const leg_ik::LegIkInitRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<leg_ik::LegIkInitResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b11527b1e6596cdbc96c7b94f75be9c7";
  }

  static const char* value(const leg_ik::LegIkInitResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<leg_ik::LegIkInitResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "leg_ik/LegIkInit";
  }

  static const char* value(const leg_ik::LegIkInitResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // LEG_IK_SERVICE_LEGIKINIT_H

