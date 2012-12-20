/* Auto-generated by genmsg_cpp for file /home/ariy/robil/C11_Agent/srv/override_object_properties.srv */
#ifndef C11_AGENT_SERVICE_OVERRIDE_OBJECT_PROPERTIES_H
#define C11_AGENT_SERVICE_OVERRIDE_OBJECT_PROPERTIES_H
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

#include "C11_Agent/C11C23_OBP.h"



namespace C11_Agent
{
template <class ContainerAllocator>
struct override_object_propertiesRequest_ {
  typedef override_object_propertiesRequest_<ContainerAllocator> Type;

  override_object_propertiesRequest_()
  : obp()
  {
  }

  override_object_propertiesRequest_(const ContainerAllocator& _alloc)
  : obp(_alloc)
  {
  }

  typedef  ::C11_Agent::C11C23_OBP_<ContainerAllocator>  _obp_type;
   ::C11_Agent::C11C23_OBP_<ContainerAllocator>  obp;


  typedef boost::shared_ptr< ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct override_object_propertiesRequest
typedef  ::C11_Agent::override_object_propertiesRequest_<std::allocator<void> > override_object_propertiesRequest;

typedef boost::shared_ptr< ::C11_Agent::override_object_propertiesRequest> override_object_propertiesRequestPtr;
typedef boost::shared_ptr< ::C11_Agent::override_object_propertiesRequest const> override_object_propertiesRequestConstPtr;


template <class ContainerAllocator>
struct override_object_propertiesResponse_ {
  typedef override_object_propertiesResponse_<ContainerAllocator> Type;

  override_object_propertiesResponse_()
  {
  }

  override_object_propertiesResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct override_object_propertiesResponse
typedef  ::C11_Agent::override_object_propertiesResponse_<std::allocator<void> > override_object_propertiesResponse;

typedef boost::shared_ptr< ::C11_Agent::override_object_propertiesResponse> override_object_propertiesResponsePtr;
typedef boost::shared_ptr< ::C11_Agent::override_object_propertiesResponse const> override_object_propertiesResponseConstPtr;

struct override_object_properties
{

typedef override_object_propertiesRequest Request;
typedef override_object_propertiesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct override_object_properties
} // namespace C11_Agent

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "80a5a708c1560a51133202325435fc54";
  }

  static const char* value(const  ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x80a5a708c1560a51ULL;
  static const uint64_t static_value2 = 0x133202325435fc54ULL;
};

template<class ContainerAllocator>
struct DataType< ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C11_Agent/override_object_propertiesRequest";
  }

  static const char* value(const  ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C11_Agent/C11C23_OBP   obp\n\
\n\
\n\
================================================================================\n\
MSG: C11_Agent/C11C23_OBP\n\
int16 ACT\n\
int16 ACT_MODIFIED = 0\n\
int16 ACT_NEW      = 1\n\
int16 FRZ\n\
int16 FRZ_KEEP  = 0\n\
int16 ACT_RETRY = 1\n\
int16 ID\n\
string NAME\n\
C11_Agent/pathLocation[] EXTR_VIS\n\
C11_Agent/pathLocation[] EXTR_TOP\n\
C11_Agent/D3SPACE ORI\n\
================================================================================\n\
MSG: C11_Agent/pathLocation\n\
float64 lat\n\
float64 lon\n\
================================================================================\n\
MSG: C11_Agent/D3SPACE\n\
float64 ROLL\n\
float64 PITCH\n\
float64 YAW\n\
";
  }

  static const char* value(const  ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C11_Agent/override_object_propertiesResponse";
  }

  static const char* value(const  ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::C11_Agent::override_object_propertiesRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.obp);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct override_object_propertiesRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::C11_Agent::override_object_propertiesResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct override_object_propertiesResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<C11_Agent::override_object_properties> {
  static const char* value() 
  {
    return "80a5a708c1560a51133202325435fc54";
  }

  static const char* value(const C11_Agent::override_object_properties&) { return value(); } 
};

template<>
struct DataType<C11_Agent::override_object_properties> {
  static const char* value() 
  {
    return "C11_Agent/override_object_properties";
  }

  static const char* value(const C11_Agent::override_object_properties&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<C11_Agent::override_object_propertiesRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "80a5a708c1560a51133202325435fc54";
  }

  static const char* value(const C11_Agent::override_object_propertiesRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<C11_Agent::override_object_propertiesRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C11_Agent/override_object_properties";
  }

  static const char* value(const C11_Agent::override_object_propertiesRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<C11_Agent::override_object_propertiesResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "80a5a708c1560a51133202325435fc54";
  }

  static const char* value(const C11_Agent::override_object_propertiesResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<C11_Agent::override_object_propertiesResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "C11_Agent/override_object_properties";
  }

  static const char* value(const C11_Agent::override_object_propertiesResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // C11_AGENT_SERVICE_OVERRIDE_OBJECT_PROPERTIES_H

