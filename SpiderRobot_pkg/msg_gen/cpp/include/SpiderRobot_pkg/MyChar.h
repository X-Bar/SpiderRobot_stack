/* Auto-generated by genmsg_cpp for file /home/cody/groovy_workspace/GitHubRepos/SpiderRobot_stack/SpiderRobot_pkg/msg/MyChar.msg */
#ifndef SPIDERROBOT_PKG_MESSAGE_MYCHAR_H
#define SPIDERROBOT_PKG_MESSAGE_MYCHAR_H
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


namespace SpiderRobot_pkg
{
template <class ContainerAllocator>
struct MyChar_ {
  typedef MyChar_<ContainerAllocator> Type;

  MyChar_()
  : data(0)
  {
  }

  MyChar_(const ContainerAllocator& _alloc)
  : data(0)
  {
  }

  typedef int16_t _data_type;
  int16_t data;


  typedef boost::shared_ptr< ::SpiderRobot_pkg::MyChar_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::SpiderRobot_pkg::MyChar_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MyChar
typedef  ::SpiderRobot_pkg::MyChar_<std::allocator<void> > MyChar;

typedef boost::shared_ptr< ::SpiderRobot_pkg::MyChar> MyCharPtr;
typedef boost::shared_ptr< ::SpiderRobot_pkg::MyChar const> MyCharConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::SpiderRobot_pkg::MyChar_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::SpiderRobot_pkg::MyChar_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace SpiderRobot_pkg

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::SpiderRobot_pkg::MyChar_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::SpiderRobot_pkg::MyChar_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::SpiderRobot_pkg::MyChar_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8524586e34fbd7cb1c08c5f5f1ca0e57";
  }

  static const char* value(const  ::SpiderRobot_pkg::MyChar_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8524586e34fbd7cbULL;
  static const uint64_t static_value2 = 0x1c08c5f5f1ca0e57ULL;
};

template<class ContainerAllocator>
struct DataType< ::SpiderRobot_pkg::MyChar_<ContainerAllocator> > {
  static const char* value() 
  {
    return "SpiderRobot_pkg/MyChar";
  }

  static const char* value(const  ::SpiderRobot_pkg::MyChar_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::SpiderRobot_pkg::MyChar_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int16 data\n\
\n\
\n\
";
  }

  static const char* value(const  ::SpiderRobot_pkg::MyChar_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::SpiderRobot_pkg::MyChar_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::SpiderRobot_pkg::MyChar_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MyChar_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::SpiderRobot_pkg::MyChar_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::SpiderRobot_pkg::MyChar_<ContainerAllocator> & v) 
  {
    s << indent << "data: ";
    Printer<int16_t>::stream(s, indent + "  ", v.data);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SPIDERROBOT_PKG_MESSAGE_MYCHAR_H

