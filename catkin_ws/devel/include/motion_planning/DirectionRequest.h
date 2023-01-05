// Generated by gencpp from file motion_planning/DirectionRequest.msg
// DO NOT EDIT!


#ifndef MOTION_PLANNING_MESSAGE_DIRECTIONREQUEST_H
#define MOTION_PLANNING_MESSAGE_DIRECTIONREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace motion_planning
{
template <class ContainerAllocator>
struct DirectionRequest_
{
  typedef DirectionRequest_<ContainerAllocator> Type;

  DirectionRequest_()
    : ready(false)  {
    }
  DirectionRequest_(const ContainerAllocator& _alloc)
    : ready(false)  {
  (void)_alloc;
    }



   typedef uint8_t _ready_type;
  _ready_type ready;





  typedef boost::shared_ptr< ::motion_planning::DirectionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motion_planning::DirectionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct DirectionRequest_

typedef ::motion_planning::DirectionRequest_<std::allocator<void> > DirectionRequest;

typedef boost::shared_ptr< ::motion_planning::DirectionRequest > DirectionRequestPtr;
typedef boost::shared_ptr< ::motion_planning::DirectionRequest const> DirectionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motion_planning::DirectionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motion_planning::DirectionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::motion_planning::DirectionRequest_<ContainerAllocator1> & lhs, const ::motion_planning::DirectionRequest_<ContainerAllocator2> & rhs)
{
  return lhs.ready == rhs.ready;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::motion_planning::DirectionRequest_<ContainerAllocator1> & lhs, const ::motion_planning::DirectionRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace motion_planning

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::motion_planning::DirectionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motion_planning::DirectionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motion_planning::DirectionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motion_planning::DirectionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motion_planning::DirectionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motion_planning::DirectionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motion_planning::DirectionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6f378c6311f9e6ccd2cd8c5b327003f1";
  }

  static const char* value(const ::motion_planning::DirectionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6f378c6311f9e6ccULL;
  static const uint64_t static_value2 = 0xd2cd8c5b327003f1ULL;
};

template<class ContainerAllocator>
struct DataType< ::motion_planning::DirectionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motion_planning/DirectionRequest";
  }

  static const char* value(const ::motion_planning::DirectionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motion_planning::DirectionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool ready\n"
;
  }

  static const char* value(const ::motion_planning::DirectionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motion_planning::DirectionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ready);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DirectionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motion_planning::DirectionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motion_planning::DirectionRequest_<ContainerAllocator>& v)
  {
    s << indent << "ready: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ready);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTION_PLANNING_MESSAGE_DIRECTIONREQUEST_H