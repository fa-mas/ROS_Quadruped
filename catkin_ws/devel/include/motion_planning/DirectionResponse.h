// Generated by gencpp from file motion_planning/DirectionResponse.msg
// DO NOT EDIT!


#ifndef MOTION_PLANNING_MESSAGE_DIRECTIONRESPONSE_H
#define MOTION_PLANNING_MESSAGE_DIRECTIONRESPONSE_H


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
struct DirectionResponse_
{
  typedef DirectionResponse_<ContainerAllocator> Type;

  DirectionResponse_()
    : dir()
    , ang(0.0)
    , vec()  {
      vec.assign(0.0);
  }
  DirectionResponse_(const ContainerAllocator& _alloc)
    : dir(_alloc)
    , ang(0.0)
    , vec()  {
  (void)_alloc;
      vec.assign(0.0);
  }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _dir_type;
  _dir_type dir;

   typedef float _ang_type;
  _ang_type ang;

   typedef boost::array<float, 3>  _vec_type;
  _vec_type vec;





  typedef boost::shared_ptr< ::motion_planning::DirectionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motion_planning::DirectionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DirectionResponse_

typedef ::motion_planning::DirectionResponse_<std::allocator<void> > DirectionResponse;

typedef boost::shared_ptr< ::motion_planning::DirectionResponse > DirectionResponsePtr;
typedef boost::shared_ptr< ::motion_planning::DirectionResponse const> DirectionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motion_planning::DirectionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motion_planning::DirectionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::motion_planning::DirectionResponse_<ContainerAllocator1> & lhs, const ::motion_planning::DirectionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.dir == rhs.dir &&
    lhs.ang == rhs.ang &&
    lhs.vec == rhs.vec;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::motion_planning::DirectionResponse_<ContainerAllocator1> & lhs, const ::motion_planning::DirectionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace motion_planning

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::motion_planning::DirectionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motion_planning::DirectionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motion_planning::DirectionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motion_planning::DirectionResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motion_planning::DirectionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motion_planning::DirectionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motion_planning::DirectionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e9c03e485183aab1a8776292dfa81ccd";
  }

  static const char* value(const ::motion_planning::DirectionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe9c03e485183aab1ULL;
  static const uint64_t static_value2 = 0xa8776292dfa81ccdULL;
};

template<class ContainerAllocator>
struct DataType< ::motion_planning::DirectionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motion_planning/DirectionResponse";
  }

  static const char* value(const ::motion_planning::DirectionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motion_planning::DirectionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string dir\n"
"float32 ang\n"
"float32[3] vec\n"
;
  }

  static const char* value(const ::motion_planning::DirectionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motion_planning::DirectionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.dir);
      stream.next(m.ang);
      stream.next(m.vec);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DirectionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motion_planning::DirectionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motion_planning::DirectionResponse_<ContainerAllocator>& v)
  {
    s << indent << "dir: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.dir);
    s << indent << "ang: ";
    Printer<float>::stream(s, indent + "  ", v.ang);
    s << indent << "vec[]" << std::endl;
    for (size_t i = 0; i < v.vec.size(); ++i)
    {
      s << indent << "  vec[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.vec[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTION_PLANNING_MESSAGE_DIRECTIONRESPONSE_H
