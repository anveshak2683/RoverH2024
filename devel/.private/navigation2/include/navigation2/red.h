// Generated by gencpp from file navigation2/red.msg
// DO NOT EDIT!


#ifndef NAVIGATION2_MESSAGE_RED_H
#define NAVIGATION2_MESSAGE_RED_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace navigation2
{
template <class ContainerAllocator>
struct red_
{
  typedef red_<ContainerAllocator> Type;

  red_()
    : vel(0)
    , omega(0)
    , detect(false)  {
    }
  red_(const ContainerAllocator& _alloc)
    : vel(0)
    , omega(0)
    , detect(false)  {
  (void)_alloc;
    }



   typedef int16_t _vel_type;
  _vel_type vel;

   typedef int16_t _omega_type;
  _omega_type omega;

   typedef uint8_t _detect_type;
  _detect_type detect;





  typedef boost::shared_ptr< ::navigation2::red_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navigation2::red_<ContainerAllocator> const> ConstPtr;

}; // struct red_

typedef ::navigation2::red_<std::allocator<void> > red;

typedef boost::shared_ptr< ::navigation2::red > redPtr;
typedef boost::shared_ptr< ::navigation2::red const> redConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::navigation2::red_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::navigation2::red_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::navigation2::red_<ContainerAllocator1> & lhs, const ::navigation2::red_<ContainerAllocator2> & rhs)
{
  return lhs.vel == rhs.vel &&
    lhs.omega == rhs.omega &&
    lhs.detect == rhs.detect;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::navigation2::red_<ContainerAllocator1> & lhs, const ::navigation2::red_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace navigation2

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::navigation2::red_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation2::red_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation2::red_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation2::red_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation2::red_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation2::red_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::navigation2::red_<ContainerAllocator> >
{
  static const char* value()
  {
    return "598737a047b4631c88d712e78758247d";
  }

  static const char* value(const ::navigation2::red_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x598737a047b4631cULL;
  static const uint64_t static_value2 = 0x88d712e78758247dULL;
};

template<class ContainerAllocator>
struct DataType< ::navigation2::red_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navigation2/red";
  }

  static const char* value(const ::navigation2::red_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::navigation2::red_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 vel\n"
"int16 omega\n"
"bool detect\n"
"\n"
;
  }

  static const char* value(const ::navigation2::red_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::navigation2::red_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.vel);
      stream.next(m.omega);
      stream.next(m.detect);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct red_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navigation2::red_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::navigation2::red_<ContainerAllocator>& v)
  {
    s << indent << "vel: ";
    Printer<int16_t>::stream(s, indent + "  ", v.vel);
    s << indent << "omega: ";
    Printer<int16_t>::stream(s, indent + "  ", v.omega);
    s << indent << "detect: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.detect);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAVIGATION2_MESSAGE_RED_H
