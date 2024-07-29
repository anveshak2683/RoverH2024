// Generated by gencpp from file navigation2/detection.msg
// DO NOT EDIT!


#ifndef NAVIGATION2_MESSAGE_DETECTION_H
#define NAVIGATION2_MESSAGE_DETECTION_H


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
struct detection_
{
  typedef detection_<ContainerAllocator> Type;

  detection_()
    : color()
    , depth(0.0)  {
    }
  detection_(const ContainerAllocator& _alloc)
    : color(_alloc)
    , depth(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _color_type;
  _color_type color;

   typedef float _depth_type;
  _depth_type depth;





  typedef boost::shared_ptr< ::navigation2::detection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navigation2::detection_<ContainerAllocator> const> ConstPtr;

}; // struct detection_

typedef ::navigation2::detection_<std::allocator<void> > detection;

typedef boost::shared_ptr< ::navigation2::detection > detectionPtr;
typedef boost::shared_ptr< ::navigation2::detection const> detectionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::navigation2::detection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::navigation2::detection_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::navigation2::detection_<ContainerAllocator1> & lhs, const ::navigation2::detection_<ContainerAllocator2> & rhs)
{
  return lhs.color == rhs.color &&
    lhs.depth == rhs.depth;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::navigation2::detection_<ContainerAllocator1> & lhs, const ::navigation2::detection_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace navigation2

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::navigation2::detection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation2::detection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation2::detection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation2::detection_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation2::detection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation2::detection_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::navigation2::detection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "54cdd292028a384f7f8301d873bb5638";
  }

  static const char* value(const ::navigation2::detection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x54cdd292028a384fULL;
  static const uint64_t static_value2 = 0x7f8301d873bb5638ULL;
};

template<class ContainerAllocator>
struct DataType< ::navigation2::detection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navigation2/detection";
  }

  static const char* value(const ::navigation2::detection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::navigation2::detection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string color\n"
"float32 depth\n"
;
  }

  static const char* value(const ::navigation2::detection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::navigation2::detection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.color);
      stream.next(m.depth);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct detection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navigation2::detection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::navigation2::detection_<ContainerAllocator>& v)
  {
    s << indent << "color: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.color);
    s << indent << "depth: ";
    Printer<float>::stream(s, indent + "  ", v.depth);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAVIGATION2_MESSAGE_DETECTION_H