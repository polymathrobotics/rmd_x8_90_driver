#ifndef RMD_X8_90_DRIVER__VISIBILITY_CONTROL_HPP_
#define RMD_X8_90_DRIVER__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RMD_X8_90_DRIVER_EXPORT __attribute__ ((dllexport))
    #define RMD_X8_90_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define RMD_X8_90_DRIVER_EXPORT __declspec(dllexport)
    #define RMD_X8_90_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef RMD_X8_90_DRIVER_BUILDING_LIBRARY
    #define RMD_X8_90_DRIVER_PUBLIC RMD_X8_90_DRIVER_EXPORT
  #else
    #define RMD_X8_90_DRIVER_PUBLIC RMD_X8_90_DRIVER_IMPORT
  #endif
  #define RMD_X8_90_DRIVER_PUBLIC_TYPE RMD_X8_90_DRIVER_PUBLIC
  #define RMD_X8_90_DRIVER_LOCAL
#else
  #define RMD_X8_90_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define RMD_X8_90_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define RMD_X8_90_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define RMD_X8_90_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RMD_X8_90_DRIVER_PUBLIC
    #define RMD_X8_90_DRIVER_LOCAL
  #endif
  #define RMD_X8_90_DRIVER_PUBLIC_TYPE
#endif

#endif  // RMD_X8_90_DRIVER__VISIBILITY_CONTROL_HPP_
