#ifndef MOTOR_HARDWARE_INTERFACE_HPP__VISIBILITY_CONTROL_H_
#define MOTOR_HARDWARE_INTERFACE_HPP__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MOTOR_HARDWARE_INTERFACE_HPP_EXPORT __attribute__((dllexport))
#define MOTOR_HARDWARE_INTERFACE_HPP_IMPORT __attribute__((dllimport))
#else
#define MOTOR_HARDWARE_INTERFACE_HPP_EXPORT __declspec(dllexport)
#define MOTOR_HARDWARE_INTERFACE_HPP_IMPORT __declspec(dllimport)
#endif
#ifdef MOTOR_HARDWARE_INTERFACE_HPP_BUILDING_DLL
#define MOTOR_HARDWARE_INTERFACE_HPP_PUBLIC MOTOR_HARDWARE_INTERFACE_HPP_EXPORT
#else
#define MOTOR_HARDWARE_INTERFACE_HPP_PUBLIC MOTOR_HARDWARE_INTERFACE_HPP_IMPORT
#endif
#define MOTOR_HARDWARE_INTERFACE_HPP_PUBLIC_TYPE MOTOR_HARDWARE_INTERFACE_HPP_PUBLIC
#define MOTOR_HARDWARE_INTERFACE_HPP_LOCAL
#else
#define MOTOR_HARDWARE_INTERFACE_HPP_EXPORT __attribute__((visibility("default")))
#define MOTOR_HARDWARE_INTERFACE_HPP_IMPORT
#if __GNUC__ >= 4
#define MOTOR_HARDWARE_INTERFACE_HPP_PUBLIC __attribute__((visibility("default")))
#define MOTOR_HARDWARE_INTERFACE_HPP_LOCAL __attribute__((visibility("hidden")))
#else
#define MOTOR_HARDWARE_INTERFACE_HPP_PUBLIC
#define MOTOR_HARDWARE_INTERFACE_HPP_LOCAL
#endif
#define MOTOR_HARDWARE_INTERFACE_HPP_PUBLIC_TYPE
#endif

#endif  // MOTOR_HARDWARE_INTERFACE_HPP__VISIBILITY_CONTROL_H_