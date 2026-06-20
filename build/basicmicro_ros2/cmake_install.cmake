# Install script for directory: /home/agrobot/ros2_ws/src/basicmicro_ros2

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/agrobot/ros2_ws/install/basicmicro_ros2")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/basicmicro_ros2")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/msg" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/msg/TrajectoryPoint.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/msg" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/msg/PositionPoint.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/SetMotionStrategy.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/SetMotionParameters.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/MoveDistance.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/SetPositionLimits.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/GetPositionLimits.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/ExecuteTrajectory.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/ExecutePositionSequence.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/GetAvailableHomingMethods.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/GetServoStatus.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/MoveToAbsolutePosition.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/PerformHoming.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/ReleasePositionHold.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/SetDutyCycle.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/SetDutyCycleAccel.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_type_description/basicmicro_ros2/srv/SetHomingConfiguration.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/basicmicro_ros2/basicmicro_ros2" TYPE DIRECTORY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_c/basicmicro_ros2/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/environment" TYPE FILE FILES "/opt/ros/kilted/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/environment" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/libbasicmicro_ros2__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_c.so"
         OLD_RPATH "/opt/ros/kilted/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/basicmicro_ros2/basicmicro_ros2" TYPE DIRECTORY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_typesupport_fastrtps_c/basicmicro_ros2/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/libbasicmicro_ros2__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/agrobot/ros2_ws/build/basicmicro_ros2:/opt/ros/kilted/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/basicmicro_ros2/basicmicro_ros2" TYPE DIRECTORY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_typesupport_introspection_c/basicmicro_ros2/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/libbasicmicro_ros2__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/agrobot/ros2_ws/build/basicmicro_ros2:/opt/ros/kilted/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/libbasicmicro_ros2__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_c.so"
         OLD_RPATH "/home/agrobot/ros2_ws/build/basicmicro_ros2:/opt/ros/kilted/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/basicmicro_ros2/basicmicro_ros2" TYPE DIRECTORY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_cpp/basicmicro_ros2/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/basicmicro_ros2/basicmicro_ros2" TYPE DIRECTORY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_typesupport_fastrtps_cpp/basicmicro_ros2/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/libbasicmicro_ros2__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/opt/ros/kilted/lib:/home/agrobot/ros2_ws/build/basicmicro_ros2:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/basicmicro_ros2/basicmicro_ros2" TYPE DIRECTORY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_typesupport_introspection_cpp/basicmicro_ros2/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/libbasicmicro_ros2__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/home/agrobot/ros2_ws/build/basicmicro_ros2:/opt/ros/kilted/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/libbasicmicro_ros2__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_cpp.so"
         OLD_RPATH "/home/agrobot/ros2_ws/build/basicmicro_ros2:/opt/ros/kilted/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/environment" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/environment" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2-1.0.0-py3.12.egg-info" TYPE DIRECTORY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_python/basicmicro_ros2/basicmicro_ros2.egg-info/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2" TYPE DIRECTORY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_py/basicmicro_ros2/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/home/agrobot/ros2_ws/venv/bin/python3" "-m" "compileall"
        "/home/agrobot/ros2_ws/install/basicmicro_ros2/lib/python3.12/site-packages/basicmicro_ros2"
      )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2" TYPE MODULE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_py/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/agrobot/ros2_ws/build/basicmicro_ros2:/opt/ros/kilted/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/basicmicro_ros2_s__rosidl_typesupport_fastrtps_c.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2" TYPE MODULE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_py/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/agrobot/ros2_ws/build/basicmicro_ros2:/opt/ros/kilted/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/basicmicro_ros2_s__rosidl_typesupport_introspection_c.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2" TYPE MODULE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_generator_py/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_c.so"
         OLD_RPATH "/home/agrobot/ros2_ws/build/basicmicro_ros2:/opt/ros/kilted/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/basicmicro_ros2/basicmicro_ros2_s__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/basicmicro_ros2_s__rosidl_typesupport_c.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_py.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_py.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/libbasicmicro_ros2__rosidl_generator_py.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_py.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_py.so"
         OLD_RPATH "/home/agrobot/ros2_ws/build/basicmicro_ros2:/opt/ros/kilted/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libbasicmicro_ros2__rosidl_generator_py.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/msg" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/msg/TrajectoryPoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/msg" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/msg/PositionPoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/SetMotionStrategy.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/SetMotionParameters.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/MoveDistance.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/SetPositionLimits.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/GetPositionLimits.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/ExecuteTrajectory.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/ExecutePositionSequence.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/GetAvailableHomingMethods.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/GetServoStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/MoveToAbsolutePosition.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/PerformHoming.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/ReleasePositionHold.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/SetDutyCycle.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/SetDutyCycleAccel.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_adapter/basicmicro_ros2/srv/SetHomingConfiguration.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/msg" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/msg/TrajectoryPoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/msg" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/msg/PositionPoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/SetMotionStrategy.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/SetMotionParameters.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/MoveDistance.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/SetPositionLimits.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/GetPositionLimits.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/ExecuteTrajectory.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/ExecutePositionSequence.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/GetAvailableHomingMethods.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/GetServoStatus.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/MoveToAbsolutePosition.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/PerformHoming.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/ReleasePositionHold.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/SetDutyCycle.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/SetDutyCycleAccel.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/srv" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv/SetHomingConfiguration.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/basicmicro_ros2" TYPE DIRECTORY FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/basicmicro_ros2/" FILES_MATCHING REGEX "/[^/]*\\.py$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/basicmicro_ros2" TYPE PROGRAM FILES
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/basicmicro_driver/basicmicro_node.py"
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/basicmicro_driver/motion_config_service.py"
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/basicmicro_driver/distance_movement_service.py"
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/basicmicro_driver/trajectory_service.py"
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/basicmicro_driver/duty_control_service.py"
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/basicmicro_driver/servo_position_service.py"
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/basicmicro_driver/trajectory_monitor.py"
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/basicmicro_driver/performance_monitor.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/" TYPE DIRECTORY FILES
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/launch"
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/config"
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/msg"
    "/home/agrobot/ros2_ws/src/basicmicro_ros2/srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/basicmicro_ros2")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/basicmicro_ros2")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/environment" TYPE FILE FILES "/opt/ros/kilted/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/environment" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/environment" TYPE FILE FILES "/opt/ros/kilted/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/environment" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_index/share/ament_index/resource_index/packages/basicmicro_ros2")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_cExport.cmake"
         "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_generator_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_generator_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_generator_cExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cExport.cmake"
         "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_introspection_cExport.cmake"
         "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_introspection_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_introspection_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_introspection_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_introspection_cExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_cExport.cmake"
         "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_cExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_cppExport.cmake"
         "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_generator_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_generator_cppExport.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cppExport.cmake"
         "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_typesupport_fastrtps_cppExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_introspection_cppExport.cmake"
         "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_introspection_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_introspection_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_introspection_cppExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_cppExport.cmake"
         "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/basicmicro_ros2__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/basicmicro_ros2__rosidl_typesupport_cppExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_pyExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_pyExport.cmake"
         "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_generator_pyExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_pyExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake/export_basicmicro_ros2__rosidl_generator_pyExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_generator_pyExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/CMakeFiles/Export/a200e5e11145bd1f96a7a46aa1e9aa04/export_basicmicro_ros2__rosidl_generator_pyExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES "/home/agrobot/ros2_ws/build/basicmicro_ros2/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2/cmake" TYPE FILE FILES
    "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_core/basicmicro_ros2Config.cmake"
    "/home/agrobot/ros2_ws/build/basicmicro_ros2/ament_cmake_core/basicmicro_ros2Config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basicmicro_ros2" TYPE FILE FILES "/home/agrobot/ros2_ws/src/basicmicro_ros2/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/agrobot/ros2_ws/build/basicmicro_ros2/basicmicro_ros2__py/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/agrobot/ros2_ws/build/basicmicro_ros2/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
