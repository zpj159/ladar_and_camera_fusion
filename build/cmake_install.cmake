# Install script for directory: /home/zpj/ladar_and_camera_fusion/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zpj/ladar_and_camera_fusion/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zpj/ladar_and_camera_fusion/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zpj/ladar_and_camera_fusion/install" TYPE PROGRAM FILES "/home/zpj/ladar_and_camera_fusion/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zpj/ladar_and_camera_fusion/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zpj/ladar_and_camera_fusion/install" TYPE PROGRAM FILES "/home/zpj/ladar_and_camera_fusion/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zpj/ladar_and_camera_fusion/install/setup.bash;/home/zpj/ladar_and_camera_fusion/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zpj/ladar_and_camera_fusion/install" TYPE FILE FILES
    "/home/zpj/ladar_and_camera_fusion/build/catkin_generated/installspace/setup.bash"
    "/home/zpj/ladar_and_camera_fusion/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zpj/ladar_and_camera_fusion/install/setup.sh;/home/zpj/ladar_and_camera_fusion/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zpj/ladar_and_camera_fusion/install" TYPE FILE FILES
    "/home/zpj/ladar_and_camera_fusion/build/catkin_generated/installspace/setup.sh"
    "/home/zpj/ladar_and_camera_fusion/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zpj/ladar_and_camera_fusion/install/setup.zsh;/home/zpj/ladar_and_camera_fusion/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zpj/ladar_and_camera_fusion/install" TYPE FILE FILES
    "/home/zpj/ladar_and_camera_fusion/build/catkin_generated/installspace/setup.zsh"
    "/home/zpj/ladar_and_camera_fusion/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zpj/ladar_and_camera_fusion/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zpj/ladar_and_camera_fusion/install" TYPE FILE FILES "/home/zpj/ladar_and_camera_fusion/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zpj/ladar_and_camera_fusion/build/gtest/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_cv/aslam_time/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/catkin_simple/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_offline_calibration/ethz_apriltag2/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/opencv2_catkin/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/python_module/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/realsense-ros/realsense2_description/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_common/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_boost/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_logging/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_matrix_archive/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_opencv/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_property_tree/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_random/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_eigen/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_kinematics/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_cv/aslam_cameras/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_cv/aslam_cv_serialization/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_timing/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_optimizer/sparse_block_matrix/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_optimizer/aslam_backend/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_optimizer/aslam_backend_expressions/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_cv/aslam_cv_backend/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_nonparametric_estimation/bsplines/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_incremental_calibration/incremental_calibration/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/numpy_eigen/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_optimizer/aslam_backend_python/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_cv/aslam_imgproc/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_nonparametric_estimation/aslam_splines/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_cv/aslam_cv_error_terms/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_nonparametric_estimation/bsplines_python/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_nonparametric_estimation/aslam_splines_python/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_incremental_calibration/incremental_calibration_python/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_python/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_cv/aslam_cameras_april/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_cv/aslam_cv_python/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_cv/aslam_cv_backend_python/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_offline_calibration/kalibr/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/rplidar_ros/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/CamLaserCalibraTool-master/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/realsense-ros/realsense2_camera/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/single_ladar_and_camera_fusion/cmake_install.cmake")
  include("/home/zpj/ladar_and_camera_fusion/build/usb_cam/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/zpj/ladar_and_camera_fusion/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
