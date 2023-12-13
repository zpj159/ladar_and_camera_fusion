execute_process(COMMAND "/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_optimizer/aslam_backend_python/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/zpj/ladar_and_camera_fusion/build/Kalibr/aslam_optimizer/aslam_backend_python/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
