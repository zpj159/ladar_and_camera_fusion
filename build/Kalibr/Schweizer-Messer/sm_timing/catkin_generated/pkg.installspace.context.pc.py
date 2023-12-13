# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/home/zpj/ladar_and_camera_fusion/src/Kalibr/Schweizer-Messer/sm_common/include;/home/zpj/ladar_and_camera_fusion/src/Kalibr/Schweizer-Messer/sm_random/include;/usr/local/include".split(';') if "${prefix}/include;/home/zpj/ladar_and_camera_fusion/src/Kalibr/Schweizer-Messer/sm_common/include;/home/zpj/ladar_and_camera_fusion/src/Kalibr/Schweizer-Messer/sm_random/include;/usr/local/include" != "" else []
PROJECT_CATKIN_DEPENDS = "sm_common;sm_random".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsm_timing".split(';') if "-lsm_timing" != "" else []
PROJECT_NAME = "sm_timing"
PROJECT_SPACE_DIR = "/home/zpj/ladar_and_camera_fusion/install"
PROJECT_VERSION = "0.0.1"
