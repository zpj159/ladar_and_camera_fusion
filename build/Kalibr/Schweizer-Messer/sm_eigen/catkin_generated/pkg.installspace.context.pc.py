# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include/eigen3".split(';') if "${prefix}/include;/usr/local/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "sm_common;sm_random".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsm_eigen".split(';') if "-lsm_eigen" != "" else []
PROJECT_NAME = "sm_eigen"
PROJECT_SPACE_DIR = "/home/zpj/ladar_and_camera_fusion/install"
PROJECT_VERSION = "0.0.1"
