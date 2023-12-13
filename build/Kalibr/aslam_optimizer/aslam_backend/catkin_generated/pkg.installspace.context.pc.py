# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include".split(';') if "${prefix}/include;/usr/local/include" != "" else []
PROJECT_CATKIN_DEPENDS = "sparse_block_matrix;sm_boost;sm_random;sm_timing;sm_logging;sm_property_tree".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-laslam_backend;/usr/local/lib/libboost_system.so.1.80.0;/usr/local/lib/libboost_thread.so.1.80.0".split(';') if "-laslam_backend;/usr/local/lib/libboost_system.so.1.80.0;/usr/local/lib/libboost_thread.so.1.80.0" != "" else []
PROJECT_NAME = "aslam_backend"
PROJECT_SPACE_DIR = "/home/zpj/ladar_and_camera_fusion/install"
PROJECT_VERSION = "0.0.1"
