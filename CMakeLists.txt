cmake_minimum_required(VERSION 3.0.2)

# Python 3 interpreter'ı bulmak için politikayı ayarla
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

project(gokhas_interface)

# FindPythonInterp için eski yöntemi kullan
find_package(PythonInterp 3 REQUIRED)

## Sık kullanılan paketleri bulun
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

###################################
## catkin özellikleri konfigürasyonu ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Python paketi kurulumunu etkinleştir
catkin_python_setup()

#############
## Install ##
#############

# Python scriptlerini kurma
catkin_install_python(PROGRAMS
  scripts/launch_interface.py
  scripts/camera_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Launch dosyalarını kurma
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# PYTHONPATH ayarı
set(PYTHONPATH ${CMAKE_CURRENT_SOURCE_DIR}/scripts/src)