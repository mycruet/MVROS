##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(qtest)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp image_transport  nodelet  message_generation  test_nodelet)
find_package(cv_bridge REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_nodelet
    CATKIN_DEPENDS  message_runtime
   #DEPENDS  OpenCV
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/qtest/*.h)
message(">>>>>>>> ${QT_FORMS}>>>>>")
message(">>>>>>>> ${QT_RESOURCES}>>>>>")
message(">>>>>>>> ${QT_MOC}>>>>>")
QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
message(">>>>>>>> ${QT_MOC_HPP}>>>>>")


add_executable(qtest ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
message(">>>>>>>> ${QT_LIBRARIES}>>>>>")

message("[\n${catkin_LIBRARIES}\n]")
message("[\n${OpenCV_LIBRARIES}\n]")
message("[\n${cv_bridge_LIBRARIES}\n]")

target_link_libraries(qtest ${QT_LIBRARIES} ${catkin_LIBRARIES} ${cv_bridge_LIBRARIES})
install(TARGETS qtest RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

