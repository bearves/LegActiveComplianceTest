# Find the QtWidgets library
find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5Network)
find_package(Qt5PrintSupport)
# Instruct CMake to run moc automatically when needed.

# Thus the autogenerated ui_mainwindow.h can be found
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(../)

# Tell CMake to create the RobotUI executable
add_executable(RobotUI main.cpp mainwindow.cpp qcustomplot.cpp ParamSetWindow.cpp)

# Use the Widgets module from Qt 5.
target_link_libraries(RobotUI Qt5::Widgets Qt5::Network Qt5::PrintSupport ${ARIS_NORT_LINK_LIBS})
