cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project (pcl-visualizer)

list (APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package (Qt5Widgets REQUIRED)
find_package (VTK 6.3 REQUIRED COMPONENTS vtkGUISupportQt vtkRenderingQt)
find_package (PCL 1.8 REQUIRED COMPONENTS common io visualization)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set (project_SOURCES main.cpp pclviewer.cpp)
set (project_HEADERS pclviewer.h)
set (project_FORMS   pclviewer.ui)

# Setup moc and uic for Qt 5
set (CMAKE_INCLUDE_CURRENT_DIR ON)
QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

add_executable (pcl_visualizer ${project_SOURCES} ${project_FORMS_HEADERS} ${project_HEADERS_MOC})
target_link_libraries (pcl_visualizer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
