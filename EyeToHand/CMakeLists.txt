CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(EyeHandCal)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
vtk_module_config(VTK
  vtkCommonCore
  vtkFiltersSources
  vtkFiltersSources
  vtkInteractionStyle
  vtkRendering${VTK_RENDERING_BACKEND}
)
include(${VTK_USE_FILE})

include_directories("/usr/include/eigen3")
include_directories("./include/")
add_compile_options(-std=c++11)

add_executable( solution ./src/main.cpp ./src/utils.cpp)
target_link_libraries( solution ${PCL_LIBRARIES} ${VTK_LIBRARIES} )