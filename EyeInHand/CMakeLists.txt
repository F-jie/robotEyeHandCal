cmake_miniMum_required(VERSION 3.3 FATAL_ERROR)

project(EyeIH)

# PCL relative package
find_package(PCL 1.7 REQUIRED COMPONENTS
  )
if (NOT PCL_FOUND)
    message("skipping EyeIH: ${PCL_FOUND_MESSAGE}")
    return ()
endif ()
message(STATUS "PCL_VERSION: ${PCL_VERSION}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# VTK relative package
find_package(VTK REQUIRED COMPONENTS
    vtkCommonCore
    vtkFiltersSources
    vtkFiltersSources
    vtkInteractionStyle
    vtkRendering${VTK_RENDERING_BACKEND}
)
if (NOT VTK_FOUND)
    message("skipping EyeIH: ${VTK_FOUND_MESSAGE}")
    return ()
endif ()
message(STATUS "VTK_VERSION: ${VTK_VERSION}")
if (VTK_VERSION VERSION_LESS "8.90.0")
    # old system
    include(${VTK_USE_FILE})
endif ()

include_directories("/usr/include/eigen3")
include_directories("./include/")
add_compile_options(-std=c++14)

add_executable(solution ./src/main.cpp ./src/utils.cpp ./src/EyeIH.cpp)
target_link_libraries( solution ${PCL_LIBRARIES} ${VTK_LIBRARIES} )
