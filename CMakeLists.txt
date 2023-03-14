cmake_minimum_required(VERSION 3.11.3)

# Set the C++ standard we will use
set(CMAKE_CXX_STANDARD 17)

# Add the path of the cmake files to the CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

project(OpenStreetMap_Render)

# Set library output path to /lib
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")

# Locate project prerequisites
find_package(io2d REQUIRED)
find_package(Cairo)
find_package(GraphicsMagick)

# Add the pugixml library subdirectories
add_subdirectory(thirdparty/pugixml)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../dash_code/dash_ipc/dashIceoryx ${CMAKE_BINARY_DIR}/dashIceoryx EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../dash_code/dash_types ${CMAKE_BINARY_DIR}/dash_types EXCLUDE_FROM_ALL)

# Add project executable
add_executable(OpenStreetMapRender src/main.cpp src/model.cpp src/render.cpp src/draw_image.cpp)

target_link_libraries(OpenStreetMapRender
    PRIVATE 
     io2d::io2d
    PUBLIC 
     pugixml
     dash::iceoryx 
     dash::types
     dash::serialiser
)

# Set options for Linux or Microsoft Visual C++
if( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
    target_link_libraries(OpenStreetMapRender PUBLIC pthread)
endif()

if(MSVC)
	target_compile_options(OpenStreetMapRender PUBLIC /D_SILENCE_CXX17_ALLOCATOR_VOID_DEPRECATION_WARNING /wd4459)
endif()