project( pedestrian_detection_calibration )
cmake_minimum_required(VERSION 2.8)  

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

find_package( OpenCV REQUIRED )
file(GLOB all_pedestrian_detection_calibration_src
    "include/*.h"
    "src/*.cpp"
)

include_directories("include/")
add_library( pedestrian_detection_calibration SHARED
    ${all_pedestrian_detection_calibration_src}
)

target_link_libraries(pedestrian_detection_calibration ${OpenCV_LIBS} ${CONFIG_LIB})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=c++1y -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS} -mavx -o3")
