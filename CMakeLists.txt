project( pedestrian_detection_calibration )
cmake_minimum_required(VERSION 2.8)  

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package( OpenCV REQUIRED )
file(GLOB all_pedestrian_detection_calibration_src
    "include/*.h"
    "src/*.cpp"
)

include_directories("include/")
add_executable( pedestrian_detection_calibration 
    ${all_pedestrian_detection_calibration_src}
)

target_link_libraries(pedestrian_detection_calibration ${OpenCV_LIBS} ${CONFIG_LIB})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=c++1y -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS} -mavx -o3")
