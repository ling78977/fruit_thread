cmake_minimum_required(VERSION 3.10.0)
project(fruit VERSION 2023.7.12 LANGUAGES CXX)
# Find and add opencv as required
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# add_library(serial_port SHARED src/module/uart/uart.cpp)
add_library(detect_turn SHARED src/module/turn/turn.cpp)

add_library(coor_2_claw SHARED src/module/coor2caw/coor2claw.cpp)

add_library(qr_decodor SHARED src/module/qr/qr.cpp)

add_library(detect_ball SHARED src/module/ball/ball.cpp)

add_library(uart SHARED src/module/uart/uart.cpp) 

add_library(socket SHARED src/module/cocket/socket.cpp)


target_link_libraries(detect_turn ${OpenCV_LIBS})
target_link_libraries(detect_ball ${OpenCV_LIBS})
target_link_libraries(qr_decodor ${OpenCV_LIBS})
target_link_libraries(uart ${OpenCV_LIBS})
target_link_libraries(coor_2_claw ${OpenCV_LIBS})



set_target_properties(detect_turn PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ../lib
)
set_target_properties(coor_2_claw PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ../lib
)
set_target_properties(qr_decodor PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ../lib
)
set_target_properties(detect_ball PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ../lib
)
set_target_properties(uart PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ../lib
)




add_executable(testballdetect utils/testballdetect.cpp)
target_link_libraries(testballdetect detect_ball)

add_executable(test_turn utils/testturn.cpp)
target_link_libraries(test_turn detect_turn)

add_executable(test_qr utils/test_qr.cpp)
target_link_libraries(test_qr qr_decodor)

add_executable(test_coord utils/test_coor2clw.cpp)
target_link_libraries(test_coord coor_2_claw)



