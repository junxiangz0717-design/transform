set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(decision_semantic_map STATIC src/semantic_map.cpp)
target_include_directories(decision_semantic_map PUBLIC include PRIVATE ../../include/path)
target_link_libraries(decision_semantic_map PRIVATE tinyxml2)

add_executable(semantic_test test/test.cpp)
target_link_libraries(semantic_test decision_semantic_map)

add_executable(angle_test test/angle_test.cpp)
target_link_libraries(angle_test decision_semantic_map)

add_executable(25_test01 test/25_test01.cpp)
target_link_libraries(25_test01 decision_semantic_map)

add_executable(self_circle test/self_circle.cpp)
target_link_libraries(self_circle decision_semantic_map)