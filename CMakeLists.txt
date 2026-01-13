cmake_minimum_required(VERSION 3.8)
project(coresense_understanding)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(coresense_vampire REQUIRED)
find_package(coresense_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(nlohmann_json_schema_validator_vendor REQUIRED)
 
# remove warnings
add_definitions(-w) 


include_directories(include)

add_library(theory STATIC src/theory.cpp)
add_library(model STATIC src/model.cpp)
add_library(agent_model STATIC src/agent_model.cpp)
add_library(understanding_graph STATIC src/understanding_graph.cpp)
add_library(vampire_interface STATIC src/vampire_interface.cpp)



add_executable(understanding_system_node src/coresense_understanding.cpp)


target_include_directories(understanding_system_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(understanding_system_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(
  model
  ${nlohmann_json_schema_validator_vendor_TARGETS}
  )
  
target_link_libraries(agent_model
  model
  ${nlohmann_json_schema_validator_vendor_TARGETS}
  )
  
target_link_libraries(understanding_graph
  uuid
  )
  
target_link_libraries(vampire_interface
  understanding_graph
  tinyxml2::tinyxml2
  )
  

  target_link_libraries(theory
  rclcpp::rclcpp
  )

target_link_libraries(understanding_system_node PUBLIC
  ${coresense_msgs_TARGETS}
  agent_model
  theory
  understanding_graph
  vampire_interface
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  )

ament_python_install_package(${PROJECT_NAME})

# install python node
install(PROGRAMS
  scripts/understanding_system_node.py
  DESTINATION lib/${PROJECT_NAME})

# install cpp node
install(TARGETS understanding_system_node
  DESTINATION lib/${PROJECT_NAME})

# install understanding logic
install(
  DIRECTORY understanding-logic
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
