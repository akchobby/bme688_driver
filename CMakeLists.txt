cmake_minimum_required(VERSION 3.8)
project(bme688_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gas_sensor_msgs REQUIRED)


ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
        ${PROJECT_NAME}/gas_sensor.py
        ${PROJECT_NAME}/${PROJECT_NAME}.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
