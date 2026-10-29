cmake_minimum_required(VERSION 3.5)
project(description)

find_package(ament_cmake REQUIRED)

# Instala as pastas cruciais para que o launch file as encontre
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()