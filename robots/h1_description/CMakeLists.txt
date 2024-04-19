cmake_minimum_required(VERSION 3.5)
project(h1_description)

find_package(ament_cmake REQUIRED)

find_package(xacro REQUIRED)


foreach(dir launch meshes urdf)
  install(DIRECTORY ${dir}/
          DESTINATION ${PROJECT_NAME}/${dir})
endforeach(dir)

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
