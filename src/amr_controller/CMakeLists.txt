cmake_minimum_required(VERSION 3.5)
project(amr_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
   add_compile_options(-WALL -WEXTRA -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
 
  install(
     DIRECTORY CONFIG launch
     DESTINATION SHARE/${PROJECT_NAME}
  )
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()