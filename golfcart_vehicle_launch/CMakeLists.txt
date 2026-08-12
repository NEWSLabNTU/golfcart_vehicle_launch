cmake_minimum_required(VERSION 3.5)
project(golfcart_vehicle_launch)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

install(PROGRAMS
  scripts/teleop_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installed to share/ rather than lib/ so launch files can name it with a plain
# $(find-pkg-share ...) path - it is a launch-prefix wrapper, not a node.
install(PROGRAMS
  scripts/run_in_tmux.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
