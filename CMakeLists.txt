cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME HammeringTaskNew)
set(PROJECT_DESCRIPTION "HammeringTaskNew")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)
add_subdirectory(plugin)

set(AROBASE "@")
configure_file(etc/HammeringTaskNew.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/HammeringTaskNew.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/HammeringTaskNew.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/HammeringTaskNew.yaml")
set(STATES_INSTALL_PREFIX ${MC_CONTROLLER_INSTALL_PREFIX})

configure_file(etc/HammeringTaskNew.in.yaml "${CONFIG_OUT}" @ONLY)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)
