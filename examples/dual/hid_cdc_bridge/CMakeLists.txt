cmake_minimum_required(VERSION 3.20)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../hw/bsp/family_support.cmake)

project(hid_cdc_bridge C CXX ASM)

# Checks this example is valid for the family and initializes the project
family_initialize_project(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR})

add_executable(${PROJECT_NAME})

# Example source
target_sources(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_descriptors.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge_protocol.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/usbfs_cdc.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/usbfs_dcd.c
  )

# Example include
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

# Configure compilation flags and libraries for the example without RTOS.
family_configure_dual_usb_example(${PROJECT_NAME} noos)
