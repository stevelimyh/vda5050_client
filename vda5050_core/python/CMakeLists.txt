find_package(ament_cmake_python REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(vda5050_master_python
  master_bindings.cpp
  src/bind_types.cpp
  src/bind_factsheet.cpp
  src/bind_master.cpp
)

target_link_libraries(vda5050_master_python
  PRIVATE
    Python3::Python
    ${PROJECT_NAME}::master
)

target_include_directories(vda5050_master_python
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(
  TARGETS vda5050_master_python
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
)
