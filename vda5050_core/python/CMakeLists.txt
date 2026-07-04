find_package(ament_cmake_python REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(vda5050_master_python
  master_bindings.cpp
)

target_link_libraries(vda5050_master_python
  PRIVATE
    Python3::Python
    ${PROJECT_NAME}::master
)

install(
  TARGETS vda5050_master_python
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
)
