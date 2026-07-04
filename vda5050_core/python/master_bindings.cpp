/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

#include <memory>

#include "vda5050_core/python/master_callbacks.hpp"
#include "vda5050_core/python/register.hpp"

namespace py = pybind11;
namespace pym = vda5050_core::python::master;

PYBIND11_MODULE(vda5050_master_python, m)
{
  m.doc() = "VDA5050 fleet-master Python bindings";

  auto m_types = m.def_submodule("types", "VDA5050 message types and enums");
  vda5050_core::python::register_types(m_types);

  auto m_master = m.def_submodule("master", "VDA5050 fleet master API");
  vda5050_core::python::register_master(m_master);

  py::class_<pym::PyMaster, std::shared_ptr<pym::PyMaster>>(m_master, "Master")
    .def(
      py::init(&pym::PyMaster::create), py::arg("broker_address"),
      py::arg("client_id"))
    .def_readwrite("on_state", &pym::PyMaster::on_state_cb)
    .def_readwrite("on_connection", &pym::PyMaster::on_connection_cb)
    .def_readwrite("on_factsheet", &pym::PyMaster::on_factsheet_cb)
    .def_readwrite("on_visualization", &pym::PyMaster::on_visualization_cb);
}
