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

#ifndef VDA5050_CORE__PYTHON__REGISTER_HPP_
#define VDA5050_CORE__PYTHON__REGISTER_HPP_

#include <pybind11/pybind11.h>

namespace vda5050_core {

namespace python {

/// \brief Bind VDA5050 message types and enums into module `m`.
///
/// Every type is registered module_local so this module can be imported
/// alongside another pybind module that binds the same vda5050_core::types.
void register_types(pybind11::module_& m);

/// \brief Bind master control results, snapshots, and enums into module `m`.
void register_master(pybind11::module_& m);

/// \brief Bind the AGV factsheet types and enums into module `m`.
void register_factsheet(pybind11::module_& m);

}  // namespace python
}  // namespace vda5050_core

#endif  // VDA5050_CORE__PYTHON__REGISTER_HPP_
