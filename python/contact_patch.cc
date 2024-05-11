//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2024 INRIA
//  Author: Louis Montaut
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of INRIA nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#include <eigenpy/eigenpy.hpp>

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/contact_patch.h>
#include <hpp/fcl/serialization/collision_data.h>

#include "fcl.hh"
#include "deprecation.hh"
#include "serializable.hh"

#ifdef HPP_FCL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#include "doxygen_autodoc/hpp/fcl/collision_data.h"
#endif

#include "../doc/python/doxygen.hh"
#include "../doc/python/doxygen-boost.hh"

using namespace boost::python;
using namespace hpp::fcl;
using namespace hpp::fcl::python;

namespace dv = doxygen::visitor;

void exposeContactPatchAPI() {
  if (!eigenpy::register_symbolic_link_to_registered_type<
          ContactPatch::PatchDirection>()) {
    enum_<ContactPatch::PatchDirection>("ContactPatchDirection")
        .value("DEFAULT", ContactPatch::PatchDirection::DEFAULT)
        .value("INVERTED", ContactPatch::PatchDirection::INVERTED)
        .export_values();
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<ContactPatch>()) {
    class_<ContactPatch>(
        "ContactPatch", doxygen::class_doc<ContactPatch>(),
        init<optional<size_t>>((arg("self"), arg("preallocated_size")),
                               "ContactPatch constructor."))
        .DEF_RW_CLASS_ATTRIB(ContactPatch, tf)
        .DEF_RW_CLASS_ATTRIB(ContactPatch, direction)
        .DEF_RW_CLASS_ATTRIB(ContactPatch, penetration_depth)
        .DEF_CLASS_FUNC(ContactPatch, size)
        .DEF_CLASS_FUNC(ContactPatch, getNormal)
        .DEF_CLASS_FUNC(ContactPatch, computeBarycenter)
        .DEF_CLASS_FUNC(ContactPatch, getBarycenter)
        .DEF_CLASS_FUNC(ContactPatch, addPoint)
        .DEF_CLASS_FUNC(ContactPatch, getPoint)
        .DEF_CLASS_FUNC(ContactPatch, computeSubPatch)
        .DEF_CLASS_FUNC(ContactPatch, getSubPatchPoint)
        .DEF_CLASS_FUNC(ContactPatch, getPointShape1)
        .DEF_CLASS_FUNC(ContactPatch, getSubPatchPointShape1)
        .DEF_CLASS_FUNC(ContactPatch, getPointShape2)
        .DEF_CLASS_FUNC(ContactPatch, getSubPatchPointShape2)
        .DEF_CLASS_FUNC(ContactPatch, clear)
        .DEF_CLASS_FUNC(ContactPatch, isSame);
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          std::vector<ContactPatch>>()) {
    class_<std::vector<ContactPatch>>("StdVec_ContactPatch")
        .def(vector_indexing_suite<std::vector<ContactPatch>>());
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          ContactPatchRequest>()) {
    class_<ContactPatchRequest>(
        "ContactPatchRequest", doxygen::class_doc<ContactPatchRequest>(),
        init<optional<size_t, size_t, size_t, FCL_REAL>>(
            (arg("self"), arg("max_num_patch"), arg("max_patch_size"),
             arg("num_samples_curved_shapes"), arg("patch_tolerance")),
            "ContactPatchRequest constructor."))
        .def(dv::init<ContactPatchRequest, const CollisionRequest&,
                      bp::optional<size_t, size_t, FCL_REAL>>())
        .DEF_RW_CLASS_ATTRIB(ContactPatchRequest, max_num_patch)
        .DEF_CLASS_FUNC(ContactPatchRequest, getMaxSubPatchSize)
        .DEF_CLASS_FUNC(ContactPatchRequest, setMaxSubPatchSize)
        .DEF_CLASS_FUNC(ContactPatchRequest, getNumSamplesCurvedShapes)
        .DEF_CLASS_FUNC(ContactPatchRequest, setNumSamplesCurvedShapes)
        .DEF_CLASS_FUNC(ContactPatchRequest, getPatchTolerance)
        .DEF_CLASS_FUNC(ContactPatchRequest, setPatchTolerance);
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          std::vector<ContactPatchRequest>>()) {
    class_<std::vector<ContactPatchRequest>>("StdVec_ContactPatchRequest")
        .def(vector_indexing_suite<std::vector<ContactPatchRequest>>());
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          ContactPatchResult>()) {
    class_<ContactPatchResult>("ContactPatchResult",
                               doxygen::class_doc<ContactPatchResult>(),
                               init<>(arg("self"), "Default constructor."))
        .def(dv::init<ContactPatchResult, ContactPatchRequest>())
        .DEF_CLASS_FUNC(ContactPatchResult, numContactPatches)
        .DEF_CLASS_FUNC(ContactPatchResult, getUnusedContactPatch)
        .DEF_CLASS_FUNC2(ContactPatchResult, getContactPatch,
                         return_value_policy<copy_const_reference>())
        .DEF_CLASS_FUNC(ContactPatchResult, clear)
        .DEF_CLASS_FUNC(ContactPatchResult, set)
        .DEF_CLASS_FUNC(ContactPatchResult, check);
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          std::vector<ContactPatchResult>>()) {
    class_<std::vector<ContactPatchResult>>("StdVec_ContactPatchResult")
        .def(vector_indexing_suite<std::vector<ContactPatchResult>>());
  }

  doxygen::def(
      "computeContactPatch",
      static_cast<void (*)(const CollisionObject*, const CollisionObject*,
                           const CollisionResult&, const ContactPatchRequest&,
                           ContactPatchResult&)>(&computeContactPatch));
  doxygen::def(
      "computeContactPatch",
      static_cast<void (*)(const CollisionGeometry*, const Transform3f&,
                           const CollisionGeometry*, const Transform3f&,
                           const CollisionResult&, const ContactPatchRequest&,
                           ContactPatchResult&)>(&computeContactPatch));

  if (!eigenpy::register_symbolic_link_to_registered_type<
          ComputeContactPatch>()) {
    class_<ComputeContactPatch>("ComputeContactPatch",
                                doxygen::class_doc<ComputeContactPatch>(),
                                no_init)
        .def(dv::init<ComputeContactPatch, const CollisionGeometry*,
                      const CollisionGeometry*>())
        .def("__call__",
             static_cast<void (ComputeContactPatch::*)(
                 const Transform3f&, const Transform3f&, const CollisionResult&,
                 const ContactPatchRequest&, ContactPatchResult&) const>(
                 &ComputeContactPatch::operator()));
  }
}
