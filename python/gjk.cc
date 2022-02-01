//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2020 CNRS-LAAS
//  Author: Joseph Mirabel
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
//   * Neither the name of CNRS-LAAS. nor the names of its
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

#include "fcl.hh"

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/narrowphase/gjk.h>

#ifdef HPP_FCL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#include "doxygen_autodoc/hpp/fcl/narrowphase/gjk.h"
#endif

#ifdef HPP_FCL_BUILD_LIBCCD_INTERFACE
#include <hpp/fcl/narrowphase/gjk_ccd_interface.h>
#endif

#include "../doc/python/doxygen.hh"
#include "../doc/python/doxygen-boost.hh"

#define DEF_RW_CLASS_ATTRIB(CLASS, ATTRIB)                                     \
  def_readwrite (#ATTRIB, &CLASS::ATTRIB,                                      \
      doxygen::class_attrib_doc<CLASS>(#ATTRIB))
#define DEF_CLASS_FUNC(CLASS, ATTRIB)                                          \
  def (#ATTRIB, &CLASS::ATTRIB, doxygen::member_func_doc(&CLASS::ATTRIB))

using namespace boost::python;

using namespace hpp::fcl;
using hpp::fcl::details::MinkowskiDiff;
using hpp::fcl::details::GJK;
using hpp::fcl::details::EPA;

#ifdef HPP_FCL_BUILD_LIBCCD_INTERFACE
using hpp::fcl::details::GJKCCDWrapper;
#endif

void exposeGJK()
{
  if(!eigenpy::register_symbolic_link_to_registered_type<GJK::Status>())
  {
    enum_ <GJK::Status> ("GJKStatus")
      .value ("Valid", GJK::Valid)
      .value ("Inside", GJK::Inside)
      .value ("Failed", GJK::Failed)
      .export_values()
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type<GJK::GJKVariant>())
  {
    enum_ <GJK::GJKVariant> ("GJKVariant")
      .value ("Vanilla", GJK::Vanilla)
      .value ("HeavyBall", GJK::HeavyBall)
      .value ("Nesterov", GJK::Nesterov)
      .export_values()
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type<GJK::ConvergenceCriterion>())
  {
    enum_ <GJK::ConvergenceCriterion> ("ConvergenceCriterion")
      .value ("VDB", GJK::VDB)
      .value ("DG", GJK::DG)
      .value ("DG_RELATIVE", GJK::DG_RELATIVE)
      .value ("IDG", GJK::IDG)
      .value ("IDG_RELATIVE", GJK::IDG_RELATIVE)
      .export_values()
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type<MinkowskiDiff>())
  {
    class_ <MinkowskiDiff> ("MinkowskiDiff", doxygen::class_doc<MinkowskiDiff>(), no_init)
      .def (doxygen::visitor::init<MinkowskiDiff>())
      .def ("set", static_cast < void (MinkowskiDiff::*)(
            const ShapeBase*, const ShapeBase*)> (&MinkowskiDiff::set),
          doxygen::member_func_doc(
            static_cast < void (MinkowskiDiff::*)(
              const ShapeBase*, const ShapeBase*)> (&MinkowskiDiff::set)))
      .def ("set", static_cast < void (MinkowskiDiff::*)(
            const ShapeBase*, const ShapeBase*,
            const Transform3f& tf0, const Transform3f& tf1)> (&MinkowskiDiff::set),
          doxygen::member_func_doc(
            static_cast < void (MinkowskiDiff::*)(
              const ShapeBase*, const ShapeBase*,
              const Transform3f& tf0, const Transform3f& tf1)> (&MinkowskiDiff::set)))
      .def("support0", static_cast<Vec3f (MinkowskiDiff::*)(const Vec3f&, bool)> (&MinkowskiDiff::support0))
      .def("support1", static_cast<Vec3f (MinkowskiDiff::*)(const Vec3f&, bool)> (&MinkowskiDiff::support1))
      .DEF_RW_CLASS_ATTRIB(MinkowskiDiff, inflation)
      .DEF_RW_CLASS_ATTRIB(MinkowskiDiff, index_support0)
      .DEF_RW_CLASS_ATTRIB(MinkowskiDiff, index_support1)
      ;
  }

#ifdef HPP_FCL_BUILD_LIBCCD_INTERFACE
  if(!eigenpy::register_symbolic_link_to_registered_type<GJKCCDWrapper>())
  {
    class_ <GJKCCDWrapper> ("GJKCCDWrapper", doxygen::class_doc<GJKCCDWrapper>(), no_init)
     .def (doxygen::visitor::init<GJK, unsigned int>())
      .DEF_CLASS_FUNC(GJKCCDWrapper, evaluate)
      .DEF_CLASS_FUNC(GJKCCDWrapper, measureRunTime)
      .DEF_CLASS_FUNC(GJKCCDWrapper, getGJKRunTimeEarly)
      .DEF_CLASS_FUNC(GJKCCDWrapper, computeGJKAverageRunTime)
      .DEF_CLASS_FUNC(GJKCCDWrapper, getAverageGJKRunTimeEarly)
      ;
  }
#endif

  if(!eigenpy::register_symbolic_link_to_registered_type<GJK>())
  {
    class_ <GJK> ("GJK", doxygen::class_doc<GJK>(), no_init)
     .def (doxygen::visitor::init<GJK, unsigned int, FCL_REAL>())
      .DEF_RW_CLASS_ATTRIB (GJK, distance)
      .DEF_RW_CLASS_ATTRIB (GJK, ray)
      .DEF_RW_CLASS_ATTRIB (GJK, support_hint)
      .DEF_RW_CLASS_ATTRIB (GJK, nfree)
      .DEF_CLASS_FUNC(GJK, evaluate)
      .DEF_CLASS_FUNC(GJK, computeGJKAverageRunTime)
      .DEF_CLASS_FUNC(GJK, hasClosestPoints)
      .DEF_CLASS_FUNC(GJK, hasPenetrationInformation)
      .DEF_CLASS_FUNC(GJK, getClosestPoints)
      .DEF_CLASS_FUNC(GJK, setDistanceEarlyBreak)
      .DEF_CLASS_FUNC(GJK, getGuessFromSimplex)
      // Momentum related
      .DEF_CLASS_FUNC(GJK, setGJKVariant)
      .DEF_CLASS_FUNC(GJK, setNormalizeSupportDirection)
      .DEF_CLASS_FUNC(GJK, setConvergenceCriterion)
      // Metrics to measure GJK performances
      .DEF_CLASS_FUNC(GJK, measureRunTime)
      .DEF_CLASS_FUNC(GJK, getIterations)
      .DEF_CLASS_FUNC(GJK, getIterationsEarly)
      .DEF_CLASS_FUNC(GJK, getNumCallSupport)
      .DEF_CLASS_FUNC(GJK, getNumCallSupportEarly)
      .DEF_CLASS_FUNC(GJK, getNumCallProjection)
      .DEF_CLASS_FUNC(GJK, getNumCallProjectionEarly)
      .DEF_CLASS_FUNC(GJK, getCumulativeSupportDotprods)
      .DEF_CLASS_FUNC(GJK, getCumulativeSupportDotprodsEarly)
      .DEF_CLASS_FUNC(GJK, getGJKRunTime)
      .DEF_CLASS_FUNC(GJK, getGJKRunTimeEarly)
      .DEF_CLASS_FUNC(GJK, getAverageGJKRunTime)
      .DEF_CLASS_FUNC(GJK, getAverageGJKRunTimeEarly)
      .def("projectLineOrigin", static_cast<bool (GJK::*)(const GJK::Simplex&, GJK::Simplex&)> (&GJK::projectLineOrigin))
      .def("projectTriangleOrigin", static_cast<bool (GJK::*)(const GJK::Simplex&, GJK::Simplex&)> (&GJK::projectTriangleOrigin))
      .def("projectTetrahedraOrigin", static_cast<bool (GJK::*)(const GJK::Simplex&, GJK::Simplex&)> (&GJK::projectTetrahedraOrigin))
      ;

  if(!eigenpy::register_symbolic_link_to_registered_type<GJK::Simplex>())
  {
    class_ <GJK::Simplex> ("Simplex", doxygen::class_doc<GJK::Simplex>(), no_init)
     .def (doxygen::visitor::init<GJK::Simplex>())
      .DEF_RW_CLASS_ATTRIB (GJK::Simplex, rank)
      .DEF_RW_CLASS_ATTRIB (GJK::Simplex, cp0)
      .DEF_RW_CLASS_ATTRIB (GJK::Simplex, cp1)
      .DEF_CLASS_FUNC(GJK::Simplex, getVertex)
      .DEF_CLASS_FUNC(GJK::Simplex, setVertex)
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type<GJK::SimplexV>())
  {
    class_ <GJK::SimplexV> ("SimplexV", doxygen::class_doc<GJK::SimplexV>(), no_init)
     .def (doxygen::visitor::init<GJK::SimplexV>())
      .DEF_RW_CLASS_ATTRIB (GJK::SimplexV, w0)
      .DEF_RW_CLASS_ATTRIB (GJK::SimplexV, w1)
      .DEF_RW_CLASS_ATTRIB (GJK::SimplexV, w)
      .DEF_RW_CLASS_ATTRIB (GJK::SimplexV, index_w0)
      .DEF_RW_CLASS_ATTRIB (GJK::SimplexV, index_w1)
      ;
  }
  }
}
