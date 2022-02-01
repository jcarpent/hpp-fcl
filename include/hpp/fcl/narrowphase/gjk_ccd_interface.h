// TODO: Add licensing (the one from libCCD)
#ifndef HPP_FCL_CCD_INTERFACE_H
#define HPP_FCL_CCD_INTERFACE_H
#ifdef HPP_FCL_BUILD_LIBCCD_INTERFACE

#include <hpp/fcl/narrowphase/gjk.h>
#include <ccd/ccd.h>
#include <ccd/vec3.h>

namespace hpp {
namespace fcl {

namespace details {

/// @brief converts a Vec3f to ccd_vec3_t
// \note: FCL_REAL and ccd_real_t are typedefs for double
void eigenToccdVec(const Vec3f &vec_eigen, ccd_vec3_t *vec_ccd);

/// @brief converts a ccd_vec3_t to Vec3f
// \ref eigenToccdVec
void ccdToEigenVec(const ccd_vec3_t &vec_ccd, Vec3f &vec_eigen);

// @brief support function for shape0 used by CCD ccd_t struct.
// Since CCD requires to give user defined support functions, we give it HPPFCL's.
// *shape is a pointer to an instance of MinkowskiDiff object
void support0CCD(const void *shape, const ccd_vec3_t *dir, ccd_vec3_t *vec);

// @brief support function for shape1 used by CCD ccd_t struct.
// See \ref support1
void support1CCD(const void *shape, const ccd_vec3_t *dir, ccd_vec3_t *vec);

// @brief fills dir with direction which will be used by CCD for first support call.
// Equivalent in HPPFCL of guess we give to \ref GJK::evaluate
void firstDirCCD(const void *_shape1, const void *_shape2, ccd_vec3_t *dir);


struct HPP_FCL_DLLAPI GJKCCDWrapper{

  GJKCCDWrapper(unsigned int max_iterations_): max_iterations(max_iterations_){
    initialize();
  }

  void initialize();

  // @brief CCD implements the early stopping version of GJK.
  // We cannot use it solve the distance computation problem unless we use GJK + EPA.
  // For now it is outside our scope as we wish to study GJK alone.
  // All the information regarding the pair of objects and their relative pose
  // is contained in the shape instance of the MinkowskiDiff class.
  GJK::Status evaluate(MinkowskiDiff &shape, const Vec3f &guess, const support_func_guess_t &supportHint);

  GJK::Status computeGJKAverageRunTime(MinkowskiDiff &shape, const Vec3f &guess,
    const support_func_guess_t &supportHint);

  void measureRunTime() { measure_run_time = true; }
  inline FCL_REAL getGJKRunTimeEarly() { return gjk_run_time_early.user; }
  inline FCL_REAL getAverageGJKRunTimeEarly() { return average_gjk_run_time_early; }

private:
  bool measure_run_time;
  /* Timer timer; */
  Timer timer_early;

  unsigned int max_iterations;
  /* CPUTimes gjk_run_time; */
  CPUTimes gjk_run_time_early;
  /* FCL_REAL average_gjk_run_time; */
  FCL_REAL average_gjk_run_time_early;
  GJK::Status status;
};

} // namespace details

} // namespace fcl

} // namespace hpp

#endif
#endif
