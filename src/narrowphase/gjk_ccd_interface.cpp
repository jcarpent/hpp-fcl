#ifdef HPP_FCL_BUILD_LIBCCD_INTERFACE
#include <hpp/fcl/narrowphase/gjk_ccd_interface.h>

namespace hpp {
namespace fcl {

namespace details {
Vec3f d0;
Vec3f d1;

Vec3f s0;
Vec3f s1;

int hint0;
int hint1;

void eigenToccdVec(const Vec3f &vec_eigen, ccd_vec3_t *vec_ccd)
{
  ccdVec3Set(vec_ccd, vec_eigen[0], vec_eigen[1], vec_eigen[2]);
}

void ccdToEigenVec(const ccd_vec3_t &vec_ccd, Vec3f &vec_eigen)
{
  vec_eigen[0] = vec_ccd.v[0];
  vec_eigen[1] = vec_ccd.v[1];
  vec_eigen[2] = vec_ccd.v[2];
}

void support0CCD(const void *_shape, const ccd_vec3_t *dir, ccd_vec3_t *vec)
{
  MinkowskiDiff *shape = (MinkowskiDiff *)_shape;
  hint0 = shape->support_hint[0];
  ccdToEigenVec(*dir, d0);
  s0 = shape->support0(d0, false, hint0);
  shape->support_hint[0] = hint0;
  eigenToccdVec(s0, vec);
}

void support1CCD(const void *_shape, const ccd_vec3_t *dir, ccd_vec3_t *vec)
{
  MinkowskiDiff *shape = (MinkowskiDiff *)_shape;
  hint1 = shape->support_hint[1];
  ccdToEigenVec(*dir, d1);
  s1 = shape->support1(d1, false, hint1);
  shape->support_hint[1] = hint1;
  eigenToccdVec(s1, vec);
}

void firstDirCCD(const void *_shape1, const void *_shape2, ccd_vec3_t *dir)
{
  assert(_shape1 == _shape2);
  MinkowskiDiff *shape1 = (MinkowskiDiff *)_shape1;
  MinkowskiDiff *shape2 = (MinkowskiDiff *)_shape2;
  eigenToccdVec(shape1->guess, dir);
}

void GJKCCDWrapper::initialize()
{
  measure_run_time = false;
  /* timer.stop(); */
  timer_early.stop();
}

GJK::Status GJKCCDWrapper::computeGJKAverageRunTime(MinkowskiDiff &shape, const Vec3f &guess,
    const support_func_guess_t &supportHint)
{
  assert(measure_run_time);
  /* std::array<FCL_REAL, 100> times; */
  std::array<FCL_REAL, 100> times_early;
  for (size_t i=0; i < 100; i++) {
    status = evaluate(shape, guess, supportHint);
    /* times[i] = gjk_run_time.user; */
    times_early[i] = gjk_run_time_early.user;
  }
  // Sort arrays
  // And compute mean over first 95 elements
  /* std::sort(times.begin(), times.end()); */
  std::sort(times_early.begin(), times_early.end());

  /* average_gjk_run_time = 0.; */
  average_gjk_run_time_early = 0.;
  for (size_t i=0; i < 90; i++) {
    /* average_gjk_run_time += times[i]; */
    average_gjk_run_time_early += times_early[i];
  }
  /* average_gjk_run_time /= 90; */
  average_gjk_run_time_early /= 90;
  return status;
}

GJK::Status GJKCCDWrapper::evaluate(MinkowskiDiff &shape, const Vec3f &guess, const support_func_guess_t &supportHint)
{
  ccd_t ccd;
  CCD_INIT(&ccd);

  // Need a fair comparison between HPPFCL and CCD GJK.
  // CCD needs user-defined support functions hence we give the ones in HPPFCL.
  // Both of them need to: start from the same guess, perform hill-climbing and start hill-climbing on the same index.
  shape.support_hint = supportHint;
  shape.guess = guess;

  // TODO: also replace ccd.firstdir function (which should return guess instead of vec (1, 0, 0))
  // set up ccd_t struct
  ccd.first_dir = firstDirCCD;
  ccd.support1       = support0CCD; // support function for first object
  ccd.support2       = support1CCD; // support function for second object
  ccd.max_iterations = max_iterations;     // maximal number of iterations

  if (measure_run_time)
  {
    /* timer.stop(); */
    timer_early.stop();
    /* timer.stop(); */
    timer_early.start();
  }
  int intersect = ccdGJKIntersect(&shape, &shape, &ccd);
  // now intersect holds true if obj1 and obj2 intersect, false otherwise
  gjk_run_time_early = timer_early.elapsed();

  if (intersect == 1)
    return status = GJK::Inside;
  else
    return status = GJK::Valid;
  return status;
}


} // namespace details

} // namespace fcl

} // namespace hpp
#endif
