// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef AP_MATH_H
#define AP_MATH_H

// Assorted useful math operations for ArduPilot(Mega)

#include <AP_Common.h>
#include <AP_Param.h>
#include <math.h>
#ifdef __AVR__
# include <AP_Math_AVR_Compat.h>
#endif
#include <stdint.h>
#include "rotations.h"
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"
#include "quaternion.h"
#include "polygon.h"

#ifndef PI
#define PI 3.141592653589793f
#endif
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define RadiansToCentiDegrees(x) ((x) * 5729.578f)

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS 9.80665f

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100

#define ROTATION_COMBINATION_SUPPORT 0

// convert a longitude or latitude point to meters or centimeteres.
// Note: this does not include the longitude scaling which is dependent upon location
#define LATLON_TO_M  0.01113195f
#define LATLON_TO_CM 1.113195f

// define AP_Param types AP_Vector3f and Ap_Matrix3f
//AP_PARAMDEFV(Matrix3f, Matrix3f, AP_PARAM_MATRIX3F);
//AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);
typedef AP_ParamV<Matrix3f, AP_PARAM_MATRIX3F> AP_Matrix3f;
typedef AP_ParamV<Vector3f, AP_PARAM_VECTOR3F> AP_Vector3f;

// a varient of asin() that always gives a valid answer.
float           safe_asin(float v);

// a varient of sqrt() that always gives a valid answer.
float           safe_sqrt(float v);

#if ROTATION_COMBINATION_SUPPORT
// find a rotation that is the combination of two other
// rotations. This is used to allow us to add an overall board
// rotation to an existing rotation of a sensor such as the compass
enum Rotation           rotation_combination(enum Rotation r1, enum Rotation r2, bool *found = NULL);
#endif

// longitude_scale - returns the scaler to compensate for shrinking longitude as you move north or south from the equator
// Note: this does not include the scaling to convert longitude/latitude points to meters or centimeters
float                   longitude_scale(const struct Location *loc);

// return distance in meters between two locations
float                   get_distance(const struct Location *loc1, const struct Location *loc2);

// return distance in centimeters between two locations
uint32_t                get_distance_cm(const struct Location *loc1, const struct Location *loc2);

// return bearing in centi-degrees between two locations
int32_t                 get_bearing_cd(const struct Location *loc1, const struct Location *loc2);

// see if location is past a line perpendicular to
// the line between point1 and point2. If point1 is
// our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool        location_passed_point(const struct Location & location,
                                  const struct Location & point1,
                                  const struct Location & point2);

//  extrapolate latitude/longitude given bearing and distance
void        location_update(struct Location *loc, float bearing, float distance);

// extrapolate latitude/longitude given distances north and east
void        location_offset(struct Location *loc, float ofs_north, float ofs_east);

/**
 * Constrains a Value.
 * If amt isn't in the range [low, high], it's set to either low or high.
 *
 * correct usage:
 * If amt, low, high have different types, the type that can hold the largest
 * range has to be chosen for T.
 * Otherwise a value might wrap around and lead to unexpected results.
 *
 * Wrong Usage example:
 * 	int32_t tmp = 65437;
 * 	int16_t i = constrain_int16(tmp, -1000, 1000);
 *
 * 	One could expect that i is set to 1000, since tmp > 1000.
 * 	Instead the conversion from temp to an int16_t (that cant't hold 65437)
 * 	sets the value to -99.
 * 	Since -99 is in the range [-1000, 1000] that's the value i is set to.
 *
 * @param amt value to be constrained
 * @param low lower bound
 * @param high upper bound
 * @return constrained value
 */
template<typename T>
T constrain__(T amt, T low, T high) {
	return amt < low ? low : ( amt > high ? high : amt);
}

// constrain a value
inline float constrain_float(float amt, float low, float high) {
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return constrain__<float>(amt, low, high);
}
#define constrain_int8  constrain__<int8_t>
#define constrain_int16 constrain__<int16_t>
#define constrain_int32 constrain__<int32_t>
#define constrain_int   constrain__<int>

/*
  wrap an angle in centi-degrees
 */
int32_t wrap_360_cd(int32_t error);
int32_t wrap_180_cd(int32_t error);

// degrees -> radians
float radians(float deg);

// radians -> degrees
float degrees(float rad);

// square
float sq(float v);

// sqrt of sum of squares
float pythagorous2(float a, float b);
float pythagorous3(float a, float b, float c);

#ifdef radians
#error "Build is including Arduino base headers"
#endif

/* The following three functions used to be arduino core macros */
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))


#endif // AP_MATH_H

