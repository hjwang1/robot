/*!
 * \file diffgeom.h
 * \brief common tools
 * 
 * diff geom tools.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2018-04-21
 */

#ifndef AIETHAN_COMMON_DIFFGEOM_H_
#define AIETHAN_COMMON_DIFFGEOM_H_

/// \brief private package aiethan::common
///
/// top namespace of aiethan, tools for common
namespace aiethan {
namespace common {
  static constexpr double AI_PI = 3.14159265358979323846;
  static constexpr double diff_epsilon = 0.0000000000000005;

  int cal_curvature(double x1, double y1, double z1,
		    double x2, double y2, double z2,
		    double x3, double y3, double z3,
		    double& kg1, double& kg2, double& kg3,
		    double& am1, double& am2, double& am3
 		  );
  int cal_curvature(double x1, double y1, double z1,
		    double x2, double y2, double z2,
		    double x3, double y3, double z3,
		    double& kg1, double& kg2, double& kg3,
		    double& am1, double& am2, double& am3,
		    double nx1, double ny1, double nz1,
		    double nx2, double ny2, double nz2,
		    double nx3, double ny3, double nz3,
		    double& kh1, double& kh2, double& kh3
 		  );
}  // namespace common
}  // namespace aiethan

#endif  // AIETHAN_COMMON_DIFFGEOM_H_