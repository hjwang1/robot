/*
 * Copyright 2017 The Aiethan Authors Hongjun Wang.
 *
 * You may reference to
 *
 *      http://www.aiethan.com/
 *
 * Good luck.
 */

#ifndef AIETHAN_PLANE_QUAD_UTIL_H_
#define AIETHAN_PLANE_QUAD_UTIL_H_

#include <cinttypes>
#include <limits>
#include <array>
#include <list>
#include <eigen3/Eigen/Core>

#include "aiethan/common/math.h"

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

using Point2D = Eigen::Vector2d;

namespace aiethan {
namespace plain {
namespace QuadUtil {
  inline int getDualQuad(int quad) {
    if(quad < 0 || quad > 3) {
      return -1;
    }
    return (quad + 2) % 4;
  }
  
  inline int getSignX(int quad) {
    if(quad == 0 || quad == 3) {
      return 1;
    } else if(quad == 1 || quad == 2) {
      return -1;
    } else {
      return 0;
    }
  }
  
  inline int getSignY(int quad) {
    if(quad == 0 || quad == 1) {
      return 1;
    } else if(quad == 2 || quad == 3) {
      return -1;
    } else {
      return 0;
    }
  }
  
  inline int getQuad(double x, double y) {
    if(y <= 0) {
      if(x > 0) {
	return 3;
      } else {
	return 2;
      }
    } else {
      if(x > 0) {
	return 0;
      } else {
	return 1;
      }
    }
  }
  
} // namespace QuadUtil

    
namespace LinePoint {
  
  inline std::list<Point2D> middlePoints(double fromx, double fromy, double tox, double toy) {
    std::list<Point2D> ret;
    
    double x = fromx;
    double y = fromy;
    int signx = aiethan::common::sign<double>(fromx, tox);
    int signy = aiethan::common::sign<double>(fromy, toy);
    
    double tx = tox - fromx;
    double ty = toy - fromy;
    double lx = fromx * ty;
    double ly = fromy * tx;
    double ix = (fromx + 0.5 * signx) * ty;
    double iy = (fromy + 0.5 * signy) * tx;
    
    double dx,dy;
    
    while(!(x == tox && y == toy)) {
      if(aiethan::common::Abs(x - fromx) > aiethan::common::Abs(tox - fromx)
	&& aiethan::common::Abs(y - fromy) > aiethan::common::Abs(toy - fromy)) {
	//some input error;
	return ret;
      }
      if(!(x == fromx && y == fromy)) {
	ret.push_back(Point2D(x, y));
      }
      dx = ix - lx;
      dy = iy - ly;
      
      if(aiethan::common::Abs(dx) < aiethan::common::Abs(dy)) {
	x = x + signx;
	lx = ix;
	ly = lx - fromx * ty + fromy * tx;
	ix = ix + signx * ty;
      } else if(aiethan::common::Abs(dx) > aiethan::common::Abs(dy)) {
	y = y + signy;
	ly = iy;
	lx = ly - fromy * tx + fromx * ty;
	iy = iy + signy * tx;
      } else {
	x = x + signx;
	y = y + signy;
	lx = ix;
	ly = iy;
	ix = ix + signx * ty;
	iy = iy + signy * tx;
      }
    }
    
    return ret;
  }
  
} // namespace LinePoint

namespace Descartes {
  
  constexpr double kS = std::sqrt(2.0);
  
  inline double getDistance(double px, double py, double pax, double pay, double pbx, double pby) {
    return aiethan::common::Abs((px-pax)*(pby-pay)-(py-pay)*(pbx-pax)) / 
	    aiethan::common::RealSqrt<double>((pbx-pax)*(pbx-pax) + (pby-pay)*(pby-pay));
  }
  
  inline double getRelation(double px, double py, double pax, double pay, double pbx, double pby) {
    double factor = 0;
    double dis = getDistance(px, py, pax, pay, pbx, pby);
    if((pbx-pax) == 0 || (pby-pay) == 0) {
      if(dis > 0.5) {
	factor = 0;
      } else {
	factor = (0.5 - dis) * 2;
      }
    } else if((pbx-pax) == (pby-pay) || (pbx-pax) == (pay-pby)) {
      if(dis > kS/2) {
	factor = 0;
      } else {
	factor = (kS/2 - dis) * (kS/2 - dis) * 2;
      }
    } else {
      double tan = aiethan::common::Abs((pby-pay) / (pbx-pax));
      if(tan > 1) {
	tan = 1.0 / tan;
      }
      double h = aiethan::common::RealSqrt<double>(1 + tan * tan);
      double maxdis = (1 + tan) / (2 * h);
      if(dis > maxdis) {
	factor = 0;
      } else {
	if((maxdis - dis) < (tan / h)) {
	  factor = (maxdis - dis) * (maxdis - dis) * (1 + tan * tan) / tan;
	} else {
	  factor = (maxdis - dis) * h * 2 - tan;
	}
      }
    }
      
    return factor;
  }
  
  inline uint8 getQuadrant(int x, int y) {
    if(x > 0 && y >=0) {
      if(x > y) {
	return 1;
      } else {
	return 2;
      }
    } else if(x <= 0 && y > 0) {
      if(-x < y) {
	return 3;
      } else {
	return 4;
      }
    } else if(x < 0 && y <= 0) {
      if(-x > -y) {
	return 5;
      } else {
	return 6;
      }
    } else if(x >= 0 && y < 0) {
      if(x < -y) {
	return 7;
      } else {
	return 8;
      }
    } else {
      return 0;
    }
  }

  
} // namespace Descartes

namespace MV {
  
  constexpr double S = std::sqrt(2.0);
  constexpr double T = -S;
  constexpr double U = -1;
  constexpr double MATRIX[2][16] = {
    {1,U,S,0,1,1,0,S,U,1,T,0,U,U,0,T},
    {0,S,U,1,T,0,U,U,0,T,1,U,S,0,1,1}
  };
  
  constexpr double E = std::sqrt(2.0) / 2;
  constexpr double F = -E;
  constexpr double H_MATRIX[2][16] = {
    {1,E,E,0,0,F,F,U,U,F,F,0,0,E,E,1},
    {0,E,E,1,1,E,E,0,0,F,F,U,U,F,F,0}
  };
  
  constexpr double MAX = 1000000;
  constexpr double WALSH_HADAMA[8][8] = {
    {1,1,1,1,1,1,1,1},
    {1,U,1,U,1,U,1,U},
    {1,1,U,U,1,1,U,U},
    {1,U,U,1,1,U,U,1},
    {1,1,1,1,U,U,U,U},
    {1,U,1,U,U,1,U,1},
    {1,1,U,U,U,U,1,1},
    {1,U,U,1,U,1,1,U}
  };
  
  inline double executeX(uint8 index, double i, double j) {
    if(index < 1 || index > 8) {
      return 0;
    }
    return MATRIX[0][index * 2 - 2] * i + MATRIX[0][index * 2 - 1] * j;
  }
  
  inline double executeY(uint8 index, double i, double j) {
    if(index < 1 || index > 8) {
      return 0;
    }
    return MATRIX[1][index * 2 - 2] * i + MATRIX[1][index * 2 - 1] * j;
  }
  
  inline double executeEigen(uint8 index, std::array<double, 8> f) {
    if(index < 1 || index > 8 || f.size() != 8) {
      return MAX;
    }
    
    return (
      WALSH_HADAMA[index - 1][0] * f[0] +
      WALSH_HADAMA[index - 1][1] * f[1] +
      WALSH_HADAMA[index - 1][2] * f[2] +
      WALSH_HADAMA[index - 1][3] * f[3] +
      WALSH_HADAMA[index - 1][4] * f[4] +
      WALSH_HADAMA[index - 1][5] * f[5] +
      WALSH_HADAMA[index - 1][6] * f[6] +
      WALSH_HADAMA[index - 1][7] * f[7]
      ) / 8.0;
  }
  
  inline double executeSpectrum(uint8 index, std::array<double, 8> t) {
    if(index < 1 || index > 4 || t.size() != 8) {
      return MAX;
    }
    switch (index) {
      case 1:
	return t[0] * t[0];
      case 2:
	return t[1] * t[1];
      case 3:
	return t[2] * t[2] + t[3] * t[3];
      case 4:
	return t[4] * t[4] + t[5] * t[5] + t[6] * t[6] + t[7] * t[7];
      default:
	return MAX;
    }	
  }
  
} // namespace MV

}  // namespace plain
}  // namespace aiethan

#endif  // AIETHAN_PLANE_QUAD_UTIL_H_