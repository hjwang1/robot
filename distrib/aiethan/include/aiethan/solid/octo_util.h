/*!
 * \file octo_util.h
 * \brief 3D vision for octo util
 * 
 * Octo Tool.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-03-31
 */

#ifndef AIETHAN_SOLID_OCTO_UTIL_H_
#define AIETHAN_SOLID_OCTO_UTIL_H_

#include <cinttypes>
#include <limits>
#include <vector>
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

using Point3D = Eigen::Vector3d;

namespace aiethan {
namespace solid {
namespace OctoUtil {
  constexpr int QUADS[3][3][3] = {
    {{2,2,4},{2,2,4},{1,1,7}},
    {{2,2,4},{2,2,4},{1,1,7}},
    {{3,3,5},{3,3,5},{0,0,6}}
  };
  constexpr int DUAL_QUAD[8] = {4,5,6,7,0,1,2,3};
  constexpr int SIGN_X_QUAD[8] = {1,-1,-1,1,-1,1,1,-1};
  constexpr int SIGN_Y_QUAD[8] = {1,1,-1,-1,-1,-1,1,1};
  constexpr int SIGN_Z_QUAD[8] = {-1,-1,-1,-1,1,1,1,1};
  
  inline int getDualQuad(int quad) {
    if(quad < 0 || quad > 7) {
      return -1;
    }
    //return (quad + 4) % 8;
    return DUAL_QUAD[quad];
  }
  
  inline int getSignX(int quad) {
    if(quad < 0 || quad > 7) {
      return 0;
    }
    /*
    if(quad == 0 || quad == 3 || quad == 5 || quad == 6) {
      return 1;
    } else {
      return -1;
    }*/
    return SIGN_X_QUAD[quad];
  }
  
  inline int getSignY(int quad) {
    if(quad < 0 || quad > 7) {
      return 0;
    }
    /*
    if(quad == 0 || quad == 1 || quad == 7 || quad == 6) {
      return 1;
    } else {
      return -1;
    }*/
    return SIGN_Y_QUAD[quad];
  }
  
  inline int getSignZ(int quad) {
    if(quad < 0 || quad > 7) {
      return 0;
    }
    /*
    if(quad == 4 || quad == 5 || quad == 7 || quad == 6) {
      return 1;
    } else {
      return -1;
    }*/
    return SIGN_Z_QUAD[quad];
  }
  
  inline int getQuad(double x, double y, double z) {
    /*
    if(y <= 0) {
      if(x > 0) {
	return (z <= 0 ? 3 : 5);
      } else {
	return (z <= 0 ? 2 : 4);
      }
    } else {
      if(x > 0) {
	return (z <= 0 ? 0 : 6);
      } else {
	return (z <= 0 ? 1 : 7);
      }
    }*/
    return QUADS[(x > 0 ? 1 : -1)+1][(y > 0 ? 1 : -1)+1][(z > 0 ? 1 : -1)+1];
  }
  
} // namespace OctoUtil

namespace Manifold3D {
  
  inline int sign(double from, double to) {
    if((to - from) > 0) {
      return 1;
    } else if((to - from) < 0) {
      return -1;
    } else {
      return 0;
    }
  }
  
}

namespace LinePoint3D {
  
  inline std::list<Point3D> middlePoints(Point3D from, Point3D to) {
    std::list<Point3D> ret;
    
    double c[] = {from.x(), from.y(), from.z()};
    int sign[] = {Manifold3D::sign(from.x(), to.x()),
		  Manifold3D::sign(from.y(), to.y()),
		  Manifold3D::sign(from.z(), to.z())
    };
    double t[] = {to.x() - from.x(),
		  to.y() - from.y(),
		  to.z() - from.z()
    };
    double l[] = {from.x(), from.y(), from.z()};
    double i[] = {from.x() + 0.5 * sign[0],
		  from.y() + 0.5 * sign[1],
		  from.z() + 0.5 * sign[2]
    };
    for(int index = 0; index < 3; index++) {
      if(t[index] != 0) {
	l[(index + 1) % 3] *= t[index];
	l[(index + 2) % 3] *= t[index];
	i[(index + 1) % 3] *= t[index];
	i[(index + 2) % 3] *= t[index];
      }
    }
    double d[] = {i[0] - l[0],
		  i[1] - l[1],
		  i[2] - l[2]
    };
    
    std::list<int> mins;
    std::list<int> others;
    do {
      mins.clear();
      others.clear();
      for(int index = 0; index < 3; index++) {
	if(d[index] != 0) {
	  if(mins.empty()) {
	    mins.push_back(index);
	  } else {
	    if(aiethan::common::Abs(d[index]) > aiethan::common::Abs(d[mins.front()])) {
	      others.push_back(index);
	    } else if(aiethan::common::Abs(d[index]) == aiethan::common::Abs(d[mins.front()])) {
	      mins.push_back(index);
	    } else {
	      others.splice(others.end(), mins);//mins is cleared
	      mins.push_back(index);
	    }
	  }
	}
      }
      if(mins.empty()) {
	break;
      }
      for(auto p : others) {
	l[p] += (i[mins.front()] - l[mins.front()]);
      }
      for(auto p : mins) {
	c[p] += sign[p];
	l[p] = i[p];
	i[p] += sign[p] * (t[(p+1)%3]==0 ? 1 : t[(p+1)%3]) * (t[(p+2)%3]==0 ? 1 : t[(p+2)%3]);
      }
      if(c[0] == to.x() && c[1] == to.y() && c[2] == to.z()) {
	break;
      } else {
	ret.push_back(Point3D(c[0], c[1], c[2]));
      }
      for(int index = 0; index < 3; index++) {
	d[index] = i[index] - l[index];
      }
    } while(!mins.empty());
    
    return ret;
  }
  
} // namespace LinePoint3D

namespace Descartes3D {
  
  inline uint8 getInnerDist(double x, double y) {
    if(x < 0 || y <=0) {
      return 0;
    }
    
    //x >=0, y > 0
    if(x < 0.26795*y) {
      return 1;
    } else if(x < y) {
      return 2;
    } else if(x < 3.73205*y) {
      return 3;
    } else {
      return 4;
    }
  }
  
  inline uint8 getDistribute(double x, double y) {
    if(x < 0) {
      return 0;
    }
    
    if(y == 0) {
      return 4;
    } else if(y > 0) {
      return getInnerDist(x, y);
    } else {
      return (8 - getInnerDist(x, -y));
    }
  }
  
  inline uint8 getQuadrant(double x, double y) {
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

  
} // namespace Descartes3D

namespace MV3D {
  
  constexpr double SIN_75 = 0.96593;
  constexpr double S_N_75 = -SIN_75;
  constexpr double COS_15 = SIN_75;
  constexpr double SIN_45 = 0.70711;
  constexpr double S_N_45 = -SIN_45;
  constexpr double COS_45 = SIN_45;
  constexpr double SIN_15 = 0.25882;
  constexpr double S_N_15 = -SIN_15;
  constexpr double COS_75 = SIN_15;
  
  constexpr double MATRIX[2][14] = {
    {-3.73205,1,-1.41421, 1.41421,-0.51764, 1.93185,0.51764, 1.93185, 1.41421, 1.41421, 1.93185, 0.51764, 3.86370, 0},
    { 3.86370,0, 1.93185,-0.51764, 1.41421,-1.41421,0.51764,-1.93185,-0.51764,-1.93185,-1.41421,-1.41421,-3.73205,-1}
  };
  constexpr double H_MATRIX[2][14] = {
    {0,COS_75,COS_75,COS_45,COS_45,COS_15,COS_15,COS_15,COS_15,COS_45,COS_45,COS_75,COS_75, 0},
    {1,SIN_75,SIN_75,SIN_45,SIN_45,SIN_15,SIN_15,S_N_15,S_N_15,S_N_45,S_N_45,S_N_75,S_N_75,-1}
  };
  constexpr double U = -1;
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
  
  inline double executeUp(uint8 index, double i, double j) {
    if(index < 1 || index > 7) {
      return 0;
    }
    return MATRIX[0][index * 2 - 2] * i + MATRIX[0][index * 2 - 1] * j;
  }
  
  inline double executeDown(uint8 index, double i, double j) {
    if(index < 1 || index > 7) {
      return 0;
    }
    return MATRIX[1][index * 2 - 2] * i + MATRIX[1][index * 2 - 1] * j;
  }
  
} // namespace MV3D

}  // namespace solid
}  // namespace aiethan

#endif  // AIETHAN_SOLID_OCTO_UTIL_H_