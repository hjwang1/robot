/*!
 * \file port.h
 * \brief common tools
 * 
 * Quadtree can be used for robot perception and road planning.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-03-27
 */

#ifndef AIETHAN_COMMON_PORT_H_
#define AIETHAN_COMMON_PORT_H_

#include <cinttypes>
#include <cmath>

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

/// \brief private package aiethan::common
///
/// top namespace of aiethan, tools for common
namespace aiethan {
namespace common {

  inline int RoundToInt(const float x) { return std::lround(x); }

  inline int RoundToInt(const double x) { return std::lround(x); }

  inline int64 RoundToInt64(const float x) { return std::lround(x); }

  inline int64 RoundToInt64(const double x) { return std::lround(x); }
  
  inline double Abs(const double x) { return std::abs(x); }
  
  constexpr int kPOWER[] = {1,2,4,8,16,32,64,128,256,512,
    1024,2048,4096,8192,16384,32768,65536
  };
  inline double Powerdouble(double base, int exponent) {
    if(base == 2) {
      if(exponent <= 16 && exponent >= 0) {
	return kPOWER[exponent];
      } else if(exponent < 0 && exponent >= -16) {
	return 1.0/kPOWER[-exponent];
      }
    }
    return std::pow<double>(base, exponent);
  }  


}  // namespace common
}  // namespace aiethan

#endif  // AIETHAN_COMMON_PORT_H_
