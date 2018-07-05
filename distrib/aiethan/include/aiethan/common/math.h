/*!
 * \file math.h
 * \brief common tools
 * 
 * Quadtree can be used for robot perception and road planning.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-03-27
 */

#ifndef AIETHAN_COMMON_MATH_H_
#define AIETHAN_COMMON_MATH_H_

#include <sstream>
#include <string>
#include <iomanip>
#include "aiethan/common/port.h"

/// \brief private package aiethan::common
///
/// top namespace of aiethan, tools for common
namespace aiethan {
namespace common {
  // Clamps 'value' to be in the range ['min', 'max'].
  template <typename T>
  T Clamp(const T value, const T min, const T max) {
    if (value > max) {
      return max;
    }
    if (value < min) {
      return min;
    }
    return value;
  }

  // Calculates 'base'^'exponent'.
  template <typename T>
  constexpr T PowerX(T base, int exponent) {
    return (exponent != 0) ? base * PowerX(base, exponent - 1) : T(1);
  }
  
  // Calculates 'base'^'exponent'.
  template <typename T>
  constexpr T Powerd(T base, int exponent) {
    return std::pow<T>(base, exponent);
  }

  // Calculates a^2.
  template <typename T>
  constexpr T Pow2(T a) {
    return Powerd(a, 2);
  }
  
  template <typename T>
  constexpr T RealSqrt(T a) {
    return sqrt(std::max(T(0.), a));
  }
  
  template <typename T>
  T stringToNum(const std::string& str) {
    std::stringstream streamx(str);
    T num;
    streamx>>num;
    return num;
  }
  
  template <typename T>
  std::string decToHex(const T& num) {
    std::string str;
    std::stringstream streamx;
    streamx<<std::hex<<num;
    streamx>>str;
    return str;
  }
  
  template <typename T>
  T hexToDec(const std::string& str) {
    std::stringstream streamx;
    streamx<<std::hex<<str;
    T num;
    streamx>>num;
    return num;
  }
  
  template <typename T>
  T hexarrayToDec(const char* rgb) {
    std::stringstream streamx;
    std::string temp(rgb);
    streamx<<std::hex<<temp;
    T num;
    streamx>>num;
    return num;
  }
  
  template <typename T>
  T sign(T from, T to) {
    if((to - from) > 0) {
      return T(1);
    } else if((to - from) < 0) {
      return T(-1);
    } else {
      return T(0);
    }
  }
  
  /// \brief only for static array
  template <class T, size_t N>
  int getArrayLen(T(&t)[N]) {
    return N;
  }
  
  // Converts from degrees to radians.
  constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

  // Converts form radians to degrees.
  constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

}  // namespace common
}  // namespace aiethan

#endif  // AIETHAN_COMMON_MATH_H_