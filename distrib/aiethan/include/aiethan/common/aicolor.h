/*!
 * \file aicolor.h
 * \brief common tools
 * 
 * Vector can be used for robot perception and road planning.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-08-30
 */

#ifndef AIETHAN_COMMON_AICOLOR_H_
#define AIETHAN_COMMON_AICOLOR_H_

#include "aiethan/common/port.h"

/// \brief private package aiethan::common
///
/// top namespace of aiethan, tools for common
namespace aiethan {
namespace common {
  
struct BGR
{
    uint8 b;
    uint8 g;
    uint8 r;
};

struct HSV
{
    int h;
    double s;
    double v;
};

bool IsEquals(double val1 , double val2);

// BGR(BGR: 0~255)转HSV(H: [0~360), S: [0~1], V: [0~1])
void BGR2HSV(BGR &bgr, HSV &hsv);

// HSV转BGR
void HSV2BGR(HSV &hsv, BGR &bgr);

  
}  // namespace common
}  // namespace aiethan

#endif  // AIETHAN_COMMON_AICOLOR_H_