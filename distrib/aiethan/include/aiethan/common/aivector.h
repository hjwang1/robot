/*!
 * \file aivector.h
 * \brief common tools
 * 
 * Vector can be used for robot perception and road planning.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-08-01
 */

#ifndef AIETHAN_COMMON_AIVECTOR_H_
#define AIETHAN_COMMON_AIVECTOR_H_

#include "aiethan/common/port.h"

/// \brief private package aiethan::common
///
/// top namespace of aiethan, tools for common
namespace aiethan {
namespace common {
  class vector {
  public:
    static constexpr int kCAPACITY = 50000;   
    
    explicit vector();
    ~vector();
    
    vector(const vector&) = delete;
    vector& operator=(const vector&) = delete;
    
    /// \brief get pre-frame
    ///
    /// const return.
    /// \param pre-frame
    /// \return the number of matching.
    int size();
    void push_back(uint64_t);
    void clear();
    uint64_t& operator[](int);
    int capacity();
    uint64_t* arrayHead();
    
  private:
    int m_size;
    uint64_t m_array[kCAPACITY];
  };
  
}  // namespace common
}  // namespace aiethan

#endif  // AIETHAN_COMMON_AIVECTOR_H_