/*!
 * \file permutation.h
 * \brief common tools
 * 
 * permutation can be used for robot perception and road planning.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-07-24
 */

#ifndef AIETHAN_COMMON_PERMUTATION_H_
#define AIETHAN_COMMON_PERMUTATION_H_

/// \brief private package aiethan::common
///
/// top namespace of aiethan, tools for common
namespace aiethan {
namespace common {
  class Permutation {
  public:
    static constexpr int kLEN = 32;
    static int kPERM[kLEN];
    static int kINIT[kLEN];
    
    explicit Permutation() {}
    ~Permutation() {}
    
    Permutation(const Permutation&) = delete;
    Permutation& operator=(const Permutation&) = delete;
    
    /// \brief get a perm
    ///
    /// make sure that count=A(n,m) of permutation and n>=m>0 && index between[0,count).
    /// \param count 
    /// \param n
    /// \param m
    /// \param index
    /// \return the number of matching.
    static int getPerm(int count, int n, int m, int index);
    
  };
  
  
  
}  // namespace common
}  // namespace aiethan

#endif  // AIETHAN_COMMON_PERMUTATION_H_