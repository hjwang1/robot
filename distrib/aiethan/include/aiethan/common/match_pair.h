/*!
 * \file match_pair.h
 * \brief common tools
 * 
 * MatchPair can be used for robot perception and road planning.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-07-24
 */

#ifndef AIETHAN_COMMON_MATCHPAIR_H_
#define AIETHAN_COMMON_MATCHPAIR_H_

/// \brief private package aiethan::common
///
/// top namespace of aiethan, tools for common
namespace aiethan {
namespace common {
  struct MatchPair {
    int base_;
    int refer_;
    
    MatchPair(int base, int refer):base_(base), refer_(refer) {
    }
   
  };
  
  struct Error {
    double angle_;
    double dis_;
    
    Error(double angle, double distance):angle_(angle), dis_(distance) {
    }
   
  };
  
}  // namespace common
}  // namespace aiethan

#endif  // AIETHAN_COMMON_MATCHPAIR_H_