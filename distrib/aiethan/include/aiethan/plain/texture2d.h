/*
 * Copyright 2017 The Aiethan Authors Hongjun Wang.
 *
 * You may reference to
 *
 *      http://www.aiethan.com/
 *
 * Good luck.
 */

#ifndef AIETHAN_PLANE_TEXTURE2D_H_
#define AIETHAN_PLANE_TEXTURE2D_H_

#include "aiethan/plain/quad_util.h"

namespace aiethan {
namespace plain {
  
  constexpr uint8 kD = 8;
  constexpr uint8 kDS = 4;
  
  class Texture2d {
  public:
    explicit Texture2d();
    explicit Texture2d(uint8 grid);    
    ~Texture2d();
    
    Texture2d(const Texture2d&) = delete;
    Texture2d& operator=(const Texture2d&) = delete;
    
    std::array<double, kD> getField(void);
    std::array<double, kD> getEigen(void);
    void init(void);
    void reset(void);
    void addField(uint8 index, double value);
    void addPotential(double potential);
    void setEigen(uint8 index, double value);
    void setSpectrum(uint8 index, double value);
    
    double getConformal(void);
    
    uint8 getGrid(void);
    uint8 getFlag(void);
    void setFlag(uint8 flag);
    double getPotential(void);
    
  private:
    std::array<double, kD> field_;
    std::array<double, kD> eigen_;
    std::array<double, kDS> spectrum_;
    uint8 grid_;
    uint8 flag_;
    double potential_;
  };
  
}  // namespace plain
}  // namespace aiethan

#endif  // AIETHAN_PLANE_TEXTURE2D_H_