/*!
 * \file textrue3d.h
 * \brief 3D vision for occupied grid
 * 
 * Information about grid.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-03-31
 */

#ifndef AIETHAN_SOLID_TEXTURE3D_H_
#define AIETHAN_SOLID_TEXTURE3D_H_

#include "aiethan/solid/octo_util.h"

/// \brief private package aiethan::solid
///
/// top namespace of aiethan, then 2D for plain and 3D for solid
namespace aiethan {
namespace solid {
  constexpr uint8 kD = 8;
  constexpr uint8 kDS = 4;
  
  class Texture3d {
  public:
    explicit Texture3d();
    explicit Texture3d(uint8 grid);    
    ~Texture3d();
    
    Texture3d(const Texture3d&) = delete;
    Texture3d& operator=(const Texture3d&) = delete;
    
    std::vector<std::vector<double>> getField(void);
    std::vector<std::vector<double>> getEigen(void);
    void init(void);
    void reset(void);
    void addField(int px, int py, double value);
    void addPotential(double potential);
    void setEigen(void);
    void setSpectrum(void);
    
    double getConformal(void);
    std::vector<double> getConformalRaw(void);
    
    uint8 getGrid(void);
    uint8 getFlag(void);
    void setFlag(uint8 flag);
    void setGrid(uint8 grid);
    double getPotential(void);
    void addOverlap(double x, double y, double z);
    void meanOverlap(double &x, double &y, double &z);
    int overlapPoints(void);
    void transOverlap(double x, double y, double z);
    
  private:
    std::vector<std::vector<double>> field_;
    std::vector<std::vector<double>> eigen_;
    std::vector<double> spectrum_;
    uint8 grid_;
    uint8 flag_;
    double potential_;
    
    double sumx;
    double sumy;
    double sumz;
    int sum;
    
    int getSpecIndex(int b);
  };
  
}  // namespace solid
}  // namespace aiethan

#endif  // AIETHAN_SOLID_TEXTURE3D_H_