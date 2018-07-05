/*!
 * \file octo_node.h
 * \brief 3D vision for occupied grid
 * 
 * Information about octo node.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2018-04-05
 */

#ifndef AIETHAN_SOLID_HADAMARD_H_
#define AIETHAN_SOLID_HADAMARD_H_

#include <Eigen/Dense>
#include <aiethan/solid/octo_util.h>
#include <json/json.h>

namespace aiethan {
namespace solid {
  class Hadamard {
  public:
    static constexpr int RES = 1024;
    static constexpr int GRADE = 10;
    static constexpr int SPECSIZE = 11;
    static int POWE[SPECSIZE];		///< POWE[i] = 2^i
    static constexpr double epsilon = 0.00000000000000000001;
    static constexpr double sqrt11  = 3.3166247903;
    static std::string CURV[SPECSIZE];
    
    explicit Hadamard(int depth);    
    ~Hadamard();
    
    Hadamard(const Hadamard&) = delete;
    Hadamard& operator=(const Hadamard&) = delete;
    
    int8** getH();
    int* getExpoint();
    int trans(double *in, int rows, int cols, double *out);
    double* getSpectrum(double *in, int rows, int cols);
    Eigen::MatrixXd getMat();
    
    int multiResis(Eigen::MatrixXd& in, int rows, int cols, Json::Value& jsoninfo, int limit);
    int multiCover(Eigen::MatrixXd& in, int rank, Json::Value& jsoninfo, int limit);
    
  private:
    double spectrum[SPECSIZE];		///< index= 0 1 2 3 4 5 6 7 8 9 10
    int expoint[RES];			///< [0 1 2 2 3 3 3 3 ... 10]  i's value is index & 2^(index-1) <= i < 2^index
    int8** H;				///< low speed
    
    Eigen::MatrixXd m;
    double getEigen(Eigen::MatrixXd& in, int rank);
    
    int depth_;
  };
  
}  // namespace solid
}  // namespace aiethan

#endif  // AIETHAN_SOLID_HADAMARD_H_