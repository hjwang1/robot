/*!
 * \file octo_tree.h
 * \brief 3D vision by octotree
 * 
 * Octotree can be used for robot perception and road planning.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-04-05
 */

#ifndef AIETHAN_SOLID_OCTO_TREE_H_
#define AIETHAN_SOLID_OCTO_TREE_H_

#include "aiethan/solid/octo_node.h"

/// \brief private package aiethan::solid
///
/// top namespace of aiethan, then 2D for plain and 3D for solid
namespace aiethan {
  constexpr double kBASE_POW[23] = {0.5,1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,32768,65536,131072,262144,524288,1048576,2097152};
namespace solid {
  /// \brief class of octotree
  ///
  ///application interface of octotree 
  class OctoTree {
  public:
    static constexpr uint8 kRESTRUCT = 1;
    static constexpr int kFIELD_LEN = 4;
    
    static constexpr int kWHITE_NUM = 4;
    
    static constexpr uint8 kSUPER_LIMIT = 22;
    
    static constexpr int kNODE_DATA = 5;
    
    explicit OctoTree(uint8 maxDepth); 
    explicit OctoTree(uint8 maxDepth, uint8 limit);
    explicit OctoTree(uint8 maxDepth, uint8 limit, double resolution, bool forceField);
    ~OctoTree();
    
    OctoTree(const OctoTree&) = delete;
    OctoTree& operator=(const OctoTree&) = delete;
    
    /// \brief insert a node into this tree
    ///
    /// construct a node by coordinate(x,y) with occupied and expanding.
    /// \param x point of x axis
    /// \param y point of y axis
    /// \param z point of z axis
    /// \return void.
    OctoNode* updateNode(double x, double y, double z);
    /// \brief insert a node into this tree
    ///
    /// construct a node by coordinate(x,y) and occupation considering whether or not expanding.
    /// \param x point of x axis
    /// \param y point of y axis
    /// \param z point of z axis
    /// \param expand whether or not expanding
    /// \param occu whether or not occupied
    /// \return void.
    OctoNode* updateNode(double x, double y, double z, bool expand, uint8 occu);
    /// \brief make sure this tree's depth
    ///
    /// get (x,y, z) from vision of (a, b, c).
    /// \param a vison of x axis
    /// \param b vison of y axis
    /// \param c vison of z axis
    /// \param x point of x axis
    /// \param y point of y axis
    /// \param z point of z axis
    /// \return void.
    void updateNode(double a, double b, double c, double x, double y, double z);
    /// \brief init initField
    ///
    /// tree is generated, and to initialize field for vision and road planning
    void initField(void);
    /// \brief get node whoes coordinate is (x, y)
    ///
    /// \param x axis x
    /// \param y axis y
    /// \param z axis z
    /// \return target OctoNode
    OctoNode* getNode(double x, double y, double z);
    /// \brief get a distance node
    ///
    /// get the node whoes distance is (x, y) with leaf node
    /// \param leaf start node
    /// \param x axis distance x
    /// \param y axis distance y
    /// \param z axis distance z
    /// \return target OctoNode
    OctoNode* getDisNode(OctoNode* leaf, double x, double y, double z);
    /// \brief to get eigen and spectrum for perception
    ///
    /// tree was generated and initialized field before calling this function 
    void initEigenAndSpectrum(void);
    void bottomUP(void);
    void overlap(void);
    
    double getResolution(void);
    uint8 getLimit(void);
    uint8 getMaxDepthCurrent(void);
    OctoNode* getRoot(void) const;
    
  private:
    uint8 maxDepth_; 	///< current max depth of this tree
    OctoNode* root_; 	///< root node of this tree
    uint8 limit_;	///< when expanding, limit is max depth for tree
    double resolution_;	///< resolution of this tree
    
    bool forceField_; 	///< force to get field whether or not blank and unscanned
    
    /// \brief init constructor function
    ///
    /// init some complex computer when constructor.
    /// \param maxDepth current depth of tree
    /// \param limit max depth of tree when expanding
    /// \return void.
    void init_cs(uint8 maxDepth, uint8 limit);
    
    /// \brief check coordinate
    ///
    /// check that if coordinate(x,y,z) is out of this tree or not.
    /// \param x coordinate of x axis
    /// \param y coordinate of y axis
    /// \param z coordinate of z axis
    /// \return true if (x,y,z) is in this tree or else false.
    bool checkBoundary(double x, double y, double z) const;
    /// \brief expand this tree
    ///
    /// tree's old root is in quad of tree's new root
    /// \param quad old root's position in new root
    /// \return void
    void expand(int quad);
    /// \brief node's field over its neighbor
    ///
    /// init field for vision and rp
    void doField(OctoNode* node);
    /// \brief check object on road
    ///
    /// there is or not one or more points which are occupied.
    /// \param leaf start point
    /// \param i axis x distance
    /// \param j axis y distance
    /// \return true if there is one or more objects
    bool checkMiddle(OctoNode* leaf, double i, double j, double k);
    
    void handle(OctoNode* pNode);
    double getRiemann(OctoNode* pNode);
    void expand(OctoNode* leaf);
    
  };
  
}  // namespace solid
}  // namespace aiethan

#endif  // AIETHAN_SOLID_OCTO_TREE_H_