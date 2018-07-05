/*!
 * \file quad_tree.h
 * \brief 2D vision by quadtree
 * 
 * Quadtree can be used for robot perception and road planning.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-03-27
 */

#ifndef AIETHAN_PLANE_QUAD_TREE_H_
#define AIETHAN_PLANE_QUAD_TREE_H_

#include "aiethan/plain/quad_node.h"

/// \brief private package aiethan::plain
///
/// top namespace of aiethan, then 2D for plain and 3D for solid
namespace aiethan {
namespace plain {
  /// \brief class of quadtree
  ///
  ///application interface of quadtree 
  class QuadTree {
  public:
    static constexpr uint8 kRESTRUCT = 1;
    static constexpr int kFIELD_LEN = 4;
    
    explicit QuadTree(uint8 maxDepth);   
    explicit QuadTree(uint8 maxDepth, uint8 limit, double resolution);
    ~QuadTree();
    
    QuadTree(const QuadTree&) = delete;
    QuadTree& operator=(const QuadTree&) = delete;
    
    /// \brief insert a node into this tree
    ///
    /// construct a node by coordinate(x,y) with occupied and expanding.
    /// \param x point of x axis
    /// \param y point of y axis
    /// \return void.
    void updateNode(double x, double y);
    /// \brief insert a node into this tree
    ///
    /// construct a node by coordinate(x,y) and occupation considering whether or not expanding.
    /// \param x point of x axis
    /// \param y point of y axis
    /// \param expand whether or not expanding
    /// \param occu whether or not occupied
    /// \return void.
    void updateNode(double x, double y, bool expand, uint8 occu);
    /// \brief init initField
    ///
    /// tree is generated, and to initialize field for vision and road planning
    void initField(void);
    /// \brief get node whoes coordinate is (x, y)
    ///
    /// \param x axis x
    /// \param y axis y
    /// \return target QuadNode
    QuadNode* getNode(double x, double y);
    /// \brief get a distance node
    ///
    /// get the node whoes distance is (x, y) with leaf node
    /// \param leaf start node
    /// \param x axis distance x
    /// \param y axis distance y
    /// \return target QuadNode
    QuadNode* getDisNode(QuadNode* leaf, double x, double y);
    /// \brief to get eigen and spectrum for perception
    ///
    /// tree was generated and initialized field before calling this function 
    void initEigenAndSpectrum(void);
    
    double getResolution(void);
    uint8 getLimit(void);
    uint8 getMaxDepthCurrent(void);
    QuadNode* getRoot(void);
    
  private:
    uint8 maxDepth_; 	///< current max depth of this tree
    QuadNode* root_; 	///< root node of this tree
    uint8 limit_;	///< when expanding, limit is max depth for tree
    double resolution_;	///< resolution of this tree
    
    /// \brief init constructor function
    ///
    /// init some complex computer when constructor.
    /// \param maxDepth current depth of tree
    /// \param limit max depth of tree when expanding
    /// \return void.
    void init_cs(uint8 maxDepth, uint8 limit);
    
    /// \brief check coordinate
    ///
    /// check that if coordinate(x,y) is out of this tree or not.
    /// \param x coordinate of x axis
    /// \param y coordinate of y axis
    /// \return true if (x,y) is in this tree or else false.
    bool checkBoundary(double x, double y) const;
    /// \brief expand this tree
    ///
    /// tree's old root is in quad of tree's new root
    /// \param quad old root's position in new root
    /// \return void
    void expand(int quad);
    /// \brief node's field over its neighbor
    ///
    /// init field for vision and rp
    void doField(QuadNode* node);
    /// \brief check object on road
    ///
    /// there is or not one or more points which are occupied.
    /// \param leaf start point
    /// \param i axis x distance
    /// \param j axis y distance
    /// \return true if there is one or more objects
    bool checkMiddle(QuadNode* leaf, double i, double j);
    
  };
  
}  // namespace plain
}  // namespace aiethan

#endif  // AIETHAN_PLANE_QUAD_TREE_H_