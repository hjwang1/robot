/*
 * Copyright 2017 The Aiethan Authors Hongjun Wang.
 *
 * You may reference to
 *
 *      http://www.aiethan.com/
 *
 * Good luck.
 */

#ifndef AIETHAN_PLANE_QUAD_NODE_H_
#define AIETHAN_PLANE_QUAD_NODE_H_

#include "aiethan/plain/quad_util.h"

namespace aiethan {
namespace plain {
  class QuadNode {
  public:
    static constexpr uint8 kQUAD_1ST = 0;
    static constexpr uint8 kQUAD_2ND = 1;
    static constexpr uint8 kQUAD_3RD = 2;
    static constexpr uint8 kQUAD_4TH = 3;
    static constexpr uint8 kQUAD = 4;
    
    explicit QuadNode(uint8 depth);    
    ~QuadNode();
    
    QuadNode(const QuadNode&) = delete;
    QuadNode& operator=(const QuadNode&) = delete;
    
    void setChild(QuadNode* node, int quad_index);
    QuadNode* getChild(int8 quad);
    uint8 getDepth();
    void setDepth(uint8 depth);
    void setParent(QuadNode* parent);
    QuadNode* getParent(void);
    void setParentQuad(int parentQuad);
    int getParentQuad(void);
    void setAttr(void* attr);
    void* getAttr(void);
    
    double getData(void);
    void setData(double data);
    double getX(void);
    void setX(double x);
    double getY(void);
    void setY(double y);
    
  private:
    double data_;		///< may be expanded later
    uint8 depth_;		///< node's depth into this tree
    QuadNode* children_[kQUAD];	///< children_[4]
    QuadNode* parent_;		///< node's parent into this tree
    int parentQuad_;		///< parent's position relative to this node
    
    void* attr_;
    
    /// only for tree's writing and expanded
    double x_;
    double y_;
  };
  
}  // namespace plain
}  // namespace aiethan

#endif  // AIETHAN_PLANE_QUAD_NODE_H_