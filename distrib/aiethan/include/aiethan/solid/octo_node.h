/*!
 * \file octo_node.h
 * \brief 3D vision for occupied grid
 * 
 * Information about octo node.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-04-05
 */

#ifndef AIETHAN_SOLID_OCTO_NODE_H_
#define AIETHAN_SOLID_OCTO_NODE_H_

#include "aiethan/solid/octo_util.h"

namespace aiethan {
namespace solid {
  class OctoNode {
  public:
    static constexpr uint8 kQUAD_1ST = 0;
    static constexpr uint8 kQUAD_2ND = 1;
    static constexpr uint8 kQUAD_3RD = 2;
    static constexpr uint8 kQUAD_4TH = 3;
    static constexpr uint8 kQUAD_1ST_U = 6;
    static constexpr uint8 kQUAD_2ND_U = 7;
    static constexpr uint8 kQUAD_3RD_U = 4;
    static constexpr uint8 kQUAD_4TH_U = 5;
    static constexpr uint8 kOCTO = 8;
    
    explicit OctoNode(uint8 depth);    
    ~OctoNode();
    
    OctoNode(const OctoNode&) = delete;
    OctoNode& operator=(const OctoNode&) = delete;
    
    void setChild(OctoNode* node, int quad_index);
    OctoNode* getChild(int quad);
    uint8 getDepth();
    void setDepth(uint8 depth);
    void setParent(OctoNode* parent);
    OctoNode* getParent(void);
    void setParentQuad(int parentQuad);
    int getParentQuad(void);
    void setAttr(void* attr);
    void* getAttr(void) const;
    
    double getData(void);
    void setData(double data);
    double getX(void);
    void setX(double x);
    double getY(void);
    void setY(double y);
    double getZ(void);
    void setZ(double z);
    void setBase(double x, double y, double z);
    
  private:
    double data_;		///< may be expanded later
    uint8 depth_;		///< node's depth into this tree
    OctoNode* children_[kOCTO];	///< children_[4]
    OctoNode* parent_;		///< node's parent into this tree
    int parentQuad_;		///< parent's position relative to this node
    
    void* attr_;
    
    /// only for tree's writing and expanded
    double x_;
    double y_;
    double z_;
  };
  
}  // namespace solid
}  // namespace aiethan

#endif  // AIETHAN_SOLID_OCTO_NODE_H_