/*!
 * \file analysis.h
 * \brief common tools
 * 
 * Analysis can be used for robot perception and road planning.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2017-07-24
 */

#ifndef AIETHAN_COMMON_ANALYSIS_H_
#define AIETHAN_COMMON_ANALYSIS_H_

#include "aiethan/solid/octo_util.h"
#include "aiethan/solid/octo_node.h"
#include "aiethan/solid/octo_tree.h"
#include "aiethan/solid/texture3d.h"
#include "aiethan/common/match_pair.h"
#include "aiethan/common/aivector.h"

/// \brief private package aiethan::common
///
/// top namespace of aiethan, tools for common
namespace aiethan {
namespace solid {
  class Analysis {
  public:
    static constexpr double kVARIANCE = 4;
    static constexpr double kTHRESHOLD = 0.005; //0.000001;0.005;0.0006;0.000006
    static constexpr double kMatchCovering = 100;
    
    explicit Analysis();
    ~Analysis();
    
    Analysis(const Analysis&) = delete;
    Analysis& operator=(const Analysis&) = delete;
    
    /// \brief get pre-frame
    ///
    /// const return.
    /// \param pre-frame
    /// \return the number of matching.
    const aiethan::common::vector& getFrameNow() const;
    void setFrame(OctoTree* tree);
    //void setFrame(OctoTree *tree);
    void doFrame();
    /// \brief get feature of movement tensor
    ///
    /// return cos distance of feature
    /// \return cos distance of feature. -1 means that no movement feature.
    double cosDis();
    double getXDist();
    double getYDist();
    double getZDist();
    int getMatchSize();
    void setMatchFlag(double);
    void printVision(void);
    void printErrors(void);
    //check for manual
    void setManual(bool);
    double getManualN(void);
    
    //only for test
    void initFrames(OctoTree* first, OctoTree* second);
    void viewFrames(uint8 layer);
    void setTest(bool);
    
  private:
    static aiethan::common::vector array;
    static aiethan::common::vector arrayThen;
    
    static std::vector<aiethan::common::MatchPair> matches;
    static std::vector<aiethan::common::MatchPair> matchesTmp;
    
    static std::vector<aiethan::common::Error> errors;
    
    static Texture3d* visionX;
    static Texture3d* visionY;
    static Texture3d* visionZ;    
    
    static double s[3];
    static double sp[3];
    static double v[3];
    
    OctoTree* pre;
    OctoTree* next;
    bool direcFlag;
    
    //check for manual
    bool manual=false;
    std::vector<double> checks;
    double mSup=0;
    std::vector<double> checkp;
    bool testFlag=false;
    
    void listNodes();
    void centerNode(OctoNode* parent, double &x, double &y, double &z);//estimate tree
    void matchCenter(OctoTree* tree, OctoNode* parent, double &x, double &y, double &z);//estimate tree
    static bool sortvec(uint64_t v1, uint64_t v2);
    static bool sortdis(uint64_t v1, uint64_t v2);
    void comp();
    void getMatches(int preIndex, int preNum, int nextIndex, int nextNum);
    void toplogyMatch(int preIndex, int preNum, int nextIndex, int nextNum);
    double getVar();
    void getWave();
    bool comdis(uint64_t v1, uint64_t v2, aiethan::common::Error error);
    
    void handle(OctoNode* tmp, double pval);//only for setMatchFlag
    
    //only for test
    void listFrames(bool first, uint8 layer);
  };
  
}  // namespace solid
}  // namespace aiethan

#endif  // AIETHAN_COMMON_ANALYSIS_H_