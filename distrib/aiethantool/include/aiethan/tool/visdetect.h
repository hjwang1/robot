/*!
 * \file octo_node.h
 * \brief 3D vision for detect
 * 
 * Information about vision.
 * 
 * \author Hongjun Wang, email:hjwang1@163.com, Copyright Aiethan http://www.aiethan.com/.
 * \version 1.0
 * \date 2018-04-28
 */

#ifndef AIETHAN_SOLID_VISDETECT_H_
#define AIETHAN_SOLID_VISDETECT_H_

#include <pcl/io/ply_io.h>
#include <json/json.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>

namespace aiethan {
namespace solid {
  struct MyTraits : public OpenMesh::DefaultTraits
  {
    // store barycenter of neighbors in this member
    VertexTraits
    {
    private:
      Point  cog_;
    public:

      VertexT() : cog_( Point(0.0f, 0.0f, 0.0f ) ) { }

      const Point& cog() const { return cog_; }
      void set_cog(const Point& _p) { cog_ = _p; }
    };
  };

  typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  MyMesh;
  typedef OpenMesh::TriMesh_ArrayKernelT<>          MyMesh2;

  class VisionDetect {
  public:
    static constexpr int RES = 1024;
    static constexpr int WIDTH = 640;
    static constexpr int HEIGHT = 480;
    static constexpr double X_FOV = 0.5588811;
    static constexpr double Y_FOV = 0.4193348;

    static constexpr int minz = 600;
    static constexpr int maxz = 1000;
    static constexpr int minx = -400;
    static constexpr int maxx = 400;
    static constexpr int miny = -400;
    static constexpr int maxy = 400;
    
    explicit VisionDetect(int depth);    
    ~VisionDetect();
    
    VisionDetect(const VisionDetect&) = delete;
    VisionDetect& operator=(const VisionDetect&) = delete;
    
    int detect(pcl::PointCloud<pcl::PointXYZ>& cloud, Json::Value& jsoninfo);
    int manifold(MyMesh& mesh);
    
    int filter(pcl::PointCloud<pcl::PointXYZ>& cloud, Json::Value& jsoninfo);
    int normalEsti(pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointNormal>& pointnormal, Json::Value& jsoninfo);
    int mesh(pcl::PointCloud<pcl::PointXYZ>& cloud, MyMesh& mesh, Json::Value& jsoninfo);
    int smooth(MyMesh& mesh, Json::Value& jsoninfo);
    int segment(MyMesh& mesh, Json::Value& jsoninfo, int (*func)(MyMesh&, Json::Value&), int(*colorout)(int, MyMesh&));
    int curvature(MyMesh& mesh, Json::Value& jsoninfo);
    int baseperceive(MyMesh& mesh, Json::Value& jsoninfo);
    int mulperceive(MyMesh& mesh, Json::Value& jsoninfo);
    virtual int processmesh(MyMesh& mesh, Json::Value& jsoninfo);
    virtual int colormesh(int indexes, MyMesh& mesh);
    
  private:    
    int depth_;
    int coreRangePerceive(MyMesh& mesh, Json::Value& jsoninfo, OpenMesh::VPropHandleT<double>& keycurvature, int inf, int sup);
    int lengRangePerceive(MyMesh& mesh, Json::Value& jsoninfo, OpenMesh::VPropHandleT<double>& keycurvature, int inf, int sup);
    int riemman(Json::Value& jsoninfo, int index);
  };
  
}  // namespace solid
}  // namespace aiethan

#endif  // AIETHAN_SOLID_VISDETECT_H_