#include <iostream>
#include <string>

#include <aiethan/tool/visdetect.h>

#include<vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>

#include<vcg/complex/algorithms/clean.h>
#include<vcg/complex/algorithms/smooth.h>

#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>
#include <wrap/ply/plylib.cpp>

// vcglib declare and definition
using namespace vcg;
class MyFace;
class MyVertex;

struct MyUsedTypes : public UsedTypes<	Use<MyVertex>::AsVertexType, Use<MyFace>::AsFaceType>{};
class MyVertex  : public Vertex< MyUsedTypes, vertex::VFAdj, vertex::Coord3f, vertex::Normal3f, vertex::BitFlags  >{};
class MyFace    : public Face  < MyUsedTypes, face::VFAdj, face::Normal3f, face::VertexRef, face::BitFlags > {};
class MyMesh    : public vcg::tri::TriMesh<vector<MyVertex>, vector<MyFace> > {};

/// \brief get timestamp
/// 
/// \return timestamp
int64_t GetCurrentStamp64()
{
  boost::posix_time::ptime epoch(boost::gregorian::date(1970, boost::gregorian::Jan, 1));
  boost::posix_time::time_duration time_from_epoch =
    boost::posix_time::microsec_clock::universal_time() - epoch;
    //boost::posix_time::second_clock::universal_time() - epoch;

  //return time_from_epoch.total_milliseconds();
  return time_from_epoch.total_microseconds();
  //return time_from_epoch.total_seconds();
} 

/// \brief vision perception class
///
/// public inherit the virtual class VisionDetect
class MyVisionDetect : public aiethan::solid::VisionDetect {
public:
  explicit MyVisionDetect(int depth);    
  ~MyVisionDetect();
  
  MyVisionDetect(const MyVisionDetect&) = delete;
  MyVisionDetect& operator=(const MyVisionDetect&) = delete;
  
  int processmesh(aiethan::solid::MyMesh& mesh, Json::Value& jsoninfo);
  int colormesh(int indexes, aiethan::solid::MyMesh& mesh);
};
MyVisionDetect::MyVisionDetect(int depth): VisionDetect(depth) {}
MyVisionDetect::~MyVisionDetect() {}

/// \brief implementing this virtual method of its base class
/// 
/// \param mesh triangle mesh
/// \param jsoninfo some params used by method and all the vision data
/// \return process result
int MyVisionDetect::processmesh(aiethan::solid::MyMesh& mesh, Json::Value& jsoninfo) {
  //std::cout <<"processmesh microsec=" << GetCurrentStamp64() << std::endl;
  //Json::FastWriter fw;
  //std::cout << fw.write(jsoninfo) << std::endl;
  //*
  if(mesh.n_vertices() > 1000) {
    Json::Int64 stamp = GetCurrentStamp64();
    std::stringstream ss;
    ss <<"-" << stamp;//ss.str()
    jsoninfo["id"] = Json::Value(jsoninfo["id"].asString()+ss.str());
    jsoninfo["persis_s"] = Json::Value(jsoninfo["persis_s"].asString()+ss.str());
    baseperceive(mesh, jsoninfo);
    mulperceive(mesh, jsoninfo);
    Json::StyledWriter sw;
    std::ofstream os;
    os.open(jsoninfo["persis_s"].asString()+".json");
    jsoninfo["persis_s"] = Json::Value("");
    os << sw.write(jsoninfo);
    os.close();
  }//*/
  return 0;
}

/// \brief implementing this virtual method of its base class. Ignore it.
/// 
/// \param indexes mesh's segment flag
/// \param mesh triangle mesh
/// \return process result
int MyVisionDetect::colormesh(int indexes, aiethan::solid::MyMesh& mesh) {
  return 0;
}

/// \brief surface reconstruction
/// 
/// \return result
int hjwang( int argc , char* argv[] );

/// \brief surface trim
/// 
/// \return result
int bing( int argc , char* argv[] );

double procrawl(Json::Value& jsoninfo, Json::Value& root, int index) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudsrc (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader reader;
  if (reader.read (jsoninfo["srcdir"].asString()+jsoninfo["src"].asString()+jsoninfo["index"].asString()+".ply", *cloudsrc) < 0) {
    return (false);
  }
  std::cout<<"read over and size="<<cloudsrc->size()<<std::endl;
  
  // Constructed class MyVisionDetect with 6 or 8.
  MyVisionDetect visd(6);
  visd.filter(*cloudsrc, root);
  std::cout<<"filtered size="<<cloudsrc->size()<<std::endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  visd.normalEsti(*cloudsrc, *cloud_with_normals, root);
  //* save filtered PointCloud with normals  
  pcl::PLYWriter writer;
  writer.write (jsoninfo["filterdir"].asString()+jsoninfo["filtered"].asString()+jsoninfo["index"].asString()+"filterNormal.ply", *cloud_with_normals, false, false);//*/
  
  // surface reconstruction and trim, then save them in PLY file.
  std::string infile = jsoninfo["filterdir"].asString()+jsoninfo["filtered"].asString()+jsoninfo["index"].asString()+"filterNormal.ply";
  std::string outfile = jsoninfo["filterdir"].asString()+jsoninfo["filtered"].asString()+jsoninfo["index"].asString()+"filterNormalMesh.ply";
  char* argvv[] = {"poissonrecon", "--in", const_cast<char*>(infile.c_str()),
    "--out", const_cast<char*>(outfile.c_str()),
    "--depth", "10", "--density"
  };
  hjwang(8, argvv);
  std::string trimfile = jsoninfo["filterdir"].asString()+jsoninfo["filtered"].asString()+jsoninfo["index"].asString()+"filterNormalMeshTrim.ply";
  char* trimv[] = {"surfacetrim", "--in", const_cast<char*>(outfile.c_str()),
    "--out", const_cast<char*>(trimfile.c_str()),
    "--smooth", "8",
    "--trim", "7"
  };
  bing(9, trimv);
  
  // smooth mesh and save it in a file.
  MyMesh m;
  int err = tri::io::Importer<MyMesh>::Open(m,trimfile.c_str());
  if(err) { // all the importers return 0 in case of success
    std::cout<<"Error in reading"<< trimfile.c_str()<<": '"<<tri::io::Importer<MyMesh>::ErrorMsg(err)<<std::endl;
    exit(-1);
  }
  // some cleaning to get rid of bad file formats like stl that duplicate vertexes..
  int dup = tri::Clean<MyMesh>::RemoveDuplicateVertex(m);
  int unref = tri::Clean<MyMesh>::RemoveUnreferencedVertex(m);
  std::cout<<"Removed "<<dup<<" duplicate and "<<unref<<" unreferenced vertices from mesh"<<std::endl;
  tri::UpdateTopology<MyMesh>::VertexFace(m);
  std::string trimsmoothfile = jsoninfo["filterdir"].asString()+jsoninfo["filtered"].asString()+jsoninfo["index"].asString()+"filterNormalMeshTrimSmooth.ply";
  tri::Smooth<MyMesh>::VertexCoordLaplacian(m,8);
  tri::io::ExporterPLY<MyMesh>::Save(m,trimsmoothfile.c_str(), false);
  
  // vision process......
  aiethan::solid::MyMesh mesh;
  //visd.mesh(*cloudsrc, mesh, root);//this line is an algorithms that builds mesh from PointCloud by delauney
  //*
  try
  {
    OpenMesh::IO::Options opt_read;
    if ( !OpenMesh::IO::read_mesh(mesh, trimsmoothfile, opt_read) )
    {
      std::cerr << "Cannot read mesh file 'meshtrim.ply'" << std::endl;
      return 1;
    }
  }
  catch( std::exception& x )
  {
    std::cerr << x.what() << std::endl;
    return 1;
  }//*/
  visd.smooth(mesh, root);
  /*
  try
  {
    OpenMesh::IO::Options opt_write;
    if ( !OpenMesh::IO::write_mesh(mesh, jsoninfo["filterdir"].asString()+jsoninfo["filtered"].asString()+jsoninfo["index"].asString()+"vismooth.ply", opt_write) )
    {
      std::cerr << "Cannot write mesh to file 'meshsmooth.ply'" << std::endl;
      return 1;
    }
  }
  catch( std::exception& x )
  {
    std::cerr << x.what() << std::endl;
    return 1;
  }//*/
  visd.curvature(mesh, root);
  Json::Value datajson;
  datajson["id"] = Json::Value(jsoninfo["id"].asString()+jsoninfo["index"].asString());
  datajson["directed_by"] = jsoninfo["directed_by"];//Json::Value("hjwang");
  datajson["initial_release_date"] = jsoninfo["initial_release_date"];//Json::Value("2018-03-14");
  datajson["name"] = jsoninfo["name"];//Json::Value("gesture-four");
  datajson["content"] = jsoninfo["content"];//Json::Value("This is a gesture of four.");
  datajson["shap"] = jsoninfo["shap"];
  datajson["color"] = jsoninfo["color"];
  datajson["material"] = jsoninfo["material"];
  datajson["phase"] = jsoninfo["phase"];
  datajson["note"] = jsoninfo["note"];
  datajson["vprefix_s"] = jsoninfo["vprefix_s"];
  datajson["vsuffix_s"] = jsoninfo["vsuffix_s"];
  datajson["vsize_d"] = jsoninfo["vsize_d"];
  datajson["lenth_d"] = jsoninfo["lenth_d"];
  datajson["width_d"] = jsoninfo["width_d"];
  datajson["height_d"] = jsoninfo["height_d"];
  datajson["persis_s"] = Json::Value(jsoninfo["importdir"].asString()+jsoninfo["import"].asString()+jsoninfo["index"].asString());
  datajson["key_i"] = root["key_i"];
  datajson["inf_i"] = root["inf_i"];
  datajson["sup_i"] = root["sup_i"];
  
  visd.processmesh(mesh, datajson);
  //visd.segment(mesh, datajson);
  
  return 0;
}


int main(int argc, char **argv) {
  //read configuration file of scancrawler.json
  Json::Reader jsonreader;
  Json::Value jsoninfo;
  std::ifstream in("/home/hongjun/hjwang/data3D/camera/scancrawler.json");
  if( !in.is_open() ) {
    std::cout<<"scan crawler file: open failed."<<std::endl;
    return -1;
  }
  if (!jsonreader.parse(in, jsoninfo, false)) {
    std::cout<<"scan crawler file: parse failed."<<std::endl;
    return -1;
  }
  in.close();
  
  //read configuration file of scaninfo.json
  Json::Reader rootreader;
  Json::Value root;
  std::ifstream info("/home/hongjun/hjwang/data3D/camera/scaninfo.json");
  if( !info.is_open() ) {
    std::cout<<"scan info file: open failed."<<std::endl;
    return -1;
  }
  if (!rootreader.parse(info, root, false)) {
    std::cout<<"scan info file: parse failed."<<std::endl;
    return -1;
  }
  info.close();
  
  // print some configuration in json file
  std::cout<<"src="<<jsoninfo["src"].asString()<<std::endl;
  std::cout<<"filtered="<<jsoninfo["filtered"].asString()<<std::endl;
  std::cout<<"id="<<jsoninfo["id"].asString()<<std::endl;
  std::cout<<"directed_by="<<jsoninfo["directed_by"].asString()<<std::endl;
  std::cout<<"date="<<jsoninfo["initial_release_date"].asString()<<std::endl;
  std::cout<<"name="<<jsoninfo["name"].asString()<<std::endl;
  std::cout<<"content="<<jsoninfo["content"].asString()<<std::endl;
  std::cout<<"import="<<jsoninfo["import"].asString()<<std::endl;
  std::cout<<"start-index="<<jsoninfo["startindex"].asInt()<<std::endl;
  std::cout<<"end-index="<<jsoninfo["endindex"].asInt()<<std::endl;
  std::cout<<"threshold="<<root["threshold"].asInt()<<std::endl;
  std::cout<<"filter_minz="<<root["filter_minz"].asInt()<<std::endl;
  std::cout<<"filter_maxz="<<root["filter_maxz"].asInt()<<std::endl;
  std::cout<<"filter_miny="<<root["filter_miny"].asInt()<<std::endl;
  std::cout<<"filter_maxy="<<root["filter_maxy"].asInt()<<std::endl;
  std::cout<<"filter_minx="<<root["filter_minx"].asInt()<<std::endl;
  std::cout<<"filter_maxx="<<root["filter_maxx"].asInt()<<std::endl;
  std::cout<<"distance="<<root["distance"].asDouble()<<std::endl;
  std::cout<<"Niters="<<root["Niters"].asInt()<<std::endl;
  std::cout<<"key_i="<<root["key_i"].asInt()<<std::endl;
  std::cout<<"inf_i="<<root["inf_i"].asInt()<<std::endl;
  std::cout<<"sup_i="<<root["sup_i"].asInt()<<std::endl;
  
  // process files of 3D PointCloud one by one
  std::vector<double> eigs;
  double d = 0;
  for(int i = jsoninfo["startindex"].asInt(); i <= jsoninfo["endindex"].asInt(); i++) {
    jsoninfo["index"] = Json::Value(i);
    std::cout<<"now-index="<<jsoninfo["index"].asInt() <<std::endl;
    d = procrawl(jsoninfo, root, i);
    eigs.push_back(d);
  }
  
  // print all the result
  int j = 0;
  for(auto val : eigs) {
    j++;
    std::cout<<"distribution("<<j <<")=" <<val <<std::endl;
  }
    
  return 0;
}