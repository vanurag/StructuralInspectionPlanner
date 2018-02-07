/*!
 * \file bigBen.cpp
 *
 * More elaborate description
 */
#include <ros/ros.h>
#include "koptplanner/inspection.h"
#include "shape_msgs/SolidPrimitive.h"
#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <ros/package.h>
#include "tf/tf.h"
#include <Eigen/Dense>
#include <octomap/octomap.h>
#include "../../utils/Lowess.h"

using namespace octomap;

bool isInsideVoxel(const geometry_msgs::Point32& p, const octomap::OcTreeVolume v);
bool isInsideVoxel(const octomap::point3d& p, const octomap::OcTreeVolume v);
std::vector<nav_msgs::Path> * readSTLfile(std::string name);
double g_maxSpeed;
double g_maxAngularSpeed;
Eigen::Vector3d g_mavStartPos;
std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> g_mavRequiredPose;  // x,y,z,roll,pitch,yaw for start, intermediate and final poses
int g_numIntermediatePos = 11;  // the hemisphere around the tree is divided into these many +1 slices for spitting out path segments
int g_ocTreeInspectionDepth = 16;
std::vector<octomap::OcTreeVolume> g_ocTreeInspectionVoxels;  // list of volumes at depth g_ocTreeInspectionDepth, to be inspected
std::vector<std::vector<octomap::OcTreeVolume>> *g_ocTreeInspectionObstacles;  // list of obstacles corresponding to each entry in g_ocTreeInspectionVoxels
int g_inspectionVolumeIndex;
Eigen::VectorXd g_boundingBox(6);
double g_incidenceAngle;
Eigen::Vector2d g_minmaxDistance;
int g_numIterations;
CppLowess::TemplatedLowess<std::vector<double>, double> g_dlowess;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tree");
  ROS_INFO("speed tree");
  if (argc != 1)
  {
    ROS_INFO("usage: plan");
    return 1;
  }

  ros::NodeHandle n;
  ros::Publisher obstacle_pub = n.advertise<visualization_msgs::Marker>("scenario", 1);
  ros::Publisher stl_pub = n.advertise<nav_msgs::Path>("stl_mesh", 1);
  ros::ServiceClient client = n.serviceClient<koptplanner::inspection>("inspectionPath");

  ros::Rate r(50.0);
  ros::Rate r1(50.0);
  ros::Rate r2(1.0);
  r2.sleep();

  if(!ros::param::get("/Inspection_Planner/rotorcraft/maxSpeed", g_maxSpeed)) {
    std::cout << "couldn't get max speed param" << std::endl;
    exit(1);
  } else {
    std::cout << "got max speed param: " << g_maxSpeed << std::endl;
  }
  if(!ros::param::get("/Inspection_Planner/rotorcraft/maxAngularSpeed", g_maxAngularSpeed)) {
    std::cout << "couldn't get max angular speed param" << std::endl;
    exit(1);
  } else {
    std::cout << "got max angular speed param: " << g_maxAngularSpeed << std::endl;
  }
  if(!ros::param::get("/Inspection_Planner/octree/inspectionDepth", g_ocTreeInspectionDepth)) {
    std::cout << "couldn't get octree inspection depth param" << std::endl;
    exit(1);
  } else {
    std::cout << "got octree inspection depth param: " << g_ocTreeInspectionDepth << std::endl;
  }
  if(!ros::param::get("/Inspection_Planner/octree/inspectionVolumeIndex", g_inspectionVolumeIndex)) {
    std::cout << "couldn't get octree inspection volume index" << std::endl;
    exit(1);
  } else {
    std::cout << "got octree inspection volume index: " << g_inspectionVolumeIndex << std::endl;
  }
  std::vector<double> mav_start_pos;
  if(!ros::param::get("/Inspection_Planner/interface/mavStartPosition", mav_start_pos)) {
    std::cout << "couldn't get start position for MAV" << std::endl;
    exit(1);
  } else {
    std::cout << "got start position for MAV: " << mav_start_pos[0] << " " << mav_start_pos[1] << " " << mav_start_pos[2] << std::endl;
  }
  g_mavStartPos.x() = mav_start_pos[0];
  g_mavStartPos.y() = mav_start_pos[1];
  g_mavStartPos.z() = mav_start_pos[2];
  if(!ros::param::get("/Inspection_Planner/interface/numIntermediatePositions", g_numIntermediatePos)) {
    std::cout << "couldn't get number of intermediate positions needed to break the total path" << std::endl;
    exit(1);
  } else {
    std::cout << "got number of intermediate positions needed: " << g_numIntermediatePos << std::endl;
  }
  std::vector<double> bounding_box;
  if(!ros::param::get("/Inspection_Planner/interface/boundingBox", bounding_box)) {
    std::cout << "couldn't get bounding box" << std::endl;
    exit(1);
  } else {
    std::cout << "got bounding box: " << bounding_box[0] << " " << bounding_box[1] << " " << bounding_box[2] << " " <<
                                         bounding_box[3] << " " << bounding_box[4] << " " << bounding_box[5] << std::endl;
  }
  for (int i = 0; i < 6; ++i) g_boundingBox[i] = bounding_box[i];
  if(!ros::param::get("/Inspection_Planner/interface/incidenceAngle", g_incidenceAngle)) {
    std::cout << "couldn't get incidence angle" << std::endl;
    exit(1);
  } else {
    std::cout << "got incidence angle: " << g_incidenceAngle << std::endl;
  }
  if(!ros::param::get("/Inspection_Planner/interface/minDist", g_minmaxDistance[0])) {
    std::cout << "couldn't get min distance" << std::endl;
    exit(1);
  } else {
    std::cout << "got minimum distance: " << g_minmaxDistance[0] << std::endl;
  }
  if(!ros::param::get("/Inspection_Planner/interface/maxDist", g_minmaxDistance[1])) {
    std::cout << "couldn't get max distance" << std::endl;
    exit(1);
  } else {
    std::cout << "got maximum distance: " << g_minmaxDistance[1] << std::endl;
  }
  if(!ros::param::get("/Inspection_Planner/interface/numIterations", g_numIterations)) {
    std::cout << "couldn't get num interations" << std::endl;
    exit(1);
  } else {
    std::cout << "got num iterations: " << g_numIterations << std::endl;
  }

  /* Get MAV required positions */
  double slice_radians = 2*M_PI/(g_numIntermediatePos+1);
  Eigen::AngleAxisd rot_matrix(slice_radians, Eigen::Vector3d(0,0,1));
//  std::cout << "check interm1: " << rot_matrix.matrix() << std::endl;
//  std::cout << "check interm2: " << rot_matrix.matrix() * g_mavStartPos << std::endl;
  for (int i = 0; i < g_numIntermediatePos+2; ++i) {
    if (i == 0 || i == g_numIntermediatePos+1) {
      g_mavRequiredPose.push_back(std::make_pair(g_mavStartPos,Eigen::Vector3d(0,0,0)));
    } else {
      g_mavRequiredPose.push_back(
          std::make_pair(Eigen::Vector3d(rot_matrix.matrix() * g_mavRequiredPose.back().first),
                         Eigen::Vector3d(0,0,i*slice_radians)));
    }
  }

  /* Read octree */
  std::cout << "\nReading OcTree file\n===========================\n" << std::endl;
  OcTree* tree = new OcTree(std::string(ros::package::getPath("request")+"/meshes/tree.bt"));
  std::cout << "Octree depth: " << tree->getTreeDepth() << std::endl;
  std::cout << "Num leaf nodes: " << tree->getNumLeafNodes() << std::endl;
  // get inspection volumes
  std::cout << "Inspection Volumes: " << std::endl;
  for(OcTree::leaf_iterator it = tree->begin_leafs(g_ocTreeInspectionDepth), end=tree->end_leafs();
      it!= end; ++it){
    if (it->getOccupancy() > 0) {
      std::cout << it.getDepth() << "  " << it.getCoordinate() << "  " << it.getSize() << "  " << it->getOccupancy() << std::endl;
      g_ocTreeInspectionVoxels.push_back(std::make_pair(it.getCoordinate(), it.getSize()));
    }
  }
  std::cout << "No. inspection volumes: " << g_ocTreeInspectionVoxels.size() << " " << g_inspectionVolumeIndex << std::endl;
  if (g_inspectionVolumeIndex >= (int)(g_ocTreeInspectionVoxels.size())) {
    ROS_ERROR("Inspection volume index provided is outside limit. choose in [0 - %d] range", g_ocTreeInspectionVoxels.size()-1);
    exit(1);
  }
  g_ocTreeInspectionObstacles = new std::vector<std::vector<octomap::OcTreeVolume>>(g_ocTreeInspectionVoxels.size());
  std::cout << "Inspection Obstacles: " << std::endl;
  // get insepction obstacles
  int v_min = 0;
  for(OcTree::leaf_iterator it = tree->begin_leafs(), end=tree->end_leafs();
      it!= end; ++it){
    if (it->getValue() > 0) {
      std::cout << it.getDepth() << "  " << it.getCoordinate() << "  " << it.getSize() << "  " << it->getValue() << " " << it->getOccupancy();
      for (int v = v_min; v < g_ocTreeInspectionVoxels.size(); ++v) {
        if (isInsideVoxel(it.getCoordinate(), g_ocTreeInspectionVoxels[v])) {
          v_min = v;
          g_ocTreeInspectionObstacles->at(v).push_back( std::make_pair(it.getCoordinate(), it.getSize()) );
          std::cout << " belong to " << v << std::endl;
          break;
        }
      }
    }
  }
  std::cout << "Finished reading octree..." << std::endl;

  /* define the bounding box */
  koptplanner::inspection srv;
  srv.request.spaceSize.push_back(g_boundingBox[3]);
  srv.request.spaceSize.push_back(g_boundingBox[4]);
  srv.request.spaceSize.push_back(g_boundingBox[5]);
  srv.request.spaceCenter.push_back(g_boundingBox[0]);
  srv.request.spaceCenter.push_back(g_boundingBox[1]);
  srv.request.spaceCenter.push_back(g_boundingBox[2]);

  /* required poses*/
  geometry_msgs::Pose reqPose;
  tf::Quaternion q;
  for (int i = 0; i < g_numIntermediatePos+2; ++i) {
    reqPose.position.x = g_mavRequiredPose[i].first[0];
    reqPose.position.y = g_mavRequiredPose[i].first[1];
    reqPose.position.z = g_mavRequiredPose[i].first[2];
    q = tf::createQuaternionFromRPY(g_mavRequiredPose[i].second[0],
                                    g_mavRequiredPose[i].second[1],
                                    g_mavRequiredPose[i].second[2]);
    reqPose.orientation.x = q.x();
    reqPose.orientation.y = q.y();
    reqPose.orientation.z = q.z();
    reqPose.orientation.w = q.w();
    std::cout << "adding req. pose: " << reqPose.position << "\n" << reqPose.orientation << std::endl;
    srv.request.requiredPoses.push_back(reqPose);
  }

  /* parameters for the path calculation (such as may change during mission) */
  srv.request.incidenceAngle = g_incidenceAngle;//M_PI/18.0;
  srv.request.minDist = g_minmaxDistance[0];
  srv.request.maxDist = g_minmaxDistance[1];
  srv.request.numIterations = g_numIterations;

  /* read STL file and publish to rviz */
  std::string fname(ros::package::getPath("request")+std::string("/meshes/tree.stl"));
  ROS_INFO("Reading mesh: %s", fname.c_str());
  std::vector<nav_msgs::Path> * mesh = readSTLfile(ros::package::getPath("request")+"/meshes/tree.stl");
  ROS_INFO("mesh size = %i", (int)mesh->size());
  int num_mesh_tri = 0;
  for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end() && ros::ok(); it++)
  {
    geometry_msgs::Polygon p;
    geometry_msgs::Point32 p32;
    p32.x = it->poses[0].pose.position.x;
    p32.y = it->poses[0].pose.position.y;
    p32.z = it->poses[0].pose.position.z;
//    if (! isInsideVoxel(p32, g_ocTreeInspectionVoxels[g_inspectionVolumeIndex])) continue;
    p.points.push_back(p32);
    p32.x = it->poses[1].pose.position.x;
    p32.y = it->poses[1].pose.position.y;
    p32.z = it->poses[1].pose.position.z;
//    if (! isInsideVoxel(p32, g_ocTreeInspectionVoxels[g_inspectionVolumeIndex])) continue;
    p.points.push_back(p32);
    p32.x = it->poses[2].pose.position.x;
    p32.y = it->poses[2].pose.position.y;
    p32.z = it->poses[2].pose.position.z;
//    if (! isInsideVoxel(p32, g_ocTreeInspectionVoxels[g_inspectionVolumeIndex])) continue;
    p.points.push_back(p32);
    srv.request.inspectionArea.push_back(p);
    stl_pub.publish(*it);
    num_mesh_tri++;
    r.sleep();
  }
  ROS_INFO("No. of mesh triangles: %d", num_mesh_tri);

  /* define obstacle regions as cuboids that are coordinate system aligned */
  int obstacle_id = 0;
  for (int v = 0; v < g_ocTreeInspectionVoxels.size(); ++v) {
    if (v != g_inspectionVolumeIndex) {
      for (std::vector<octomap::OcTreeVolume>::iterator ito = g_ocTreeInspectionObstacles->at(v).begin();
           ito != g_ocTreeInspectionObstacles->at(v).end(); ++ito) {
        shape_msgs::SolidPrimitive body;
        body.type = shape_msgs::SolidPrimitive::BOX;
        body.dimensions.push_back(ito->second);
        body.dimensions.push_back(ito->second);
        body.dimensions.push_back(ito->second);
        srv.request.obstacles.push_back(body);
        geometry_msgs::Pose pose;
        pose.position.x = ito->first.x();
        pose.position.y = ito->first.y();
        pose.position.z = ito->first.z();
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
//        if (pose.position.z > 8.0 || abs(pose.position.x) > 1.0 || abs(pose.position.y) > 1.0) continue;
        srv.request.obstaclesPoses.push_back(pose);
        srv.request.obstacleIntransparancy.push_back(1);

        // publish obstacles for rviz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/kopt_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacles";
        marker.id = obstacle_id; // enumerate when adding more obstacles
        obstacle_id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pose.position.x;
        marker.pose.position.y = pose.position.y;
        marker.pose.position.z = pose.position.z;
        marker.pose.orientation.x = pose.orientation.x;
        marker.pose.orientation.y = pose.orientation.y;
        marker.pose.orientation.z = pose.orientation.z;
        marker.pose.orientation.w = pose.orientation.w;

        marker.scale.x = body.dimensions[0];
        marker.scale.y = body.dimensions[1];
        marker.scale.z = body.dimensions[2];

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.2;

        marker.lifetime = ros::Duration();
        obstacle_pub.publish(marker);
        r1.sleep();
      }
    }
  }

  if (client.call(srv))
  {
    // Smoothed headings
    // Evaluate offset directions
    std::vector<double> offset_headings, offset_headings_smooth, v_xval;
    int h_id = 1;
    for (std::vector<geometry_msgs::PoseStamped>::iterator it = srv.response.inspectionPath.poses.begin();
        it != srv.response.inspectionPath.poses.end(); it++, ++h_id) {
      tf::Pose pose;
      tf::poseMsgToTF(it->pose, pose);
      double heading = tf::getYaw(it->pose.orientation);
    //    std::cout << heading << std::endl;
      offset_headings.push_back(heading);
      v_xval.push_back(h_id);
    }
    std::vector<double> tmp1(offset_headings.size());
    std::vector<double> tmp2(offset_headings.size());
    std::vector<double> offset_headings_smooth_low(offset_headings.size());
    g_dlowess.lowess(v_xval, offset_headings, 0.25, 2, 0.0, offset_headings_smooth_low, tmp1, tmp2);
    // Print
//    std::cout << "raw headings: " << std::endl;
//    for (auto h : offset_headings) {
//      std::cout << h << ", ";
//    }
//    std::cout << std::endl;
//    std::cout << "smoothed headings: " << std::endl;
//    for (auto h : offset_headings_smooth_low) {
//      std::cout << h << ", ";
//    }
//    std::cout << std::endl;


    /* writing results of scenario to m-file. use supplied script to visualize */
    std::fstream pathPublication, mavPathPublication;
    std::string pkgPath = ros::package::getPath("request");
    pathPublication.open((pkgPath+"/visualization/inspectionScenario.m").c_str(), std::ios::out);
    mavPathPublication.open((pkgPath+"/visualization/mavPath.txt").c_str(), std::ios::out);
    Eigen::Vector3d last_mav_p(g_mavStartPos);
    double last_mav_yaw;
    double mav_time = 0;
    int mav_wayp_no = 0;
    if(!pathPublication.is_open())
    {
      ROS_ERROR("Could not open 'inspectionScenario.m'! Inspection path is not written to file");
      return 1;
    }
    if(!mavPathPublication.is_open())
    {
      ROS_ERROR("Could not open 'mavPath.txt'! Mav path is not written to file");
      return 1;
    }
    pathPublication << "inspectionPath = [";
    for(std::pair<std::vector<geometry_msgs::PoseStamped>::iterator, std::vector<double>::iterator>
        it(srv.response.inspectionPath.poses.begin(), offset_headings_smooth_low.begin());
        it.first != srv.response.inspectionPath.poses.end(); ++it.first, ++it.second)
    {
      double yaw_angle = *it.second;
      pathPublication << it.first->pose.position.x << ", " << it.first->pose.position.y << ", " << it.first->pose.position.z << ", 0, 0, " << yaw_angle << ";\n";

      Eigen::Vector3d mav_p(it.first->pose.position.x, it.first->pose.position.y, it.first->pose.position.z);
      double mav_yaw = yaw_angle;
      // time estimation
      if (mav_wayp_no != 0) {
//        std::cout << "g_maxSpeed: " << g_maxSpeed << std::endl;
        double tp = (mav_p - last_mav_p).norm() / g_maxSpeed;
        std::cout << "tp: " << tp << std::endl;
//        std::cout << "g_maxAngularSpeed: " << g_maxAngularSpeed << std::endl;
        double ty = std::abs(mav_yaw - last_mav_yaw) / g_maxAngularSpeed;
        std::cout << "ty: " << ty << std::endl;
        if (std::max(tp, ty) <= 0.01) continue;
        mav_time += std::max(tp, ty) * 2;
      }
      mav_wayp_no++;
      last_mav_p = mav_p;
      last_mav_yaw = mav_yaw;

      std::cout << mav_wayp_no << " " << mav_time << " " << mav_p[0] << " " << mav_p[1] << " " << mav_p[2] << " " << mav_yaw * 180/M_PI << "\n";
      mavPathPublication << mav_time << " " << mav_p[0] << " " << mav_p[1] << " " << mav_p[2] << " " << mav_yaw * 180/M_PI << "\n";
    }
    pathPublication << "];\n";
    pathPublication << "inspectionCost = " << srv.response.cost << ";\n";
    pathPublication << "numObstacles = " << std::min(srv.request.obstacles.size(),srv.request.obstaclesPoses.size()) << ";\n";
    for(int i = 0; i<std::min(srv.request.obstacles.size(),srv.request.obstaclesPoses.size()); i++)
    {
      pathPublication << "obstacle{" << i+1 << "} = [" << srv.request.obstacles[i].dimensions[0] << ", " << srv.request.obstacles[i].dimensions[1] << ", " << srv.request.obstacles[i].dimensions[2] << ";\n";
      pathPublication << srv.request.obstaclesPoses[i].position.x << ", " << srv.request.obstaclesPoses[i].position.y << ", " << srv.request.obstaclesPoses[i].position.z << "];\n";
    }
    pathPublication << "meshX = [";
    for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end() && ros::ok(); it++)
    {
      pathPublication << it->poses[0].pose.position.x << ", " << it->poses[1].pose.position.x << ", " << it->poses[2].pose.position.x << ";\n";
    }
    pathPublication << "];\nmeshY = [";
    for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end() && ros::ok(); it++)
    {
      pathPublication << it->poses[0].pose.position.y << ", " << it->poses[1].pose.position.y << ", " << it->poses[2].pose.position.y << ";\n";
    }
    pathPublication << "];\nmeshZ = [";
    for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end() && ros::ok(); it++)
    {
      pathPublication << it->poses[0].pose.position.z << ", " << it->poses[1].pose.position.z << ", " << it->poses[2].pose.position.z << ";\n";
    }
    pathPublication << "];\n";
  }
  else
  {
    ROS_ERROR("Failed to call service planner");
    return 1;
  }

  return 0;
}

/**
*  \brief This function returns true if a point is within an Octree voxel
*/
bool isInsideVoxel(const octomap::point3d& p, const octomap::OcTreeVolume v) {
  bool res = true;
  res = res & (p.x() <= v.first.x() + v.second/2.0);
  res = res & (p.x() >= v.first.x() - v.second/2.0);

  res = res & (p.y() <= v.first.y() + v.second/2.0);
  res = res & (p.y() >= v.first.y() - v.second/2.0);

  res = res & (p.z() <= v.first.z() + v.second/2.0);
  res = res & (p.z() >= v.first.z() - v.second/2.0);

  return res;
}
/**
*  \brief This function returns true if a point is within an Octree voxel
*/
bool isInsideVoxel(const geometry_msgs::Point32& p, const octomap::OcTreeVolume v) {
  bool res = true;
  res = res & (p.x <= v.first.x() + v.second/2.0);
  res = res & (p.x >= v.first.x() - v.second/2.0);

  res = res & (p.y <= v.first.y() + v.second/2.0);
  res = res & (p.y >= v.first.y() - v.second/2.0);

  res = res & (p.z <= v.first.z() + v.second/2.0);
  res = res & (p.z >= v.first.z() - v.second/2.0);

  return res;
}

/**
*  \brief This function reads an ACII STL file for inspection planning
*/
std::vector<nav_msgs::Path> * readSTLfile(std::string name)
{
  std::vector<nav_msgs::Path> * mesh = new std::vector<nav_msgs::Path>;
  std::fstream f;
  f.open(name.c_str());
  assert(f.is_open());
  int MaxLine = 0;
  char* line;
  double maxX = -DBL_MAX;
  double maxY = -DBL_MAX;
  double maxZ = -DBL_MAX;
  double minX = DBL_MAX;
  double minY = DBL_MAX;
  double minZ = DBL_MAX;
  assert(line = (char *) malloc(MaxLine = 80));
  f.getline(line, MaxLine);
  if(0 != strcmp(strtok(line, " "), "solid"))
  {
    ROS_ERROR("Invalid mesh file! Make sure the file is given in ascii-format.");
    ros::shutdown();
  }
  assert(line = (char *) realloc(line, MaxLine));
  f.getline(line, MaxLine);
  int k = 0;
  while(0 != strcmp(strtok(line, " "), "endsolid") && !ros::isShuttingDown())
  {
    int q = 0;
    nav_msgs::Path p;
    geometry_msgs::PoseStamped v1;
    for(int i = 0; i<7; i++)
    {
      while(line[q] == ' ')
        q++;
      if(line[q] == 'v')
      {
        const double yawTrafo = 0.0;      // used to rotate the mesh before processing
        const double scaleFactor = 1.0;   // used to scale the mesh before processing
        const double offsetX = 0.0;       // used to offset the mesh before processing
        const double offsetY = 0.0;       // used to offset the mesh before processing
        const double offsetZ = 0.0;       // used to offset the mesh before processing

        geometry_msgs::PoseStamped vert;
        char* v = strtok(line+q," ");
        v = strtok(NULL," ");
        double xtmp = atof(v)/scaleFactor;
        v = strtok(NULL," ");
        double ytmp = atof(v)/scaleFactor;
        vert.pose.position.x = cos(yawTrafo)*xtmp-sin(yawTrafo)*ytmp;
        vert.pose.position.y =  sin(yawTrafo)*xtmp+cos(yawTrafo)*ytmp;
        v = strtok(NULL," ");
        vert.pose.position.z =  atof(v)/scaleFactor;
        vert.pose.position.x -= offsetX;
        vert.pose.position.y -= offsetY;
        vert.pose.position.z -= offsetZ;
        if(maxX<vert.pose.position.x)
          maxX=vert.pose.position.x;
        if(maxY<vert.pose.position.y)
          maxY=vert.pose.position.y;
        if(maxZ<vert.pose.position.z)
          maxZ=vert.pose.position.z;
        if(minX>vert.pose.position.x)
          minX=vert.pose.position.x;
        if(minY>vert.pose.position.y)
          minY=vert.pose.position.y;
        if(minZ>vert.pose.position.z)
          minZ=vert.pose.position.z;
        vert.pose.orientation.x =  0.0;
        vert.pose.orientation.y =  0.0;
        vert.pose.orientation.z =  0.0;
        vert.pose.orientation.w =  1.0;
        p.poses.push_back(vert);
        if(p.poses.size() == 1)
          v1 = vert;
      }
      assert(line = (char *) realloc(line, MaxLine));
      f.getline(line, MaxLine);
    }
    p.poses.push_back(v1);
    p.header.frame_id = "/kopt_frame";
    p.header.stamp = ros::Time::now();
    p.header.seq = k;
    mesh->push_back(p);
    k++;
  }
  free(line);
  f.close();
  ROS_INFO("Mesh area is bounded by: [%2.2f,%2.2f]x[%2.2f,%2.2f]x[%2.2f,%2.2f]", minX,maxX,minY,maxY,minZ,maxZ);
  return mesh;
}
