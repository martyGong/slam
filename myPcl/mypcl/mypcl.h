#ifndef MY_PCL_H_H_H
#define MY_PCL_H_H_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <vector>

using namespace std;
typedef   pcl::PointCloud<pcl::PointXYZ>  PointCloudType ;
typedef   pcl::PointXYZ  PointType ;

typedef  struct 
{
    float neigborPointDis;
    float neigborPointAgl;
    float pointDis;
    float pointsNum;
    float dockLen;
}DockSpec;


typedef struct 
{
   PointCloudType cluster;
   PointType startPoint;
   PointType endPoint;
   int size;
} Cluster;

class MYDOCKFIT
{
   public:
    MYDOCKFIT();
    ~MYDOCKFIT();

    bool DockFitUsingICP(const PointCloudType &Point_in,PointCloudType &Point_out);
    bool DockFitUsingNDT(const PointCloudType &Point_in,PointCloudType &Point_out);
    bool GetDock();
    bool GetCluster();
    bool FilterCluster();
    inline float GetPointDistance(PointType     a,PointType b)
    {
         return std::hypot((a.x - b.x), (a.y - b.y));
    }
    bool LoadTargetPointCloud();
    bool getTargetPointCloud();
    bool GetSourcePointCloud();
    bool GetSourcePointCloud(const sensor_msgs::LaserScan::ConstPtr& scan_ptr);
    bool getSourePointCloudFromTest();
    bool getTargetPointCloud160();
    bool GetBreakPointIndex(const PointCloudType &pointCloud,const int start_index,const float distance, int &breakPoint_index);
    bool CheckBreakPointIndex(const PointCloudType &pointCloud,const int start_index,const float distance);

    private:
        int iter;
        float err;
        PointCloudType point_target;
        PointCloudType point_origin;
        vector<PointCloudType>clusterVector;
        DockSpec dockSpec;
        
};
#endif
