#include "mypcl.h"


MYDOCKFIT::MYDOCKFIT()
{

}


MYDOCKFIT::~MYDOCKFIT()
{

}

bool MYDOCKFIT::LoadTargetPointCloud()
{
    PointCloudType temp;
    if(-1 == pcl::io::loadPCDFile<pcl::PointXYZ>("/project/uv/myPcl/rabbit.pcd",temp))
    {
        return false;
    }
    std::cout<<"size is " << temp.size()<<endl;

    int i = 0;
    for(auto & point :  temp)
    {
        if(i++%36== 0)
        point_target.push_back(point);

    }
std::cout<<"size is " << point_target.size()<<endl;
    return true;
}


bool MYDOCKFIT::getTargetPointCloud160()
{
    float l = 0.31;
    float theta = (160*M_PI)/180;
    PointType point;
    PointType point_reverse;
    for(int i = 0; i<600;i++)
    {
       point.x = i*l/(2*600);
       point.y = point.x * atan(M_PI/2 - theta/2);
       point.z = 0;

       point_reverse.x = -point.x;
       point_reverse.y = point.y;
       point_reverse.z = 0;
       point_target.push_back(point);
       point_target.push_back(point_reverse);
       
    }
}

bool MYDOCKFIT::getTargetPointCloud()
{
    float R = 0.27;
    float l = 0.31;
    float theta = asin(l/(2*R));
    PointType point;
    PointType point_reverse;
    for(int i = 0; i <600; i++)
    {
        point.x = R*cos(M_PI/2 - (theta/2- theta*i/600));
        point.y = R*sin(M_PI/2 - (theta/2- theta*i/600));
        point.z = 0;


       point_reverse.x = -point.x;
       point_reverse.y = point.y;
       point_reverse.z = 0;
       point_target.push_back(point);
       point_target.push_back(point_reverse);
    }
}
bool MYDOCKFIT::getSourePointCloudFromTest()
{
    PointType point;
    PointType point_reverse;
    float theta = (160*M_PI)/180;
    float l = 0.31;
    for(int i = 0 ; i <600;i++)
    {
        point.x = i*l/(2*600);
        //point.y = 0.5+point.x * atan(M_PI/2 - theta/2);
        point.y = 0;
        point.z = 0;

        point_reverse.x = -point.x;
        point_reverse.y = point.y;
        point_reverse.z = 0;
        point_origin.push_back(point);
        point_origin.push_back(point_reverse);
        
    }
}
bool MYDOCKFIT::GetSourcePointCloud()
{

     // 定义一个旋转矩阵 
  float theta = M_PI/3; // 弧度角
 
  /* 使用 Affine3f
    这种方法简单，不易出错
  */
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  // 在 X 轴上定义一个 2.5 米的平移.
  transform_2.translation() << 0.5, 0.0, 0.0;
 
  // 和前面一样的旋转; Z 轴上旋转 theta 弧度
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
 
  // 打印变换矩阵
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  pcl::transformPointCloud (point_target,point_origin, transform_2);

  

         pcl::visualization::PCLVisualizer viewer ("Matrix transformation ");
   // 为点云定义 R,G,B 颜色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_target_color_handler (point_target.makeShared(), 255, 255, 255);

  // 输出点云到查看器，使用颜色管理
  viewer.addPointCloud (point_target.makeShared(), point_target_color_handler, "target_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> origin_cloud_color_handler (point_origin.makeShared(), 230, 20, 20); // 红
  viewer.addPointCloud (point_origin.makeShared(), origin_cloud_color_handler, "origin_cloud");
 


  //viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // 设置背景为深灰
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "origin_cloud");

  //viewer.setPosition(800, 400); // 设置窗口位置
 
  while (!viewer.wasStopped ()) { // 在按下 "q" 键之前一直会显示窗口
    viewer.spinOnce ();
  }
  

  return true;

}


bool MYDOCKFIT::GetSourcePointCloud(const sensor_msgs::LaserScan::ConstPtr& scan_ptr)
{
    point_origin.clear();
    float start_angle  = scan_ptr->angle_min;
    for (int i = 0; i < scan_ptr->ranges.size(); i++)
    {
        if (std::isfinite(scan_ptr->ranges[i])&& scan_ptr->ranges[i] <= dockSpec.pointDis)
        {
            PointType point;
            float point_angle =  start_angle+ i*scan_ptr->angle_increment;
            point.x = scan_ptr->ranges[i] * cos(point_angle);
            point.y = scan_ptr->ranges[i] * sin(point_angle);
            point.z = 0;
            point_origin.push_back(point);
        }
    }
}


bool MYDOCKFIT::GetBreakPointIndex(const PointCloudType &pointCloud,const int start_index,const float distance, int &breakPoint_index)
{
    int max_size = pointCloud.points.size();
    int i = 0;
    for(i = start_index; i <= max_size-1; i++)
    {
        if(i+1 < max_size)
        {
            if(GetPointDistance(pointCloud.points[i],pointCloud.points[i+1]) > distance) 
            {
                int count = 0;
                for (int j = i+2; j < i+5; ++ j)
                {
                   if(j < max_size)
                   {
                       if(GetPointDistance(pointCloud.points[i],pointCloud.points[j]) <= distance)
                       {
                            break;
                       }
                       count++;
                       
                   }
                }
                if (count >= 3 || (i+count+2 == max_size))
                {
                    breakPoint_index = i;
                    return true;
                }
            }
        }
    }
    if(max_size-1 == i)
    {
        return false;
    }
    return true;
}


bool MYDOCKFIT::CheckBreakPointIndex(const PointCloudType &pointCloud,const int start_index,const float distance)
{
    int max_size = pointCloud.points.size();

    if(start_index+1 >= max_size) return false;

    if(GetPointDistance(pointCloud.points[start_index],pointCloud.points[start_index+1]) > distance)
    {
         int count = 0;
         for (int j = start_index+2; j < start_index+5; ++ j)
         {
            if(j < max_size)
            {
                if(GetPointDistance(pointCloud.points[start_index],pointCloud.points[j]) <= distance)
                {
                     break;
                }
                count++;
                
            }
         }
         if (count >= 3 || (start_index+count+2 == max_size))
         {
             return true;
         }
    }
    else
    {
        return false;
    }
}

bool MYDOCKFIT::GetCluster()
{
    vector<int> indexVec;
    int index = 0;
    int max_point_size = point_origin.points.size();
    //find the first break point   modified by marty.gong:2021-3-01-15:38:25 
    if(false == GetBreakPointIndex(point_origin,0,dockSpec.neigborPointDis,index))
    {
         //所有点都在一个cluster中   modified by marty.gong:2021-3-01-16:59:38 
         clusterVector.push_back(point_origin);
         return true;
    }

   for(int i = index+1;i <= max_point_size-1;i++)
   {
        indexVec.push_back(i);

        if(true == CheckBreakPointIndex(point_origin,i,dockSpec.neigborPointDis))
        {
             PointCloudType pointCuster;
             pcl::copyPointCloud(point_origin, indexVec,pointCuster);
             clusterVector.push_back(pointCuster);
             indexVec.clear();
        }
   }

   //判断后面的点是否和一开始的点相邻   modified by marty.gong:2021-3-02-15:28:10 
   if(!indexVec.empty())
   {
       if(GetPointDistance(point_origin.points[max_point_size-1],point_origin.points[0]) > dockSpec.neigborPointDis)
       {
            PointCloudType pointCuster;
            pcl::copyPointCloud(point_origin, indexVec,pointCuster);
            clusterVector.push_back(pointCuster);
            indexVec.clear(); 
       }
   }
   else
   {

        //将最后一个cluster补充完整    modified by marty.gong:2021-3-01-17:4:49 
       for(int i = 0; i < index+1;i++)
       {
          indexVec.push_back(i);
       }
       PointCloudType pointCuster;
       pcl::copyPointCloud(point_origin, indexVec,pointCuster);
       clusterVector.push_back(pointCuster);
   }

   return true;
}



bool MYDOCKFIT::FilterCluster()
{
    vector<PointCloudType> tempVector; 
    for(auto &pointCloud_ :clusterVector)
    {

        float pointCloudLen =  
        GetPointDistance(pointCloud_.points[0],pointCloud_.points[pointCloud_.points.size()-1]);

        //长度太短丢弃   modified by marty.gong:2021-3-02-10:59:58 
        if(pointCloud_.points.size() < dockSpec.pointsNum || pointCloudLen < 0.8*dockSpec.dockLen)
        {
            continue;
        }

        // 长度太长，拆分2个Cluster   modified by marty.gong:2021-3-02-11:0:59 
        if(pointCloudLen > 1.5*dockSpec.dockLen) 
        {
            PointCloudType pointCusterTemp;
            pcl::copyPointCloud(pointCloud_,pointCusterTemp);

            for(int i = pointCloud_.points.size()-1; i >= 0 ; i--)
            {
                if(GetPointDistance(pointCusterTemp.points[0],pointCusterTemp.points[pointCusterTemp.points.size()-1])<= 1.2*dockSpec.dockLen)
                {
                     tempVector.push_back(pointCusterTemp);
                }
                pointCusterTemp.erase(pointCusterTemp.begin()+pointCusterTemp.points.size()-1);
            }

            pcl::copyPointCloud(pointCloud_,pointCusterTemp);
            for(int i =  0; i <= pointCloud_.points.size()-1;i++)
            {
                 if(GetPointDistance(pointCusterTemp.points[0],pointCusterTemp.points[pointCusterTemp.points.size()-1])<= 1.2*dockSpec.dockLen)
                {
                     tempVector.push_back(pointCusterTemp);
                }
                pointCusterTemp.erase(pointCusterTemp.begin());
            }
        
         }else
        {
            tempVector.push_back(pointCloud_);
        }
        
    }
    clusterVector.clear();
    clusterVector = tempVector;
    return true;
    
}


//source 激光雷达扫描的点   ;target：构造的点云
bool MYDOCKFIT::DockFitUsingICP(const PointCloudType &Point_in,PointCloudType &Point_out)
{
   int target_num = point_target.size();
   int source_num = Point_in.size();


   //点数太小   modified by marty.gong:2021-2-22-14:7:30 
   if(5 > source_num)
   {
    return false;
   }

   pcl::IterativeClosestPoint<PointType, PointType> icp;
/*
   pcl::KdTreeFLANN<pcl::PointXYZ>tree_source;
   pcl::KdTreeFLANN<pcl::PointXYZ> tree_target;
   tree_source.setInputCloud(Point_in.makeShared());
   tree_target.setInputCloud(point_target.makeShared());
   //过滤最大距离点   modified by marty.gong:2021-2-22-14:20:15 

   icp.setSearchMethodSource(&tree_source);
   icp.setSearchMethodTarget(&tree_target);
   */
   icp.setMaxCorrespondenceDistance(0.6);
   icp.setTransformationEpsilon(1e-10);
   icp.setEuclideanFitnessEpsilon(0.0001);
   icp.setMaximumIterations (1000);
   
   icp.setInputSource(Point_in.makeShared());
   icp.setInputTarget(point_target.makeShared());
      
   PointCloudType Final;
   icp.align(Final);

   if(true == icp.hasConverged())
   {
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        Point_out  = Final;
        PointCloudType final_point;
        
        std::cout << "score: " <<icp.getFitnessScore() << std::endl; 
        std::cout <<"final transform is :"<<endl;
        std::cout<<transformation<<Final;
        std::cout << "final.size()"<<Final.size()<<endl;
        pcl::transformPointCloud (point_origin,final_point, transformation);
        pcl::visualization::PCLVisualizer viewer ("Matrix transformation ");
   // 为点云定义 R,G,B 颜色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_target_color_handler (point_target.makeShared(), 255, 255, 255);

  // 输出点云到查看器，使用颜色管理
  viewer.addPointCloud (point_target.makeShared(), point_target_color_handler, "target_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> origin_cloud_color_handler (point_origin.makeShared(), 230, 20, 20); // 红
  viewer.addPointCloud (point_origin.makeShared(), origin_cloud_color_handler, "origin_cloud");
 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_final_color_handler (Final.makeShared(), 162, 167, 63);
     viewer.addPointCloud (Final.makeShared(), point_final_color_handler, "final_cloud");

  //viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // 设置背景为深灰
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "origin_cloud");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "final_cloud");
  //viewer.setPosition(800, 400); // 设置窗口位置
 
  while (!viewer.wasStopped ()) { // 在按下 "q" 键之前一直会显示窗口
    viewer.spinOnce ();
  }


        return true;
   }
}



bool MYDOCKFIT::DockFitUsingNDT(const PointCloudType &Point_in,PointCloudType &Point_out)
{

    return true;
}
bool MYDOCKFIT::GetDock()
{
   PointCloudType point_out;
    return (DockFitUsingICP(point_origin,point_origin));

}

int main()
{
    MYDOCKFIT dockfit; 
    dockfit.getTargetPointCloud();
    dockfit.getSourePointCloudFromTest();
    dockfit.GetDock();
    return 0;
}




