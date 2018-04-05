#include "PointCloudOpt.h"

pcl::PointCloud<pcl::PointXYZ> PointCloudOpt::pointCloudOrigin(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput)
{
  pcl::PointCloud<pcl::PointXYZ> pointcloudoutput;

  cv::Mat pointcloudMat = cv::Mat(3,pointcloudinput.size(),CV_32F);
  for(int i=0;i<pointcloudinput.size();++i)
  {
    pointcloudMat.at<float>(0,i)=pointcloudinput.at(i).x+0.001;
    pointcloudMat.at<float>(1,i)=pointcloudinput.at(i).y+0.001;
    pointcloudMat.at<float>(2,i)=pointcloudinput.at(i).z+0.001;
  }

  pointcloudoutput.resize(pointcloudinput.size());
  for(int i=0;i<pointcloudMat.cols;++i)
  {
    pointcloudoutput.at(i).x=pointcloudMat.at<float>(0,i);
    pointcloudoutput.at(i).y=pointcloudMat.at<float>(1,i);
    pointcloudoutput.at(i).z=pointcloudMat.at<float>(2,i);
  }

  return  pointcloudoutput;
}

pcl::PointCloud<pcl::PointXYZ> PointCloudOpt::pointCloudTranslation(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput)
{
  pcl::PointCloud<pcl::PointXYZ> pointcloudoutput;
  
  /*
   * please finish this program to implement 3D points translation.
   * input: pointcloudinput
   * output: pointcloudoutput
   * description: calculate pointcloudoutput by adding translation vector to pointcloudoutput
   */
  
  cv::Mat pointcloudMat = cv::Mat(3,pointcloudinput.size(),CV_32F);
  for(int i=0;i<pointcloudinput.size();++i)
  {
    pointcloudMat.at<float>(0,i)=pointcloudinput.at(i).x+0.215;
    pointcloudMat.at<float>(1,i)=pointcloudinput.at(i).y+0.215;
    pointcloudMat.at<float>(2,i)=pointcloudinput.at(i).z+0.215;
  }

  pointcloudoutput.resize(pointcloudinput.size());
  for(int i=0;i<pointcloudMat.cols;++i)
  {
    pointcloudoutput.at(i).x=pointcloudMat.at<float>(0,i);
    pointcloudoutput.at(i).y=pointcloudMat.at<float>(1,i);
    pointcloudoutput.at(i).z=pointcloudMat.at<float>(2,i);
  }

  return  pointcloudoutput;
}

pcl::PointCloud<pcl::PointXYZ> PointCloudOpt::pointCloudRot(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput)
{
  pcl::PointCloud<pcl::PointXYZ> pointcloudoutput;
  /*
   * please finish this program to implement 3D points rotation.
   * input: pointcloudinput
   * output: pointcloudoutput
   * description: calculate pointcloudoutput by multipling rotation matrix to pointcloudoutput
   */
   cv::Mat pointcloudMat = cv::Mat(3,pointcloudinput.size(),CV_32F);
  for(int i=0;i<pointcloudinput.size();++i)
  {
    pointcloudMat.at<float>(0,i)=pointcloudinput.at(i).x;
    pointcloudMat.at<float>(1,i)=pointcloudinput.at(i).y;
    pointcloudMat.at<float>(2,i)=pointcloudinput.at(i).z;
  }

    cv::Mat R1= cv::Mat(3,3,CV_32F);
    R1.at<float>(0,0)=cos(3.14/6);
    R1.at<float>(0,1)=-sin(3.14/6);
    R1.at<float>(0,2)=0;
    R1.at<float>(1,0)=sin(3.14/6);
    R1.at<float>(1,1)=cos(3.14/6);
    R1.at<float>(2,1)=0;
    R1.at<float>(2,0)=0;
    R1.at<float>(2,1)=0;
    R1.at<float>(2,2)=1;
    pointcloudMat = R1*pointcloudMat;

  pointcloudoutput.resize(pointcloudinput.size());
  for(int i=0;i<pointcloudMat.cols;++i)
  {
    pointcloudoutput.at(i).x=pointcloudMat.at<float>(0,i);
    pointcloudoutput.at(i).y=pointcloudMat.at<float>(1,i);
    pointcloudoutput.at(i).z=pointcloudMat.at<float>(2,i);
  }
  
  
  
  
  
  
  return  pointcloudoutput;
}

pcl::PointCloud<pcl::PointXYZ> PointCloudOpt::pointCloudTransformation(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput)
{
  pcl::PointCloud<pcl::PointXYZ> pointcloudoutput;
  
  /*
   * please finish this program to implement 3D points Transformation.
   * input: pointcloudinput
   * output: pointcloudoutput
   * description: calculate pointcloudoutput by multipling Transformation matrix to pointcloudoutput
   */
  
  
  
  
  
  return  pointcloudoutput;
}

pcl::PointCloud<pcl::PointXYZ> PointCloudOpt::linkageTransformation(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput)
{
  pcl::PointCloud<pcl::PointXYZ> pointcloudoutput;
  
  /*
   * please finish this program to implement 3D points linkage Transformation.
   * input: pointcloudinput
   * output: pointcloudoutput
   * description: calculate pointcloudoutput by Transformating linkage point cloud
   */
  
  
  
  
  
  return  pointcloudoutput;
}
