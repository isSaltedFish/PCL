#include "PointCloudOpt.h"
int tx,ty,tz,rot,rot1;
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
    pointcloudMat.at<float>(0,i)=pointcloudinput.at(i).x+tx;
    pointcloudMat.at<float>(1,i)=pointcloudinput.at(i).y+ty;
    pointcloudMat.at<float>(2,i)=pointcloudinput.at(i).z+tz;
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
    R1.at<float>(0,0)=cos(rot);
    R1.at<float>(0,1)=-sin(rot);
    R1.at<float>(0,2)=0;
    R1.at<float>(1,0)=sin(rot);
    R1.at<float>(1,1)=cos(rot);
    R1.at<float>(1,2)=0;
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
 // cout<<"Please input the translation amount:"<<endl;
 // int tx,ty,tz;
 // cout<<"tx=";
 // cin>>tx;
 // cout<<"ty=";
 // cin>>ty;
 // cout<<"tz=";
 // cin>>tz;

  cv::Mat pointcloudMat = cv::Mat(4,pointcloudinput.size(),CV_32F);
  for(int i=0;i<pointcloudinput.size();++i)
  {
    pointcloudMat.at<float>(0,i)=pointcloudinput.at(i).x;
    pointcloudMat.at<float>(1,i)=pointcloudinput.at(i).y;
    pointcloudMat.at<float>(2,i)=pointcloudinput.at(i).z;
    pointcloudMat.at<float>(3,i)=1;
  }

  //cout<<"Please input the Rot with the z axis amount:"<<endl;
  //int rot;
  //cin>>rot;
    cv::Mat R1= cv::Mat(4,4,CV_32F);
    R1.at<float>(0,0)=cos(rot);
    R1.at<float>(0,1)=-sin(rot);
    R1.at<float>(0,2)=0;
    R1.at<float>(0,3)=tx;
    R1.at<float>(1,0)=sin(rot);
    R1.at<float>(1,1)=cos(rot);
    R1.at<float>(1,2)=0;
    R1.at<float>(1,3)=ty;
    R1.at<float>(2,0)=0;
    R1.at<float>(2,1)=0;
    R1.at<float>(2,2)=1;
    R1.at<float>(2,3)=tz;
    R1.at<float>(3,0)=0;
    R1.at<float>(3,1)=0;
    R1.at<float>(3,2)=0;
    R1.at<float>(3,3)=1;

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

pcl::PointCloud<pcl::PointXYZ> PointCloudOpt::linkageTransformation(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput )
{
  pcl::PointCloud<pcl::PointXYZ> pointcloudoutput;
  
  /*
   * please finish this program to implement 3D points linkage Transformation.
   * input: pointcloudinput
   * output: pointcloudoutput
   * description: calculate pointcloudoutput by Transformating linkage point cloud
   */
  cv::Mat l1 = cv::Mat(3,pointcloudinput.size(),CV_32F);
  for(int i=0;i<pointcloudinput.size();++i)
  {
    l1.at<float>(0,i)=pointcloudinput.at(i).x;
    l1.at<float>(1,i)=pointcloudinput.at(i).y;
    l1.at<float>(2,i)=pointcloudinput.at(i).z;
  }

  cv::Mat R1=cv::Mat(3,3,CV_32F);
      R1.at<float>(0,0)=cos(rot);
      R1.at<float>(0,1)=-sin(rot);
      R1.at<float>(0,2)=0;
      R1.at<float>(1,0)=sin(rot);
      R1.at<float>(1,1)=cos(rot);
      R1.at<float>(1,2)=0;
      R1.at<float>(2,0)=0;
      R1.at<float>(2,1)=0;
      R1.at<float>(2,2)=1;
    l1=R1*l1;

  cv::Mat l2 = cv::Mat(3,pointcloudinput.size(),CV_32F);
  for(int i=0;i<pointcloudinput.size();++i)
  {
    l2.at<float>(0,i)=pointcloudinput.at(i).x;
    l2.at<float>(1,i)=pointcloudinput.at(i).y;
    l2.at<float>(2,i)=pointcloudinput.at(i).z;
  }
  cv::Mat R2=cv::Mat(3,3,CV_32F);
      R2.at<float>(0,0)=cos(rot1);
      R2.at<float>(0,1)=-sin(rot1);
      R2.at<float>(0,2)=0;
      R2.at<float>(1,0)=sin(rot1);
      R2.at<float>(1,1)=cos(rot1);
      R2.at<float>(1,2)=0;
      R2.at<float>(2,0)=0;
      R2.at<float>(2,1)=0;
      R2.at<float>(2,2)=1;
    l2=R2*l2;

    pcl::PointCloud<pcl::PointXYZ> pointcloudl1,pointcloudl2;
    pointcloudl1.resize(l1.cols);
    pointcloudl2.resize(l2.cols);

    for(int i=0;i<l1.cols;++i)
    {
      pointcloudl1.at(i).x=l1.at<float>(0,i);
     pointcloudl1.at(i).y=l1.at<float>(1,i);
     pointcloudl1.at(i).z=l1.at<float>(2,i);
    pointcloudoutput.push_back(pointcloudl1.at(i));
    }

    for(int i=0;i<l2.cols;++i)
    {
      pointcloudl2.at(i).x=l2.at<float>(0,i);
     pointcloudl2.at(i).y=l2.at<float>(1,i);
     pointcloudl2.at(i).z=l2.at<float>(2,i);
    pointcloudoutput.push_back(pointcloudl2.at(i));
    }
  
  
  
  return  pointcloudoutput;
}
