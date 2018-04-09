#include "PointCloudOpt.h"

int main(int argc, char ** argv)
{    
    if(argc < 2)
    {
      cout << "input arguments number not enough, System EXIT!!" << endl;
      exit(1);
    }
    //define pointcloud
    pcl::PointCloud<pcl::PointXYZ> pointcloudORI;
    pcl::PointCloud<pcl::PointXYZ> pointcloudMet;
    pcl::PointCloud<pcl::PointXYZ> pointcloudMet1;
    pcl::PointCloud<pcl::PointXYZ> pointcloudMet2;
    pcl::PointCloud<pcl::PointXYZ> pointcloudMet3;
    pcl::PointCloud<pcl::PointXYZ> pointcloudMet4;
    
    //read 3D points from *.ply file to pointcloud
    pcl::PLYReader reader;    
    
    //new instance of PointCloudOpt
    PointCloudOpt pco;
    
    //point cloud operation.   YOU NEED to finish them experimental classes.
    if(atoi(argv[1])==1)
    {
      tx=atoi(argv[2]);
      ty=atoi(argv[3]);
      tz=atoi(argv[4]);
      reader.read ("bunny_10.ply", pointcloudORI);
      pointcloudMet1 = pco.pointCloudTranslation(pointcloudORI);
    }
    else if(atoi(argv[1])==2)
    {
      float tmp;
      tmp=atoi(argv[2]);
      rot=tmp/180*3.14;
      reader.read ("bunny_10.ply", pointcloudORI);
      pointcloudMet2 = pco.pointCloudRot(pointcloudORI);
    }
    else if(atoi(argv[1])==3)
    {
      tx=atoi(argv[2]);
      ty=atoi(argv[3]);
      tz=atoi(argv[4]);
      float tmp;
      tmp=atoi(argv[5]);
      rot=tmp/180*3.14;
      reader.read ("bunny_10.ply", pointcloudORI);
      pointcloudMet3 = pco.pointCloudTransformation(pointcloudORI);
    }
    else if(atoi(argv[1])==4)
    {
      float tmp;
      tmp=atoi(argv[2]);
      rot=tmp/180*3.14;
      tmp=atoi(argv[3]);
      rot1=tmp/180*3.14;
      reader.read ("partation.ply", pointcloudORI);
      pointcloudMet4 = pco.linkageTransformation(pointcloudORI);
    }
    else
    {
      cout << "your input num is incorrect!!\nSystem exit!!" << endl;
      exit(1);
    }   
    
    //point cloud visualization
    pcl::visualization::PCLVisualizer viewer1("pcl");  
    viewer1.setBackgroundColor(0.3, 0.3, 0.3);
    reader.read ("bunny_10.ply", pointcloudORI);
    pointcloudMet = pco.pointCloudOrigin(pointcloudORI);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(pointcloudMet.makeShared(), 255, 0, 0);
    if(atoi(argv[1])==1)
    {
      viewer1.addCoordinateSystem(1);
      viewer1.addPointCloud (pointcloudMet.makeShared(), single_color1, "pointcloudMet");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(pointcloudMet1.makeShared(), 0, 255, 0);
      viewer1.addPointCloud (pointcloudMet1.makeShared(), single_color1, "pointcloudMet1");
    }
    else if(atoi(argv[1])==2)
    {
      viewer1.addCoordinateSystem(1);
      viewer1.addPointCloud (pointcloudMet.makeShared(), single_color1, "pointcloudMet");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(pointcloudMet2.makeShared(), 0, 255, 0);
      viewer1.addPointCloud (pointcloudMet2.makeShared(), single_color1, "pointcloudMet2");
    }
    else if(atoi(argv[1])==3)
    {
      viewer1.addCoordinateSystem(1);
      viewer1.addPointCloud (pointcloudMet.makeShared(), single_color1, "pointcloudMet");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(pointcloudMet3.makeShared(), 0, 255, 0);
      viewer1.addPointCloud (pointcloudMet3.makeShared(), single_color1, "pointcloudMet3");
    }
    else if(atoi(argv[1])==4)
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(pointcloudMet4.makeShared(),0, 255, 0);
      viewer1.addPointCloud (pointcloudMet4.makeShared(), single_color1, "pointcloudMet4");
    }
    else
    {
      cout << "your input num is incorrect!!\nSystem exit!!" << endl;
      exit(1);
    }   

    while (!viewer1.wasStopped ())
    {
      viewer1.spinOnce ();
    }
}
