#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <cv.h>
#include <highgui.h>
using namespace std;
extern int tx,ty,tz,rot,rot1;
class PointCloudOpt
{
public:
  cv::Mat RaboutX(const double& theta);//rotation about X axis
  cv::Mat RaboutY(const double& theta);//rotation about Y axis
  cv::Mat RaboutZ(const double& theta);//rotation about Z axis
  pcl::PointCloud<pcl::PointXYZ> pointCloudOrigin(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput);
  pcl::PointCloud<pcl::PointXYZ> pointCloudTranslation(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput);
  pcl::PointCloud<pcl::PointXYZ> pointCloudRot(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput);
  pcl::PointCloud<pcl::PointXYZ> pointCloudTransformation(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput);
  pcl::PointCloud<pcl::PointXYZ> linkageTransformation(const pcl::PointCloud<pcl::PointXYZ>& pointcloudinput);
};

inline 
cv::Mat PointCloudOpt::RaboutX(const double& theta)
{
	cv::Mat raboutx = (cv::Mat_<double>(3,3) << 1,         0,          0,
		0,cos(theta),-sin(theta),
		0,sin(theta),cos(theta));
	return raboutx;
}

inline
cv::Mat PointCloudOpt::RaboutY(const double& theta)
{
	cv::Mat rabouty = (cv::Mat_<double>(3,3) << cos(theta), 0, sin(theta),
		0,          1,          0,
		-sin(theta),0, cos(theta));

	return rabouty;
}

inline
cv::Mat PointCloudOpt::RaboutZ(const double& theta)
{
	cv::Mat raboutz = (cv::Mat_<double>(3,3) << cos(theta),-sin(theta),0,
		sin(theta), cos(theta),0,
		0,       0,            1);

	return raboutz;
}
