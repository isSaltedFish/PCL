#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <cv.h>
#include <highgui.h>
#include "rply.h"
using namespace std;

typedef struct {
  double x;
  double y;
  double z;
} aPoint;

cv::Mat RaboutX(const float& theta)
{
	cv::Mat raboutx = (cv::Mat_<float>(3,3) << 1,         0,          0,
		0,cos(theta),-sin(theta),
		0,sin(theta),cos(theta));
	return raboutx;
}

cv::Mat RaboutY(const float& theta)
{
	cv::Mat rabouty = (cv::Mat_<float>(3,3) << cos(theta), 0, sin(theta),
		0,          1,          0,
		-sin(theta),0, cos(theta));

	return rabouty;
}

cv::Mat RaboutZ(const float& theta)
{
	cv::Mat raboutz = (cv::Mat_<float>(3,3) << cos(theta),-sin(theta),0,
		sin(theta), cos(theta),0,
		0,       0,            1);

	return raboutz;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr createCircle(float radiusupper, float z, int ite)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cv::Mat onepoint = (cv::Mat_<float>(3, 1) << 0, radiusupper, 0);
	cv::Mat newpoint;
	for(int i = 0; i <ite; ++i)
	{
		//cout << RaboutZ(3.1415926f*2.0f/ite*i) << endl;		
		newpoint = RaboutZ(3.1415926f*2.0f/ite*i) * onepoint;
		//cout << newpoint << endl;
		//cout << onepoint << endl;
		pcl::PointXYZ tmppoint;
		tmppoint.x = newpoint.at<float>(0, 0);
		tmppoint.y = newpoint.at<float>(1, 0);
		tmppoint.z = z;
		cloud->push_back(tmppoint);
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr createRect(float length, float width, int itel, int itew)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudnew(new pcl::PointCloud<pcl::PointXYZ>);
	cv::Mat onepoint = (cv::Mat_<float>(3, 1) << 0, width/2, 0);
	cv::Mat newpoint;
	for(int i = 0; i <=itew; ++i)
	{
		cv::Mat translation = (cv::Mat_<float>(3, 1) << 0, width/itew, 0);
		newpoint = onepoint - translation*i;
		//cout << newpoint << endl;
		//cout << onepoint << endl;
		pcl::PointXYZ tmppoint;
		tmppoint.x = newpoint.at<float>(0, 0);
		tmppoint.y = newpoint.at<float>(1, 0);
		tmppoint.z = newpoint.at<float>(2, 0);
		cloud->push_back(tmppoint);
		cloudnew->push_back(tmppoint);
	}
	
	for(int i = 0; i < cloud->size(); ++i)
	{
	  for(int j =1; j <= itel; j ++)
	  {
	    cloud->at(i).x += length/itel;
	    cloudnew->push_back(cloud->at(i));
	  }
	}
	
	return cloudnew;
}

static int vertex_cb(p_ply_argument argument)
{
    void *pdata;
    long indexCoord;
    
    ply_get_argument_user_data(argument, &pdata, &indexCoord);
    
    aPoint *points = *((aPoint**)pdata);
    
    long index;
    ply_get_argument_element(argument, NULL, &index);
    
    if (indexCoord == 1){
	points[index].x = ply_get_argument_value(argument);
    }
    if (indexCoord == 2){
	points[index].y = ply_get_argument_value(argument);
    }
    if (indexCoord == 3){
	points[index].z = ply_get_argument_value(argument);
    }
    
    return 1;
}
    
int readPointnum(aPoint* points, aPoint* normal, const char* input_ply)
{
    int pointnum = 0;
    
    p_ply ply = ply_open(input_ply, NULL);
    if (!ply) return 1;
    if (!ply_read_header(ply)) return 1;
    
    long nvertices = 
    ply_set_read_cb(ply, "vertex", "x", vertex_cb, &points, 1);
    pointnum = int(nvertices);
    
    points = new aPoint[pointnum];	
    normal = new aPoint[pointnum];	
    
    ply_close(ply);
    
    return pointnum;
}
    
int readPLY(aPoint* points, aPoint* normal,  aPoint* color,const char* input_ply)
{
    p_ply ply = ply_open(input_ply, NULL);
    if (!ply) return 1;
    if (!ply_read_header(ply)) return 1;
    
    ply_set_read_cb(ply, "vertex", "x", vertex_cb, &points, 1);
    ply_set_read_cb(ply, "vertex", "y", vertex_cb, &points, 2);
    ply_set_read_cb(ply, "vertex", "z", vertex_cb, &points, 3);
    ply_set_read_cb(ply, "vertex", "nx", vertex_cb, &normal, 1);
    ply_set_read_cb(ply, "vertex", "ny", vertex_cb, &normal, 2);
    ply_set_read_cb(ply, "vertex", "nz", vertex_cb, &normal, 3);
    ply_set_read_cb(ply, "vertex", "diffuse_red", vertex_cb, &color, 1);
    ply_set_read_cb(ply, "vertex", "diffuse_green", vertex_cb, &color, 2);
    ply_set_read_cb(ply, "vertex", "diffuse_blue", vertex_cb, &color, 3);
    
    if (!points || !normal|| !color) {
	cout<<"Error: new aPoint.\n"<<endl;
	if(!points) delete [] points;
	if(!normal) delete [] normal;
	if(!color) delete [] color;
	return 1;
    }
    
    if (!ply_read(ply)) return 1;  // read entire data at once
    ply_close(ply);
    
    return 0;
}

int savePLY(string plyname,pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)	
{
	int nvertices = pointCloud->size();
   // create a ply file to write data
	  p_ply oply; 
	  oply = ply_create(plyname.c_str(), PLY_ASCII, NULL);

	  // add this element to output file
	  if (!ply_add_element(oply, "vertex", nvertices)) return 0;

	  // add this property to output file
	  if (!ply_add_property(oply, "x", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT)) return 0;
	  if (!ply_add_property(oply, "y", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT)) return 0;
	  if (!ply_add_property(oply, "z", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT)) return 0;

	  if (!ply_add_property(oply, "nx", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT)) return 0;
	  if (!ply_add_property(oply, "ny", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT)) return 0;
	  if (!ply_add_property(oply, "nz", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT)) return 0;

	  if (!ply_add_property(oply, "diffuse_red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR)) return 0;
	  if (!ply_add_property(oply, "diffuse_green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR)) return 0;
	  if (!ply_add_property(oply, "diffuse_blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR)) return 0;
	  // write output header
	  if (!ply_write_header(oply)) return 0; 


	  for(int i = 0; i < nvertices; i++){

		  ply_write(oply, (double)pointCloud->points.at(i).x);
		  ply_write(oply, (double)pointCloud->points.at(i).y);
		  ply_write(oply, (double)pointCloud->points.at(i).z);

		  ply_write(oply, 0.0);
		  ply_write(oply, 0.0);
		  ply_write(oply, 0.0);
	
		  ply_write(oply, 255);
		  ply_write(oply, 0);
		  ply_write(oply, 0);
	  }

	  // close file
	  if (!ply_close(oply)) return 0;

  //
  // end of writing outputPLY file
  //

  return 1;
}


int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudCirles(new pcl::PointCloud<pcl::PointXYZ>);
  
  for(float i = 10.0f; i >= 5.0f; i-=0.01f)
  {
    float newridus = 1.0f/10.0*i;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtmp = createCircle(newridus, 0, 100);
    for(int j = 0; j < cloudtmp->size(); ++j)
    {
      pointcloudCirles->push_back(cloudtmp->at(j));      
    }  
  }
  for(int i = 0; i < pointcloudCirles->size(); ++i)
  {
    if(pointcloudCirles->at(i).x > 0)
    {
      pointcloudCirles->at(i).x+=8.0f;
    }
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRect = createRect(8.0f, 2.0f, 200, 40);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRectnew(new pcl::PointCloud<pcl::PointXYZ>);
  for(int i = 0; i < cloudRect->size(); ++i)
  {
    if(cloudRect->at(i).x*cloudRect->at(i).x+cloudRect->at(i).y*cloudRect->at(i).y>0.5*0.5 && (cloudRect->at(i).x-8.0f)*(cloudRect->at(i).x-8.0f)+cloudRect->at(i).y*cloudRect->at(i).y>0.5*0.5)
    {
      cloudRectnew->push_back(cloudRect->at(i));
    }
  }
  for(int i = 0; i < pointcloudCirles->size(); ++i)
  {
    cloudRectnew->push_back(pointcloudCirles->at(i));
  }
  
  int num = cloudRectnew->size();
  for(int i = 0; i < num; ++i)
  {
    pcl::PointXYZ tmppoint = cloudRectnew->at(i);
    for(float j = 0.0f; j < 0.9f; j+=0.2f)
    {
      tmppoint.z = j;
      cloudRectnew->push_back(tmppoint);
    }
  }
  
  pcl::visualization::PCLVisualizer viewer1("pcl");  
  viewer1.setBackgroundColor(1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloudRectnew, 0, 0, 0);
  viewer1.addPointCloud (cloudRectnew, single_color1, "pointcloudORI");
  while (!viewer1.wasStopped ())
  {
    viewer1.spinOnce ();
  }
  savePLY("partation.ply", cloudRectnew);
  return 1;
}