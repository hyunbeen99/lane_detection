#include <ros/ros.h>
#include <ros/package.h>

#include <math.h>
#include <iostream>
#include <cmath>
#include <string>  
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <vector>

#include "lane_detection/DbMsg.h"
#include "lane_detection/DbPoint.h"

using namespace std;

class LanePoint{
	public:
		float x, y;
		int layer;

		LanePoint() { }

		LanePoint(float x, float y, int layer) {
			this->x = x;
			this->y = y;
			this->layer = layer;
		}
};

class LaneDetect {
    private:
        ros::Subscriber sub_, sub_2;
        ros::Publisher pub_, marker_pub_, waypoint_pub_;
		vector<float> left_poly_;
		vector<float> right_poly_;
    public:
        ofstream writeFile;
        ros::NodeHandle nh_;
        void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr & input);
        void initsetup();
		void visualize(vector<LanePoint> left_lane, vector<LanePoint> right_lane, vector<geometry_msgs::Point> waypoint);
};
