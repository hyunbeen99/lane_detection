#include "lane_detection.h"
#include "dbscan.cpp"
#include <visualization_msgs/Marker.h>
#include <velodyne_pointcloud/point_types.h>

#define MINIMUM_POINTS 4     // minimum number of cluster
#define EPSILON (0.75*0.75) 

#define MIN_INTEN 25	// minimum value of intensity to detect lane
#define MAX_INTEN 70	// maximum value of intensity to detect lane

using namespace std;

float LEFT_L;
float LEFT_R;
float RIGHT_L;
float RIGHT_R;
float FRONT_OFFSET = 0.5;

void LaneDetect::initsetup(){
    sub_ = nh_.subscribe("/velodyne_points", 1, &LaneDetect::pointcloudCallback, this);
	marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("result", 10);
    //pub_ = nh_.advertise<lane_detection::DbMsg>("db_msg",10);
}


void LaneDetect::pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
    //ROS_INFO("callback");
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::fromROSMsg(*input,*cloud_XYZIR);
    // now use cloud_XYZIR

    vector<Point> db_points;

    for (std::size_t j = 0; j < cloud_XYZIR->points.size(); ++j){
        velodyne_pointcloud::PointXYZIR pt_point = cloud_XYZIR->points[j];

        if (MIN_INTEN <= pt_point.intensity && pt_point.intensity <= MAX_INTEN) {
            Point db_point(pt_point.x, pt_point.y, pt_point.z, pt_point.intensity, pt_point.ring); // class 'Point' in dbscan.h
            db_points.push_back(db_point);
        }
    }

    DBSCAN ds(MINIMUM_POINTS, EPSILON, db_points);
    ds.run(); 

    visualization_msgs::Marker marker; //points, line_strip, line_list;
    marker.ns = "points_and_lines";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;  
    marker.id = 0;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1;
    marker.scale.y = 0.1;
    //marker.scale.z = 0.1;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;


    //   +
    //   x     + y -
    //   -

    vector<LanePoint> left_lane; 
    vector<LanePoint> right_lane; 
    
    geometry_msgs::Point p;
    velodyne_pointcloud::PointXYZIR result_point;
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr result (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);

    // unfiltered points
    LanePoint lp;
    vector<LanePoint> temp_left_lane;
    vector<LanePoint> temp_right_lane;
    
    for (auto point : ds.m_points) {
        lp.x = point.x;
        lp.y = point.y;
        lp.layer = point.layer;

        if (LEFT_R < point.y && point.y < LEFT_L && point.x > FRONT_OFFSET){
            temp_left_lane.push_back(lp);
        }
        else if (RIGHT_R < point.y && point.y < RIGHT_L && point.x > FRONT_OFFSET){
            temp_right_lane.push_back(lp);
        }
    }
    
    // for sorting
    vector<vector<LanePoint>> left_layer_list(16);
    vector<vector<LanePoint>> right_layer_list(16);

    // index == layer
    for (auto point : temp_left_lane) {
        left_layer_list.at(point.layer).push_back(point);
    }

    for (auto point : temp_right_lane) {
        right_layer_list.at(point.layer).push_back(point);
    }
    
    // calculate mean point and push it into left_lane, right_lane
    for (auto lpv : left_layer_list) {
        int sum_x, sum_y = 0;

        for (auto lpoint : lpv) {
            sum_x += lpoint.x;
            sum_y += lpoint.y;
        }

        LanePoint lp;

        lp.x = sum_x / lpv.size();
        lp.y = sum_y / lpv.size();
        left_lane.push_back(lp);
    }

    for (auto lpv : right_layer_list) {
        int sum_x, sum_y = 0;

        for (auto lpoint : lpv) {
            sum_x += lpoint.x;
            sum_y += lpoint.y;
        }

        LanePoint lp;

        lp.x = sum_x / lpv.size();
        lp.y = sum_y / lpv.size();
        right_lane.push_back(lp);
    }

    /*for (auto point : cloud_XYZIR->points) {
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = point.z;
    }*/

    //points.points.push_back(p);
    //line_list.points.push_back(p);
    //line_strip.points.push_back(p);

    (*result).header.frame_id = marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();	
    pub_.publish(result);
    //marker_pub.publish(marker);
    
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "lane_detection_node");
	LaneDetect lanedetect;
    lanedetect.initsetup();  
	ros::spin();
    
	return 0;
}
