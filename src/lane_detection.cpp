#include "lane_detection.h"
#include "dbscan.cpp"
#include "polyfit.h"
#include "polyfit.c"
#include <visualization_msgs/Marker.h>
#include <velodyne_pointcloud/point_types.h>

#define MINIMUM_POINTS 4     // minimum number of cluster
#define EPSILON (0.75*0.75) 

#define MIN_INTEN 25	// minimum value of intensity to detect lane
#define MAX_INTEN 70	// maximum value of intensity to detect lane

using namespace std;

float LEFT_L = 3.5;
float LEFT_R = 0.8;
float RIGHT_L = -0.5;
float RIGHT_R = -2;
float FRONT_MIN_OFFSET = 0.3;
float FRONT_MAX_OFFSET = 7.0;

const unsigned int ORDER = 3;
const double ACCEPTABLE_ERROR = 0.01;

vector<float> left_poly(4);
vector<float> right_poly(4);
vector<float> pre_left_poly(4);
vector<float> pre_right_poly(4);

void LaneDetect::initsetup(){
    sub_ = nh_.subscribe("/velodyne_points", 1, &LaneDetect::pointcloudCallback, this);
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	//pub_ = nh_.advertise<sensor_msgs::PointCloud2>("result", 10);
    //pub_ = nh_.advertise<lane_detection::DbMsg>("db_msg",10);
}

void LaneDetect::pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::fromROSMsg(*input,*cloud_XYZIR);

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
    
    // unfiltered points
    vector<LanePoint> temp_left_lane;
    vector<LanePoint> temp_right_lane;

    for (auto point : ds.m_points) {
		LanePoint lp(point.x, point.y, point.layer);

        if (LEFT_R < point.y && point.y < LEFT_L && FRONT_MIN_OFFSET < point.x && point.x < FRONT_MAX_OFFSET){
            temp_left_lane.push_back(lp);
        }
        else if (RIGHT_R < point.y && point.y < RIGHT_L && FRONT_MIN_OFFSET < point.x && point.x < FRONT_MAX_OFFSET){
            temp_right_lane.push_back(lp);
        }
    }
    
    // for sorting
    vector<vector<LanePoint>> left_layer_list(16);
    vector<vector<LanePoint>> right_layer_list(16);

    // index == layer
    for (auto point : temp_left_lane) {
        left_layer_list[point.layer].push_back(point);
    }

    for (auto point : temp_right_lane) {
        right_layer_list.at(point.layer).push_back(point);
    }
    

    //   +
    //   x     + y -
    //   -
    vector<LanePoint> left_lane; 
    vector<LanePoint> right_lane; 

    int left_invalidated = 0;

    // calculate mean point and push it into left_lane, right_lane
    for (auto lpv : left_layer_list) {
        LanePoint lp;

        if (lpv.size() != 0) {
            float sum_x = 0.0;
            float sum_y = 0.0;

            for (auto lpoint : lpv) {
                sum_x += lpoint.x;
                sum_y += lpoint.y;
            }

            lp.x = sum_x / lpv.size();
            lp.y = sum_y / lpv.size();
            left_lane.push_back(lp);
        } else {
            lp.x = -1000.0;
            lp.y = -1000.0;
            left_lane.push_back(lp);
            left_invalidated++;
        }
    }

    if (left_invalidated <= 12) { // layers detected 4 or more
        int result;

        double xData[left_lane.size()];
        double yData[left_lane.size()];

        for (int i = 0 ; i < left_lane.size() ; i++) {
            xData[i] = left_lane.at(i).x;
            yData[i] = left_lane.at(i).y;
        }

        double coef[ORDER + 1];
        result = polyfit(xData, yData, left_lane.size(), ORDER, coef);

        cout << endl;
        cout << "left lane" << endl;
        cout << coef[3] << " : " << coef[2] << " : " << coef[1] << " : " << coef[0] << endl;
        cout << endl;

    } else { // less than 4 -> use previous poly
    }
    
    int right_invalidated = 0;

    // calculate mean point and push it into left_lane, right_lane
    for (auto lpv : right_layer_list) {
        LanePoint lp;

        if (lpv.size() != 0) {
            float sum_x = 0.0;
            float sum_y = 0.0;

            for (auto lpoint : lpv) {
                sum_x += lpoint.x;
                sum_y += lpoint.y;
            }

            lp.x = sum_x / lpv.size();
            lp.y = sum_y / lpv.size();
            right_lane.push_back(lp);
        } else {
            lp.x = -1000.0;
            lp.y = -1000.0;
            right_lane.push_back(lp);
            right_invalidated++;
        }
    }

    if (right_invalidated <= 12) { // layers detected 4 or more
        int result;

        double xData[right_lane.size()];
        double yData[right_lane.size()];

        for (int i = 0 ; i < right_lane.size() ; i++) {
            xData[i] = right_lane.at(i).x;
            yData[i] = right_lane.at(i).y;
        }

        double coef[ORDER + 1];
        result = polyfit(xData, yData, right_lane.size(), ORDER, coef);

        cout << endl;
        cout << "right lane" << endl;
        cout << coef[3] << " : " << coef[2] << " : " << coef[1] << " : " << coef[0] << endl;
        cout << endl;

    } else { // less than 4 -> use previous poly
    }

	visualize(left_lane, right_lane);
}

void LaneDetect::visualize(vector<LanePoint> left_lane, vector<LanePoint> right_lane) {
    geometry_msgs::Point p;
    velodyne_pointcloud::PointXYZIR result_point;
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr result (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);

	visualization_msgs::Marker marker; //points, line_strip, line_list;
    marker.ns = "points_and_lines";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;  
    marker.id = 0;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    //marker.scale.z = 0.1;

    marker.color.a = 1.0;
    marker.color.r = 1.0f;

    for (auto point : left_lane) {
        p.x = point.x;
        p.y = point.y;
        p.z = 0.1;
        marker.points.push_back(p);
        //result->points.push_back(p);
    }

    for (auto point : right_lane) {
        p.x = point.x;
        p.y = point.y;
        p.z = 0.1;
        marker.points.push_back(p);
        //result->points.push_back(p);    
    }

    //line_list.points.push_back(p);
    //line_strip.points.push_back(p);

    //result->header.frame_id = "velodyne";
    //pub_.publish(result);
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();	
    marker_pub_.publish(marker);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lane_detection_node");
	LaneDetect lanedetect;
    lanedetect.initsetup();  
	ros::spin();
    
	return 0;
}
