#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Range.h>
#include <string>
#include <algorithm>
#include <vector>
#include <limits>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>

static double realsense_frontLeft2front;
static double realsense_front2frontRight;
static double warning_distance_front;
static double stop_distance_front;
static double warning_distance_back;
static double stop_distance_back;
static double warning_distance_side;
static double stop_distance_side;

enum
{
  FRONT = 0,
  FRONT_RIGHT,
  SIDE_RIGHT,
  BACK_RIGHT,
  BACK,
  BACK_LEFT,
  SIDE_LEFT,
  FRONT_LEFT,
  NUMBER_OF_MARKERS
};

static bool footprint_subscribed;

static double max_x;
static double min_x = std::numeric_limits<double>::infinity();
static double max_y;
static double min_y = std::numeric_limits<double>::infinity();

void footprint_cb(const geometry_msgs::Polygon& current_footprint);

std::vector<std::string> namespace_list =
{
    "front", "front_right", "side_right", "back_right", "back", "back_left", "side_left", "front_left"};

std::vector<std::string> topic_list =
{
    "front", "front_right", "side_right", "back_right", "back", "back_left", "side_left", "front_left"};

class RvizMarker
{
    public:
    explicit RvizMarker(ros::NodeHandle nh, int sensor_id, std::string marker_type)
    {
        distance_type = marker_type;
        std::string topic = topic_list.at(sensor_id) + "_" + marker_type + "_area";
        marker_pub_ = nh.advertise<visualization_msgs::Marker>(topic, 10);
        MarkerCreator_(sensor_id, &marker, distance_type);
    }

    void MarkerCreator_(int sensor_id, visualization_msgs::Marker* marker, std::string marker_type);
    void MarkerPublish_();
    visualization_msgs::Marker marker;

    ros::Publisher marker_pub_;

    private:
    std::string distance_type;
};

void RvizMarker::MarkerCreator_(int sensor_id, visualization_msgs::Marker* marker, std::string marker_type)
{
    uint32_t shape_ = visualization_msgs::Marker::CUBE;

    double distance_front = (marker_type == "stop") ? stop_distance_front : warning_distance_front;
    double distance_side = (marker_type == "stop") ? stop_distance_side : warning_distance_side;
    double distance_back = (marker_type == "stop") ? stop_distance_back : warning_distance_back;

    marker->header.frame_id = "base_link";
    marker->header.stamp = ros::Time::now();

    marker->ns = namespace_list.at(sensor_id);
    marker->id = sensor_id;
    marker->type = shape_;
    marker->action = visualization_msgs::Marker::ADD;



    if (sensor_id == SIDE_LEFT || sensor_id == SIDE_RIGHT)
    {
        marker->pose.position.x = 0;
    }
    else if (sensor_id == FRONT_LEFT || sensor_id == FRONT_RIGHT || sensor_id == FRONT)
    {
        marker->pose.position.x = max_x + distance_front / 2;
    }
    else
    {
        marker->pose.position.x = min_x - distance_back / 2;
    }

    if (sensor_id == FRONT || sensor_id == BACK)
    {
        marker->pose.position.y = 0;
    }
    else if (sensor_id == SIDE_LEFT || sensor_id == FRONT_LEFT || sensor_id == BACK_LEFT)
    {
        marker->pose.position.y = max_y + distance_side / 2;
    }
    else
    {
        marker->pose.position.y = min_y - distance_side / 2;
    }

    marker->pose.position.z = 0.2;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;

    // x size
    if (sensor_id == FRONT_LEFT || sensor_id == FRONT_RIGHT || sensor_id == FRONT)
    {
        marker->scale.x = distance_front;
    }
    else if (sensor_id == BACK_LEFT || sensor_id == BACK_RIGHT || sensor_id == BACK)
    {
        marker->scale.x = distance_back;
    }
    else
    {
        marker->scale.x = max_x - min_x;
    }

    // y size
    if (sensor_id == FRONT || sensor_id == BACK)
    {
        marker->scale.y = max_y - min_y;
    }
    else
    {
        marker->scale.y = distance_side;
    }
    marker->scale.z = 1.0;

    // red = 1.0 and green = 1.0 generates yellow
    marker->color.r = 1.0f;
    marker->color.g = (marker_type == "stop") ? 0.0f : 1.0f;
    marker->color.b = 0.0f;
    marker->color.a = 0.4;

    marker->lifetime = ros::Duration();  // never letting this object to delete automatically
}

void RvizMarker::MarkerPublish_()
{
    marker.header.stamp = ros::Time::now();

    RvizMarker::MarkerCreator_(marker.id, &marker, distance_type);
    marker_pub_.publish(marker);
}


class ussRangeMaker
{
    public:
    explicit ussRangeMaker(ros::NodeHandle nh, std::string frame_id, std::string position)
    {
        uss_location = position;
        uss_warn_pub_ = nh.advertise<sensor_msgs::Range>(frame_id + "/warn", 10);
        uss_stop_pub_ = nh.advertise<sensor_msgs::Range>(frame_id + "/stop", 10);

        Range_creator(frame_id, "warn", &uss_warn_range, uss_location);
        Range_creator(frame_id, "stop", &uss_stop_range, uss_location);
    }

    void Range_creator(std::string frame_id, std::string range_type,
                        sensor_msgs::Range* uss_range, std::string uss_locatoin);

    sensor_msgs::Range uss_warn_range;
    sensor_msgs::Range uss_stop_range;

    ros::Publisher uss_warn_pub_;
    ros::Publisher uss_stop_pub_;

    std::string uss_location;

    private:
};

void ussRangeMaker::Range_creator(std::string frame_id, std::string range_type,
                                    sensor_msgs::Range* uss_range, std::string uss_locatoin)
{
        uss_range->header.stamp = ros::Time::now();
        uss_range->header.frame_id = frame_id;
        // 0: uss, 1: infrared
        uss_range->radiation_type = 0;
        uss_range->field_of_view = 1.0;
        uss_range->min_range = 0.0;
        uss_range->max_range = 100;  // some big enough value so that the range wont be discarded

        if (range_type == "warn")
        {
            uss_range->range = (uss_location == "front") ? warning_distance_front : warning_distance_back;
        }
        else if (range_type == "stop")
        {
            uss_range->range = (uss_location == "front") ? stop_distance_front : stop_distance_back;
        }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Subscriber footprint_sub_ = nh.subscribe("raw_footprint", 1, footprint_cb);

    nh_private.param("realsense_frontLeft2front", realsense_frontLeft2front, 0.0);
    nh_private.param("realsense_front2frontRight", realsense_front2frontRight, 0.0);
    nh_private.param("warning_distance_front", warning_distance_front, 0.6);
    nh_private.param("stop_distance_front", stop_distance_front, 0.2);
    nh_private.param("warning_distance_back", warning_distance_back, 0.6);
    nh_private.param("stop_distance_back", stop_distance_back, 0.2);
    nh_private.param("warning_distance_side", warning_distance_side, 0.6);
    nh_private.param("stop_distance_side", stop_distance_side, 0.2);

    // stop area marker
    RvizMarker stop_front_area(nh_private, FRONT, "warn");
    RvizMarker stop_front_right_area(nh_private, FRONT_RIGHT, "warn");
    RvizMarker stop_side_right_area(nh_private, SIDE_RIGHT, "warn");
    RvizMarker stop_back_right_area(nh_private, BACK_RIGHT, "warn");
    RvizMarker stop_back_area(nh_private, BACK, "warn");
    RvizMarker stop_back_left_area(nh_private, BACK_LEFT, "warn");
    RvizMarker stop_side_left_area(nh_private, SIDE_LEFT, "warn");
    RvizMarker stop_front_left_area(nh_private, FRONT_LEFT, "warn");

    // warning area marker
    RvizMarker warn_front_area(nh_private, FRONT, "stop");
    RvizMarker warn_front_right_area(nh_private, FRONT_RIGHT, "stop");
    RvizMarker warn_side_right_area(nh_private, SIDE_RIGHT, "stop");
    RvizMarker warn_back_right_area(nh_private, BACK_RIGHT, "stop");
    RvizMarker warn_back_area(nh_private, BACK, "stop");
    RvizMarker warn_back_left_area(nh_private, BACK_LEFT, "stop");
    RvizMarker warn_side_left_area(nh_private, SIDE_LEFT, "stop");
    RvizMarker warn_front_left_area(nh_private, FRONT_LEFT, "stop");

    ussRangeMaker uss_front_right(nh_private, "uss_front_right", "front");
    ussRangeMaker uss_front_left(nh_private, "uss_front_left", "front");
    ussRangeMaker uss_rear_right(nh_private, "uss_rear_right", "back");
    ussRangeMaker uss_rear_left(nh_private, "uss_rear_left", "back");

    ros::Rate r(10);
    while (ros::ok())
    {
        uss_front_right.uss_warn_range.header.stamp = ros::Time::now();
        uss_front_right.uss_warn_pub_.publish(uss_front_right.uss_warn_range);
        uss_front_right.uss_stop_range.header.stamp = ros::Time::now();
        uss_front_right.uss_stop_pub_.publish(uss_front_right.uss_stop_range);

        uss_front_left.uss_warn_range.header.stamp = ros::Time::now();
        uss_front_left.uss_warn_pub_.publish(uss_front_left.uss_warn_range);
        uss_front_left.uss_stop_range.header.stamp = ros::Time::now();
        uss_front_left.uss_stop_pub_.publish(uss_front_left.uss_stop_range);

        uss_rear_right.uss_warn_range.header.stamp = ros::Time::now();
        uss_rear_right.uss_warn_pub_.publish(uss_rear_right.uss_warn_range);
        uss_rear_right.uss_stop_range.header.stamp = ros::Time::now();
        uss_rear_right.uss_stop_pub_.publish(uss_rear_right.uss_stop_range);

        uss_rear_left.uss_warn_range.header.stamp = ros::Time::now();
        uss_rear_left.uss_warn_pub_.publish(uss_rear_left.uss_warn_range);
        uss_rear_left.uss_stop_range.header.stamp = ros::Time::now();
        uss_rear_left.uss_stop_pub_.publish(uss_rear_left.uss_stop_range);

        stop_front_area.MarkerPublish_();
        stop_front_right_area.MarkerPublish_();
        stop_side_right_area.MarkerPublish_();
        stop_back_right_area.MarkerPublish_();
        stop_back_area.MarkerPublish_();
        stop_back_left_area.MarkerPublish_();
        stop_side_left_area.MarkerPublish_();
        stop_front_left_area.MarkerPublish_();

        warn_front_area.MarkerPublish_();
        warn_front_right_area.MarkerPublish_();
        warn_side_right_area.MarkerPublish_();
        warn_back_right_area.MarkerPublish_();
        warn_back_area.MarkerPublish_();
        warn_back_left_area.MarkerPublish_();
        warn_side_left_area.MarkerPublish_();
        warn_front_left_area.MarkerPublish_();
        r.sleep();

        ros::spinOnce();
    }

    return 0;
}

// this callback function defines min/max_x and min/max_y for determining marker position and size
void footprint_cb(const geometry_msgs::Polygon& current_footprint)
{
    footprint_subscribed = 1;
    std::cout << footprint_subscribed << std::endl;
    for (int i = 0; i < current_footprint.points.size(); i++)
    {
        min_x = std::min(min_x, static_cast<double>(current_footprint.points.at(i).x));
        min_y = std::min(min_y, static_cast<double>(current_footprint.points.at(i).y));
        max_x = std::max(max_x, static_cast<double>(current_footprint.points.at(i).x));
        max_y = std::max(max_y, static_cast<double>(current_footprint.points.at(i).y));
    }
}
