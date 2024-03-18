#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

#include <algorithm>
#include <memory>

using namespace std;

class LidarConvert : public ros::NodeHandle{ // Ros의 노드를 상속받는거
    public:
        LidarConvert(){
            this->lidar_sub_ = this->subscribe("lidar2D", 10, &LidarConvert::lidarCallback, this);
            this->lidar_pub_ = this->advertise<sensor_msgs::LaserScan>("scan", 10);
        }
    private:
        ros::Publisher lidar_pub_;
        ros::Subscriber lidar_sub_;

        sensor_msgs::LaserScan scan_msg_;

        bool init_flag = false;

        void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg){
            if(!this->init_flag){
                // this->scan_msg_.header.frame_id = msg->header.frame_id;
                this->scan_msg_.header.frame_id = "lidar";
                this->scan_msg_.angle_min = msg->angle_min;
                this->scan_msg_.angle_max = msg->angle_max;
                this->scan_msg_.angle_increment = msg->angle_increment;
                this->scan_msg_.time_increment = msg->time_increment;
                this->scan_msg_.scan_time = msg->scan_time;
                this->scan_msg_.range_min = msg->range_min;
                this->scan_msg_.range_max = msg->range_max;
                this->scan_msg_.ranges.resize(msg->ranges.size());
                this->init_flag = true;
            }

            // this->scan_msg_.header.stamp = ros::Time::now();
            this->scan_msg_.header.stamp = msg->header.stamp;
            std::transform(msg->ranges.begin(), msg->ranges.begin() + 180, scan_msg_.ranges.begin()+180, [](auto value){ // value를 받아서 넣어주는 lambda 표현식이다.
                return value;
            });
            std::transform(msg->ranges.begin() + 180, msg->ranges.end(), scan_msg_.ranges.begin(), [](auto value){
                return value;
            });
            this->lidar_pub_.publish(scan_msg_);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_convert");
    auto lidar_convert=make_shared<LidarConvert>(); // 스마트 포인터를 사용해서 메모리를 관리한다.

    ros::spin();
    return 0;
}