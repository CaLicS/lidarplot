#include "lidarplot/lidarplot.hpp"

LIDARPLOT::LIDARPLOT() : Node("lidar")
{
  lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&LIDARPLOT::scanCb, this, _1));
}
void LIDARPLOT::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan) const
{
  int count = scan->scan_time / scan->time_increment;

  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
  
  for (int i = 0; i < count; i++) {
    float degree = scan->angle_min + scan->angle_increment * i;
    printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", RAD2DEG(degree), scan->ranges[i]);
    int x = 250 + scan->ranges[i] * 50 * sin(degree);
    int y = 250 + scan->ranges[i] * 50 * cos(degree);
    cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), 1);
  }
  cv::imshow("img", img);
  cv::waitKey(1);
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LIDARPLOT>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}