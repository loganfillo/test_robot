#include <opencv2/core/fast_math.hpp>
#include <sstream>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tr_pipeline_interfaces/msg/circle_detection.hpp"
#include "tr_pipeline_interfaces/msg/pipeline_feedback.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
using std::placeholders::_1;

namespace circle_approach
{
    class CircleDetector: public rclcpp::Node
    {
        public:
            explicit CircleDetector(const rclcpp::NodeOptions & options)
            : Node("detector", options) 
            {   
                rmw_qos_profile_t subscriber_qos_profile = rmw_qos_profile_sensor_data;
                rmw_qos_profile_t publisher_qos_profile = rmw_qos_profile_default;

                point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
                    "pose_estimator/point", 10, std::bind(&CircleDetector::point_callback, this, _1)
                );

                circle_pub_ = this->create_publisher<tr_pipeline_interfaces::msg::CircleDetection>(
                    "detector/circle", 10
                );

                feedback_pub_ = this->create_publisher<tr_pipeline_interfaces::msg::PipelineFeedback>(
                    "/tr/pipeline_feedback" , 10
                );

                img_pub_ = image_transport::create_publisher(this, 
                                                        "detector/image", 
                                                        publisher_qos_profile);


                img_sub_ = image_transport::create_subscription(this, 
                                                            "detector/in",
                                                            std::bind(&CircleDetector::img_callback, this, _1),
                                                            "raw",
                                                            subscriber_qos_profile);
            }

            void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
            {
                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                }
                catch (cv_bridge::Exception& e)
                {
                    return;
                }

                cv::Mat gray;
                cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
                cv::medianBlur(gray, gray, 3);
                std::vector<cv::Vec3f> circles;
                cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows/2, 100,30,5,640);
                circle_history_.insert(circle_history_.end(), circles.begin(), circles.end());
                if (circle_history_.size() > 2)
                {
                    detected_circle_ = medianOfCircles(circle_history_);
                    draw_ = true;
                    circle_history_.clear();

                    std::stringstream ss;
                    ss << "Circle Detected at (" << detected_circle_[0] <<", " << detected_circle_[1]<< ")";
                    ss <<  "with radius " << detected_circle_[2];
                    auto msg = tr_pipeline_interfaces::msg::PipelineFeedback();
                    msg.message = ss.str();
                    feedback_pub_->publish(msg);

                }
                if (draw_)
                {
                    int x = cvRound(detected_circle_[0]);
                    int y = cvRound(detected_circle_[1]);
                    cv::Point center(x, y);
                    int radius = cvRound(detected_circle_[2]);
                    cv::circle( cv_ptr->image, center, radius, color_, 3);
                    auto msg = tr_pipeline_interfaces::msg::CircleDetection();
                    msg.x = x;
                    msg.y = y;
                    msg.radius = radius;
                    circle_pub_->publish(msg);
                }

                sensor_msgs::msg::Image::SharedPtr out_msg = cv_ptr->toImageMsg();
                img_pub_.publish(out_msg);
            }

            void point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
            {
                double x = msg->x;
                double y = msg->y;
                double z = msg->z;
                if (isDistWithinLimits(x, DESIRED_X_DIST) &&
                    isDistWithinLimits(y, 0) &&
                    isDistWithinLimits(z, 0))
                {
                    color_ = cv::Scalar(0,255,0);
                    succesful_detections_ += 1;
                    if (succesful_detections_ > DETECTION_THRESH)
                    {
                        auto msg = tr_pipeline_interfaces::msg::PipelineFeedback();
                        msg.success = true;
                        msg.message = "Circle approach success!";
                        feedback_pub_->publish(msg);
                    }
                }
                else
                {
                    color_ = cv::Scalar(0,0,255);
                    succesful_detections_ = 0;
                }
                
            }

        private:
            rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;
            rclcpp::Publisher<tr_pipeline_interfaces::msg::CircleDetection>::SharedPtr circle_pub_;
            rclcpp::Publisher<tr_pipeline_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;
            image_transport::Publisher img_pub_;
            image_transport::Subscriber img_sub_;
            std::vector<cv::Vec3f> circle_history_;
            cv::Vec3f detected_circle_;
            cv::Scalar color_ = cv::Scalar(0,0,255);
            float DIST_TOL = 5;
            float DESIRED_X_DIST = 50;
            float DIFF_TOL = 100.0; 
            int DETECTION_THRESH = 100;
            bool draw_ = false;
            int succesful_detections_ = 0;

            cv::Vec3f medianOfCircles(std::vector<cv::Vec3f> &circles)
            {   
                size_t size = circles.size();
                cv::Vec3f median;
                std::vector<double> x;
                std::vector<double> y;
                std::vector<double> r;
                for (auto const& circle : circles)
                {
                    x.push_back(circle[0]);
                    y.push_back(circle[1]);
                    r.push_back(circle[2]);
                }
                median[0] = medianOfVector(x, size);
                median[1] = medianOfVector(y, size);
                median[2] = medianOfVector(r, size);
                return median;
            }

            double medianOfVector(std::vector<double>& vec, size_t size)
            {
                std::sort(vec.begin(), vec.end());
                if (size%2 == 0)
                {
                    return (vec[size/2-1]+vec[size/2])/2;
                } else 
                {
                    return vec[size/2];
                }
            }

            bool isDiffBelowTol(const cv::Vec3f& circle)
            {
                double x_diff = diff(circle[0], detected_circle_[0]);
                double y_diff = diff(circle[1], detected_circle_[1]);
                double r_diff = diff(circle[2], detected_circle_[2]);
                return (x_diff < DIFF_TOL && y_diff < DIFF_TOL && r_diff < DIFF_TOL);
            }

            bool isDistWithinLimits(double dist, double desired_dist)
            {
                return (dist < desired_dist + DIST_TOL && dist > desired_dist - DIST_TOL);
            }

            double diff(double a, double b)
            {
                return std::abs(a-b);
            }


    };

} // namespace circle_approach

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(circle_approach::CircleDetector)
