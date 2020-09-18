#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "tr_pipeline_interfaces/msg/circle_detection.hpp"
#include "tr_pipeline_interfaces/msg/pipeline_feedback.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
using std::placeholders::_1;


namespace circle_approach
{
    class CirclePoseEstimator: public rclcpp::Node
    {
        public:
            explicit CirclePoseEstimator(const rclcpp::NodeOptions & options)
            : Node("pose_estimator", options) 
            {
                rmw_qos_profile_t subscriber_qos_profile = rmw_qos_profile_sensor_data;
                rmw_qos_profile_t publisher_qos_profile = rmw_qos_profile_default;

                point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("pose_estimator/point", 10);

                circle_sub_ = this->create_subscription<tr_pipeline_interfaces::msg::CircleDetection>(
                    "detector/circle", 10, std::bind(&CirclePoseEstimator::circle_callback, this, _1)
                );

                feedback_pub_ = this->create_publisher<tr_pipeline_interfaces::msg::PipelineFeedback>(
                    "/tr/pipeline_feedback" , 10
                );

                img_pub_ = image_transport::create_publisher(this, 
                                                        "pose_estimator/image", 
                                                        publisher_qos_profile);


                img_sub_ = image_transport::create_subscription(this, 
                                                            "detector/image",
                                                            std::bind(&CirclePoseEstimator::img_callback, this, _1),
                                                            "raw",
                                                            subscriber_qos_profile);
            }

            void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
            {
                try
                {
                    cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
                }
                catch (cv_bridge::Exception& e)
                {
                    return;
                }
            }

            void circle_callback(const tr_pipeline_interfaces::msg::CircleDetection::SharedPtr msg)
            {
                double radius = msg->radius;
                double x = msg->x;
                double y = msg->y;
            
                double x_dist = (TENNIS_BALL_RADIUS*FOCAL_LENGTH)/radius; // (cm*px)/px
                double y_dist = ((CENTRE_PIXEL_Y - x)*x_dist)/FOCAL_LENGTH;
                double z_dist = ((CENTRE_PIXEL_Z - y)*x_dist)/FOCAL_LENGTH;
                auto point_msg = geometry_msgs::msg::Point();
                point_msg.x = x_dist;
                point_msg.y = y_dist;
                point_msg.z = z_dist;
                point_pub_->publish(point_msg);     

                std::stringstream ss;
                ss << x_dist << "cm";
                if (cv_ptr_->image.rows != 0)
                {
                    cv::putText(cv_ptr_->image, ss.str(), cv::Point(10,40), cv::FONT_HERSHEY_SIMPLEX,1.5, cv::Scalar(255,255,255),2);
                    sensor_msgs::msg::Image::SharedPtr out_msg = cv_ptr_->toImageMsg();
                    img_pub_.publish(out_msg);
                }
            }
            
        private:
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
            rclcpp::Subscription<tr_pipeline_interfaces::msg::CircleDetection>::SharedPtr circle_sub_;
            rclcpp::Publisher<tr_pipeline_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;
            image_transport::Publisher img_pub_;
            image_transport::Subscriber img_sub_;

            cv_bridge::CvImagePtr cv_ptr_;

            const int WIDTH = 640;
            const int HEIGHT = 480;
            const double FOCAL_LENGTH = 368.101127; //px
            const double  TENNIS_BALL_RADIUS = 3.429; //cm
            const int CENTRE_PIXEL_Y = 320;
            const int CENTRE_PIXEL_Z = 240;

    };

} // namespace circle_approach

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(circle_approach::CirclePoseEstimator)
