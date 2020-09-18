#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
using std::placeholders::_1;

namespace bar_pipeline
{
    class ComponentOne: public rclcpp::Node
    {
        public:
            explicit ComponentOne(const rclcpp::NodeOptions & options)
            : Node("component_one", options) 
            {
                rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
                pub_ = image_transport::create_publisher(this, 
                                                        "component_one/out", 
                                                        custom_qos_profile);


                sub_ = image_transport::create_subscription(this, 
                                                            "component_one/in",
                                                            std::bind(&ComponentOne::callback, this, _1),
                                                            "raw",
                                                            custom_qos_profile);
            }

            void callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
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
                sensor_msgs::msg::Image::SharedPtr out_msg = cv_ptr->toImageMsg();
                pub_.publish(out_msg);
            }

        private:
            image_transport::Publisher pub_;
            image_transport::Subscriber sub_;

    };

} // namespace bar_pipeline

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bar_pipeline::ComponentOne)
