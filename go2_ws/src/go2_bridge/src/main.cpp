#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

constexpr const char *go2_port = "ttyUSB0";

class Go2Bridge : public rclcpp::Node {

private:
    // Go2 Sport Client
    unitree::robot::go2::SportClient sportClient;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sitService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr riseService;

    void sit_command(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        (void)request;
        int32_t res = sportClient.Sit();
        if (res != 0) {
            response->message = "Error while sitting down";
            response->success = false;
            return;
        }
        response->message = "Go2 sitting down...";
        response->success = true;
    }

    void rise_command(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        (void)request;
        int32_t res = sportClient.RiseSit();
        if (res != 0) {
            response->message = "Error while standing up down";
            response->success = false;
            return;
        }
        response->message = "Go2 standing up...";
        response->success = true;
    }

public:
    Go2Bridge() : Node("go2_bridge") {
        // Initialize sport client for go2
        unitree::robot::ChannelFactory::Instance()->Init(0, go2_port);
        sportClient.SetTimeout(10.0f);
        sportClient.Init();

        // Set up services
        sitService = this->create_service<std_srvs::srv::Trigger>(
            "go2_sit",
            std::bind(&Go2Bridge::sit_command, this, std::placeholders::_1, std::placeholders::_2)
        );

        riseService = this->create_service<std_srvs::srv::Trigger>(
            "go2_rise",
            std::bind(&Go2Bridge::rise_command, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2Bridge>());
    rclcpp::shutdown();
    return 0;
}