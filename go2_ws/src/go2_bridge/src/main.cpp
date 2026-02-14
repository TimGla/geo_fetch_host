#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/int8.hpp>
#include "go2_interfaces/srv/mode.hpp"

enum class EspGeoFetchState: u_int8_t {
    UNKNOWN = 0,
    PREPARING = 1,
    READY = 2,
    DRILLING = 3,
    COLLECTING_MATERIAL = 4,
    CLEANING = 5,
    ERROR = 6,
};

enum class SampleStep {
    IDLE,
    INITIATED,
    WAITING_FOR_SYSTEM_PREPARE,
    SITTING,
    DRILLING,
    READY_FOR_COLLECTION,
    COLLECTING,
};

class Go2Bridge : public rclcpp::Node {

private:
    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeSampleService;

    // go2_driver mode service
    rclcpp::Client<go2_interfaces::srv::Mode>::SharedPtr go2ModeService;

    // Services on esp
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr geoFetchPrepareService;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr geoFetchDrillService;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr geoFetchCollectService;

    // State subscription on esp
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr geoFetchStateSubscriber;

    // Callback timer for sampling procedure
    rclcpp::TimerBase::SharedPtr samplingCallback;

    SampleStep currentSamplingStep = SampleStep::IDLE;
    EspGeoFetchState espState = EspGeoFetchState::UNKNOWN;

    void take_sample_command(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        if (currentSamplingStep != SampleStep::IDLE) {
            response->success = false;
            response->message = "Sampling already in progress";
            return;
        }
        currentSamplingStep = SampleStep::INITIATED;
        response->success = true;
        response->message = "Sampling initiated";
    }

    void timer_callback() {
        switch (currentSamplingStep) {
            case SampleStep::INITIATED:
                call_geo_fetch_prepare();
                currentSamplingStep = SampleStep::WAITING_FOR_SYSTEM_PREPARE;
                break;
            case SampleStep::WAITING_FOR_SYSTEM_PREPARE:
                if (espState == EspGeoFetchState::READY) {
                    set_go2_mode('sit');
                    currentSamplingStep = SampleStep::SITTING;
                }
                break;
            case SampleStep::SITTING:
                call_geo_fetch_drill();
                currentSamplingStep = SampleStep::DRILLING;
                break;
            case SampleStep::DRILLING:
                if (espState == EspGeoFetchState::READY) {
                    set_go2_mode('rise_sit');
                    currentSamplingStep = SampleStep::READY_FOR_COLLECTION;
                }
                break;
            case SampleStep::READY_FOR_COLLECTION:
                call_geo_fetch_collect();
                currentSamplingStep = SampleStep::COLLECTING;
                break;
            case SampleStep::COLLECTING:
                if (espState == EspGeoFetchState::READY) {
                    currentSamplingStep = SampleStep::IDLE;
                }
                break;
            default:
                break;
        }
    }

    void set_go2_mode(std::string mode_str) {
        if (!go2ModeService->wait_for_service(std::chrono::seconds(2))) {
            return;
        }
        auto request = std::make_shared<go2_interfaces::srv::Mode::Request>();
        request->mode = mode_str; 

        go2ModeService->async_send_request(request,
            [this, mode_str](rclcpp::Client<go2_interfaces::srv::Mode>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Response: Success=%d, Message='%s'", 
                            response->success, response->message.c_str());
            }
        );
    }

    void call_geo_fetch_prepare() {
        if (!geoFetchPrepareService->wait_for_service(std::chrono::seconds(2))) {
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        
        geoFetchPrepareService->async_send_request(
            request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Response: Success=%d, Message='%s'", 
                            response->success, response->message.c_str());
            }
        );
    }

    void call_geo_fetch_drill() {
        if (!geoFetchDrillService->wait_for_service(std::chrono::seconds(2))) {
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        
        geoFetchDrillService->async_send_request(
            request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Response: Success=%d, Message='%s'", 
                            response->success, response->message.c_str());
            }
        );
    }

    void call_geo_fetch_collect() {
        if (!geoFetchCollectService->wait_for_service(std::chrono::seconds(2))) {
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        
        geoFetchCollectService->async_send_request(
            request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Response: Success=%d, Message='%s'", 
                            response->success, response->message.c_str());
            }
        );
    }

    void esp_state_callback(const std_msgs::msg::Int8::SharedPtr msg) {
        espState = static_cast<EspGeoFetchState>(msg->data);
    }

    

public:
    Go2Bridge() : Node("geo_fetch_system") {
        takeSampleService = this->create_service<std_srvs::srv::Trigger>(
            "geofetch_take_sample",
            std::bind(&Go2Bridge::take_sample_command, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Make services on esp available
        geoFetchPrepareService = this->create_client<std_srvs::srv::Trigger>("/system/prepare");
        geoFetchDrillService = this->create_client<std_srvs::srv::Trigger>("/system/drill");
        geoFetchCollectService = this->create_client<std_srvs::srv::Trigger>("/system/collect");

        // Subscribe to esp state
        geoFetchStateSubscriber = this->create_subscription<std_msgs::msg::Int8>(
            "system/state",
            10,
            std::bind(&Go2Bridge::esp_state_callback, this, std::placeholders::_1)
        );

        go2ModeService = this->create_client<go2_interfaces::srv::Mode>("/mode");

        samplingCallback = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&Go2Bridge::timer_callback, this)
        );
    }
};

int main(int argc, char **argv) {
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<Go2Bridge>());
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
