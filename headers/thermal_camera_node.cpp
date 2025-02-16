#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define MLX90640_I2C_ADDR 0x33
#define IMAGE_WIDTH 32
#define IMAGE_HEIGHT 24

class ThermalCameraNode : public rclcpp::Node {
public:
    ThermalCameraNode() : Node("thermal_camera_node") {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ThermalCameraNode::capture_image, this));

        RCLCPP_INFO(this->get_logger(), "Initializing MLX90640...");
        
        MLX90640_I2CInit();
        if (MLX90640_SetRefreshRate(MLX90640_I2C_ADDR, 0x05) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set refresh rate for MLX90640!");
        }
        if (MLX90640_DumpEE(MLX90640_I2C_ADDR, eeprom_data_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to dump EEPROM data!");
        }
        if (MLX90640_ExtractParameters(eeprom_data_, &mlx90640_params_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract MLX90640 parameters!");
        }
        RCLCPP_INFO(this->get_logger(), "MLX90640 initialized successfully.");
    }

private:
    void capture_image() {
        uint16_t frame_data[834];
        float image_data[768];

        if (MLX90640_GetFrameData(MLX90640_I2C_ADDR, frame_data) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get frame data from MLX90640!");
            return;
        }

        MLX90640_CalculateTo(frame_data, &mlx90640_params_, 0.95, MLX90640_GetTa(frame_data, &mlx90640_params_) - 8, image_data);

        cv::Mat thermal_image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
        for (int i = 0; i < 768; ++i) {
            thermal_image.data[i] = static_cast<uint8_t>(std::clamp((image_data[i] - 20) * 255.0 / 50.0, 0.0, 255.0));
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", thermal_image).toImageMsg();
        image_pub_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Published thermal image.");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint16_t eeprom_data_[832];
    paramsMLX90640 mlx90640_params_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThermalCameraNode>());
    rclcpp::shutdown();
    return 0;
}
