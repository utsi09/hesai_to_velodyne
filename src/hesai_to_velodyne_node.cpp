#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cstring>
#include <cmath>

class HesaiToVelodyneNode : public rclcpp::Node
{
public:
    HesaiToVelodyneNode() : Node("hesai_to_velodyne")
    {
        this->declare_parameter<std::string>("input_topic", "/lidar_points");
        this->declare_parameter<std::string>("output_topic", "/velodyne_points");
        this->declare_parameter<std::string>("output_frame_id", "velodyne");

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::SensorDataQoS());
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&HesaiToVelodyneNode::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "hesai_to_velodyne started: %s -> %s (frame_id: %s)",
            input_topic.c_str(), output_topic.c_str(), output_frame_id_.c_str());
    }

private:
    // Hesai PointCloud2 layout (point_step=26):
    //   x(4) y(4) z(4) intensity(4) ring(2) timestamp(8)
    //   offsets: 0    4    8    12          16       18
    //
    // Velodyne PointCloud2 layout (point_step=22):
    //   x(4) y(4) z(4) intensity(4) ring(2) time(4)
    //   offsets: 0    4    8    12          16      18

    static constexpr uint32_t HESAI_POINT_STEP = 26;
    static constexpr uint32_t VELO_POINT_STEP = 22;
    static constexpr uint32_t XYZ_IR_RING_SIZE = 18;  // x+y+z+intensity+ring bytes to copy directly

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const uint32_t num_points = msg->width * msg->height;
        if (num_points == 0) return;

        // Verify input is Hesai XYZIRT format
        if (msg->point_step != HESAI_POINT_STEP) {
            RCLCPP_WARN_ONCE(this->get_logger(),
                "Unexpected point_step %u (expected %u). Check input format.",
                msg->point_step, HESAI_POINT_STEP);
            return;
        }

        // Get base timestamp from first point for relative time calculation
        double base_timestamp;
        std::memcpy(&base_timestamp, &msg->data[XYZ_IR_RING_SIZE], sizeof(double));

        // Build output message
        auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
        out->header = msg->header;
        out->header.frame_id = output_frame_id_;
        out->height = 1;
        out->width = num_points;
        out->is_bigendian = false;
        out->is_dense = msg->is_dense;
        out->point_step = VELO_POINT_STEP;
        out->row_step = num_points * VELO_POINT_STEP;

        // Define output fields
        out->fields.resize(6);
        out->fields[0].name = "x";        out->fields[0].offset = 0;  out->fields[0].datatype = 7; out->fields[0].count = 1;
        out->fields[1].name = "y";        out->fields[1].offset = 4;  out->fields[1].datatype = 7; out->fields[1].count = 1;
        out->fields[2].name = "z";        out->fields[2].offset = 8;  out->fields[2].datatype = 7; out->fields[2].count = 1;
        out->fields[3].name = "intensity"; out->fields[3].offset = 12; out->fields[3].datatype = 7; out->fields[3].count = 1;
        out->fields[4].name = "ring";     out->fields[4].offset = 16; out->fields[4].datatype = 4; out->fields[4].count = 1;
        out->fields[5].name = "time";     out->fields[5].offset = 18; out->fields[5].datatype = 7; out->fields[5].count = 1;

        // Allocate output buffer
        out->data.resize(num_points * VELO_POINT_STEP);

        const uint8_t *src = msg->data.data();
        uint8_t *dst = out->data.data();

        for (uint32_t i = 0; i < num_points; ++i) {
            // Copy x, y, z, intensity, ring (18 bytes) directly
            std::memcpy(dst, src, XYZ_IR_RING_SIZE);

            // Convert absolute timestamp (double) to relative time (float)
            double ts;
            std::memcpy(&ts, src + XYZ_IR_RING_SIZE, sizeof(double));
            float rel_time = static_cast<float>(ts - base_timestamp);
            std::memcpy(dst + XYZ_IR_RING_SIZE, &rel_time, sizeof(float));

            src += HESAI_POINT_STEP;
            dst += VELO_POINT_STEP;
        }

        pub_->publish(std::move(out));
    }

    std::string output_frame_id_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HesaiToVelodyneNode>());
    rclcpp::shutdown();
    return 0;
}
