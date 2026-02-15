#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Ring ID remapping tables (from original hesai_to_velodyne)
static int RING_ID_MAP_RUBY[] = {
    3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
    35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
    67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
    99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
    7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
    39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
    71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
    103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60
};

static int RING_ID_MAP_16[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

// Hesai point cloud format: XYZIRT with double timestamp
struct HesaiPointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(HesaiPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring)(double, timestamp, timestamp))

// Velodyne point cloud format: XYZIRT with float time (relative)
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring)(float, time, time))

// Velodyne point cloud format: XYZIR (no time)
struct VelodynePointXYZIR {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring))

template<typename T>
bool has_nan(const T &point) {
    return (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z));
}

class HesaiToVelodyneNode : public rclcpp::Node
{
public:
    HesaiToVelodyneNode() : Node("hesai_to_velodyne")
    {
        this->declare_parameter<std::string>("input_type", "XYZIRT");
        this->declare_parameter<std::string>("output_type", "XYZIRT");
        this->declare_parameter<std::string>("input_topic", "/lidar_points");
        this->declare_parameter<std::string>("output_topic", "/velodyne_points");
        this->declare_parameter<std::string>("output_frame_id", "velodyne");

        input_type_ = this->get_parameter("input_type").as_string();
        output_type_ = this->get_parameter("output_type").as_string();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::SensorDataQoS());

        if (input_type_ == "XYZI") {
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                input_topic, rclcpp::SensorDataQoS(),
                std::bind(&HesaiToVelodyneNode::hesaiHandler_XYZI, this, std::placeholders::_1));
        } else if (input_type_ == "XYZIRT") {
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                input_topic, rclcpp::SensorDataQoS(),
                std::bind(&HesaiToVelodyneNode::hesaiHandler_XYZIRT, this, std::placeholders::_1));
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Unsupported input type: '%s'. Use 'XYZI' or 'XYZIRT'.", input_type_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "hesai_to_velodyne started: %s [%s] -> %s [%s] (frame_id: %s)",
            input_topic.c_str(), input_type_.c_str(),
            output_topic.c_str(), output_type_.c_str(),
            output_frame_id_.c_str());
    }

private:
    template<typename T>
    void publish_points(T &new_pc, const sensor_msgs::msg::PointCloud2 &old_msg) {
        new_pc->is_dense = true;
        sensor_msgs::msg::PointCloud2 pc_new_msg;
        pcl::toROSMsg(*new_pc, pc_new_msg);
        pc_new_msg.header = old_msg.header;
        pc_new_msg.header.frame_id = output_frame_id_;
        pub_->publish(pc_new_msg);
    }

    void hesaiHandler_XYZI(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_new(new pcl::PointCloud<VelodynePointXYZIR>());
        pcl::fromROSMsg(*pc_msg, *pc);

        for (size_t point_id = 0; point_id < pc->points.size(); ++point_id) {
            if (has_nan(pc->points[point_id]))
                continue;

            VelodynePointXYZIR new_point;
            new_point.x = pc->points[point_id].x;
            new_point.y = pc->points[point_id].y;
            new_point.z = pc->points[point_id].z;
            new_point.intensity = pc->points[point_id].intensity;

            if (pc->height == 16) {
                new_point.ring = RING_ID_MAP_16[point_id / pc->width];
            } else if (pc->height == 128) {
                new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
            } else {
                new_point.ring = 0;
            }
            pc_new->points.push_back(new_point);
        }

        publish_points(pc_new, *pc_msg);
    }

    void hesaiHandler_XYZIRT(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
        pcl::PointCloud<HesaiPointXYZIRT>::Ptr pc_in(new pcl::PointCloud<HesaiPointXYZIRT>());
        pcl::fromROSMsg(*pc_msg, *pc_in);

        if (output_type_ == "XYZIRT") {
            pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIRT>());
            for (size_t i = 0; i < pc_in->points.size(); ++i) {
                if (has_nan(pc_in->points[i]))
                    continue;
                VelodynePointXYZIRT p;
                p.x = pc_in->points[i].x;
                p.y = pc_in->points[i].y;
                p.z = pc_in->points[i].z;
                p.intensity = pc_in->points[i].intensity;
                p.ring = pc_in->points[i].ring;
                p.time = static_cast<float>(pc_in->points[i].timestamp - pc_in->points[0].timestamp);
                pc_out->points.push_back(p);
            }
            publish_points(pc_out, *pc_msg);

        } else if (output_type_ == "XYZIR") {
            pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIR>());
            for (size_t i = 0; i < pc_in->points.size(); ++i) {
                if (has_nan(pc_in->points[i]))
                    continue;
                VelodynePointXYZIR p;
                p.x = pc_in->points[i].x;
                p.y = pc_in->points[i].y;
                p.z = pc_in->points[i].z;
                p.intensity = pc_in->points[i].intensity;
                p.ring = pc_in->points[i].ring;
                pc_out->points.push_back(p);
            }
            publish_points(pc_out, *pc_msg);

        } else if (output_type_ == "XYZI") {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZI>());
            for (size_t i = 0; i < pc_in->points.size(); ++i) {
                if (has_nan(pc_in->points[i]))
                    continue;
                pcl::PointXYZI p;
                p.x = pc_in->points[i].x;
                p.y = pc_in->points[i].y;
                p.z = pc_in->points[i].z;
                p.intensity = pc_in->points[i].intensity;
                pc_out->points.push_back(p);
            }
            publish_points(pc_out, *pc_msg);
        }
    }

    std::string input_type_;
    std::string output_type_;
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
