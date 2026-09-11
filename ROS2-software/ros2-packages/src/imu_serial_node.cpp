#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <cmath>
#include <cstring>
#include <sstream>
#include <string>

class ImuSerialNode : public rclcpp::Node
{
public:
    ImuSerialNode()
        : Node("imu_serial_node"),
          serial_fd_(-1)
    {
        // Parameters
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        declare_parameter<int>("baudrate", 115200);

        port_ = get_parameter("port").as_string();
        baudrate_ = get_parameter("baudrate").as_int();

        // Publisher
        imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data",
            10
        );

        // Open serial port
        if (!openSerialPort())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to open serial port: %s",
                port_.c_str()
            );

            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Serial port opened: %s @ %d baud",
            port_.c_str(),
            baudrate_
        );

        // Timer for reading serial data
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ImuSerialNode::readSerialData, this)
        );
    }

    ~ImuSerialNode()
    {
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
        }
    }

private:

    bool openSerialPort()
    {
        serial_fd_ = open(
            port_.c_str(),
            O_RDWR | O_NOCTTY | O_NONBLOCK
        );

        if (serial_fd_ < 0)
        {
            return false;
        }

        struct termios tty{};

        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // Baudrate
        speed_t speed;

        switch (baudrate_)
        {
            case 9600:
                speed = B9600;
                break;

            case 115200:
                speed = B115200;
                break;

            default:
                RCLCPP_ERROR(
                    get_logger(),
                    "Unsupported baudrate: %d",
                    baudrate_
                );

                close(serial_fd_);
                serial_fd_ = -1;
                return false;
        }

        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        // 8N1
        tty.c_cflag &= ~PARENB; // No parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;     // 8 data bits

        // Disable hardware flow control
        tty.c_cflag &= ~CRTSCTS;

        // Enable receiver
        tty.c_cflag |= CREAD | CLOCAL;

        // Raw input
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        // Raw output
        tty.c_oflag &= ~OPOST;

        // No special input processing
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(INLCR | ICRNL);

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        return true;
    }

    void readSerialData()
    {
        if (serial_fd_ < 0)
        {
            return;
        }

        char buffer[256];

        int bytes_read = read(
            serial_fd_,
            buffer,
            sizeof(buffer) - 1
        );

        if (bytes_read <= 0)
        {
            return;
        }

        buffer[bytes_read] = '\0';

        serial_buffer_ += buffer;

        // Process complete lines
        size_t newline_pos;

        while ((newline_pos = serial_buffer_.find('\n'))
               != std::string::npos)
        {
            std::string line =
                serial_buffer_.substr(0, newline_pos);

            serial_buffer_.erase(
                0,
                newline_pos + 1
            );

            processLine(line);
        }
    }

    void processLine(const std::string &line)
    {
        double roll;
        double pitch;

        if (!parseImuData(line, roll, pitch))
        {
            RCLCPP_WARN(
                get_logger(),
                "Invalid data: %s",
                line.c_str()
            );

            return;
        }

        // Debug output
        RCLCPP_INFO(
            this->get_logger(),
            "Roll: %.2f deg, Pitch: %.2f deg",
            roll,
            pitch
        );

        publishImu(roll, pitch);
    }

    bool parseImuData(
        const std::string &line,
        double &roll,
        double &pitch)
    {
        // Expected:
        // Roll:12.34,Pitch:-5.67

        if (sscanf(
                line.c_str(),
                "Roll:%lf;Pitch:%lf",
                &roll,
                &pitch) == 2)
        {
            return true;
        }

        return false;
    }

    void publishImu(
        double roll_deg,
        double pitch_deg)
    {
        // Convert degrees -> radians
        const double roll =
            roll_deg * M_PI / 180.0;

        const double pitch =
            pitch_deg * M_PI / 180.0;

        // Yaw is not available in Phase 1
        const double yaw = 0.0;

        // Roll/Pitch/Yaw -> Quaternion
        const double cy = cos(yaw * 0.5);
        const double sy = sin(yaw * 0.5);
        const double cp = cos(pitch * 0.5);
        const double sp = sin(pitch * 0.5);
        const double cr = cos(roll * 0.5);
        const double sr = sin(roll * 0.5);

        sensor_msgs::msg::Imu msg;

        msg.header.stamp = now();
        msg.header.frame_id = "imu_link";

        msg.orientation.w =
            cr * cp * cy + sr * sp * sy;

        msg.orientation.x =
            sr * cp * cy - cr * sp * sy;

        msg.orientation.y =
            cr * sp * cy + sr * cp * sy;

        msg.orientation.z =
            cr * cp * sy - sr * sp * cy;

        // Debug quaternion
        RCLCPP_INFO(
            this->get_logger(),
            "Quaternion: x=%.3f y=%.3f z=%.3f w=%.3f",
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        );

        imu_publisher_->publish(msg);
    }

private:

    std::string port_;
    int baudrate_;

    int serial_fd_;

    std::string serial_buffer_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr
        imu_publisher_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ImuSerialNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}