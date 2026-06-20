#include <QApplication>
#include <QLabel>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <status_interface/msg/system_status.hpp>

using whatever = status_interface::msg::SystemStatus;

class DispString : public rclcpp::Node
{
    private:
        rclcpp::Subscription<whatever>::SharedPtr subscriber_;
        QLabel * label;

    public:
        DispString(/* args */) : Node("disp_status_node")
        {
            subscriber_ = this->create_subscription<whatever>(
                "sys_status",
                10,
                [&](const whatever::SharedPtr msg)->void {
                    label->setText(get_qstr_from_msg(msg));
                }
            );
            label = new QLabel(get_qstr_from_msg(std::make_shared<whatever>()));
            label->show();
        }
        QString get_qstr_from_msg(const whatever::SharedPtr msg)
        {
            std::stringstream show_str;
            show_str
                << "系统状态: \n" 
                << "数据时间：\t" << msg->stamp.sec << "\ts\n"
                << "用户名：  \t" << msg->host_name << "\t\n"
                << "CPU使用率：\t" << msg->cpu_percent << "\t%\n"
                << "内存使用率：\t" << msg->memory_percent << "\t%\n"
                << "内存总量：\t" << msg->memory_total << "\tMB\n"
                << "剩余内存：\t" << msg->memory_available << "\tMB\n"
                << "网络发送：\t" << msg->net_sent << "\tMB\n"
                << "网络接收：\t" << msg->net_recv << "\tMB\n";
            return QString::fromStdString(show_str.str());
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<DispString>();
    std::thread spin_thread([&](){
        rclcpp::spin(node);
    });
    spin_thread.detach();
    app.exec();
    rclcpp::shutdown();
    return 0;
}
