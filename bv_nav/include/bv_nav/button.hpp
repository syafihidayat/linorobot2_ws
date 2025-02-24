#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using std::placeholders::_1;

class GamePadButton : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr button_sub;

public:
    GamePadButton() : Node("gamepadButton")
    {
        this->button_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "button", 10, std::bind(&GamePadButton::button_callback, this, std::placeholders::_1));
    }
     struct b
        {
            int X;
            int CIRCLE;
            int SQUARE;
            int TRIANGLE;
            int L1;
            int R1;
            int L2;
            int R2;
            int share;
            int Options;
            int analogLEFTHOLD;
            int analogRIGHTHOLD;
            int PS;
        } button;
         void button_callback(const std_msgs::msg::Int32MultiArray &msg)
    {

            if (msg.data.size() >= 15) // Ensure array has at least 15 elements
        {

        button.X = msg.data[0];
        button.CIRCLE = msg.data[1];
        button.SQUARE = msg.data[2];
        button.TRIANGLE = msg.data[3];
        button.L1 = msg.data[4];
        button.R1 = msg.data[5];
        button.L2 = msg.data[6];
        button.R2 = msg.data[7];
        button.share = msg.data[8];
        button.Options = msg.data[9];
        button.analogLEFTHOLD = msg.data[10];
        button.analogRIGHTHOLD = msg.data[11];
        button.PS = msg.data[16];

           }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received array with insufficient size");
        }
    }
};