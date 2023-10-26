#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <curses.h>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

std::mutex pressedMTX;
std::unique_ptr<char> pressed;

double V_MAX_LIMIT = 1;
double O_MAX_LIMIT = 1;
double ACC_V = 0.1;
double DEC_V = 0.2;
double ACC_OMEGA = 0.1;
double DEC_OMEGA = 0.3;
double STEP = 0.01;
double A_LAT = 2.0;

std::string robot_name;

/**
 * @brief ROS2 node to send velocities commands to the **cmd_vel** topic using the keyboard
 * 
 */
class CMDPublisher : public rclcpp::Node
{
  public:
    /**
     * @brief Construct a new CMDPublisher object
     * 
     */
    CMDPublisher()
    : Node("remote_control")
    {
      client_ = this->create_client<std_srvs::srv::SetBool>(robot_name+"/power");
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      request->data = true;
      client_->async_send_request(request);

      publisher_ = rclcpp::Node::create_publisher<geometry_msgs::msg::Twist>(robot_name+"/cmd_vel", 10);
      initscr();
      noecho();
      last = std::chrono::system_clock::now();
      timer_ = this->create_wall_timer(
      100ms, std::bind(&CMDPublisher::timer_callback, this));
    }

    private:
    /// @brief Method callback to publish the velocities command from the keyboard input.
    void timer_callback()
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = false;
      geometry_msgs::msg::Twist msg;
      auto curr = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = curr - last;
      double dT = diff.count();
      last = curr;
      int v = 0;
      int omega = 0;
      {
        std::unique_lock<std::mutex> lock(pressedMTX);
        key_p = *pressed;
        *pressed = 0;
      }
      /* Look for a keypress */
      switch( key_p ){
        case '4':
        case 'D':
        case 'a':
          omega = 1;
          break;
        case '6':
        case 'C':
        case 'd':
          omega = -1;
          break;
        case '8':
        case 'A':
        case 'w':
          v = 1;
          break;
        case '2':
        case 'B':
        case 'x':
          v = -1;
          break;
        case '5':
        case 's':
          V_MAX = 0;
          O_MAX = 0;
          break;
        case 'q':
          terminate = true;
          client_->async_send_request(request);
          break;
        default:
          break;
      }
      double V_MAX_N = std::min(A_LAT/std::abs(omega_c), V_MAX);

      V_MAX += STEP * v;
      V_MAX = std::min(V_MAX, V_MAX_LIMIT);
      O_MAX += STEP * omega;
      O_MAX = std::min(O_MAX, O_MAX_LIMIT);

      if (std::abs(V_MAX) > std::abs(v_c)) {
        v_c += ACC_V*(v_c==0?(V_MAX>0?1:-1):(v_c>0?1:-1)) * dT;
        v_c = V_MAX>0?std::min(v_c, V_MAX_N):std::max(v_c, V_MAX_N);
      } else if(std::abs(V_MAX) < std::abs(v_c)) {
        if(std::abs(v_c) < STEP*1.5)v_c = 0;
        else v_c -= DEC_V*(v_c>0?1:-1) * dT;
      }
      if (std::abs(O_MAX) > std::abs(omega_c)) {
        omega_c += ACC_OMEGA*(omega_c==0?(O_MAX>0?1:-1):(omega_c>0?1:-1)) * dT;
        omega_c = O_MAX>0?std::min(omega_c, O_MAX):std::max(omega_c, O_MAX);
      } else if(std::abs(O_MAX) < std::abs(omega_c)) {
        if(std::abs(omega_c) < STEP*1.5)omega_c = 0;
        else omega_c -= DEC_OMEGA*(omega_c>0?1:-1) * dT;
      }

      clear();
      move(0,0);
      printw("*** USE q TO STOP THE REMOTE CONTROLLER ***\nUse 8, w or arrow up to move forward\nUse 2, x or arrow down to move backwards\nUse 4, a or arrow left to curve left\nUse 6, d or arrow right to curve right\nUse 5 or s to brake\n*******************************************\n");
      printw("V: %f OMEGA: %f\n", V_MAX, O_MAX);

      msg.linear.x = v_c;
      msg.angular.z = omega_c;
      publisher_->publish(msg);
      
      //printw("dT: %f\n", dT);
      printw("v_c: %f omega_c: %f\n", v_c, omega_c);
      //printw("key: %c\n", key_p);
      refresh();
      if(terminate){
        endwin();
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        rclcpp::shutdown();
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
    char key_p;
    bool terminate = false;
    double v_c = 0.0, omega_c = 0.0, V_MAX = 0.0, O_MAX = 0.0;
    std::chrono::system_clock::time_point last;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

/// @brief Function that checks if there is new data available from the keyboard without having to wait for the return key.
void kbhit()
{
  int ch = 0;
  while((char)ch != 'q'){
    ch = getch();

    if (ch != ERR) {
      std::unique_lock<std::mutex> lock(pressedMTX);
      //printw("Key pressed! It was: %d\n", ch);
      //refresh();
      *pressed = (char)ch;
    } else {
      //printw("No key pressed yet...\n");
      //refresh();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

/**
 * @brief Function to initialize the ROS node.
 * 
 * @param argc Number of arguments passed from the command line.
 * @param argv Array of arguments passed from the command line.
 */
void start_ros_node(int argc, char * argv[])
{
  if(argc < 2)  robot_name = "";
  else          robot_name = argv[1];
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CMDPublisher>());
}


/**
 * @brief Main function that start two separete threads for the ROS node and the keyboard listener.
 * 
 * @param argc Number of arguments passed from the command line.
 * @param argv Array of arguments passed from the command line.
 */
int main(int argc, char * argv[])
{
  pressed = std::make_unique<char>();

  std::thread remote_control(start_ros_node, argc, argv);
  std::thread key_listener(kbhit);

  remote_control.join();
  key_listener.join();

  return 0;
}

