#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <curses.h>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

std::mutex pressedMTX;
std::unique_ptr<char> pressed;

double V_MAX_LIMIT = 1;
double O_MAX_LIMIT = 1;
double ACC_V = 0.1;
double DEC_V = 0.3;
double ACC_OMEGA = 0.2;
double DEC_OMEGA = 0.5;
double STEP = 0.01;
double A_LAT = 2.0;

class CMDPublisher : public rclcpp::Node
{
  public:
    CMDPublisher()
    : Node("remote_control")
    {
      publisher_ = rclcpp::Node::create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      initscr();
      noecho();
      last = std::chrono::system_clock::now();
      timer_ = this->create_wall_timer(
      100ms, std::bind(&CMDPublisher::timer_callback, this));
    }

    private:
    void timer_callback()
    {
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
        v_c -= DEC_V*(v_c>0?1:-1) * dT;
        if(std::abs(v_c) < 0.005)v_c = 0;
      }
      if (std::abs(O_MAX) > std::abs(omega_c)) {
        omega_c += ACC_OMEGA*(omega_c==0?(O_MAX>0?1:-1):(omega_c>0?1:-1)) * dT;
        omega_c = O_MAX>0?std::min(omega_c, O_MAX):std::max(omega_c, O_MAX);
      } else if(std::abs(O_MAX) < std::abs(omega_c)) {
        omega_c -= DEC_OMEGA*(omega_c>0?1:-1) * dT;
        if(std::abs(omega_c) < 0.01)omega_c = 0;
      }

      clear();
      move(0,0);
      printw("*** USE q TO STOP THE REMOTE CONTROLLER ***\nUse 8, w or arrow up to move forward\nUse 2, x or arrow down to move backwards\nUse 4, a or arrow left to curve left\nUse 6, d or arrow right to curve right\nUse 5 or s to brake\n*******************************************\n");
      printw("V_MAX: %f O_MAX: %f\n", V_MAX, O_MAX);

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
};

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

void start_ros_node(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CMDPublisher>());
}

int main(int argc, char * argv[])
{
  pressed = std::make_unique<char>();

  std::thread remote_control(start_ros_node, argc, argv);
  std::thread key_listener(kbhit);

  remote_control.join();
  key_listener.join();

  return 0;
}

