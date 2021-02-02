
#include "rclcpp/rclcpp.hpp"
#include <cstdio>

class Cup : public rclcpp::Node
{
};

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  printf("hello world cup package\n");
  return 0;
}
