#include "xc_motor_hardware/motor_config.hpp"

#include <cassert>
#include <iostream>
#include <string>
#include <vector>

using namespace xc_motor_hardware;

int main(int argc, char **argv)
{
  assert(argc >= 2 && "test_config 需要配置文件路径");
  std::string error;
  const auto config = load_hardware_config(argv[1], &error);
  assert(config && error.empty());
  std::vector<std::string> names;
  for (const auto &joint : config->joints) names.push_back(joint.joint_name);
  assert(config->validate(names, &error));
  assert(config->joints.size() == 5);
  assert(config->joints[3].gear_ratio == 4.2);
  assert(config->joints[0].motor_id == 34);
  std::cout << "config validation passed\n";
  return 0;
}
