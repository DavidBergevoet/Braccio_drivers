#include "Services.hpp"

#include <thread>
#include <chrono>

#define INIT_CLOSE_SPEED 3000  // ms
#define INIT_CLOSE_POSITION "straight"
#define BUFFER_SIZE 20

int main(int argc, char** argv)
{
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  if (argc < 2)
  {
    ROS_FATAL("Arg 1 is the path to the configfile");
    return -1;
  }
  ROS_INFO("%s", argv[1]);

  services::initHLI(argv[1]);
  std::this_thread::sleep_for(std::chrono::seconds(5));
  services::getHLI().addDefaultServos();

  services::getHLI().executePosition(INIT_CLOSE_POSITION, INIT_CLOSE_SPEED / 1000);

  ros::init(argc, argv, "RobotArmService");
  ros::NodeHandle n;
  ros::Subscriber servoSub = n.subscribe("braccio/servo", BUFFER_SIZE, services::moveServo);
  ros::Subscriber positionSub = n.subscribe("braccio/position", BUFFER_SIZE, services::moveToPosition);
  ros::Subscriber stopSub = n.subscribe("braccio/stop", BUFFER_SIZE, services::emergencyStop);
  ros::spin();
  services::getHLI().executePosition(INIT_CLOSE_POSITION, INIT_CLOSE_SPEED / 1000);
  std::this_thread::sleep_for(std::chrono::seconds(INIT_CLOSE_SPEED/1000));
  return 0;
}
