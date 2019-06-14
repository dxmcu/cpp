#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include "speech_synthesis/speech_application.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string node_name = "robot_speech_synthesis_node_name_";
  std::string topic_subscribe = "robot_speech_synthesis_topic_text";
  int ch;
  while ((ch = getopt(argc, argv, "n:s:")) != -1) {
    switch (ch) {
      case 'n':
        node_name = optarg;
        break;
      case 's':
        topic_subscribe = optarg;
        break;
      default:
        break;
    }
  }

  SpeechApplication speech;
  if (!speech.start(node_name, topic_subscribe)) {
    std::cout << "init failed." << std::endl;
    exit(-1);
  }

  while (true) {
    char c;
    std::cin.get(c);
    if (c == 'q') break;
    usleep(10000);
  }

  speech.stop();
  return 0;
}
