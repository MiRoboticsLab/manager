#include <unistd.h>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_info_uploader/cyberdog_info_uploader.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("test_upload_log");
  auto ros_node = rclcpp::Node::make_shared("test_upload_log");
  cyberdog::manager::InfoUploader info_uploader(ros_node);
  sleep(3);
  auto send_log_file = [&]()
  {
    std::cout << "start call sending file" << std::endl;
    std::string response;
    info_uploader.UploadLog(response);
    std::cout << "response is " << response << std::endl;
  };
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ros_node);
  std::thread send_file_thread(send_log_file);
  executor.spin();
  send_file_thread.join();
  return 0;
}
