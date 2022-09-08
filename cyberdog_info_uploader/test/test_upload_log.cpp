#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_info_uploader/cyberdog_info_uploader.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("test_upload_log");
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_upload_log");
  cyberdog::manager::InfoUploader info_uploader(node.get());
  sleep(3);
  std::cout << "start call sending file" << std::endl;
  std::string response;
  info_uploader.UploadLog(response);
  std::cout << "response is " << response << std::endl;
  rclcpp::spin(node);
  return 0;
}