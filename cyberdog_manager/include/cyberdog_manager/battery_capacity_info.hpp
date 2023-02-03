#ifndef CYBERDOG_MANAGER_BATTERY_CAPATICY_INFO_HPP
#define CYBERDOG_MANAGER_BATTERY_CAPATICY_INFO_HPP


#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/bms_status.hpp"

namespace cyberdog
{
namespace manager
{

struct LedMode
{
  bool occupation;
  std::string client;
  uint8_t target;
  uint8_t mode;
  uint8_t effect;
  uint8_t r_value;
  uint8_t g_value;
  uint8_t b_value;
};

class BatteryCapacityInfoNode final
{
  using BCIN_CALLBACK = std::function<void ()>;
  using BCINSOC_CALLBACK = std::function<void (uint8_t val)>;
public:
  explicit BatteryCapacityInfoNode(rclcpp::Node::SharedPtr node_ptr, BCINSOC_CALLBACK soc_cb,\
    BCIN_CALLBACK protect_cb, BCIN_CALLBACK lowpower_cb,\
    BCIN_CALLBACK active_cb, BCIN_CALLBACK shutdown_cb)
  : battery_capacity_info_node_(node_ptr_), socnotify_handler(soc_cb),
    active_handler(active_cb), protect_handler(protect_cb),
    lowpower_handler(lowpower_cb), shutdown_handler(shutdown_cb)
  {
    bc_callback_group_ =battery_capacity_info_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback = bc_callback_group;

    bms_status_sub_ = battery_capacity_info_node_->create_subscription<protocol::msg::BmsStatus>(
      "bms_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&BatteryCapacityInfoNode::BmsStatus, this, std::placeholders::_1), sub_options);

    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = bc_callback_group_;
    audio_play_extend_pub =
      battery_capacity_info_node_->create_publisher<protocol::msg::AudioPlayExtend>(
      "speech_play_extend", rclcpp::SystemDefaultsQoS(), pub_options);

  }

  void Init()
  {

  }

  void ShutdownHandler(BCIN_CALLBACK callback)
  {
    shutdown_handler = callback;
  }

  void LowpowerHandler(BCIN_CALLBACK callback)
  {
    lowpower_handler = callback;
  }

  void ProtectHandler(BCIN_CALLBACK callback)
  {
    protect_handler = callback;
  }

  void ActiveHandler(BCIN_CALLBACK callback)
  {
    active_handler = callback;
  }


private:
  private:
  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
socnotify_handler(bcin_soc),
    static bool is_exec_zero = false;
    static bool is_exec_five = false;
    static bool is_exec_twenty = false;
    static bool is_exec_thirty = false;
    
    bms_status_ = *msg;
    static int pre_bms_soc = bms_status.batt_volt;

    INFO_MILLSECONDS(30000, "Battery Capacity Info:%d", bms_status_.batt_soc);

    if (bms_status.batt_soc == 0 && !power_wired_charging) {
      if (!is_exec_zero) {
        SetLed(bms_status.batt_soc);
        shutdown_handler();
        std::string text{"电量为0,关机中"};
        AudioPrompts(text);
        return;
      }
    }

    if (bms_status.batt_soc < 5) {
      if (!is_exec_five) {
        is_exe_five = true;
        is_exe_twity = false;
        SetLed(bms_status.batt_soc);
        lowpower_handler();
        if (bms_status.batt_volt < pre_bms_soc) {
            std::string text{"电量低于5%，电池即将耗尽，请尽快充电!"};
            AudioPrompts(text);
        }
      }    
    } else if (bms_status.batt_soc < 20) {
        if (!is_exec_twity) {
          is_exe_five = false;
          is_exe_twity = true;
          is_exe_twity = false;
          SetLed(bms_status.batt_soc);
          protect_handler();
          if (bms_status.batt_volt < pre_bms_soc) {
            std::string text{"电量低于20%，部分功能受限"};
            AudioPrompts(text);
          }
        }
    } else if (bms_status.batt_volt < 30) {
      if (!is_exec_thirty) {
        is_exec_twenty = false;
        is_exec_thirty = true;
        SetLed(bms_status.batt_soc)
        active_handler();
        if (bms_status.batt_volt < pre_bms_soc) {
          std::string text{"电量低于30%，请尽快充电"};
          AudioPrompts(text);
        }
      }
    } else {
        is_exec_thirty = false;
      }
    pre_bms_soc = bms_status.batt_volt;
  }

  viod AudioPrompts(std::string & text)
  {
    protocol::msg::AudioPlayExtend msg;
    msg.is_online = true;
    msg.module_name = battery_capacity_info_node_->get_name();
    msg.text = text;
    audio_play_extend_pub->publish(msg);
  }

  void SetLed(int soc)
  {
    LedMode head_tail{true, "bms", 1, 0x02, 0x09, 0xFF, 0x32, 0x32};
    LedMode mini{true, "bms", 3, 0x02, 0x30, 0xFF, 0x32, 0x32};

    if (soc == 0) {
      // 电量为0，红灯快闪
      head_tail.mode = 0x01;
      head_tail.effect = 0xA3;
      mimi.mode = 0x01;
      mini.effect = 0x31;
      ReqLedService(LedMode & head_tail, LedMode & mini)
    } else if (soc <= 20) {
      // 电量低于20,占用灯效
      ReqLedService(LedMode & head_tail, LedMode & mini)
    } else {
      // 电量大于20,释放灯效占用
      head_tail.occupation = false;
      mini.occupation = false;
      ReqLedService(LedMode & head_tail, LedMode & mini)
    }
  }

  void ReqLedService(LedMode & head_tail, LedMode & mini)
  {
    if (!led_excute_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call led_excute server not avalible");
      return false;
    }

    auto request_led = std::make_shared<protocol::srv::LedExecute::Request>();
    ReqAssignment(request_led, head_tail);
    auto future_result_head = led_excute_client_->async_send_request(request_led);
    std::future_status status_head = future_result_head.wait_for(std::chrono::seconds(2));

    request_led->target = 2
    auto future_result_tail = led_excute_client_->async_send_request(request_led);
    std::future_status status_head = future_result_head.wait_for(std::chrono::seconds(2));

    ReqAssignment(request_led, mini);
    auto future_result_mini = led_excute_client_->async_send_request(request_led);
    std::future_status status_head = future_result_head.wait_for(std::chrono::seconds(2));

    if (status_head != std::future_status::ready ||
      status_tail != std::future_status::ready ||
      status_mini != std::future_status::ready)
    {
      ERROR("call led_execute service failed");
    }

    if (future_result_head.get()->code == 0 &&
      future_result_tail.get()->code == 0 &&
      future_result_mini.get()->code == 0)
    {
      INFO("call led service successed");
    } else {
      ERROR(
        "call led service fialed, error code[head, tail, mini] is:%d %d %d",
        future_result_head.get()->code,
        future_result_tail.get()->code,
        future_result_mini.get()->code);
    }

  }

  void ReqAssignment(std::shared_ptr<protocol::srv::LedExecute::Request> req, LedMode & data)
  {
    req->occupation = data.occupation;
    req->client = data.client;
    req->target = data.target;
    req->mode = data.mode;
    req->effect = data.effect;
    req->r_value = data.r_value;
    req->g_value = data.g_value;
    req->b_value = data.b_value;
  }

private:
    rclcpp::Node::SharedPtr battery_capacity_info_node_ {nullptr};
    rclcpp::CallBackGroup::SharedPtr bc_callback_group_;
    rclcpp::Subscription<protocol::msg::bms_status>::SharedPtr_;
    rclcpp::rclcpp::rclcpp::Publisher<protocol::msg::AudioPlayExtend>::SharedPtr audio_play_extend_pub_;
    BCINSOC_CALLBACK socnotify_handler,
    BCIN_CALLBACK shutdown_handler;
    BCIN_CALLBACK protect_handler;
    BCIN_CALLBACK lowpower_handler;
    BCIN_CALLBACK active_handler;
    BCIN_CALLBACK shutdown_handler;
    protocol::msg::BmsStatus bms_status_;
    
};
}
}

endif CYBERDOG_MANAGER_BATTERY_CAPATICY_INFO_HPP