// ===================================================================
// nav_to_pose — 给 Go1 发导航目标的节点
// 就像一个遥控器：你告诉它"去(x, y)这个位置"，它帮你转告 Nav2
// ===================================================================
//
// 用法：
//   ros2 run unitree_nav nav_to_pose
//   ros2 service call /unitree_nav_to_pose unitree_nav_interfaces/srv/NavToPose "{x: 2.0, y: 1.0, yaw: 0.0}"
//   ros2 service call /unitree_cancel_nav std_srvs/srv/Empty
//
// 工作流程：
//   收到目标(x,y,角度)
//     → 发给 Nav2 Action Server "navigate_to_pose"
//       → Nav2 规划路径 + 控制机器人走过去
//         → 边走边打印当前位置
//           → 到达后打印"成功"
// ===================================================================

#include <exception>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"                          // ROS 2 基础库
#include "rclcpp_action/rclcpp_action.hpp"            // ROS 2 Action 客户端（异步任务）
#include "std_srvs/srv/empty.hpp"                     // 无参数 Service
#include "geometry_msgs/msg/quaternion.hpp"            // 四元数（表示旋转）
#include "tf2/LinearMath/Quaternion.h"                // 四元数 ↔ 欧拉角 转换工具
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"       // Nav2 的导航 Action
#include "unitree_nav_interfaces/srv/nav_to_pose.hpp"  // 我们自定义的导航 Service

// ---------- 状态机：四个状态转圈圈 ----------
//
//   IDLE ──收到目标──→ SEND_GOAL ──→ WAIT_FOR_GOAL_RESPONSE ──→ WAIT_FOR_MOVEMENT_COMPLETE ──→ IDLE
//    ↑                     ↑                    ↑                            ↑
//   闲着                发目标给Nav2       等Nav2确认收到                  等机器人走到

#define STATES \
X(IDLE, "IDLE") \
X(SEND_GOAL, "SEND_GOAL") \
X(WAIT_FOR_GOAL_RESPONSE, "WAIT_FOR_GOAL_RESPONSE") \
X(WAIT_FOR_MOVEMENT_COMPLETE, "WAIT_FOR_MOVEMENT_COMPLETE")

// 这一小段宏技巧是把 enum 值和它的名字（字符串）绑定在一起，方便打印日志
#define X(state, name) state,
enum class State : size_t {STATES};   // 生成: IDLE, SEND_GOAL, ...
#undef X

#define X(state, name) name,
std::vector<std::string> STATE_NAMES = {STATES};  // 生成: "IDLE", "SEND_GOAL", ...
#undef X

// 工具函数：把 enum 转成数字（用来从 STATE_NAMES 里取字符串）
template <typename Enumeration>
auto to_value(Enumeration const value)
  -> typename std::underlying_type<Enumeration>::type
{
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

auto get_state_name(State state) {
  return STATE_NAMES[to_value(state)];
}

// 前向声明：四元数 ↔ 欧拉角 互转（实现在文件最下面）
std::tuple<double, double, double> quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q);
geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);

using namespace std::chrono_literals;

// ==============================
// NavToPose 节点类
// ==============================
class NavToPose : public rclcpp::Node
{
public:
  NavToPose()
  : Node("nav_to_pose")   // 节点名
  {
    // --- 参数：目标位姿用哪个坐标系（默认 map）---
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    param.description = "The frame in which poses are sent.";
    declare_parameter("pose_frame", "map", param);
    goal_msg_.pose.header.frame_id = get_parameter("pose_frame").get_parameter_value().get<std::string>();

    // --- 定时器：每 10ms 跑一次主循环（100Hz）---
    timer_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0)),
      std::bind(&NavToPose::timer_callback, this)
    );

    // --- Service 1：接收导航目标 ---
    srv_nav_to_pose_ = create_service<unitree_nav_interfaces::srv::NavToPose>(
      "unitree_nav_to_pose",
      std::bind(&NavToPose::srv_nav_to_pose_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    // --- Service 2：取消导航 ---
    srv_cancel_nav_ = create_service<std_srvs::srv::Empty>(
      "unitree_cancel_nav",
      std::bind(&NavToPose::cancel_nav_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    // --- Action 客户端：用来给 Nav2 发导航请求 ---
    act_nav_to_pose_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this,
      "navigate_to_pose"
    );

    RCLCPP_INFO_STREAM(get_logger(), "nav_to_pose node started");
  }

private:
  // --- 成员变量 ---
  rclcpp::TimerBase::SharedPtr timer_;                // 定时器
  rclcpp::Service<unitree_nav_interfaces::srv::NavToPose>::SharedPtr srv_nav_to_pose_;  // 导航 Service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_cancel_nav_;                      // 取消 Service
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr act_nav_to_pose_;  // Nav2 Action 客户端

  double rate_ = 100.0;              // 循环频率 100Hz
  double interval_ = 1.0 / rate_;    // 每次循环间隔 0.01 秒
  State state_ = State::IDLE;        // 当前状态
  State state_last_ = state_;        // 上一个状态（用来检测变化）
  State state_next_ = state_;        // 下一个状态
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal goal_msg_ {};  // 要发的目标
  bool goal_response_received_ = false;  // Nav2 是否回应了我们的请求
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_ {};
  std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback_ = nullptr;  // 导航过程中的实时反馈
  std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>
    result_ = nullptr;               // 导航最终结果

  // ==============================
  // 主循环：每秒跑 100 次
  // ==============================
  void timer_callback()
  {
    // 把"下一个状态"应用到"当前状态"
    state_ = state_next_;

    auto new_state = state_ != state_last_;
    if (new_state) {
      RCLCPP_INFO_STREAM(get_logger(), "nav_to_pose state changed to " << get_state_name(state_));
      state_last_ = state_;
    }

    // 根据当前状态做不同的事
    switch(state_) {
      // ----- 闲着：什么都不做 -----
      case State::IDLE:
      {
        break;
      }
      // ----- 发目标：把 (x, y, 角度) 发给 Nav2 -----
      case State::SEND_GOAL:
      {
        // 检查 Nav2 Action Server 是否在线
        if(act_nav_to_pose_->wait_for_action_server(0s)) {
          // 重置状态变量
          goal_response_received_ = false;
          goal_handle_ = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr {};
          result_ = nullptr;

          // 准备三种回调：Nav2 收到目标时、反馈进度时、完成时
          auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
          send_goal_options.goal_response_callback =
            std::bind(&NavToPose::goal_response_callback, this, std::placeholders::_1);
          send_goal_options.feedback_callback =
            std::bind(&NavToPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
          send_goal_options.result_callback =
            std::bind(&NavToPose::result_callback, this, std::placeholders::_1);

          // 发出导航请求！
          act_nav_to_pose_->async_send_goal(goal_msg_, send_goal_options);
          state_next_ = State::WAIT_FOR_GOAL_RESPONSE;
        } else {
          // Nav2 不在线，放弃
          RCLCPP_ERROR_STREAM(get_logger(), "Action server not available, aborting.");
          state_next_ = State::IDLE;
        }
        break;
      }
      // ----- 等回复：看 Nav2 有没有收下我们的目标 -----
      case State::WAIT_FOR_GOAL_RESPONSE:
      {
        if (goal_response_received_) {
          if (goal_handle_) {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            state_next_ = State::WAIT_FOR_MOVEMENT_COMPLETE;
          } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was rejected by server");
            state_next_ = State::IDLE;
          }
        }
        break;
      }
      // ----- 等走完：机器人正在路上，等它到终点 -----
      case State::WAIT_FOR_MOVEMENT_COMPLETE:
      {
        if (result_) {
          state_next_ = State::IDLE;  // 到了！回到空闲
        }
        break;
      }
      default:
        auto msg = "Unhandled state: " + get_state_name(state_);
        RCLCPP_ERROR_STREAM(get_logger(), msg);
        throw std::logic_error(msg);
        break;
    }
  }

  // ==============================
  // Service 回调：有人调了 /unitree_nav_to_pose
  // ==============================
  void srv_nav_to_pose_callback(
    const std::shared_ptr<unitree_nav_interfaces::srv::NavToPose::Request> request,
    std::shared_ptr<unitree_nav_interfaces::srv::NavToPose::Response>
  ) {
    // 把请求里的 x, y, 角度 填进目标消息
    goal_msg_.pose.pose.position.x = request->x;
    goal_msg_.pose.pose.position.y = request->y;
    goal_msg_.pose.pose.orientation = rpy_to_quaternion(0.0, 0.0, request->theta);

    // 触发状态机：去发目标
    state_next_ = State::SEND_GOAL;
  }

  // ==============================
  // Service 回调：有人调了 /unitree_cancel_nav
  // ==============================
  void cancel_nav_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    RCLCPP_INFO_STREAM(get_logger(), "Cancelling navigation.");
    act_nav_to_pose_->async_cancel_all_goals();  // 取消所有正在执行的导航
    state_next_ = State::IDLE;
  }

  // ==============================
  // Action 回调：Nav2 收到了我们的目标
  // ==============================
  void goal_response_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle
  ) {
    goal_response_received_ = true;
    goal_handle_ = goal_handle;
    RCLCPP_INFO_STREAM(get_logger(), "Goal response");
  }

  // ==============================
  // Action 回调：导航进行中，Nav2 定期汇报机器人在哪
  // ==============================
  void feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
  ) {
    feedback_ = feedback;

    if (feedback_) {
      // 打印当前位姿 (x, y, 角度)
      auto [roll, pitch, yaw] = quaternion_to_rpy(feedback_->current_pose.pose.orientation);
      RCLCPP_INFO_STREAM(get_logger(), "x = " << feedback_->current_pose.pose.position.x
                                  << ", y = " << feedback_->current_pose.pose.position.y
                                  << ", theta = " << yaw
      );
    }
  }

  // ==============================
  // Action 回调：导航结束了（成功 / 失败 / 被取消）
  // ==============================
  void result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result
  ) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");     // 到达了！
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");  // 失败了（比如被障碍卡住）
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled"); // 被人取消了
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    // 保存结果，主循环看到 result_ 不为空就会回到 IDLE
    result_ = std::make_shared<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>();
    *result_ = result;
  }
};

// ==============================
// 工具函数：四元数 → 欧拉角 (w, x, y, z → roll, pitch, yaw)
// ==============================
std::tuple<double, double, double> quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q) {
  tf2::Quaternion q_temp;
  tf2::fromMsg(q, q_temp);
  tf2::Matrix3x3 m(q_temp);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}

// ==============================
// 工具函数：欧拉角 → 四元数 (roll, pitch, yaw → w, x, y, z)
// ==============================
geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

// ==============================
// main：启动节点，原地转圈等回调
// ==============================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavToPose>());  // 阻塞，等回调被触发
  rclcpp::shutdown();
  return 0;
}
