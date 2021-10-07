#ifndef APRILTAG_SEARCH_APRILTAG_SEARCH_SERVER_H
#define APRILTAG_SEARCH_APRILTAG_SEARCH_SERVER_H

#include <apriltag_search/action/search.hpp>
#include <apriltag_search/visibility_control.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <functional>
#include <optional>
#include <memory>
#include <thread>
#include <sensor_msgs/msg/image.hpp>

namespace apriltag_search
{

class ApriltagSearchServer : public rclcpp::Node
{
  public:
    using SearchAction = apriltag_search::action::Search;
    using SearchGoalHandle = rclcpp_action::ServerGoalHandle<SearchAction>;
    using ImageMsg = sensor_msgs::msg::Image;

    CPP_PUBLIC
    explicit ApriltagSearchServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("apriltag_searcher", options)
    {
        using namespace std::placeholders;

        _action_server = rclcpp_action::create_server<SearchAction>(
            this,
            "apriltag_searcher",
            std::bind(&ApriltagSearchServer::HandleGoal, this, _1, std::placeholders::_2),
            std::bind(&ApriltagSearchServer::HandleCancel, this, std::placeholders::_1),
            std::bind(&ApriltagSearchServer::HandleAccepted, this, std::placeholders::_1));

        _image_subscriber = create_subscription<ImageMsg>()

    }

  private:

    std::optional<rclcpp_action::GoalUUID> _current_uuid;
    rclcpp::Subscription<ImageMsg> _image_subscriber;

    rclcpp_action::Server<SearchAction>::SharedPtr _action_server;

    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const SearchAction::Goal> goal);

    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<SearchGoalHandle> goal_handle);

    void HandleAccepted(const std::shared_ptr<SearchGoalHandle> goal_handle);

    void execute(const std::shared_ptr<SearchGoalHandle> goal_handle);

};  // class ApriltagSearchServer

}  // namespace apriltag_search

#endif  // APRILTAG_SEARCH_APRILTAG_SEARCH_SERVER_H
