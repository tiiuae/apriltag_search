#include <apriltag_search/apriltag_search_server.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace apriltag_search
{

using std::placeholders::_1;
using std::placeholders::_2;

rclcpp_action::GoalResponse ApriltagSearchServer::HandleGoal(const rclcpp_action::GoalUUID& uuid,
                                                             std::shared_ptr<const SearchAction::Goal> goal)
{
    const int n = std::min(goal->lats.size(), goal->longs.size());
    const int m = goal->tag_ids.size();
    RCLCPP_INFO(this->get_logger(), "Received goal request with %d GPS coordinates and %d Tag IDs", n, m);
    if(_current_uuid.has_value())
    {
        RCLCPP_ERROR(get_logger(), "Already have a goal!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    else
    {
        _current_uuid = uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
}

rclcpp_action::CancelResponse ApriltagSearchServer::HandleCancel(const std::shared_ptr<SearchGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ApriltagSearchServer::HandleAccepted(const std::shared_ptr<SearchGoalHandle> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ApriltagSearchServer::execute, this, _1), goal_handle}.detach();
}

void ApriltagSearchServer::execute(const std::shared_ptr<SearchGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SearchAction::Feedback>();
    // auto &sequence = feedback->partial_sequence;
    //    sequence.push_back(0);
    //    sequence.push_back(1);
    auto result = std::make_shared<SearchAction::Result>();

    //    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    //      // Check if there is a cancel request
    //      if (goal_handle->is_canceling()) {
    //        result->sequence = sequence;
    //        goal_handle->canceled(result);
    //        RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //        return;
    //      }
    //      // Update sequence
    //      sequence.push_back(sequence[i] + sequence[i - 1]);
    //      // Publish feedback
    //      goal_handle->publish_feedback(feedback);
    //      RCLCPP_INFO(this->get_logger(), "Publish feedback");
    //
    //      loop_rate.sleep();
    //    }

    // Check if goal is done
    if (rclcpp::ok())
    {
        // result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

}  // namespace apriltag_search

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_search::ApriltagSearchServer)