
#include "GoTo.hpp"

#include <string>
#include <iostream>
#include "messages/math.hpp"

namespace isaac {
namespace examples {

namespace {

// Stopwatch to change arrival status in the feedback
constexpr char kSleepHelper[] = "arrival-helper";

}  // namespace

void GoTo::start() {
  maybe_goal_ = std::nullopt;
  tickPeriodically();
}

void GoTo::tick() {
  publishFeedback();
  processGoal();
}

void GoTo::publishFeedback() {
  // Do not publish a feedback if no goal message is received yet.
  if (!maybe_goal_) {
    return;
  }

  // Check if we "arrived"
  bool arrived = false;
  if (stopwatch(kSleepHelper).read() > get_time_until_arrival()) {
    arrived = true;
    stopwatch(kSleepHelper).stop();
  }

  // Send a dummy feedback message
  auto proto_out = tx_feedback().initProto();
  proto_out.setHasGoal(true);
  ToProto(Pose2d::Identity(), proto_out.initRobotTGoal());
  proto_out.setHasArrived(arrived);
  proto_out.setIsStationary(false);
  tx_feedback().publish(maybe_goal_->acqtime);
}

void GoTo::processGoal() {
  // Process all messages received
  rx_goal().processAllNewMessages([this](auto proto, int64_t pubtime, int64_t acqtime) {
    // Process and print goal message that is received
    if (proto.getStopRobot()) {
      LOG_WARNING("Stopping the robot");
      return;
    }
    const Pose2d goal_pose = FromProto(proto.getGoal());
    const double goal_tolerance = proto.getTolerance();
    const std::string goal_frame_name = proto.getGoalFrame();
    const Goal new_goal = Goal{acqtime, goal_frame_name, goal_pose};
    // If the goal location has changed, print the new goal to console and restart the timer to
    // imitate "arrival". Note that we are ignoring the angle information in this mock-up.
    if (!maybe_goal_ || new_goal.frame_name != maybe_goal_->frame_name ||
        (new_goal.frame_T_goal.translation - maybe_goal_->frame_T_goal.translation).norm() >
            goal_tolerance) {
      LOG_INFO("Heading to a goal defined in '%s' frame: p=(%f, %f), q=%f", goal_frame_name.c_str(),
               goal_pose.translation.x(), goal_pose.translation.y(), goal_pose.rotation.angle());
      // Restart the timer. We arrive get_time_until_arrival() seconds after receiving a new goal
      // location.

      std::cout << "Processing: " <<  get_time_until_arrival() << std::endl;
      stopwatch(kSleepHelper).stop();
      stopwatch(kSleepHelper).start();
    }
    // Save the goal information
    maybe_goal_ = Goal{acqtime, goal_frame_name, goal_pose};
  });
}

}  // namespace tutorials
}  // namespace isaac
