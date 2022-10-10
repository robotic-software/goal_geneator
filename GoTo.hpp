#include <string>
#include "engine/alice/alice_codelet.hpp"
#include "messages/differential_base.capnp.h"

namespace isaac {
  namespace examples {
    class GoTo : public alice::Codelet {

    public:
      
      void start() override;
      void tick() override;
    
      // Feedback about the last received goal
      ISAAC_PROTO_TX(Goal2FeedbackProto, feedback);
      // The target destination received
      ISAAC_PROTO_RX(Goal2Proto, goal);
      // Feedback will switch from "not arrived" to "arrived" after this many seconds.
      ISAAC_PARAM(double, time_until_arrival, 5.0);

    private:
        // Information about the desired goal
        struct Goal {
          // Acquisition time to which the goal relates
          int64_t acqtime;
          // The name of the goal coordinate frame
          std::string frame_name;
          // The pose of the goal in the goal coordinate frame
          Pose2d frame_T_goal;
        };

        // Reads input messages, updates goal timestamp, and starts timer for arrival.
        void processGoal();
        // Publishes feedback for the last goal received.
        void publishFeedback();

        // Save the last goal for two reasons:
        // 1. Goal acqtime is saved to publish feedback with correct timestamp,
        // 2. Pose and frame are saved to detect changes in goal location.
        std::optional<Goal> maybe_goal_;
};

}  // namespace examples
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::examples::GoTo);
