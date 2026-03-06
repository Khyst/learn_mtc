#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode {
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  void loadParameters();

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  // Parameters
  std::string arm_group_;
  std::string hand_group_;
  std::string hand_frame_;
  std::string world_frame_;
  std::string object_name_;

  double obj_x_;
  double obj_y_;
  double obj_z_;

  double place_x_;
  double place_y_;
  double place_z_;

  double gripper_left_joint_;
  double gripper_right_joint_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) } 
{
  loadParameters();
}

void MTCTaskNode::loadParameters() {
  auto get_str = [this](const std::string& name, const std::string& def) {
    if (node_->has_parameter(name)) return node_->get_parameter(name).as_string();
    return node_->declare_parameter<std::string>(name, def);
  };
  auto get_dbl = [this](const std::string& name, double def) {
    if (node_->has_parameter(name)) return node_->get_parameter(name).as_double();
    return node_->declare_parameter<double>(name, def);
  };

  arm_group_ = get_str("arm_group", "fr3_arm");
  hand_group_ = get_str("hand_group", "fr3_hand");
  hand_frame_ = get_str("hand_frame", "fr3_hand");
  world_frame_ = get_str("world_frame", "world");
  object_name_ = get_str("object_name", "object");

  obj_x_ = get_dbl("obj_x", 0.55);
  obj_y_ = get_dbl("obj_y", 0.0);
  obj_z_ = get_dbl("obj_z", 0.2);

  place_x_ = get_dbl("place_x", 0.55);
  place_y_ = get_dbl("place_y", 0.3);
  place_z_ = get_dbl("place_z", 0.2);

  gripper_left_joint_ = get_dbl("gripper_left_joint", 0.0225);
  gripper_right_joint_ = get_dbl("gripper_right_joint", 0.0225);

  RCLCPP_INFO_STREAM(LOGGER, "Loaded parameters:");
  RCLCPP_INFO_STREAM(LOGGER, "  obj_x: " << obj_x_ << ", obj_y: " << obj_y_ << ", obj_z: " << obj_z_);
  RCLCPP_INFO_STREAM(LOGGER, "  place_x: " << place_x_ << ", place_y: " << place_y_ << ", place_z: " << place_z_);
  RCLCPP_INFO_STREAM(LOGGER, "  gripper: L=" << gripper_left_joint_ << ", R=" << gripper_right_joint_);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface() {
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene() {
  auto client = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  while (!client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) return;
    RCLCPP_INFO(LOGGER, "Waiting for /apply_planning_scene service...");
  }

  moveit::planning_interface::PlanningSceneInterface psi;

  // 대상 물체만 추가
  moveit_msgs::msg::CollisionObject object;
  object.id = object_name_;
  object.header.frame_id = world_frame_;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 }; 
  
  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = obj_x_;
  obj_pose.position.y = obj_y_;
  obj_pose.position.z = obj_z_;
  obj_pose.orientation.w = 1.0;
  object.primitive_poses.push_back(obj_pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  psi.applyCollisionObject(object);

  RCLCPP_INFO(LOGGER, "Planning scene setup complete. (Object only)");
}

mtc::Task MTCTaskNode::createTask() {
  mtc::Task task;
  task.stages()->setName("panda pick & place (No Plate)");
  task.loadRobotModel(node_);

  task.setProperty("group",    arm_group_);
  task.setProperty("eef",      hand_group_);
  task.setProperty("hand",     hand_group_);
  task.setProperty("ik_frame", hand_frame_);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setStepSize(0.005);

  // 블록 외부에서 먼저 선언해야 나중에 'place' 단계에서도 접근 가능합니다.
  mtc::Stage* pick_stage_ptr = nullptr;

  // 1. Current State
  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  // 2. Open Hand
  mtc::Stage* open_hand_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_);
    stage->setGoal("open");
    open_hand_ptr = stage.get();
    task.add(std::move(stage));
  }

  // 3. Move to Pick
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_, sampling_planner } });
    stage->setTimeout(10.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // ============================================================
  // 4. Pick Object (SerialContainer)
  // ============================================================
  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(pick->properties(), { "eef", "hand", "group", "ik_frame" });
    pick->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

    // 4-1. Approach Object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.15);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    // 4-2. Generate Grasp Pose & IK
    {
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->setPreGraspPose("open");
        stage->setObject(object_name_);
        stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(open_hand_ptr);

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(20);
        wrapper->setMinSolutionDistance(0.1);

        Eigen::Isometry3d grasp_frame_transform =
            Eigen::Translation3d(0, 0, 0.13) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

        wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        pick->insert(std::move(wrapper));
    }

    // 4-3. Allow Collision (hand ↔ object) — IK 이후, Close Hand 이전에 배치
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          object_name_,
          task.getRobotModel()->getJointModelGroup(hand_group_)->getLinkModelNamesWithCollisionGeometry(),
          true);
      pick->insert(std::move(stage));
    }

    // 4-4. Close Hand
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        stage->setGroup(hand_group_);
        std::map<std::string, double> grasp_goal;
        grasp_goal["fr3_finger_joint1"] = gripper_left_joint_;
        grasp_goal["fr3_finger_joint2"] = gripper_right_joint_;
        stage->setGoal(grasp_goal);
        pick->insert(std::move(stage));
    }

    // 4-5. Attach & Lift
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name_, hand_frame_);
      pick->insert(std::move(stage));
    }

    // ---- [변경] 테이블과의 충돌 허용/금지 스테이지 삭제 ----

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.2);
      stage->setIKFrame(hand_frame_);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    pick_stage_ptr = pick.get();
    task.add(std::move(pick));
  }

  // 5. Move to Place
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group_, sampling_planner } });
    stage->setTimeout(10.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // 6. Place Object
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "hand", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

    {
      auto GPP = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      GPP->properties().configureInitFrom(mtc::Stage::PARENT);
      GPP->setObject(object_name_);
      GPP->setMonitoredStage(pick_stage_ptr); // Use pick_stage_ptr as monitored stage

      geometry_msgs::msg::PoseStamped target;
      target.header.frame_id = world_frame_;
      target.pose.position.x = place_x_;
      target.pose.position.y = place_y_;
      target.pose.position.z = place_z_; // Use PLACE_Z for target position
      target.pose.orientation.w = 1.0;
      GPP->setPose(target);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(GPP));
      wrapper->setMaxIKSolutions(4);
      
      // grasp와 동일한 IK 프레임 사용 (위에서 아래로 놓기)
      Eigen::Isometry3d place_frame_transform =
          Eigen::Translation3d(0, 0, 0.13) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
      wrapper->setIKFrame(place_frame_transform, hand_frame_);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name_, hand_frame_);
      place->insert(std::move(stage));
    }

    // 6-5. Retreat after place
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.2);
      stage->setIKFrame(hand_frame_);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  // 7. Return Home
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  return task;
}

void MTCTaskNode::doTask() {
  task_ = createTask();
  try { task_.init(); }
  catch (mtc::InitStageException& e) { // RCLCPP_ERROR_STREAM(node->get_logger(), e);
  return; }

  if (!task_.plan(5)) return;
  task_.introspection().publishSolution(*task_.solutions().front());
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with error code: " << result.val);
  } else {
    RCLCPP_INFO_STREAM(LOGGER, "Task execution succeeded!");
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  
  auto mtc_node = std::make_shared<MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spin_thread([&executor, mtc_node]() {
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
  });

  mtc_node->setupPlanningScene();
  mtc_node->doTask();

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}