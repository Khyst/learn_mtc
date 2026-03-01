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

const std::string ARM_GROUP = "panda_arm";
const std::string HAND_GROUP = "hand";
const std::string HAND_FRAME = "panda_hand";
const std::string WORLD_FRAME = "world";
const std::string OBJECT_NAME = "object";

// 물체 초기 위치 (테이블 위에서 약간 띄운 위치로 설정)
const double OBJ_X = 0.55; 
const double OBJ_Y = 0.0;
const double OBJ_Z = 0.2; // 0.05에서 0.2로 변경 (공중에 확실히 띄움)

// 내려놓을 위치
const double PLACE_X = 0.55;
const double PLACE_Y = 0.3;
const double PLACE_Z = 0.05;

class MTCTaskNode {
  """
    C++은 기본적으로 Structured Programming 언어이기 때문에, 미리 선언하는 것이 추천됨
  """
  public:

  // 생성자
  MTCTaskNode(const rclcpp::NodeOptions& options);
  
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  
  void setupPlanningScene();
  
  void doTask();

private:

  mtc::Task createTask();
  
  mtc::Task task_;

  rclcpp::Node::SharedPtr node_; // ROS2 노드는 인스턴스 내의 멤버 변수로써 관리 및 사용

};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) } {}

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
  object.id = OBJECT_NAME;
  object.header.frame_id = WORLD_FRAME;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 }; 
  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = OBJ_X;
  obj_pose.position.y = OBJ_Y;
  obj_pose.position.z = OBJ_Z;
  obj_pose.orientation.w = 1.0;
  object.primitive_poses.push_back(obj_pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  psi.applyCollisionObject(object);

  RCLCPP_INFO(LOGGER, "Planning scene setup complete. (Object only)");
}

void MTCTaskNode::doTask() {
  task_ = createTask();
  try { task_.init(); }
  catch (mtc::InitStageException& e) { return; }

  if (!task_.plan(5)) return;
  task_.introspection().publishSolution(*task_.solutions().front());
  task_.execute(*task_.solutions().front());
}

mtc::Task MTCTaskNode::createTask() {
  mtc::Task task;
  task.stages()->setName("panda pick & place (No Plate)");
  task.loadRobotModel(node_);

  task.setProperty("group",    ARM_GROUP);
  task.setProperty("eef",      HAND_GROUP);
  task.setProperty("hand",     HAND_GROUP);
  task.setProperty("ik_frame", HAND_FRAME);

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
    stage->setGroup(HAND_GROUP);
    stage->setGoal("open");
    open_hand_ptr = stage.get();
    task.add(std::move(stage));
  }

  // 3. Move to Pick
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{ { ARM_GROUP, sampling_planner } });
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
      stage->properties().set("link", HAND_FRAME);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.15);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = HAND_FRAME;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    // 4-2. Generate Grasp Pose & IK
    {
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->setPreGraspPose("open");
        stage->setObject(OBJECT_NAME);
        stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(open_hand_ptr);

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(20);
        wrapper->setMinSolutionDistance(0.1);

        Eigen::Isometry3d grasp_frame_transform =
            Eigen::Translation3d(0, 0, 0.13) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

        wrapper->setIKFrame(grasp_frame_transform, HAND_FRAME);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        pick->insert(std::move(wrapper));
    }

    // 4-3. Allow Collision (hand ↔ object) — IK 이후, Close Hand 이전에 배치
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          OBJECT_NAME,
          task.getRobotModel()->getJointModelGroup(HAND_GROUP)->getLinkModelNamesWithCollisionGeometry(),
          true);
      pick->insert(std::move(stage));
    }

    // 4-4. Close Hand
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        stage->setGroup(HAND_GROUP);
        std::map<std::string, double> grasp_goal;
        grasp_goal["panda_finger_joint1"] = 0.02;
        grasp_goal["panda_finger_joint2"] = 0.02;
        stage->setGoal(grasp_goal);
        pick->insert(std::move(stage));
    }

    // 4-5. Attach & Lift
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(OBJECT_NAME, HAND_FRAME);
      pick->insert(std::move(stage));
    }

    // ---- [변경] 테이블과의 충돌 허용/금지 스테이지 삭제 ----

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.2);
      stage->setIKFrame(HAND_FRAME);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = WORLD_FRAME;
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
        "move to place", mtc::stages::Connect::GroupPlannerVector{ { ARM_GROUP, sampling_planner } });
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
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "ik_frame" });
      stage->setObject(OBJECT_NAME);
      stage->setMonitoredStage(pick_stage_ptr);

      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = WORLD_FRAME;
      p.pose.position.x = PLACE_X; p.pose.position.y = PLACE_Y; p.pose.position.z = PLACE_Z;
      p.pose.orientation.w = 1.0;
      stage->setPose(p);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(4);
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation().z() = 0.03;
      wrapper->setIKFrame(tf, HAND_FRAME);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(HAND_GROUP);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(OBJECT_NAME, HAND_FRAME);
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

int main(int argc, char** argv) {
  
  rclcpp::init(argc, argv);

  // MoveIt과 같은 경우는 수많은 파라미터를 사용하기 때문에, declare_parmeter 로직을 하나씩 추가하지 않고 자동으로 파라미터를 로드하게 해주는 아래와 같은 옵션이 사용됨
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  
  // MTC를 구동하는 핵심 로직인 MoveIt Task Constructor Node 인스턴스 정의 (smart pointer를 통해서 MTC Task Node 인스턴스에 대한 주소 관리 용이)
  auto mtc_node = std::make_shared<MTCTaskNode>(options); // 위에서 정의한 옵션을 인자로 갖게 함

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