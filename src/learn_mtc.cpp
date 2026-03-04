#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

// TF2 관련 헤더 (버전 호환성 처리)
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode {

  public:
    // ──────────────────────────────────────────────
    // 1. 설정값 (연구 시 이 부분만 수정하면 됩니다)
    // ──────────────────────────────────────────────
    const std::string ARM_GROUP   = "panda_arm";
    const std::string HAND_GROUP  = "hand";
    const std::string HAND_FRAME  = "panda_hand";
    const std::string WORLD_FRAME = "world";
    const std::string OBJECT_NAME = "object";

    // 물체 및 목표 위치
    const double OBJ_X = 0.5;
    const double OBJ_Y = -0.25;
    const double PLACE_Y = 0.5;

    MTCTaskNode(const rclcpp::NodeOptions& options)
        : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) } 
    {
        // 1초 뒤에 executeSequence를 실행하는 타이머 (일회성)
        timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MTCTaskNode::executeSequence, this)
        );
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface() {
        return node_->get_node_base_interface();
    }

private:
    // ──────────────────────────────────────────────
    // 2. 전체 공정 관리 (Main Workflow)
    // ──────────────────────────────────────────────
    void executeSequence() {
        timer_->cancel(); // 실행 즉시 타이머 종료

        RCLCPP_INFO(LOGGER, "--- 시퀀스 시작 ---");
        
        setupPlanningScene(); // 1단계: 물체 배치
        doTask();             // 2단계: MTC 실행

        RCLCPP_INFO(LOGGER, "--- 모든 작업 완료 ---");
    }

    // ──────────────────────────────────────────────
    // 3. 환경 설정 (Planning Scene)
    // ──────────────────────────────────────────────
    void setupPlanningScene() {
        moveit_msgs::msg::CollisionObject object;
        object.id = OBJECT_NAME;
        object.header.frame_id = WORLD_FRAME;
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        object.primitives[0].dimensions = { 0.1, 0.02 }; // 높이 0.1, 반지름 0.02

        geometry_msgs::msg::Pose pose;
        pose.position.x = OBJ_X;
        pose.position.y = OBJ_Y;
        pose.orientation.w = 1.0;
        object.pose = pose;

        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObject(object);
        RCLCPP_INFO(LOGGER, "물체가 환경에 추가되었습니다.");
    }

    // ──────────────────────────────────────────────
    // 4. MTC 작업 정의 (Task Construction)
    // ──────────────────────────────────────────────
    mtc::Task createTask() {
        mtc::Task task;
        task.stages()->setName("Demo Pick and Place");
        task.loadRobotModel(node_);

        // 공통 속성 설정
        task.setProperty("group", ARM_GROUP);
        task.setProperty("eef", HAND_GROUP);
        task.setProperty("ik_frame", HAND_FRAME);

        // 플래너(Planner) 설정
        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setStepSize(0.01);

        // [Stage 1] 현재 상태
        auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
        mtc::Stage* current_state_ptr = current_state.get();
        task.add(std::move(current_state));

        // [Stage 2] 그리퍼 열기
        auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        open_hand->setGroup(HAND_GROUP);
        open_hand->setGoal("open");
        task.add(std::move(open_hand));

        // [Stage 3] 물체로 이동 (Connect)
        auto move_to_pick = std::make_unique<mtc::stages::Connect>(
            "move to pick", mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, sampling_planner}});
        move_to_pick->setTimeout(5.0);
        move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(move_to_pick));

        // [Stage 4] 물체 집기 (Pick Container)
        mtc::Stage* attach_stage_ptr = nullptr;
        {
            auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
            task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
            grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

            // 4-1. 접근 (Approach)
            auto approach = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
            approach->properties().set("link", HAND_FRAME);
            approach->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            approach->setMinMaxDistance(0.1, 0.15);
            geometry_msgs::msg::Vector3Stamped direction;
            direction.header.frame_id = HAND_FRAME;
            direction.vector.z = 1.0;
            approach->setDirection(direction);
            grasp->insert(std::move(approach));

            // 4-2. 집기 자세 생성 및 IK
            auto GGP = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            GGP->properties().configureInitFrom(mtc::Stage::PARENT);
            GGP->setPreGraspPose("open");
            GGP->setObject(OBJECT_NAME);
            GGP->setAngleDelta(M_PI / 12);
            GGP->setMonitoredStage(current_state_ptr);

            auto ik = std::make_unique<mtc::stages::ComputeIK>("grasp IK", std::move(GGP));
            ik->setMaxIKSolutions(8);
            ik->setMinSolutionDistance(1.0);
            ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            
            // 잡기 위치 오프셋 (Z축 10cm 지점)
            Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
            grasp_frame_transform.translation().z() = 0.1; 
            ik->setIKFrame(grasp_frame_transform, HAND_FRAME);
            grasp->insert(std::move(ik));

            // 4-3. 충돌 허용 및 닫기
            auto allow_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
            allow_collision->allowCollisions(OBJECT_NAME, 
                task.getRobotModel()->getJointModelGroup(HAND_GROUP)->getLinkModelNamesWithCollisionGeometry(), true);
            grasp->insert(std::move(allow_collision));

            auto close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
            close_hand->setGroup(HAND_GROUP);
            close_hand->setGoal("close");
            grasp->insert(std::move(close_hand));

            // 4-4. 물체 부착 및 들어올리기 (Lift)
            auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            attach->attachObject(OBJECT_NAME, HAND_FRAME);
            attach_stage_ptr = attach.get();
            grasp->insert(std::move(attach));

            auto lift = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            lift->setMinMaxDistance(0.1, 0.3);
            lift->setIKFrame(HAND_FRAME);
            geometry_msgs::msg::Vector3Stamped up_dir;
            up_dir.header.frame_id = WORLD_FRAME;
            up_dir.vector.z = 1.0;
            lift->setDirection(up_dir);
            grasp->insert(std::move(lift));

            task.add(std::move(grasp));
        }

        // [Stage 5] 내려놓기 (Place Container)
        {
            auto place = std::make_unique<mtc::SerialContainer>("place object");
            task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
            place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

            // 5-1. 내려놓을 자세 생성
            auto GPP = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
            GPP->properties().configureInitFrom(mtc::Stage::PARENT);
            GPP->setObject(OBJECT_NAME);
            GPP->setMonitoredStage(attach_stage_ptr);

            geometry_msgs::msg::PoseStamped target;
            target.header.frame_id = WORLD_FRAME;
            target.pose.position.x = OBJ_X;
            target.pose.position.y = PLACE_Y;
            target.pose.orientation.w = 1.0;
            GPP->setPose(target);

            auto ik = std::make_unique<mtc::stages::ComputeIK>("place IK", std::move(GPP));
            ik->setMaxIKSolutions(2);
            ik->setIKFrame(OBJECT_NAME);
            ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            place->insert(std::move(ik));

            // 5-2. 물체 분리 (Detach)
            auto open = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
            open->setGroup(HAND_GROUP);
            open->setGoal("open");
            place->insert(std::move(open));

            auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            detach->detachObject(OBJECT_NAME, HAND_FRAME);
            place->insert(std::move(detach));

            task.add(std::move(place));
        }

        // [Stage 6] 홈으로 복귀
        auto home = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        home->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        home->setGoal("ready");
        task.add(std::move(home));

        return task;
    }

    // ──────────────────────────────────────────────
    // 5. 실행 (Planning & Execution)
    // ──────────────────────────────────────────────
    void doTask() {
        task_ = createTask();

        try {
            task_.init();
        } catch (mtc::InitStageException& e) {
            RCLCPP_ERROR_STREAM(LOGGER, "MTC 초기화 실패: " << e.what());
            return;
        }

        if (!task_.plan(5)) {
            RCLCPP_ERROR(LOGGER, "계획 수립 실패");
            return;
        }

        task_.introspection().publishSolution(*task_.solutions().front());
        auto result = task_.execute(*task_.solutions().front());

        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(LOGGER, "실행 실패");
        }
    }

    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ──────────────────────────────────────────────
// 깔끔한 MAIN 함수
// ──────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_node = std::make_shared<MTCTaskNode>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mtc_node->getNodeBaseInterface());

    RCLCPP_INFO(LOGGER, "노드 스핀 시작...");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}