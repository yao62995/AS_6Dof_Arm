/**
 * check collision node
 * author: yaojian <yao62995<yao_62995@163.com>>
 * last revise: 2016-11-10
**/

#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <as_arm_description/CheckCollisionValid.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

/*
rosservice call example:
rosservice call /check_collision 'values: [0, 0, 0, 0, 0, 0, 0]'
rosservice call /check_collision 'values: [1.5707970000000002, 1.4835305, -1.047198, -1.5707970000000002, -1.5707970000000002, 0, 0]'
*/


using std::string;
using std::vector;
using as_arm_description::CheckCollisionValid;
using planning_scene::PlanningScene;
using robot_state::RobotState;


class CheckCollisionNode {
public:
    CheckCollisionNode();

    void Init();

    ~CheckCollisionNode();

    bool CheckCollision(vector<double>& values);

    void ShowCurrentJoints();

private:
    vector<string> m_jointNames;
    vector<double> m_jointValues;
    PlanningScene* m_planScene;
    collision_detection::CollisionRequest m_collRequest;
    collision_detection::CollisionResult m_collResult;
};

CheckCollisionNode::CheckCollisionNode() {}

void CheckCollisionNode::Init() {
    // init robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    this->m_planScene = new PlanningScene(robot_model);
    // init collision
    m_collRequest.group_name = "arm";
    // init joints
    m_jointNames.push_back("base_joint");
    m_jointNames.push_back("shoulder_joint");
    m_jointNames.push_back("elbow_joint");
    m_jointNames.push_back("wrist_flex_joint");
    m_jointNames.push_back("wrist_rot_joint");
    m_jointNames.push_back("finger_joint1");
    m_jointNames.push_back("finger_joint2");
    m_jointValues.resize(m_jointNames.size());
}

CheckCollisionNode::~CheckCollisionNode() {
    delete this->m_planScene;
}

bool CheckCollisionNode::CheckCollision(vector<double>& values) {
    RobotState& robot_state = this->m_planScene->getCurrentStateNonConst();
    // format Float64MultiArray  to  vector<double>
    for (int i = 0; i < this->m_jointNames.size(); i++) {
        this->m_jointValues[i] = values[i];
        robot_state.setJointPositions(this->m_jointNames[i], &this->m_jointValues[i]);
    }
    // check
    m_collResult.clear();
    m_planScene->checkSelfCollision(this->m_collRequest, this->m_collResult);
    return this->m_collResult.collision;
}

void CheckCollisionNode::ShowCurrentJoints() {
    std::stringstream oss;
    oss << "Joint: [";
    for (int i = 0; i < this->m_jointNames.size(); i++) {
        string joint = this->m_jointNames[i];
        oss << joint << ":" << this->m_jointValues[i] << ", ";
    }
    oss << "]";
    ROS_INFO_STREAM(oss.str());
}

CheckCollisionNode handler;

bool CheckCollisionRef(CheckCollisionValid::Request& req, CheckCollisionValid::Response& res) {
    res.valid = handler.CheckCollision(req.values);
    // handler.ShowCurrentJoints();
    // ROS_INFO_STREAM("Joint Collision Status:" << res.valid);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "check_collision_node");
    ros::NodeHandle n;
    handler.Init();
    // define service
    ros::ServiceServer service = n.advertiseService("check_collision", CheckCollisionRef);

    ros::spin();


    return 0;
}