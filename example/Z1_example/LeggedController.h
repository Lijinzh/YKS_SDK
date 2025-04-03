//
// Created by qiayuan on 2022/6/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <legged_interface/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>
#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"
#include "std_msgs/Float64MultiArray.h"
#include <atomic>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
// #include "sensor_msgs/"
#include <dynamic_reconfigure/server.h>
#include "legged_controllers/TutorialsConfig.h"
#include <ocs2_robotic_tools/common/LoopshapingRobotInterface.h>
#include "legged_interface/foot_planner/InverseKinematics.h"

#include <deque>
#include <mutex>
#include <iostream>

#include <algorithm>
namespace legged
{
using namespace ocs2;
using namespace legged_robot;

//***************************start sjf-0310************** */
struct RLRobotCfg
  {
    struct ControlCfg
    {
      std::map<std::string, float> stiffness;
      std::map<std::string, float> damping;
      float actionScale;
      int decimation;
      float user_torque_limit;
      float user_power_limit;
      float cycle_time;
    };

    struct InitState
    {
      // default joint angles
      scalar_t leg_l1_joint;
      scalar_t leg_l2_joint;
      scalar_t leg_l3_joint;
      scalar_t leg_l4_joint;
      scalar_t leg_l5_joint;
      scalar_t leg_r1_joint;
      scalar_t leg_r2_joint;
      scalar_t leg_r3_joint;
      scalar_t leg_r4_joint;
      scalar_t leg_r5_joint;
    };

    struct ObsScales
    {
      scalar_t linVel;
      scalar_t angVel;
      scalar_t dofPos;
      scalar_t dofVel;
      scalar_t quat;
      scalar_t heightMeasurements;
    };

    bool encoder_nomalize;

    scalar_t clipActions;
    scalar_t clipObs;

    InitState initState;
    ObsScales obsScales;
    ControlCfg controlCfg;
  };

  struct JointState
  {
    scalar_t leg_l1_joint;
    scalar_t leg_l2_joint;
    scalar_t leg_l3_joint;
    scalar_t leg_l4_joint;
    scalar_t leg_l5_joint;
    scalar_t leg_r1_joint;
    scalar_t leg_r2_joint;
    scalar_t leg_r3_joint;
    scalar_t leg_r4_joint;
    scalar_t leg_r5_joint;
  };

  struct JoyInfo
  {
    float axes[8];
    int buttons[11];
  };

  struct Proprioception
  {
    vector_t jointPos;
    vector_t jointVel;
    vector3_t baseAngVel;
    vector3_t baseEulerXyz;
    // vector3_t baseAngZyx;  // base angular pos eular (zyx)
    // quaternion_t baseAngQuat;  // base angular pos quat (zyx)
    vector3_t projectedGravity;
  };
struct Command
{
  std::atomic<scalar_t> x;
  std::atomic<scalar_t> y;
  std::atomic<scalar_t> yaw;
};
//***************************end sjf-0310************** */

// class LeggedController
//   : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
//                                                           ContactSensorInterface>
class LeggedController
  : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                          ContactSensorInterface>
{
public:
  LeggedController() = default;
  ~LeggedController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override
  {
    mpcRunning_ = false;
  }
//***************************start sjf-0310************** */
  enum class Mode : uint8_t
  {
    LIE,
    STAND,
    WALK,
    PASSIVE,
    SQUAT,
    CALCU,
    TEST
  };

  virtual void handleSquatMode();
  virtual void handleStandMode();
  virtual void handlePassiveMode();
  virtual void handleTestMode();

//***************************end sjf-0310************** */
protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                    const std::string& referenceFile, bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  void dynamicParamCallback(legged_controllers::TutorialsConfig& config, uint32_t level);

  void setWalkCallback(const std_msgs::Float32::ConstPtr& msg);
  void loadControllerCallback(const std_msgs::Float32::ConstPtr& msg);
  void EmergencyStopCallback(const std_msgs::Float32::ConstPtr& msg);
  void CalcuCallback(const std_msgs::Float32::ConstPtr& msg);
  
  void modeSwitchCallback(const std_msgs::String::ConstPtr& msg);


  void ResetTargetCallback(const std_msgs::Float32::ConstPtr& msg);

  void resetMPC();
  void RetrievingParameters();
  void ModeSubscribe();

  ///gsj imu sub
  bool real_robot_flag;
  // sensor_msgs::ImuPtr imumsg_;
  hardware_interface::ImuSensorHandle imuSensorHandlefromMsg_;
  void FromImuMsg(const sensor_msgs::ImuConstPtr& imu_msg);
  std::mutex imulock;
  void IMUCallback(const sensor_msgs::ImuConstPtr& msg);
  ros::Subscriber subimu_;
  bool real_flag;

  double rosmsg_quat[4];
  contact_flag_t rosmsg_cmdContactFlag;
  vector3_t rosmsg_angularVel, rosmsg_linearAccel;
  matrix3_t rosmsg_orientationCovariance, rosmsg_angularVelCovariance, rosmsg_linearAccelCovariance;


  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;
  sensor_msgs::JointState joint_state_msg;
  sensor_msgs::JointState joint_cmd_msg;



  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<legged::LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;
  ros::Publisher joint_state_publisher_;
  ros::Publisher joint_cmd_publisher_;

  // Emergency stop
  ros::Subscriber subSetWalk_;
  ros::Subscriber subLoadcontroller_;
  ros::Subscriber subEmgstop_;
  ros::Subscriber subResetTarget_;
  ros::Subscriber subcalcu_;  // sjf
  ros::Subscriber subModeSwitch_;
  
  ros::Duration startingTime_;

  std::unique_ptr<dynamic_reconfigure::Server<legged_controllers::TutorialsConfig>> serverPtr_;
//***************************start sjf-0310************** */
  int actuatedDofNum_= 10;
  Mode mode_;
  int test_count;
  int64_t loopCount_;
  
  Command command_;
  RLRobotCfg robotCfg_{};
  JointState standjointState_{0.45, 0.0, 0.0,
                              0.85, 0.41,
                              -0.45, 0.00, 0.00,
                              0.85, -0.41};

  JointState SquatState_{0.95, 0.0, 0.0,
                           1.5, 0.65,
                          -0.95, 0.00, 0.00,
                           1.5, -0.65};

private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;

  bool loadControllerFlag_{ false };
  bool setWalkFlag_{ false };
  bool emergencyStopFlag_{ false };
  bool calcu_flag_{ false };
  std::atomic_bool firstStartMpc_{ false };

  // Initializing based on the number of joints
  vector_t posDes_ = vector_t::Zero(10);
  vector_t velDes_ = vector_t::Zero(10);
  vector_t TorDes_ = vector_t::Zero(10);
  vector_t posDesOutput_ = vector_t::Zero(10);
  vector_t velDesOutput_ = vector_t::Zero(10);

  size_t stateDim_{ 0 };
  size_t inputDim_{ 0 };
  size_t jointDim_{ 0 };
  size_t footDim_{ 0 };
  size_t gencoordDim_{ 0 };
  size_t dofPerLeg_{ 0 };

  std::atomic<scalar_t> kp_position{ 0 };
  std::atomic<scalar_t> kd_position{ 0 };
  std::atomic<scalar_t> kp_big_stance{ 0 };
  std::atomic<scalar_t> kp_big_swing{ 0 };
  std::atomic<scalar_t> kp_small_stance{ 0 };
  std::atomic<scalar_t> kp_small_swing{ 0 };
  std::atomic<scalar_t> kd_small{ 0 };
  std::atomic<scalar_t> kd_big{ 0 };
  std::atomic<scalar_t> kp_feet_stance{ 0 };
  std::atomic<scalar_t> kp_feet_swing{ 0 };
  std::atomic<scalar_t> kd_feet{ 0 };

  vector_t defalutJointPos_;

  InverseKinematics inverseKinematics_;

    //********************start sjf***************************
    vector_t pos_des_output_{};
    vector_t vel_des_output_{};
  
    size_t joint_dim_{0};
  
    std::vector<scalar_t> currentJointAngles_;
    vector_t standJointAngles_;
    vector_t squatJointAngles_;
    vector_t jointOffset_;
    
  
    scalar_t standPercent_;
    scalar_t swingPercent_;
    scalar_t standDuration_;
    scalar_t swingDuration_;
    tf::TransformBroadcaster tfBroadcaster_;
    bool standflag;
    bool left_leg_swing_flag;
    bool right_leg_swing_flag;
};

}  // namespace legged