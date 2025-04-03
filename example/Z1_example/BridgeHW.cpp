#include "legged_bridge_hw/BridgeHW.h"
#include "std_msgs/Float64MultiArray.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <cmath>

namespace legged
{
bool BridgeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  root_nh.setParam("gsmp_controller_switch", "null");
  char *ifconfig_name = "enp8s0";
  EtherCAT_Init(ifconfig_name);
  // printf("xxxxxxxxx %d\r\n", ec_slavecount);
  std::cout << "开始EtherCAT初始化" << std::endl;
  if (ec_slavecount <= 0)
  {
    std::cout << "未找到从站，程序退出" << std::endl;
    exit(1);
    return false;
  }
  else{
    printf("从站数量： %d\r\n", ec_slavecount);
    startRun();
  }
  // EtherCAT_Send_Command((YKSMotorData*)yksSendcmdzero_);
  
  // EtherCAT_Get_State();
  if (!LeggedHW::init(root_nh, robot_hw_nh))
    return false;


  robot_hw_nh.getParam("power_limit", powerLimit_);

  // subimu_ = ros::NodeHandle().subscribe("/imu/data",2000,&BridgeHW::IMUCallback,this);
  yesense_imu_sub_ = root_nh.subscribe("/imu/data",2000,&BridgeHW::IMUCallback,this);
  setupJoints();
  // setupImu();

  // setupImuXS();
  // printf("xxxxxxxxx %d\r\n", ec_slavecount);



  return true;
}


void BridgeHW::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    // imulock.lock();
    // FromImuMsg(imu_msg);
    // imulock.unlock();

      // double orientation[4];
  imuData_temp.ori[0] = imu_msg->orientation.x;
  imuData_temp.ori[1] = imu_msg->orientation.y;
  imuData_temp.ori[2] = imu_msg->orientation.z;
  imuData_temp.ori[3] = imu_msg->orientation.w;

  // std::cout<<"orientation is:"<<orientation[0]<<std::endl;

  // double orientation_covariance[9];
  for(int i = 0;i<9;i++ )
  {
    imuData_temp.ori_cov[i] = imu_msg->orientation_covariance[i];
  }

  // double angular_velocity[3];
  imuData_temp.angular_vel[0] = imu_msg->angular_velocity.x;
  imuData_temp.angular_vel[1] = imu_msg->angular_velocity.y;
  imuData_temp.angular_vel[2] = imu_msg->angular_velocity.z;

  // double angular_velocity_covariance[9];
  for(int i = 0;i<9;i++ )
  {
    imuData_temp.angular_vel_cov[i] = imu_msg->angular_velocity_covariance[i];
  }

  // double linear_acceleration[3];
  imuData_temp.linear_acc[0] = imu_msg->linear_acceleration.x;
  imuData_temp.linear_acc[1] = imu_msg->linear_acceleration.y;
  imuData_temp.linear_acc[2] = imu_msg->linear_acceleration.z;

  // double linear_acceleration_covariance[9];
  for(int i = 0;i<9;i++ )
  {
    imuData_temp.linear_acc_cov[i] = imu_msg->linear_acceleration_covariance[i];
  }
}


void BridgeHW::FromImuMsg(const sensor_msgs::ImuConstPtr& imu_msg)
{
  // double orientation[4];
  orientation[0] = imu_msg->orientation.x;
  orientation[1] = imu_msg->orientation.y;
  orientation[2] = imu_msg->orientation.z;
  orientation[3] = imu_msg->orientation.w;

  // std::cout<<"orientation is:"<<orientation[0]<<std::endl;

  // double orientation_covariance[9];
  for(int i = 0;i<9;i++ )
  {
    orientation_covariance[i] = imu_msg->orientation_covariance[i];
  }

  // double angular_velocity[3];
  angular_velocity[0] = imu_msg->angular_velocity.x;
  angular_velocity[1] = imu_msg->angular_velocity.y;
  angular_velocity[2] = imu_msg->angular_velocity.z;

  // double angular_velocity_covariance[9];
  for(int i = 0;i<9;i++ )
  {
    angular_velocity_covariance[i] = imu_msg->angular_velocity_covariance[i];
  }

  // double linear_acceleration[3];
  linear_acceleration[0] = imu_msg->linear_acceleration.x;
  linear_acceleration[1] = imu_msg->linear_acceleration.y;
  linear_acceleration[2] = imu_msg->linear_acceleration.z;

  // double linear_acceleration_covariance[9];
  for(int i = 0;i<9;i++ )
  {
    linear_acceleration_covariance[i] = imu_msg->linear_acceleration_covariance[i];
  }

  // imulock.lock();
  imumark = true;
  imumsg_.clear();
  imumsg_.push_back(orientation);
  imumsg_.push_back(orientation_covariance);
  imumsg_.push_back(angular_velocity);
  imumsg_.push_back(angular_velocity_covariance);
  imumsg_.push_back(linear_acceleration);
  imumsg_.push_back(linear_acceleration_covariance);
  // imulock.unlock();

}



void BridgeHW::read(const ros::Time& time, const ros::Duration& period)
{
  YKSMotorData left_pr_motor[2] = {motorDate_recv[4],motorDate_recv[5]};
  YKSMotorData right_pr_motor[2] = {motorDate_recv[10],motorDate_recv[11]};
  EncosMotorData left_pitch_joint_state,right_pitch_joint_state;
  left_pitch_joint_state = Pitch_forwardkinematics(left_pr_motor);
  right_pitch_joint_state = Pitch_forwardkinematics(right_pr_motor);
  // EtherCAT_Get_State();
  for (int i = 0; i < 4; i++)
  {
    jointData_[i].pos_ = (motorDate_recv[i].pos_ - baseMotor_[i]) * state_directionMotor_[i];
    jointData_[i].vel_ = motorDate_recv[i].vel_ * state_directionMotor_[i];
    jointData_[i].tau_ = motorDate_recv[i].tau_ * state_directionMotor_[i];
  }
    jointData_[4].pos_ = left_pitch_joint_state.pos_;
    jointData_[4].vel_ = left_pitch_joint_state.vel_;
    jointData_[4].tau_ = left_pitch_joint_state.tau_;

    // jointData_[4].pos_ = (motorDate_recv[4].pos_ * state_directionMotor_[4]+motorDate_recv[5].pos_ * state_directionMotor_[5])/2.0;
    // jointData_[4].vel_ = motorDate_recv[4].vel_ * state_directionMotor_[4];
    // jointData_[4].tau_ = motorDate_recv[4].tau_ * state_directionMotor_[4]+motorDate_recv[5].tau_ * state_directionMotor_[5];
  for (int i = 5; i < 9; i++)
  {
    jointData_[i].pos_ = (motorDate_recv[i+1].pos_ - baseMotor_[i+1]) * state_directionMotor_[i+1];
    jointData_[i].vel_ = motorDate_recv[i+1].vel_ * state_directionMotor_[i+1];
    jointData_[i].tau_ = motorDate_recv[i+1].tau_ * state_directionMotor_[i+1];
  }
    jointData_[9].pos_ = right_pitch_joint_state.pos_;
    jointData_[9].vel_ = right_pitch_joint_state.vel_;
    jointData_[9].tau_ = right_pitch_joint_state.tau_;

    // jointData_[9].pos_ = (motorDate_recv[10].pos_ * state_directionMotor_[10]+motorDate_recv[11].pos_ * state_directionMotor_[11])/2.0;
    // jointData_[9].vel_ = motorDate_recv[10].vel_ * state_directionMotor_[10];
    // jointData_[9].tau_ = motorDate_recv[10].tau_ * state_directionMotor_[10]+motorDate_recv[11].tau_ * state_directionMotor_[11];

    // std::cout<<"left_pitch_joint_state.pos_"<<left_pitch_joint_state.pos_<<'\n';
    // std::cout<<"right_pitch_joint_state.pos_"<<right_pitch_joint_state.pos_<<'\n';
    // std::cout<<"jointData_[4].pos_"<<jointData_[4].pos_<<'\n';
    // std::cout<<"jointData_[9].pos_"<<jointData_[9].pos_<<'\n';

    // right_PR_joint_state = PR_forwardkinematics(right_pr_motor);
  // for (int j = 0; j < 12; j++)
  // {
    // std::cout<<"motorDate_recv["<<j<<"].pos_:"<< motorDate_recv[j].pos_<<std::endl;
    // printf("motorDate_recv[i].pos_ %f\r\n", motorDate_recv[j].pos_);
  // }
  
  imuData_ = imuData_temp;
  // imuData_.ori[0] = imuData_recv.quat_float[2];          
  // imuData_.ori[1] = -imuData_recv.quat_float[1];
  // imuData_.ori[2] = imuData_recv.quat_float[3];
  // imuData_.ori[3] = imuData_recv.quat_float[0];
  // imuData_.angular_vel[0] = imuData_recv.gyro_float[1];  
  // imuData_.angular_vel[1] = -imuData_recv.gyro_float[0];
  // imuData_.angular_vel[2] = imuData_recv.gyro_float[2];
  // imuData_.linear_acc[0] = imuData_recv.accel_float[1];   
  // imuData_.linear_acc[1] = -imuData_recv.accel_float[0];
  // imuData_.linear_acc[2] = imuData_recv.accel_float[2];
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names)
  {
    
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.0);
    handle.setKp(0.);
  }
  

}

void BridgeHW::write(const ros::Time& time, const ros::Duration& period)
{
  YKSMotorData joint_cmd_temp_[12];
  YKSMotorData left_ankle_a,left_ankle_b, right_ankle_a, right_ankle_b;
  for (int i = 0; i < 10; ++i)
  {
    joint_cmd_temp_[i].pos_des_ = jointData_[i].pos_des_;
    joint_cmd_temp_[i].vel_des_ = jointData_[i].vel_des_;
    joint_cmd_temp_[i].kp_ = jointData_[i].kp_;
    joint_cmd_temp_[i].kd_ = jointData_[i].kd_;
    joint_cmd_temp_[i].ff_ = jointData_[i].ff_;
  }
  for( int j= 10;j<12;j++){
    joint_cmd_temp_[j].pos_des_ = 0.0;
    joint_cmd_temp_[j].vel_des_ = 0.0;
    joint_cmd_temp_[j].kp_ = 0.0;
    joint_cmd_temp_[j].kd_ = 0.0;
    joint_cmd_temp_[j].ff_ = 0.0;
  }
    // for(int j= 0;j<12;j++){
    //   std::cout<<"joint_cmd_temp_["<<j<<"].pos_des_: "<<joint_cmd_temp_[j].pos_des_<<std::endl;
    //   // std::cout<<"cmd_temp_["<<j<<"].kp_: "<<cmd_temp_[j].kp_<<std::endl;
    //   // std::cout<<"cmd_temp_["<<j<<"].kd_: "<<cmd_temp_[j].kd_<<std::endl;
    // }
  // zdx_note: original
  // EtherCAT_Send_Command((YKSMotorData*)yksSendcmd_);
  for(int i = 0; i<4;i++){
    // motorDate_recv[i].pos_des_ = DefaultJointState[i]*directionMotor_[i];

    motorDate_recv[i].pos_des_ = joint_cmd_temp_[i].pos_des_*directionMotor_[i];
    motorDate_recv[i].vel_des_ = joint_cmd_temp_[i].vel_des_*directionMotor_[i];
    motorDate_recv[i].ff_ = joint_cmd_temp_[i].ff_*directionMotor_[i];
    motorDate_recv[i].kp_ = joint_cmd_temp_[i].kp_;
    motorDate_recv[i].kd_ = joint_cmd_temp_[i].kd_;
  }
  for(int i = 6;i<10;i++){
    // motorDate_recv[i].pos_des_ = DefaultJointState[i]*directionMotor_[i];
    motorDate_recv[i].pos_des_ = joint_cmd_temp_[i-1].pos_des_*directionMotor_[i];
    motorDate_recv[i].vel_des_ = joint_cmd_temp_[i-1].vel_des_*directionMotor_[i];
    motorDate_recv[i].ff_ = joint_cmd_temp_[i-1].ff_*directionMotor_[i];
    motorDate_recv[i].kp_ = joint_cmd_temp_[i-1].kp_;
    motorDate_recv[i].kd_ = joint_cmd_temp_[i-1].kd_;
  }
  // //for(int i = 6;i<10;i++){
  // //  motorDate_recv[i].pos_des_ = DefaultJointState[i]*directionMotor_[i];
  // //}

  // joint_cmd_temp[10]和joint_cmd_temp[11]暂定为左腿ankle_roll 和右腿ankle_roll
  left_ankle_a = AnkleA_inversekinematics(joint_cmd_temp_[4],joint_cmd_temp_[10]);
  left_ankle_b = AnkleB_inversekinematics(joint_cmd_temp_[4],joint_cmd_temp_[10]);
  right_ankle_a = AnkleA_inversekinematics(joint_cmd_temp_[9],joint_cmd_temp_[11]);
  right_ankle_b = AnkleB_inversekinematics(joint_cmd_temp_[9],joint_cmd_temp_[11]);
  motorDate_recv[Z1JointIndex::LeftAnkleA] = left_ankle_a;
  motorDate_recv[Z1JointIndex::LeftAnkleB] = left_ankle_b;
  // motorDate_recv[Z1JointIndex::LeftAnkleA].pos_des_ = joint_cmd_temp_[4].pos_des_*directionMotor_[4];
  // motorDate_recv[Z1JointIndex::LeftAnkleB].pos_des_ = joint_cmd_temp_[4].pos_des_*directionMotor_[5];
  // motorDate_recv[Z1JointIndex::LeftAnkleB].vel_des_ = joint_cmd_temp_[4].vel_des_*directionMotor_[4];
  // motorDate_recv[Z1JointIndex::LeftAnkleB].vel_des_ = joint_cmd_temp_[4].vel_des_*directionMotor_[5];
  // motorDate_recv[Z1JointIndex::LeftAnkleB].ff_ = (joint_cmd_temp_[4].ff_/2.0)*directionMotor_[4];
  // motorDate_recv[Z1JointIndex::LeftAnkleB].ff_ = (joint_cmd_temp_[4].ff_/2.0)*directionMotor_[5];

  motorDate_recv[Z1JointIndex::LeftAnkleA].kp_ = joint_cmd_temp_[4].kp_;
  motorDate_recv[Z1JointIndex::LeftAnkleB].kp_ = joint_cmd_temp_[4].kp_;
  motorDate_recv[Z1JointIndex::LeftAnkleA].kd_ = joint_cmd_temp_[4].kd_;
  motorDate_recv[Z1JointIndex::LeftAnkleB].kd_ = joint_cmd_temp_[4].kd_;

  motorDate_recv[Z1JointIndex::RightAnkleA] = right_ankle_a;
  motorDate_recv[Z1JointIndex::RightAnkleB] = right_ankle_b;

  // motorDate_recv[Z1JointIndex::RightAnkleA].pos_des_ = joint_cmd_temp_[9].pos_des_*directionMotor_[10];
  // motorDate_recv[Z1JointIndex::RightAnkleB].pos_des_ = joint_cmd_temp_[9].pos_des_*directionMotor_[11];
  // motorDate_recv[Z1JointIndex::RightAnkleA].vel_des_ = joint_cmd_temp_[9].vel_des_*directionMotor_[10];
  // motorDate_recv[Z1JointIndex::RightAnkleB].vel_des_ = joint_cmd_temp_[9].vel_des_*directionMotor_[11];
  // motorDate_recv[Z1JointIndex::RightAnkleA].ff_ = (joint_cmd_temp_[9].ff_/2.0)*directionMotor_[10];
  // motorDate_recv[Z1JointIndex::RightAnkleB].ff_ = (joint_cmd_temp_[9].ff_/2.0)*directionMotor_[11];

  motorDate_recv[Z1JointIndex::RightAnkleA].kp_ = joint_cmd_temp_[9].kp_;
  motorDate_recv[Z1JointIndex::RightAnkleB].kp_ = joint_cmd_temp_[9].kp_;
  motorDate_recv[Z1JointIndex::RightAnkleA].kd_ = joint_cmd_temp_[9].kd_;
  motorDate_recv[Z1JointIndex::RightAnkleB].kd_ = joint_cmd_temp_[9].kd_;


  // std::cout<<"---------------------------------------------------"<<'\n';
  // std::cout<<"left_ankle_a.pos_des_"<<left_ankle_a.pos_des_<<'\n';
  // std::cout<<"left_ankle_b.pos_des_"<<left_ankle_b.pos_des_<<'\n';
  // std::cout<<"right_ankle_a.pos_des_"<<right_ankle_a.pos_des_<<'\n';
  // std::cout<<"right_ankle_b.pos_des_"<<right_ankle_b.pos_des_<<'\n';

  for(int j = 0; j<12; j++){
    // motorDate_recv[j].pos_des_ = 0.0;
    // motorDate_recv[j].vel_des_ = 0.0;
    // motorDate_recv[j].ff_ = 0.0;
    // motorDate_recv[j].kp_ = 0.0;
    // motorDate_recv[j].kd_ = 3.0;
    std::cout<<"---------------------------------------------------"<<'\n';
    std::cout<<"motorDate_recv["<<j<<"].pos_des_: "<<motorDate_recv[j].pos_des_<<'\n';
    std::cout<<"motorDate_recv["<<j<<"].vel_des: "<<motorDate_recv[j].vel_des_<<'\n';
    std::cout<<"motorDate_recv["<<j<<"].kp_: "<<motorDate_recv[j].kp_<<'\n';
    std::cout<<"motorDate_recv["<<j<<"].kd_: "<<motorDate_recv[j].kd_<<'\n';
    std::cout<<"motorDate_recv["<<j<<"].ff_: "<<motorDate_recv[j].ff_<<'\n';
  }


}

bool BridgeHW::setupJoints()
{
  for (const auto& joint : urdfModel_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("leg_l") != std::string::npos)
    {
      leg_index = 0;
    }
    else if (joint.first.find("leg_r") != std::string::npos)
    {
      leg_index = 1;
    }
    else
      continue;
    if (joint.first.find("3_joint") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("1_joint") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("2_joint") != std::string::npos)
      joint_index = 2;
    else if (joint.first.find("4_joint") != std::string::npos)
      joint_index = 3;
    else if (joint.first.find("5_joint") != std::string::npos)
      joint_index = 4;
    else
      continue;

    int index = leg_index * 5 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].pos_des_,
                                                           &jointData_[index].vel_des_, &jointData_[index].kp_,
                                                           &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool BridgeHW::setupImu()
{
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu_link", "imu_link", imuData_.ori, imuData_.ori_cov, imuData_.angular_vel, imuData_.angular_vel_cov,
      imuData_.linear_acc, imuData_.linear_acc_cov));
  imuData_.ori_cov[0] = 0.0012;
  imuData_.ori_cov[4] = 0.0012;
  imuData_.ori_cov[8] = 0.0012;

  imuData_.angular_vel_cov[0] = 0.0004;
  imuData_.angular_vel_cov[4] = 0.0004;
  imuData_.angular_vel_cov[8] = 0.0004;

  return true;
}


bool BridgeHW::setupImuXS()
{

  double orientation_[4] = {0,0,0,0};
  double orientation_covariance_[9] = {0,0,0,0,0,0,0,0,0};
  double angular_velocity_[3] = {0,0,0};
  double angular_velocity_covariance_[9] = {0,0,0,0,0,0,0,0,0};
  double linear_acceleration_[3] = {0,0,0};
  double linear_acceleration_covariance_[9] = {0,0,0,0,0,0,0,0,0};

  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu_link", "imu_link", orientation_, orientation_covariance_, angular_velocity_, angular_velocity_covariance_,
      linear_acceleration_,linear_acceleration_covariance_));

  // imulock.lock();

  // imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
  //     "imu_link", "imu_link", orientation, orientation_covariance, angular_velocity, angular_velocity_covariance,
  //     linear_acceleration,linear_acceleration_covariance));


  // imulock.unlock();

  return true;
}
EncosMotorData BridgeHW::Pitch_forwardkinematics(YKSMotorData Ankle_motors[2]){
  EncosMotorData pitch_joint_data;
    const std::vector<int> PR_directionMotor_ = {1,-1};
    const double theta1 = Ankle_motors[0].pos_ * PR_directionMotor_[0];
    const double theta2 = Ankle_motors[1].pos_ * PR_directionMotor_[1];

  // 平行四边形机构正解（基于几何关系）
  pitch_joint_data.pos_ = (theta1 + theta2) / 2.0;
    // PR_joint_data[1].pos_des_  = (theta1 - theta2) / 2.0;

  // 速度映射（考虑减速比）
    const double dtheta1 = Ankle_motors[0].vel_*PR_directionMotor_[0] ;
    const double dtheta2 = Ankle_motors[1].vel_*PR_directionMotor_[1] ;
    pitch_joint_data.vel_ = (dtheta1 + dtheta2) / 2.0;
    // PR_joint_data[1].vel_des_  = (dtheta1 - dtheta2) / 2.0;

  // 力矩映射（考虑雅可比转置和减速比）
  pitch_joint_data.tau_ =Ankle_motors[0].tau_* PR_directionMotor_[0] + Ankle_motors[1].tau_* PR_directionMotor_[1];
    // PR_joint_data[1].ff_  = (Ankle_motors[0].tau_ - Ankle_motors[1].tau_) * PR_directionMotor_[1] / 2.0;

    return pitch_joint_data;

  }

EncosMotorData BridgeHW::Roll_forwardkinematics(YKSMotorData Ankle_motors[2]){
    EncosMotorData roll_joint_data;
      const std::vector<int> PR_directionMotor_ = {1,-1};
      const double theta1 = Ankle_motors[0].pos_ * PR_directionMotor_[0];
      const double theta2 = Ankle_motors[1].pos_ * PR_directionMotor_[1];
  
    // 平行四边形机构正解（基于几何关系）
      // PR_joint_data.pos_ = (theta1 + theta2) / 2.0;
      roll_joint_data.pos_  = (theta1 - theta2) / 2.0;
  
    // 速度映射（考虑减速比）
      const double dtheta1 = Ankle_motors[0].vel_*PR_directionMotor_[0] ;
      const double dtheta2 = Ankle_motors[1].vel_*PR_directionMotor_[1] ;
      // PR_joint_data.vel_ = (dtheta1 + dtheta2) / 2.0;
      roll_joint_data.vel_  = (dtheta1 - dtheta2) / 2.0;
  
    // 力矩映射（考虑雅可比转置和减速比）
      // PR_joint_data.ff_ = (Ankle_motors[0].tau_ + Ankle_motors[1].tau_) * PR_directionMotor_[0] / 2.0;
      roll_joint_data.tau_  = (Ankle_motors[0].tau_* PR_directionMotor_[0] - Ankle_motors[1].tau_ * PR_directionMotor_[1]);
  
      return roll_joint_data;
  
    }

YKSMotorData BridgeHW::AnkleA_inversekinematics(YKSMotorData pitch_joint_cmd,YKSMotorData roll_joint_cmd)
{
// 逆解计算
  YKSMotorData AnkleA_cmd;
  const std::vector<int> PR_directionMotor_{-1,1};
    const double theta1_des = pitch_joint_cmd.pos_des_ + roll_joint_cmd.pos_des_;
    const double theta2_des = pitch_joint_cmd.pos_des_ - roll_joint_cmd.pos_des_;

// 考虑减速比的电机指令
    AnkleA_cmd.pos_des_ = theta1_des * PR_directionMotor_[0];

// 速度逆解
    const double dtheta1_des = pitch_joint_cmd.vel_des_ + roll_joint_cmd.vel_des_;
    const double dtheta2_des = pitch_joint_cmd.vel_des_ - roll_joint_cmd.vel_des_;
    AnkleA_cmd.vel_des_ = dtheta1_des * PR_directionMotor_[0];

// 前馈力矩逆解（考虑雅可比矩阵和减速比）
    AnkleA_cmd.ff_ = (pitch_joint_cmd.ff_ + roll_joint_cmd.ff_)* PR_directionMotor_[0]/2.0;
    return AnkleA_cmd;
}

YKSMotorData BridgeHW::AnkleB_inversekinematics(YKSMotorData pitch_joint_cmd,YKSMotorData roll_joint_cmd)
{
// 逆解计算
  YKSMotorData AnkleB_cmd;
  const std::vector<int> PR_directionMotor_{-1,1};
    const double theta1_des = pitch_joint_cmd.pos_des_ + roll_joint_cmd.pos_des_;
    const double theta2_des = pitch_joint_cmd.pos_des_ - roll_joint_cmd.pos_des_;

// 考虑减速比的电机指令
  AnkleB_cmd.pos_des_ = theta2_des * PR_directionMotor_[1];

// 速度逆解
    const double dtheta1_des = pitch_joint_cmd.vel_des_ + roll_joint_cmd.vel_des_;
    const double dtheta2_des = pitch_joint_cmd.vel_des_ - roll_joint_cmd.vel_des_;
    AnkleB_cmd.vel_des_ = dtheta2_des * PR_directionMotor_[1];

// 前馈力矩逆解（考虑雅可比矩阵和减速比）
  AnkleB_cmd.ff_ = (pitch_joint_cmd.ff_ - roll_joint_cmd.ff_)* PR_directionMotor_[1]/2.0;
  return AnkleB_cmd;
}


}  // namespace legged
