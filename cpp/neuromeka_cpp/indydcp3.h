#ifndef INDYDCP3_H
#define INDYDCP3_H

#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <grpcpp/grpcpp.h>
#include "google/protobuf/util/json_util.h"

#include "proto/config.grpc.pb.h"
#include "proto/control.grpc.pb.h"
#include "proto/device.grpc.pb.h"
#include "proto/rtde.grpc.pb.h"

using google::protobuf::util::MessageToJsonString;
using google::protobuf::util::JsonStringToMessage;

// using Nrmk::IndyFramework::TrajState;
using Nrmk::IndyFramework::JointBaseType;
using Nrmk::IndyFramework::TaskBaseType;
using Nrmk::IndyFramework::DigitalState;
using Nrmk::IndyFramework::ProgramState;
using Nrmk::IndyFramework::EndtoolState;
using Nrmk::IndyFramework::StopCategory;
using Nrmk::IndyFramework::BlendingType_Type;
using Nrmk::IndyFramework::MotionCondition_ConditionType;
using Nrmk::IndyFramework::MotionCondition_ReactionType;
using Nrmk::IndyFramework::CircularSettingType;
using Nrmk::IndyFramework::CircularMovingType;
using Nrmk::IndyFramework::TeleMethod;

#define LIMIT_LEVEL_MIN  1
#define LIMIT_LEVEL_MAX  9
#define LIMIT_JOG_LEVEL_MIN 1
#define LIMIT_JOG_LEVEL_MAX 3
#define LIMIT_JOG_VEL_LEVEL_DEFAULT 2
#define LIMIT_JOG_ACC_LEVEL_DEFAULT 2

#define LIMIT_JOG_VEL_RATIO_DEFAULT 15  // %
#define LIMIT_JOG_ACC_RATIO_DEFAULT 100  // %
#define LIMIT_VEL_RATIO_MAX 100  // %
#define LIMIT_VEL_RATIO_MIN 1  // %
#define LIMIT_ACC_RATIO_MAX 900  // %
#define LIMIT_ACC_RATIO_MIN 1  // %


#define LIMIT_JOG_VEL_RATIO_MIN 5  // %
#define LIMIT_JOG_VEL_RATIO_MAX 25  // %
#define LIMIT_VEL_AUTO_LEVEL_VALUE (LIMIT_VEL_RATIO_MAX - LIMIT_JOG_VEL_RATIO_MAX)/(LIMIT_LEVEL_MAX - LIMIT_JOG_LEVEL_MAX)  // %
#define LIMIT_VEL_MANUAL_LEVEL_VALUE (LIMIT_JOG_VEL_RATIO_MAX - LIMIT_JOG_VEL_RATIO_MIN)/(LIMIT_JOG_LEVEL_MAX - LIMIT_JOG_LEVEL_MIN)  // %

#define LIMIT_TASK_DISP_VEL_VALUE_DEFAULT 250  // 250mm/s
#define LIMIT_Task_DISP_ACC_VALUE_DEFAULT 250  // 250mm/s^2
#define LIMIT_TASK_DISP_VEL_VALUE_MAX 1000  // mm/s
#define LIMIT_TASK_ROT_VEL_VALUE_MAX 120  // deg/s
#define LIMIT_EXTERNAL_MOTOR_SPEED_MAX 250;  // mm/s : 3000rpm -> 50 rev/sec * 5 mm/rev -> 250 mm/s


struct DCPDICond{
    std::map<unsigned int, int> di;
    std::map<unsigned int, int> end_di;
};

struct DCPVarCond{
    std::map<unsigned int, int> int_vars;
    std::map<unsigned int, float> float_vars;
    std::map<unsigned int, bool> bool_vars;
    std::map<unsigned int, int> modbus_vars;
    std::map<unsigned int, std::vector<float>> joint_vars;
    std::map<unsigned int, std::array<float, 6>> task_vars;
};

class IndyDCP3
{
    public:
        IndyDCP3(const std::string& robot_ip = "127.0.0.1", int index = 0);
        ~IndyDCP3();

        bool get_robot_data(Nrmk::IndyFramework::ControlData &control_data);
        bool get_control_data(Nrmk::IndyFramework::ControlData &control_data);
        bool get_control_state(Nrmk::IndyFramework::ControlData2 &control_state);
        bool get_motion_data(Nrmk::IndyFramework::MotionData &motion_data);

        bool get_servo_data(Nrmk::IndyFramework::ServoData &servo_data);
        bool get_violation_data(Nrmk::IndyFramework::ViolationData &violation_data);
        bool get_program_data(Nrmk::IndyFramework::ProgramData &program_data);

        bool get_di(Nrmk::IndyFramework::DigitalList &di_data);
        bool get_do(Nrmk::IndyFramework::DigitalList &do_data);
        bool set_do(const Nrmk::IndyFramework::DigitalList &do_signal_list);

        bool get_ai(Nrmk::IndyFramework::AnalogList &ai_data);
        bool get_ao(Nrmk::IndyFramework::AnalogList &ao_data);
        bool set_ao(const Nrmk::IndyFramework::AnalogList &ao_signal_list);

        bool get_endtool_di(Nrmk::IndyFramework::EndtoolSignalList &endtool_di_data);
        bool get_endtool_do(Nrmk::IndyFramework::EndtoolSignalList &endtool_do_data);
        bool set_endtool_do(const Nrmk::IndyFramework::EndtoolSignalList &end_do_signal_list);

        bool get_endtool_ai(Nrmk::IndyFramework::AnalogList &endtool_ai_data);
        bool get_endtool_ao(Nrmk::IndyFramework::AnalogList &endtool_ao_data);
        bool set_endtool_ao(const Nrmk::IndyFramework::AnalogList& end_ao_signal_list);

        bool get_device_info(Nrmk::IndyFramework::DeviceInfo& device_info);
        bool get_ft_sensor_data(Nrmk::IndyFramework::FTSensorData& ft_sensor_data);
        bool stop_motion(const StopCategory stop_category);

        bool get_home_pos(Nrmk::IndyFramework::JointPos &home_jpos);

        bool movej(const std::vector<float> jtarget,
                    const int base_type=JointBaseType::ABSOLUTE_JOINT,
                    const int blending_type=BlendingType_Type::BlendingType_Type_NONE,
                    const float blending_radius=0.0,
                    const float vel_ratio=LIMIT_JOG_VEL_RATIO_DEFAULT,
                    const float acc_ratio=LIMIT_JOG_ACC_RATIO_DEFAULT,
                    const bool const_cond=true,
                    const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                    const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                    const DCPDICond di_condition={},
                    const DCPVarCond var_condition={},
                    const bool teaching_mode=false);
        
        bool movel(const std::array<float, 6> ttarget,
                    const int base_type=TaskBaseType::ABSOLUTE_TASK,
                    const int blending_type=BlendingType_Type::BlendingType_Type_NONE,
                    const float blending_radius=0.0,
                    const float vel_ratio=LIMIT_JOG_VEL_RATIO_DEFAULT,
                    const float acc_ratio=LIMIT_JOG_ACC_RATIO_DEFAULT,
                    const bool const_cond=true,
                    const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                    const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                    const DCPDICond di_condition={},
                    const DCPVarCond var_condition={},
                    const bool teaching_mode=false);

        bool movec(const std::array<float, 6> tpos1,
                    const std::array<float, 6> tpos2,
                    const float angle=0.0,
                    const int setting_type=CircularSettingType::POINT_SET,
                    const int move_type=CircularMovingType::CONSTANT,
                    const int base_type=TaskBaseType::ABSOLUTE_TASK,
                    const int blending_type=BlendingType_Type::BlendingType_Type_NONE,
                    const float blending_radius=0.0,
                    const float vel_ratio=LIMIT_JOG_VEL_RATIO_DEFAULT,
                    const float acc_ratio=LIMIT_JOG_ACC_RATIO_DEFAULT,
                    const bool const_cond=true,
                    const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                    const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                    const DCPDICond di_condition={},
                    const DCPVarCond var_condition={},
                    const bool teaching_mode=false);

        bool move_home();
        bool start_teleop(const TeleMethod method);
        bool stop_teleop();

        bool movetelej(const std::vector<float> jpos, 
                        const float vel_ratio=1.0, 
                        const float acc_ratio=1.0,
                        const TeleMethod method=TeleMethod::TELE_JOINT_ABSOLUTE);

        bool movetelel(const std::array<float, 6> tpos, 
                        const float vel_ratio=1.0, 
                        const float acc_ratio=1.0,
                        const TeleMethod method=TeleMethod::TELE_TASK_ABSOLUTE);

        bool inverse_kin(const std::array<float, 6>& tpos, 
                        const std::vector<float>& init_jpos, 
                        std::vector<float>& jpos);

        bool set_direct_teaching(bool enable);
        bool set_simulation_mode(bool enable);
        bool recover();
        bool set_manual_recovery(bool enable);

        bool calculate_relative_pose(const std::array<float, 6>& start_pos, 
                                    const std::array<float, 6>& end_pos, 
                                    int base_type, 
                                    std::array<float, 6>& relative_pose);

        bool calculate_current_pose_rel(const std::array<float, 6>& current_pos,
                                        const std::array<float, 6>& relative_pos,
                                        int base_type,
                                        std::array<float, 6>& calculated_pose);

        bool play_program(const std::string& prog_name = "", int prog_idx = -1);
        bool pause_program();
        bool resume_program();
        bool stop_program();
        bool set_speed_ratio(unsigned int speed_ratio);

        bool get_bool_variable(std::vector<Nrmk::IndyFramework::BoolVariable>& bool_variables);
        bool get_int_variable(std::vector<Nrmk::IndyFramework::IntVariable>& int_variables);
        bool get_float_variable(std::vector<Nrmk::IndyFramework::FloatVariable>& float_variables);
        bool get_jpos_variable(std::vector<Nrmk::IndyFramework::JPosVariable>& jpos_variables);
        bool get_tpos_variable(std::vector<Nrmk::IndyFramework::TPosVariable>& tpos_variables);

        bool set_bool_variable(const std::vector<Nrmk::IndyFramework::BoolVariable>& bool_variables);
        bool set_int_variable(const std::vector<Nrmk::IndyFramework::IntVariable>& int_variables);
        bool set_float_variable(const std::vector<Nrmk::IndyFramework::FloatVariable>& float_variables);
        bool set_jpos_variable(const std::vector<Nrmk::IndyFramework::JPosVariable>& jpos_variables);
        bool set_tpos_variable(const std::vector<Nrmk::IndyFramework::TPosVariable>& tpos_variables);

        bool set_home_pos(const Nrmk::IndyFramework::JointPos& home_jpos);

        bool get_ref_frame(std::array<float, 6>& fpos);
        bool set_ref_frame(const std::array<float, 6>& fpos);

        bool set_ref_frame_planar(std::array<float, 6>& fpos_out, 
                            const std::array<float, 6>& fpos0,
                            const std::array<float, 6>& fpos1, 
                            const std::array<float, 6>& fpos2);

        bool set_tool_frame(const std::array<float, 6>& fpos);

        bool get_friction_comp(Nrmk::IndyFramework::FrictionCompSet& friction_comp);
        bool set_friction_comp(const Nrmk::IndyFramework::FrictionCompSet& friction_comp);

        bool get_tool_property(Nrmk::IndyFramework::ToolProperties& tool_properties);
        bool set_tool_property(const Nrmk::IndyFramework::ToolProperties& tool_properties);
                            
        bool get_coll_sens_level(unsigned int &level);
        bool set_coll_sens_level(unsigned int level);

        bool get_coll_sens_param(Nrmk::IndyFramework::CollisionThresholds& coll_sens_param);
        bool set_coll_sens_param(const Nrmk::IndyFramework::CollisionThresholds& coll_sens_param);

        bool get_coll_policy(Nrmk::IndyFramework::CollisionPolicy& coll_policy);
        bool set_coll_policy(const Nrmk::IndyFramework::CollisionPolicy& coll_policy);

        bool get_safety_limits(Nrmk::IndyFramework::SafetyLimits& safety_limits);
        bool set_safety_limits(const Nrmk::IndyFramework::SafetyLimits& safety_limits);

        bool activate_sdk(const Nrmk::IndyFramework::SDKLicenseInfo& request, 
                                Nrmk::IndyFramework::SDKLicenseResp& response);


        bool set_custom_control_mode(const int mode);
        bool get_custom_control_mode(int& mode);

        bool get_custom_control_gain(Nrmk::IndyFramework::CustomGainSet& custom_gains);
        bool set_custom_control_gain(const Nrmk::IndyFramework::CustomGainSet& custom_gains);

        bool start_log();
        bool end_log();
        

    private:
        bool _isConnected;
        unsigned int _cobotDOF;

        std::shared_ptr<grpc::Channel> control_channel;
        std::shared_ptr<grpc::Channel> device_channel;
        std::shared_ptr<grpc::Channel> config_channel;
        std::shared_ptr<grpc::Channel> rtde_channel;

        std::unique_ptr<Nrmk::IndyFramework::Control::Stub> control_stub;
        std::unique_ptr<Nrmk::IndyFramework::Device::Stub> device_stub;
        std::unique_ptr<Nrmk::IndyFramework::Config::Stub> config_stub;
        std::unique_ptr<Nrmk::IndyFramework::RTDataExchange::Stub> rtde_stub;

        const std::vector<int> CONTROL_SOCKET_PORT  = {20001, 30001};
        const std::vector<int> DEVICE_SOCKET_PORT   = {20002, 30002};
        const std::vector<int> CONFIG_SOCKET_PORT   = {20003, 30003};
        const std::vector<int> RTDE_SOCKET_PORT     = {20004, 30004};
};

#endif