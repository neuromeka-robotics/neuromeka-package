#include "indydcp3.h"
#include "indydcp3_rpc_utils.h"
#include <vector>

bool IndyDCP3::set_environment_list(const Nrmk::IndyFramework::EnvironmentList& environment_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetEnvironmentList(&context, environment_list, &response);
    if (!status.ok()) {
        std::cerr << "SetEnvironmentList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_environment_list(Nrmk::IndyFramework::EnvironmentList& environment_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetEnvironmentList(&context, request, &environment_list);
    if (!status.ok()) {
        std::cerr << "GetEnvironmentList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::change_password(const Nrmk::IndyFramework::ChangePasswordReq& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::ChangePassword,
                          request, response, "ChangePassword");
}

bool IndyDCP3::get_new_controller_test_on_off_state(Nrmk::IndyFramework::NewControllerTestState& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::GetNewControllerTestOnOffState,
                          request, response, "GetNewControllerTestOnOffState");
}

bool IndyDCP3::get_nonce(Nrmk::IndyFramework::Nonce& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::GetNonce,
                          request, response, "GetNonce");
}

bool IndyDCP3::get_servo_param_list(Nrmk::IndyFramework::Vector& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::GetServoParamList,
                          request, response, "GetServoParamList");
}

bool IndyDCP3::get_test_control_gain(Nrmk::IndyFramework::TestGainSet& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::GetTestControlGain,
                          request, response, "GetTestControlGain");
}

bool IndyDCP3::get_weld_position_list(Nrmk::IndyFramework::WeldPositionList& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::GetWeldPositionList,
                          request, response, "GetWeldPositionList");
}

bool IndyDCP3::get_welding_machine_config(Nrmk::IndyFramework::WeldingConfigInfo& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::GetWeldingMachineConfig,
                          request, response, "GetWeldingMachineConfig");
}

bool IndyDCP3::login(const Nrmk::IndyFramework::Digest& request, Nrmk::IndyFramework::LoginRes& response) {
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::Login,
                          request, response, "Login");
}

bool IndyDCP3::set_new_controller_test_on_off(const Nrmk::IndyFramework::NewControllerTestState& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::SetNewControllerTestOnOff,
                          request, response, "SetNewControllerTestOnOff");
}

bool IndyDCP3::set_test_control_gain(const Nrmk::IndyFramework::TestGainSet& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::SetTestControlGain,
                          request, response, "SetTestControlGain");
}

bool IndyDCP3::set_weld_position_list(const Nrmk::IndyFramework::WeldPositionList& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::SetWeldPositionList,
                          request, response, "SetWeldPositionList");
}

bool IndyDCP3::set_welding_machine_config(const Nrmk::IndyFramework::WeldingConfigInfo& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::SetWeldingMachineConfig,
                          request, response, "SetWeldingMachineConfig");
}

bool IndyDCP3::test_digest(const Nrmk::IndyFramework::Passwd& request, Nrmk::IndyFramework::Digest& response) {
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::TestDigest,
                          request, response, "TestDigest");
}

bool IndyDCP3::verify_token(const Nrmk::IndyFramework::Token& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::VerifyToken,
                          request, response, "VerifyToken");
}

bool IndyDCP3::get_ft_sensor_config_for(const Nrmk::IndyFramework::Int& request, Nrmk::IndyFramework::FTSensorDevice& response) {
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::GetFTSensorConfigFor,
                          request, response, "GetFTSensorConfigFor");
}

bool IndyDCP3::get_ref_frame_for(const Nrmk::IndyFramework::Int& request, Nrmk::IndyFramework::Frame& response) {
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::GetRefFrameFor,
                          request, response, "GetRefFrameFor");
}

bool IndyDCP3::get_tool_property_at(const Nrmk::IndyFramework::Int& request, Nrmk::IndyFramework::ToolProperties& response) {
    return call_unary_rpc(config_stub.get(),
                          &Nrmk::IndyFramework::Config::Stub::GetToolPropertyAt,
                          request, response, "GetToolPropertyAt");
}

bool IndyDCP3::get_home_pos(Nrmk::IndyFramework::JointPos &home_jpos)
{
    /*
        Joint Home Position
        jpos -> double[]
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::JointPos response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetHomePosition(&context, request, &response);

    if (!status.ok()){
        std::cerr << "Get Home Pos RPC failed." << std::endl;
        return false;
    }

    home_jpos = response;
    return true;
}

bool IndyDCP3::set_locked_joint(int index) {
    Nrmk::IndyFramework::Int request;
    request.set_value(index);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetLockedJoint(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetLockedJoint RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_locked_joint_for(int arm_index, int joint_index) {
    Nrmk::IndyFramework::LockJointReq request;
    request.set_arm_index(arm_index);
    request.set_joint_index(joint_index);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetLockedJointFor(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetLockedJointFor RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_tool_link(int index) {
    Nrmk::IndyFramework::Int request;
    request.set_value(index);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetToolLink(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetToolLink RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_speed_ratio(unsigned int speed_ratio)
{
    /*
        Set Speed Ratio
        speed_ratio -> int (0 ~ 100)
    */

    Nrmk::IndyFramework::Ratio request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_ratio(speed_ratio);

    grpc::Status status = config_stub->SetSpeedRatio(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetSpeedRatio RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_speed_ratio(unsigned int& speed_ratio)
{
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Ratio response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetSpeedRatio(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetSpeedRatio RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    speed_ratio = static_cast<unsigned int>(response.ratio());
    return true;
}

bool IndyDCP3::set_home_pos(const Nrmk::IndyFramework::JointPos& home_jpos)
{
    /*
        Joint Home Position
        jpos -> double[]
    */

    Nrmk::IndyFramework::JointPos request = home_jpos;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetHomePosition(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Home Pos RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_ref_frame(std::array<float, 6>& fpos)
{
    /*
        Reference Frame
        fpos -> float[6]
    */

    Nrmk::IndyFramework::Frame response;
    grpc::ClientContext context;
    Nrmk::IndyFramework::Empty request;

    grpc::Status status = config_stub->GetRefFrame(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetRefFrame RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        fpos[i] = response.fpos(i);
    }

    return true;
}

bool IndyDCP3::set_ref_frame(const std::array<float, 6>& fpos)
{
    /*
        Reference Frame
        fpos -> float[6]
    */

    Nrmk::IndyFramework::Frame request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : fpos) {
        request.add_fpos(value);
    }

    grpc::Status status = config_stub->SetRefFrame(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetRefFrame RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::load_reference_frame(Nrmk::IndyFramework::RefFrameList& list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetRefFrameList(&context, request, &list);
    if (!status.ok()) {
        std::cerr << "GetRefFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::save_reference_frame(const Nrmk::IndyFramework::RefFrameList& list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetRefFrameList(&context, list, &response);
    if (!status.ok()) {
        std::cerr << "SetRefFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::set_ref_frame_planar(std::array<float, 6>& fpos_out, const std::array<float, 6>& fpos0,
                                    const std::array<float, 6>& fpos1, const std::array<float, 6>& fpos2,
                                    const int arm_index, const int link_index)
{
    /*
        Reference Frame (Planar)
        fpos_out -> float[6]
        fpos0 -> float[6]
        fpos1 -> float[6]
        fpos2 -> float[6]
        arm_index -> int
        link_index -> int
    */

    Nrmk::IndyFramework::PlanarFrame request;
    Nrmk::IndyFramework::FrameResult response;
    grpc::ClientContext context;

    for (int i = 0; i < 6; ++i) {
        request.add_fpos0(fpos0[i]);
        request.add_fpos1(fpos1[i]);
        request.add_fpos2(fpos2[i]);
    }
    request.set_arm_index(arm_index);
    request.set_link_index(link_index);

    grpc::Status status = config_stub->SetRefFramePlanar(&context, request, &response);

    for (int i = 0; i < 6; ++i) {
        fpos_out[i] = response.fpos(i);
    }

    if (!status.ok()) {
        std::cerr << "SetRefFramePlanar RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_tool_frame(const std::array<float, 6>& fpos)
{
    /*
        Tool Frame
        fpos -> float[6]
    */

    Nrmk::IndyFramework::Frame request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : fpos) {
        request.add_fpos(value);
    }

    grpc::Status status = config_stub->SetToolFrame(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetToolFrame RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_path_config(Nrmk::IndyFramework::PathConfig& path_config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetPathConfig(&context, request, &path_config);
    if (!status.ok()) {
        std::cerr << "GetPathConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_language(std::string& language) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Name response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetLanguage(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetLanguage RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    language = response.name();
    return true;
}

bool IndyDCP3::set_language(const std::string& language) {
    Nrmk::IndyFramework::Name request;
    request.set_name(language);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetLanguage(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetLanguage RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_friction_comp(Nrmk::IndyFramework::FrictionCompSet& friction_comp)
{
    /*
        Friction Compensation Set:
        joint_idx   -> uint32
        control_comp_enable   -> bool
        control_comp_levels   -> int32[6]
        teaching_comp_enable   -> bool
        teaching_comp_levels   -> int32[6]
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetFrictionComp(&context, request, &friction_comp);
    if (!status.ok()) {
        std::cerr << "GetFrictionComp RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_friction_comp(const Nrmk::IndyFramework::FrictionCompSet& friction_comp)
{
    /*
        Friction Compensation Set:
        control_comp_enable   -> bool
        control_comp_levels   -> int32[]
        teaching_comp_enable   -> bool
        teaching_comp_levels   -> int32[]
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    if ((_cobotDOF != friction_comp.control_comp_levels_size()) ||
        (_cobotDOF != friction_comp.teaching_comp_levels_size())) {
        return false;
    }

    grpc::Status status = config_stub->SetFrictionComp(&context, friction_comp, &response);
    if (!status.ok()) {
        std::cerr << "SetFrictionComp RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_tool_property(Nrmk::IndyFramework::ToolProperties& tool_properties)
{
    /*
        Tool Properties:
        mass   -> float
        center_of_mass   -> float[3]
        inertia   -> float[6]
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetToolProperty(&context, request, &tool_properties);
    if (!status.ok()) {
        std::cerr << "GetToolProperty RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_tool_list(const Nrmk::IndyFramework::ToolList& tool_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetToolList(&context, tool_list, &response);
    if (!status.ok()) {
        std::cerr << "SetToolList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_tool_list(Nrmk::IndyFramework::ToolList& tool_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetToolList(&context, request, &tool_list);
    if (!status.ok()) {
        std::cerr << "GetToolList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_vision_server_list(const Nrmk::IndyFramework::VisionServerList& vision_server_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetVisionServerList(&context, vision_server_list, &response);
    if (!status.ok()) {
        std::cerr << "SetVisionServerList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_vision_server_list(Nrmk::IndyFramework::VisionServerList& vision_server_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetVisionServerList(&context, request, &vision_server_list);
    if (!status.ok()) {
        std::cerr << "GetVisionServerList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_modbus_server_list(const Nrmk::IndyFramework::ModbusServerList& modbus_server_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetModbusServerList(&context, modbus_server_list, &response);
    if (!status.ok()) {
        std::cerr << "SetModbusServerList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_modbus_server_list(Nrmk::IndyFramework::ModbusServerList& modbus_server_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetModbusServerList(&context, request, &modbus_server_list);
    if (!status.ok()) {
        std::cerr << "GetModbusServerList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_conveyor_list(Nrmk::IndyFramework::ConveyorList& conveyor_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetConveyorList(&context, request, &conveyor_list);
    if (!status.ok()) {
        std::cerr << "GetConveyorList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_list(const Nrmk::IndyFramework::ConveyorList& conveyor_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetConveyorList(&context, conveyor_list, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_compliance_control_joint_gain(const Nrmk::IndyFramework::ComplianceGainSet& gains) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetComplianceControlJointGain(&context, gains, &response);
    if (!status.ok()) {
        std::cerr << "SetComplianceControlJointGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_compliance_control_joint_gain(Nrmk::IndyFramework::ComplianceGainSet& gains) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetComplianceControlJointGain(&context, request, &gains);
    if (!status.ok()) {
        std::cerr << "GetComplianceControlJointGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_tool_frame_list(Nrmk::IndyFramework::ToolFrameList& list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetToolFrameList(&context, request, &list);
    if (!status.ok()) {
        std::cerr << "GetToolFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_tool_frame_list(const Nrmk::IndyFramework::ToolFrameList& list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetToolFrameList(&context, list, &response);
    if (!status.ok()) {
        std::cerr << "SetToolFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_ref_frame_list(Nrmk::IndyFramework::RefFrameList& list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetRefFrameList(&context, request, &list);
    if (!status.ok()) {
        std::cerr << "GetRefFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_ref_frame_list(const Nrmk::IndyFramework::RefFrameList& list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetRefFrameList(&context, list, &response);
    if (!status.ok()) {
        std::cerr << "SetRefFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_custom_pos_list(Nrmk::IndyFramework::CustomPosList& list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetCustomPosList(&context, request, &list);
    if (!status.ok()) {
        std::cerr << "GetCustomPosList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_custom_pos_list(const Nrmk::IndyFramework::CustomPosList& list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetCustomPosList(&context, list, &response);
    if (!status.ok()) {
        std::cerr << "SetCustomPosList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_tool_shape_list(const Nrmk::IndyFramework::ToolShapeList& list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetToolShapeList(&context, list, &response);
    if (!status.ok()) {
        std::cerr << "SetToolShapeList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_tool_shape_list(Nrmk::IndyFramework::ToolShapeList& list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetToolShapeList(&context, request, &list);
    if (!status.ok()) {
        std::cerr << "GetToolShapeList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_tool_property(const Nrmk::IndyFramework::ToolProperties& tool_properties)
{
    /*
        Tool Properties:
        mass -> float
        center_of_mass -> float[3]
        inertia -> float[6]
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetToolProperty(&context, tool_properties, &response);

    if (!status.ok()) {
        std::cerr << "SetToolProperty RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_mount_pos(float rot_y, float rot_z)
{
    Nrmk::IndyFramework::MountingAngles request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_ry(rot_y);
    request.set_rz(rot_z);

    grpc::Status status = config_stub->SetMountPos(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "SetMountPos RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_mount_pos(const Nrmk::IndyFramework::MountingAngles& mounting_angles)
{
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetMountPos(&context, mounting_angles, &response);

    if (!status.ok()) {
        std::cerr << "SetMountPos RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_mount_pos(float &rot_y, float &rot_z)
{
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::MountingAngles response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetMountPos(&context, request, &response);

    rot_y = response.ry();
    rot_z = response.rz();

    if (!status.ok()) {
        std::cerr << "GetMountPos RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_mount_pos(Nrmk::IndyFramework::MountingAngles& mounting_angles)
{
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetMountPos(&context, request, &mounting_angles);

    if (!status.ok()) {
        std::cerr << "GetMountPos RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_coll_sens_level(unsigned int &level)
{
    /*
        Collision Sensitivity Level:
        level -> uint32
    */

    Nrmk::IndyFramework::CollisionSensLevel response;
    grpc::ClientContext context;
    Nrmk::IndyFramework::Empty request;

    grpc::Status status = config_stub->GetCollSensLevel(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetCollSensLevel RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    level = response.level();
    return true;
}

bool IndyDCP3::set_coll_sens_level(unsigned int level)
{
    /*
        Collision Sensitivity Level:
        level -> uint32
    */

    Nrmk::IndyFramework::CollisionSensLevel request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_level(level);

    grpc::Status status = config_stub->SetCollSensLevel(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetCollSensLevel RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_coll_sens_param(Nrmk::IndyFramework::CollisionThresholds& coll_sens_param)
{
    /*
        Collision Params:
        j_torque_bases                  -> double[6]
        j_torque_tangents               -> double[6]
        t_torque_bases                  -> double[6]
        t_torque_tangents               -> double[6]
        error_bases                     -> double[6]
        error_tangents                  -> double[6]
        t_constvel_torque_bases         -> double[6]
        t_constvel_torque_tangents      -> double[6]
        t_conveyor_torque_bases         -> double[6]
        t_conveyor_torque_tangents      -> double[6]
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetCollSensParam(&context, request, &coll_sens_param);

    if (!status.ok()) {
        std::cerr << "GetCollSensParam RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_coll_sens_param(const Nrmk::IndyFramework::CollisionThresholds& coll_sens_param)
{
    /*
        Collision Params:
        j_torque_bases                  -> double[6]
        j_torque_tangents               -> double[6]
        t_torque_bases                  -> double[6]
        t_torque_tangents               -> double[6]
        error_bases                     -> double[6]
        error_tangents                  -> double[6]
        t_constvel_torque_bases         -> double[6]
        t_constvel_torque_tangents      -> double[6]
        t_conveyor_torque_bases         -> double[6]
        t_conveyor_torque_tangents      -> double[6]
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetCollSensParam(&context, coll_sens_param, &response);

    if (!status.ok()) {
        std::cerr << "SetCollSensParam RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_default_coll_sens_param(Nrmk::IndyFramework::CollisionThresholds& coll_sens_param) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetDefaultCollSensParam(&context, request, &coll_sens_param);
    if (!status.ok()) {
        std::cerr << "GetDefaultCollSensParam RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_coll_policy(Nrmk::IndyFramework::CollisionPolicy& coll_policy)
{
    /*
        Collision Policy:
        policy -> uint32
        sleep_time -> float
        gravity_time -> float
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetCollPolicy(&context, request, &coll_policy);

    if (!status.ok()) {
        std::cerr << "GetCollPolicy RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_coll_policy(const Nrmk::IndyFramework::CollisionPolicy& coll_policy)
{
    /*
        Collision Policies:
        policy -> uint32
        sleep_time -> float
        gravity_time -> float
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetCollPolicy(&context, coll_policy, &response);

    if (!status.ok()) {
        std::cerr << "SetCollPolicy RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_simple_coll_threshold() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetSimpleCollThreshold(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetSimpleCollThreshold RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_collison_model_margin(Nrmk::IndyFramework::CollisionModelMargin& margin) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetCollisonModelMargin(&context, request, &margin);
    if (!status.ok()) {
        std::cerr << "GetCollisonModelMargin RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_collison_model_margin(const Nrmk::IndyFramework::CollisionModelMargin& margin) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetCollisonModelMargin(&context, margin, &response);
    if (!status.ok()) {
        std::cerr << "SetCollisonModelMargin RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::set_sensorless_params(const Nrmk::IndyFramework::SensorlessParams& params) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetSensorlessParams(&context, params, &response);
    if (!status.ok()) {
        std::cerr << "SetSensorlessParams RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_sensorless_params(Nrmk::IndyFramework::SensorlessParams& params) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetSensorlessParams(&context, request, &params);
    if (!status.ok()) {
        std::cerr << "GetSensorlessParams RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_on_start_program_config(const Nrmk::IndyFramework::OnStartProgramConfig& config) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetOnStartProgramConfig(&context, config, &response);
    if (!status.ok()) {
        std::cerr << "SetOnStartProgramConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_on_start_program_config(Nrmk::IndyFramework::OnStartProgramConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetOnStartProgramConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetOnStartProgramConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_safety_limits(Nrmk::IndyFramework::SafetyLimits& safety_limits)
{
    /*
        Safety Limits:
        power_limit             -> float
        power_limit_ratio       -> float
        tcp_force_limit         -> float
        tcp_force_limit_ratio   -> float
        tcp_speed_limit         -> float
        tcp_speed_limit_ratio   -> float
        joint_upper_limits      -> float[]
        joint_lower_limits      -> float[]
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetSafetyLimits(&context, request, &safety_limits);

    if (!status.ok()) {
        std::cerr << "GetSafetyLimits RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_safety_limits(const Nrmk::IndyFramework::SafetyLimits& safety_limits)
{
    /*
        Safety Limits:
        power_limit             -> float
        power_limit_ratio       -> float
        tcp_force_limit         -> float
        tcp_force_limit_ratio   -> float
        tcp_speed_limit         -> float
        tcp_speed_limit_ratio   -> float
        joint_upper_limits      -> float[]
        joint_lower_limits      -> float[]
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetSafetyLimits(&context, safety_limits, &response);

    if (!status.ok()) {
        std::cerr << "SetSafetyLimits RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_custom_control_gain(Nrmk::IndyFramework::CustomGainSet& custom_gains) {
    /*
        Get Custom Control Gain
        custom_gains -> Nrmk::IndyFramework::CustomGainSet
    */
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetCustomControlGain(&context, Nrmk::IndyFramework::Empty(), &custom_gains);
    if (!status.ok()) {
        std::cerr << "Get Custom Control Gain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_custom_control_gain(const Nrmk::IndyFramework::CustomGainSet& custom_gains) {
    /*
        Set Custom Control Gain
        custom_gains -> Nrmk::IndyFramework::CustomGainSet
    */
    grpc::ClientContext context;
    Nrmk::IndyFramework::Response response;

    grpc::Status status = config_stub->SetCustomControlGain(&context, custom_gains, &response);
    if (!status.ok()) {
        std::cerr << "Set Custom Control Gain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::socket_cmd_set_config(const Nrmk::IndyFramework::SocketCommandConfig& config) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetSocketCommandConfig(&context, config, &response);
    if (!status.ok()) {
        std::cerr << "SetSocketCommandConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::socket_cmd_get_config(Nrmk::IndyFramework::SocketCommandConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetSocketCommandConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetSocketCommandConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_ft_sensor_config(const Nrmk::IndyFramework::FTSensorDevice& sensor_config) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetFTSensorConfig(&context, sensor_config, &response);
    if (!status.ok()) {
        std::cerr << "SetFTSensorConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_ft_sensor_config(Nrmk::IndyFramework::FTSensorDevice& sensor_config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetFTSensorConfig(&context, request, &sensor_config);
    if (!status.ok()) {
        std::cerr << "GetFTSensorConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_do_config_list(const Nrmk::IndyFramework::DOConfigList& do_config_list) {
    /*
        DO Configuration List
        {
            'do_configs': [
                {
                    'state_code': 2,
                    'state_name': "name",
                    'onSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}],
                    'offSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                }
            ]
        }
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetDOConfigList(&context, do_config_list, &response);
    if (!status.ok()) {
        std::cerr << "SetDOConfigList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_do_config_list(Nrmk::IndyFramework::DOConfigList& do_config_list) {
    /*
        DO Configuration List
        {
            'do_configs': [
                {
                    'state_code': 2,
                    'state_name': "name",
                    'onSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}],
                    'offSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                }
            ]
        }
    */
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetDOConfigList(&context, request, &do_config_list);
    if (!status.ok()) {
        std::cerr << "GetDOConfigList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::restore_factory_control_gains() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->RestorFactoryControlGains(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "RestorFactoryControlGains RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_imu_auto_mount(Nrmk::IndyFramework::MountingAngles& mounting_angles) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetIMUAutoMount(&context, request, &mounting_angles);
    if (!status.ok()) {
        std::cerr << "GetIMUAutoMount RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_tool_property_list(const Nrmk::IndyFramework::ToolPropertyEntries& entries) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetToolPropertyList(&context, entries, &response);
    if (!status.ok()) {
        std::cerr << "SetToolPropertyList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_tool_property_list(Nrmk::IndyFramework::ToolPropertyEntries& entries) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetToolPropertyList(&context, request, &entries);
    if (!status.ok()) {
        std::cerr << "GetToolPropertyList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_joint_limit_config(Nrmk::IndyFramework::JointLimitConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetJointLimitConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetJointLimitConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_joint_limit_config(const Nrmk::IndyFramework::JointLimitConfig& config) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetJointLimitConfig(&context, config, &response);
    if (!status.ok()) {
        std::cerr << "SetJointLimitConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_original_joint_limit_config(Nrmk::IndyFramework::JointLimitConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetOriginalJointLimitConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetOriginalJointLimitConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::save_safety_snapshot(const Nrmk::IndyFramework::SaveSafetySnapshotReq& request,
                                    Nrmk::IndyFramework::SafetySnapshotInfo& snapshot_info) {
    grpc::ClientContext context;

    grpc::Status status = config_stub->SaveSafetySnapshot(&context, request, &snapshot_info);
    if (!status.ok()) {
        std::cerr << "SaveSafetySnapshot RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::list_safety_snapshots(Nrmk::IndyFramework::SafetySnapshotList& snapshot_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->ListSafetySnapshots(&context, request, &snapshot_list);
    if (!status.ok()) {
        std::cerr << "ListSafetySnapshots RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::restore_safety_snapshot(const Nrmk::IndyFramework::SafetySnapshotId& snapshot_id) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->RestoreSafetySnapshot(&context, snapshot_id, &response);
    if (!status.ok()) {
        std::cerr << "RestoreSafetySnapshot RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::delete_safety_snapshot(const Nrmk::IndyFramework::SafetySnapshotId& snapshot_id) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->DeleteSafetySnapshot(&context, snapshot_id, &response);
    if (!status.ok()) {
        std::cerr << "DeleteSafetySnapshot RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::restor_factory_safety_config() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->RestorFactorySafetyConfig(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "RestorFactorySafetyConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_operation_mode_config(const Nrmk::IndyFramework::OperationModeConfig& config) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetOperationModeConfig(&context, config, &response);
    if (!status.ok()) {
        std::cerr << "SetOperationModeConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_operation_mode_config(Nrmk::IndyFramework::OperationModeConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetOperationModeConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetOperationModeConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_di_config_list(const Nrmk::IndyFramework::DIConfigList& di_config_list) {
    /*
        DI Configuration List
        {
            'di_configs': [
                {
                    'function_code': 2,
                    'function_name': "name",
                    'triggerSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                    'successSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                    'failureSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                }
            ]
        }
    */
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetDIConfigList(&context, di_config_list, &response);
    if (!status.ok()) {
        std::cerr << "Set DI Config List RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_di_config_list(Nrmk::IndyFramework::DIConfigList& di_config_list) {
    /*
        DI Configuration List
        {
            'di_configs': [
                {
                    'function_code': 2,
                    'function_name': "name",
                    'triggerSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}],
                    'successSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}],
                    'failureSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                }
            ]
        }
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetDIConfigList(&context, request, &di_config_list);
    if (!status.ok()) {
        std::cerr << "Get DI Config List RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_auto_servo_off(const Nrmk::IndyFramework::AutoServoOffConfig& config) {
    /*
        Auto Servo-Off Config
        enable -> bool
        time -> float
    */
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetAutoServoOff(&context, config, &response);
    if (!status.ok()) {
        std::cerr << "Set Auto Servo Off Config RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_auto_servo_off(Nrmk::IndyFramework::AutoServoOffConfig& config) {
    /*
        Auto Servo-Off Config
        enable -> bool
        time -> float
    */
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetAutoServoOff(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "Get Auto Servo Off Config RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_safety_stop_config(const Nrmk::IndyFramework::SafetyStopConfig& config) {
    /*
        Safety Stop Category:
        jpos_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        jvel_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        jtau_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        tvel_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        tforce_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        power_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
    */
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetSafetyStopConfig(&context, config, &response);
    if (!status.ok()) {
        std::cerr << "Set Safety Stop Config RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_safety_stop_config(Nrmk::IndyFramework::SafetyStopConfig& config) {
    /*
        Safety Stop Category:
        joint_position_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        joint_speed_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        joint_torque_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        tcp_speed_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        tcp_force_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        power_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetSafetyStopConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "Get Safety Stop Config RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_reduced_ratio(float& ratio) {
    Nrmk::IndyFramework::GetReducedRatioRes response;
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetReducedRatio(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get Reduced Ratio RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    ratio = response.ratio();
    return true;
}

bool IndyDCP3::get_reduced_speed(float& speed) {
    Nrmk::IndyFramework::GetReducedSpeedRes response;
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetReducedSpeed(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get Reduced Speed RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    speed = response.speed();
    return true;
}

bool IndyDCP3::set_reduced_speed(const float speed) {
    Nrmk::IndyFramework::SetReducedSpeedReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_speed(speed);

    grpc::Status status = config_stub->SetReducedSpeed(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Reduced Speed RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_teleop_params(const Nrmk::IndyFramework::TeleOpParams& request) {
    // check request
    if (request.cutoff_freq_input() < 0.1 || request.cutoff_freq_input() > 20.0) {
        std::cerr << "Cutoff frequency must be between 0.1 and 20.0 Hz." << std::endl;
        return false;
    }

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetTeleOpParams(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set TeleOp Params RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_teleop_params(Nrmk::IndyFramework::TeleOpParams& response) {
    /*
        IO Data:
        cutoff_freq_input -> float
    */
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetTeleOpParams(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get TeleOp Params RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_kinematics_params(Nrmk::IndyFramework::KinematicsParams& response) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetKinematicsParams(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get Kinematics Params RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_pack_pos(std::vector<float>& jpos) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::JointPos response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetPackPosition(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPackPosition RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    jpos.clear();
    for (int i = 0; i < response.jpos_size(); ++i) {
        jpos.push_back(response.jpos(i));
    }
    return true;
}

bool IndyDCP3::set_joint_control_gain(const std::vector<float>& kp, const std::vector<float>& kv, const std::vector<float>& kl2) {
    Nrmk::IndyFramework::JointGainSet request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : kp) request.add_kp(value);
    for (const auto& value : kv) request.add_kv(value);
    for (const auto& value : kl2) request.add_kl2(value);

    grpc::Status status = config_stub->SetJointControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetJointControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_joint_control_gain(std::vector<float>& kp, std::vector<float>& kv, std::vector<float>& kl2) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::JointGainSet response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetJointControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetJointControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    kp.assign(response.kp().begin(), response.kp().end());
    kv.assign(response.kv().begin(), response.kv().end());
    kl2.assign(response.kl2().begin(), response.kl2().end());
    return true;
}

bool IndyDCP3::set_task_control_gain(const std::vector<float>& kp, const std::vector<float>& kv, const std::vector<float>& kl2) {
    Nrmk::IndyFramework::TaskGainSet request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : kp) request.add_kp(value);
    for (const auto& value : kv) request.add_kv(value);
    for (const auto& value : kl2) request.add_kl2(value);

    grpc::Status status = config_stub->SetTaskControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetTaskControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_task_control_gain(std::vector<float>& kp, std::vector<float>& kv, std::vector<float>& kl2) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::TaskGainSet response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetTaskControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetTaskControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    kp.assign(response.kp().begin(), response.kp().end());
    kv.assign(response.kv().begin(), response.kv().end());
    kl2.assign(response.kl2().begin(), response.kl2().end());
    return true;
}

bool IndyDCP3::set_impedance_control_gain(const std::vector<float>& mass,
                                          const std::vector<float>& damping,
                                          const std::vector<float>& stiffness,
                                          const std::vector<float>& kl2) {
    Nrmk::IndyFramework::ImpedanceGainSet request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : mass) request.add_mass(value);
    for (const auto& value : damping) request.add_damping(value);
    for (const auto& value : stiffness) request.add_stiffness(value);
    for (const auto& value : kl2) request.add_kl2(value);

    grpc::Status status = config_stub->SetImpedanceControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetImpedanceControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_impedance_control_gain(std::vector<float>& mass,
                                          std::vector<float>& damping,
                                          std::vector<float>& stiffness,
                                          std::vector<float>& kl2) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ImpedanceGainSet response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetImpedanceControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetImpedanceControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    mass.assign(response.mass().begin(), response.mass().end());
    damping.assign(response.damping().begin(), response.damping().end());
    stiffness.assign(response.stiffness().begin(), response.stiffness().end());
    kl2.assign(response.kl2().begin(), response.kl2().end());
    return true;
}

bool IndyDCP3::set_force_control_gain(const std::vector<float>& kp,
                                      const std::vector<float>& kv,
                                      const std::vector<float>& kl2,
                                      const std::vector<float>& mass,
                                      const std::vector<float>& damping,
                                      const std::vector<float>& stiffness,
                                      const std::vector<float>& kpf,
                                      const std::vector<float>& kif) {
    Nrmk::IndyFramework::ForceGainSet request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : kp) request.add_kp(value);
    for (const auto& value : kv) request.add_kv(value);
    for (const auto& value : kl2) request.add_kl2(value);
    for (const auto& value : mass) request.add_mass(value);
    for (const auto& value : damping) request.add_damping(value);
    for (const auto& value : stiffness) request.add_stiffness(value);
    for (const auto& value : kpf) request.add_kpf(value);
    for (const auto& value : kif) request.add_kif(value);

    grpc::Status status = config_stub->SetForceControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetForceControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_force_control_gain(std::vector<float>& kp,
                                      std::vector<float>& kv,
                                      std::vector<float>& kl2,
                                      std::vector<float>& mass,
                                      std::vector<float>& damping,
                                      std::vector<float>& stiffness,
                                      std::vector<float>& kpf,
                                      std::vector<float>& kif) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ForceGainSet response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetForceControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetForceControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    kp.assign(response.kp().begin(), response.kp().end());
    kv.assign(response.kv().begin(), response.kv().end());
    kl2.assign(response.kl2().begin(), response.kl2().end());
    mass.assign(response.mass().begin(), response.mass().end());
    damping.assign(response.damping().begin(), response.damping().end());
    stiffness.assign(response.stiffness().begin(), response.stiffness().end());
    kpf.assign(response.kpf().begin(), response.kpf().end());
    kif.assign(response.kif().begin(), response.kif().end());
    return true;
}
