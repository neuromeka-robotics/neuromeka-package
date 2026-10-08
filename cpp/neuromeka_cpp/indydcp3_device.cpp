#include "indydcp3.h"
#include "indydcp3_rpc_utils.h"
#include <vector>

bool IndyDCP3::configure_pickit_3d(const Nrmk::IndyFramework::ConfigurePickit3DReq& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::ConfigurePickit3D,
                          request, response, "ConfigurePickit3D");
}

bool IndyDCP3::get_conveyor_object_distances(Nrmk::IndyFramework::ConveyorObjectDistances& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::GetConveyorObjectDistances,
                          request, response, "GetConveyorObjectDistances");
}

bool IndyDCP3::get_do_channels(Nrmk::IndyFramework::DOChannelModes& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::GetDOChannels,
                          request, response, "GetDOChannels");
}

bool IndyDCP3::get_pickit_3d_detection(const Nrmk::IndyFramework::VisionRequest& request, Nrmk::IndyFramework::VisionResult& response) {
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::GetPickit3DDetection,
                          request, response, "GetPickit3DDetection");
}

bool IndyDCP3::get_pickit_3d_retrieval(const Nrmk::IndyFramework::VisionRequest& request, Nrmk::IndyFramework::VisionResult& response) {
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::GetPickit3DRetrieval,
                          request, response, "GetPickit3DRetrieval");
}

bool IndyDCP3::set_ai(const Nrmk::IndyFramework::AnalogList& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::SetAI,
                          request, response, "SetAI");
}

bool IndyDCP3::set_di(const Nrmk::IndyFramework::DigitalList& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::SetDI,
                          request, response, "SetDI");
}

bool IndyDCP3::set_do_channels(const Nrmk::IndyFramework::DOChannelModes& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::SetDOChannels,
                          request, response, "SetDOChannels");
}

bool IndyDCP3::set_end_ai(const Nrmk::IndyFramework::AnalogList& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::SetEndAI,
                          request, response, "SetEndAI");
}

bool IndyDCP3::set_end_di(const Nrmk::IndyFramework::EndtoolSignalList& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::SetEndDI,
                          request, response, "SetEndDI");
}

bool IndyDCP3::sim_di_config(const Nrmk::IndyFramework::DISignals& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::SimDIConfig,
                          request, response, "SimDIConfig");
}

bool IndyDCP3::get_end_rs485_rx_for(const Nrmk::IndyFramework::Int& request, Nrmk::IndyFramework::EndtoolRS485Rx& response) {
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::GetEndRS485RxFor,
                          request, response, "GetEndRS485RxFor");
}

bool IndyDCP3::get_end_rs485_tx_for(const Nrmk::IndyFramework::Int& request, Nrmk::IndyFramework::EndtoolRS485Tx& response) {
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::GetEndRS485TxFor,
                          request, response, "GetEndRS485TxFor");
}

bool IndyDCP3::get_ft_sensor_data_for(const Nrmk::IndyFramework::Int& request, Nrmk::IndyFramework::FTSensorData& response) {
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::GetFTSensorDataFor,
                          request, response, "GetFTSensorDataFor");
}

bool IndyDCP3::get_sander_command_for(const Nrmk::IndyFramework::Int& request, Nrmk::IndyFramework::SanderCommand& response) {
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::GetSanderCommandFor,
                          request, response, "GetSanderCommandFor");
}

bool IndyDCP3::set_conveyor_arm_index(const Nrmk::IndyFramework::Int& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(device_stub.get(),
                          &Nrmk::IndyFramework::Device::Stub::SetConveyorArmIndex,
                          request, response, "SetConveyorArmIndex");
}

bool IndyDCP3::get_di(Nrmk::IndyFramework::DigitalList &di_data) {
    /*
        address = uint32
        state = DigitalState
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::DigitalList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetDI(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetDI RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    di_data = response;
    return true;
}

bool IndyDCP3::get_do(Nrmk::IndyFramework::DigitalList &do_data) {
    /*
    signals = index
    address = uint32
    state = DigitalState
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::DigitalList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetDO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetDO RPC failed." << std::endl;
        return false;
    }

    do_data = response;

    return true;
}

bool IndyDCP3::set_do(const Nrmk::IndyFramework::DigitalList &do_signal_list) {

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetDO(&context, do_signal_list, &response);
    if (!status.ok()) {
        std::cerr << "SetDO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_ai(Nrmk::IndyFramework::AnalogList &ai_data) {
    /*
        address = uint32
        voltage = int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::AnalogList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetAI(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetAI RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    ai_data = response;
    return true;
}

bool IndyDCP3::get_ao(Nrmk::IndyFramework::AnalogList &ao_data) {
    /*
        address = uint32
        voltage = int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::AnalogList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetAO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetAO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    ao_data = response;
    return true;
}

bool IndyDCP3::set_ao(const Nrmk::IndyFramework::AnalogList &ao_signal_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetAO(&context, ao_signal_list, &response);
    if (!status.ok()) {
        std::cerr << "SetAO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_endtool_di(Nrmk::IndyFramework::EndtoolSignalList &endtool_di_data) {
    /*
        state = EndtoolState
        port = char value [A,B,C]
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::EndtoolSignalList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndDI(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetEndDI RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    endtool_di_data = response;
    return true;
}

bool IndyDCP3::get_endtool_do(Nrmk::IndyFramework::EndtoolSignalList &endtool_do_data) {
    /*
        state = EndtoolState
        port = char value [A,B,C]
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::EndtoolSignalList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndDO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetEndDO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    endtool_do_data = response;
    return true;
}

bool IndyDCP3::set_endtool_do(const Nrmk::IndyFramework::EndtoolSignalList &end_do_signal_list) {
    /*
        state = EndtoolState
        port = string
    */
    grpc::ClientContext context;
    Nrmk::IndyFramework::EndtoolSignalList request = end_do_signal_list;
    Nrmk::IndyFramework::Response response;

    grpc::Status status = device_stub->SetEndDO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetEndDO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_endtool_ai(Nrmk::IndyFramework::AnalogList &endtool_ai_data) {
    /*
        address = uint32
        voltage = int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::AnalogList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndAI(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetEndAI RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    endtool_ai_data = response;
    return true;
}

bool IndyDCP3::get_endtool_ao(Nrmk::IndyFramework::AnalogList &endtool_ao_data) {
    /*
        address = uint32
        voltage = int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::AnalogList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndAO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetEndAO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    endtool_ao_data = response;
    return true;
}

bool IndyDCP3::set_endtool_ao(const Nrmk::IndyFramework::AnalogList& end_ao_signal_list) {
    /*
        address = uint32
        voltage = int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetEndAO(&context, end_ao_signal_list, &response);
    if (!status.ok()) {
        std::cerr << "SetEndAO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_device_info(Nrmk::IndyFramework::DeviceInfo& device_info) {
    /*
        Device Info:
            num_joints          -> uint32
            robot_serial        -> string
            payload             -> float
            io_board_fw_ver     -> string
            core_board_fw_vers  -> string[]
            endtool_board_fw_ver-> string
            controller_ver      -> string
            controller_detail   -> string
            controller_date     -> string
            teleop_loaded       -> bool
            calibrated          -> bool
            response            -> Response
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::DeviceInfo response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetDeviceInfo(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetDeviceInfo RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    device_info = response;
    return true;
}

bool IndyDCP3::get_ft_sensor_data(Nrmk::IndyFramework::FTSensorData& ft_sensor_data) {
    /*
    FTSensorData:
        ft_Fx   -> float ;
        ft_Fy   -> float ;
        ft_Fz   -> float ;
        ft_Tx   -> float ;
        ft_Ty   -> float ;
        ft_Tz   -> float ;
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::FTSensorData response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetFTSensorData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetFTSensorData RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    ft_sensor_data = response;
    return true;
}

bool IndyDCP3::commit_violation(const Nrmk::IndyFramework::ViolationRequest& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = device_stub->CommitViolation(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "CommitViolation RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_rt_task_times(Nrmk::IndyFramework::TaskTimes& task_times) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = device_stub->GetRTTaskTimes(&context, request, &task_times);
    if (!status.ok()) {
        std::cerr << "GetRTTaskTimes RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_locked_joint(int index) {
    Nrmk::IndyFramework::Int request;
    request.set_value(index);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = device_stub->SetConveyorLockedJoint(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorLockedJoint RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_tool_link(int index) {
    Nrmk::IndyFramework::Int request;
    request.set_value(index);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = device_stub->SetConveyorToolLink(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorToolLink RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_endtool_rs485_rx(const Nrmk::IndyFramework::EndtoolRS485Rx& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetEndRS485Rx(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Endtool RS485 RX RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_endtool_rs485_rx(Nrmk::IndyFramework::EndtoolRS485Rx& rx_data) {
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndRS485Rx(&context, Nrmk::IndyFramework::Empty(), &rx_data);
    if (!status.ok()) {
        std::cerr << "Get Endtool RS485 RX RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_endtool_rs485_tx(Nrmk::IndyFramework::EndtoolRS485Tx& tx_data) {
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndRS485Tx(&context, Nrmk::IndyFramework::Empty(), &tx_data);
    if (!status.ok()) {
        std::cerr << "Get Endtool RS485 TX RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_end_led_dim(const Nrmk::IndyFramework::EndLedDim& request) {
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetEndLedDim(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set End LED Dim RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_conveyor(Nrmk::IndyFramework::Conveyor& conveyor_data) {
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetConveyor(&context, Nrmk::IndyFramework::Empty(), &conveyor_data);
    if (!status.ok()) {
        std::cerr << "Get Conveyor RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_by_name(const Nrmk::IndyFramework::Name& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorByName(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Conveyor By Name RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_conveyor_state(Nrmk::IndyFramework::ConveyorState& conveyor_state) {
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetConveyorState(&context, Nrmk::IndyFramework::Empty(), &conveyor_state);
    if (!status.ok()) {
        std::cerr << "Get Conveyor State RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_sander_command(const Nrmk::IndyFramework::SanderCommand::SanderType& sander_type,
                                  const std::string& ip,
                                  const float speed,
                                  const bool state) {
    Nrmk::IndyFramework::SanderCommand request;
    request.set_type(sander_type);
    request.set_ip(ip);
    request.set_speed(speed);
    request.set_state(state);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetSanderCommand(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Sander Command RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_sander_command(Nrmk::IndyFramework::SanderCommand& sander_command) {
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetSanderCommand(&context, Nrmk::IndyFramework::Empty(), &sander_command);
    if (!status.ok()) {
        std::cerr << "Get Sander Command RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_inspire_hand_command(const Nrmk::IndyFramework::InspireHandCommand& command) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetInspireHandCommand(&context, command, &response);
    if (!status.ok()) {
        std::cerr << "SetInspireHandCommand RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_inspire_hand_state(int tool_index, Nrmk::IndyFramework::InspireHandState& state) {
    Nrmk::IndyFramework::Int request;
    grpc::ClientContext context;

    request.set_value(tool_index);
    grpc::Status status = device_stub->GetInspireHandState(&context, request, &state);
    if (!status.ok()) {
        std::cerr << "GetInspireHandState RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_load_factors(Nrmk::IndyFramework::GetLoadFactorsRes& load_factors_res) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetLoadFactors(&context, request, &load_factors_res);
    if (!status.ok()) {
        std::cerr << "Get Load Factors RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_auto_mode(const bool on) {
    Nrmk::IndyFramework::SetAutoModeReq request;
    request.set_on(on);

    Nrmk::IndyFramework::SetAutoModeRes response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetAutoMode(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Auto Mode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::check_auto_mode(Nrmk::IndyFramework::CheckAutoModeRes& check_auto_mode_res) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->CheckAutoMode(&context, request, &check_auto_mode_res);
    if (!status.ok()) {
        std::cerr << "Check Auto Mode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::check_reduced_mode(Nrmk::IndyFramework::CheckReducedModeRes& reduced_mode_res) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->CheckReducedMode(&context, request, &reduced_mode_res);
    if (!status.ok()) {
        std::cerr << "Check Reduced Mode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_safety_function_state(Nrmk::IndyFramework::SafetyFunctionState& safety_function_state) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetSafetyFunctionState(&context, request, &safety_function_state);
    if (!status.ok()) {
        std::cerr << "Get Safety Function State RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::request_safety_function(const Nrmk::IndyFramework::SafetyFunctionState& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->RequestSafetyFunction(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Request Safety Function RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_safety_control_data(Nrmk::IndyFramework::SafetyControlData& safety_control_data) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetSafetyControlData(&context, request, &safety_control_data);
    if (!status.ok()) {
        std::cerr << "Get Safety Control Data RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_gripper_data(Nrmk::IndyFramework::GripperData& gripper_data) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetGripperData(&context, request, &gripper_data);
    if (!status.ok()) {
        std::cerr << "Get Gripper Data RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_gripper_data_for(int tool_index, Nrmk::IndyFramework::GripperData& gripper_data) {
    Nrmk::IndyFramework::Int request;
    grpc::ClientContext context;

    request.set_value(tool_index);

    grpc::Status status = device_stub->GetGripperDataFor(&context, request, &gripper_data);
    if (!status.ok()) {
        std::cerr << "Get Gripper Data For RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_gripper_command(const Nrmk::IndyFramework::GripperCommand& gripper_command) {
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetGripperCommand(&context, gripper_command, &response);
    if (!status.ok()) {
        std::cerr << "Set Gripper Command RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_brakes(const std::vector<bool>& brake_state_list) {
    Nrmk::IndyFramework::MotorList request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (size_t i = 0; i < brake_state_list.size(); ++i) {
        Nrmk::IndyFramework::Motor* motor = request.add_motors();
        motor->set_index(static_cast<uint32_t>(i));
        motor->set_enable(brake_state_list[i]);
    }

    grpc::Status status = device_stub->SetBrakes(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetBrakes RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_servo_all(const bool enable) {
    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetServoAll(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetServoAll RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_servo(const uint32_t index, const bool enable) {
    Nrmk::IndyFramework::Servo request;
    request.set_index(index);
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetServo(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetServo RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_endtool_led_dim(const uint32_t led_dim) {
    Nrmk::IndyFramework::EndLedDim request;
    request.set_led_dim(led_dim);

    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetEndLedDim(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetEndLedDim RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::execute_tool(const std::string& name) {
    Nrmk::IndyFramework::Name request;
    request.set_name(name);

    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->ExecuteTool(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ExecuteTool RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_brake_control_style(int& style) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::BrakeControlStyle response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetBrakeControlStyle(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetBrakeControlStyle RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    style = static_cast<int>(response.style());
    return true;
}

bool IndyDCP3::set_conveyor_name(const std::string& name) {
    Nrmk::IndyFramework::Name request;
    request.set_name(name);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorName(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorName RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_encoder(int encoder_type, int64_t channel1, int64_t channel2, int64_t sample_num,
                                    float mm_per_tick, float vel_const_mmps, bool reversed) {
    Nrmk::IndyFramework::Encoder request;
    request.set_type(static_cast<Nrmk::IndyFramework::Encoder::EncoderType>(encoder_type));
    request.set_channel1(channel1);
    request.set_channel2(channel2);
    request.set_sample_num(sample_num);
    request.set_mm_per_tick(mm_per_tick);
    request.set_vel_const_mmps(vel_const_mmps);
    request.set_reversed(reversed);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorEncoder(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorEncoder RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_trigger(int trigger_type, int64_t channel, bool detect_rise) {
    Nrmk::IndyFramework::Trigger request;
    request.set_type(static_cast<Nrmk::IndyFramework::Trigger::TriggerType>(trigger_type));
    request.set_channel(channel);
    request.set_detect_rise(detect_rise);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorTrigger(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorTrigger RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_offset(float offset_mm) {
    Nrmk::IndyFramework::Float request;
    request.set_value(offset_mm);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorOffset(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorOffset RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_starting_pose(const std::vector<float>& jpos, const std::vector<float>& tpos) {
    Nrmk::IndyFramework::PosePair request;

    for (const auto& pos : jpos) {
        request.add_q(pos);
    }

    for (const auto& pos : tpos) {
        request.add_p(pos);
    }

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorStartingPose(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorStartingPose RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_terminal_pose(const std::vector<float>& jpos, const std::vector<float>& tpos) {
    Nrmk::IndyFramework::PosePair request;

    for (const auto& pos : jpos) {
        request.add_q(pos);
    }

    for (const auto& pos : tpos) {
        request.add_p(pos);
    }

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorTerminalPose(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorTerminalPose RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::add_photoneo_calib_point(const std::string& vision_name, double px, double py, double pz) {
    Nrmk::IndyFramework::AddPhotoneoCalibPointReq request;
    request.set_vision_name(vision_name);
    request.set_px(px);
    request.set_py(py);
    request.set_pz(pz);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->AddPhotoneoCalibPoint(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "AddPhotoneoCalibPoint RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_photoneo_detection(const Nrmk::IndyFramework::VisionServer& vision_server,
                                      const std::string& object,
                                      const Nrmk::IndyFramework::VisionFrameType frame_type,
                                      Nrmk::IndyFramework::VisionResult& result,
                                      uint32_t arm_index) {
    Nrmk::IndyFramework::VisionRequest request;
    *request.mutable_vision_server() = vision_server;
    request.set_object(object);
    request.set_frame_type(frame_type);
    request.set_arm_index(arm_index);

    grpc::ClientContext context;
    grpc::Status status = device_stub->GetPhotoneoDetection(&context, request, &result);

    if (!status.ok()) {
        std::cerr << "GetPhotoneoDetection RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_photoneo_retrieval(const Nrmk::IndyFramework::VisionServer& vision_server,
                                      const std::string& object,
                                      const Nrmk::IndyFramework::VisionFrameType frame_type,
                                      Nrmk::IndyFramework::VisionResult& result,
                                      uint32_t arm_index) {
    Nrmk::IndyFramework::VisionRequest request;
    *request.mutable_vision_server() = vision_server;
    request.set_object(object);
    request.set_frame_type(frame_type);
    request.set_arm_index(arm_index);

    grpc::ClientContext context;
    grpc::Status status = device_stub->GetPhotoneoRetrieval(&context, request, &result);

    if (!status.ok()) {
        std::cerr << "GetPhotoneoRetrieval RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}
