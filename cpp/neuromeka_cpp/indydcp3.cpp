#include "indydcp3.h"
#include <vector>

IndyDCP3::IndyDCP3(const std::string& robot_ip, int index)
:_isConnected(false)
{
    if (index != 0 && index != 1) {
        throw std::invalid_argument("Index must be 0 or 1");
    }

    if (!_isConnected) {

        device_channel  = grpc::CreateChannel(robot_ip + ":" + std::to_string(DEVICE_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        control_channel = grpc::CreateChannel(robot_ip + ":" + std::to_string(CONTROL_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        config_channel  = grpc::CreateChannel(robot_ip + ":" + std::to_string(CONFIG_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        rtde_channel    = grpc::CreateChannel(robot_ip + ":" + std::to_string(RTDE_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        teleop_channel  = grpc::CreateChannel(robot_ip + ":" + std::to_string(TELEOP_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        cri_channel     = grpc::CreateChannel(robot_ip + ":" + std::to_string(CRI_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        boot_channel    = grpc::CreateChannel(robot_ip + ":" + std::to_string(BOOT_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());

        device_stub     = Nrmk::IndyFramework::Device::NewStub(device_channel);
        control_stub    = Nrmk::IndyFramework::Control::NewStub(control_channel);
        config_stub     = Nrmk::IndyFramework::Config::NewStub(config_channel);
        rtde_stub       = Nrmk::IndyFramework::RTDataExchange::NewStub(rtde_channel);
        teleop_stub     = Nrmk::IndyFramework::TeleOp::NewStub(teleop_channel);
        cri_stub        = Nrmk::IndyFramework::CRI::NewStub(cri_channel);
        boot_stub       = Nrmk::IndyFramework::Boot::NewStub(boot_channel);

        Nrmk::IndyFramework::Empty request;
        Nrmk::IndyFramework::DeviceInfo response;
        grpc::ClientContext context;
        grpc::Status status = device_stub->GetDeviceInfo(&context, request, &response);
        if(status.ok()){
            _cobotDOF = response.num_joints();
            std::cout << "Cobot" << index << " DOF:" << _cobotDOF << std::endl;
            _isConnected = true;
        }
    }
}

IndyDCP3::~IndyDCP3() {}

bool IndyDCP3::get_control_data(Nrmk::IndyFramework::ControlData &control_data) {
    return get_robot_data(control_data);
}

bool IndyDCP3::get_locked_joint(int& index) {
    Nrmk::IndyFramework::ControlData control_data;
    if (!get_control_data(control_data)) {
        return false;
    }

    index = control_data.locked_joint();
    return true;
}

bool IndyDCP3::add_joint_waypoint(const std::vector<float>& waypoint) {
    _joint_waypoint.push_back(waypoint);
    return true;
}

bool IndyDCP3::get_joint_waypoint(std::vector<std::vector<float>>& waypoints) const {
    if (_joint_waypoint.empty()) {
        return false;
    }
    waypoints = _joint_waypoint;
    return true;
}

bool IndyDCP3::clear_joint_waypoint() {
    _joint_waypoint.clear();
    return true;
}

bool IndyDCP3::move_joint_waypoint(float move_time)
{
    for (const auto& wp : _joint_waypoint) {
        if (move_time < 0) {
            movej(wp,
                JointBaseType::ABSOLUTE_JOINT,
                BlendingType_Type::BlendingType_Type_OVERRIDE);
        } else {
            movej_time(wp,
                JointBaseType::ABSOLUTE_JOINT,
                BlendingType_Type::BlendingType_Type_OVERRIDE,
                0.0,
                move_time);
        }
        wait_progress(100);
    }
    return true;
}

bool IndyDCP3::add_task_waypoint(const std::array<float, 6>& waypoint) {
    _task_waypoint.push_back(waypoint);
    return true;
}

bool IndyDCP3::get_task_waypoint(std::vector<std::array<float, 6>>& waypoints) const{
    if (_task_waypoint.empty()) {
        return false;
    }
    waypoints = _task_waypoint;
    return true;
}

bool IndyDCP3::clear_task_waypoint() {
    _task_waypoint.clear();
    return true;
}

bool IndyDCP3::move_task_waypoint(float move_time)
{
    for (const auto& wp : _task_waypoint) {
        if (move_time < 0) {
            movel(wp,
                TaskBaseType::ABSOLUTE_TASK,
                BlendingType_Type::BlendingType_Type_OVERRIDE);
        } else {
            movel_time(wp,
                TaskBaseType::ABSOLUTE_TASK,
                BlendingType_Type::BlendingType_Type_OVERRIDE,
                0.0,
                move_time);
        }
        wait_progress(100);
    }
    return true;
}

bool IndyDCP3::move_home(){
    Nrmk::IndyFramework::JointPos home_pos;
    bool is_success = false;
    is_success = get_home_pos(home_pos);
    if (!is_success) {
        return false;
    }

    std::vector<float> j_pos;
    for (int i = 0; i < home_pos.jpos_size(); i++) {
        j_pos.push_back(home_pos.jpos(i));
    }

    is_success = movej(j_pos);
    if (is_success) {
        return true;
    } else {
        return false;
    }
}

bool IndyDCP3::wait_for_operation_state(int wait_op_state) {
    if (wait_op_state != -1) {
        bool isDone = false;
        while (!isDone) {
            Nrmk::IndyFramework::ControlData crr_control_data;
            bool is_success = get_robot_data(crr_control_data);
            if (is_success){
                if (crr_control_data.op_state() == wait_op_state){
                    std::cout << "Wait finish" << std::endl;
                    isDone = true;
                }
            }
            else{
                std::cout << "Failed to get_robot_data" << std::endl;
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return true;
}

bool IndyDCP3::wait_for_motion_state(const std::string& wait_motion_state) {
    std::vector<std::string> motion_list = {"is_in_motion", "is_target_reached", "is_pausing", "is_stopping", "has_motion"};

    if (!wait_motion_state.empty() && std::find(motion_list.begin(), motion_list.end(), wait_motion_state) != motion_list.end()) {

        bool isDone = false;
        while (!isDone) {
            Nrmk::IndyFramework::MotionData crr_motion_data;
            bool is_success = get_motion_data(crr_motion_data);

            if (wait_motion_state == "is_in_motion") {
                isDone = crr_motion_data.is_in_motion();
            } else if (wait_motion_state == "is_target_reached") {
                isDone = crr_motion_data.is_target_reached();
            } else if (wait_motion_state == "is_pausing") {
                isDone = crr_motion_data.is_pausing();
            } else if (wait_motion_state == "is_stopping") {
                isDone = crr_motion_data.is_stopping();
            } else if (wait_motion_state == "has_motion") {
                isDone = crr_motion_data.has_motion();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    else {
        std::cerr << "Invalid or empty motion state: " << wait_motion_state << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::start_log() {
    /*
        Start realtime data logging
    */
    std::vector<Nrmk::IndyFramework::IntVariable> int_vars_to_set;
    Nrmk::IndyFramework::IntVariable int_var;
    int_var.set_addr(300);
    int_var.set_value(1);
    int_vars_to_set.push_back(int_var);

    return set_int_variable(int_vars_to_set);
}

bool IndyDCP3::end_log() {
    /*
        Finish realtime data logging and save the realtime data in STEP
        saved path:
            /home/user/release/IndyDeployments/RTlog/RTLog.csv
    */
    std::vector<Nrmk::IndyFramework::IntVariable> int_vars_to_set;
    Nrmk::IndyFramework::IntVariable int_var;
    int_var.set_addr(300);
    int_var.set_value(2);
    int_vars_to_set.push_back(int_var);

    return set_int_variable(int_vars_to_set);
}
