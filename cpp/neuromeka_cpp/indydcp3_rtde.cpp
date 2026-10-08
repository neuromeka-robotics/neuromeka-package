#include "indydcp3.h"
#include "indydcp3_rpc_utils.h"
#include <vector>

bool IndyDCP3::get_robot_data(Nrmk::IndyFramework::ControlData &control_data) {
    /*
    Control Data:
        running_hours   -> uint32
        running_mins   -> uint32
        running_secs  -> uint32
        op_state  -> OpState
        sim_mode  -> bool
        q  -> float[6]
        qdot  -> float[6]
        p  -> float[6]
        pdot  -> float[6]
        ref_frame  -> float[6]
        tool_frame  -> float[6]
        ref_links  -> int[]
        response  -> Response
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ControlData response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetControlData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetControlData/RobotData RPC failed." << std::endl;
        return false;
    }

    control_data.set_running_hours(response.running_hours());
    control_data.set_running_mins(response.running_mins());
    control_data.set_running_secs(response.running_secs());

    control_data.set_op_state(response.op_state());
    control_data.set_sim_mode(response.sim_mode());

    control_data.clear_q();
    control_data.clear_qdot();
    for(unsigned int i=0; i<_cobotDOF; i++){
        control_data.add_q(response.q(i));
        control_data.add_qdot(response.qdot(i));
    }

    for(int i=0; i<6; i++) {
        control_data.add_p(response.p(i));
        control_data.add_pdot(response.pdot(i));

        control_data.add_ref_frame(response.ref_frame(i));
        control_data.add_tool_frame(response.tool_frame(i));
    }

    control_data.clear_ref_links();
    for (const auto link_index : response.ref_links()) {
        control_data.add_ref_links(link_index);
    }

    return true;
}

bool IndyDCP3::test_function(const Nrmk::IndyFramework::TestRequest& request, Nrmk::IndyFramework::TestResponse& response) {
    return call_unary_rpc(rtde_stub.get(),
                          &Nrmk::IndyFramework::RTDataExchange::Stub::TestFunction,
                          request, response, "TestFunction");
}

bool IndyDCP3::get_control_state(Nrmk::IndyFramework::ControlData2 &control_state) {
    /*
    Control Data:
        q  -> float[]
        qdot  -> float[]
        qddot  -> float[]
        qdes  -> float[]
        qdotdes  -> float[]
        qddotdes  -> float[]
        p  -> float[]
        pdot  -> float[]
        pddot  -> float[]
        pdes  -> float[]
        pdotdes  -> float[]
        pddotdes  -> float[]
        tau  -> float[]
        tau_act  -> float[]
        tau_ext  -> float[]
        tau_jts  -> float[]
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ControlData2 response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetControlState(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetControlState RPC failed." << std::endl;
        return false;
    }

    control_state.clear_q();
    control_state.clear_qdot();
    control_state.clear_qddot();
    control_state.clear_qdes();
    control_state.clear_qdotdes();
    control_state.clear_qddotdes();
    control_state.clear_tau();
    control_state.clear_tau_act();
    control_state.clear_tau_ext();
    control_state.clear_tau_jts();
    for(unsigned int i=0; i<_cobotDOF; i++){
        control_state.add_q(response.q(i));
        control_state.add_qdot(response.qdot(i));
        control_state.add_qddot(response.qddot(i));
        control_state.add_qdes(response.qdes(i));
        control_state.add_qdotdes(response.qdotdes(i));
        control_state.add_qddotdes(response.qddotdes(i));
        control_state.add_tau(response.tau(i));
        control_state.add_tau_act(response.tau_act(i));
        control_state.add_tau_ext(response.tau_ext(i));
        control_state.add_tau_jts(response.tau_jts(i));
    }

    for(int i=0; i<6; i++){
        control_state.add_p(response.p(i));
        control_state.add_pdot(response.pdot(i));
        control_state.add_pddot(response.pddot(i));
        control_state.add_pdes(response.pdes(i));
        control_state.add_pdotdes(response.pdotdes(i));
        control_state.add_pddotdes(response.pddotdes(i));
    }

    return true;
}

bool IndyDCP3::get_motion_data(Nrmk::IndyFramework::MotionData &motion_data) {
    /*
    Motion Data:
        traj_state   -> TrajState
        traj_progress   -> int32
        is_in_motion  -> bool
        is_target_reached  -> bool
        is_pausing  -> bool
        is_stopping  -> bool
        has_motion  -> bool
        speed_ratio  -> int32
        motion_id  -> int32
        remain_distance  -> float
        motion_queue_size  -> uint32
        cur_traj_progress  -> int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::MotionData response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetMotionData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetMotionData RPC failed." << std::endl;
        return false;
    }

    motion_data.set_traj_state(static_cast<Nrmk::IndyFramework::TrajState>(response.traj_state()));
    motion_data.set_traj_progress(response.traj_progress());

    motion_data.set_is_in_motion(response.is_in_motion());
    motion_data.set_is_target_reached(response.is_target_reached());
    motion_data.set_is_pausing(response.is_pausing());
    motion_data.set_is_stopping(response.is_stopping());
    motion_data.set_has_motion(response.has_motion());

    motion_data.set_speed_ratio(response.speed_ratio());
    motion_data.set_motion_id(response.motion_id());
    motion_data.set_remain_distance(response.remain_distance());
    motion_data.set_motion_queue_size(response.motion_queue_size());
    motion_data.set_cur_traj_progress(response.cur_traj_progress());

    return true;
}

bool IndyDCP3::get_servo_data(Nrmk::IndyFramework::ServoData &servo_data) {
    /*
    Servo Data:
        status_codes   -> string[]
        temperatures   -> float[]
        voltages  -> float[]
        currents  -> float[]
        servo_actives  -> bool[]
        brake_actives  -> bool[]
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ServoData response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetServoData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetServoData RPC failed." << std::endl;
        return false;
    }

    servo_data.clear_status_codes();
    servo_data.clear_temperatures();
    servo_data.clear_voltages();
    servo_data.clear_currents();
    servo_data.clear_servo_actives();
    servo_data.clear_brake_actives();
    for(unsigned int i=0; i<_cobotDOF; i++){
        servo_data.add_status_codes(response.status_codes(i));
        servo_data.add_temperatures(response.temperatures(i));
        servo_data.add_voltages(response.voltages(i));
        servo_data.add_currents(response.currents(i));
        servo_data.add_servo_actives(response.servo_actives(i));
        servo_data.add_brake_actives(response.brake_actives(i));
    }

    return true;
}

bool IndyDCP3::get_violation_data(Nrmk::IndyFramework::ViolationData &violation_data) {
    /*
    Violation Data:
        violation_code   -> uint64
        j_index   -> uint32
        i_args  -> int32[]
        f_args  -> float[]
        violation_str  -> string
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ViolationData response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetViolationData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetViolationData RPC failed." << std::endl;
        return false;
    }

    violation_data.set_violation_code(response.violation_code());
    violation_data.set_j_index(response.j_index());

    violation_data.clear_i_args();
    for(int i=0; i<response.i_args_size(); i++){
        violation_data.add_i_args(response.i_args(i));
    }

    violation_data.clear_f_args();
    for(int i=0; i<response.f_args_size(); i++){
        violation_data.add_f_args(response.f_args(i));
    }
    violation_data.set_violation_str(response.violation_str());

    return true;
}

bool IndyDCP3::get_program_data(Nrmk::IndyFramework::ProgramData &program_data) {
    /*
    Program Data:
        program_state   -> ProgramState
        cmd_id   -> int32
        sub_cmd_id  -> int32
        running_hours  -> int32
        running_mins  -> int32
        running_secs  -> int32
        program_name  -> string
        program_alarm  -> string
        program_annotation  -> string
        speed_ratio -> int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ProgramData response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetProgramData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetProgramData RPC failed." << std::endl;
        return false;
    }

    program_data.set_program_state(static_cast<ProgramState>(response.program_state()));
    program_data.set_cmd_id(response.cmd_id());
    program_data.set_sub_cmd_id(response.sub_cmd_id());
    program_data.set_running_hours(response.running_hours());
    program_data.set_running_mins(response.running_mins());
    program_data.set_running_secs(response.running_secs());
    program_data.set_program_name(response.program_name());
    program_data.set_program_alarm(response.program_alarm());
    program_data.set_program_annotation(response.program_annotation());
    program_data.set_speed_ratio(response.speed_ratio());

    return true;
}

bool IndyDCP3::get_collision_model_state(Nrmk::IndyFramework::CollisionModelState& state) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = rtde_stub->GetCollisionModelState(&context, request, &state);
    if (!status.ok()) {
        std::cerr << "GetCollisionModelState RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_reserved_data(Nrmk::IndyFramework::ReservedData& data) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = rtde_stub->GetReservedData(&context, request, &data);
    if (!status.ok()) {
        std::cerr << "GetReservedData RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_violation_message_queue(Nrmk::IndyFramework::ViolationMessageQueue& violation_queue) {
    /*
    Violation Data:
        violation_queue   -> ViolationData[]
    */
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetViolationMessageQueue(&context, Nrmk::IndyFramework::Empty(), &violation_queue);
    if (!status.ok()) {
        std::cerr << "Get Violation Message RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_stop_state(Nrmk::IndyFramework::StopState& stop_state) {
    /*
    Program Data:
        category   -> StopCategory
    */
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetStopState(&context, Nrmk::IndyFramework::Empty(), &stop_state);
    if (!status.ok()) {
        std::cerr << "Get Stop State RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_io_data(Nrmk::IndyFramework::IOData& response) {
    /*
        IO Data:
        di   -> DigitalSignal[]
        do   -> DigitalSignal[]
        ai  -> AnalogSignal[]
        ao  -> AnalogSignal[]
        end_di  -> EndtoolSignal[]
        end_do  -> EndtoolSignal[]
        end_ai  -> AnalogSignal[]
        end_ao  -> AnalogSignal[]
        response  -> Response
    */
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetIOData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get IO Data RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}
