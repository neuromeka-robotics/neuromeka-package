#include "indydcp3.h"
#include "indydcp3_rpc_utils.h"
#include <vector>

bool IndyDCP3::get_modbus_variable(Nrmk::IndyFramework::ModbusVars& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::GetModbusVariable,
                          request, response, "GetModbusVariable");
}

bool IndyDCP3::get_variable_name_list(Nrmk::IndyFramework::AllVars& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::GetVariableNameList,
                          request, response, "GetVariableNameList");
}

bool IndyDCP3::move_j_cond(const Nrmk::IndyFramework::MoveJCondReq& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::MoveJCond,
                          request, response, "MoveJCond");
}

bool IndyDCP3::pause_motion(const Nrmk::IndyFramework::PauseCat& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::PauseMotion,
                          request, response, "PauseMotion");
}

bool IndyDCP3::reset() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::Reset,
                          request, response, "Reset");
}

bool IndyDCP3::search_program(const Nrmk::IndyFramework::Program& request, Nrmk::IndyFramework::ProgramInfo& response) {
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::SearchProgram,
                          request, response, "SearchProgram");
}

bool IndyDCP3::send_alarm(const Nrmk::IndyFramework::Message& request) {
    Nrmk::IndyFramework::Empty response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::SendAlarm,
                          request, response, "SendAlarm");
}

bool IndyDCP3::send_annotation(const Nrmk::IndyFramework::Message& request) {
    Nrmk::IndyFramework::Empty response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::SendAnnotation,
                          request, response, "SendAnnotation");
}

bool IndyDCP3::set_modbus_variable(const Nrmk::IndyFramework::ModbusVars& request) {
    Nrmk::IndyFramework::Empty response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::SetModbusVariable,
                          request, response, "SetModbusVariable");
}

bool IndyDCP3::set_modbus_variable_name_list(const Nrmk::IndyFramework::ModbusVariableList& request) {
    Nrmk::IndyFramework::Empty response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::SetModbusVariableNameList,
                          request, response, "SetModbusVariableNameList");
}

bool IndyDCP3::set_tele_calib_arm(const Nrmk::IndyFramework::UInt& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::SetTeleCalibArm,
                          request, response, "SetTeleCalibArm");
}

bool IndyDCP3::set_variable_name_list(const Nrmk::IndyFramework::AllVars& request) {
    Nrmk::IndyFramework::Empty response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::SetVariableNameList,
                          request, response, "SetVariableNameList");
}

bool IndyDCP3::get_motion_c(const Nrmk::IndyFramework::GetMotionCReq& request, Nrmk::IndyFramework::GetMotionRes& response) {
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::GetMotionC,
                          request, response, "GetMotionC");
}

bool IndyDCP3::get_motion_j(const Nrmk::IndyFramework::GetMotionJReq& request, Nrmk::IndyFramework::GetMotionRes& response) {
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::GetMotionJ,
                          request, response, "GetMotionJ");
}

bool IndyDCP3::get_motion_l(const Nrmk::IndyFramework::GetMotionLReq& request, Nrmk::IndyFramework::GetMotionRes& response) {
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::GetMotionL,
                          request, response, "GetMotionL");
}

bool IndyDCP3::get_program_break_points(Nrmk::IndyFramework::ProgramBreakPoints& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::GetProgramBreakPoints,
                          request, response, "GetProgramBreakPoints");
}

bool IndyDCP3::get_transformed_ft_sensor_data_for(const Nrmk::IndyFramework::Int& request, Nrmk::IndyFramework::TransformedFTSensorData& response) {
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::GetTransformedFTSensorDataFor,
                          request, response, "GetTransformedFTSensorDataFor");
}

bool IndyDCP3::program_step_into() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::ProgramStepInto,
                          request, response, "ProgramStepInto");
}

bool IndyDCP3::program_step_out() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::ProgramStepOut,
                          request, response, "ProgramStepOut");
}

bool IndyDCP3::program_step_over() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::ProgramStepOver,
                          request, response, "ProgramStepOver");
}

bool IndyDCP3::read_teleop_input_for(const Nrmk::IndyFramework::Int& request, Nrmk::IndyFramework::TeleP& response) {
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::ReadTeleOpInputFor,
                          request, response, "ReadTeleOpInputFor");
}

bool IndyDCP3::set_program_break_points(const Nrmk::IndyFramework::ProgramBreakPoints& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(control_stub.get(),
                          &Nrmk::IndyFramework::Control::Stub::SetProgramBreakPoints,
                          request, response, "SetProgramBreakPoints");
}

bool IndyDCP3::stop_motion(const StopCategory stop_category) {
    /*
        stop_category -> StopCategory
            CAT0  = 0
            CAT1  = 1
            CAT2  = 2
    */
    Nrmk::IndyFramework::StopCat request;
    request.set_category(stop_category);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->StopMotion(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "StopMotion RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::movej(const std::vector<float>& jtarget,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float vel_ratio,
                     const float acc_ratio,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition,
                     const bool teaching_mode,
                     const std::optional<int> arm_index)
{
    /*
        Joint Move:
            blending_type -> BlendingType.Type
                NONE
                OVERRIDE
                DUPLICATE
            base_type -> JointBaseType
                ABSOLUTE
                RELATIVE
            vel_ratio (0-100) -> float
            acc_ratio (0-100) -> float
            teaching_mode -> bool
            arm_index -> optional<int>; nullopt omits the protobuf field
    */

    Nrmk::IndyFramework::MoveJReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // tarJ.set_j_start();
        if(jtarget.size()!=_cobotDOF) {
            return false;
        }

        // set target
        // for (std::size_t i=0;i<jtarget.size();i++){
        //     request.mutable_target()->add_j_target(jtarget[i]);
        // }
        for (const auto& pos : jtarget){
            request.mutable_target()->add_j_target(pos);
        }

        // set base type
        // request.mutable_target()->set_base_type(static_cast<JointBaseType>(base_type));
        request.mutable_target()->set_base_type(
            base_type == 1 ? JointBaseType::RELATIVE_JOINT : JointBaseType::ABSOLUTE_JOINT);

        // set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        //    BlendingType_Type_NONE      = 0
        //    BlendingType_Type_OVERRIDE  = 1
        //    BlendingType_Type_DUPLICATE = 2

        request.mutable_blending()->set_blending_radius(blending_radius);

        // set post_condition
        request.mutable_post_condition()->set_const_cond(const_cond);

        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        //    CONST_COND = 0;
        //    IO_COND    = 1;
        //    VAR_COND   = 2;

        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));
        //    NONE_COND  = 0;
        //    STOP_COND  = 1;
        //    PAUSE_COND = 2;

        for (const auto& [address, state] : di_condition.di)
        {
            Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }

        for (const auto& [address, state] : di_condition.end_di)
        {
            Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set post_condition - Variable condition
        for (const auto& [addr, value] : var_condition.bool_vars)
        {
            Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(addr);
            bool_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.int_vars)
        {
            Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(addr);
            int_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.float_vars)
        {
            Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(addr);
            float_buff->set_value(value);
        }

        for (const auto& [addr, values] : var_condition.joint_vars)
        {
            Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(addr);

            if (_cobotDOF != values.size())
                return false;

            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, values[i]);
        }

        for (const auto& [addr, values] : var_condition.task_vars)
        {
            Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(addr);

            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, values[i]);
        }

        for (const auto& [addr, value] : var_condition.modbus_vars)
        {
            Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(addr);
            modbus_buff->set_value(value);
        }

        request.set_vel_ratio(vel_ratio);
        request.set_acc_ratio(acc_ratio);
        request.set_teaching_mode(teaching_mode);
        if (arm_index.has_value()) {
            request.set_arm_index(*arm_index);
        }

        grpc::Status status = control_stub->MoveJ(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveJ RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch(std::string &err)
    {
        std::cout<<"error occur"<<err<<std::endl;
        return false;
    }
}

bool IndyDCP3::movej_time(const std::vector<float>& jtarget,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float move_time,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition,
                     const std::optional<int> arm_index)
{
    /*
        Joint Move Time:
            blending_type -> BlendingType.Type
                NONE
                OVERRIDE
                DUPLICATE
            base_type -> JointBaseType
                ABSOLUTE
                RELATIVE
            move_time -> float
            arm_index -> optional<int>; nullopt omits the protobuf field
    */

    Nrmk::IndyFramework::MoveJTReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // tarJ.set_j_start();
        if(jtarget.size()!=_cobotDOF) {
            return false;
        }

        // set target
        for (const auto& pos : jtarget){
            request.mutable_target()->add_j_target(pos);
        }

        // set base type
        request.mutable_target()->set_base_type(
            base_type == 1 ? JointBaseType::RELATIVE_JOINT : JointBaseType::ABSOLUTE_JOINT);

        // set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        //    BlendingType_Type_NONE      = 0
        //    BlendingType_Type_OVERRIDE  = 1
        //    BlendingType_Type_DUPLICATE = 2

        request.mutable_blending()->set_blending_radius(blending_radius);

        // set post_condition
        request.mutable_post_condition()->set_const_cond(const_cond);

        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        //    CONST_COND = 0;
        //    IO_COND    = 1;
        //    VAR_COND   = 2;

        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));
        //    NONE_COND  = 0;
        //    STOP_COND  = 1;
        //    PAUSE_COND = 2;

        // set post_condition - io condition
        for (const auto& [address, state] : di_condition.di)
        {
            Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }

        for (const auto& [address, state] : di_condition.end_di)
        {
            Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set post_condition - Variable condition
        for (const auto& [addr, value] : var_condition.bool_vars)
        {
            Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(addr);
            bool_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.int_vars)
        {
            Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(addr);
            int_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.float_vars)
        {
            Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(addr);
            float_buff->set_value(value);
        }

        for (const auto& [addr, values] : var_condition.joint_vars)
        {
            Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(addr);

            if (_cobotDOF != values.size())
                return false;

            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, values[i]);
        }

        for (const auto& [addr, values] : var_condition.task_vars)
        {
            Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(addr);

            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, values[i]);
        }

        for (const auto& [addr, value] : var_condition.modbus_vars)
        {
            Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(addr);
            modbus_buff->set_value(value);
        }

        request.set_time(move_time);
        if (arm_index.has_value()) {
            request.set_arm_index(*arm_index);
        }

        grpc::Status status = control_stub->MoveJT(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveJT RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch(std::string &err)
    {
        std::cout<<"error occur"<<err<<std::endl;
        return false;
    }
}

bool IndyDCP3::movel(const std::array<float, 6>& ttarget,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float vel_ratio,
                     const float acc_ratio,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition,
                     const bool teaching_mode,
                     const bool bypass_singular)
{
    /*
        ttarget = [mm, mm, mm, deg, deg, deg]
        base_tye -> TaskBaseType
            ABSOLUTE
            RELATIVE
            TCP
    */

    Nrmk::IndyFramework::MoveLReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // Set target
        // for (std::size_t i = 0; i < ttarget.size(); i++){
        //     request.mutable_target()->add_t_target(ttarget[i]);
        // }
        for (const auto& pos : ttarget){
            request.mutable_target()->add_t_target(pos);
        }

        // Set base type
        // request.mutable_target()->set_base_type(static_cast<TaskBaseType>(base_type));
        request.mutable_target()->set_base_type(
            base_type == 1 ? TaskBaseType::RELATIVE_TASK : TaskBaseType::ABSOLUTE_TASK);

        // Set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        request.mutable_blending()->set_blending_radius(blending_radius);

        // Set post_condition
        request.mutable_post_condition()->set_const_cond(const_cond);
        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

        // Set post_condition - IO condition
        for (const auto& [address, state] : di_condition.di)
        {
            Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }

        for (const auto& [address, state] : di_condition.end_di)
        {
            Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set post_condition - Variable condition
        for (const auto& [addr, value] : var_condition.bool_vars)
        {
            Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(addr);
            bool_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.int_vars)
        {
            Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(addr);
            int_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.float_vars)
        {
            Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(addr);
            float_buff->set_value(value);
        }

        for (const auto& [addr, values] : var_condition.joint_vars)
        {
            Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(addr);

            if (_cobotDOF != values.size())
                return false;

            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, values[i]);
        }

        for (const auto& [addr, values] : var_condition.task_vars)
        {
            Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(addr);

            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, values[i]);
        }

        for (const auto& [addr, value] : var_condition.modbus_vars)
        {
            Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(addr);
            modbus_buff->set_value(value);
        }

        request.set_vel_ratio(vel_ratio);
        request.set_acc_ratio(acc_ratio);
        request.set_teaching_mode(teaching_mode);
        request.set_bypass_singular(bypass_singular);

        grpc::Status status = control_stub->MoveL(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveL RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::string& err)
    {
        std::cerr << "Error occurred: " << err << std::endl;
        return false;
    }
}

bool IndyDCP3::movel_time(const std::array<float, 6>& ttarget,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float move_time,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition)
{
    /*
        ttarget = [mm, mm, mm, deg, deg, deg]
        base_tye -> TaskBaseType
            ABSOLUTE
            RELATIVE
            TCP
    */

    Nrmk::IndyFramework::MoveLTReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // Set target
        for (const auto& pos : ttarget){
            request.mutable_target()->add_t_target(pos);
        }

        // Set base type
        // request.mutable_target()->set_base_type(static_cast<TaskBaseType>(base_type));
        request.mutable_target()->set_base_type(
            base_type == 1 ? TaskBaseType::RELATIVE_TASK : TaskBaseType::ABSOLUTE_TASK);

        // Set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        request.mutable_blending()->set_blending_radius(blending_radius);

        // Set post_condition
        request.mutable_post_condition()->set_const_cond(const_cond);
        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

        // Set post_condition - IO condition
        for (const auto& [address, state] : di_condition.di)
        {
            Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }

        for (const auto& [address, state] : di_condition.end_di)
        {
            Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set post_condition - Variable condition
        for (const auto& [addr, value] : var_condition.bool_vars)
        {
            Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(addr);
            bool_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.int_vars)
        {
            Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(addr);
            int_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.float_vars)
        {
            Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(addr);
            float_buff->set_value(value);
        }

        for (const auto& [addr, values] : var_condition.joint_vars)
        {
            Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(addr);

            if (_cobotDOF != values.size())
                return false;

            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, values[i]);
        }

        for (const auto& [addr, values] : var_condition.task_vars)
        {
            Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(addr);

            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, values[i]);
        }

        for (const auto& [addr, value] : var_condition.modbus_vars)
        {
            Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(addr);
            modbus_buff->set_value(value);
        }

        request.set_time(move_time);

        grpc::Status status = control_stub->MoveLT(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveLT RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::string& err)
    {
        std::cerr << "Error occurred: " << err << std::endl;
        return false;
    }
}

bool IndyDCP3::movec(const std::array<float, 6>& tpos1,
                     const std::array<float, 6>& tpos2,
                     const float angle,
                     const int setting_type,
                     const int move_type,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float vel_ratio,
                     const float acc_ratio,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition,
                     const bool teaching_mode,
                     const bool bypass_singular)
{
    /*
        tstart = [mm, mm, mm, deg, deg, deg]
        ttarget = [mm, mm, mm, deg, deg, deg]
    */
    Nrmk::IndyFramework::MoveCReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // Set target positions
        // for (int i = 0; i < 6; i++)
        // {
        //     request.mutable_target()->add_t_pos0(tpos1[i]);
        //     request.mutable_target()->add_t_pos1(tpos2[i]);
        // }
        for (const auto& pos : tpos1){
            request.mutable_target()->add_t_pos0(pos);
        }
        for (const auto& pos : tpos2){
            request.mutable_target()->add_t_pos1(pos);
        }

        // Set base type
        request.mutable_target()->set_base_type(
            base_type == 1 ? TaskBaseType::RELATIVE_TASK : TaskBaseType::ABSOLUTE_TASK);

        // Set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        request.mutable_blending()->set_blending_radius(blending_radius);

        // Set post condition
        request.mutable_post_condition()->set_const_cond(const_cond);
        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

        // Set IO condition
        for (const auto& [address, state] : di_condition.di)
        {
            auto* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }
        for (const auto& [address, state] : di_condition.end_di)
        {
            auto* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set variable condition
        for (const auto& [address, value] : var_condition.bool_vars)
        {
            auto* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(address);
            bool_buff->set_value(value);
        }
        for (const auto& [address, value] : var_condition.int_vars)
        {
            auto* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(address);
            int_buff->set_value(value);
        }
        for (const auto& [address, value] : var_condition.float_vars)
        {
            auto* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(address);
            float_buff->set_value(value);
        }
        for (const auto& [address, jpos] : var_condition.joint_vars)
        {
            if (_cobotDOF != jpos.size())
                return false;

            auto* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(address);
            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, jpos[i]);
        }
        for (const auto& [address, tpos] : var_condition.task_vars)
        {
            auto* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(address);
            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, tpos[i]);
        }
        for (const auto& [address, value] : var_condition.modbus_vars)
        {
            auto* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(address);
            modbus_buff->set_value(value);
        }

        request.set_vel_ratio(vel_ratio);
        request.set_acc_ratio(acc_ratio);
        request.set_teaching_mode(teaching_mode);
        request.set_setting_type(static_cast<CircularSettingType>(setting_type));
        request.set_move_type(static_cast<CircularMovingType>(move_type));
        request.set_bypass_singular(bypass_singular);

        grpc::Status status = control_stub->MoveC(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveC RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return false;
    }
}

bool IndyDCP3::movec_time(const std::array<float, 6>& tpos1,
                     const std::array<float, 6>& tpos2,
                     const float angle,
                     const int setting_type,
                     const int move_type,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float move_time,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition)
{
    /*
        tstart = [mm, mm, mm, deg, deg, deg]
        ttarget = [mm, mm, mm, deg, deg, deg]
    */
    Nrmk::IndyFramework::MoveCTReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // Set target positions
        for (const auto& pos : tpos1){
            request.mutable_target()->add_t_pos0(pos);
        }
        for (const auto& pos : tpos2){
            request.mutable_target()->add_t_pos1(pos);
        }

        // Set base type
        request.mutable_target()->set_base_type(
            base_type == 1 ? TaskBaseType::RELATIVE_TASK : TaskBaseType::ABSOLUTE_TASK);

        // Set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        request.mutable_blending()->set_blending_radius(blending_radius);

        // Set post condition
        request.mutable_post_condition()->set_const_cond(const_cond);
        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

        // Set IO condition
        for (const auto& [address, state] : di_condition.di)
        {
            auto* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }
        for (const auto& [address, state] : di_condition.end_di)
        {
            auto* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set variable condition
        for (const auto& [address, value] : var_condition.bool_vars)
        {
            auto* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(address);
            bool_buff->set_value(value);
        }
        for (const auto& [address, value] : var_condition.int_vars)
        {
            auto* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(address);
            int_buff->set_value(value);
        }
        for (const auto& [address, value] : var_condition.float_vars)
        {
            auto* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(address);
            float_buff->set_value(value);
        }
        for (const auto& [address, jpos] : var_condition.joint_vars)
        {
            if (_cobotDOF != jpos.size())
                return false;

            auto* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(address);
            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, jpos[i]);
        }
        for (const auto& [address, tpos] : var_condition.task_vars)
        {
            auto* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(address);
            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, tpos[i]);
        }
        for (const auto& [address, value] : var_condition.modbus_vars)
        {
            auto* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(address);
            modbus_buff->set_value(value);
        }

        request.set_time(move_time);
        request.set_setting_type(static_cast<CircularSettingType>(setting_type));
        request.set_move_type(static_cast<CircularMovingType>(move_type));

        grpc::Status status = control_stub->MoveCT(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveC RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return false;
    }
}

bool IndyDCP3::move_gcode(const std::string& gcode_file,
                          const bool is_smooth_mode,
                          const float smooth_radius,
                          const float vel_ratio,
                          const float acc_ratio)
{
    Nrmk::IndyFramework::MoveGcodeReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_gcode_file(gcode_file);
    request.set_is_smooth_mode(is_smooth_mode);
    request.set_smooth_radius(smooth_radius);
    request.set_vel_ratio(vel_ratio);
    request.set_acc_ratio(acc_ratio);

    grpc::Status status = control_stub->MoveGcode(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "MoveGcode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::start_teleop(const TeleMethod method)
{
    /*
        method:
        TELE_TASK_ABSOLUTE = 0
        TELE_TASK_RELATIVE = 1
        TELE_JOINT_ABSOLUTE = 10
        TELE_JOINT_RELATIVE = 11
    */

    Nrmk::IndyFramework::TeleOpState request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_method(method);
    request.set_mode(Nrmk::IndyFramework::TeleMode::TELE_RAW);

    grpc::Status status = control_stub->StartTeleOp(&context, request, &response);
    if (!status.ok()){
        std::cerr << "Start Tele RPC failed." << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::stop_teleop()
{
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->StopTeleOp(&context, request, &response);
    if (!status.ok()){
        std::cerr << "Stop Tele RPC failed." << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::movetelej(const std::vector<float>& jpos,
                        const float vel_ratio,
                        const float acc_ratio,
                        const TeleMethod method)
{
    /*
        Joint Teleoperation
        jpos = [deg, deg, deg, deg, deg, deg]
    */

    Nrmk::IndyFramework::MoveTeleJReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    if(_cobotDOF != jpos.size()){
        return false;
    }

    request.clear_jpos();
    // for (unsigned int i=0; i<_cobotDOF; i++) {
    //    request.add_jpos(jpos[i]);
    //     // std::cout << "Joint " << jpos[i] << std::endl;
    // }
    for (const auto& pos : jpos){
        request.add_jpos(pos);
    }

    request.set_vel_ratio(vel_ratio);
    request.set_acc_ratio(acc_ratio);
    request.set_method(method);

    grpc::Status status = control_stub->MoveTeleJ(&context, request, &response);
    if (!status.ok()){
        std::cerr << "MoveTeleJ RPC failed." << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::movetelel(const std::array<float, 6>& tpos,
                        const float vel_ratio,
                        const float acc_ratio,
                        const TeleMethod method)
{
    /*
        Task Teleoperation
        jpos = [mm, mm, mm, deg, deg, deg]
    */

    Nrmk::IndyFramework::MoveTeleLReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.clear_tpos();
    // for(int i=0; i<6;i++){
    //     request.add_tpos(tpos[i]);
    // }
    for (const auto& pos : tpos){
        request.add_tpos(pos);
    }

    request.set_vel_ratio(vel_ratio);
    request.set_acc_ratio(acc_ratio);
    request.set_method(method);

    grpc::Status status = control_stub->MoveTeleL(&context, request, &response);
    if (!status.ok()){
        std::cerr << "MoveTeleL RPC failed." << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::inverse_kin(const std::array<float, 6>& tpos,
                           const std::vector<float>& init_jpos,
                           std::vector<float>& jpos)
{
    /*
        Inverse Kinematics
        tpos -> float[6]
        init_jpos -> float[]
        jpos -> float[]
    */

    Nrmk::IndyFramework::InverseKinematicsReq request;
    Nrmk::IndyFramework::InverseKinematicsRes response;
    grpc::ClientContext context;

    if(_cobotDOF!=init_jpos.size()){
        return false;
    }

    for (size_t i = 0; i < 6; ++i) {
        request.add_tpos(tpos[i]);
    }
    for (size_t i = 0; i < init_jpos.size(); ++i) {
        request.add_init_jpos(init_jpos[i]);
    }

    grpc::Status status = control_stub->InverseKinematics(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "InverseKinematics RPC failed." << std::endl;
        return false;
    }

    jpos.clear();
    for (int i = 0; i < response.jpos_size(); ++i) {
        jpos.push_back(response.jpos(i));
    }

    return true;
}

bool IndyDCP3::inverse_kin(const Nrmk::IndyFramework::InverseKinematicsReq& request,
                           Nrmk::IndyFramework::InverseKinematicsRes& response)
{
    /*
        Inverse Kinematics
        request  -> InverseKinematicsReq
        response -> InverseKinematicsRes
    */
    grpc::ClientContext context;

    if (_cobotDOF != request.init_jpos_size()) {
        return false;
    }

    grpc::Status status = control_stub->InverseKinematics(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "InverseKinematics RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_direct_teaching(bool enable)
{
    /*
        Direct Teaching
        enable -> bool
    */

    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetDirectTeaching(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "SetDirectTeaching RPC failed." << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_simulation_mode(bool enable)
{
    /*
        Simulation Mode
        enable -> bool
    */

    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetSimulationMode(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "SetSimulationMode RPC failed." << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::recover()
{
    /*
        Recover from Violation
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->Recover(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "Recover RPC failed." << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_manual_recovery(bool enable)
{
    /*
        Manual Recovery Mode
        enable -> bool
    */

    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetManualRecovery(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "SetManualRecovery RPC failed." << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::calculate_relative_pose(const std::array<float, 6>& start_pos,
                                       const std::array<float, 6>& end_pos,
                                       int base_type,
                                       std::array<float, 6>& relative_pose)
{
    /*
        Calculate Relative Pose
        start_pos -> float[6]
        end_pos -> float[6]
        base_type -> int
        relative_pose -> float[6]
    */

    Nrmk::IndyFramework::CalculateRelativePoseReq request;
    Nrmk::IndyFramework::CalculateRelativePoseRes response;
    grpc::ClientContext context;

    for (const auto& sp : start_pos) {
        request.add_start_pos(sp);
    }
    for (const auto& ep : end_pos) {
        request.add_end_pos(ep);
    }

    request.set_base_type(static_cast<TaskBaseType>(base_type));

    grpc::Status status = control_stub->CalculateRelativePose(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "CalculateRelativePose RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        relative_pose[i] = response.relative_pos(i);
    }

    return true;
}

bool IndyDCP3::calculate_current_pose_rel(const std::array<float, 6>& current_pos,
                                          const std::array<float, 6>& relative_pos,
                                          int base_type,
                                          std::array<float, 6>& calculated_pose)
{
    /*
        Calculate Current Pose Relative
        current_pos -> float[6]
        relative_pos -> float[6]
        base_type -> int
        calculated_pose -> float[6]
    */

    Nrmk::IndyFramework::CalculateCurrentPoseRelReq request;
    Nrmk::IndyFramework::CalculateCurrentPoseRelRes response;
    grpc::ClientContext context;

    for (const auto& cp : current_pos) {
        request.add_current_pos(cp);
    }
    for (const auto& rp : relative_pos) {
        request.add_relative_pos(rp);
    }

    request.set_base_type(static_cast<TaskBaseType>(base_type));
    grpc::Status status = control_stub->CalculateCurrentPoseRel(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "CalculateCurrentPoseRel RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        calculated_pose[i] = response.calculated_pos(i);
    }

    return true;
}

bool IndyDCP3::play_program(const std::string& prog_name, int prog_idx)
{
    /*
        Play Program
        prog_name -> std::string
        prog_idx -> int
    */

    Nrmk::IndyFramework::Program request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_prog_name(prog_name);
    request.set_prog_idx(prog_idx);

    grpc::Status status = control_stub->PlayProgram(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "PlayProgram RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::play_program_line(const Nrmk::IndyFramework::Program& program)
{
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->PlayProgramLine(&context, program, &response);
    if (!status.ok()) {
        std::cerr << "PlayProgramLine RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::pause_program()
{
    /*
        Pause Program
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->PauseProgram(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "PauseProgram RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::resume_program()
{
    /*
        Resume Program
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->ResumeProgram(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ResumeProgram RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::resume_program_debug()
{
    /*
        Resume Program Debug
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->ResumeProgramDebug(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ResumeProgramDebug RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::stop_program()
{
    /*
        Stop Program
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->StopProgram(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "StopProgram RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_bool_variable(std::vector<Nrmk::IndyFramework::BoolVariable>& bool_variables)
{
    /*
        Get Bool Variables
        bool_variables -> std::vector<BoolVariable>
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::BoolVars response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetBoolVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetBoolVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    bool_variables.assign(response.variables().begin(), response.variables().end());
    return true;
}

bool IndyDCP3::get_int_variable(std::vector<Nrmk::IndyFramework::IntVariable>& int_variables)
{
    /*
        Get Integer Variables
        int_variables -> std::vector<IntVariable>
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::IntVars response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetIntVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetIntVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    int_variables.assign(response.variables().begin(), response.variables().end());
    return true;
}

bool IndyDCP3::get_float_variable(std::vector<Nrmk::IndyFramework::FloatVariable>& float_variables)
{
    /*
        Get Float Variables
        float_variables -> std::vector<FloatVariable>
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::FloatVars response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetFloatVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetFloatVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    float_variables.assign(response.variables().begin(), response.variables().end());
    return true;
}

bool IndyDCP3::get_jpos_variable(std::vector<Nrmk::IndyFramework::JPosVariable>& jpos_variables)
{
    /*
        Get JPos Variables
        jpos_variables -> std::vector<JPosVariable>
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::JPosVars response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetJPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetJPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    jpos_variables.assign(response.variables().begin(), response.variables().end());
    return true;
}

bool IndyDCP3::get_tpos_variable(std::vector<Nrmk::IndyFramework::TPosVariable>& tpos_variables)
{
    /*
        Get TPos Variables
        tpos_variables -> std::vector<TPosVariable>
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::TPosVars response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetTPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    tpos_variables.assign(response.variables().begin(), response.variables().end());
    return true;
}

bool IndyDCP3::set_bool_variable(const std::vector<Nrmk::IndyFramework::BoolVariable>& bool_variables)
{
    /*
        Set Bool Variables
        bool_variables -> std::vector<BoolVariable>
    */

    Nrmk::IndyFramework::BoolVars request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    for (const auto& var : bool_variables) {
        *request.add_variables() = var;
    }

    grpc::Status status = control_stub->SetBoolVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetBoolVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_int_variable(const std::vector<Nrmk::IndyFramework::IntVariable>& int_variables)
{
    /*
        Set Int Variables
        int_variables -> std::vector<IntVariable>
    */

    Nrmk::IndyFramework::IntVars request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    for (const auto& var : int_variables) {
        *request.add_variables() = var;
    }

    grpc::Status status = control_stub->SetIntVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetIntVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_float_variable(const std::vector<Nrmk::IndyFramework::FloatVariable>& float_variables)
{
    /*
        Set Float Variables
        float_variables -> std::vector<FloatVariable>
    */

    Nrmk::IndyFramework::FloatVars request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    for (const auto& var : float_variables) {
        *request.add_variables() = var;
    }

    grpc::Status status = control_stub->SetFloatVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetFloatVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_jpos_variable(const std::vector<Nrmk::IndyFramework::JPosVariable>& jpos_variables)
{
    /*
        Set JPos Variables
        jpos_variables -> std::vector<JPosVariable>
    */

    Nrmk::IndyFramework::JPosVars request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    for (const auto& var : jpos_variables) {
        *request.add_variables() = var;
    }

    grpc::Status status = control_stub->SetJPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetJPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_tpos_variable(const std::vector<Nrmk::IndyFramework::TPosVariable>& tpos_variables)
{
    /*
        Set TPos Variables
        tpos_variables -> std::vector<TPosVariable>
    */

    Nrmk::IndyFramework::TPosVars request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    for (const auto& var : tpos_variables) {
        *request.add_variables() = var;
    }

    grpc::Status status = control_stub->SetTPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetTPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_plugin_bool_variable(const std::string& name, bool value)
{
    Nrmk::IndyFramework::NamedBool request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    request.set_name(name);
    request.set_value(value);

    grpc::Status status = control_stub->SetPluginBoolVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPluginBoolVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plugin_bool_variable(const std::string& name, bool& value)
{
    Nrmk::IndyFramework::Name request;
    Nrmk::IndyFramework::NamedBool response;
    grpc::ClientContext context;

    request.set_name(name);
    grpc::Status status = control_stub->GetPluginBoolVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPluginBoolVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    value = response.value();
    return true;
}

bool IndyDCP3::set_plugin_int_variable(const std::string& name, int64_t value)
{
    Nrmk::IndyFramework::NamedInt request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    request.set_name(name);
    request.set_value(value);

    grpc::Status status = control_stub->SetPluginIntVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPluginIntVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plugin_int_variable(const std::string& name, int64_t& value)
{
    Nrmk::IndyFramework::Name request;
    Nrmk::IndyFramework::NamedInt response;
    grpc::ClientContext context;

    request.set_name(name);
    grpc::Status status = control_stub->GetPluginIntVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPluginIntVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    value = response.value();
    return true;
}

bool IndyDCP3::set_plugin_float_variable(const std::string& name, float value)
{
    Nrmk::IndyFramework::NamedFloat request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    request.set_name(name);
    request.set_value(value);

    grpc::Status status = control_stub->SetPluginFloatVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPluginFloatVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plugin_float_variable(const std::string& name, float& value)
{
    Nrmk::IndyFramework::Name request;
    Nrmk::IndyFramework::NamedFloat response;
    grpc::ClientContext context;

    request.set_name(name);
    grpc::Status status = control_stub->GetPluginFloatVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPluginFloatVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    value = response.value();
    return true;
}

bool IndyDCP3::set_plugin_jpos_variable(const std::string& name, const std::vector<float>& jpos)
{
    Nrmk::IndyFramework::NamedJointPosition request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    request.set_name(name);
    for (float v : jpos) request.add_jpos(v);

    grpc::Status status = control_stub->SetPluginJPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPluginJPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plugin_jpos_variable(const std::string& name, std::vector<float>& jpos)
{
    Nrmk::IndyFramework::Name request;
    Nrmk::IndyFramework::NamedJointPosition response;
    grpc::ClientContext context;

    request.set_name(name);
    grpc::Status status = control_stub->GetPluginJPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPluginJPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    jpos.assign(response.jpos().begin(), response.jpos().end());
    return true;
}

bool IndyDCP3::set_plugin_tpos_variable(const std::string& name, const std::vector<float>& tpos)
{
    Nrmk::IndyFramework::NamedTaskPosition request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    request.set_name(name);
    for (float v : tpos) request.add_tpos(v);

    grpc::Status status = control_stub->SetPluginTPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPluginTPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plugin_tpos_variable(const std::string& name, std::vector<float>& tpos)
{
    Nrmk::IndyFramework::Name request;
    Nrmk::IndyFramework::NamedTaskPosition response;
    grpc::ClientContext context;

    request.set_name(name);
    grpc::Status status = control_stub->GetPluginTPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPluginTPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    tpos.assign(response.tpos().begin(), response.tpos().end());
    return true;
}

bool IndyDCP3::activate_sdk(const Nrmk::IndyFramework::SDKLicenseInfo& request,
                                Nrmk::IndyFramework::SDKLicenseResp& response)
{
    /*
        Activate SDK
        request -> SDKLicenseInfo (input)
            - license_key -> std::string
            - expire_date -> std::string
        response -> SDKLicenseResp (output)
            - activated -> bool, True if activated
            - response (code, msg)
                - 0, 'Activated'                -> SDK Activated
                - 1, 'Invalid'                  -> Wrong key or expire date
                - 2, 'No Internet Connection'   -> Need Internet for License Verification
                - 3, 'Expired'                  -> License Expired
                - 4, 'HW_FAILURE'               -> Failed acquire HW ID to verify licens
    */

    grpc::ClientContext context;

    // Call the gRPC method
    grpc::Status status = control_stub->ActivateIndySDK(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "Activate SDK RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_custom_control_mode(const int mode) {
    /*
        Set Custom Control Mode
        mode -> int
    */
    Nrmk::IndyFramework::IntMode request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_mode(mode);

    grpc::Status status = control_stub->SetCustomControlMode(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Custom Control Mode RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_custom_control_mode(int& mode) {
    /*
        Get Custom Control Mode
        mode -> int (output)
    */
    Nrmk::IndyFramework::IntMode response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetCustomControlMode(&context, Nrmk::IndyFramework::Empty(), &response);
    if (!status.ok()) {
        std::cerr << "Get Custom Control Mode RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    // mode = response.mode();
    mode = static_cast<int>(response.mode());
    return true;
}

bool IndyDCP3::wait_time(float time)
{
    Nrmk::IndyFramework::WaitTimeReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_time(time);

    // Note: set_* signal lists have been removed from WaitTimeReq in latest API.

    grpc::Status status = control_stub->WaitTime(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Wait Time RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::wait_progress(int progress)
{
    Nrmk::IndyFramework::WaitProgressReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_progress(progress);

    // Note: set_* signal lists have been removed from WaitProgressReq in latest API.

    grpc::Status status = control_stub->WaitProgress(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Wait Progress RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::wait_traj(const Nrmk::IndyFramework::TrajCondition& traj_condition)
{
    Nrmk::IndyFramework::WaitTrajReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_traj_condition(traj_condition);

    // Note: set_* signal lists have been removed from WaitTrajReq in latest API.

    grpc::Status status = control_stub->WaitTraj(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Wait Traj RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::wait_radius(int radius)
{
    Nrmk::IndyFramework::WaitRadiusReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_radius(radius);

    // Note: set_* signal lists have been removed from WaitRadiusReq in latest API.

    grpc::Status status = control_stub->WaitRadius(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Wait Radius RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::movelf(const std::array<float, 6>& ttarget,
                      const std::vector<bool>& enabledaxis,
                      const std::vector<float>& desforce,
                      const int base_type,
                      const int blending_type,
                      const float blending_radius,
                      const float vel_ratio,
                      const float acc_ratio,
                      const bool const_cond,
                      const int cond_type,
                      const int react_type,
                      DCPDICond di_condition,
                      DCPVarCond var_condition,
                      const bool teaching_mode)
{
    Nrmk::IndyFramework::MoveLFReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // Set target
        for (const auto& pos : ttarget){
            request.mutable_target()->add_t_target(pos);
        }

        // Set base type
        // request.mutable_target()->set_base_type(static_cast<TaskBaseType>(base_type));
        request.mutable_target()->set_base_type(
            base_type == 1 ? TaskBaseType::RELATIVE_TASK : TaskBaseType::ABSOLUTE_TASK);

        // Set blending type and radius
        request.mutable_blending()->set_type(static_cast<Nrmk::IndyFramework::BlendingType::Type>(blending_type));
        request.mutable_blending()->set_blending_radius(blending_radius);

        // Set desforce and enabledaxis
        for (const auto& force : desforce){
            request.add_des_force(force);
        }
        for (const auto& axis : enabledaxis){
            request.add_enabled_force(axis);
        }

        // Set post condition
        request.mutable_post_condition()->set_const_cond(const_cond);
        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

        // Set post_condition - IO condition
        for (const auto& [address, state] : di_condition.di)
        {
            Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }

        for (const auto& [address, state] : di_condition.end_di)
        {
            Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set post_condition - Variable condition
        for (const auto& [addr, value] : var_condition.bool_vars)
        {
            Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(addr);
            bool_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.int_vars)
        {
            Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(addr);
            int_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.float_vars)
        {
            Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(addr);
            float_buff->set_value(value);
        }

        for (const auto& [addr, values] : var_condition.joint_vars)
        {
            Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(addr);

            if (_cobotDOF != values.size())
                return false;

            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, values[i]);
        }

        for (const auto& [addr, values] : var_condition.task_vars)
        {
            Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(addr);

            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, values[i]);
        }

        for (const auto& [addr, value] : var_condition.modbus_vars)
        {
            Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(addr);
            modbus_buff->set_value(value);
        }

        request.set_vel_ratio(vel_ratio);
        request.set_acc_ratio(acc_ratio);
        request.set_teaching_mode(teaching_mode);

        // Call the gRPC method
        grpc::Status status = control_stub->MoveLF(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveLF RPC failed: " << status.error_message() << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::string& err)
    {
        std::cerr << "Error occurred: " << err << std::endl;
        return false;
    }
}

bool IndyDCP3::get_transformed_ft_sensor_data(Nrmk::IndyFramework::TransformedFTSensorData& ft_sensor_data) {
    /*
    Transformed Force/Torque Sensor Data:
        ft_Fx -> float N
        ft_Fy -> float N
        ft_Fz -> float N
        ft_Tx -> float N*m
        ft_Ty -> float N*m
        ft_Tz -> float N*m
    */
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTransformedFTSensorData(&context, request, &ft_sensor_data);
    if (!status.ok()) {
        std::cerr << "Get Transformed FT Sensor Data RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::move_joint_traj(const std::vector<std::vector<float>>& q_list,
                               const std::vector<std::vector<float>>& qdot_list,
                               const std::vector<std::vector<float>>& qddot_list) {
    Nrmk::IndyFramework::MoveJointTrajReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try {
        // q_list
        for (const auto& q : q_list) {
            Nrmk::IndyFramework::Vector* q_vector = request.add_q_list();
            for (const auto& value : q) {
                q_vector->add_values(value);
            }
        }

        // qdot_list
        for (const auto& qdot : qdot_list) {
            Nrmk::IndyFramework::Vector* qdot_vector = request.add_qdot_list();
            for (const auto& value : qdot) {
                qdot_vector->add_values(value);
            }
        }

        // qddot_list
        for (const auto& qddot : qddot_list) {
            Nrmk::IndyFramework::Vector* qddot_vector = request.add_qddot_list();
            for (const auto& value : qddot) {
                qddot_vector->add_values(value);
            }
        }

        grpc::Status status = control_stub->MoveJointTraj(&context, request, &response);
        if (!status.ok()) {
            std::cerr << "MoveJointTraj RPC failed: " << status.error_message() << std::endl;
            return false;
        }
        return true;
    } catch (const std::string& err) {
        std::cerr << "Error occurred: " << err << std::endl;
        return false;
    }
}

bool IndyDCP3::move_task_traj(const std::vector<std::vector<float>>& p_list,
                              const std::vector<std::vector<float>>& pdot_list,
                              const std::vector<std::vector<float>>& pddot_list) {
    Nrmk::IndyFramework::MoveTaskTrajReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try {
        // p_list
        for (const auto& p : p_list) {
            Nrmk::IndyFramework::Vector* p_vector = request.add_p_list();
            for (const auto& value : p) {
                p_vector->add_values(value);
            }
        }

        // pdot_list
        for (const auto& pdot : pdot_list) {
            Nrmk::IndyFramework::Vector* pdot_vector = request.add_pdot_list();
            for (const auto& value : pdot) {
                pdot_vector->add_values(value);
            }
        }

        // pddot_list
        for (const auto& pddot : pddot_list) {
            Nrmk::IndyFramework::Vector* pddot_vector = request.add_pddot_list();
            for (const auto& value : pddot) {
                pddot_vector->add_values(value);
            }
        }

        grpc::Status status = control_stub->MoveTaskTraj(&context, request, &response);
        if (!status.ok()) {
            std::cerr << "MoveTaskTraj RPC failed: " << status.error_message() << std::endl;
            return false;
        }
        return true;
    } catch (const std::string& err) {
        std::cerr << "Error occurred: " << err << std::endl;
        return false;
    }
}

bool IndyDCP3::move_conveyor(const bool teaching_mode,
                             const bool bypass_singular,
                             const float acc_ratio,
                             const bool const_cond,
                             const int cond_type,
                             const int react_type,
                             DCPDICond di_condition,
                             DCPVarCond var_condition)
{
    Nrmk::IndyFramework::MoveConveyorReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_teaching_mode(teaching_mode);
    request.set_bypass_singular(bypass_singular);
    request.set_acc_ratio(acc_ratio);

    // Set post condition
    request.mutable_post_condition()->set_const_cond(const_cond);
    request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
    request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

    // Set post_condition - IO condition
    for (const auto& [address, state] : di_condition.di)
    {
        Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
        di_buff->set_address(address);
        di_buff->set_state(static_cast<DigitalState>(state));
    }

    for (const auto& [address, state] : di_condition.end_di)
    {
        Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
        enddi_buff->set_address(address);
        enddi_buff->set_state(static_cast<DigitalState>(state));
    }

    // Set post_condition - Variable condition
    for (const auto& [addr, value] : var_condition.bool_vars)
    {
        Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
        bool_buff->set_addr(addr);
        bool_buff->set_value(value);
    }

    for (const auto& [addr, value] : var_condition.int_vars)
    {
        Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
        int_buff->set_addr(addr);
        int_buff->set_value(value);
    }

    for (const auto& [addr, value] : var_condition.float_vars)
    {
        Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
        float_buff->set_addr(addr);
        float_buff->set_value(value);
    }

    for (const auto& [addr, values] : var_condition.joint_vars)
    {
        Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
        jpos_buff->set_addr(addr);

        if (_cobotDOF != values.size())
            return false;

        for (unsigned int i = 0; i < _cobotDOF; i++)
            jpos_buff->set_jpos(i, values[i]);
    }

    for (const auto& [addr, values] : var_condition.task_vars)
    {
        Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
        tpos_buff->set_addr(addr);

        for (int i = 0; i < 6; i++)
            tpos_buff->set_tpos(i, values[i]);
    }

    for (const auto& [addr, value] : var_condition.modbus_vars)
    {
        Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
        modbus_buff->set_addr(addr);
        modbus_buff->set_value(value);
    }

    grpc::Status status = control_stub->MoveConveyor(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "MoveConveyor RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::move_axis(const std::array<float, 3>& start_mm,
                         const std::array<float, 3>& target_mm,
                         const bool is_absolute,
                         const float vel_ratio,
                         const float acc_ratio,
                         const bool teaching_mode)
{
    Nrmk::IndyFramework::MoveAxisReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    // start and target positions
    for (const auto& pos : start_mm) {
        request.add_start_mm(pos);
    }
    for (const auto& pos : target_mm) {
        request.add_target_mm(pos);
    }

    // velocity and acceleration ratios
    request.set_vel_percentage(vel_ratio);
    request.set_acc_percentage(acc_ratio);

    // movement type and teaching mode
    request.set_is_absolute(is_absolute);
    request.set_teaching_mode(teaching_mode);

    grpc::Status status = control_stub->MoveLinearAxis(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "MoveLinearAxis RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::forward_kin(const Nrmk::IndyFramework::ForwardKinematicsReq& request,
                           Nrmk::IndyFramework::ForwardKinematicsRes& response)
{
    grpc::ClientContext context;

    grpc::Status status = control_stub->ForwardKinematics(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ForwardKinematics RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::joint_to_tcp_transform(const Nrmk::IndyFramework::JointToTcpTransformReq& request,
                                      Nrmk::IndyFramework::JointToTcpTransformRes& response)
{
    grpc::ClientContext context;

    grpc::Status status = control_stub->JointToTcpTransform(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "JointToTcpTransform RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_tact_time(const Nrmk::IndyFramework::TactTime& tact_time) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetTactTime(&context, tact_time, &response);
    if (!status.ok()) {
        std::cerr << "SetTactTime RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_tact_time(Nrmk::IndyFramework::TactTime& tact_time) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTactTime(&context, request, &tact_time);
    if (!status.ok()) {
        std::cerr << "GetTactTime RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::move_recover_joint(const std::vector<float>& jtarget,
                                const int base_type) {
    /*
        Move recover joint
        jtarget = [deg, deg, deg, deg, deg, deg]
    */

    Nrmk::IndyFramework::TargetJ request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& joint_pos : jtarget) {
        request.add_j_target(joint_pos);
    }

    request.set_base_type(base_type == 1 ? JointBaseType::RELATIVE_JOINT : JointBaseType::ABSOLUTE_JOINT);

    grpc::Status status = control_stub->MoveRecoverJoint(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "MoveRecoverJoint RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_control_info(Nrmk::IndyFramework::ControlInfo& control_info) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetControlInfo(&context, request, &control_info);
    if (!status.ok()) {
        std::cerr << "GetControlInfo RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_compliance_mode(const Nrmk::IndyFramework::ComplianceMode& mode) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->SetComplianceMode(&context, mode, &response);
    if (!status.ok()) {
        std::cerr << "SetComplianceMode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_compliance_mode(Nrmk::IndyFramework::ComplianceMode& mode) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = control_stub->GetComplianceMode(&context, request, &mode);
    if (!status.ok()) {
        std::cerr << "GetComplianceMode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::push_bus_event(const Nrmk::IndyFramework::BusEvent& event) {
    Nrmk::IndyFramework::State response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->PushBusEvent(&context, event, &response);
    if (!status.ok()) {
        std::cerr << "PushBusEvent RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.enable();
}

bool IndyDCP3::catch_bus_event(const Nrmk::IndyFramework::CatchBusEventReq& request,
                               Nrmk::IndyFramework::BusEvent& event) {
    grpc::ClientContext context;
    grpc::Status status = control_stub->CatchBusEvent(&context, request, &event);
    if (!status.ok()) {
        std::cerr << "CatchBusEvent RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_force_mode(const Nrmk::IndyFramework::ForceModeReq& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->SetForceMode(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetForceMode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_force_mode(Nrmk::IndyFramework::ForceModeReq& response) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = control_stub->GetForceMode(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetForceMode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::ping_from_conty() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->PingFromConty(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "PingFromConty RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_ft_zero() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->FTZero(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "FTZero RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_inference_data(const Nrmk::IndyFramework::ControlInferenceDataSet& data) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->SetControlInferenceData(&context, data, &response);
    if (!status.ok()) {
        std::cerr << "SetControlInferenceData RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_inference_data(Nrmk::IndyFramework::ControlInferenceDataSet& data) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = control_stub->GetControlInferenceData(&context, request, &data);
    if (!status.ok()) {
        std::cerr << "GetControlInferenceData RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::check_aproach_retract_valid(const std::array<float, 6>& tpos,
                                           const std::vector<float>& init_jpos,
                                           const std::array<float, 6>& pre_tpos,
                                           const std::array<float, 6>& post_tpos,
                                           Nrmk::IndyFramework::CheckAproachRetractValidRes& response)
{
    Nrmk::IndyFramework::CheckAproachRetractValidReq request;
    grpc::ClientContext context;

    for (const auto& pos : tpos) {
        request.add_tpos(pos);
    }
    for (const auto& jpos : init_jpos) {
        request.add_init_jpos(jpos);
    }
    for (const auto& pos : pre_tpos) {
        request.add_pre_tpos(pos);
    }
    for (const auto& pos : post_tpos) {
        request.add_post_tpos(pos);
    }

    grpc::Status status = control_stub->CheckAproachRetractValid(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Check Aproach Retract Valid RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.is_valid();
}

bool IndyDCP3::get_pallet_point_list(const std::array<float, 6>& tpos,
                                     const std::vector<float>& jpos,
                                     const std::array<float, 6>& pre_tpos,
                                     const std::array<float, 6>& post_tpos,
                                     const int pallet_pattern,
                                     const int width,
                                     const int height,
                                     Nrmk::IndyFramework::GetPalletPointListRes& response)
{
    Nrmk::IndyFramework::GetPalletPointListReq request;
    grpc::ClientContext context;

    for (const auto& pos : tpos) {
        request.add_tpos(pos);
    }
    for (const auto& joint_pos : jpos) {
        request.add_jpos(joint_pos);
    }
    for (const auto& pos : pre_tpos) {
        request.add_pre_tpos(pos);
    }
    for (const auto& pos : post_tpos) {
        request.add_post_tpos(pos);
    }
    request.set_pallet_pattern(pallet_pattern);
    request.set_width(width);
    request.set_height(height);

    grpc::Status status = control_stub->GetPalletPointList(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get Pallet Point List RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::play_tuning_program(const std::string& prog_name,
                                   const int prog_idx,
                                   const Nrmk::IndyFramework::TuningSpace tuning_space,
                                   const Nrmk::IndyFramework::TuningPrecision precision,
                                   const uint32_t vel_level_max,
                                   Nrmk::IndyFramework::CollisionThresholds& response)
{
    Nrmk::IndyFramework::TuningProgram request;
    grpc::ClientContext context;

    request.mutable_program()->set_prog_name(prog_name);
    request.mutable_program()->set_prog_idx(prog_idx);
    request.set_tuning_space(tuning_space);
    request.set_precision(precision);
    request.set_vel_level_max(vel_level_max);

    grpc::Status status = control_stub->PlayTuningProgram(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Play Tuning Program RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_io_variable(const Nrmk::IndyFramework::IOVars& io_vars) {
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetIOVariable(&context, io_vars, &response);
    if (!status.ok()) {
        std::cerr << "SetIOVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_io_variable(Nrmk::IndyFramework::IOVars& io_vars) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetIOVariable(&context, request, &io_vars);
    if (!status.ok()) {
        std::cerr << "GetIOVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::wait_io(
    const std::vector<Nrmk::IndyFramework::DigitalSignal>& di_signal_list,
    const std::vector<Nrmk::IndyFramework::DigitalSignal>& do_signal_list,
    const std::vector<Nrmk::IndyFramework::DigitalSignal>& end_di_signal_list,
    const std::vector<Nrmk::IndyFramework::DigitalSignal>& end_do_signal_list,
    const int conjunction)
{
    Nrmk::IndyFramework::WaitIOReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& signal : di_signal_list) {
        auto* di = request.add_di_list();
        di->set_address(signal.address());
        di->set_state(static_cast<Nrmk::IndyFramework::DigitalState>(signal.state()));
    }

    for (const auto& signal : do_signal_list) {
        auto* do_signal = request.add_do_list();
        do_signal->set_address(signal.address());
        do_signal->set_state(static_cast<Nrmk::IndyFramework::DigitalState>(signal.state()));
    }

    for (const auto& signal : end_di_signal_list) {
        auto* end_di = request.add_end_di_list();
        end_di->set_address(signal.address());
        end_di->set_state(static_cast<Nrmk::IndyFramework::DigitalState>(signal.state()));
    }

    for (const auto& signal : end_do_signal_list) {
        auto* end_do = request.add_end_do_list();
        end_do->set_address(signal.address());
        end_do->set_state(static_cast<Nrmk::IndyFramework::DigitalState>(signal.state()));
    }

    request.set_conjunction(conjunction);

    // Note: set_* lists have been removed from WaitIOReq in latest API.

    grpc::Status status = control_stub->WaitIO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "WaitIO RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_friction_comp_state(const bool enable) {
    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetFrictionCompensation(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetFrictionCompensation RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_friction_comp_state() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::State response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetFrictionCompensationState(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetFrictionCompensationState RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    bool is_enabled = response.enable();
    return is_enabled;
}

bool IndyDCP3::get_teleop_device(Nrmk::IndyFramework::TeleOpDevice& device) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTeleOpDevice(&context, request, &device);
    if (!status.ok()) {
        std::cerr << "GetTeleOpDevice RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_teleop_state(Nrmk::IndyFramework::TeleOpState& state) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTeleOpState(&context, request, &state);
    if (!status.ok()) {
        std::cerr << "GetTeleOpState RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::connect_teleop_device(const std::string& name,
                                     const Nrmk::IndyFramework::TeleOpDevice_TeleOpDeviceType type,
                                     const std::string& ip,
                                     const uint32_t port) {
    Nrmk::IndyFramework::TeleOpDevice request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_name(name);
    request.set_type(type);
    request.set_ip(ip);
    request.set_port(port);

    grpc::Status status = control_stub->ConnectTeleOpDevice(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ConnectTeleOpDevice RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::disconnect_teleop_device() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->DisConnectTeleOpDevice(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "DisConnectTeleOpDevice RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::read_teleop_input(Nrmk::IndyFramework::TeleP& teleop_input) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->ReadTeleOpInput(&context, request, &teleop_input);
    if (!status.ok()) {
        std::cerr << "ReadTeleOpInput RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_play_rate(const float rate) {
    Nrmk::IndyFramework::TelePlayRate request;
    request.set_rate(rate);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetPlayRate(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPlayRate RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_play_rate(float& rate) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::TelePlayRate response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetPlayRate(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPlayRate RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    rate = response.rate();
    return true;
}

bool IndyDCP3::get_tele_file_list(std::vector<std::string>& files) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::TeleOpFileList response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTeleFileList(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetTeleFileList RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    files.assign(response.files().begin(), response.files().end());
    return true;
}

bool IndyDCP3::save_tele_motion(const std::string& name) {
    Nrmk::IndyFramework::TeleFileReq request;
    request.set_name(name);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SaveTeleMotion(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SaveTeleMotion RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::load_tele_motion(const std::string& name) {
    Nrmk::IndyFramework::TeleFileReq request;
    request.set_name(name);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->LoadTeleMotion(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "LoadTeleMotion RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::delete_tele_motion(const std::string& name) {
    Nrmk::IndyFramework::TeleFileReq request;
    request.set_name(name);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->DeleteTeleMotion(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "DeleteTeleMotion RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::enable_tele_key(const bool enable) {
    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->EnableTeleKey(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "EnableTeleKey RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}
