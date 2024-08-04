#include <iostream>
#include "../neuromeka_cpp/indydcp3.h"
#include <thread>
#include <chrono>

int main() {
    IndyDCP3 indy("192.168.1.24");

    bool is_success;

    Nrmk::IndyFramework::ControlData control_data;
    is_success = indy.get_robot_data(control_data);
    if (is_success){
        std::cout << "Control data:" << std::endl;
        for (int i = 0; i < control_data.q_size(); i++)
            std::cout << "q" << i << ": " << control_data.q(i) << std::endl;
        for (int i = 0; i < control_data.qdot_size(); i++)
            std::cout << "qdot" << i << ": " << control_data.qdot(i) << std::endl;
    }
    else{
        std::cout << "Failed to get_robot _data" << std::endl;
    }

    is_success = indy.get_control_data(control_data);
    if (is_success){
        std::cout << "running_hours: " << control_data.running_hours() << std::endl;
        std::cout << "running_mins: " << control_data.running_mins() << std::endl;
        std::cout << "running_secs: " << control_data.running_secs() << std::endl;
        std::cout << "op_state: " << control_data.op_state() << std::endl;
        std::cout << "sim_mode: " << control_data.sim_mode() << std::endl;
    }
    else{
        std::cout << "Failed to get_control_data" << std::endl;
    }

    Nrmk::IndyFramework::DigitalList di_data;
    is_success = indy.get_di(di_data);
    if (is_success) {
        std::cout << "GetDI RPC succeeded." << std::endl;
        for (int i = 0; i < di_data.signals_size(); i++) {
            const auto& signal = di_data.signals(i);
            std::cout << "Address: " << signal.address() << ", State: " << signal.state() << std::endl;
        }
    } else {
        std::cerr << "GetDI RPC failed." << std::endl;
    }

    Nrmk::IndyFramework::DigitalList do_data;
    is_success = indy.get_do(do_data);
    if (is_success) {
        std::cout << "GetDO RPC succeeded." << std::endl;
        for (int i = 0; i < do_data.signals_size(); i++) {
            const auto& signal = do_data.signals(i);
            std::cout << "Address: " << signal.address() << ", State: " << signal.state() << std::endl;
        }
    } 
    else {
        std::cerr << "GetDO RPC failed." << std::endl;
    }

    // Nrmk::IndyFramework::DigitalList do_signal_list;
    // auto *do_signal1 = do_signal_list.add_signals();
    // do_signal1->set_address(1);
    // do_signal1->set_state(DigitalState::OFF_STATE);

    // auto *do_signal5 = do_signal_list.add_signals();
    // do_signal5->set_address(5);
    // do_signal5->set_state(DigitalState::ON_STATE);

    // auto *do_signal9 = do_signal_list.add_signals();
    // do_signal9->set_address(9);
    // do_signal9->set_state(DigitalState::ON_STATE);

    // is_success = indy.set_do(do_signal_list);
    // if (is_success) {
    //     std::cout << "SetDO RPC succeeded." << std::endl;
    // } else {
    //     std::cerr << "SetDO RPC failed." << std::endl;
    // }

    Nrmk::IndyFramework::AnalogList ai_data;
    is_success = indy.get_ai(ai_data);
    if (is_success) {
        std::cout << "GetDI RPC succeeded." << std::endl;
        for (int i = 0; i < ai_data.signals_size(); i++) {
            const auto& signal = ai_data.signals(i);
            std::cout << "Address: " << signal.address() << ", Voltage: " << signal.voltage() << std::endl;
        }
    } else {
        std::cerr << "GetDI RPC failed." << std::endl;
    }

    Nrmk::IndyFramework::AnalogList ao_data;
    is_success = indy.get_ao(ao_data);
    if (is_success) {
        std::cout << "GetAO RPC succeeded." << std::endl;
        for (int i = 0; i < ao_data.signals_size(); ++i) {
            const auto &signal = ao_data.signals(i);
            std::cout << "Address: " << signal.address() << ", Voltage: " << signal.voltage() << std::endl;
        }
    } else {
        std::cerr << "GetAO RPC failed." << std::endl;
    }

    // Nrmk::IndyFramework::AnalogList ao_signal_list;
    // auto* ao_signal1 = ao_signal_list.add_signals();
    // ao_signal1->set_address(1);
    // ao_signal1->set_voltage(1000);

    // auto* ao_signal2 = ao_signal_list.add_signals();
    // ao_signal2->set_address(2);
    // ao_signal2->set_voltage(2000);

    // is_success = indy.set_ao(ao_signal_list);
    // if (is_success) {
    //     std::cout << "SetAO RPC succeeded." << std::endl;
    // } else {
    //     std::cerr << "SetAO RPC failed." << std::endl;
    // }


    Nrmk::IndyFramework::EndtoolSignalList endtool_di_data;
    is_success = indy.get_endtool_di(endtool_di_data);
    if (is_success) {
        std::cout << "GetEndDI RPC succeeded." << std::endl;
        for (int i = 0; i < endtool_di_data.signals_size(); i++) {
            const auto& signal = endtool_di_data.signals(i);
            std::cout << "Port: " << signal.port() << std::endl;
            for (const auto& state : signal.states()) {
                std::cout << "State: " << state << std::endl;
            }
        }
    } else {
        std::cerr << "GetEndDI RPC failed." << std::endl;
    }


    Nrmk::IndyFramework::EndtoolSignalList endtool_do_data;
    is_success = indy.get_endtool_do(endtool_do_data);
    if (is_success) {
        std::cout << "GetEndDO RPC succeeded." << std::endl;
        for (int i = 0; i < endtool_do_data.signals_size(); i++) {
            const auto& signal = endtool_do_data.signals(i);
            std::cout << "Port: " << signal.port() << std::endl;
            for (const auto& state : signal.states()) {
                std::cout << "State: " << state << std::endl;
            }
        }
    } else {
        std::cerr << "GetEndDO RPC failed." << std::endl;
    }

    // Nrmk::IndyFramework::EndtoolSignalList end_do_signal_list;
    // auto* end_signal1 = end_do_signal_list.add_signals();
    // end_signal1->set_port("C");
    // end_signal1->add_states(EndtoolState::UNUSED);

    // is_success = indy.set_endtool_do(end_do_signal_list);
    // if (is_success) {
    //     std::cout << "SetEndDO RPC succeeded." << std::endl;
    // } else {
    //     std::cerr << "SetEndDO RPC failed." << std::endl;
    // }

    Nrmk::IndyFramework::AnalogList endtool_ai_data;
    is_success = indy.get_endtool_ai(endtool_ai_data);
    if (is_success) {
        std::cout << "GetEndAI RPC succeeded." << std::endl;
        for (int i = 0; i < endtool_ai_data.signals_size(); i++) {
            const auto& signal = endtool_ai_data.signals(i);
            std::cout << "Address: " << signal.address() << ", Voltage: " << signal.voltage() << std::endl;
        }
    } else {
        std::cerr << "GetEndAI RPC failed." << std::endl;
    }

    Nrmk::IndyFramework::AnalogList endtool_ao_data;
    is_success = indy.get_endtool_ao(endtool_ao_data);
    if (is_success) {
        std::cout << "GetEndAO RPC succeeded." << std::endl;
        for (int i = 0; i < endtool_ao_data.signals_size(); i++) {
            const auto& signal = endtool_ao_data.signals(i);
            std::cout << "Address: " << signal.address() << ", Voltage: " << signal.voltage() << std::endl;
        }
    } else {
        std::cerr << "GetEndAO RPC failed." << std::endl;
    }

    // Nrmk::IndyFramework::AnalogList end_ao_signal_list;
    // auto* end_ao_signal1 = end_ao_signal_list.add_signals();
    // end_ao_signal1->set_address(1);
    // end_ao_signal1->set_voltage(1000);

    // is_success = indy.set_endtool_ao(end_ao_signal_list);
    // if (is_success) {
    //     std::cout << "SetEndAO RPC succeeded." << std::endl;
    // } else {
    //     std::cerr << "SetEndAO RPC failed." << std::endl;
    // }


    Nrmk::IndyFramework::DeviceInfo device_info;
    is_success = indy.get_device_info(device_info);
    if (is_success) {
        std::cout << "GetDeviceInfo RPC succeeded." << std::endl;
        std::cout << "Num joints: " << device_info.num_joints() << std::endl;
        std::cout << "Robot serial: " << device_info.robot_serial() << std::endl;
        std::cout << "Payload: " << device_info.payload() << std::endl;
        std::cout << "IO board FW version: " << device_info.io_board_fw_ver() << std::endl;
        std::cout << "Core board FW versions: ";
        for (int i = 0; i < device_info.core_board_fw_vers_size(); ++i) {
            std::cout << device_info.core_board_fw_vers(i) << " ";
        }
        std::cout << std::endl;
        std::cout << "Endtool board FW version: " << device_info.endtool_board_fw_ver() << std::endl;
        std::cout << "Controller version: " << device_info.controller_ver() << std::endl;
        std::cout << "Controller detail: " << device_info.controller_detail() << std::endl;
        std::cout << "Controller date: " << device_info.controller_date() << std::endl;
    } else {
        std::cerr << "GetDeviceInfo RPC failed." << std::endl;
    }


    Nrmk::IndyFramework::FTSensorData ft_sensor_data;
    is_success = indy.get_ft_sensor_data(ft_sensor_data);
    if (is_success) {
        std::cout << "GetFTSensorData RPC succeeded." << std::endl;
        std::cout << "FT Fx: " << ft_sensor_data.ft_fx() << std::endl;
        std::cout << "FT Fy: " << ft_sensor_data.ft_fy() << std::endl;
        std::cout << "FT Fz: " << ft_sensor_data.ft_fz() << std::endl;
        std::cout << "FT Tx: " << ft_sensor_data.ft_tx() << std::endl;
        std::cout << "FT Ty: " << ft_sensor_data.ft_ty() << std::endl;
        std::cout << "FT Tz: " << ft_sensor_data.ft_tz() << std::endl;
    } else {
        std::cerr << "GetFTSensorData RPC failed." << std::endl;
    }

    // StopCategory stop_category = StopCategory::SMOOTH_BRAKE;
    // is_success = indy.stop_motion(stop_category);
    // if (is_success) {
    //     std::cout << "StopMotion succeeded." << std::endl;
    // } else {
    //     std::cerr << "StopMotion failed." << std::endl;
    // }

    Nrmk::IndyFramework::JointPos home_jpos;
    is_success = indy.get_home_pos(home_jpos);
    if (is_success) {
        std::cout << "Get Home Pos RPC succeeded." << std::endl;
        for (int i = 0; i < home_jpos.jpos_size(); i++) {
            std::cout << "Joint " << i << " position: " << home_jpos.jpos(i) << std::endl;
        }
    }
    else {
        std::cerr << "Get Home Pos RPC failed." << std::endl;
    }

    // std::vector<float> j_pos = {0.0, 0.0, -90.0, 0.0, -90.0, 0.0};
    // is_success = indy.movej(j_pos);
    // if (is_success) {
    //     std::cout << "MoveJ command executed successfully." << std::endl;
    // } else {
    //     std::cout << "MoveJ command failed." << std::endl;
    // }

    // std::array<float, 6> t_pos = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // int base_type = TaskBaseType::RELATIVE_TASK;
    // is_success = indy.movel(t_pos, base_type);
    // if (is_success) {
    //     std::cout << "MoveL command executed successfully." << std::endl;
    // } else {
    //     std::cout << "MoveL command failed." << std::endl;
    // }

    // std::array<float, 6> t_pos1 = {241.66, -51.11, 644.20, 0.0, 180.0, 23.3};
    // std::array<float, 6> t_pos2 = {401.53, -47.74, 647.50, 0.0, 180.0, 23.37};
    // float angle = 720;
    // is_success = indy.movec(t_pos1, t_pos2, angle);
    // if (is_success) {
    //     std::cout << "MoveC command executed successfully." << std::endl;
    // } else {
    //     std::cout << "MoveC command failed." << std::endl;
    // }

    is_success = indy.move_home();
    if (is_success) {
        std::cout << "MoveHome command executed successfully." << std::endl;
    } else {
        std::cout << "MoveHome command failed." << std::endl;
    }

    // // Start teleoperation
    // is_success = indy.start_teleop(TeleMethod::TELE_JOINT_RELATIVE);
    // if (is_success) {
    //     std::cout << "Teleoperation started in relative joint mode." << std::endl;
    // } else {
    //     std::cerr << "Failed to start teleoperation." << std::endl;
    // }

    // std::this_thread::sleep_for(std::chrono::seconds(2));

    // // Move the joints with the specified increments
    // std::vector<float> jpos = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // is_success = indy.movetelej(jpos, 1.0, 1.0, TeleMethod::TELE_JOINT_RELATIVE);
    // if (is_success) {
    //     std::cout << "Joint position moved successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to move joint position." << std::endl;
    // }

    // std::this_thread::sleep_for(std::chrono::seconds(3));

    // // Stop teleoperation
    // is_success = indy.stop_teleop();
    // if (is_success) {
    //     std::cout << "Teleoperation stopped successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to stop teleoperation." << std::endl;
    // }

    std::array<float, 6> tpos = {350.0, -186.5, 522.0, -180.0, 0.0, 180.0};
    std::vector<float> init_jpos = {0.0, 0.0, -90.0, 0.0, -90.0, 0.0};
    std::vector<float> jpos;

    is_success = indy.inverse_kin(tpos, init_jpos, jpos);
    if (is_success) {
        std::cout << "Inverse Kinematics successful. Joint positions: ";
        for (float jp : jpos) {
            std::cout << jp << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Inverse Kinematics failed." << std::endl;
    }

    // is_success = indy.set_direct_teaching(false);
    // if (is_success) {
    //     std::cout << "Direct teaching enabled." << std::endl;
    // } else {
    //     std::cerr << "Failed to enable direct teaching." << std::endl;
    // }

    // is_success = indy.set_simulation_mode(true);
    // if (is_success) {
    //     std::cout << "Simulation mode enabled." << std::endl;
    // } else {
    //     std::cerr << "Failed to enable simulation mode." << std::endl;
    // }

    is_success = indy.recover();
    if (is_success) {
        std::cout << "Recovery successful." << std::endl;
    } else {
        std::cerr << "Recovery failed." << std::endl;
    }

    // is_success = indy.set_manual_recovery(true);
    // if (is_success) {
    //     std::cout << "Manual recovery enabled." << std::endl;
    // } else {
    //     std::cerr << "Failed to enable manual recovery." << std::endl;
    // }

    std::array<float, 6> current_pos = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 6> relative_pos = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 6> calculated_pose;

    is_success = indy.calculate_current_pose_rel(current_pos, relative_pos, TaskBaseType::ABSOLUTE_TASK, calculated_pose);
    if (is_success) {
        std::cout << "Calculated pose: ";
        for (float cp : calculated_pose) {
            std::cout << cp << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Failed to calculate current relative pose." << std::endl;
    }

    // std::string program_name = "ExampleProgram";
    // int program_index = 1;

    // is_success = indy.play_program(program_name, program_index);
    // if (is_success) {
    //     std::cout << "Program started successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to start program." << std::endl;
    // }

    // is_success = indy.pause_program();
    // if (is_success) {
    //     std::cout << "Program paused successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to pause program." << std::endl;
    // }

    // is_success = indy.resume_program();
    // if (is_success) {
    //     std::cout << "Program resumed successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to resume program." << std::endl;
    // }

    // is_success = indy.stop_program();
    // if (is_success) {
    //     std::cout << "Program stopped successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to stop program." << std::endl;
    // }

    // unsigned int speed_ratio = 75;
    // is_success = indy.set_speed_ratio(speed_ratio);
    // if (is_success) {
    //     std::cout << "Speed ratio set to " << speed_ratio << "%" << std::endl;
    // } else {
    //     std::cerr << "Failed to set speed ratio." << std::endl;
    // }


    std::vector<Nrmk::IndyFramework::BoolVariable> set_bool_vars;

    Nrmk::IndyFramework::BoolVariable bool_var1;
    bool_var1.set_addr(1);
    bool_var1.set_value(true);
    set_bool_vars.push_back(bool_var1);

    Nrmk::IndyFramework::BoolVariable bool_var2;
    bool_var2.set_addr(2);
    bool_var2.set_value(false);
    set_bool_vars.push_back(bool_var2);

    is_success = indy.set_bool_variable(set_bool_vars);
    if (is_success) {
        std::cout << "Bool variables set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set bool variables." << std::endl;
    }

    std::vector<Nrmk::IndyFramework::IntVariable> set_int_vars;

    Nrmk::IndyFramework::IntVariable int_var1;
    int_var1.set_addr(1);
    int_var1.set_value(100);
    set_int_vars.push_back(int_var1);

    Nrmk::IndyFramework::IntVariable int_var2;
    int_var2.set_addr(2);
    int_var2.set_value(200);
    set_int_vars.push_back(int_var2);

    is_success = indy.set_int_variable(set_int_vars);
    if (is_success) {
        std::cout << "Int variables set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set int variables." << std::endl;
    }

    std::vector<Nrmk::IndyFramework::FloatVariable> set_float_vars;

    Nrmk::IndyFramework::FloatVariable float_var1;
    float_var1.set_addr(1);
    float_var1.set_value(1.23f);
    set_float_vars.push_back(float_var1);

    Nrmk::IndyFramework::FloatVariable float_var2;
    float_var2.set_addr(2);
    float_var2.set_value(4.56f);
    set_float_vars.push_back(float_var2);

    is_success = indy.set_float_variable(set_float_vars);
    if (is_success) {
        std::cout << "Float variables set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set float variables." << std::endl;
    }


    std::vector<Nrmk::IndyFramework::JPosVariable> set_jpos_vars;

    Nrmk::IndyFramework::JPosVariable jpos_var1;
    jpos_var1.set_addr(1);
    jpos_var1.add_jpos(0.1f);
    jpos_var1.add_jpos(0.2f);
    jpos_var1.add_jpos(0.3f);
    jpos_var1.add_jpos(0.4f);
    jpos_var1.add_jpos(0.5f);
    jpos_var1.add_jpos(0.6f);
    set_jpos_vars.push_back(jpos_var1);

    Nrmk::IndyFramework::JPosVariable jpos_var2;
    jpos_var2.set_addr(2);
    jpos_var2.add_jpos(1.0f);
    jpos_var2.add_jpos(1.1f);
    jpos_var2.add_jpos(1.2f);
    jpos_var2.add_jpos(1.3f);
    jpos_var2.add_jpos(1.4f);
    jpos_var2.add_jpos(1.5f);
    set_jpos_vars.push_back(jpos_var2);

    is_success = indy.set_jpos_variable(set_jpos_vars);
    if (is_success) {
        std::cout << "JPos variables set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set JPos variables." << std::endl;
    }

    std::vector<Nrmk::IndyFramework::TPosVariable> set_tpos_vars;

    Nrmk::IndyFramework::TPosVariable tpos_var1;
    tpos_var1.set_addr(1);
    tpos_var1.add_tpos(10.0f);  // x
    tpos_var1.add_tpos(20.0f);  // y
    tpos_var1.add_tpos(30.0f);  // z
    tpos_var1.add_tpos(40.0f);  // u
    tpos_var1.add_tpos(50.0f);  // v
    tpos_var1.add_tpos(60.0f);  // w
    set_tpos_vars.push_back(tpos_var1);

    Nrmk::IndyFramework::TPosVariable tpos_var2;
    tpos_var2.set_addr(2);
    tpos_var2.add_tpos(11.0f);  // x
    tpos_var2.add_tpos(21.0f);  // y
    tpos_var2.add_tpos(31.0f);  // z
    tpos_var2.add_tpos(41.0f);  // u
    tpos_var2.add_tpos(51.0f);  // v
    tpos_var2.add_tpos(61.0f);  // w
    set_tpos_vars.push_back(tpos_var2);

    is_success = indy.set_tpos_variable(set_tpos_vars);
    if (is_success) {
        std::cout << "TPos variables set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set TPos variables." << std::endl;
    }


    std::vector<Nrmk::IndyFramework::BoolVariable> get_bool_vars;
    is_success = indy.get_bool_variable(get_bool_vars);
    if (is_success) {
        std::cout << "Bool variables retrieved successfully." << std::endl;
        for (const auto& var : get_bool_vars) {
            std::cout << "Address: " << var.addr() << ", Value: " << var.value() << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve bool variables." << std::endl;
    }

    std::vector<Nrmk::IndyFramework::IntVariable> get_int_vars;
    is_success = indy.get_int_variable(get_int_vars);
    if (is_success) {
        std::cout << "Integer variables retrieved successfully." << std::endl;
        for (const auto& var : get_int_vars) {
            std::cout << "Address: " << var.addr() << ", Value: " << var.value() << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve integer variables." << std::endl;
    }

    std::vector<Nrmk::IndyFramework::FloatVariable> get_float_vars;
    is_success = indy.get_float_variable(get_float_vars);
    if (is_success) {
        std::cout << "Float variables retrieved successfully." << std::endl;
        for (const auto& var : get_float_vars) {
            std::cout << "Address: " << var.addr() << ", Value: " << var.value() << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve float variables." << std::endl;
    }

    std::vector<Nrmk::IndyFramework::JPosVariable> get_jpos_vars;
    is_success = indy.get_jpos_variable(get_jpos_vars);
    if (is_success) {
        std::cout << "JPos variables retrieved successfully." << std::endl;
        for (const auto& var : get_jpos_vars) {
            std::cout << "Address: " << var.addr() << ", JPos: ";
            for (const auto& jpos : var.jpos()) {
                std::cout << jpos << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve JPos variables." << std::endl;
    }

    std::vector<Nrmk::IndyFramework::TPosVariable> get_tpos_vars;
    is_success = indy.get_tpos_variable(get_tpos_vars);
    if (is_success) {
        std::cout << "TPos variables retrieved successfully." << std::endl;
        for (const auto& var : get_tpos_vars) {
            std::cout << "Address: " << var.addr() << ", TPos: ";
            for (const auto& tpos : var.tpos()) {
                std::cout << tpos << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Failed to retrieve TPos variables." << std::endl;
    }


    Nrmk::IndyFramework::JointPos set_home_jpos;
    set_home_jpos.add_jpos(0.0);
    set_home_jpos.add_jpos(0.0);
    set_home_jpos.add_jpos(90.0);
    set_home_jpos.add_jpos(0.0);
    set_home_jpos.add_jpos(90.0);
    set_home_jpos.add_jpos(0.0);

    is_success = indy.set_home_pos(set_home_jpos);
    if (is_success) {
        std::cout << "Home position set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set home position." << std::endl;
    }

    std::array<float, 6> get_ref_frame;
    is_success = indy.get_ref_frame(get_ref_frame);
    if (is_success) {
        std::cout << "Reference frame retrieved successfully." << std::endl;
        for (const auto& value : get_ref_frame) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Failed to retrieve reference frame." << std::endl;
    }

    std::array<float, 6> ref_frame = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    is_success = indy.set_ref_frame(ref_frame);
    if (is_success) {
        std::cout << "Reference frame set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set reference frame." << std::endl;
    }


    // std::array<float, 6> fpos_out;
    // std::array<float, 6> fpos0 = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    // std::array<float, 6> fpos1 = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    // std::array<float, 6> fpos2 = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};

    // is_success = indy.set_ref_frame_planar(fpos_out, fpos0, fpos1, fpos2);
    // if (is_success) {
    //     std::cout << "Planar reference frame set successfully. Resulting frame:" << std::endl;
    //     for (const auto& value : fpos_out) {
    //         std::cout << value << " ";
    //     }
    //     std::cout << std::endl;
    // } else {
    //     std::cerr << "Failed to set planar reference frame." << std::endl;
    // }


    // std::array<float, 6> tool_frame = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // is_success = indy.set_tool_frame(tool_frame);
    // if (is_success) {
    //     std::cout << "Tool frame set successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to set tool frame." << std::endl;
    // }


    // Default value
    // Control Compensation Enabled: No
    // Control Compensation Levels: 5 5 5 5 5 5
    // Teaching Compensation Enabled: No
    // Teaching Compensation Levels: 5 2 5 5 5 5

    // Nrmk::IndyFramework::FrictionCompSet set_friction_comp;
    // set_friction_comp.set_control_comp_enable(false);
    // set_friction_comp.set_teaching_comp_enable(false);

    // std::vector<int> control_comp_levels = {5, 5, 5, 5, 5, 5};
    // std::vector<int> dt_comp_levels = {5, 2, 5, 5, 5, 5};

    // // Add control compensation levels
    // for (int level : control_comp_levels) {
    //     set_friction_comp.add_control_comp_levels(level);
    // }

    // // Add teaching compensation levels
    // for (int level : dt_comp_levels) {
    //     set_friction_comp.add_teaching_comp_levels(level);
    // }

    // is_success = indy.set_friction_comp(set_friction_comp);
    // if (is_success) {
    //     std::cout << "Friction compensation set successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to set friction compensation." << std::endl;
    // }

    Nrmk::IndyFramework::FrictionCompSet friction_comp;
    is_success = indy.get_friction_comp(friction_comp);
    if (is_success) {
        std::cout << "Friction compensation data retrieved successfully." << std::endl;

        std::cout << "Control Compensation Enabled: " << (friction_comp.control_comp_enable() ? "Yes" : "No") << std::endl;
        std::cout << "Control Compensation Levels: ";
        for (int i = 0; i < friction_comp.control_comp_levels_size(); ++i) {
            std::cout << friction_comp.control_comp_levels(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Teaching Compensation Enabled: " << (friction_comp.teaching_comp_enable() ? "Yes" : "No") << std::endl;
        std::cout << "Teaching Compensation Levels: ";
        for (int i = 0; i < friction_comp.teaching_comp_levels_size(); ++i) {
            std::cout << friction_comp.teaching_comp_levels(i) << " ";
        }
        std::cout << std::endl;

    } else {
        std::cerr << "Failed to retrieve friction compensation data." << std::endl;
    }

    // Default value
    // Mass: 0
    // Center of Mass: 0 0 0
    // Inertia: 0 0 0 0 0 0

    // Nrmk::IndyFramework::ToolProperties set_tool_properties;
    // set_tool_properties.set_mass(0.0f);
    // std::array<float, 3> center_of_mass = {0.0f, 0.0f, 0.0f};
    // for (float value : center_of_mass) {
    //     set_tool_properties.add_center_of_mass(value);
    // }
    // std::array<float, 6> inertia = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    // for (float value : inertia) {
    //     set_tool_properties.add_inertia(value);
    // }

    // is_success = indy.set_tool_property(set_tool_properties);
    // if (is_success) {
    //     std::cout << "Tool properties set successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to set tool properties." << std::endl;
    // }

    Nrmk::IndyFramework::ToolProperties tool_properties;
    is_success = indy.get_tool_property(tool_properties);
    if (is_success) {
        std::cout << "Tool properties retrieved successfully." << std::endl;
        std::cout << "Mass: " << tool_properties.mass() << std::endl;
        std::cout << "Center of Mass: ";
        for (int i = 0; i < tool_properties.center_of_mass_size(); ++i) {
            std::cout << tool_properties.center_of_mass(i) << " ";
        }
        std::cout << std::endl;
        std::cout << "Inertia: ";
        for (int i = 0; i < tool_properties.inertia_size(); ++i) {
            std::cout << tool_properties.inertia(i) << " ";
        }
        std::cout << std::endl;

    } else {
        std::cerr << "Failed to retrieve tool properties." << std::endl;
    }


    unsigned int set_collision_level = 3;
    is_success = indy.set_coll_sens_level(set_collision_level);
    if (is_success) {
        std::cout << "Collision sensitivity level set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set collision sensitivity level." << std::endl;
    }

    unsigned int collision_level;
    is_success = indy.get_coll_sens_level(collision_level);
    if (is_success) {
        std::cout << "Collision sensitivity level: " << collision_level << std::endl;
    } else {
        std::cerr << "Failed to retrieve collision sensitivity level." << std::endl;
    }

    ////////////////////////

    Nrmk::IndyFramework::CollisionThresholds get_coll_sens_param;
    is_success = indy.get_coll_sens_param(get_coll_sens_param);
    if (is_success) {
        std::cout << "Collision sensitivity parameters retrieved successfully." << std::endl;

        std::cout << "j_torque_bases: ";
        for (int i = 0; i < get_coll_sens_param.j_torque_bases_size(); ++i) {
            std::cout << get_coll_sens_param.j_torque_bases(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "j_torque_tangents: ";
        for (int i = 0; i < get_coll_sens_param.j_torque_tangents_size(); ++i) {
            std::cout << get_coll_sens_param.j_torque_tangents(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_torque_bases: ";
        for (int i = 0; i < get_coll_sens_param.t_torque_bases_size(); ++i) {
            std::cout << get_coll_sens_param.t_torque_bases(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_torque_tangents: ";
        for (int i = 0; i < get_coll_sens_param.t_torque_tangents_size(); ++i) {
            std::cout << get_coll_sens_param.t_torque_tangents(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "error_bases: ";
        for (int i = 0; i < get_coll_sens_param.error_bases_size(); ++i) {
            std::cout << get_coll_sens_param.error_bases(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "error_tangents: ";
        for (int i = 0; i < get_coll_sens_param.error_tangents_size(); ++i) {
            std::cout << get_coll_sens_param.error_tangents(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_constvel_torque_bases: ";
        for (int i = 0; i < get_coll_sens_param.t_constvel_torque_bases_size(); ++i) {
            std::cout << get_coll_sens_param.t_constvel_torque_bases(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_constvel_torque_tangents: ";
        for (int i = 0; i < get_coll_sens_param.t_constvel_torque_tangents_size(); ++i) {
            std::cout << get_coll_sens_param.t_constvel_torque_tangents(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_conveyor_torque_bases: ";
        for (int i = 0; i < get_coll_sens_param.t_conveyor_torque_bases_size(); ++i) {
            std::cout << get_coll_sens_param.t_conveyor_torque_bases(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "t_conveyor_torque_tangents: ";
        for (int i = 0; i < get_coll_sens_param.t_conveyor_torque_tangents_size(); ++i) {
            std::cout << get_coll_sens_param.t_conveyor_torque_tangents(i) << " ";
        }
        std::cout << std::endl;

    } else {
        std::cerr << "Failed to retrieve collision sensitivity parameters." << std::endl;
    }


    // Nrmk::IndyFramework::CollisionThresholds coll_sens_param;
    // std::array<float, 6> j_torque_bases = {9042, 9040, 9019, 9014, 9012, 909};
    // std::array<float, 6> j_torque_tangents = {1.2, 1.2, 0.6, 0, 0.6, 0.4};
    // std::array<float, 6> t_torque_bases = {9037, 9016, 9023.4, 909.6, 908.4, 9012};
    // std::array<float, 6> t_torque_tangents = {5.4, 13.2, 1.8, 1.2, 0.8, 4};
    // std::array<float, 6> error_bases = {90.013, 90.007, 90.007, 90.009, 90.02, 90.024};
    // std::array<float, 6> error_tangents = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // std::array<float, 6> t_constvel_torque_bases = {900.0, 900.0, 900.0, 900.0, 900.0, 900.0};
    // std::array<float, 6> t_constvel_torque_tangents = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // std::array<float, 6> t_conveyor_torque_bases = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // std::array<float, 6> t_conveyor_torque_tangents = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // for (int i = 0; i < 6; ++i) {
    //     coll_sens_param.add_j_torque_bases(j_torque_bases[i]);
    //     coll_sens_param.add_j_torque_tangents(j_torque_tangents[i]);
    //     coll_sens_param.add_t_torque_bases(t_torque_bases[i]);
    //     coll_sens_param.add_t_torque_tangents(t_torque_tangents[i]);
    //     coll_sens_param.add_error_bases(error_bases[i]);
    //     coll_sens_param.add_error_tangents(error_tangents[i]);
    //     coll_sens_param.add_t_constvel_torque_bases(t_constvel_torque_bases[i]);
    //     coll_sens_param.add_t_constvel_torque_tangents(t_constvel_torque_tangents[i]);
    //     coll_sens_param.add_t_conveyor_torque_bases(t_conveyor_torque_bases[i]);
    //     coll_sens_param.add_t_conveyor_tangents(t_conveyor_torque_tangents[i]);
    // }

    // is_success = indy.set_coll_sens_param(coll_sens_param);
    // if (is_success) {
    //     std::cout << "Collision sensitivity parameters set successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to set collision sensitivity parameters." << std::endl;
    // }


    Nrmk::IndyFramework::CollisionPolicy get_coll_policy;
    is_success = indy.get_coll_policy(get_coll_policy);
    if (is_success) {
        std::cout << "Collision policy retrieved successfully." << std::endl;
        std::cout << "Policy: " << get_coll_policy.policy() << std::endl;
        std::cout << "Sleep Time: " << get_coll_policy.sleep_time() << " seconds" << std::endl;
        std::cout << "Gravity Time: " << get_coll_policy.gravity_time() << " seconds" << std::endl;
    } else {
        std::cerr << "Failed to retrieve collision policy." << std::endl;
    }

    // Nrmk::IndyFramework::CollisionPolicyType policy = Nrmk::IndyFramework::CollisionPolicyType::COLL_RESUME_AFTER_SLEEP;
    // float sleep_time = 3.0f;
    // float gravity_time = 0.1f;
    // is_success = indy.set_coll_policy(policy, sleep_time, gravity_time);
    // if (is_success) {
    //     std::cout << "Collision policy set successfully." << std::endl;
    //     switch (policy) {
    //         case Nrmk::IndyFramework::CollisionPolicyType::COLL_NO_DETECT:
    //             std::cout << "Policy: No Collision Detection" << std::endl;
    //             break;
    //         case Nrmk::IndyFramework::CollisionPolicyType::COLL_PAUSE:
    //             std::cout << "Policy: Collision Pause" << std::endl;
    //             break;
    //         case Nrmk::IndyFramework::CollisionPolicyType::COLL_RESUME_AFTER_SLEEP:
    //             std::cout << "Policy: Resume After Sleep" << std::endl;
    //             break;
    //         case Nrmk::IndyFramework::CollisionPolicyType::COLL_STOP:
    //             std::cout << "Policy: Collision Stop" << std::endl;
    //             break;
    //         default:
    //             std::cerr << "Unknown policy type." << std::endl;
    //             break;
    //     }
    //     std::cout << "Sleep Time: " << sleep_time << " seconds" << std::endl;
    //     std::cout << "Gravity Time: " << gravity_time << " seconds" << std::endl;
    // } else {
    //     std::cerr << "Failed to set collision policy." << std::endl;
    // }


    Nrmk::IndyFramework::SafetyLimits get_safety_limits;
    is_success = indy.get_safety_limits(get_safety_limits);
    if (is_success) {
        std::cout << "Safety limits retrieved successfully." << std::endl;
        std::cout << "Power Limit: " << get_safety_limits.power_limit() << std::endl;
        std::cout << "Power Limit Ratio: " << get_safety_limits.power_limit_ratio() << std::endl;
        std::cout << "TCP Force Limit: " << get_safety_limits.tcp_force_limit() << std::endl;
        std::cout << "TCP Force Limit Ratio: " << get_safety_limits.tcp_force_limit_ratio() << std::endl;
        std::cout << "TCP Speed Limit: " << get_safety_limits.tcp_speed_limit() << std::endl;
        std::cout << "TCP Speed Limit Ratio: " << get_safety_limits.tcp_speed_limit_ratio() << std::endl;

        std::cout << "Joint Limits: ";
        for (int i = 0; i < get_safety_limits.joint_limits_size(); ++i) {
            std::cout << get_safety_limits.joint_limits(i) << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Failed to retrieve safety limits." << std::endl;
    }

    // Nrmk::IndyFramework::SafetyLimits safety_limits;
    // safety_limits.set_power_limit(1500);
    // safety_limits.set_power_limit_ratio(100);
    // safety_limits.set_tcp_force_limit(800);
    // safety_limits.set_tcp_force_limit_ratio(100);
    // safety_limits.set_tcp_speed_limit(4);
    // safety_limits.set_tcp_speed_limit_ratio(100);
    // std::vector<float> joint_limits = {175.0, 175.0, 175.0, 175.0, 175.0, 175.0};
    // for (const auto& limit : joint_limits) {
    //     safety_limits.add_joint_limits(limit);
    // }

    // is_success = indy.set_safety_limits(safety_limits);
    // if (is_success) {
    //     std::cout << "Safety limits set successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to set safety limits." << std::endl;
    // }

    // Nrmk::IndyFramework::SDKLicenseInfo request;
    // request.set_license_key("YOUR_LICENSE_KEY");
    // request.set_expire_date("2024-12-31");

    // Nrmk::IndyFramework::SDKLicenseResp response;
    // is_success = indy.activate_sdk(request, response);
    // if (is_success) {
    //     std::cout << "SDK Activated: " << (response.activated() ? "Yes" : "No") << std::endl;
    //     std::cout << "Response Code: " << response.response().code() << ", Message: " << response.response().msg() << std::endl;
    // } else {
    //     std::cerr << "Failed to activate SDK." << std::endl;
    // }


    int get_mode;
    is_success = indy.get_custom_control_mode(get_mode);
    if (is_success) {
        std::cout << "Custom control mode: " << get_mode << std::endl;
    } else {
        std::cerr << "Failed to get custom control mode." << std::endl;
    }

    int mode = 0;
    is_success = indy.set_custom_control_mode(mode);
    if (is_success) {
        std::cout << "Custom control mode set successfully." << std::endl;
    } else {
        std::cerr << "Failed to set custom control mode." << std::endl;
    }


    Nrmk::IndyFramework::CustomGainSet get_custom_gain;
    is_success = indy.get_custom_control_gain(get_custom_gain);
    if (is_success) {
        std::cout << "Custom control gains retrieved successfully." << std::endl;

        std::cout << "Gain0: ";
        for (int i = 0; i < get_custom_gain.gain0_size(); ++i) {
            std::cout << get_custom_gain.gain0(i) << " ";
        }
        std::cout << std::endl;

        std::cout << "Gain1: ";
        for (int i = 0; i < get_custom_gain.gain1_size(); ++i) {
            std::cout << get_custom_gain.gain1(i) << " ";
        }
        std::cout << std::endl;

        //Other gain...
    } else {
        std::cerr << "Failed to retrieve custom control gains." << std::endl;
    }

    // Nrmk::IndyFramework::CustomGainSet custom_gain_set;

    // custom_gain_set.add_gain0(0.0);
    // custom_gain_set.add_gain0(0.0);
    // custom_gain_set.add_gain0(0.0);
    // custom_gain_set.add_gain0(0.0);
    // custom_gain_set.add_gain0(0.0);
    // custom_gain_set.add_gain0(0.0);

    // custom_gain_set.add_gain1(0.0);
    // custom_gain_set.add_gain1(0.0);
    // custom_gain_set.add_gain1(0.0);
    // custom_gain_set.add_gain1(0.0);
    // custom_gain_set.add_gain1(0.0);
    // custom_gain_set.add_gain1(0.0);

    // is_success = indy.set_custom_control_gain(custom_gain_set);
    // if (is_success) {
    //     std::cout << "Custom control gains set successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to set custom control gains." << std::endl;
    // }

    // Start realtime data logging
    is_success = indy.start_log();
    if (is_success) {
        std::cout << "Started realtime data logging." << std::endl;
    } else {
        std::cerr << "Failed to start data logging." << std::endl;
    }

    // End realtime data logging and save the data
    is_success = indy.end_log();
    if (is_success) {
        std::cout << "Finished and saved realtime data logging." << std::endl;
    } else {
        std::cerr << "Failed to end data logging." << std::endl;
    }

    return 0;
}
