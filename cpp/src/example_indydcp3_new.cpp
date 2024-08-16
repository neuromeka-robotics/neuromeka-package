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


    // // Example usage of wait_time function
    // is_success = indy.wait_time(3.0);
    // if (is_success) {
    //     std::cout << "Successfully waited for 3 seconds." << std::endl;
    // } else {
    //     std::cerr << "Failed to wait." << std::endl;
    // }

    is_success = indy.wait_for_operation_state(OpState::OP_MOVING);
    if (is_success) {
        std::cout << "The robot has reached the target position." << std::endl;
    } else {
        std::cerr << "Failed to wait for the robot to reach the target position." << std::endl;
    }

    is_success = indy.wait_for_motion_state("is_target_reached");
    if (is_success) {
        std::cout << "The robot has reached the target position." << std::endl;
    } else {
        std::cerr << "Failed to wait for the robot to reach the target position." << std::endl;
    }

/*
    // Move joint waypoint
    indy.add_joint_waypoint({0, 0, 0, 0, 0, 0});
    indy.add_joint_waypoint({-44, 25, -63, 48, -7, -105});
    indy.add_joint_waypoint({0, 0, 90, 0, 90, 0});
    indy.add_joint_waypoint({-145, 31, -33, 117, -7, -133});
    indy.add_joint_waypoint({-90, -15, -90, 0, -75, 0});

    std::vector<std::vector<float>> waypoints;
    if (indy.get_joint_waypoint(waypoints)) {
        std::cout << "Successfully retrieved joint waypoints:" << std::endl;
        for (const auto& wp : waypoints) {
            for (float joint : wp) {
                std::cout << joint << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "No joint waypoints available." << std::endl;
    }

    indy.move_joint_waypoint();

    float move_time=3.0;
    indy.move_joint_waypoint(move_time);

    indy.clear_joint_waypoint();


    // Move task waypoint
    indy.add_task_waypoint({-186.54f, -454.45f, 415.61f, 179.99f, -0.06f, 89.98f});
    indy.add_task_waypoint({-334.67f, -493.07f, 259.00f, 179.96f, -0.12f, 89.97f});
    indy.add_task_waypoint({224.79f, -490.20f, 508.08f, 179.96f, -0.14f, 89.97f});
    indy.add_task_waypoint({-129.84f, -416.84f, 507.38f, 179.95f, -0.16f, 89.96f});
    indy.add_task_waypoint({-186.54f, -454.45f, 415.61f, 179.99f, -0.06f, 89.98f});

    std::vector<std::array<float, 6>> t_waypoints;
    if (indy.get_task_waypoint(t_waypoints)) {
        std::cout << "Successfully retrieved task waypoints:" << std::endl;
        for (const auto& wp : t_waypoints) {
            for (float task : wp) {
                std::cout << task << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "No task waypoints available." << std::endl;
    }

    indy.move_task_waypoint();

    move_time = 1.0f;
    indy.move_task_waypoint(move_time);

    indy.clear_task_waypoint();
*/

    return 0;
}
