#include <array>
#include <iostream>
#include <string>

#include "../neuromeka_cpp/indydcp3.h"

namespace {

void print_values(const google::protobuf::RepeatedField<float>& values) {
    std::cout << "[";
    for (int i = 0; i < values.size(); ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << values.Get(i);
    }
    std::cout << "]";
}

void print_values(const std::array<float, 6>& values) {
    std::cout << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << values[i];
    }
    std::cout << "]";
}

bool test_set_ref_frame(IndyDCP3& indy,
                        const std::array<float, 6>& fpos,
                        int arm_index,
                        int link_index) {
    std::cout << "indy.set_ref_frame(";
    print_values(fpos);
    std::cout << ", arm_index=" << arm_index
              << ", link_index=" << link_index << ")" << std::endl;

    const bool rpc_ok = indy.set_ref_frame(fpos, arm_index, link_index);
    std::cout << "{'rpc_ok': " << std::boolalpha << rpc_ok << "}\n" << std::endl;
    return rpc_ok;
}

void print_tool_frame_list(const Nrmk::IndyFramework::ToolFrameList& list) {
    std::cout << "Current tool frames:" << std::endl;
    std::cout << "  default_name: " << list.default_name() << std::endl;
    for (int i = 0; i < list.tool_frames_size(); ++i) {
        const auto& tool_frame = list.tool_frames(i);
        std::cout << "  [" << i << "] name: " << tool_frame.name() << ", tpos: ";
        print_values(tool_frame.tpos());
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: " << argv[0] << " <robot_ip> [robot_index]" << std::endl;
        std::cerr << "Example: " << argv[0] << " 192.168.1.24 0" << std::endl;
        return 2;
    }

    const std::string robot_ip = argv[1];
    int robot_index = 0;
    if (argc == 3) {
        try {
            robot_index = std::stoi(argv[2]);
        } catch (const std::exception&) {
            std::cerr << "robot_index must be 0 or 1." << std::endl;
            return 2;
        }
    }

    if (robot_index != 0 && robot_index != 1) {
        std::cerr << "robot_index must be 0 or 1." << std::endl;
        return 2;
    }

    IndyDCP3 indy(robot_ip, robot_index);
    const std::array<float, 6> zero_frame = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    std::cout << "indy.get_ref_frame()" << std::endl;
    std::array<float, 6> current_ref_frame = {};
    const bool get_ref_frame_ok = indy.get_ref_frame(current_ref_frame);
    std::cout << "{'rpc_ok': " << std::boolalpha << get_ref_frame_ok << ", 'fpos': ";
    print_values(current_ref_frame);
    std::cout << "}\n" << std::endl;

    bool all_rpc_calls_ok = get_ref_frame_ok;
    all_rpc_calls_ok = test_set_ref_frame(indy, zero_frame, 0, 0) && all_rpc_calls_ok;
    all_rpc_calls_ok = test_set_ref_frame(indy, zero_frame, 1, 0) && all_rpc_calls_ok;
    all_rpc_calls_ok = test_set_ref_frame(indy, zero_frame, 0, 1) && all_rpc_calls_ok;

    std::cout << "indy.get_tool_frame_list()" << std::endl;
    Nrmk::IndyFramework::ToolFrameList tool_frame_list;
    const bool get_tool_frame_list_ok = indy.get_tool_frame_list(tool_frame_list);
    std::cout << "{'rpc_ok': " << std::boolalpha << get_tool_frame_list_ok << "}" << std::endl;
    if (get_tool_frame_list_ok) {
        print_tool_frame_list(tool_frame_list);

        std::cout << "indy.set_tool_frame_list(current_tool_frame_list)" << std::endl;
        const bool set_tool_frame_list_ok = indy.set_tool_frame_list(tool_frame_list);
        std::cout << "{'rpc_ok': " << std::boolalpha << set_tool_frame_list_ok << "}" << std::endl;
        all_rpc_calls_ok = set_tool_frame_list_ok && all_rpc_calls_ok;
    } else {
        all_rpc_calls_ok = false;
    }

    return all_rpc_calls_ok ? 0 : 1;
}
