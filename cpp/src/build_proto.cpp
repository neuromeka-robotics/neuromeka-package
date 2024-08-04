#include <iostream>
#include <filesystem>
#include <cstdlib>

int main() {

    #ifdef _WIN32
        // For Windows
        std::string proto_src_dir = "../../../proto";
        std::string proto_out_dir = "../../../proto/cpp_generated";
        std::string grpc_plugin_path = "E:\\Example\\install\\bin\\grpc_cpp_plugin.exe";
        std::string protoc_executable = "E:\\Example\\install\\bin\\protoc.exe";
    #else
        // For Linux
        std::string proto_src_dir = "../../proto";
        std::string proto_out_dir = "../../proto/cpp_generated";
        std::string grpc_plugin_path = "/home/user/.local/bin/grpc_cpp_plugin";
        std::string protoc_executable = "protoc";
    #endif
    
    for (const auto& entry : std::filesystem::directory_iterator(proto_src_dir)) {

        if (entry.path().extension() == ".proto") {
            std::string proto_file = entry.path().string();

            // grpc and proto
            std::string command = protoc_executable + " --grpc_out=" + proto_out_dir +
                                  " --cpp_out=" + proto_out_dir +
                                  " --proto_path=" + proto_src_dir +
                                  " " + proto_file +
                                  " --plugin=protoc-gen-grpc=" + grpc_plugin_path;
            std::cout << "Running command: " << command << std::endl;
            int result = std::system(command.c_str());
            if (result != 0) {
                std::cerr << "Error running protoc command on " << proto_file << std::endl;
                return result;
            }
        }
    }

    std::cout << "Protobuf files generated successfully." << std::endl;
    return 0;
}
