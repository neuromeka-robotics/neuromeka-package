#include <iostream>
#include <cstdlib>
#include <string>

#ifdef _WIN32
#include <windows.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#endif

int main() {

    #ifdef _WIN32
        // For Windows
        std::string proto_src_dir = "../../../proto";
        std::string proto_out_dir = "../../../proto/cpp_generated";
        std::string grpc_plugin_path = "D:\\WORKS\\Neuromeka\\Neuromeka-packages\\install14\\bin\\grpc_cpp_plugin.exe";
        std::string protoc_executable = "D:\\WORKS\\Neuromeka\\Neuromeka-packages\\install14\\bin\\protoc.exe";
    #else
        // For Linux
        std::string proto_src_dir = "../../proto";
        std::string proto_out_dir = "../../proto/cpp_generated";
        std::string grpc_plugin_path = "/home/user/.local/bin/grpc_cpp_plugin";
        std::string protoc_executable = "protoc";
    #endif
    
    #ifdef _WIN32
        WIN32_FIND_DATA findFileData;
        HANDLE hFind = FindFirstFile((proto_src_dir + "\\*").c_str(), &findFileData);

        if (hFind == INVALID_HANDLE_VALUE) {
            std::cerr << "Error opening directory: " << proto_src_dir << std::endl;
            return 1;
        }

        do {
            const std::string fileOrDir = findFileData.cFileName;
            if (fileOrDir != "." && fileOrDir != ".." && !(findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
                if (fileOrDir.substr(fileOrDir.find_last_of(".") + 1) == "proto") {
                    std::string proto_file = proto_src_dir + "\\" + fileOrDir;

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
                        FindClose(hFind);
                        return result;
                    }
                }
            }
        } while (FindNextFile(hFind, &findFileData) != 0);

        FindClose(hFind);

    #else
        DIR* dir = opendir(proto_src_dir.c_str());
        if (dir == nullptr) {
            std::cerr << "Error opening directory: " << proto_src_dir << std::endl;
            return 1;
        }

        struct dirent* entity;
        while ((entity = readdir(dir)) != nullptr) {
            std::string name = entity->d_name;
            if (name != "." && name != "..") {
                std::string proto_file = proto_src_dir + "/" + name;
                
                struct stat s;
                if (stat(proto_file.c_str(), &s) == 0 && S_ISREG(s.st_mode)) {
                    if (name.substr(name.find_last_of(".") + 1) == "proto") {

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
                            closedir(dir);
                            return result;
                        }
                    }
                }
            }
        }
        closedir(dir);
    #endif

    std::cout << "Protobuf files generated successfully." << std::endl;
    return 0;
}



// #include <iostream>
// #include <filesystem>
// #include <cstdlib>

// int main() {

//     #ifdef _WIN32
//         // For Windows
//         std::string proto_src_dir = "../../../proto";
//         std::string proto_out_dir = "../../../proto/cpp_generated";
//         // std::string proto_src_dir = "../../proto";
//         // std::string proto_out_dir = "../../proto/cpp_generated";
//         // std::string grpc_plugin_path = "C:\\Workspace\\install\\bin\\grpc_cpp_plugin.exe";
//         // std::string protoc_executable = "C:\\Workspace\\install\\bin\\protoc.exe";
//         // std::string grpc_plugin_path = "C:\\Workspace\\vcpkg\\installed\\x64-windows\\tools\\grpc\\grpc_cpp_plugin.exe";
//         // std::string protoc_executable = "C:\\Workspace\\vcpkg\\installed\\x64-windows\\tools\\protobuf\\protoc.exe";
//         std::string grpc_plugin_path = "D:\\WORKS\\Neuromeka\\Neuromeka-packages\\install\\bin\\grpc_cpp_plugin.exe";
//         std::string protoc_executable = "D:\\WORKS\\Neuromeka\\Neuromeka-packages\\install\\bin\\protoc.exe";
//     #else
//         // For Linux
//         std::string proto_src_dir = "../../proto";
//         std::string proto_out_dir = "../../proto/cpp_generated";
//         std::string grpc_plugin_path = "/home/user/.local/bin/grpc_cpp_plugin";
//         std::string protoc_executable = "protoc";
//     #endif
    
//     for (const auto& entry : std::filesystem::directory_iterator(proto_src_dir)) {

//         if (entry.path().extension() == ".proto") {
//             std::string proto_file = entry.path().string();

//             // grpc and proto
//             std::string command = protoc_executable + " --grpc_out=" + proto_out_dir +
//                                   " --cpp_out=" + proto_out_dir +
//                                   " --proto_path=" + proto_src_dir +
//                                   " " + proto_file +
//                                   " --plugin=protoc-gen-grpc=" + grpc_plugin_path;
//             std::cout << "Running command: " << command << std::endl;
//             int result = std::system(command.c_str());
//             if (result != 0) {
//                 std::cerr << "Error running protoc command on " << proto_file << std::endl;
//                 return result;
//             }
//         }
//     }

//     std::cout << "Protobuf files generated successfully." << std::endl;
//     return 0;
// }
