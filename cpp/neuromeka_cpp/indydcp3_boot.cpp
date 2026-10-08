#include "indydcp3.h"
#include <vector>

bool IndyDCP3::get_boot_status(Nrmk::IndyFramework::BootStatus& status) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status rpc = boot_stub->GetBootStatus(&context, request, &status);
    if (!rpc.ok()) {
        std::cerr << "GetBootStatus RPC failed: " << rpc.error_message() << std::endl;
        return false;
    }
    return true;
}
