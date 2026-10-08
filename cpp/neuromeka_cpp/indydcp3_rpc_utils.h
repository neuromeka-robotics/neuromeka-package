#ifndef INDYDCP3_RPC_UTILS_H
#define INDYDCP3_RPC_UTILS_H

#include <iostream>

#include <grpcpp/grpcpp.h>

namespace neuromeka_detail {

template <typename Stub, typename Request, typename Response>
bool call_unary_rpc(
        Stub* stub,
        grpc::Status (Stub::*rpc)(grpc::ClientContext*, const Request&, Response*),
        const Request& request,
        Response& response,
        const char* rpc_name) {
    grpc::ClientContext context;
    grpc::Status status = (stub->*rpc)(&context, request, &response);
    if (!status.ok()) {
        std::cerr << rpc_name << " RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

}  // namespace neuromeka_detail

using neuromeka_detail::call_unary_rpc;

#endif  // INDYDCP3_RPC_UTILS_H
