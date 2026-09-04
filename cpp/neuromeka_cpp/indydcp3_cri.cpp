#include "indydcp3.h"
#include "indydcp3_rpc_utils.h"
#include <vector>

bool IndyDCP3::generate_sfd_token(const Nrmk::IndyFramework::SFDAccount& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::GenerateSFDToken,
                          request, response, "GenerateSFDToken");
}

bool IndyDCP3::get_cri_record_mode(Nrmk::IndyFramework::CRIRecordModeState& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::GetCRIRecordMode,
                          request, response, "GetCRIRecordMode");
}

bool IndyDCP3::get_sfd_login_info(Nrmk::IndyFramework::SFDAccount& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::GetSFDLoginInfo,
                          request, response, "GetSFDLoginInfo");
}

bool IndyDCP3::get_sfd_target(Nrmk::IndyFramework::SFDTarget& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::GetSFDTarget,
                          request, response, "GetSFDTarget");
}

bool IndyDCP3::is_sfd_target_valid(Nrmk::IndyFramework::State& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::IsSFDTargetValid,
                          request, response, "IsSFDTargetValid");
}

bool IndyDCP3::load_sfd_auto_set(Nrmk::IndyFramework::SFDAutoSet& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::LoadSFDAutoSet,
                          request, response, "LoadSFDAutoSet");
}

bool IndyDCP3::load_sfd_login_info(Nrmk::IndyFramework::SFDAccount& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::LoadSFDLoginInfo,
                          request, response, "LoadSFDLoginInfo");
}

bool IndyDCP3::logout_sfd() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::LogoutSFD,
                          request, response, "LogoutSFD");
}

bool IndyDCP3::release_sfd_target(Nrmk::IndyFramework::State& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::ReleaseSFDTarget,
                          request, response, "ReleaseSFDTarget");
}

bool IndyDCP3::save_sfd_auto_set(const Nrmk::IndyFramework::SFDAutoSet& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::SaveSFDAutoSet,
                          request, response, "SaveSFDAutoSet");
}

bool IndyDCP3::save_sfd_login_info(const Nrmk::IndyFramework::SFDAccount& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::SaveSFDLoginInfo,
                          request, response, "SaveSFDLoginInfo");
}

bool IndyDCP3::start_cri_playback() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::StartCRIPlayback,
                          request, response, "StartCRIPlayback");
}

bool IndyDCP3::start_cri_record() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::StartCRIRecord,
                          request, response, "StartCRIRecord");
}

bool IndyDCP3::stop_cri_playback() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::StopCRIPlayback,
                          request, response, "StopCRIPlayback");
}

bool IndyDCP3::stop_cri_record() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(cri_stub.get(),
                          &Nrmk::IndyFramework::CRI::Stub::StopCRIRecord,
                          request, response, "StopCRIRecord");
}

bool IndyDCP3::activate_cri(const bool on) {
    Nrmk::IndyFramework::State request;
    request.set_enable(on);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->ActiveCRIVel(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ActiveCRIVel RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::is_cri_active(bool& is_active) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::State response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->IsSFDLogin(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "IsSFDLogin RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    is_active = response.enable();
    return true;
}

bool IndyDCP3::login_cri_server(const Nrmk::IndyFramework::SFDAccount& account) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->LoginSFD(&context, account, &response);
    if (!status.ok()) {
        std::cerr << "LoginSFD RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::is_cri_login(bool& is_logged_in) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::State response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->IsSFDLogin(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "IsSFDLogin RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    is_logged_in = response.enable();
    return true;
}

bool IndyDCP3::set_cri_target(const Nrmk::IndyFramework::SFDTarget& target) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->SelectSFDTarget(&context, target, &response);
    if (!status.ok()) {
        std::cerr << "SelectSFDTarget RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::set_cri_option(const Nrmk::IndyFramework::State& option) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->ActiveCRIVel(&context, option, &response);
    if (!status.ok()) {
        std::cerr << "ActiveCRIVel RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_cri_proj_list(Nrmk::IndyFramework::SFDProjectList& project_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->GetSFDProjList(&context, request, &project_list);
    if (!status.ok()) {
        std::cerr << "GetSFDProjList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_cri(Nrmk::IndyFramework::CriData& cri_data) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = cri_stub->GetCRI(&context, request, &cri_data);
    if (!status.ok()) {
        std::cerr << "Get CRI RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}
