#include "indydcp3.h"
#include "indydcp3_rpc_utils.h"
#include <vector>

bool IndyDCP3::apply_teleop_constraint_batch(const Nrmk::IndyFramework::ApplyTeleopConstraintBatchRequest& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::ApplyTeleopConstraintBatch,
                          request, response, "ApplyTeleopConstraintBatch");
}

bool IndyDCP3::get_self_collision_config(Nrmk::IndyFramework::SelfCollisionConstraintConfig& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::GetSelfCollisionConfig,
                          request, response, "GetSelfCollisionConfig");
}

bool IndyDCP3::get_teleop_constraint_runtime_status(Nrmk::IndyFramework::TeleopConstraintRuntimeStatus& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::GetTeleopConstraintRuntimeStatus,
                          request, response, "GetTeleopConstraintRuntimeStatus");
}

bool IndyDCP3::get_teleop_tuning_params(Nrmk::IndyFramework::TeleopTuningParams& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::GetTeleopTuningParams,
                          request, response, "GetTeleopTuningParams");
}

bool IndyDCP3::get_tool_collision_sphere_config(Nrmk::IndyFramework::Message& response) {
    Nrmk::IndyFramework::Empty request;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::GetToolCollisionSphereConfig,
                          request, response, "GetToolCollisionSphereConfig");
}

bool IndyDCP3::set_dynamic_obstacle_constraint_config(const Nrmk::IndyFramework::ObstacleConstraintConfig& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::SetDynamicObstacleConstraintConfig,
                          request, response, "SetDynamicObstacleConstraintConfig");
}

bool IndyDCP3::set_joint_constraint_config(const Nrmk::IndyFramework::JointConstraintConfig& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::SetJointConstraintConfig,
                          request, response, "SetJointConstraintConfig");
}

bool IndyDCP3::set_orientation_deviation_config(const Nrmk::IndyFramework::OrientationDeviationConfig& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::SetOrientationDeviationConfig,
                          request, response, "SetOrientationDeviationConfig");
}

bool IndyDCP3::set_plane_constraint_config(const Nrmk::IndyFramework::PlaneConstraintConfig& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::SetPlaneConstraintConfig,
                          request, response, "SetPlaneConstraintConfig");
}

bool IndyDCP3::set_self_collision_config(const Nrmk::IndyFramework::SelfCollisionConstraintConfig& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::SetSelfCollisionConfig,
                          request, response, "SetSelfCollisionConfig");
}

bool IndyDCP3::set_self_collision_pairs(const Nrmk::IndyFramework::SelfCollisionPairs& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::SetSelfCollisionPairs,
                          request, response, "SetSelfCollisionPairs");
}

bool IndyDCP3::set_static_obstacle_constraint_config(const Nrmk::IndyFramework::ObstacleConstraintConfig& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::SetStaticObstacleConstraintConfig,
                          request, response, "SetStaticObstacleConstraintConfig");
}

bool IndyDCP3::set_teleop_tuning_params(const Nrmk::IndyFramework::TeleopTuningParams& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::SetTeleopTuningParams,
                          request, response, "SetTeleopTuningParams");
}

bool IndyDCP3::set_tool_collision_sphere_config(const Nrmk::IndyFramework::Message& request) {
    Nrmk::IndyFramework::Response response;
    return call_unary_rpc(teleop_stub.get(),
                          &Nrmk::IndyFramework::TeleOp::Stub::SetToolCollisionSphereConfig,
                          request, response, "SetToolCollisionSphereConfig");
}

bool IndyDCP3::set_obstacle_info(const Nrmk::IndyFramework::ObstacleInfo& obstacle_info) {
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    grpc::Status status = teleop_stub->SetObstacleInfo(&context, obstacle_info, &response);
    if (!status.ok()) {
        std::cerr << "SetObstacleInfo RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_obstacle_info(uint32_t idx, Nrmk::IndyFramework::ObstacleInfo& obstacle_info) {
    Nrmk::IndyFramework::ObstacleIndex request;
    grpc::ClientContext context;

    request.set_idx(idx);
    grpc::Status status = teleop_stub->GetObstacleInfo(&context, request, &obstacle_info);
    if (!status.ok()) {
        std::cerr << "GetObstacleInfo RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_collision_spheres(Nrmk::IndyFramework::CollisionSpheresInfo& collision_spheres_info) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = teleop_stub->GetCollisionSpheres(&context, request, &collision_spheres_info);
    if (!status.ok()) {
        std::cerr << "GetCollisionSpheres RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_joint_constraint_config(Nrmk::IndyFramework::JointConstraintConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = teleop_stub->GetJointConstraintConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetJointConstraintConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_self_collision_pairs(Nrmk::IndyFramework::SelfCollisionPairs& pairs) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = teleop_stub->GetSelfCollisionPairs(&context, request, &pairs);
    if (!status.ok()) {
        std::cerr << "GetSelfCollisionPairs RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plane_constraint_config(Nrmk::IndyFramework::PlaneConstraintConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = teleop_stub->GetPlaneConstraintConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetPlaneConstraintConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_static_obstacle_constraint_config(Nrmk::IndyFramework::ObstacleConstraintConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = teleop_stub->GetStaticObstacleConstraintConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetStaticObstacleConstraintConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_dynamic_obstacle_constraint_config(Nrmk::IndyFramework::ObstacleConstraintConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = teleop_stub->GetDynamicObstacleConstraintConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetDynamicObstacleConstraintConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_orientation_deviation_config(Nrmk::IndyFramework::OrientationDeviationConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = teleop_stub->GetOrientationDeviationConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetOrientationDeviationConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_desired_position(Nrmk::IndyFramework::DesiredPosition& position) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = teleop_stub->GetDesiredPosition(&context, request, &position);
    if (!status.ok()) {
        std::cerr << "GetDesiredPosition RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_current_position(Nrmk::IndyFramework::CurrentPosition& position) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = teleop_stub->GetCurrentPosition(&context, request, &position);
    if (!status.ok()) {
        std::cerr << "GetCurrentPosition RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}
