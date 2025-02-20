// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: config.proto

#include "config.pb.h"

#include <algorithm>
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/extension_set.h"
#include "google/protobuf/wire_format_lite.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/reflection_ops.h"
#include "google/protobuf/wire_format.h"
#include "google/protobuf/generated_message_tctable_impl.h"
// @@protoc_insertion_point(includes)

// Must be included last.
#include "google/protobuf/port_def.inc"
PROTOBUF_PRAGMA_INIT_SEG
namespace _pb = ::google::protobuf;
namespace _pbi = ::google::protobuf::internal;
namespace _fl = ::google::protobuf::internal::field_layout;
namespace Nrmk {
namespace IndyFramework {
}  // namespace IndyFramework
}  // namespace Nrmk
static constexpr const ::_pb::EnumDescriptor**
    file_level_enum_descriptors_config_2eproto = nullptr;
static constexpr const ::_pb::ServiceDescriptor**
    file_level_service_descriptors_config_2eproto = nullptr;
const ::uint32_t TableStruct_config_2eproto::offsets[1] = {};
static constexpr ::_pbi::MigrationSchema* schemas = nullptr;
static constexpr ::_pb::Message* const* file_default_instances = nullptr;
const char descriptor_table_protodef_config_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
    "\n\014config.proto\022\022Nrmk.IndyFramework\032\021conf"
    "ig_msgs.proto\032\021common_msgs.proto2\232#\n\006Con"
    "fig\022E\n\013GetRefFrame\022\031.Nrmk.IndyFramework."
    "Empty\032\031.Nrmk.IndyFramework.Frame\"\000\022H\n\013Se"
    "tRefFrame\022\031.Nrmk.IndyFramework.Frame\032\034.N"
    "rmk.IndyFramework.Response\"\000\022W\n\021SetRefFr"
    "amePlanar\022\037.Nrmk.IndyFramework.PlanarFra"
    "me\032\037.Nrmk.IndyFramework.FrameResult\"\000\022I\n"
    "\014SetToolFrame\022\031.Nrmk.IndyFramework.Frame"
    "\032\034.Nrmk.IndyFramework.Response\"\000\022J\n\rSetS"
    "peedRatio\022\031.Nrmk.IndyFramework.Ratio\032\034.N"
    "rmk.IndyFramework.Response\"\000\022S\n\017SetDICon"
    "figList\022 .Nrmk.IndyFramework.DIConfigLis"
    "t\032\034.Nrmk.IndyFramework.Response\"\000\022P\n\017Get"
    "DIConfigList\022\031.Nrmk.IndyFramework.Empty\032"
    " .Nrmk.IndyFramework.DIConfigList\"\000\022S\n\017S"
    "etDOConfigList\022 .Nrmk.IndyFramework.DOCo"
    "nfigList\032\034.Nrmk.IndyFramework.Response\"\000"
    "\022P\n\017GetDOConfigList\022\031.Nrmk.IndyFramework"
    ".Empty\032 .Nrmk.IndyFramework.DOConfigList"
    "\"\000\022O\n\017SetHomePosition\022\034.Nrmk.IndyFramewo"
    "rk.JointPos\032\034.Nrmk.IndyFramework.Respons"
    "e\"\000\022L\n\017GetHomePosition\022\031.Nrmk.IndyFramew"
    "ork.Empty\032\034.Nrmk.IndyFramework.JointPos\""
    "\000\022L\n\017GetPackPosition\022\031.Nrmk.IndyFramewor"
    "k.Empty\032\034.Nrmk.IndyFramework.JointPos\"\000\022"
    "Y\n\017SetAutoServoOff\022&.Nrmk.IndyFramework."
    "AutoServoOffConfig\032\034.Nrmk.IndyFramework."
    "Response\"\000\022V\n\017GetAutoServoOff\022\031.Nrmk.Ind"
    "yFramework.Empty\032&.Nrmk.IndyFramework.Au"
    "toServoOffConfig\"\000\022W\n\023SetJointControlGai"
    "n\022 .Nrmk.IndyFramework.JointGainSet\032\034.Nr"
    "mk.IndyFramework.Response\"\000\022T\n\023GetJointC"
    "ontrolGain\022\031.Nrmk.IndyFramework.Empty\032 ."
    "Nrmk.IndyFramework.JointGainSet\"\000\022U\n\022Set"
    "TaskControlGain\022\037.Nrmk.IndyFramework.Tas"
    "kGainSet\032\034.Nrmk.IndyFramework.Response\"\000"
    "\022R\n\022GetTaskControlGain\022\031.Nrmk.IndyFramew"
    "ork.Empty\032\037.Nrmk.IndyFramework.TaskGainS"
    "et\"\000\022_\n\027SetImpedanceControlGain\022$.Nrmk.I"
    "ndyFramework.ImpedanceGainSet\032\034.Nrmk.Ind"
    "yFramework.Response\"\000\022\\\n\027GetImpedanceCon"
    "trolGain\022\031.Nrmk.IndyFramework.Empty\032$.Nr"
    "mk.IndyFramework.ImpedanceGainSet\"\000\022W\n\023S"
    "etForceControlGain\022 .Nrmk.IndyFramework."
    "ForceGainSet\032\034.Nrmk.IndyFramework.Respon"
    "se\"\000\022T\n\023GetForceControlGain\022\031.Nrmk.IndyF"
    "ramework.Empty\032 .Nrmk.IndyFramework.Forc"
    "eGainSet\"\000\022U\n\022SetTestControlGain\022\037.Nrmk."
    "IndyFramework.TestGainSet\032\034.Nrmk.IndyFra"
    "mework.Response\"\000\022R\n\022GetTestControlGain\022"
    "\031.Nrmk.IndyFramework.Empty\032\037.Nrmk.IndyFr"
    "amework.TestGainSet\"\000\022Y\n\024SetCustomContro"
    "lGain\022!.Nrmk.IndyFramework.CustomGainSet"
    "\032\034.Nrmk.IndyFramework.Response\"\000\022V\n\024GetC"
    "ustomControlGain\022\031.Nrmk.IndyFramework.Em"
    "pty\032!.Nrmk.IndyFramework.CustomGainSet\"\000"
    "\022g\n\031SetNewControllerTestOnOff\022*.Nrmk.Ind"
    "yFramework.NewControllerTestState\032\034.Nrmk"
    ".IndyFramework.Response\"\000\022i\n\036GetNewContr"
    "ollerTestOnOffState\022\031.Nrmk.IndyFramework"
    ".Empty\032*.Nrmk.IndyFramework.NewControlle"
    "rTestState\"\000\022V\n\017SetFrictionComp\022#.Nrmk.I"
    "ndyFramework.FrictionCompSet\032\034.Nrmk.Indy"
    "Framework.Response\"\000\022S\n\017GetFrictionComp\022"
    "\031.Nrmk.IndyFramework.Empty\032#.Nrmk.IndyFr"
    "amework.FrictionCompSet\"\000\022Q\n\013SetMountPos"
    "\022\".Nrmk.IndyFramework.MountingAngles\032\034.N"
    "rmk.IndyFramework.Response\"\000\022N\n\013GetMount"
    "Pos\022\031.Nrmk.IndyFramework.Empty\032\".Nrmk.In"
    "dyFramework.MountingAngles\"\000\022U\n\017SetToolP"
    "roperty\022\".Nrmk.IndyFramework.ToolPropert"
    "ies\032\034.Nrmk.IndyFramework.Response\"\000\022R\n\017G"
    "etToolProperty\022\031.Nrmk.IndyFramework.Empt"
    "y\032\".Nrmk.IndyFramework.ToolProperties\"\000\022"
    "Z\n\020SetCollSensLevel\022&.Nrmk.IndyFramework"
    ".CollisionSensLevel\032\034.Nrmk.IndyFramework"
    ".Response\"\000\022W\n\020GetCollSensLevel\022\031.Nrmk.I"
    "ndyFramework.Empty\032&.Nrmk.IndyFramework."
    "CollisionSensLevel\"\000\022[\n\020SetCollSensParam"
    "\022\'.Nrmk.IndyFramework.CollisionThreshold"
    "s\032\034.Nrmk.IndyFramework.Response\"\000\022X\n\020Get"
    "CollSensParam\022\031.Nrmk.IndyFramework.Empty"
    "\032\'.Nrmk.IndyFramework.CollisionThreshold"
    "s\"\000\022T\n\rSetCollPolicy\022#.Nrmk.IndyFramewor"
    "k.CollisionPolicy\032\034.Nrmk.IndyFramework.R"
    "esponse\"\000\022Q\n\rGetCollPolicy\022\031.Nrmk.IndyFr"
    "amework.Empty\032#.Nrmk.IndyFramework.Colli"
    "sionPolicy\"\000\022S\n\017SetSafetyLimits\022 .Nrmk.I"
    "ndyFramework.SafetyLimits\032\034.Nrmk.IndyFra"
    "mework.Response\"\000\022P\n\017GetSafetyLimits\022\031.N"
    "rmk.IndyFramework.Empty\032 .Nrmk.IndyFrame"
    "work.SafetyLimits\"\000\022[\n\023SetSafetyStopConf"
    "ig\022$.Nrmk.IndyFramework.SafetyStopConfig"
    "\032\034.Nrmk.IndyFramework.Response\"\000\022X\n\023GetS"
    "afetyStopConfig\022\031.Nrmk.IndyFramework.Emp"
    "ty\032$.Nrmk.IndyFramework.SafetyStopConfig"
    "\"\000\022V\n\017GetReducedRatio\022\031.Nrmk.IndyFramewo"
    "rk.Empty\032&.Nrmk.IndyFramework.GetReduced"
    "RatioRes\"\000\022V\n\017GetReducedSpeed\022\031.Nrmk.Ind"
    "yFramework.Empty\032&.Nrmk.IndyFramework.Ge"
    "tReducedSpeedRes\"\000\022Y\n\017SetReducedSpeed\022&."
    "Nrmk.IndyFramework.SetReducedSpeedReq\032\034."
    "Nrmk.IndyFramework.Response\"\000\022W\n\021SetFTSe"
    "nsorConfig\022\".Nrmk.IndyFramework.FTSensor"
    "Device\032\034.Nrmk.IndyFramework.Response\"\000\022T"
    "\n\021GetFTSensorConfig\022\031.Nrmk.IndyFramework"
    ".Empty\032\".Nrmk.IndyFramework.FTSensorDevi"
    "ce\"\000\022S\n\017SetTeleOpParams\022 .Nrmk.IndyFrame"
    "work.TeleOpParams\032\034.Nrmk.IndyFramework.R"
    "esponse\"\000\022P\n\017GetTeleOpParams\022\031.Nrmk.Indy"
    "Framework.Empty\032 .Nrmk.IndyFramework.Tel"
    "eOpParams\"\000\022X\n\023GetKinematicsParams\022\031.Nrm"
    "k.IndyFramework.Empty\032$.Nrmk.IndyFramewo"
    "rk.KinematicsParams\"\000b\006proto3"
};
static const ::_pbi::DescriptorTable* const descriptor_table_config_2eproto_deps[2] =
    {
        &::descriptor_table_common_5fmsgs_2eproto,
        &::descriptor_table_config_5fmsgs_2eproto,
};
static ::absl::once_flag descriptor_table_config_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_config_2eproto = {
    false,
    false,
    4589,
    descriptor_table_protodef_config_2eproto,
    "config.proto",
    &descriptor_table_config_2eproto_once,
    descriptor_table_config_2eproto_deps,
    2,
    0,
    schemas,
    file_default_instances,
    TableStruct_config_2eproto::offsets,
    nullptr,
    file_level_enum_descriptors_config_2eproto,
    file_level_service_descriptors_config_2eproto,
};

// This function exists to be marked as weak.
// It can significantly speed up compilation by breaking up LLVM's SCC
// in the .pb.cc translation units. Large translation units see a
// reduction of more than 35% of walltime for optimized builds. Without
// the weak attribute all the messages in the file, including all the
// vtables and everything they use become part of the same SCC through
// a cycle like:
// GetMetadata -> descriptor table -> default instances ->
//   vtables -> GetMetadata
// By adding a weak function here we break the connection from the
// individual vtables back into the descriptor table.
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_config_2eproto_getter() {
  return &descriptor_table_config_2eproto;
}
// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2
static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_config_2eproto(&descriptor_table_config_2eproto);
namespace Nrmk {
namespace IndyFramework {
// @@protoc_insertion_point(namespace_scope)
}  // namespace IndyFramework
}  // namespace Nrmk
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google
// @@protoc_insertion_point(global_scope)
#include "google/protobuf/port_undef.inc"
