// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: rtde.proto

#include "rtde.pb.h"

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
    file_level_enum_descriptors_rtde_2eproto = nullptr;
static constexpr const ::_pb::ServiceDescriptor**
    file_level_service_descriptors_rtde_2eproto = nullptr;
const ::uint32_t TableStruct_rtde_2eproto::offsets[1] = {};
static constexpr ::_pbi::MigrationSchema* schemas = nullptr;
static constexpr ::_pb::Message* const* file_default_instances = nullptr;
const char descriptor_table_protodef_rtde_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
    "\n\nrtde.proto\022\022Nrmk.IndyFramework\032\017rtde_m"
    "sgs.proto\032\021common_msgs.proto2\273\006\n\016RTDataE"
    "xchange\022L\n\rGetMotionData\022\031.Nrmk.IndyFram"
    "ework.Empty\032\036.Nrmk.IndyFramework.MotionD"
    "ata\"\000\022N\n\016GetControlData\022\031.Nrmk.IndyFrame"
    "work.Empty\032\037.Nrmk.IndyFramework.ControlD"
    "ata\"\000\022P\n\017GetControlState\022\031.Nrmk.IndyFram"
    "ework.Empty\032 .Nrmk.IndyFramework.Control"
    "Data2\"\000\022D\n\tGetIOData\022\031.Nrmk.IndyFramewor"
    "k.Empty\032\032.Nrmk.IndyFramework.IOData\"\000\022J\n"
    "\014GetServoData\022\031.Nrmk.IndyFramework.Empty"
    "\032\035.Nrmk.IndyFramework.ServoData\"\000\022R\n\020Get"
    "ViolationData\022\031.Nrmk.IndyFramework.Empty"
    "\032!.Nrmk.IndyFramework.ViolationData\"\000\022b\n"
    "\030GetViolationMessageQueue\022\031.Nrmk.IndyFra"
    "mework.Empty\032).Nrmk.IndyFramework.Violat"
    "ionMessageQueue\"\000\022N\n\016GetProgramData\022\031.Nr"
    "mk.IndyFramework.Empty\032\037.Nrmk.IndyFramew"
    "ork.ProgramData\"\000\022J\n\014GetStopState\022\031.Nrmk"
    ".IndyFramework.Empty\032\035.Nrmk.IndyFramewor"
    "k.StopState\"\000\022S\n\014TestFunction\022\037.Nrmk.Ind"
    "yFramework.TestRequest\032 .Nrmk.IndyFramew"
    "ork.TestResponse\"\000b\006proto3"
};
static const ::_pbi::DescriptorTable* const descriptor_table_rtde_2eproto_deps[2] =
    {
        &::descriptor_table_common_5fmsgs_2eproto,
        &::descriptor_table_rtde_5fmsgs_2eproto,
};
static ::absl::once_flag descriptor_table_rtde_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_rtde_2eproto = {
    false,
    false,
    906,
    descriptor_table_protodef_rtde_2eproto,
    "rtde.proto",
    &descriptor_table_rtde_2eproto_once,
    descriptor_table_rtde_2eproto_deps,
    2,
    0,
    schemas,
    file_default_instances,
    TableStruct_rtde_2eproto::offsets,
    nullptr,
    file_level_enum_descriptors_rtde_2eproto,
    file_level_service_descriptors_rtde_2eproto,
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
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_rtde_2eproto_getter() {
  return &descriptor_table_rtde_2eproto;
}
// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2
static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_rtde_2eproto(&descriptor_table_rtde_2eproto);
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
