// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: shared_msgs.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_shared_5fmsgs_2eproto_2epb_2eh
#define GOOGLE_PROTOBUF_INCLUDED_shared_5fmsgs_2eproto_2epb_2eh

#include <limits>
#include <string>
#include <type_traits>

#include "google/protobuf/port_def.inc"
#if PROTOBUF_VERSION < 4024000
#error "This file was generated by a newer version of protoc which is"
#error "incompatible with your Protocol Buffer headers. Please update"
#error "your headers."
#endif  // PROTOBUF_VERSION

#if 4024003 < PROTOBUF_MIN_PROTOC_VERSION
#error "This file was generated by an older version of protoc which is"
#error "incompatible with your Protocol Buffer headers. Please"
#error "regenerate this file with a newer version of protoc."
#endif  // PROTOBUF_MIN_PROTOC_VERSION
#include "google/protobuf/port_undef.inc"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/arena.h"
#include "google/protobuf/arenastring.h"
#include "google/protobuf/generated_message_tctable_decl.h"
#include "google/protobuf/generated_message_util.h"
#include "google/protobuf/metadata_lite.h"
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/message.h"
#include "google/protobuf/repeated_field.h"  // IWYU pragma: export
#include "google/protobuf/extension_set.h"  // IWYU pragma: export
#include "google/protobuf/unknown_field_set.h"
// @@protoc_insertion_point(includes)

// Must be included last.
#include "google/protobuf/port_def.inc"

#define PROTOBUF_INTERNAL_EXPORT_shared_5fmsgs_2eproto

namespace google {
namespace protobuf {
namespace internal {
class AnyMetadata;
}  // namespace internal
}  // namespace protobuf
}  // namespace google

// Internal implementation detail -- do not use these members.
struct TableStruct_shared_5fmsgs_2eproto {
  static const ::uint32_t offsets[];
};
extern const ::google::protobuf::internal::DescriptorTable
    descriptor_table_shared_5fmsgs_2eproto;
namespace IndyFramework {
namespace Protobuf {
namespace Shared {
class NamedReferencePosition;
struct NamedReferencePositionDefaultTypeInternal;
extern NamedReferencePositionDefaultTypeInternal _NamedReferencePosition_default_instance_;
}  // namespace Shared
}  // namespace Protobuf
}  // namespace IndyFramework
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google

namespace IndyFramework {
namespace Protobuf {
namespace Shared {

// ===================================================================


// -------------------------------------------------------------------

class NamedReferencePosition final :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:IndyFramework.Protobuf.Shared.NamedReferencePosition) */ {
 public:
  inline NamedReferencePosition() : NamedReferencePosition(nullptr) {}
  ~NamedReferencePosition() override;
  template<typename = void>
  explicit PROTOBUF_CONSTEXPR NamedReferencePosition(::google::protobuf::internal::ConstantInitialized);

  NamedReferencePosition(const NamedReferencePosition& from);
  NamedReferencePosition(NamedReferencePosition&& from) noexcept
    : NamedReferencePosition() {
    *this = ::std::move(from);
  }

  inline NamedReferencePosition& operator=(const NamedReferencePosition& from) {
    CopyFrom(from);
    return *this;
  }
  inline NamedReferencePosition& operator=(NamedReferencePosition&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::google::protobuf::UnknownFieldSet>(::google::protobuf::UnknownFieldSet::default_instance);
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::google::protobuf::UnknownFieldSet>();
  }

  static const ::google::protobuf::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::google::protobuf::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::google::protobuf::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const NamedReferencePosition& default_instance() {
    return *internal_default_instance();
  }
  static inline const NamedReferencePosition* internal_default_instance() {
    return reinterpret_cast<const NamedReferencePosition*>(
               &_NamedReferencePosition_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(NamedReferencePosition& a, NamedReferencePosition& b) {
    a.Swap(&b);
  }
  inline void Swap(NamedReferencePosition* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::google::protobuf::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(NamedReferencePosition* other) {
    if (other == this) return;
    ABSL_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  NamedReferencePosition* New(::google::protobuf::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<NamedReferencePosition>(arena);
  }
  using ::google::protobuf::Message::CopyFrom;
  void CopyFrom(const NamedReferencePosition& from);
  using ::google::protobuf::Message::MergeFrom;
  void MergeFrom( const NamedReferencePosition& from) {
    NamedReferencePosition::MergeImpl(*this, from);
  }
  private:
  static void MergeImpl(::google::protobuf::Message& to_msg, const ::google::protobuf::Message& from_msg);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  ::size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::google::protobuf::internal::ParseContext* ctx) final;
  ::uint8_t* _InternalSerialize(
      ::uint8_t* target, ::google::protobuf::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _impl_._cached_size_.Get(); }

  private:
  void SharedCtor(::google::protobuf::Arena* arena);
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(NamedReferencePosition* other);

  private:
  friend class ::google::protobuf::internal::AnyMetadata;
  static ::absl::string_view FullMessageName() {
    return "IndyFramework.Protobuf.Shared.NamedReferencePosition";
  }
  protected:
  explicit NamedReferencePosition(::google::protobuf::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::google::protobuf::Message::ClassData*GetClassData() const final;

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kTposFieldNumber = 2,
    kTpos0FieldNumber = 3,
    kTpos1FieldNumber = 4,
    kTpos2FieldNumber = 5,
    kJpos0FieldNumber = 6,
    kJpos1FieldNumber = 7,
    kJpos2FieldNumber = 8,
    kNameFieldNumber = 1,
  };
  // repeated float tpos = 2;
  int tpos_size() const;
  private:
  int _internal_tpos_size() const;

  public:
  void clear_tpos() ;
  float tpos(int index) const;
  void set_tpos(int index, float value);
  void add_tpos(float value);
  const ::google::protobuf::RepeatedField<float>& tpos() const;
  ::google::protobuf::RepeatedField<float>* mutable_tpos();

  private:
  const ::google::protobuf::RepeatedField<float>& _internal_tpos() const;
  ::google::protobuf::RepeatedField<float>* _internal_mutable_tpos();

  public:
  // repeated float tpos0 = 3;
  int tpos0_size() const;
  private:
  int _internal_tpos0_size() const;

  public:
  void clear_tpos0() ;
  float tpos0(int index) const;
  void set_tpos0(int index, float value);
  void add_tpos0(float value);
  const ::google::protobuf::RepeatedField<float>& tpos0() const;
  ::google::protobuf::RepeatedField<float>* mutable_tpos0();

  private:
  const ::google::protobuf::RepeatedField<float>& _internal_tpos0() const;
  ::google::protobuf::RepeatedField<float>* _internal_mutable_tpos0();

  public:
  // repeated float tpos1 = 4;
  int tpos1_size() const;
  private:
  int _internal_tpos1_size() const;

  public:
  void clear_tpos1() ;
  float tpos1(int index) const;
  void set_tpos1(int index, float value);
  void add_tpos1(float value);
  const ::google::protobuf::RepeatedField<float>& tpos1() const;
  ::google::protobuf::RepeatedField<float>* mutable_tpos1();

  private:
  const ::google::protobuf::RepeatedField<float>& _internal_tpos1() const;
  ::google::protobuf::RepeatedField<float>* _internal_mutable_tpos1();

  public:
  // repeated float tpos2 = 5;
  int tpos2_size() const;
  private:
  int _internal_tpos2_size() const;

  public:
  void clear_tpos2() ;
  float tpos2(int index) const;
  void set_tpos2(int index, float value);
  void add_tpos2(float value);
  const ::google::protobuf::RepeatedField<float>& tpos2() const;
  ::google::protobuf::RepeatedField<float>* mutable_tpos2();

  private:
  const ::google::protobuf::RepeatedField<float>& _internal_tpos2() const;
  ::google::protobuf::RepeatedField<float>* _internal_mutable_tpos2();

  public:
  // repeated float jpos0 = 6;
  int jpos0_size() const;
  private:
  int _internal_jpos0_size() const;

  public:
  void clear_jpos0() ;
  float jpos0(int index) const;
  void set_jpos0(int index, float value);
  void add_jpos0(float value);
  const ::google::protobuf::RepeatedField<float>& jpos0() const;
  ::google::protobuf::RepeatedField<float>* mutable_jpos0();

  private:
  const ::google::protobuf::RepeatedField<float>& _internal_jpos0() const;
  ::google::protobuf::RepeatedField<float>* _internal_mutable_jpos0();

  public:
  // repeated float jpos1 = 7;
  int jpos1_size() const;
  private:
  int _internal_jpos1_size() const;

  public:
  void clear_jpos1() ;
  float jpos1(int index) const;
  void set_jpos1(int index, float value);
  void add_jpos1(float value);
  const ::google::protobuf::RepeatedField<float>& jpos1() const;
  ::google::protobuf::RepeatedField<float>* mutable_jpos1();

  private:
  const ::google::protobuf::RepeatedField<float>& _internal_jpos1() const;
  ::google::protobuf::RepeatedField<float>* _internal_mutable_jpos1();

  public:
  // repeated float jpos2 = 8;
  int jpos2_size() const;
  private:
  int _internal_jpos2_size() const;

  public:
  void clear_jpos2() ;
  float jpos2(int index) const;
  void set_jpos2(int index, float value);
  void add_jpos2(float value);
  const ::google::protobuf::RepeatedField<float>& jpos2() const;
  ::google::protobuf::RepeatedField<float>* mutable_jpos2();

  private:
  const ::google::protobuf::RepeatedField<float>& _internal_jpos2() const;
  ::google::protobuf::RepeatedField<float>* _internal_mutable_jpos2();

  public:
  // string name = 1;
  void clear_name() ;
  const std::string& name() const;
  template <typename Arg_ = const std::string&, typename... Args_>
  void set_name(Arg_&& arg, Args_... args);
  std::string* mutable_name();
  PROTOBUF_NODISCARD std::string* release_name();
  void set_allocated_name(std::string* ptr);

  private:
  const std::string& _internal_name() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_name(
      const std::string& value);
  std::string* _internal_mutable_name();

  public:
  // @@protoc_insertion_point(class_scope:IndyFramework.Protobuf.Shared.NamedReferencePosition)
 private:
  class _Internal;

  friend class ::google::protobuf::internal::TcParser;
  static const ::google::protobuf::internal::TcParseTable<3, 8, 0, 73, 2> _table_;
  template <typename T> friend class ::google::protobuf::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::google::protobuf::RepeatedField<float> tpos_;
    ::google::protobuf::RepeatedField<float> tpos0_;
    ::google::protobuf::RepeatedField<float> tpos1_;
    ::google::protobuf::RepeatedField<float> tpos2_;
    ::google::protobuf::RepeatedField<float> jpos0_;
    ::google::protobuf::RepeatedField<float> jpos1_;
    ::google::protobuf::RepeatedField<float> jpos2_;
    ::google::protobuf::internal::ArenaStringPtr name_;
    mutable ::google::protobuf::internal::CachedSize _cached_size_;
    PROTOBUF_TSAN_DECLARE_MEMBER
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_shared_5fmsgs_2eproto;
};

// ===================================================================




// ===================================================================


#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// -------------------------------------------------------------------

// NamedReferencePosition

// string name = 1;
inline void NamedReferencePosition::clear_name() {
  _impl_.name_.ClearToEmpty();
}
inline const std::string& NamedReferencePosition::name() const {
  // @@protoc_insertion_point(field_get:IndyFramework.Protobuf.Shared.NamedReferencePosition.name)
  return _internal_name();
}
template <typename Arg_, typename... Args_>
inline PROTOBUF_ALWAYS_INLINE void NamedReferencePosition::set_name(Arg_&& arg,
                                                     Args_... args) {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  ;
  _impl_.name_.Set(static_cast<Arg_&&>(arg), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:IndyFramework.Protobuf.Shared.NamedReferencePosition.name)
}
inline std::string* NamedReferencePosition::mutable_name() {
  std::string* _s = _internal_mutable_name();
  // @@protoc_insertion_point(field_mutable:IndyFramework.Protobuf.Shared.NamedReferencePosition.name)
  return _s;
}
inline const std::string& NamedReferencePosition::_internal_name() const {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return _impl_.name_.Get();
}
inline void NamedReferencePosition::_internal_set_name(const std::string& value) {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  ;
  _impl_.name_.Set(value, GetArenaForAllocation());
}
inline std::string* NamedReferencePosition::_internal_mutable_name() {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  ;
  return _impl_.name_.Mutable( GetArenaForAllocation());
}
inline std::string* NamedReferencePosition::release_name() {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  // @@protoc_insertion_point(field_release:IndyFramework.Protobuf.Shared.NamedReferencePosition.name)
  return _impl_.name_.Release();
}
inline void NamedReferencePosition::set_allocated_name(std::string* value) {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  _impl_.name_.SetAllocated(value, GetArenaForAllocation());
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
        if (_impl_.name_.IsDefault()) {
          _impl_.name_.Set("", GetArenaForAllocation());
        }
  #endif  // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:IndyFramework.Protobuf.Shared.NamedReferencePosition.name)
}

// repeated float tpos = 2;
inline int NamedReferencePosition::_internal_tpos_size() const {
  return _internal_tpos().size();
}
inline int NamedReferencePosition::tpos_size() const {
  return _internal_tpos_size();
}
inline void NamedReferencePosition::clear_tpos() {
  _internal_mutable_tpos()->Clear();
}
inline float NamedReferencePosition::tpos(int index) const {
  // @@protoc_insertion_point(field_get:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos)
  return _internal_tpos().Get(index);
}
inline void NamedReferencePosition::set_tpos(int index, float value) {
  _internal_mutable_tpos()->Set(index, value);
  // @@protoc_insertion_point(field_set:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos)
}
inline void NamedReferencePosition::add_tpos(float value) {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  _internal_mutable_tpos()->Add(value);
  // @@protoc_insertion_point(field_add:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos)
}
inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::tpos() const {
  // @@protoc_insertion_point(field_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos)
  return _internal_tpos();
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::mutable_tpos() {
  // @@protoc_insertion_point(field_mutable_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  return _internal_mutable_tpos();
}

inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::_internal_tpos() const {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return _impl_.tpos_;
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::_internal_mutable_tpos() {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return &_impl_.tpos_;
}

// repeated float tpos0 = 3;
inline int NamedReferencePosition::_internal_tpos0_size() const {
  return _internal_tpos0().size();
}
inline int NamedReferencePosition::tpos0_size() const {
  return _internal_tpos0_size();
}
inline void NamedReferencePosition::clear_tpos0() {
  _internal_mutable_tpos0()->Clear();
}
inline float NamedReferencePosition::tpos0(int index) const {
  // @@protoc_insertion_point(field_get:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos0)
  return _internal_tpos0().Get(index);
}
inline void NamedReferencePosition::set_tpos0(int index, float value) {
  _internal_mutable_tpos0()->Set(index, value);
  // @@protoc_insertion_point(field_set:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos0)
}
inline void NamedReferencePosition::add_tpos0(float value) {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  _internal_mutable_tpos0()->Add(value);
  // @@protoc_insertion_point(field_add:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos0)
}
inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::tpos0() const {
  // @@protoc_insertion_point(field_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos0)
  return _internal_tpos0();
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::mutable_tpos0() {
  // @@protoc_insertion_point(field_mutable_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos0)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  return _internal_mutable_tpos0();
}

inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::_internal_tpos0() const {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return _impl_.tpos0_;
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::_internal_mutable_tpos0() {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return &_impl_.tpos0_;
}

// repeated float tpos1 = 4;
inline int NamedReferencePosition::_internal_tpos1_size() const {
  return _internal_tpos1().size();
}
inline int NamedReferencePosition::tpos1_size() const {
  return _internal_tpos1_size();
}
inline void NamedReferencePosition::clear_tpos1() {
  _internal_mutable_tpos1()->Clear();
}
inline float NamedReferencePosition::tpos1(int index) const {
  // @@protoc_insertion_point(field_get:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos1)
  return _internal_tpos1().Get(index);
}
inline void NamedReferencePosition::set_tpos1(int index, float value) {
  _internal_mutable_tpos1()->Set(index, value);
  // @@protoc_insertion_point(field_set:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos1)
}
inline void NamedReferencePosition::add_tpos1(float value) {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  _internal_mutable_tpos1()->Add(value);
  // @@protoc_insertion_point(field_add:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos1)
}
inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::tpos1() const {
  // @@protoc_insertion_point(field_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos1)
  return _internal_tpos1();
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::mutable_tpos1() {
  // @@protoc_insertion_point(field_mutable_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos1)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  return _internal_mutable_tpos1();
}

inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::_internal_tpos1() const {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return _impl_.tpos1_;
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::_internal_mutable_tpos1() {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return &_impl_.tpos1_;
}

// repeated float tpos2 = 5;
inline int NamedReferencePosition::_internal_tpos2_size() const {
  return _internal_tpos2().size();
}
inline int NamedReferencePosition::tpos2_size() const {
  return _internal_tpos2_size();
}
inline void NamedReferencePosition::clear_tpos2() {
  _internal_mutable_tpos2()->Clear();
}
inline float NamedReferencePosition::tpos2(int index) const {
  // @@protoc_insertion_point(field_get:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos2)
  return _internal_tpos2().Get(index);
}
inline void NamedReferencePosition::set_tpos2(int index, float value) {
  _internal_mutable_tpos2()->Set(index, value);
  // @@protoc_insertion_point(field_set:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos2)
}
inline void NamedReferencePosition::add_tpos2(float value) {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  _internal_mutable_tpos2()->Add(value);
  // @@protoc_insertion_point(field_add:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos2)
}
inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::tpos2() const {
  // @@protoc_insertion_point(field_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos2)
  return _internal_tpos2();
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::mutable_tpos2() {
  // @@protoc_insertion_point(field_mutable_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.tpos2)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  return _internal_mutable_tpos2();
}

inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::_internal_tpos2() const {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return _impl_.tpos2_;
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::_internal_mutable_tpos2() {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return &_impl_.tpos2_;
}

// repeated float jpos0 = 6;
inline int NamedReferencePosition::_internal_jpos0_size() const {
  return _internal_jpos0().size();
}
inline int NamedReferencePosition::jpos0_size() const {
  return _internal_jpos0_size();
}
inline void NamedReferencePosition::clear_jpos0() {
  _internal_mutable_jpos0()->Clear();
}
inline float NamedReferencePosition::jpos0(int index) const {
  // @@protoc_insertion_point(field_get:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos0)
  return _internal_jpos0().Get(index);
}
inline void NamedReferencePosition::set_jpos0(int index, float value) {
  _internal_mutable_jpos0()->Set(index, value);
  // @@protoc_insertion_point(field_set:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos0)
}
inline void NamedReferencePosition::add_jpos0(float value) {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  _internal_mutable_jpos0()->Add(value);
  // @@protoc_insertion_point(field_add:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos0)
}
inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::jpos0() const {
  // @@protoc_insertion_point(field_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos0)
  return _internal_jpos0();
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::mutable_jpos0() {
  // @@protoc_insertion_point(field_mutable_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos0)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  return _internal_mutable_jpos0();
}

inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::_internal_jpos0() const {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return _impl_.jpos0_;
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::_internal_mutable_jpos0() {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return &_impl_.jpos0_;
}

// repeated float jpos1 = 7;
inline int NamedReferencePosition::_internal_jpos1_size() const {
  return _internal_jpos1().size();
}
inline int NamedReferencePosition::jpos1_size() const {
  return _internal_jpos1_size();
}
inline void NamedReferencePosition::clear_jpos1() {
  _internal_mutable_jpos1()->Clear();
}
inline float NamedReferencePosition::jpos1(int index) const {
  // @@protoc_insertion_point(field_get:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos1)
  return _internal_jpos1().Get(index);
}
inline void NamedReferencePosition::set_jpos1(int index, float value) {
  _internal_mutable_jpos1()->Set(index, value);
  // @@protoc_insertion_point(field_set:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos1)
}
inline void NamedReferencePosition::add_jpos1(float value) {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  _internal_mutable_jpos1()->Add(value);
  // @@protoc_insertion_point(field_add:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos1)
}
inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::jpos1() const {
  // @@protoc_insertion_point(field_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos1)
  return _internal_jpos1();
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::mutable_jpos1() {
  // @@protoc_insertion_point(field_mutable_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos1)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  return _internal_mutable_jpos1();
}

inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::_internal_jpos1() const {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return _impl_.jpos1_;
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::_internal_mutable_jpos1() {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return &_impl_.jpos1_;
}

// repeated float jpos2 = 8;
inline int NamedReferencePosition::_internal_jpos2_size() const {
  return _internal_jpos2().size();
}
inline int NamedReferencePosition::jpos2_size() const {
  return _internal_jpos2_size();
}
inline void NamedReferencePosition::clear_jpos2() {
  _internal_mutable_jpos2()->Clear();
}
inline float NamedReferencePosition::jpos2(int index) const {
  // @@protoc_insertion_point(field_get:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos2)
  return _internal_jpos2().Get(index);
}
inline void NamedReferencePosition::set_jpos2(int index, float value) {
  _internal_mutable_jpos2()->Set(index, value);
  // @@protoc_insertion_point(field_set:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos2)
}
inline void NamedReferencePosition::add_jpos2(float value) {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  _internal_mutable_jpos2()->Add(value);
  // @@protoc_insertion_point(field_add:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos2)
}
inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::jpos2() const {
  // @@protoc_insertion_point(field_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos2)
  return _internal_jpos2();
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::mutable_jpos2() {
  // @@protoc_insertion_point(field_mutable_list:IndyFramework.Protobuf.Shared.NamedReferencePosition.jpos2)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  return _internal_mutable_jpos2();
}

inline const ::google::protobuf::RepeatedField<float>& NamedReferencePosition::_internal_jpos2() const {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return _impl_.jpos2_;
}
inline ::google::protobuf::RepeatedField<float>* NamedReferencePosition::_internal_mutable_jpos2() {
  PROTOBUF_TSAN_READ(&_impl_._tsan_detect_race);
  return &_impl_.jpos2_;
}

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)
}  // namespace Shared
}  // namespace Protobuf
}  // namespace IndyFramework


// @@protoc_insertion_point(global_scope)

#include "google/protobuf/port_undef.inc"

#endif  // GOOGLE_PROTOBUF_INCLUDED_shared_5fmsgs_2eproto_2epb_2eh
