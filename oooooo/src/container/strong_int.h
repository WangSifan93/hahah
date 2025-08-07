

#ifndef ONBOARD_CONTAINER_STRONG_INT_H_
#define ONBOARD_CONTAINER_STRONG_INT_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>
#include <system_error>
#include <type_traits>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

namespace e2e_noa {
namespace intops {

struct NullStrongIntValidator {
  template <typename T, typename U>
  static constexpr bool ValidateInit(U arg) {
    return true;
  }

  template <typename T>
  static void ValidateNegate(T value) {}

  template <typename T>
  static void ValidateBitNot(T value) {}

  template <typename T>
  static void ValidateAdd(T lhs, T rhs) {}

  template <typename T>
  static void ValidateSubtract(T lhs, T rhs) {}

  template <typename T, typename U>
  static void ValidateMultiply(T lhs, U rhs) {}

  template <typename T, typename U>
  static void ValidateDivide(T lhs, U rhs) {}

  template <typename T, typename U>
  static void ValidateModulo(T lhs, U rhs) {}

  template <typename T>
  static void ValidateLeftShift(T lhs, int64_t rhs) {}

  template <typename T>
  static void ValidateRightShift(T lhs, int64_t rhs) {}

  template <typename T>
  static void ValidateBitAnd(T lhs, T rhs) {}

  template <typename T>
  static void ValidateBitOr(T lhs, T rhs) {}

  template <typename T>
  static void ValidateBitXor(T lhs, T rhs) {}
};

template <typename TagType, typename NativeType,
          typename ValidatorType = NullStrongIntValidator>
class StrongInt {
 public:
  typedef NativeType ValueType;

  constexpr StrongInt()
      : value_((ValidatorType::template ValidateInit<ValueType>(NativeType()),
                NativeType())) {}

  template <typename ArgTagType, typename ArgNativeType,
            typename ArgValidatorType>
  explicit StrongInt(
      StrongInt<ArgTagType, ArgNativeType, ArgValidatorType> arg) {
    StrongInt* dummy = nullptr;
    StrongInt converted = StrongIntConvert(arg, dummy);
    value_ = converted.value();
  }

  template <typename T, typename = typename std::enable_if<
                            std::is_convertible<T, ValueType>::value>::type>
  explicit constexpr StrongInt(T init_value)
      : value_((ValidatorType::template ValidateInit<ValueType>(init_value),
                static_cast<ValueType>(init_value))) {}

  constexpr ValueType value() const { return value_; }

  template <typename ValType>
  constexpr ValType value() const {
    return static_cast<ValType>(value_);
  }

  static ValueType Max() { return std::numeric_limits<ValueType>::max(); }
  static ValueType Min() { return std::numeric_limits<ValueType>::min(); }

  bool operator!() const { return value_ == 0; }
  const StrongInt operator+() const { return StrongInt(value_); }
  const StrongInt operator-() const {
    ValidatorType::template ValidateNegate<ValueType>(value_);
    return StrongInt(-value_);
  }
  const StrongInt operator~() const {
    ValidatorType::template ValidateBitNot<ValueType>(value_);
    return StrongInt(ValueType(~value_));
  }

  StrongInt& operator++() {
    ValidatorType::template ValidateAdd<ValueType>(value_, ValueType(1));
    ++value_;
    return *this;
  }
  const StrongInt operator++(int postfix_flag) {
    ValidatorType::template ValidateAdd<ValueType>(value_, ValueType(1));
    StrongInt temp(*this);
    ++value_;
    return temp;
  }
  StrongInt& operator--() {
    ValidatorType::template ValidateSubtract<ValueType>(value_, ValueType(1));
    --value_;
    return *this;
  }
  const StrongInt operator--(int postfix_flag) {
    ValidatorType::template ValidateSubtract<ValueType>(value_, ValueType(1));
    StrongInt temp(*this);
    --value_;
    return temp;
  }

  StrongInt& operator+=(StrongInt arg) {
    ValidatorType::template ValidateAdd<ValueType>(value_, arg.value());
    value_ += arg.value();
    return *this;
  }
  StrongInt& operator-=(StrongInt arg) {
    ValidatorType::template ValidateSubtract<ValueType>(value_, arg.value());
    value_ -= arg.value();
    return *this;
  }
  template <typename ArgType>
  StrongInt& operator*=(ArgType arg) {
    ValidatorType::template ValidateMultiply<ValueType, ArgType>(value_, arg);
    value_ *= arg;
    return *this;
  }
  template <typename ArgType>
  StrongInt& operator/=(ArgType arg) {
    ValidatorType::template ValidateDivide<ValueType, ArgType>(value_, arg);
    value_ /= arg;
    return *this;
  }
  template <typename ArgType>
  StrongInt& operator%=(ArgType arg) {
    ValidatorType::template ValidateModulo<ValueType, ArgType>(value_, arg);
    value_ %= arg;
    return *this;
  }
  StrongInt& operator<<=(int64_t arg) {
    ValidatorType::template ValidateLeftShift<ValueType>(value_, arg);
    value_ <<= arg;
    return *this;
  }
  StrongInt& operator>>=(int64_t arg) {
    ValidatorType::template ValidateRightShift<ValueType>(value_, arg);
    value_ >>= arg;
    return *this;
  }
  StrongInt& operator&=(StrongInt arg) {
    ValidatorType::template ValidateBitAnd<ValueType>(value_, arg.value());
    value_ &= arg.value();
    return *this;
  }
  StrongInt& operator|=(StrongInt arg) {
    ValidatorType::template ValidateBitOr<ValueType>(value_, arg.value());
    value_ |= arg.value();
    return *this;
  }
  StrongInt& operator^=(StrongInt arg) {
    ValidatorType::template ValidateBitXor<ValueType>(value_, arg.value());
    value_ ^= arg.value();
    return *this;
  }

  friend absl::FormatConvertResult<absl::FormatConversionCharSet::kIntegral>
  AbslFormatConvert(StrongInt p, const absl::FormatConversionSpec& spec,
                    absl::FormatSink* s) {
    s->Append(absl::StrCat(p.value()));
    return {true};
  }

#define STRONG_INT_COMPARISON_OP(op)             \
  inline bool operator op(StrongInt rhs) const { \
    return value() op rhs.value();               \
  }
  STRONG_INT_COMPARISON_OP(==);
  STRONG_INT_COMPARISON_OP(!=);
  STRONG_INT_COMPARISON_OP(<);
  STRONG_INT_COMPARISON_OP(<=);
  STRONG_INT_COMPARISON_OP(>);
  STRONG_INT_COMPARISON_OP(>=);
#undef STRONG_INT_COMPARISON_OP

  operator absl::AlphaNum() const { return absl::AlphaNum(value()); }

 private:
  ValueType value_;

  static_assert(std::is_integral<ValueType>::value,
                "invalid integer type for strong int");
};

template <typename TagType, typename ValueType, typename ValidatorType>
std::ostream& operator<<(std::ostream& os,
                         StrongInt<TagType, ValueType, ValidatorType> arg) {
  return os << arg.value();
}

template <typename TagType, typename ValidatorType>
std::ostream& operator<<(std::ostream& os,
                         StrongInt<TagType, int8_t, ValidatorType> arg) {
  return os << static_cast<int>(arg.value());
}

template <typename TagType, typename ValidatorType>
std::ostream& operator<<(std::ostream& os,
                         StrongInt<TagType, uint8_t, ValidatorType> arg) {
  return os << static_cast<unsigned int>(arg.value());
}

}  // namespace intops
}  // namespace e2e_noa

#define DEFINE_STRONG_INT_TYPE(type_name, value_type)                         \
  struct type_name##_strong_int_tag_ {};                                      \
  typedef e2e_noa::intops::StrongInt<type_name##_strong_int_tag_, value_type, \
                                     e2e_noa::intops::NullStrongIntValidator> \
      type_name;

#define STRONG_INT_VS_STRONG_INT_BINARY_OP(op)                             \
  template <typename TagType, typename ValueType, typename ValidatorType>  \
  inline e2e_noa::intops::StrongInt<TagType, ValueType, ValidatorType>     \
  operator op(                                                             \
      e2e_noa::intops::StrongInt<TagType, ValueType, ValidatorType> lhs,   \
      e2e_noa::intops::StrongInt<TagType, ValueType, ValidatorType> rhs) { \
    lhs op## = rhs;                                                        \
    return lhs;                                                            \
  }
STRONG_INT_VS_STRONG_INT_BINARY_OP(+);
STRONG_INT_VS_STRONG_INT_BINARY_OP(-);
STRONG_INT_VS_STRONG_INT_BINARY_OP(&);
STRONG_INT_VS_STRONG_INT_BINARY_OP(|);
STRONG_INT_VS_STRONG_INT_BINARY_OP(^);
#undef STRONG_INT_VS_STRONG_INT_BINARY_OP

#define STRONG_INT_VS_NUMERIC_BINARY_OP(op)                               \
  template <typename TagType, typename ValueType, typename ValidatorType, \
            typename NumType>                                             \
  inline e2e_noa::intops::StrongInt<TagType, ValueType, ValidatorType>    \
  operator op(                                                            \
      e2e_noa::intops::StrongInt<TagType, ValueType, ValidatorType> lhs,  \
      NumType rhs) {                                                      \
    lhs op## = rhs;                                                       \
    return lhs;                                                           \
  }
#define NUMERIC_VS_STRONG_INT_BINARY_OP(op)                                \
  template <typename TagType, typename ValueType, typename ValidatorType,  \
            typename NumType>                                              \
  inline e2e_noa::intops::StrongInt<TagType, ValueType, ValidatorType>     \
  operator op(                                                             \
      NumType lhs,                                                         \
      e2e_noa::intops::StrongInt<TagType, ValueType, ValidatorType> rhs) { \
    rhs op## = lhs;                                                        \
    return rhs;                                                            \
  }
STRONG_INT_VS_NUMERIC_BINARY_OP(*);
NUMERIC_VS_STRONG_INT_BINARY_OP(*);
STRONG_INT_VS_NUMERIC_BINARY_OP(/);
STRONG_INT_VS_NUMERIC_BINARY_OP(%);
STRONG_INT_VS_NUMERIC_BINARY_OP(<<);
STRONG_INT_VS_NUMERIC_BINARY_OP(>>);
#undef STRONG_INT_VS_NUMERIC_BINARY_OP
#undef NUMERIC_VS_STRONG_INT_BINARY_OP

namespace std {
template <typename Tag, typename Value, typename Validator>
struct hash<e2e_noa::intops::StrongInt<Tag, Value, Validator> > {
  size_t operator()(
      const e2e_noa::intops::StrongInt<Tag, Value, Validator>& idx) const {
    return static_cast<size_t>(idx.value());
  }
};

template <typename Tag, typename Value, typename Validator>
std::string to_string(e2e_noa::intops::StrongInt<Tag, Value, Validator> i) {
  return std::to_string(i.value());
}
}  // namespace std

#endif
