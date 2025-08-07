#ifndef ONBOARD_UTILS_PROTO_UTIL_H_
#define ONBOARD_UTILS_PROTO_UTIL_H_

#include <optional>
#include <string>

#include "absl/strings/string_view.h"
#include "google/protobuf/io/tokenizer.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/message.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/field_comparator.h"
#include "google/protobuf/util/message_differencer.h"

namespace e2e_noa {

inline bool ProtoEquals(const google::protobuf::Message& lhs,
                        const google::protobuf::Message& rhs) {
  return google::protobuf::util::MessageDifferencer::Equivalent(lhs, rhs);
}

inline bool operator==(const google::protobuf::Message& lhs,
                       const google::protobuf::Message& rhs) {
  return ProtoEquals(lhs, rhs);
}

inline bool operator!=(const google::protobuf::Message& lhs,
                       const google::protobuf::Message& rhs) {
  return !ProtoEquals(lhs, rhs);
}

template <class Proto>
void TextToProto(const std::string& text, Proto* proto) {
  google::protobuf::TextFormat::ParseFromString(text, proto);
}

template <class Proto>
std::optional<std::string> ProtoDiff(const Proto& lhs, const Proto& rhs,
                                     bool partial, double margin = 0.0) {
  using google::protobuf::util::MessageDifferencer;
  MessageDifferencer diff;
  diff.set_report_matches(false);
  diff.set_report_moves(false);
  if (partial) {
    diff.set_scope(MessageDifferencer::PARTIAL);
  }
  using google::protobuf::util::DefaultFieldComparator;
  DefaultFieldComparator field_comparator;
  if (margin != 0.0) {
    field_comparator.set_float_comparison(DefaultFieldComparator::APPROXIMATE);
    field_comparator.SetDefaultFractionAndMargin(0.0, margin);
    diff.set_field_comparator(&field_comparator);
  }
  std::string report_diff;
  bool same = true;
  {
    google::protobuf::io::StringOutputStream ostream(&report_diff);
    MessageDifferencer::StreamReporter reporter(&ostream);
    diff.ReportDifferencesTo(&reporter);
    same = diff.Compare(lhs, rhs);
    if (same) return std::nullopt;
  }
  return report_diff;
}

struct StringToProtoLogCollector : public google::protobuf::io::ErrorCollector {
  struct LastParseError {
    int line = -1;
    int column = -1;
    std::string message;
    std::string DebugString() const;
  };
  StringToProtoLogCollector() = default;
  virtual ~StringToProtoLogCollector() = default;
  void AddError(int line, int column, const std::string& message) override;
  void AddWarning(int line, int column, const std::string& message) override;

  const LastParseError& GetLastParseError() const { return last_parse_error; }

  LastParseError last_parse_error;
};

bool DetailedStringProtoLogging(
    std::string_view proto_string, google::protobuf::Message* proto,
    google::protobuf::io::ErrorCollector* error_collector);

void FillInMissingFieldsWithDefault(
    const google::protobuf::Message& default_proto,
    google::protobuf::Message* proto);

}  
#endif
