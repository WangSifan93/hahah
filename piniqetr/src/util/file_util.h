#ifndef ST_PLANNING_UTIL_FILE_UTIL
#define ST_PLANNING_UTIL_FILE_UTIL

#include <string>
#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/message.h"

namespace e2e_noa {
namespace file_util {

absl::Status RemoveFile(std::string_view fpath);

absl::Status CreateDirectory(std::string_view dir);

absl::Status DeleteFolderStructure(std::string_view dir);

absl::StatusOr<std::vector<std::string>> ListDirectory(std::string_view dir);

absl::StatusOr<std::vector<std::string>> ListDirectoryRecursive(
    std::string_view dir);

bool GetFileContent(const std::string& filename, std::string* content);

bool GetFileContentByGetline(const std::string& filename, std::string* content);

std::string GetFileContentOrDie(const std::string& filename);

bool SetFileContent(const std::string& content, const std::string& filename);

void SetFileContentOrDie(const std::string& content,
                         const std::string& filename);

std::string GetFileExtension(const std::string& filename);

bool StringToProto(const std::string& proto_string,
                   google::protobuf::Message* proto);

bool TextFileToProto(const std::string& filename,
                     google::protobuf::Message* proto);

bool BinaryFileToProtocol(const std::string& filename,
                          google::protobuf::Message* proto);

bool FileToProto(const std::string& filename, google::protobuf::Message* proto);

bool ProtocolToTextFile(const google::protobuf::Message& proto,
                        const std::string& filename);

bool ProtocolToBinaryFile(const google::protobuf::Message& proto,
                          const std::string& filename);

bool ProtoToJsonFile(const google::protobuf::Message& proto,
                     const std::string& filename);

}  // namespace file_util
}  // namespace e2e_noa

#endif
