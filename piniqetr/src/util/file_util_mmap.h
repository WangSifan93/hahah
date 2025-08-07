#ifndef ONBOARD_UTILS_FILE_UTIL_MMAP_H_
#define ONBOARD_UTILS_FILE_UTIL_MMAP_H_

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "base/singleton.h"

namespace e2e_noa::file_util {
using ScopedFileDescriptor = std::unique_ptr<int, void (*)(int*)>;
absl::StatusOr<ScopedFileDescriptor> OpenFile(
    const std::string& filename, int flags,
    std::optional<mode_t> mode = std::nullopt);

absl::StatusOr<struct stat> GetFileStat(const std::string& filename);
absl::StatusOr<struct stat> GetFileStat(int fd);

class MMapFile {
 public:
  explicit MMapFile(const std::string& filename, bool writable = false);
  ~MMapFile() { Close(); }
  bool GetFileContent(std::string* content) const;
  std::string GetFileContentOrDie() const;
  bool GetFileContentView(std::string_view* content) const;
  std::string_view GetFileContentViewOrDie() const;
  void Close();
  absl::Status status() const { return status_; }
  size_t size() const { return file_size_; }
  void* data() { return mmap_ptr_; }

  absl::Status DumpTo(const std::string& filename);

  absl::Status DumpTo(const std::string& filename, size_t step,
                      const std::function<void(size_t, size_t)>& callback);

  static absl::Status FileAllocate(const std::string& filename, size_t size,
                                   bool maybe_truncate = false);

 private:
  void* mmap_ptr_ = MAP_FAILED;
  char* char_ptr_ = nullptr;
  absl::Status status_;
  size_t file_size_ = 0;
  absl::Status CreateMMap(const std::string& filename, bool writable);
  DISALLOW_COPY_AND_ASSIGN(MMapFile);
};
}  // namespace e2e_noa::file_util

#endif
