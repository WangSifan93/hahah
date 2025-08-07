#ifndef PREDICTION_UTIL_PERCEPTION_UTIL_H_
#define PREDICTION_UTIL_PERCEPTION_UTIL_H_

#include "absl/status/status.h"
#include "perception.pb.h"

namespace e2e_noa {
namespace prediction {

absl::Status AlignPerceptionObjectTime(double current_time,
                                       ObjectProto* object);

}
}  // namespace e2e_noa

#endif
