#ifndef ONBOARD_PLANNER_DECISION_CONSTRAINT_BUILDER_H_
#define ONBOARD_PLANNER_DECISION_CONSTRAINT_BUILDER_H_

#include "absl/status/statusor.h"
#include "descriptor/descriptor_input.h"
#include "descriptor/descriptor_output.h"

namespace e2e_noa::planning {
absl::StatusOr<Descriptor> BuildDescriptorResults(
    const DescriptorInput& descriptor_input);
}

#endif
