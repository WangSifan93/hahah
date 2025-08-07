#include "plan/planner_params_builder.h"

#include <cmath>
#include <string>

#include "aabox3d.pb.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "math/util.h"
#include "plan/planner_flags.h"
#include "speed_planning_params.pb.h"
#include "util/file_util.h"
#include "util/proto_util.h"
#include "util/status_macros.h"

namespace e2e_noa {
namespace planning {

namespace {

std::string GetDefaultParamsFile() {
  return absl::StrCat("planner_default_params.pb.txt");
}

absl::Status ValidateParams(const google::protobuf::Message& params) {
  const google::protobuf::Descriptor* descriptor = params.GetDescriptor();
  const google::protobuf::Reflection* reflection = params.GetReflection();
  for (int i = 0; i < descriptor->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = descriptor->field(i);
    if (!field->is_optional()) continue;
    if (!reflection->HasField(params, field)) {
      return absl::NotFoundError(
          absl::StrCat("Missing field: ", field->full_name()));
    }
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      RETURN_IF_ERROR(ValidateParams(reflection->GetMessage(params, field)));
    }
  }
  return absl::OkStatus();
}

void ComputeVehicleModelParamsOfPlanner(
    const VehicleGeometryParamsProto& vehicle_geo_params,
    PlannerVehicleModelParamsProto* vehicle_models_params) {
  const double half_width = 0.5 * vehicle_geo_params.width();

  vehicle_models_params->mutable_trajectory_optimizer_vehicle_model_params()
      ->clear_circles();
  vehicle_models_params->mutable_trajectory_optimizer_vehicle_model_params()
      ->clear_mirror_circles();
  {
    constexpr double kFrontCircleDefualtRadius = 0.3;

    const auto rac = vehicle_models_params
                         ->mutable_trajectory_optimizer_vehicle_model_params()
                         ->add_circles();
    rac->set_dist_to_rac(half_width - vehicle_geo_params.back_edge_to_center());
    rac->set_angle_to_axis(0.0);
    rac->set_radius(half_width);
    rac->set_name("Rear axis center");

    const auto fac = vehicle_models_params
                         ->mutable_trajectory_optimizer_vehicle_model_params()
                         ->add_circles();
    fac->set_dist_to_rac(vehicle_geo_params.front_edge_to_center() -
                         half_width);
    fac->set_angle_to_axis(0.0);
    fac->set_radius(half_width);
    fac->set_name("Front axis center");

    constexpr double kLengthWidthRateThreshold = 3.0;

    if (vehicle_geo_params.length() / vehicle_geo_params.width() >
        kLengthWidthRateThreshold) {
      const auto mfac =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_circles();
      const double dist_to_rac1 =
          2.0 / 3.0 * (vehicle_geo_params.front_edge_to_center() - half_width) +
          1.0 / 3.0 * (half_width - vehicle_geo_params.back_edge_to_center());
      mfac->set_dist_to_rac(dist_to_rac1);
      mfac->set_angle_to_axis(0.0);
      mfac->set_radius(half_width);
      mfac->set_name("Middle Front axis center");
      const auto mrac =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_circles();
      const double dist_to_rac2 =
          1.0 / 3.0 * (vehicle_geo_params.front_edge_to_center() - half_width) +
          2.0 / 3.0 * (half_width - vehicle_geo_params.back_edge_to_center());
      mrac->set_dist_to_rac(dist_to_rac2);
      mrac->set_angle_to_axis(0.0);
      mrac->set_radius(half_width);
      mrac->set_name("Middle Rear axis center");
    } else {
      if (!vehicle_models_params->trajectory_optimizer_vehicle_model_params()
               .use_less_circles()) {
        const auto mac =
            vehicle_models_params
                ->mutable_trajectory_optimizer_vehicle_model_params()
                ->add_circles();
        mac->set_dist_to_rac(0.5 * (vehicle_geo_params.front_edge_to_center() -
                                    vehicle_geo_params.back_edge_to_center()));
        mac->set_angle_to_axis(0.0);
        mac->set_radius(half_width);
        mac->set_name("Middle axis center");
      }
    }

    if (!vehicle_models_params->trajectory_optimizer_vehicle_model_params()
             .use_less_circles()) {
      const double dist_to_front_corner =
          Hypot(half_width - kFrontCircleDefualtRadius,
                vehicle_geo_params.front_edge_to_center() -
                    kFrontCircleDefualtRadius);
      const double front_angle =
          std::atan2(half_width - kFrontCircleDefualtRadius,
                     vehicle_geo_params.front_edge_to_center() -
                         kFrontCircleDefualtRadius);

      const auto flc = vehicle_models_params
                           ->mutable_trajectory_optimizer_vehicle_model_params()
                           ->add_circles();
      flc->set_dist_to_rac(dist_to_front_corner);
      flc->set_angle_to_axis(front_angle);
      flc->set_radius(kFrontCircleDefualtRadius * 1.414);
      flc->set_name("Front left corner");

      const auto frc = vehicle_models_params
                           ->mutable_trajectory_optimizer_vehicle_model_params()
                           ->add_circles();
      frc->set_dist_to_rac(dist_to_front_corner);
      frc->set_angle_to_axis(-front_angle);
      frc->set_radius(kFrontCircleDefualtRadius * 1.414);
      frc->set_name("Front right corner");
    }

    if (vehicle_geo_params.has_left_mirror() &&
        vehicle_geo_params.has_right_mirror()) {
      const auto left_mirror_circle =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_mirror_circles();
      left_mirror_circle->set_dist_to_rac(
          Hypot(vehicle_geo_params.left_mirror().x(),
                vehicle_geo_params.left_mirror().y()));
      left_mirror_circle->set_angle_to_axis(
          std::atan2(vehicle_geo_params.left_mirror().y(),
                     vehicle_geo_params.left_mirror().x()));
      left_mirror_circle->set_radius(vehicle_geo_params.left_mirror().length() *
                                     0.5);
      left_mirror_circle->set_name("Left mirror");

      const auto right_mirror_circle =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_mirror_circles();
      right_mirror_circle->set_dist_to_rac(
          Hypot(vehicle_geo_params.right_mirror().x(),
                vehicle_geo_params.right_mirror().y()));
      right_mirror_circle->set_angle_to_axis(
          std::atan2(vehicle_geo_params.right_mirror().y(),
                     vehicle_geo_params.right_mirror().x()));
      right_mirror_circle->set_radius(
          vehicle_geo_params.right_mirror().length() * 0.5);
      right_mirror_circle->set_name("Right mirror");
    }
  }

  vehicle_models_params->set_is_vehicle_bus_model(false);
}

void FillAlccConfigurationMissingFieldsWithDefault(
    const PlannerParamsProto& default_planner_params,
    AlccTaskParamsProto* alcc_params) {
  FillInMissingFieldsWithDefault(default_planner_params.speed_planning_params(),
                                 alcc_params->mutable_speed_planning_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_params(),
      alcc_params->mutable_trajectory_optimizer_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.decision_constraint_config(),
      alcc_params->mutable_decision_constraint_config());
  FillInMissingFieldsWithDefault(default_planner_params.initialization_params(),
                                 alcc_params->mutable_initialization_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.spacetime_constraint_params(),
      alcc_params->mutable_spacetime_constraint_params());
  FillInMissingFieldsWithDefault(default_planner_params.vehicle_models_params(),
                                 alcc_params->mutable_vehicle_models_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.planner_functions_params(),
      alcc_params->mutable_planner_functions_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.spacetime_planner_object_trajectories_params(),
      alcc_params->mutable_spacetime_planner_object_trajectories_params());

  FillInMissingFieldsWithDefault(
      default_planner_params.speed_planning_lc_radical_params(),
      alcc_params->mutable_speed_planning_lc_radical_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.speed_planning_lc_conservative_params(),
      alcc_params->mutable_speed_planning_lc_conservative_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_radical_params(),
      alcc_params->mutable_trajectory_optimizer_lc_radical_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_normal_params(),
      alcc_params->mutable_trajectory_optimizer_lc_normal_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_conservative_params(),
      alcc_params->mutable_trajectory_optimizer_lc_conservative_params());
}

PlannerParamsProto CreateDefaultParam(
    const std::string& params_dir,
    const VehicleGeometryParamsProto& vehicle_geo_params) {
  PlannerParamsProto default_planner_params;
  CHECK(file_util::FileToProto(params_dir + "/planner_default_params.pb.txt",
                               &default_planner_params));

  SpeedPlanningParamsProto default_speed_planning_params;
  CHECK(
      file_util::FileToProto(params_dir + "/speed_planning_default_params.pb.txt",
                             &default_speed_planning_params));

  TrajectoryOptimizerParamsProto default_trajectory_optimizer_params;
  CHECK(file_util::FileToProto(
      params_dir + "/trajectory_optimizer_default_params.pb.txt",
      &default_trajectory_optimizer_params));

  FillInMissingFieldsWithDefault(
      default_speed_planning_params,
      default_planner_params.mutable_speed_planning_params());
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params.mutable_trajectory_optimizer_params());

  FillInMissingFieldsWithDefault(
      default_speed_planning_params,
      default_planner_params.mutable_fallback_planner_params()
          ->mutable_speed_planning_params());



  FillInMissingFieldsWithDefault(default_speed_planning_params,
                                 default_planner_params.mutable_acc_params()
                                     ->mutable_speed_planning_params());

  CHECK(file_util::FileToProto(
      params_dir + "/noa_req_params.pb.txt",
      default_planner_params.mutable_noa_params()->mutable_noa_req_params()));



  CHECK(file_util::FileToProto(
      params_dir + "/speed_planning_lc_radical_params.pb.txt",
      default_planner_params.mutable_speed_planning_lc_radical_params()));
  FillInMissingFieldsWithDefault(
      default_speed_planning_params,
      default_planner_params.mutable_speed_planning_lc_radical_params());

  CHECK(file_util::FileToProto(
      params_dir + "/speed_planning_lc_conservative_params.pb.txt",
      default_planner_params.mutable_speed_planning_lc_conservative_params()));
  FillInMissingFieldsWithDefault(
      default_speed_planning_params,
      default_planner_params.mutable_speed_planning_lc_conservative_params());

  CHECK(file_util::FileToProto(
      params_dir + "/trajectory_optimizer_lc_radical_params.pb.txt",
      default_planner_params.mutable_trajectory_optimizer_lc_radical_params()));
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params.mutable_trajectory_optimizer_lc_radical_params());

  CHECK(file_util::FileToProto(
      params_dir + "/trajectory_optimizer_lc_normal_params.pb.txt",
      default_planner_params.mutable_trajectory_optimizer_lc_normal_params()));
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params.mutable_trajectory_optimizer_lc_normal_params());

  CHECK(file_util::FileToProto(
      params_dir + "/trajectory_optimizer_lc_conservative_params.pb.txt",
      default_planner_params
          .mutable_trajectory_optimizer_lc_conservative_params()));
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params
          .mutable_trajectory_optimizer_lc_conservative_params());

  ComputeVehicleModelParamsOfPlanner(
      vehicle_geo_params,
      default_planner_params.mutable_vehicle_models_params());

  FillAlccConfigurationMissingFieldsWithDefault(
      default_planner_params, default_planner_params.mutable_alcc_params());
  return default_planner_params;
}

}  

absl::StatusOr<PlannerParamsProto> BuildPlannerParams(
    const std::string& params_dir,
    const VehicleGeometryParamsProto& vehicle_geo_params) {
  PlannerParamsProto run_planner_params;
  const bool load_file_success = file_util::FileToProto(
      params_dir + "/planner_default_params.pb.txt", &run_planner_params);

  if (!load_file_success) {
    return absl::NotFoundError(
        absl::StrCat("Cannot find param file in dir " + params_dir));
  }

  PlannerParamsProto planner_params;
  FillInMissingFieldsWithDefault(run_planner_params, &planner_params);

  const auto default_planner_params =
      CreateDefaultParam(params_dir, vehicle_geo_params);

  FillInMissingFieldsWithDefault(default_planner_params, &planner_params);

  RETURN_IF_ERROR(ValidateParams(planner_params));
  return planner_params;
}

}  
}  
