#ifndef MATH_GEOMETRY_AFFINE_TRANSFORMATION_H_
#define MATH_GEOMETRY_AFFINE_TRANSFORMATION_H_

#include <vector>

#include "affine_transformation.pb.h"
#include "math/eigen.h"
#include "math/vec.h"

namespace e2e_noa {

class AffineTransformation {
 public:
  AffineTransformation();

  explicit AffineTransformation(const Mat4d& mat);

  AffineTransformation(const AffineTransformation& other);

  explicit AffineTransformation(const std::vector<double>& coeffs);

  explicit AffineTransformation(const TranslationProto& translation);
  explicit AffineTransformation(const RotationProto& rotation);
  explicit AffineTransformation(const ScalingProto& scaling);
  explicit AffineTransformation(const RigidTransformationProto& transformation);
  explicit AffineTransformation(
      const AffineTransformationProto& transformation);
  explicit AffineTransformation(
      const AffineTransformationSequenceProto& sequence);

  static Vec3d TranslationProtoToVec3d(const TranslationProto& translation);
  static void Vec3dToTranslationProto(const Vec3d& vec,
                                      TranslationProto* translation);

  static Quaternion RotationProtoToQuaternion(const RotationProto& rotation);
  static void QuaternionToRotationProto(Quaternion quaternion,
                                        RotationProto* rotation);

  static Vec3d ScalingProtoToVec3d(const ScalingProto& scaling);
  static void Vec3dToScalingProto(const Vec3d& vec, ScalingProto* scaling);

  static Quaternion YawPitchRollToQuaternion(const Vec3d& yaw_pitch_roll);
  static Vec3d QuaternionToYawPitchRoll(const Quaternion& quaternion);

  static AffineTransformation FromTranslation(double x, double y, double z);
  static AffineTransformation FromTranslation(const Vec3d& translation);

  static AffineTransformation FromRotation(double theta, double ax, double ay,
                                           double az);
  static AffineTransformation FromRotation(double theta, const Vec3d& axis);
  static AffineTransformation FromRotation(const Quaternion& quaternion);

  static AffineTransformation FromScaling(double s);
  static AffineTransformation FromScaling(double sx, double sy, double sz);
  static AffineTransformation FromScaling(const Vec3d& scaling);

  static AffineTransformation FromYawPitchRoll(const Vec3d& yaw_pitch_roll);
  static AffineTransformation FromYawPitchRoll(double yaw, double pitch,
                                               double roll);

  static AffineTransformation FromMat4d(const Mat4d& mat);
  static AffineTransformation FromTranslationProto(
      const TranslationProto& translation);
  static AffineTransformation FromRotationProto(const RotationProto& rotation);
  static AffineTransformation FromScalingProto(const ScalingProto& scaling);
  static AffineTransformation FromRigidTransformationProto(
      const RigidTransformationProto& transformation);
  static AffineTransformation FromAffineTransformationProto(
      const AffineTransformationProto& transformation);
  static AffineTransformation FromAffineTransformationSequenceProto(
      const AffineTransformationSequenceProto& sequence);

  template <typename ExtrinsicsType>
  static AffineTransformation FromExtrinsics(const ExtrinsicsType& extrinsics) {
    return AffineTransformation::FromTranslation(extrinsics.x(), extrinsics.y(),
                                                 extrinsics.z())
        .ApplyYawPitchRoll(extrinsics.yaw(), extrinsics.pitch(),
                           extrinsics.roll());
  }

  AffineTransformation& SetIdentity();

  AffineTransformation& SetTranslation(double x, double y, double z);
  AffineTransformation& SetTranslation(const Vec3d& translation);

  AffineTransformation& SetRotation(double theta, double ax, double ay,
                                    double az);
  AffineTransformation& SetRotation(double theta, const Vec3d& axis);
  AffineTransformation& SetRotation(Quaternion quaternion);

  AffineTransformation& SetScaling(double s);
  AffineTransformation& SetScaling(double sx, double sy, double sz);
  AffineTransformation& SetScaling(const Vec3d& scaling);

  AffineTransformation& SetYawPitchRoll(const Vec3d& yaw_pitch_roll);
  AffineTransformation& SetYawPitchRoll(double yaw, double pitch, double roll);

  AffineTransformation& Set(const Mat4d& mat);
  AffineTransformation& Set(const AffineTransformation& other);
  AffineTransformation& Set(const TranslationProto& translation);
  AffineTransformation& Set(const RotationProto& rotation);
  AffineTransformation& Set(const ScalingProto& scaling);
  AffineTransformation& Set(const RigidTransformationProto& transformation);
  AffineTransformation& Set(const AffineTransformationProto& transformation);
  AffineTransformation& Set(const AffineTransformationSequenceProto& sequence);

  AffineTransformation& ApplyTranslation(double x, double y, double z);
  AffineTransformation& ApplyTranslation(const Vec3d& translation);

  AffineTransformation& ApplyRotation(double theta, double ax, double ay,
                                      double az);
  AffineTransformation& ApplyRotation(double theta, const Vec3d& axis);
  AffineTransformation& ApplyRotation(const Quaternion& quaternion);

  AffineTransformation& ApplyScaling(double s);
  AffineTransformation& ApplyScaling(double sx, double sy, double sz);
  AffineTransformation& ApplyScaling(const Vec3d& scaling);

  AffineTransformation& ApplyYawPitchRoll(const Vec3d& yaw_pitch_roll);
  AffineTransformation& ApplyYawPitchRoll(double yaw, double pitch,
                                          double roll);

  AffineTransformation& Apply(const Mat4d& mat);
  AffineTransformation& Apply(const AffineTransformation& transformation);
  AffineTransformation& Apply(const TranslationProto& translation);
  AffineTransformation& Apply(const RotationProto& rotation);
  AffineTransformation& Apply(const ScalingProto& scaling);
  AffineTransformation& Apply(const RigidTransformationProto& transformation);
  AffineTransformation& Apply(const AffineTransformationProto& transformation);
  AffineTransformation& Apply(
      const AffineTransformationSequenceProto& sequence);

  void ToProto(AffineTransformationProto* transformation) const;

  AffineTransformation operator*(const Mat4d& mat) const {
    return AffineTransformation(mat_ * mat);
  }
  AffineTransformation operator*(
      const AffineTransformation& transformation) const {
    return AffineTransformation(mat_ * transformation.mat_);
  }

  AffineTransformation Inverse() const {
    return AffineTransformation(mat_.inverse());
  }

  const Mat4d& mat() const { return mat_; }

  Vec3d GetTranslation() const;
  void GetTranslationProto(TranslationProto* translation) const;

  Quaternion GetRotation() const;
  double GetRotationAngle(Vec3d* axis = nullptr) const;
  void GetRotationProto(RotationProto* rotation) const;
  Vec3d GetRotationYawPitchRoll() const;

  Vec3d GetScaling() const;
  void GetScalingProto(ScalingProto* scaling) const;

  void Get3x3SVD(Mat3d* u, Mat3d* v, Vec3d* s) const;

  Vec4d Transform(const Vec4d& x) const { return mat_ * x; }

  Vec3d TransformPoint(const Vec3d& x) const {
    return Vec3d(mat_.block<3, 3>(0, 0) * x + mat_.block<3, 1>(0, 3));
  }

  Vec3d TransformVector(const Vec3d& x) const {
    return Vec3d(mat_.block<3, 3>(0, 0) * x);
  }

  Vec3d TransformCovector(const Vec3d& x) const {
    return Vec3d(mat_.block<3, 3>(0, 0).transpose().fullPivLu().solve(x));
  }

 private:
  void GetRotation(double* theta, Vec3d* axis) const;

  Mat4d mat_;
};

}  // namespace e2e_noa

#endif
