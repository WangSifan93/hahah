#ifndef CURVE1D_H_
#define CURVE1D_H_

#include <cstdint>
#include <string>

namespace e2e_noa {
namespace planning {
class Curve1d {
 public:
  Curve1d() = default;

  virtual ~Curve1d() = default;

  virtual double Evaluate(const std::uint32_t order,
                          const double param) const = 0;

  virtual double ParamLength() const = 0;

  virtual std::string ToString() const = 0;
};

}  // namespace planning
}  // namespace e2e_noa
#endif
