/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <vector>

namespace zark {
namespace planning {
// create the quadratic programming proto
// 1/2 x^T Q x + c^T x
// w.r.t
// A x = b
// C x >= d
// specified input: input_marker
// as well as optimal solution optimal param

class QPMatrix {
 public:
  QPMatrix() : row_size_(0), col_size_(0), element_() {}

  int row_size() const { return row_size_; }
  void set_row_size(int row_size) { row_size_ = row_size; }

  int col_size() const { return col_size_; }
  void set_col_size(int col_size) { col_size_ = col_size; }

  const std::vector<double>& element() const { return element_; }
  std::vector<double>* mutable_element() { return &element_; }
  void set_element(std::vector<double> element) { element_ = element; }

 private:
  int row_size_;
  int col_size_;
  // element with element(col_size * r + c)
  std::vector<double> element_;
};

class QuadraticProgrammingProblem {
 public:
  QuadraticProgrammingProblem()
      : param_size_(0),
        quadratic_matrix_(),
        bias_(),
        equality_matrix_(),
        equality_value_(),
        inequality_matrix_(),
        inequality_value_(),
        input_marker_(),
        optimal_param_() {}

  int param_size() const { return param_size_; }
  void set_param_size(int param_size) { param_size_ = param_size; }

  const QPMatrix& quadratic_matrix() const { return quadratic_matrix_; }
  QPMatrix* mutable_quadratic_matrix() { return &quadratic_matrix_; }
  void set_quadratic_matrix(QPMatrix quadratic_matrix) {
    quadratic_matrix_ = quadratic_matrix_;
  }

  const std::vector<double>& bias() const { return bias_; }
  std::vector<double>* mutable_bias() { return &bias_; }
  void set_bias(std::vector<double> bais) { bias_ = bais; }

  const QPMatrix& equality_matrix() const { return equality_matrix_; }
  QPMatrix* mutable_equality_matrix() { return &equality_matrix_; }
  void set_equality_matrix(QPMatrix equality_matrix) {
    equality_matrix_ = equality_matrix;
  }

  const std::vector<double>& equality_value() const { return equality_value_; }
  std::vector<double>* mutable_equality_value() { return &equality_value_; }
  void set_equality_value(std::vector<double> equality_value) {
    equality_value_ = equality_value;
  }

  const QPMatrix& inequality_matrix() const { return inequality_matrix_; }
  QPMatrix* mutable_inequality_matrix() { return &inequality_matrix_; }
  void set_inequality_matrix(QPMatrix inequality_matrix) {
    inequality_matrix_ = inequality_matrix;
  }

  const std::vector<double>& inequality_value() const {
    return inequality_value_;
  }
  std::vector<double>* mutable_inequality_value() { return &inequality_value_; }
  void set_inequality_value(std::vector<double> inequality_value) {
    inequality_value_ = inequality_value;
  }

  const std::vector<double>& input_marker() const { return input_marker_; }
  void set_input_marker(std::vector<double> input_marker) {
    input_marker_ = input_marker;
  }

  const std::vector<double>& optimal_param() const { return optimal_param_; }
  std::vector<double>* mutable_optimal_param() { return &optimal_param_; }
  void set_optimal_param(std::vector<double> optimal_param) {
    optimal_param_ = optimal_param;
  }

 private:
  int param_size_;                        // specified parameter size
  QPMatrix quadratic_matrix_;             // Q matrix
  std::vector<double> bias_;              // c
  QPMatrix equality_matrix_;              // A matrix
  std::vector<double> equality_value_;    // b vector
  QPMatrix inequality_matrix_;            // C matrix
  std::vector<double> inequality_value_;  // d vector
  std::vector<double> input_marker_;      // marker for the specified matrix
  std::vector<double> optimal_param_;     // optimal result
};

class QuadraticProgrammingProblemSet {
 public:
  const std::vector<QuadraticProgrammingProblem>& problem() const {
    return problem_;
  }
  void set_problem(std::vector<QuadraticProgrammingProblem> problem) {
    problem_ = problem;
  }

 private:
  std::vector<QuadraticProgrammingProblem> problem_;  // QPProblem
};

}  // namespace planning
}  // namespace zark
