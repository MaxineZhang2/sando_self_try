#pragma once

// Small compatibility layer used by the existing SANDO model builder.
// It keeps the old, local GRB* names in the solver implementation while
// routing all model construction and solution calls to COPT.

#include <string>
#include <vector>

#include "coptcpp_pch.h"

using GRBVar = Var;
using GRBLinExpr = Expr;
using GRBQuadExpr = QuadExpr;
using GRBConstr = Constraint;
using GRBQConstr = QConstraint;
using GRBGenConstr = GenConstr;

using GRBException = CoptException;

// The source code uses these names as opaque attribute/parameter tokens.
enum CoptCompatToken {
  GRB_DoubleAttr_X,
  GRB_DoubleAttr_Runtime,
  GRB_DoubleAttr_ObjVal,
  GRB_IntAttr_NumVars,
  GRB_IntAttr_Status,
  GRB_StringAttr_ModelName,
  GRB_IntParam_OutputFlag,
  GRB_IntParam_LogToConsole,
  GRB_IntParam_Threads,
  GRB_DoubleParam_TimeLimit,
};

constexpr double GRB_INFINITY = COPT_INFINITY;
constexpr char GRB_CONTINUOUS = COPT_CONTINUOUS;
constexpr char GRB_BINARY = COPT_BINARY;
constexpr char GRB_LESS_EQUAL = COPT_LESS_EQUAL;
constexpr int GRB_MINIMIZE = COPT_MINIMIZE;

// Status values are intentionally aliases, because COPT uses the same
// semantic states required by SANDO (optimal, infeasible, unbounded, etc.).
constexpr int GRB_OPTIMAL = COPT_STATUS_OPTIMAL;
constexpr int GRB_INFEASIBLE = COPT_STATUS_INFEASIBLE;
constexpr int GRB_UNBOUNDED = COPT_STATUS_UNBOUNDED;
constexpr int GRB_INF_OR_UNBD = COPT_STATUS_INF_OR_UNB;
constexpr int GRB_NUMERIC = COPT_STATUS_NUMERICAL;
constexpr int GRB_INTERRUPTED = COPT_STATUS_INTERRUPTED;

class GRBEnv {
 public:
  explicit GRBEnv(bool /*empty*/ = false) {}

  void set(int token, int value) {
    (void)token;
    (void)value;
  }

  void start() {}

  Envr env_;
};

class GRBModel {
 public:
  explicit GRBModel(GRBEnv& env) : model_(env.env_.CreateModel("planning")) {}

  void set(int token, int value) {
    if (token == GRB_IntParam_OutputFlag || token == GRB_IntParam_LogToConsole) {
      model_.SetIntParam(COPT_INTPARAM_LOGTOCONSOLE, value);
    } else if (token == GRB_IntParam_Threads) {
      model_.SetIntParam(COPT_INTPARAM_THREADS, value);
    }
  }

  void set(int token, double value) {
    if (token == GRB_DoubleParam_TimeLimit)
      model_.SetDblParam(COPT_DBLPARAM_TIMELIMIT, value);
  }

  void set(int /*token*/, const std::string& /*value*/) {}
  template <typename Callback>
  void setCallback(Callback* /*callback*/) {}

  template <typename Builder>
  GRBConstr addConstr(const Builder& builder, const std::string& name = {}) {
    return model_.AddConstr(builder, name.c_str());
  }

  template <typename Builder>
  GRBQConstr addQConstr(const Builder& builder, const std::string& name = {}) {
    return model_.AddQConstr(builder, name.c_str());
  }

  GRBGenConstr addGenConstrIndicator(const GRBVar& bin_var, int bin_value,
                                     const GRBLinExpr& expr, char sense, double rhs,
                                     const std::string& name = {}) {
    return model_.AddGenConstrIndicator(bin_var, bin_value, expr, sense, rhs,
                                        COPT_INDICATOR_IF, name.c_str());
  }

  GRBVar addVar(double lb, double ub, double obj, char type, const std::string& name = {}) {
    return model_.AddVar(lb, ub, obj, type, name.c_str());
  }

  void remove(GRBVar& var) { var.Remove(); }
  void remove(GRBConstr& constr) { constr.Remove(); }
  void remove(GRBQConstr& constr) { constr.Remove(); }
  void remove(GRBGenConstr& constr) { constr.Remove(); }

  void setObjective(const GRBQuadExpr& objective, int sense) {
    model_.SetQuadObjective(objective, sense);
  }

  void optimize() { model_.Solve(); }
  void interrupt() { model_.Interrupt(); }

  std::vector<GRBVar> getVars() {
    const VarArray vars = model_.GetVars();
    std::vector<GRBVar> result;
    result.reserve(static_cast<size_t>(vars.Size()));
    for (int i = 0; i < vars.Size(); ++i) result.push_back(vars.GetVar(i));
    return result;
  }

  struct AttributeValue {
    double value{0.0};
    operator int() const { return static_cast<int>(value); }
    operator double() const { return value; }
  };

  AttributeValue get(int token) const {
    if (token == GRB_IntAttr_NumVars)
      return {static_cast<double>(model_.GetIntAttr(COPT_INTATTR_COLS))};
    if (token == GRB_IntAttr_Status)
      return {static_cast<double>(model_.GetIntAttr(COPT_INTATTR_STATUS))};
    if (token == GRB_DoubleAttr_Runtime)
      return {model_.GetDblAttr(COPT_DBLATTR_SOLVINGTIME)};
    if (token == GRB_DoubleAttr_ObjVal)
      return {model_.GetDblAttr(COPT_DBLATTR_OBJVAL)};
    return {};
  }

 private:
  Model model_;
};
