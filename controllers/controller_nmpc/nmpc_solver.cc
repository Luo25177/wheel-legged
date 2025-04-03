#include "nmpc_solver.hh"

NMPCSolver::NMPCSolver() {
}
NMPCSolver::~NMPCSolver() {
}

void NMPCSolver::setup(
  const int& _predict_n,
  const double& _predict_step,
  const int& _state_n,
  const int& _control_n,
  Function _dynamics,
  const DM& _Q,
  const DM& _R,
  const DM& _u_min,
  const DM& _u_max) {

  state_n = _state_n;
  control_n = _control_n;
  predict_n = _predict_n;
  predict_step = _predict_step;

  SX J = 0;
  int i;
  SX U = SX::sym("u", control_n, predict_n);
  SX X = SX::sym("x", state_n, predict_n + 1);
  SX x0 = SX::sym("x0", state_n);
  SX x_ref = SX::sym("x_ref", state_n);

  X(Slice(), 0) = x0;

  for (i = 0; i < predict_n; ++i) {
    SX k1 = _dynamics(SXDict{ {"x", X(Slice(), i)}, {"u", U(Slice(), i)} }).at("xdot");
    SX k2 = _dynamics(SXDict{ {"x", X(Slice(), i) + 0.5 * predict_step * k1}, {"u", U(Slice(), i)} }).at("xdot");
    SX k3 = _dynamics(SXDict{ {"x", X(Slice(), i) + 0.5 * predict_step * k2}, {"u", U(Slice(), i)} }).at("xdot");
    SX k4 = _dynamics(SXDict{ {"x", X(Slice(), i) + 0.5 * predict_step * k3}, {"u", U(Slice(), i)} }).at("xdot");
    X(Slice(), i + 1) = X(Slice(), i) + (predict_step / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
  }

  for (i = 0; i < predict_n; ++i)
    J += SX::dot((X(Slice(), i) - x_ref), SX::mtimes(_Q, (X(Slice(), i) - x_ref))) + 
      SX::dot(U(Slice(), i), SX::mtimes(_R, U(Slice(), i)));
  J += SX::mtimes((X(Slice(), i) - x_ref).T(), SX::mtimes(_Q, (X(Slice(), i) - x_ref))); 

  SXDict nlp = {
    {"x", reshape(U, -1, 1)},
    {"f", J},
    {"p", vertcat(x0, x_ref)}
  };

  Dict opts;
  opts["ipopt.max_iter"] = 5000;
  opts["print_time"] = 0;
  opts["ipopt.print_level"] = 0;
  opts["ipopt.acceptable_tol"] =  1e-6;
  opts["ipopt.acceptable_obj_change_tol"] = 1e-4;

  solver = nlpsol("solver", "ipopt", nlp, opts);

  SX lbu = repmat(_u_min, predict_n, 1);
  SX ubu = repmat(_u_max, predict_n, 1);

  args["lbx"] = lbu;
  args["ubx"] = ubu;
}

DM NMPCSolver::solve(const DM &x0, const DM &x_ref) {

  args["p"] = vertcat(x0, x_ref);
  args["x0"] = DM::zeros(control_n * predict_n, 1);

  DMDict res = solver(args);
  //std::cout << "----- 求解结果 -----" << std::endl;
  //std::cout << "最优解: " << res.at("x") << std::endl;
  //std::cout << "代价: " << res.at("f") << std::endl;

  return reshape(res.at("x"), control_n, predict_n);
}

//void NMPCSolver::setup(
//  const int& _predict_n,
//  const double& _predict_step,
//  const int& _state_n,
//  const int& _control_n,
//  SX(*_state_transition_function)(const SX&, const SX&),
//  DM _Q, DM _R) {
//
//  // 0-state_n: x_now
//  // state_n - state_n + _control_n : U_now
//  // state_n + _control_n - 2 * state_n + _control_n: x_ref
//  SX param = SX::sym("param", 2 * _state_n + _control_n);
//
//  SX x = SX::sym("X", _state_n, _predict_n + 1);
//  SX u = SX::sym("U", _control_n, _predict_n);
//
//  SX u_now = param(Slice(_state_n, _state_n + _control_n));
//  SX x_ref = param(Slice(_state_n + _control_n, _state_n + _state_n + _control_n));
//
//  x(Slice(), 0) = param(Slice(0, _state_n));
//  
//  for (int i = 0; i < _predict_n; i++)
//    x(Slice(), i + 1) = _state_transition_function(x(Slice(), i), u(Slice(), i)) * _predict_step + x(Slice(), i);
//
//  SX J = SX::sym("cost_func");
//  J = 0;
//
//  for (int i = 0; i < _predict_n; i++)
//    J += SX::mtimes({ (- x(Slice(), i) + x_ref).T(), _Q, (- x(Slice(), i) + x_ref) }) + 
//         SX::mtimes({ (u(Slice(), i)).T(), _R, (u(Slice(), i)) });
//
//  SX opt_var = SX::reshape(u.T(),-1,1);
//
//  SXDict nlp = { {"x", opt_var}, {"f", J}, {"p", param} };
//
//  Dict nlp_opts;
//  nlp_opts["expand"] = true;
//  nlp_opts["ipopt.max_iter"] = 5000;
//  nlp_opts["print_time"] = 0;
//  nlp_opts["ipopt.print_level"] = 0;
//  nlp_opts["ipopt.acceptable_tol"] =  1e-6;
//  nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;
//
//  try {
//      this->solver = nlpsol("solver", "ipopt", nlp, nlp_opts);
//  } catch (const casadi::CasadiException& e) {
//      std::cerr << "CasadiException: " << e.what() << std::endl;
//  }
//}

