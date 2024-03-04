function [A, B] = stateSpaceEquation()
% 一些变量
syms theta_l(t) theta_r(t) phi(t) x_l(t) x_r(t)
% 轮子参数
syms mw f_l f_r Nw_l Nw_r Pw_l Pw_r Iw R Tw_l Tw_r
% 腿部参数
syms ml_l ml_r Il_l Il_r Lw_l Lw_r Lb_l Lb_r L_l L_r
% 机体参数
syms mb Ib_y l Tb_l Tb_r Nb_l Nb_r Pb_l Pb_r
% 基本参数
syms g
% 整车参数
syms Ic_z R_l
% 求解需要用到的
syms x_l_dot x_r_dot theta_l_dot theta_r_dot phi_dot
syms x_l_ddot x_r_ddot theta_l_ddot theta_r_ddot phi_ddot

func1 = [mw * x_l_ddot == f_l - Nw_l;
    mw * x_r_ddot == f_r - Nw_r;
    ml_l * (x_l_ddot + Lw_l * theta_l_ddot) == Nw_l - Nb_l;
    Pw_l - Pb_l - ml_l * g == 0;
    ml_r * (x_r_ddot + Lw_r * theta_r_ddot) == Nw_r - Nb_r;
    Pw_r - Pb_r - ml_r * g == 0;
    mb * (0.5 * (x_l_ddot + L_l * theta_l_ddot + x_r_ddot + L_r * theta_r_ddot) - l * phi_ddot) == Nb_l + Nb_r;
    Pb_l + Pb_r - mb * g == 0;
    Pb_l == Pb_r;
    Ic_z * (x_r_ddot + L_r * theta_r_ddot - x_l_ddot - L_l * theta_l_ddot) / (2 * R_l) == (f_r - f_l) * R_l;
    ];

[Pw_l, Pw_r, Nw_l, Nw_r, Pb_l, Pb_r, Nb_l, Nb_r, f_l, f_r] = solve(func1, [Pw_l, Pw_r, Nw_l, Nw_r, Pb_l, Pb_r, Nb_l, Nb_r, f_l, f_r]);

func2 = [Iw * x_l_ddot / R == Tw_l - f_l * R;
    Iw * x_r_ddot / R == Tw_r - f_r * R;
    Il_l * theta_l_ddot == Tb_l - Tw_l + (Pb_l * Lb_l + Pw_l * Lw_l) * theta_l - (Nb_l * Lb_l + Nw_l * Lw_l);
    Il_r * theta_r_ddot == Tb_r - Tw_r + (Pb_r * Lb_r + Pw_r * Lw_r) * theta_r - (Nb_r * Lb_r + Nw_r * Lw_r);
    Ib_y * phi_ddot == (Tb_l + Tb_r) + (Nb_l + Nb_r) * l + (Pb_l + Pb_r) * l * phi;
    ];

[x_l_ddot, x_r_ddot, theta_l_ddot, theta_r_ddot, phi_ddot] = solve(func2, [x_l_ddot, x_r_ddot, theta_l_ddot, theta_r_ddot, phi_ddot]);

X_dot = [x_l_dot, x_l_ddot, x_r_dot, x_r_ddot, theta_l_dot, theta_l_ddot, theta_r_dot, theta_r_ddot, phi_dot, phi_ddot];
X = [x_l, x_l_dot, x_r, x_r_dot, theta_l, theta_l_dot, theta_r, theta_r_dot, phi, phi_dot];
U = [Tw_r, Tw_r, Tb_l, Tb_r];

A = jacobian(X_dot, X);
A = simplify(A);
B = jacobian(X_dot, U);
B = simplify(B);

end
