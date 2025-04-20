
% 拟合质心位置，转动惯量和杆长的函数
I = zeros(1, 90);
L = zeros(1, 90);
Lw = zeros(1, 90);
Lb = zeros(1, 90);

for angle4 = 0 : 1 : 89
    [ml, Il, L_, Lw_, Lb_] = GetLegBaryCenter(180 - angle4, angle4, 0);
    I(angle4 + 1) = Il;
    L(angle4 + 1) = L_;
    Lw(angle4 + 1) = Lw_;
    Lb(angle4 + 1) = Lb_;
end

KI = polyfit(L, I, 1);
valKI = polyval(KI,L);
figure(1);hold on;plot(L,I,'r*',L,valKI,'b-.');

KLw = polyfit(L, Lw, 1);
valKLw = polyval(KLw,L);
figure(1);hold on;plot(L,Lw,'r*',L,valKLw,'b-.');


KLb = polyfit(L, Lb, 1);
valKLb = polyval(KLb,L);
figure(1);hold on;plot(L,Lb,'r*',L,valKLb,'b-.');

numsize = 20;

K_vals = zeros(numsize * numsize, 4, 10);

L_lranges = linspace(0.183, 0.593, numsize);
L_rranges = linspace(0.183, 0.593, numsize);

L_lvals = zeros(numsize * numsize, 1);
L_rvals = zeros(numsize * numsize, 1);

a = 1;
for i = 1 : 1 : numsize
    for j = 1 : 1 : numsize
% 一些变量
syms thetal_l(t) thetal_r(t) phi(t) s(t) yaw(t) thetaw_l(t) thetaw_r(t)
% 轮子参数
syms f_l f_r Nw_l Nw_r Pw_l Pw_r Tw_l Tw_r
% 腿部参数
% 机体参数
syms Tb_l Tb_r Nb_l Nb_r Pb_l Pb_r
% 基本参数
syms t
% 整车参数
% 求解需要用到的
syms thetaw_l_dot thetaw_r_dot thetal_l_dot thetal_r_dot phi_dot s_dot yaw_dot
syms thetaw_l_ddot thetaw_r_ddot thetal_l_ddot thetal_r_ddot phi_ddot s_ddot yaw_ddot

%% 需要定义的参数
mw = 0.88357;
R = 0.075;
Iw = 0.00249;
mb = 12.09048;
Ib_y = 0.20065;
g = 9.81;
Ic_z = 0.652;
R_l = 0.63;
l = 0;
L_l = L_lranges(i);
L_r = L_rranges(j);
Il_l = KI(1, 1) * L_l + KI(1, 2);
Il_r = KI(1, 1) * L_r + KI(1, 2);
Lw_l = KLw(1, 1) * L_l + KLw(1, 2);
Lw_r = KLw(1, 1) * L_r + KLw(1, 2);
Lb_l = KLb(1, 1) * L_l + KLb(1, 2);
Lb_r = KLb(1, 1) * L_r + KLb(1, 2);

equation1 = (Iw * L_l / R + mw * R * L_l + ml * R * Lb_l) * thetaw_l_ddot + (ml * Lw_l * Lb_l - Il_l) * thetal_l_ddot + (ml * Lw_l + 0.5 * mb * L_l) * g * thetal_l + Tb_l - Tw_l * (1 + L_l / R);
equation2 = (Iw * L_r / R + mw * R * L_r + ml * R * Lb_r) * thetaw_r_ddot + (ml * Lw_r * Lb_r - Il_r) * thetal_r_ddot + (ml * Lw_r + 0.5 * mb * L_r) * g * thetal_r + Tb_r - Tw_r * (1 + L_r / R);
equation3 = -(mw * R * R + Iw + ml * R * R + 0.5 * mb * R * R) * thetaw_l_ddot - (mw * R * R + Iw + ml * R * R + 0.5 * mb * R * R) * thetaw_r_ddot - (ml * R * Lw_l + 0.5 * mb * R * L_l) * thetal_l_ddot - (ml * R * Lw_r + 0.5 * mb * R * L_r) * thetal_r_ddot + Tw_l + Tw_r;
equation4 = (mw * R * l + Iw * l / R + ml * R * l) * thetaw_l_ddot + (mw * R * l + Iw * l / R + ml * R * l) * thetaw_r_ddot + ml * Lw_l * l * thetal_l_ddot + ml * Lw_r * l * thetal_r_ddot - Ib_y * phi_ddot + mb * g * l * phi - (Tw_l + Tw_r) * l / R - (Tb_l + Tb_r);
equation5 = (0.5 * Ic_z * R / R_l + Iw * R_l / R) * thetaw_l_ddot - (0.5 * Ic_z * R / R_l + Iw * R_l / R) * thetaw_r_ddot + 0.5 * Ic_z * L_l / R_l * thetal_l_ddot - 0.5 * Ic_z * L_r / R_l * thetal_r_ddot - Tw_l * R_l / R + Tw_r * R_l / R;

func = [equation1 == 0;equation2 == 0;equation3 == 0;equation4 == 0;equation5 == 0];

[thetaw_l_ddot, thetaw_r_ddot, thetal_l_ddot, thetal_r_ddot, phi_ddot] = solve(func, [thetaw_l_ddot, thetaw_r_ddot, thetal_l_ddot, thetal_r_ddot, phi_ddot]);

a_25 = R / 2 * (diff(thetaw_l_ddot, thetal_l) + diff(thetaw_r_ddot, thetal_l));
a_27 = R / 2 * (diff(thetaw_l_ddot, thetal_r) + diff(thetaw_r_ddot, thetal_r));
a_29 = R / 2 * (diff(thetaw_l_ddot, phi) + diff(thetaw_r_ddot, phi));

a_45 = R / (2 * R_l) * (-diff(thetaw_l_ddot, thetal_l) + diff(thetaw_r_ddot, thetal_l)) - L_l / (2 * R_l) * diff(thetal_l_ddot, thetal_l) + L_r / (2 * R_l) * diff(thetal_r_ddot, thetal_l);
a_47 = R / (2 * R_l) * (-diff(thetaw_l_ddot, thetal_r) + diff(thetaw_r_ddot, thetal_r)) - L_l / (2 * R_l) * diff(thetal_l_ddot, thetal_r) + L_r / (2 * R_l) * diff(thetal_r_ddot, thetal_r);
a_49 = R / (2 * R_l) * (-diff(thetaw_l_ddot, phi) + diff(thetaw_r_ddot, phi)) - L_l / (2 * R_l) * diff(thetal_l_ddot, phi) + L_r / (2 * R_l) * diff(thetal_r_ddot, phi);

a_65 = diff(thetal_l_ddot, thetal_l);
a_67 = diff(thetal_l_ddot, thetal_r);
a_69 = diff(thetal_l_ddot, phi);

a_85 = diff(thetal_r_ddot, thetal_l);
a_87 = diff(thetal_r_ddot, thetal_r);
a_89 = diff(thetal_r_ddot, phi);

a_x5 = diff(phi_ddot, thetal_l);
a_x7 = diff(phi_ddot, thetal_r);
a_x9 = diff(phi_ddot, phi);

A = [0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 a_25 0 a_27 0 a_29 0;
    0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 a_45 0 a_47 0 a_49 0;
    0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 a_65 0 a_67 0 a_69 0;
    0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 a_85 0 a_87 0 a_89 0;
    0 0 0 0 0 0 0 0 0 1;
    0 0 0 0 a_x5 0 a_x7 0 a_x9 0;
    ];

b_21 = R / 2 * (diff(thetaw_l_ddot, Tw_l) + diff(thetaw_r_ddot, Tw_l));
b_22 = R / 2 * (diff(thetaw_l_ddot, Tw_r) + diff(thetaw_r_ddot, Tw_r));
b_23 = R / 2 * (diff(thetaw_l_ddot, Tb_l) + diff(thetaw_r_ddot, Tb_l));
b_24 = R / 2 * (diff(thetaw_l_ddot, Tb_r) + diff(thetaw_r_ddot, Tb_r));

b_41 = R / (2 * R_l) * (-diff(thetaw_l_ddot, Tw_l) + diff(thetaw_r_ddot, Tw_l)) - L_l / (2 * R_l) * diff(thetal_l_ddot, Tw_l) + L_r / (2 * R_l) * diff(thetal_r_ddot, Tw_l);
b_42 = R / (2 * R_l) * (-diff(thetaw_l_ddot, Tw_r) + diff(thetaw_r_ddot, Tw_r)) - L_l / (2 * R_l) * diff(thetal_l_ddot, Tw_r) + L_r / (2 * R_l) * diff(thetal_r_ddot, Tw_r);
b_43 = R / (2 * R_l) * (-diff(thetaw_l_ddot, Tb_l) + diff(thetaw_r_ddot, Tb_l)) - L_l / (2 * R_l) * diff(thetal_l_ddot, Tb_l) + L_r / (2 * R_l) * diff(thetal_r_ddot, Tb_l);
b_44 = R / (2 * R_l) * (-diff(thetaw_l_ddot, Tb_r) + diff(thetaw_r_ddot, Tb_r)) - L_l / (2 * R_l) * diff(thetal_l_ddot, Tb_r) + L_r / (2 * R_l) * diff(thetal_r_ddot, Tb_r);

b_61 = diff(thetal_l_ddot, Tw_l);
b_62 = diff(thetal_l_ddot, Tw_r);
b_63 = diff(thetal_l_ddot, Tb_l);
b_64 = diff(thetal_l_ddot, Tb_r);

b_81 = diff(thetal_r_ddot, Tw_l);
b_82 = diff(thetal_r_ddot, Tw_r);
b_83 = diff(thetal_r_ddot, Tb_l);
b_84 = diff(thetal_r_ddot, Tb_r);

b_x1 = diff(phi_ddot, Tw_l);
b_x2 = diff(phi_ddot, Tw_r);
b_x3 = diff(phi_ddot, Tb_l);
b_x4 = diff(phi_ddot, Tb_r);

B = [0 0 0 0;
    b_21 b_22 b_23 b_24;
    0 0 0 0;
    b_41 b_42 b_43 b_44;
    0 0 0 0;
    b_61 b_62 b_63 b_64;
    0 0 0 0;
    b_81 b_82 b_83 b_84;
    0 0 0 0;
    b_x1 b_x2 b_x3 b_x4;
    ];

A = double(A);
B = double(B);

if(rank(ctrb(A, B)) == size(A, 1))
    disp('系统可控')
else
    disp('系统不可控')
    K = 0;
    return
end

C = eye(10);
D = zeros(10,4);
Q = diag([10 1 10 1 10 10 10 10 100 1]);
R = diag([1 1 0.25 0.25]);
sys = ss(A, B, C, D);
K = lqr(sys, Q, R);
K_vals(a, : , :) = K;
L_lvals(a, 1) = L_l;
L_rvals(a, 1) = L_r;
disp(a);
a = a + 1;
    end
end

K11 = K_vals(:, 1, 1);
K12 = K_vals(:, 1, 2);
K13 = K_vals(:, 1, 3);
K14 = K_vals(:, 1, 4);
K15 = K_vals(:, 1, 5);
K16 = K_vals(:, 1, 6);
K17 = K_vals(:, 1, 7);
K18 = K_vals(:, 1, 8);
K19 = K_vals(:, 1, 9);
K1x = K_vals(:, 1, 10);

K21 = K_vals(:, 2, 1);
K22 = K_vals(:, 2, 2);
K23 = K_vals(:, 2, 3);
K24 = K_vals(:, 2, 4);
K25 = K_vals(:, 2, 5);
K26 = K_vals(:, 2, 6);
K27 = K_vals(:, 2, 7);
K28 = K_vals(:, 2, 8);
K29 = K_vals(:, 2, 9);
K2x = K_vals(:, 2, 10);

K31 = K_vals(:, 3, 1);
K32 = K_vals(:, 3, 2);
K33 = K_vals(:, 3, 3);
K34 = K_vals(:, 3, 4);
K35 = K_vals(:, 3, 5);
K36 = K_vals(:, 3, 6);
K37 = K_vals(:, 3, 7);
K38 = K_vals(:, 3, 8);
K39 = K_vals(:, 3, 9);
K3x = K_vals(:, 3, 10);

K41 = K_vals(:, 4, 1);
K42 = K_vals(:, 4, 2);
K43 = K_vals(:, 4, 3);
K44 = K_vals(:, 4, 4);
K45 = K_vals(:, 4, 5);
K46 = K_vals(:, 4, 6);
K47 = K_vals(:, 4, 7);
K48 = K_vals(:, 4, 8);
K49 = K_vals(:, 4, 9);
K4x = K_vals(:, 4, 10);
