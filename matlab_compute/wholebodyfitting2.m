clc
clear

%% 拟合质心位置，转动惯量和杆长的函数
I = zeros(1, 116);
L = zeros(1, 116);
Lw = zeros(1, 116);
Lb = zeros(1, 116);

for angle4 = -15 : 1 : 100
    [ml, Il, L_, Lw_, Lb_] = GetLegBaryCenter(180 - angle4, angle4, 0);
    I(angle4 + 16) = Il;
    L(angle4 + 16) = L_;
    Lw(angle4 + 16) = Lw_;
    Lb(angle4 + 16) = Lb_;
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

%% 开始解算
% 一些变量
syms thetal_l(t) thetal_r(t) phi(t) s(t) yaw(t) x_l(t) x_r(t)
% 轮子参数
syms f_l f_r Nw_l Nw_r Pw_l Pw_r Tw_l Tw_r
% 腿部参数
syms L_l L_r
% 机体参数
syms Ib_x Ib_z Tb_l Tb_r Nb_l Nb_r Pb_l Pb_r
% 基本参数
% 整车参数
% 求解需要用到的
syms x_l_dot x_r_dot thetal_l_dot thetal_r_dot phi_dot s_dot yaw_dot
syms x_l_ddot x_r_ddot thetal_l_ddot thetal_r_ddot phi_ddot s_ddot yaw_ddot

% 需要定义的参数
mw = 1.267245;
R = 0.1;
Iw = 0.00379267;
mb = 5.4940204;
Ib_y = 0.05026821;
g = 9.81;
Ic_z = 0.393267165;
R_l = 0.482000001;
l = -0.01994485;

Il_l = KI(1, 1) * L_l + KI(1, 2);
Il_r = KI(1, 1) * L_r + KI(1, 2);
Lw_l = KLw(1, 1) * L_l + KLw(1, 2);
Lw_r = KLw(1, 1) * L_r + KLw(1, 2);
Lb_l = KLb(1, 1) * L_l + KLb(1, 2);
Lb_r = KLb(1, 1) * L_r + KLb(1, 2);

%% 轮子分析
xw_l = x_l;
xw_r = x_r;

% 左侧 水平
FrontW_l = mw * diff(diff(xw_l, t), t) == f_l - Nw_l;
% 左侧 转矩
TorqueW_l = Iw * diff(diff(xw_l, t), t) / R == Tw_l - f_l * R; 

% 右侧 水平
FrontW_r = mw * diff(diff(xw_r, t), t) == f_r - Nw_r;
% 右侧 转矩
TorqueW_r = Iw * diff(diff(xw_r, t), t) / R == Tw_r - f_r * R; 

%% 腿部分析
xl_l = x_l + Lw_l * sin(thetal_l);
xl_r = x_r + Lw_r * sin(thetal_r);
yl_l = Lw_l * cos(thetal_l);
yl_r = Lw_r * cos(thetal_r);

% 左侧 水平
FrontL_l = ml * diff(diff(xl_l, t), t) == Nw_l - Nb_l;
% 左侧 竖直
UpL_l = ml * diff(diff(yl_l, t), t) == Pw_l - Pb_l - ml * g;
% 左侧 转矩
TorqueL_l = Il_l * diff(diff(thetal_l, t), t) == Tb_l - Tw_l + (Pb_l * Lb_l + Pw_l * Lw_l) * sin(thetal_l) - (Nb_l * Lb_l + Nw_l * Lw_l) * cos(thetal_l);

% 右侧 水平
FrontL_r = ml * diff(diff(xl_r, t), t) == Nw_r - Nb_r;
% 右侧 竖直
UpL_r = ml * diff(diff(yl_r, t), t) == Pw_r - Pb_r - ml * g;
% 右侧 转矩
TorqueL_r = Il_r * diff(diff(thetal_r, t), t) == Tb_r - Tw_r + (Pb_r * Lb_r + Pw_r * Lw_r) * sin(thetal_r) - (Nb_r * Lb_r + Nw_r * Lw_r) * cos(thetal_r);

%% 机体分析
xb = 0.5 * (x_l + L_l * sin(thetal_l) + x_r + L_r * sin(thetal_r)) - l * sin(phi);
yb = 0.5 * (L_l * cos(thetal_l) + L_r * cos(thetal_r)) + l * cos(phi);

% 水平
FrontB = mb * diff(diff(xb, t), t) == Nb_l + Nb_r;
% 竖直
UpB = mb * diff(diff(yb, t), t) == Pb_l + Pb_r - mb * g;
% 转矩
TorqueB = Ib_y * diff(diff(phi, t), t) == (Tb_l + Tb_r) + (Nb_l + Nb_r) * l * cos(phi) + (Pb_l + Pb_r) * l * sin(phi);
% 假设机体两侧支持力一致
ForceEqual = Pw_l == Pw_r;

%% 整车分析
yaw_ = (x_r - x_l + L_r * sin(thetal_r) - L_l * sin(thetal_l)) / 2 / R_l;
WholeTurn = Ic_z * diff(diff(yaw_, t), t) == (f_r - f_l) * R_l;

%% 求解中间变量 Pw Pb Nw Nb f
func1 = [FrontW_l; FrontW_r; FrontL_l; FrontL_r; UpL_l; UpL_r; FrontB; UpB; ForceEqual; WholeTurn];

func1 = subs(func1, ...
    [diff(diff(x_l, t), t), diff(diff(x_r, t), t), diff(diff(thetal_l, t), t), diff(diff(thetal_r, t), t), diff(diff(phi, t), t), diff(x_l, t), diff(x_r, t), diff(thetal_l, t), diff(thetal_r, t), diff(phi, t)], ...
    [x_l_ddot, x_r_ddot, thetal_l_ddot, thetal_r_ddot, phi_ddot, x_l_dot, x_r_dot, thetal_l_dot, thetal_r_dot, phi_dot]);
[valPw_l, valPw_r, valNw_l, valNw_r, valPb_l, valPb_r, valNb_l, valNb_r, valf_l, valf_r] = solve(func1, [Pw_l, Pw_r, Nw_l, Nw_r, Pb_l, Pb_r, Nb_l, Nb_r, f_l, f_r]);

%% 求解变量
func2 = [TorqueW_l; TorqueW_r; TorqueL_l; TorqueL_r; TorqueB];

func2 = subs(func2, ...
    [Pw_l, Pw_r, Nw_l, Nw_r, Pb_l, Pb_r, Nb_l, Nb_r, f_l, f_r], ...
    [valPw_l, valPw_r, valNw_l, valNw_r, valPb_l, valPb_r, valNb_l, valNb_r, valf_l, valf_r]);
func2 = subs(func2, ...
    [diff(diff(x_l, t), t), diff(diff(x_r, t), t), diff(diff(thetal_l, t), t), diff(diff(thetal_r, t), t), diff(diff(phi, t), t), diff(x_l, t), diff(x_r, t), diff(thetal_l, t), diff(thetal_r, t), diff(phi, t)], ...
    [x_l_ddot, x_r_ddot, thetal_l_ddot, thetal_r_ddot, phi_ddot, x_l_dot, x_r_dot, thetal_l_dot, thetal_r_dot, phi_dot]);

[x_l_ddot, x_r_ddot, thetal_l_ddot, thetal_r_ddot, phi_ddot] = solve(func2, [x_l_ddot, x_r_ddot, thetal_l_ddot, thetal_r_ddot, phi_ddot]);

a_25 = 1 / 2 * (diff(x_l_ddot, thetal_l) + diff(x_r_ddot, thetal_l));
a_27 = 1 / 2 * (diff(x_l_ddot, thetal_r) + diff(x_r_ddot, thetal_r));
a_29 = 1 / 2 * (diff(x_l_ddot, phi) + diff(x_r_ddot, phi));

a_45 = 1 / (2 * R_l) * (-diff(x_l_ddot, thetal_l) + diff(x_r_ddot, thetal_l)) - L_l / (2 * R_l) * diff(thetal_l_ddot, thetal_l) + L_r / (2 * R_l) * diff(thetal_r_ddot, thetal_l);
a_47 = 1 / (2 * R_l) * (-diff(x_l_ddot, thetal_r) + diff(x_r_ddot, thetal_r)) - L_l / (2 * R_l) * diff(thetal_l_ddot, thetal_r) + L_r / (2 * R_l) * diff(thetal_r_ddot, thetal_r);
a_49 = 1 / (2 * R_l) * (-diff(x_l_ddot, phi) + diff(x_r_ddot, phi)) - L_l / (2 * R_l) * diff(thetal_l_ddot, phi) + L_r / (2 * R_l) * diff(thetal_r_ddot, phi);

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

b_21 = 1 / 2 * (diff(x_l_ddot, Tw_l) + diff(x_r_ddot, Tw_l));
b_22 = 1 / 2 * (diff(x_l_ddot, Tw_r) + diff(x_r_ddot, Tw_r));
b_23 = 1 / 2 * (diff(x_l_ddot, Tb_l) + diff(x_r_ddot, Tb_l));
b_24 = 1 / 2 * (diff(x_l_ddot, Tb_r) + diff(x_r_ddot, Tb_r));

b_41 = 1 / (2 * R_l) * (-diff(x_l_ddot, Tw_l) + diff(x_r_ddot, Tw_l)) - L_l / (2 * R_l) * diff(thetal_l_ddot, Tw_l) + L_r / (2 * R_l) * diff(thetal_r_ddot, Tw_l);
b_42 = 1 / (2 * R_l) * (-diff(x_l_ddot, Tw_r) + diff(x_r_ddot, Tw_r)) - L_l / (2 * R_l) * diff(thetal_l_ddot, Tw_r) + L_r / (2 * R_l) * diff(thetal_r_ddot, Tw_r);
b_43 = 1 / (2 * R_l) * (-diff(x_l_ddot, Tb_l) + diff(x_r_ddot, Tb_l)) - L_l / (2 * R_l) * diff(thetal_l_ddot, Tb_l) + L_r / (2 * R_l) * diff(thetal_r_ddot, Tb_l);
b_44 = 1 / (2 * R_l) * (-diff(x_l_ddot, Tb_r) + diff(x_r_ddot, Tb_r)) - L_l / (2 * R_l) * diff(thetal_l_ddot, Tb_r) + L_r / (2 * R_l) * diff(thetal_r_ddot, Tb_r);

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

A = subs(A, [x_l_dot, x_r_dot, thetal_l(t), thetal_l_dot, thetal_r(t), thetal_r_dot, phi(t), phi_dot, Tw_l, Tw_r, Tb_l, Tb_r], zeros(1, 12));
B = subs(B, [x_l_dot, x_r_dot, thetal_l(t), thetal_l_dot, thetal_r(t), thetal_r_dot, phi(t), phi_dot, Tw_l, Tw_r, Tb_l, Tb_r], zeros(1, 12));

%% 设定拟合次数，开始拟合
numsize = 29;
minleglen = 0.120;
maxleglen = 0.400;

K_vals = zeros(numsize * numsize, 4, 10);

L_lranges = linspace(minleglen, maxleglen, numsize);
L_rranges = linspace(minleglen, maxleglen, numsize);

L_lvals = zeros(numsize * numsize, 1);
L_rvals = zeros(numsize * numsize, 1);

a = 1;

for i = 1 : 1 : numsize
    for j = 1 : 1 : numsize

valL_l = L_lranges(i);
valL_r = L_rranges(j);

% s = 0.5 * (x_l + x_r);
% s_dot = 0.5 * (x_l_dot + x_r_dot);
% s_ddot = 0.5 * (x_l_ddot + x_r_ddot);
valA = subs(A, [L_l L_r], [valL_l valL_r]);
valB = subs(B, [L_l L_r], [valL_l valL_r]);

valA = double(valA);
valB = double(valB);

if(rank(ctrb(valA, valB)) == size(valA, 1))
    disp('系统可控')
else
    disp('系统不可控')
    K = 0;
    return
end

C = eye(10);
D = zeros(10,4);
Q = diag([60 40 60 40 10 1 10 1 120 10]);
R = diag([1 1 0.25 0.25]);
sys = ss(valA, valB, C, D);
K = lqr(sys, Q, R);
K_vals(a, : , :) = K;
L_lvals(a, 1) = valL_l;
L_rvals(a, 1) = valL_r;
disp(a);
a = a + 1;
    end
end

%% 参数引出处理
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
