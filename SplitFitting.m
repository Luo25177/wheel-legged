
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

K_vals = zeros(numsize, 2, 6);
L_ranges = linspace(0.183, 0.593, numsize);

for i = 1 : 1 : numsize

L = L_ranges(i);
Il = KI(1, 1) * L + KI(1, 2);
Lw = KLw(1, 1) * L + KLw(1, 2);
Lb = KLb(1, 1) * L + KLb(1, 2);

%% 需要定义的参数
syms x(t) theta(t) phi(t)
syms Tw Tb
syms x_ddot theta_ddot phi_ddot x_dot theta_dot phi_dot
mw = 0.88357;
R = 0.075;
Iw = 0.00249;
mb = 12.09048;
Ib_y = 0.20065;
g = 9.81;
Ic_z = 0.652;
R_l = 0.63;
l = 0;

Pw = mb * diff(diff(L * cos(theta) + l * cos(phi), t), t) + mb * g + ml * Lw * diff(diff(cos(theta), t), t) + ml * g;
Nw = ml * diff(diff(x + Lw * sin(theta), t), t) + mb * diff(diff(x + L * sin(theta) - l * sin(phi), t), t);
Pb = mb * diff(diff(L * cos(theta) + l * cos(phi), t), t) + mb * g;
Nb = mb * diff(diff(x + L * sin(theta) - l * sin(phi), t), t);

funcs = [x_ddot == (Tw * R - Nw * R * R) / (Iw + mw * R * R);
         Il * theta_ddot == Tb - Tw + (Pb * Lb + Pw * Lw) * sin(theta) - (Nb * Lb + Nw * Lw) * cos(theta);
         Ib_y * phi_ddot == Tb  + Nb * l * cos(phi) + Pb * l * sin(phi)];

funcs = subs(funcs, ...
            [diff(x, t), diff(theta, t), diff(phi, t), diff(diff(x, t), t), diff(diff(theta, t), t), diff(diff(phi, t), t)], ...
            [x_dot, theta_dot, phi_dot, x_ddot, theta_ddot, phi_ddot]);

[x_ddot, theta_ddot, phi_ddot] = solve(funcs, [x_ddot theta_ddot phi_ddot]);

X = [x, x_dot, theta, theta_dot, phi, phi_dot];
U = [Tw Tb];
X_dot = [x_dot, x_ddot, theta_dot, theta_ddot, phi_dot, phi_ddot];
A = jacobian(X_dot, X);
B = jacobian(X_dot, U);
A = subs(A, [x_dot, theta(t), theta_dot, phi(t), phi_dot, Tw, Tb], zeros(1,7));
B = subs(B, [x_dot, theta(t), theta_dot, phi(t), phi_dot, Tw, Tb], zeros(1,7));
A = double(A);
B = double(B);

if(rank(ctrb(A, B)) == size(A, 1))
    disp('系统可控')
else
    disp('系统不可控')
    K = 0;
    return
end

%% LQR
C = eye(6);
D = zeros(6,2);
Q = diag([100 100 100 10 5000 1]);
R = diag([1 0.25]);
sys = ss(A, B, C, D);
KLQR = lqr(sys, Q, R);%得到反馈增益矩阵
K_vals(i, :, :) = KLQR;
end

%% LQR 拟合:比较倾向于使用Curve Fitting Toolbox，简单好用
K11 = K_vals(:, 1, 1);
K12 = K_vals(:, 1, 2);
K13 = K_vals(:, 1, 3);
K14 = K_vals(:, 1, 4);
K15 = K_vals(:, 1, 5);
K16 = K_vals(:, 1, 6);

K21 = K_vals(:, 2, 1);
K22 = K_vals(:, 2, 2);
K23 = K_vals(:, 2, 3);
K24 = K_vals(:, 2, 4);
K25 = K_vals(:, 2, 5);
K26 = K_vals(:, 2, 6);

%% 状态反馈次优gamma-Hinfinty 控制器
B_1 = [0 0 0 0 0 0; 0 0 0 0 0 0];
B_2 = B;
C_1 = diag([1 1 1 1 1 1]);
D_11 = 0;
D_12 = [0 0 0 0 0 0; 0 0 0 0 0 0];
C_2 = diag([1 1 1 1 1 1]);
D_21 = [0 0 0 0 0 0; 0 0 0 0 0 0];
D_22 = [0 0 0 0 0 0; 0 0 0 0 0 0];
syssize = size(A, 1);
inputsize = size(B, 2);
gamma = 3.3;

setlmis([]);
Xh = lmivar(1, [syssize 1]);
Wh = lmivar(2, [inputsize syssize]);
lmiterm([1 1 1 Xh], A, 1, 's'); % AX+(AX)'
lmiterm([1 1 1 Wh], B_2, 1, 's'); %B_2W+(B_2W)'
lmiterm([1 2 1 0], B_1'); % B1'
lmiterm([1 2 2 0], -1); % -I
lmiterm([1 3 1 Xh], C_1, 1); % C1X
lmiterm([1 3 1 Wh], D_12, 1); % D12W
lmiterm([1 3 2 0], D_11); % D11
lmiterm([1 3 3 0], -1 * gamma ^ 2); % -γ^2I
lmiterm([-2 1 1 Xh], 1, 1); % X>0	特别注意不能漏掉

lmisys = getlmis;
[tmin, xfeas] = feasp(lmisys);
XX2 = dec2mat(lmisys, xfeas, Xh);
WW2 = dec2mat(lmisys, xfeas, Wh);
KHinfinty = WW2 * inv(XX2);


