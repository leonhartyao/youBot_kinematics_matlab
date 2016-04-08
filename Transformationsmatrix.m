
clear all;
clc;


syms t1 t2 t3 t4 t5 d1 d5 a1 a2 a3;
theta = [t1; t2+0.5*pi; t3; t4-0.5*pi; t5];
d = [d1;0;0;0;d5];
a = [a1;a2;a3;0;0];
alpha = [0.5*pi;0;0;-0.5*pi;0];
T = cell(5,1);

for i = 1:5
    salpha = sin(alpha(i));
    if abs(salpha) < 1e-7
        salpha = 0;
    end
    calpha = cos(alpha(i));
    if abs(calpha) < 1e-7
        calpha = 0;
    end
    temp = [cos(theta(i)), -sin(theta(i))*calpha, sin(theta(i))*salpha, a(i)*cos(theta(i));...
            sin(theta(i)), cos(theta(i))*calpha, -cos(theta(i))*salpha, a(i)*sin(theta(i));...
            0, salpha, calpha, d(i);...
            0, 0, 0, 1];
    T{i} = simplify(temp);
end

T_10 = T{1};    
T_20 = simplify(T_10 * T{2});
T_30 = simplify(T_20 * T{3});
T_40 = simplify(T_30 * T{4});
T_50 = simplify(T_40 * T{5});

z0=[0;0;1];
p0=[0;0;0];
z1 = T_10(1:3,3);
p1 = T_10(1:3,4);
z2 = T_20(1:3,3);
p2 = T_20(1:3,4);
z3= T_30(1:3,3);
p3= T_30(1:3,4);
z4 = T_40(1:3,3);
p4 = T_40(1:3,4);
z5 = T_50(1:3,3);
p5 = T_50(1:3,4);

j1 = [cross(z0,(p5-p0));z0];
j2 = [cross(z1,(p5-p1));z1];
j3 = [cross(z2,(p5-p2));z2];
j4 = [cross(z3,(p5-p3));z3];
j5 = [cross(z4,(p5-p4));z4];