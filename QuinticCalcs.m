%{
Calculating Quintic scaling between non-zero positions and velocities and
zero accelerations
Last modified by Hannah Kolano 2/19/2021
%}

syms a0 a1 a2 a3 a4 a5 s t T th1 th2 dth1 dth2

s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
sd(t) = diff(s(t), t);
sdd(t) = diff(sd(t), t);

eqns = [s(0) == th1, s(T) == th2, sd(0) == dth1, sd(T) == dth2, sdd(0) == 0, sdd(T) ==0];
A = solve(eqns, [a0 a1 a2 a3 a4 a5]);

sSub = subs(s(t),[a0, a1, a2, a3, a4, a5], [A.a0, A.a1, A.a2, A.a3, A.a4, A.a5])
sdSub = subs(sd(t),[a1, a2, a3, a4, a5], [A.a1, A.a2, A.a3, A.a4, A.a5])