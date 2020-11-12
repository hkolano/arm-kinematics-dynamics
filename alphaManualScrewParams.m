%% Twist Form
w_1 = [0 0 -1];
v_1 = [0 0 0];
TW1 = Twist('R', w_1, v_1); Screw1 = TW1.S;
w_2 = [0 1 0];
v_2 = [-20 0 46.2];
TW2 = Twist('R', w_2, v_2); Screw2 = TW2.S;
w_3 = [0 -1 0];
v_3 = [-60 0 -99.1];
TW3 = Twist('R', w_3, v_3); Screw3 = TW3.S;
w_4 = [0 0 -1];
v_4 = [-80 0 -99.1];
TW4 = Twist('R', w_4, v_4); Screw4 = TW4.S;
w_5 = [-1 0 0];
v_5 = [-80 0 80.895];
TW5 = Twist('R', w_5, v_5); Screw5 = TW5.S;

Slist = [Screw1, Screw2, Screw3, Screw4, Screw5]

PoEFkine0 = prod([TW1.exp(0) TW2.exp(0) TW3.exp(0) TW4.exp(0) TW5.exp(0) M_home]);
PoEFkine1 = prod([TW1.exp(0) TW2.exp(10*pi/180) TW3.exp(0) TW4.exp(0) TW5.exp(0) M_home])