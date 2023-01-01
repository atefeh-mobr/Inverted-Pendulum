T = 9;
T_F_c = tf([0 0 0 -0.000004875],[1 0.1173 0.0023 0])
T_F_d = c2d(T_F_c, T, 'zoh')

figure;
rlocus(T_F_d)

figure;
rlocus(-T_F_d)

figure;
rlocus(T_F_d)
zgrid()

figure;
margin(T_F_d)

