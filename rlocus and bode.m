T = 9;
T_F_c = tf([0 0 0 -0.000004875],[1 0.1173 0.0023 0])
T_F_d = c2d(T_F_c, T, 'zoh')

G_D = tf([0 -8.5297],[1 0.3394], -1)

figure;
rlocus(series(T_F_d, G_D))

figure;
rlocus(-series(T_F_d, G_D))

figure;
margin(series(T_F_d, G_D))

