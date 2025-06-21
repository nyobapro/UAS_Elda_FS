% Define constants (same as current loop)
Vin = 16.8;
L = 0.0029166667;
C = 3.125e-06;
R = 10;
Ksen_i = 3.3/47;
Ksen_v = 3.3/50;
Kpwm = Vin;
fx_i = 2e3;
fx_v = 500; % Voltage loop crossover freq (Hz)

% Define s
s = tf('s');

% Inner loop design
Hi = 1 / (L*s + R / (1 + R*C*s));
Ti_target_i = 2*pi*fx_i;
[C_pi, info_i] = pidtune(Hi * Ksen_i * Kpwm, 'PI', Ti_target_i);
Gi = C_pi;
Ti_loop = Hi * Gi * Ksen_i * Kpwm;
Tic_loop = feedback(Ti_loop, 1) / Ksen_i;

% Voltage plant: Hv(s) = Vo / iL = R / (RCs + 1)
Hv = R / (R*C*s + 1);

% Outer loop tuning
Ti_target_v = 2*pi*fx_v;
opts = pidtuneOptions(PhaseMargin=60);  % Desired PM
[C_pv, info_v] = pidtune(Hv * Tic_loop * Ksen_v, 'PI', Ti_target_v, opts);
Gv = C_pv;

% Voltage loop open-loop and closed-loop TFs
Tv_loop = Hv * Gv * Ksen_v * Tic_loop;
Tvo_cl = feedback(Tv_loop, 1);

% Display tuned gains
Kpv = C_pv.Kp;
Kiv = C_pv.Ki;
disp("Outer Loop PI Gains:");
fprintf("Kp_v = %.5f, Ki_v = %.5f\n", Kpv, Kiv);

% Bode and Step plots
figure;
margin(Tv_loop);
title('Voltage Loop Open-Loop Bode Plot');

figure;
step(Tvo_cl);
grid on;
title('Voltage Loop Closed-Loop Step Response');
ylabel('V_o / V_{ref}');
