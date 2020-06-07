% --------------------------------------------------|
% Nowoczesne Metody Teorii Sterowania Projekt 1     |
% Micha³ Chruœciñski    149741                      |
% Adam Krempczyñski     165250                      |
% --------------------------------------------------|

% Clear workspace
clear;

% *** Plant definition in state space ***
% Transfer function (G) creation 
h11 = tf([0 0 1], [1 12 32]);
h12 = tf([0 1 0 1], [1 6 11 6]);
h21 = tf([0 0 1 2], [1 10 29 20]);
h22 = tf([0 1], [1 3]);

G = [h11 h12;
     h21 h22];

% Create matrices for state space model equivallent to 
% transfer function G derivation of this model is in 
% attached images
A = [-4, 0, 0, 0, 0, 0, 0;
     0, -3, 0, 0, 0, 0, 0;
     1, 0, -8, 0, 0, 0, 0;
     0, 1, 0, -3, -2, 0, 0;
     0, 0, 0, 1, 0, 0, 0;
     1, 0, 0, 0, 0, -6, -5;
     0, 0, 0, 0, 0, 1, 0];
 B = [1, 0;
     0, 1;
     0, 0;
     0, 0;
     0, 0;
     0, 0;
     0, 0];
 C = [0 1 1 -3 -1 0 0;
      0 1 0 0 0 1 2];
 D = [0 0; 0 0];
 
 % Create base system space state representation
 sys = ss(A, B, C, D);
 
 % Simulate and plot step response of the base system
 [otp1, t1] = step(sys);
 plot_step(otp1, t1, "Base system simulation using state space model");
 
 % *** State feedback controller design ***
 % Choose poles of the system with state feedback
 p = [-1 -1.23 -5.0 -2, -4, -8, -3];
 
 % Calculate gain matrix 
 K = place(A, B, p);
 
 % Calculate new A matrix
 Astar = A - B*K;
 
 % Create closed loop system space state representation and plot step response
 SYScl = ss(Astar, B, C, D);
 
 % Simulate and plot step response of the closed loop system
 [otp, t2] = step(SYScl);
 plot_step(otp, t2, "Closed loop only system simulation using state space model");
 
 % *** Controller with integral action to eliminate steady-state error ***
 % Closed loop with integrator
 % Create new matrices Al and Bl to calculate state's gains and integration gains
 Al = [A, [0; 0; 0; 0; 0; 0; 0], [0; 0; 0; 0; 0; 0; 0];
     -C, [0; 0], [0; 0]];
 Bl = [B; [0; 0], [0; 0]];
 
 % Choose desired new system's poles in that case the same poles 
 % as in closed loop system with 2 additional poles introduced by
 % 2 added integrators, all poles with Re < 0 
 poles = [p, -2.4, -4.2];
 
 % Check controllability of the Al and Bl pair. 
 % It's needed to calculate gains in the usual way
 ControllabilityMatrix = ctrb(Al, Bl);
 
 if rank(ControllabilityMatrix) ~= 9
    disp("System described by matrices Al, Bl is not controllable. Good bye.")
    return
 end
     
disp("System described by matrices Al, Bl is controllable.")

% Calculate gain matrix the usual way
K2 = place(Al, Bl, poles);

% State's gain matrix
Kloop = K2(:, 1:7);

% Integrators gain matrix
Ki = -1 * K2(:, 8:9);

% Partial result for new system with state-feedback controller with
% integrators 
GKe = B*Ki;
FGK = A - B*Kloop;

% New A matrix
Astar2 = [FGK, GKe;
       -C, [0; 0], [0; 0]];

% New C matrix  
Cstar = [C, [0; 0], [0; 0]];

% New B matrix
Bstar = zeros(7, 2);
Bstar = [Bstar; [1, 0]; [0, 1]];

% Create a new system with state-feedback controller with integrators
sysClosedLoopIntegr = ss(Astar2, Bstar, Cstar, D);

% Simulate and plot step response of the new system
[otpa, t3] = step(sysClosedLoopIntegr);
plot_step(otpa, t3, "Closed loop with integrators system simulation using state space model");

% *** OBSERVER DESIGN ***
% Verify observability
Mo = obsv(A, C);
if rank(Mo) ~= 7
    disp("System described by matrices A, C is not observable. Good bye.")
    return
end
disp("System described by matrices A, C is observable.")

% Observer poles will be 2x faster than controller poles
obsrv_poles = 2*p;
% Calculate observer gain
L_T = place(A', C', obsrv_poles);
% Need to transpose L matrix
L = L_T';

% Now observer + controller
% Observer + controller (only gain, no integral action)
A_obsv_ctr = [A-B*K, B*K;
              zeros(7)       , A-L*C];
B_obsv_ctr = [B; zeros(7, 2)];
C_obsv_ctr = [C, zeros(2,7)];
D_obsv_ctr = zeros(2, 2);


SYSobsv = ss(A_obsv_ctr, B_obsv_ctr, C_obsv_ctr, D_obsv_ctr);

% Step response of the system with zero initial conditions
[otp, t4] = step(SYSobsv);
plot_step(otp, t4, "Observer + gain controller, zero initial conditions");

% Step response with initial conditions
x0 = [3, 4, 2, 1, 8, 12, 3, 0, 0, 0, 0, 0, 0, 0];
[y,t5] = lsim(SYSobsv, ones(length(t2),2), t2, x0);

Y1 = y(:, 1);
Y2 = y(:, 2);

figure;
subplot(2, 1, 1);
plot(t5, Y1);
title(["Observer + gain controller, with initial conditions",'Output Y1']);
xlabel('time [s]');
ylabel('Amplitude');

subplot(2, 1, 2);
plot(t5, Y2);
title(["Observer + gain controller, with initial conditions",'Output Y2']);
xlabel('time [s]');
ylabel('Amplitude');
 
