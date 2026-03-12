%% VALIDATION: PMSM Motor Plant
%
%  Validates against analytical properties of permanent magnet synchronous
%  motor dynamics in dq-frame.
%
%  Reference parameters from:
%    Krause et al. (2013), "Analysis of Electric Machinery", Ch 5
%    Pillay & Krishnan (1989), "Modeling of PM motor drives", IEEE TIE
%
%  Tests:
%    1. Steady-state speed: with constant vq, omega = (vq - Rs*iq_ss) / (pp*psi_f)
%    2. Torque formula: Te = 1.5*pp*psi_f*iq (for SPMSM where Ld=Lq)
%    3. No-load free spin: with TL=0, motor accelerates under torque
%    4. Back-EMF: at known speed, back-EMF = pp*omega*psi_f
%    5. Step response to voltage input matches analytical first-order dynamics

clear; clc;
fprintf('=== PMSM Validation ===\n\n');
pass_count = 0;
fail_count = 0;

motor = plants.PMSM('Rs', 1.2, 'Ld', 6.8e-3, 'Lq', 6.8e-3, ...
    'psi_f', 0.175, 'pp', 4, 'J', 0.003, 'B', 0.001, 'TL', 0);

%% TEST 1: Torque formula for SPMSM (Ld = Lq)
fprintf('Test 1: Torque formula Te = 1.5*pp*psi_f*iq... ');
p = motor.params;

% Test at multiple iq values
test1_pass = true;
for iq_test = [-5, -1, 0, 0.5, 1, 3, 10]
    x_test = [0; iq_test; 0; 0];  % id=0, iq=iq_test
    Te = motor.get_torque(x_test);
    Te_expected = 1.5 * p.pp * p.psi_f * iq_test;

    if abs(Te - Te_expected) > 1e-12
        test1_pass = false;
        fprintf('FAIL at iq=%.1f: Te=%.6f, expected=%.6f\n', iq_test, Te, Te_expected);
        break;
    end
end

if test1_pass
    fprintf('PASS (7 iq values tested)\n');
    pass_count = pass_count + 1;
else
    fail_count = fail_count + 1;
end

%% TEST 2: Reluctance torque is zero for SPMSM (Ld = Lq)
fprintf('Test 2: Reluctance torque = 0 when Ld = Lq... ');
x_test = [5; 3; 100; 0];  % nonzero id and iq
Te = motor.get_torque(x_test);
Te_psi_only = 1.5 * p.pp * p.psi_f * x_test(2);  % psi_f component only

if abs(Te - Te_psi_only) < 1e-12
    fprintf('PASS (reluctance component = %.2e)\n', abs(Te - Te_psi_only));
    pass_count = pass_count + 1;
else
    fprintf('FAIL (reluctance component = %.6f, expected 0)\n', Te - Te_psi_only);
    fail_count = fail_count + 1;
end

%% TEST 3: At zero state, zero input => zero dynamics
fprintf('Test 3: Equilibrium at origin... ');
xdot = motor.dynamics(0, zeros(4,1), [0; 0], zeros(4,1));
if norm(xdot) < 1e-15
    fprintf('PASS\n');
    pass_count = pass_count + 1;
else
    fprintf('FAIL (xdot norm = %.2e)\n', norm(xdot));
    fail_count = fail_count + 1;
end

%% TEST 4: Electrical time constant check
%  For di_q/dt at t=0 with step vq, omega=0, iq=0:
%    Lq * diq/dt = vq - Rs*0 - pp*0*(Ld*0 + psi_f) = vq
%    => diq/dt = vq / Lq
fprintf('Test 4: Electrical time constant (diq/dt = vq/Lq at t=0)... ');
vq_test = 10;
xdot = motor.dynamics(0, zeros(4,1), [0; vq_test], zeros(4,1));

diq_dt_expected = vq_test / p.Lq;
diq_dt_actual   = xdot(2);

err = abs(diq_dt_actual - diq_dt_expected) / abs(diq_dt_expected);
if err < 1e-10
    fprintf('PASS (diq/dt = %.1f, expected %.1f)\n', diq_dt_actual, diq_dt_expected);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (diq/dt = %.4f, expected %.4f, err=%.2e)\n', ...
        diq_dt_actual, diq_dt_expected, err);
    fail_count = fail_count + 1;
end

%% TEST 5: Back-EMF at known speed
%  At omega=100 rad/s, id=0, iq=0, vd=vq=0:
%    did/dt = (0 - 0 + pp*omega*Lq*0) / Ld = 0
%    diq/dt = (0 - 0 - pp*omega*(0 + psi_f)) / Lq = -pp*omega*psi_f/Lq
fprintf('Test 5: Back-EMF at omega=100 rad/s... ');
omega_test = 100;
x_backemf = [0; 0; omega_test; 0];
xdot = motor.dynamics(0, x_backemf, [0; 0], zeros(4,1));

back_emf_effect = -p.pp * omega_test * p.psi_f / p.Lq;
err = abs(xdot(2) - back_emf_effect);

if err < 1e-8
    fprintf('PASS (diq/dt = %.2f, expected %.2f from back-EMF)\n', xdot(2), back_emf_effect);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (diq/dt = %.4f, expected %.4f)\n', xdot(2), back_emf_effect);
    fail_count = fail_count + 1;
end

%% TEST 6: Steady-state current under constant voltage
%  Apply constant vq, simulate until steady state. At SS:
%    did/dt = 0 => id_ss = pp*omega_ss*Lq*iq_ss / Rs  (coupling)
%    diq/dt = 0 => iq_ss = (vq - pp*omega_ss*psi_f) / Rs  (ignoring Ld*id coupling for SPMSM with id≈0)
%    domega/dt = 0 => Te_ss = B*omega_ss  (no load)
%  Solve: 1.5*pp*psi_f*iq_ss = B*omega_ss
fprintf('Test 6: Steady-state speed under constant vq=10V... ');
motor_ss = plants.PMSM('Rs', 1.2, 'Ld', 6.8e-3, 'Lq', 6.8e-3, ...
    'psi_f', 0.175, 'pp', 4, 'J', 0.003, 'B', 0.001, 'TL', 0);
motor_ss.x0 = zeros(4,1);

dt = 1e-5;
T_sim = 2.0;
N = round(T_sim/dt);
x = motor_ss.x0;
vq_ss = 10;

for k = 1:N
    t = (k-1)*dt;
    u = [0; vq_ss];
    k1 = motor_ss.dynamics(t, x, u, zeros(4,1));
    k2 = motor_ss.dynamics(t+dt/2, x+dt/2*k1, u, zeros(4,1));
    k3 = motor_ss.dynamics(t+dt/2, x+dt/2*k2, u, zeros(4,1));
    k4 = motor_ss.dynamics(t+dt, x+dt*k3, u, zeros(4,1));
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

omega_ss_sim = x(3);
iq_ss_sim    = x(2);

% Analytical steady-state (approximate for SPMSM with small id):
%   iq_ss = (vq - pp*omega_ss*psi_f) / Rs
%   1.5*pp*psi_f*iq_ss = B*omega_ss
%   => 1.5*pp*psi_f*(vq - pp*omega_ss*psi_f)/Rs = B*omega_ss
%   => omega_ss = 1.5*pp*psi_f*vq / (Rs*B + 1.5*pp^2*psi_f^2)
Kt = 1.5 * p.pp * p.psi_f;
omega_ss_analytical = Kt * vq_ss / (p.Rs * p.B + Kt * p.pp * p.psi_f);

err = abs(omega_ss_sim - omega_ss_analytical) / abs(omega_ss_analytical);
if err < 0.02  % 2% tolerance (coupling ignored in analytical)
    fprintf('PASS\n');
    fprintf('         omega_ss_sim = %.2f rad/s\n', omega_ss_sim);
    fprintf('         omega_ss_analytical ≈ %.2f rad/s\n', omega_ss_analytical);
    fprintf('         relative error = %.2f%%\n', err*100);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (sim=%.2f, analytical=%.2f, err=%.1f%%)\n', ...
        omega_ss_sim, omega_ss_analytical, err*100);
    fail_count = fail_count + 1;
end

%% TEST 7: Voltage clamping
fprintf('Test 7: Voltage clamping at V_max... ');
xdot_clamp = motor.dynamics(0, zeros(4,1), [100; 100], zeros(4,1));
xdot_limit = motor.dynamics(0, zeros(4,1), [48; 48], zeros(4,1));
if norm(xdot_clamp - xdot_limit) < 1e-12
    fprintf('PASS (100V clamped to 48V)\n');
    pass_count = pass_count + 1;
else
    fprintf('FAIL\n');
    fail_count = fail_count + 1;
end

%% SUMMARY
fprintf('\n=== PMSM: %d PASSED, %d FAILED ===\n', pass_count, fail_count);
if fail_count == 0
    fprintf('>> ALL VALIDATIONS PASSED <<\n');
else
    fprintf('>> %d VALIDATIONS FAILED — REVIEW IMPLEMENTATION <<\n', fail_count);
end
