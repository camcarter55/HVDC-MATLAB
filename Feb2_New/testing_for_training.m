% testing_for_training_fixed.m
% Parameters file (ML/batch-safe)
% Designed to be called from your model InitFcn / PreLoadFcn / Model Workspace.
%
% This file defines:
%   - fault_enabled (bool)
%   - fault_type    (string)  -> must be "p-n" (pole–pole)
%   - R_fault       (ohms)
%   - t_fault       (s)
%   - fault_distance_km (km from End A, 0..total_line_km)
%
% And creates common ALIASES used by fault blocks so the model & scripts
% can interoperate robustly.

%% Sample times
Ts_PWM     = 5e-6;
Ts_Control = 50e-6;
Ts_Power   = Ts_PWM;

%% Grid parameters
Fnom      = 60;
Vnom_grid = 50e3;
Psc_grid  = 200e6;

%% SEC parameters
Pnom_dc_3L     = 3e6;
Vnom_dc_3L     = 3000;
H_3L           = 1/Fnom*1;
Clink_3L       = Pnom_dc_3L*H_3L*2 /Vnom_dc_3L^2;
Vc_Initial_3L  = Vnom_dc_3L/2;

Pnom_3L      = Pnom_dc_3L;
Vnom_prim_3L = Vnom_grid;
m_nom_3L     = 0.8;
Vnom_sec_3L  = 0.5*Vnom_dc_3L/sqrt(2)*sqrt(3)*m_nom_3L;
Lxfo_3L      = 0.10;
Rxfo_3L      = 0.10/30;
Rm_3L        = 500;
Lm_3L        = 500;

Qnom_Filter1  = 0.2*Pnom_dc_3L;
Fn_Filter1    = 33*Fnom;
Q_Filter1     = 10;

Fc_3L         = 33*Fnom;
Freq_Filter   = 1000;

Lact = 0.15*((Vnom_sec_3L/1000)*1e3)^2/(Pnom_3L)/314.159;

Kp_VDCreg_3L   = 3;
Ki_VDCreg_3L   = 300;
LimitU_VDCreg_3L = 1.5;
LimitL_VDCreg_3L = -1.5;

Rff_3L         = Rxfo_3L;
Lff_3L         = Lxfo_3L;
Kp_Ireg_3L     = 0.2/2;
Ki_Ireg_3L     = 15;
LimitU_Ireg_3L = 1.5;
LimitL_Ireg_3L = -1.5;

%% =========================
% Fault configuration (BATCH-SAFE, pole–pole only)
%% =========================
% total_line_km MAY be overridden by the dataset script via SimulationInput
if ~exist("total_line_km","var"); total_line_km = 546; end

NEVER_TIME = 999;     % "never triggers" switch time (must be > StopTime)

% Defaults so model can run manually without generator script
if ~exist("fault_enabled","var");      fault_enabled = false; end
if ~exist("fault_type","var");         fault_type = "p-n"; end   % enforce pole–pole for MVP
if ~exist("R_fault","var");            R_fault = 0.01; end
if ~exist("t_fault","var");            t_fault = 0.75; end
if ~exist("fault_distance_km","var");  fault_distance_km = 200; end

% Clamp distance to line bounds
fault_distance_km = max(0, min(fault_distance_km, total_line_km));

% Normalize (0=End A, 1=End B)
fault_distance_pu = fault_distance_km / total_line_km;

% Enforce pole–pole only
if fault_enabled && string(fault_type) ~= "p-n"
    error("Pole-to-pole only: set fault_type='p-n'. Got: %s", string(fault_type));
end

% Switch times used in Simulink (time-controlled breakers/switches)
if fault_enabled
    p_n_switch_time = t_fault;
else
    p_n_switch_time = NEVER_TIME;
end

% Keep these defined in case your model references them
p_g_switch_time = NEVER_TIME;
n_g_switch_time = NEVER_TIME;

%% =========================
% Aliases (harmless if unused, VERY helpful if naming varies)
%% =========================
% Resistance aliases
p_n_fault_resistance = R_fault;
pn_fault_R           = R_fault;

% Location aliases
p_n_fault_distance_km = fault_distance_km;
p_n_fault_distance_pu = fault_distance_pu;
pn_fault_km           = fault_distance_km;
pn_fault_pu           = fault_distance_pu;

% Time aliases
p_n_fault_time = t_fault;
pn_fault_t      = t_fault;

% Enable aliases
p_n_fault_enabled = fault_enabled;
pn_fault_enable   = fault_enabled;
