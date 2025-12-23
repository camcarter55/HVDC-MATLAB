% Parameters file
clc;
clear all;

%% Sample times
Ts_PWM = 5e-6;           
Ts_Control = 50e-6;      % Control systems sample Ts_Control = 1/F_Switching.
Ts_Power = Ts_PWM;       % Default value  Ts_PWM  = Ts_Control/10
 
%% Grid parameters
Fnom = 60;               % Nominal system frequency (Hz)
Vnom_grid = 50e3;        % nominal voltage (L-L rms)
Psc_grid = 200e6;        % Short-circuit level (VA)


%% SEC parameters

% DC link:
Pnom_dc_3L = 3e6;                            % Nominal DC link Power (VA)
Vnom_dc_3L = 3000;                           % Nominal DC link voltage (V)
H_3L = 1/Fnom*1;                             % DC link stored energy constant(s) = 1 cycles
Clink_3L = Pnom_dc_3L*H_3L*2 /Vnom_dc_3L^2;  % PM DC link capacitor (F)
Vc_Initial_3L = Vnom_dc_3L/2;                % PM capacitor initial voltage (V)

% Transformer:
Pnom_3L = Pnom_dc_3L;         % Transformer nominal power (VA)
Vnom_prim_3L = Vnom_grid;     % Nominal primary voltage (V)
m_nom_3L = 0.8;               % Nominal modulation index for 3-Level rectifier
Vnom_sec_3L = 0.5*Vnom_dc_3L/sqrt(2)*sqrt(3)*m_nom_3L;  % Nominal secondary voltage (V)
Lxfo_3L = 0.10;               % Total Leakage inductance (pu)
Rxfo_3L = 0.10/30;            % Total winding resistance (pu)
Rm_3L = 500;                  % Magnetization resistance (pu)
Lm_3L = 500;                  % Magnetization inductance (pu)

% Filter:
Qnom_Filter1=0.2*Pnom_dc_3L; % Nominal reactive power (VA)  20% of the nomial DC power
Fn_Filter1=33*Fnom;           % Tuning frequency (Hz)        is the switching frequency  
Q_Filter1=10;                  % Quality factor               higher Q, sharper the filter

%  Control Parameters
Fc_3L=33*Fnom;                % PWM carrier frequency (Hz)
Freq_Filter=1000;             % Measurement filters natural frequency (Hz)


Lact = 0.15*((Vnom_sec_3L/1000)*1e3)^2/(Pnom_3L)/314.159


% VDC controller 
Kp_VDCreg_3L= 3;            % Proportional gain
Ki_VDCreg_3L= 300;            % Integral gain
LimitU_VDCreg_3L= 1.5;        % Output (Idref) Upper limit (pu)
LimitL_VDCreg_3L= -1.5;       % Output (Idref) Lower limit (pu)

% Current controller
Rff_3L= Rxfo_3L;              % Feedforward R
Lff_3L= Lxfo_3L;              % Feedforward L
Kp_Ireg_3L= 0.2/2;            % Proportional gain
Ki_Ireg_3L= 15;                % Integral gain
LimitU_Ireg_3L= 1.5;          % Output (Vdq_conv) Upper limit (pu)
LimitL_Ireg_3L= -1.5;         % Output (Vdq_conv) Lower limit (pu)






% fault configuration

total_line_km = 546;   % length of the DC link (km)

fault = struct;
fault.enabled   = false;
fault.mode      = 'none';   % 'none', 'random', 'custom'
fault.type      = 'none';   % 'p-n', 'p-gnd', 'n-gnd'
fault.R         = 1;      % ohms
fault.time_s    = 999;      % seconds
fault.dist_km   = 200;      % 0–546 km from sending end; this default doesn't physically mean anything

% Use fault?
useFault = input('Enable fault? (1 = yes, 0 = no): ');

if useFault ~= 1
    disp('Fault disabled.');
else
    fault.enabled = true;

    % Ask user
    disp('Select fault type:');
    disp('  1 = pole-to-pole (p-n)');
    disp('  2 = positive pole to ground (p-gnd)');
    disp('  3 = negative pole to ground (n-gnd)');
    typeChoice = input('Enter type number: ');

    switch typeChoice
        case 1
            fault.type = 'p-n';
        case 2
            fault.type = 'p-gnd';
        case 3
            fault.type = 'n-gnd';
        otherwise
            error('Invalid choice for fault type.');
    end

    % Choose mode
    modeChoice = input('Fault parameters: (1 = random, 2 = custom): ');

    if modeChoice == 1
        fault.mode = 'random';

        % Randomized resistance
        switch fault.type
            case 'p-n'  % pole-to-pole
                Rmin = 0.003;
                Rmax = 0.3;

            case {'p-gnd','n-gnd'}  % pole-to-ground
                Rmin = 0.03;
                Rmax = 3;
        end

        a = log10(Rmin);
        b = log10(Rmax);
        fault.R = 10^(a + (b - a)*rand);

        % Random time between 0.5 s and 1.0 s
        fault.time_s = 0.5 + 0.5*rand;

        % Random location along line (km)
        fault.dist_km = total_line_km * rand;   % 0–546 km

    elseif modeChoice == 2
        fault.mode = 'custom';

        % Custom resistance
        fault.R        = input('Enter fault resistance (ohms): ');

        % Custom time
        fault.time_s   = input('Enter fault time (seconds): ');

        % Custom location in km
        fault.dist_km  = input(sprintf('Enter fault location (0–%.1f km from sending end): ', ...
                                       total_line_km));

    else
        error('Invalid mode. Choose 1 = random, 2 = custom.');
    end

    % Print parameters
    fprintf('\n=== Fault configuration ===\n');
    fprintf('Enabled   : %d\n', fault.enabled);
    fprintf('Type      : %s\n', fault.type);
    fprintf('Mode      : %s\n', fault.mode);
    fprintf('R_fault   : %.4f Ω\n', fault.R);
    fprintf('t_fault   : %.4f s\n', fault.time_s);
    fprintf('Location  : %.2f km (0 = send end)\n', fault.dist_km);
    fprintf('===========================\n\n');
end

% Remove variables from the struct for Simulink

fault_enabled   = fault.enabled;
fault_type      = fault.type;
R_fault         = fault.R;
t_fault         = fault.time_s;
fault_distance_km = fault.dist_km;

% Breaker switch times for each fault type
NEVER_TIME = 999; % Out of sampling range

switch fault.type
    case 'p-n'
        p_n_switch_time = t_fault;
        p_g_switch_time = NEVER_TIME;
        n_g_switch_time = NEVER_TIME;

    case 'p-gnd'
        p_n_switch_time = NEVER_TIME;
        p_g_switch_time = t_fault;
        n_g_switch_time = NEVER_TIME;

    case 'n-gnd'
        p_n_switch_time = NEVER_TIME;
        p_g_switch_time = NEVER_TIME;
        n_g_switch_time = t_fault;

    otherwise
        p_n_switch_time = NEVER_TIME;
        p_g_switch_time = NEVER_TIME;
        n_g_switch_time = NEVER_TIME;
end
