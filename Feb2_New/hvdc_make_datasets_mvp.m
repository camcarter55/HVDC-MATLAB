function hvdc_make_datasets_mvp()
% hvdc_make_datasets_mvp_fixed
%
% Purpose:
%   Run many pole–pole (p-n) HVDC fault simulations while varying:
%     - fault time (t_fault)
%     - fault resistance (R_fault)
%     - fault location along the line (fault_distance_km)
%   and save a dataset suitable for later ML training (NOT training here).
%
% Outputs:
%   datasets/<datasetBaseName>.mat containing:
%     Xloc      cell{N,1} each is [T x 8] single
%     Yloc_km   [N x 1]  fault location km (label)
%     meta      struct with Rf, tf, dist_km, windowing, etc.
%
%   Channels per sample (8 total):
%     [VpA IpA VnA InA VpB IpB VnB InB]
%
% Notes:
%   - This script assumes your model uses the variables defined in
%     testing_for_training_fixed.m (or compatible names):
%       fault_enabled, fault_type, R_fault, t_fault, fault_distance_km, total_line_km
%   - Signal logging must produce logsout (Dataset) with the signals below.
%     If your log names differ, edit the SIG_CANDIDATES mapping.
%
% Requirements:
%   - Simulink + Simscape Electrical Specialized Power Systems (if using powergui)
%   - Parallel Computing Toolbox recommended (parsim)

%% =========================
% MODEL + OUTPUT PATHS
%% =========================
mdl = "simulink_testing";            % <-- CHANGE to your model name (without .slx)
outDir = "datasets";
datasetBaseName = "hvdc_pn_mvp_v1";

if ~exist(outDir,"dir"); mkdir(outDir); end
finalFile = fullfile(outDir, datasetBaseName + ".mat");
tmpFile   = fullfile(outDir, datasetBaseName + "_checkpoint.mat");

%% =========================
% SIGNAL LOGGING NAMES
% Provide candidates for each required channel.
% The first match found in logsout will be used.
%% =========================
SIG_CANDIDATES = struct( ...
    "VpA", {{ "V_Plus_End_A",  "Vp_endA","V_p_A","Vpos_A" }}, ...
    "IpA", {{ "I_Plus_End_A",  "Ip_endA","I_p_A","Ipos_A" }}, ...
    "VnA", {{ "V_Minus_End_A", "Vn_endA","V_n_A","Vneg_A" }}, ...
    "InA", {{ "I_Minus_End_A", "In_endA","I_n_A","Ineg_A" }}, ...
    "VpB", {{ "V_Plus_End_B",  "Vp_endB","V_p_B","Vpos_B" }}, ...
    "IpB", {{ "I_Plus_End_B",  "Ip_endB","I_p_B","Ipos_B" }}, ...
    "VnB", {{ "V_Minus_End_B", "Vn_endB","V_n_B","Vneg_B" }}, ...
    "InB", {{ "I_Minus_End_B", "In_endB","I_n_B","Ineg_B" }} ...
);


%% =========================
% SIM SETTINGS
%% =========================
rng(1,"twister");

N = 5;                 % number of fault simulations
total_line_km = 546;      % must match model line length convention

% Fault parameter ranges
Rmin = 0.003;
Rmax = 0.3;
tFaultMin = 0.5;
tFaultMax = 1.0;

stopTime_s = 1.5;         % your model stops at 1.5 s

%% =========================
% ABSOLUTE WINDOWING + DOWNSAMPLE
% Window is independent of fault time.
%% =========================
Tstart = 0.4;             % seconds
Tend   = 1.4;             % seconds
ds = 10;                  % keep every ds-th sample AFTER windowing

%% =========================
% CHECKPOINTING
%% =========================
checkpointEvery = 1;

%% =========================
% PREP MODEL
%% =========================
load_system(mdl);
set_param(mdl, "SignalLogging","on", ...
               "SignalLoggingName","logsout", ...
               "SignalLoggingSaveFormat","Dataset", ...
               "StopTime", num2str(stopTime_s));

%% =========================
% BUILD RANDOM FAULTS
%% =========================
Rf      = zeros(N,1);
dist_km = zeros(N,1);
tf      = zeros(N,1);

% log-uniform resistance
a = log10(Rmin);
b = log10(Rmax);

simIn(N,1) = Simulink.SimulationInput(mdl);

for k = 1:N
    Rf(k)      = 10^(a + (b - a)*rand);
    dist_km(k) = total_line_km * rand;                    % uniform along line
    tf(k)      = tFaultMin + (tFaultMax - tFaultMin)*rand;

    simIn(k) = Simulink.SimulationInput(mdl) ...
        .setVariable("fault_enabled", true) ...
        .setVariable("fault_type", "p-n") ...
        .setVariable("R_fault", Rf(k)) ...
        .setVariable("t_fault", tf(k)) ...
        .setVariable("fault_distance_km", dist_km(k)) ...
        .setVariable("total_line_km", total_line_km); ...
        %.setUserData(struct("R_fault",Rf(k), "t_fault",tf(k), "fault_distance_km",dist_km(k)));
end

%% =========================
% RUN BATCH SIM
%% =========================
out = parsim(simIn, ...
    "ShowProgress","on", ...
    "TransferBaseWorkspaceVariables","on", ...
    "StopOnError","off");     % keep going if a run fails; failed runs will be skipped

%% =========================
% EXTRACT WINDOWS
%% =========================
Xloc = cell(N,1);
Yloc_km   = dist_km;
Yloc_norm = Yloc_km / total_line_km;

failed = false(N,1);

for k = 1:N
    try
        if isempty(out(k).logsout)
            error("Missing logsout (Signal Logging).");
        end

        logs = out(k).logsout;

        % Resolve actual signal names once per run (robust to naming differences)
        sigResolved = resolveSigNames(logs, SIG_CANDIDATES);

        % Absolute-time window (0.4s to 1.4s)
        Xf = extractWindow8_absolute(logs, sigResolved, Tstart, Tend, ds);

        % Store as single
        Xloc{k} = single(Xf);

    catch ME
        failed(k) = true;
        Xloc{k} = [];
        warning("Run %d failed/skipped: %s", k, ME.message);
    end

    % checkpoint
    if mod(k,checkpointEvery) == 0
        meta = buildMeta(mdl, N, total_line_km, Rf, tf, dist_km, Tstart, Tend, ds, failed);
        save(tmpFile, "Xloc","Yloc_km","Yloc_norm","meta","k","-v7.3");
        fprintf("Checkpoint saved: k=%d  failed=%d  -> %s\n", k, nnz(failed(1:k)), tmpFile);
    end
end

% Drop failed runs (keeps arrays aligned)
keep = ~failed;
Xloc = Xloc(keep);
Yloc_km = Yloc_km(keep);
Yloc_norm = Yloc_norm(keep);

meta = buildMeta(mdl, N, total_line_km, Rf, tf, dist_km, Tstart, Tend, ds, failed);

%% =========================
% FINAL SAVE
%% =========================
save(finalFile, "Xloc","Yloc_km","Yloc_norm","meta","-v7.3");
fprintf("\nSaved dataset: %s\nKept %d / %d runs (failed %d)\n", finalFile, numel(Xloc), N, nnz(failed));

clear out
end

%% ===== helper functions =====

function meta = buildMeta(mdl, N, total_line_km, Rf, tf, dist_km, Tstart, Tend, ds, failed)
meta = struct();
meta.model = mdl;
meta.created = datetime("now");
meta.N_requested = N;
meta.total_line_km = total_line_km;
meta.R_fault = Rf;
meta.t_fault = tf;
meta.fault_distance_km = dist_km;
meta.window.type = "absolute";
meta.window.t_start = Tstart;
meta.window.t_end = Tend;
meta.window.ds = ds;
meta.failed_mask = failed;
meta.channel_order = ["VpA","IpA","VnA","InA","VpB","IpB","VnB","InB"];
end

function sig = resolveSigNames(logs, SIG_CANDIDATES)
% Return a struct with fields VpA,IpA,VnA,InA,VpB,IpB,VnB,InB
% whose values are the ACTUAL element names in logsout.
names = string(logs.getElementNames);

fields = ["VpA","IpA","VnA","InA","VpB","IpB","VnB","InB"];
sig = struct();

for f = fields
    candidates = string(SIG_CANDIDATES.(f));
    hit = "";
    for c = candidates
        if any(names == c)
            hit = c;
            break;
        end
    end

    if hit == ""
        % Try case-insensitive contains match as a last resort
        for c = candidates
            mask = contains(lower(names), lower(c));
            if any(mask)
                hit = names(find(mask,1,"first"));
                break;
            end
        end
    end

    if hit == ""
        error("Missing required logged signal for %s. Tried: %s. Available logsout names include e.g.: %s", ...
            f, strjoin(candidates,", "), strjoin(names(1:min(15,numel(names))), ", "));
    end

    sig.(f) = char(hit);
end
end

function X = extractWindow8_absolute(logs, sig, t0, t1, ds)
% Absolute-time window extractor
% Output X: [T x 8] = [VpA IpA VnA InA VpB IpB VnB InB]

VpA = logs.get(sig.VpA).Values;
IpA = logs.get(sig.IpA).Values;
VnA = logs.get(sig.VnA).Values;
InA = logs.get(sig.InA).Values;

VpB = logs.get(sig.VpB).Values;
IpB = logs.get(sig.IpB).Values;
VnB = logs.get(sig.VnB).Values;
InB = logs.get(sig.InB).Values;

VpA = getsampleusingtime(VpA, t0, t1);
IpA = getsampleusingtime(IpA, t0, t1);
VnA = getsampleusingtime(VnA, t0, t1);
InA = getsampleusingtime(InA, t0, t1);

VpB = getsampleusingtime(VpB, t0, t1);
IpB = getsampleusingtime(IpB, t0, t1);
VnB = getsampleusingtime(VnB, t0, t1);
InB = getsampleusingtime(InB, t0, t1);

n = min([ ...
    numel(VpA.Time), numel(IpA.Time), ...
    numel(VnA.Time), numel(InA.Time), ...
    numel(VpB.Time), numel(IpB.Time), ...
    numel(VnB.Time), numel(InB.Time)]);

VpA = VpA.Data(1:n);
IpA = IpA.Data(1:n);
VnA = VnA.Data(1:n);
InA = InA.Data(1:n);

VpB = VpB.Data(1:n);
IpB = IpB.Data(1:n);
VnB = VnB.Data(1:n);
InB = InB.Data(1:n);

if ds > 1
    idx = 1:ds:n;
    VpA = VpA(idx); IpA = IpA(idx);
    VnA = VnA(idx); InA = InA(idx);
    VpB = VpB(idx); IpB = IpB(idx);
    VnB = VnB(idx); InB = InB(idx);
end

X = [VpA, IpA, VnA, InA, VpB, IpB, VnB, InB];
end
