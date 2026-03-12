function hvdc_make_datasets_mvp()
% hvdc_make_datasets_mvp (ROBUST v3 - "none" safe, no leading-dot chaining)
% Fixes:
%  - Does NOT pass fault_type="none" into model (common init/start failure)
%  - Avoids MATLAB leading-dot chaining syntax (compatible with older MATLAB)
%  - Always defines switch-time variables and fault params
%  - Stronger diagnostics if sim fails at init/start (no logsout)

%% ---------------- USER SETTINGS ----------------
mdl = "simulink_testing";
outDir = "datasets";
datasetBaseName = "ds_TEST_v3_NONE_SAFE";

N = 20;
P_none = 0.25;
classNames = ["none","p-g","p-n","n-g"];

total_line_km = 546;

% Fault randomization
Rmin = 0.003; Rmax = 0.3;
tFaultMin = 0.5; tFaultMax = 1.0;
stopTime_s = 1.5;

% Windowing
win_pre  = 1.5e-3;
win_post = 6.0e-3;
search_pre  = 2.0e-3;
search_post = 6.0e-3;
z_k = 10;
min_hold = 3;
ds = 1;

anchorMin = 0.45;
anchorMax = stopTime_s - 0.05;
noneAnchorRetries = 10;

% "Never" time for fault switch controls (must exceed StopTime)
NEVER_TIME = 999;

% Noise
noiseCfg.enabled = true;
noiseCfg.sigma_base = 0.001;
noiseCfg.sigma_scale_none  = 1.5;
noiseCfg.sigma_scale_fault = 1.0;
noiseCfg.bias_frac = 0.0005;

% Logging name candidates (edit if needed)
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

rng(1,"twister");

%% ---------------- PATHS ----------------
if ~exist(outDir,"dir"); mkdir(outDir); end
finalFile = fullfile(outDir, datasetBaseName + ".mat");

load_system(mdl);

%% ---------------- DETERMINISTIC LABELS ----------------
nNone  = round(N * P_none);
nFault = N - nNone;
nEach = floor(nFault/3);
rem = nFault - 3*nEach;

labels = [ ...
    repmat("none", nNone, 1); ...
    repmat("p-g",  nEach + (rem>=1), 1); ...
    repmat("p-n",  nEach + (rem>=2), 1); ...
    repmat("n-g",  nEach, 1) ...
];
labels = labels(randperm(numel(labels)));
Yclass_str = labels;

fprintf("Planned class counts:\n");
disp(countcats(categorical(Yclass_str, classNames)));

%% ---------------- BUILD SIM INPUTS ----------------
Rf       = nan(N,1);
tf       = nan(N,1);
dist_km  = nan(N,1);
anchor_t = nan(N,1);

a = log10(Rmin); b = log10(Rmax);

simIn(N,1) = Simulink.SimulationInput(mdl);

VALID_MODEL_TYPES = ["p-g","p-n","n-g"];
DEFAULT_MODEL_TYPE = "p-g"; % used for "none" runs (model never sees "none")

for k = 1:N
    % Base simulation input w/ logging enabled
    baseIn = Simulink.SimulationInput(mdl);
    baseIn = baseIn.setModelParameter("StopTime", num2str(stopTime_s));
    baseIn = baseIn.setModelParameter("ReturnWorkspaceOutputs","on");
    baseIn = baseIn.setModelParameter("SaveOutput","on");
    baseIn = baseIn.setModelParameter("OutputSaveName","yout");
    baseIn = baseIn.setModelParameter("SaveTime","on");
    baseIn = baseIn.setModelParameter("TimeSaveName","tout");
    baseIn = baseIn.setModelParameter("SignalLogging","on");
    baseIn = baseIn.setModelParameter("SignalLoggingName","logsout");
    baseIn = baseIn.setModelParameter("SignalLoggingSaveFormat","Dataset");

    % Defaults: no fault (switches never)
    p_g_switch_time = NEVER_TIME;
    p_n_switch_time = NEVER_TIME;
    n_g_switch_time = NEVER_TIME;

    % Always define these too
    R_fault = 0.01;
    t_fault = 0.75;
    fault_distance_km = 0;

    % Model-facing controls (IMPORTANT): never "none"
    fault_enabled_model = false;
    fault_type_model = char(DEFAULT_MODEL_TYPE);

    if Yclass_str(k) == "none"
        anchor_t(k) = anchorMin + (anchorMax - anchorMin)*rand;
        t_fault = anchor_t(k); % harmless placeholder

        fault_enabled_model = false;
        fault_type_model = char(DEFAULT_MODEL_TYPE);

        Rf(k) = NaN; tf(k) = NaN; dist_km(k) = NaN;

    else
        R_fault = 10^(a + (b-a)*rand);
        t_fault = tFaultMin + (tFaultMax - tFaultMin)*rand;
        fault_distance_km = total_line_km * rand;

        Rf(k) = R_fault;
        tf(k) = t_fault;
        dist_km(k) = fault_distance_km;

        fault_enabled_model = true;

        if any(VALID_MODEL_TYPES == string(Yclass_str(k)))
            fault_type_model = char(string(Yclass_str(k)));
        else
            fault_type_model = char(DEFAULT_MODEL_TYPE);
        end

        switch string(Yclass_str(k))
            case "p-g"
                p_g_switch_time = t_fault;
            case "p-n"
                p_n_switch_time = t_fault;
            case "n-g"
                n_g_switch_time = t_fault;
        end
    end

    % Assign variables WITHOUT chaining
    simIn(k) = baseIn;

    simIn(k) = simIn(k).setVariable("total_line_km", total_line_km);

    % Model-facing (safe)
    simIn(k) = simIn(k).setVariable("fault_enabled", fault_enabled_model);
    simIn(k) = simIn(k).setVariable("fault_type", fault_type_model);

    % Common fault params
    simIn(k) = simIn(k).setVariable("R_fault", R_fault);
    simIn(k) = simIn(k).setVariable("t_fault", t_fault);
    simIn(k) = simIn(k).setVariable("fault_distance_km", fault_distance_km);

    % Switch times
    simIn(k) = simIn(k).setVariable("p_g_switch_time", p_g_switch_time);
    simIn(k) = simIn(k).setVariable("p_n_switch_time", p_n_switch_time);
    simIn(k) = simIn(k).setVariable("n_g_switch_time", n_g_switch_time);

    % Per-fault params
    simIn(k) = simIn(k).setVariable("p_g_fault_resistance", R_fault);
    simIn(k) = simIn(k).setVariable("p_n_fault_resistance", R_fault);
    simIn(k) = simIn(k).setVariable("n_g_fault_resistance", R_fault);

    simIn(k) = simIn(k).setVariable("p_g_fault_distance_km", fault_distance_km);
    simIn(k) = simIn(k).setVariable("p_n_fault_distance_km", fault_distance_km);
    simIn(k) = simIn(k).setVariable("n_g_fault_distance_km", fault_distance_km);
end

%% ---------------- SIMULATE ----------------
out = parsim(simIn, ...
    "ShowProgress","on", ...
    "TransferBaseWorkspaceVariables","on", ...
    "StopOnError","off");

%% ---------------- EXTRACT ----------------
Xloc = cell(N,1);
Yclass = categorical(Yclass_str, classNames);
Yloc_km = dist_km;                 % NaN for none
Yloc_norm = Yloc_km / total_line_km;

failed = false(N,1);
failReason = strings(N,1);

for k = 1:N
    try
        logs = getLogsOrThrowRealError(out(k));
        sig  = resolveSigNames(logs, SIG_CANDIDATES);

        if Yclass_str(k) == "none"
            ok = false;
            lastErr = "";
            for tries = 1:noneAnchorRetries
                t_anchor = anchorMin + (anchorMax - anchorMin)*rand;
                t0 = t_anchor - win_pre;
                t1 = t_anchor + win_post;
                try
                    Xf = extractWindow8_by_time_robust(logs, sig, t0, t1, ds);
                    ok = true;
                    break
                catch MEa
                    lastErr = string(MEa.message);
                end
            end
            if ~ok
                error("None extraction failed after retries. Last: %s", lastErr);
            end

            if noiseCfg.enabled
                Xf = addMeasurementNoise(Xf, "none", noiseCfg);
            end
        else
            Xf = extractWindow8_onset_robust8(logs, sig, tf(k), ...
                win_pre, win_post, search_pre, search_post, z_k, min_hold, ds);

            if noiseCfg.enabled
                Xf = addMeasurementNoise(Xf, Yclass_str(k), noiseCfg);
            end
        end

        Xloc{k} = single(Xf);

    catch ME
        failed(k) = true;
        failReason(k) = string(ME.message);
        warning("Run %d (%s) failed: %s", k, Yclass_str(k), ME.message);
        Xloc{k} = [];
    end
end

keep = ~failed;
Xloc = Xloc(keep);
Yclass = Yclass(keep);
Yloc_km = Yloc_km(keep);
Yloc_norm = Yloc_norm(keep);

fprintf("\nKept %d / %d (failed %d)\n", numel(Xloc), N, nnz(failed));
disp("Kept class counts:");
disp(countcats(Yclass));

disp("Failure reasons (grouped):");
fr = failReason(failed);
if isempty(fr)
    disp("(none)");
else
    [grp,~,idx] = unique(fr);
    counts = accumarray(idx,1);
    [counts,ord] = sort(counts,'descend');
    disp(table(grp(ord), counts, 'VariableNames', {'Reason','Count'}));
end

meta = struct();
meta.model = mdl;
meta.created = datetime("now");
meta.N_requested = N;
meta.total_line_km = total_line_km;
meta.Yclass_planned = Yclass_str;
meta.failed_mask = failed;
meta.fail_reason = failReason;
meta.channel_order = ["VpA","IpA","VnA","InA","VpB","IpB","VnB","InB"];
meta.note = "v3 none-safe + no leading-dot chaining. Model never receives fault_type='none'.";

save(finalFile, "Xloc","Yclass","Yloc_km","Yloc_norm","meta","-v7.3");
fprintf("Saved: %s\n", finalFile);

end

%% ================= HELPERS =================

function logs = getLogsOrThrowRealError(simOut)
if ~isa(simOut,"Simulink.SimulationOutput")
    error("Not a Simulink.SimulationOutput.");
end

names = string(simOut.who);

% Surface error diagnostics if present
if any(names=="ErrorMessage")
    em = string(simOut.ErrorMessage);
    if strlength(em) > 0
        error("Sim error: %s", em);
    end
end

% Some versions store errors here
if any(names=="SimulationMetadata")
    try
        md = simOut.SimulationMetadata;
        if isfield(md,"ExecutionInfo")
            ei = md.ExecutionInfo;
            if isfield(ei,"ErrorDiagnostic") && ~isempty(ei.ErrorDiagnostic)
                error("Sim error diagnostic: %s", string(ei.ErrorDiagnostic));
            end
            if isfield(ei,"ErrorMessage") && ~isempty(ei.ErrorMessage)
                error("Sim error message: %s", string(ei.ErrorMessage));
            end
        end
    catch
        % ignore
    end
end

if any(names=="logsout")
    logs = simOut.logsout;
    if isempty(logs); error("logsout exists but is empty."); end
    return
end

if any(names=="yout")
    logs = simOut.yout;
    if isempty(logs); error("yout exists but is empty."); end
    return
end

if numel(names)==0
    error("No logsout/yout found; SimulationOutput has no variables (sim likely failed at init/start).");
else
    error("No logsout/yout found. Available: %s", strjoin(names,", "));
end
end

function sig = resolveSigNames(logs, SIG_CANDIDATES)
names = string(logs.getElementNames);
fields = ["VpA","IpA","VnA","InA","VpB","IpB","VnB","InB"];
sig = struct();

for f = fields
    candidates = string(SIG_CANDIDATES.(f));
    hit = "";
    for c = candidates
        if any(names == c); hit = c; break; end
    end
    if hit == ""
        for c = candidates
            mask = contains(lower(names), lower(c));
            if any(mask)
                hit = names(find(mask,1,"first"));
                break;
            end
        end
    end
    if hit == ""
        ex = names(1:min(30,numel(names)));
        error("Missing required signal for %s. Tried: %s. Example available: %s", ...
            f, strjoin(candidates,", "), strjoin(ex,", "));
    end
    sig.(f) = char(hit);
end
end

function X = extractWindow8_by_time_robust(logs, sig, t0, t1, ds)
VpA_ts = logs.get(sig.VpA).Values; IpA_ts = logs.get(sig.IpA).Values;
VnA_ts = logs.get(sig.VnA).Values; InA_ts = logs.get(sig.InA).Values;
VpB_ts = logs.get(sig.VpB).Values; IpB_ts = logs.get(sig.IpB).Values;
VnB_ts = logs.get(sig.VnB).Values; InB_ts = logs.get(sig.InB).Values;

tmin = max([VpA_ts.Time(1), IpA_ts.Time(1), VnA_ts.Time(1), InA_ts.Time(1), ...
            VpB_ts.Time(1), IpB_ts.Time(1), VnB_ts.Time(1), InB_ts.Time(1)]);
tmax = min([VpA_ts.Time(end), IpA_ts.Time(end), VnA_ts.Time(end), InA_ts.Time(end), ...
            VpB_ts.Time(end), IpB_ts.Time(end), VnB_ts.Time(end), InB_ts.Time(end)]);

t0c = max(t0, tmin);
t1c = min(t1, tmax);
if t1c <= t0c
    error("Requested [%.6f,%.6f] outside overlap [%.6f,%.6f].", t0, t1, tmin, tmax);
end

VpA = getsampleusingtime(VpA_ts, t0c, t1c);
IpA = getsampleusingtime(IpA_ts, t0c, t1c);
VnA = getsampleusingtime(VnA_ts, t0c, t1c);
InA = getsampleusingtime(InA_ts, t0c, t1c);

VpB = getsampleusingtime(VpB_ts, t0c, t1c);
IpB = getsampleusingtime(IpB_ts, t0c, t1c);
VnB = getsampleusingtime(VnB_ts, t0c, t1c);
InB = getsampleusingtime(InB_ts, t0c, t1c);

n = min([numel(VpA.Time),numel(IpA.Time),numel(VnA.Time),numel(InA.Time), ...
         numel(VpB.Time),numel(IpB.Time),numel(VnB.Time),numel(InB.Time)]);
if n < 50
    error("Too few samples extracted: n=%d.", n);
end

VpA = VpA.Data(1:n); IpA = IpA.Data(1:n);
VnA = VnA.Data(1:n); InA = InA.Data(1:n);
VpB = VpB.Data(1:n); IpB = IpB.Data(1:n);
VnB = VnB.Data(1:n); InB = InB.Data(1:n);

if ds > 1
    idx = 1:ds:n;
    VpA = VpA(idx); IpA = IpA(idx);
    VnA = VnA(idx); InA = InA(idx);
    VpB = VpB(idx); IpB = IpB(idx);
    VnB = VnB(idx); InB = InB(idx);
end

X = [VpA, IpA, VnA, InA, VpB, IpB, VnB, InB];
end

function X = extractWindow8_onset_robust8(logs, sig, t_fault, win_pre, win_post, search_pre, search_post, z_k, min_hold, ds)
tS0 = t_fault - search_pre;
tS1 = t_fault + search_post;

VpA_ts = getsampleusingtime(logs.get(sig.VpA).Values, tS0, tS1);
IpA_ts = getsampleusingtime(logs.get(sig.IpA).Values, tS0, tS1);
VnA_ts = getsampleusingtime(logs.get(sig.VnA).Values, tS0, tS1);
InA_ts = getsampleusingtime(logs.get(sig.InA).Values, tS0, tS1);
VpB_ts = getsampleusingtime(logs.get(sig.VpB).Values, tS0, tS1);
IpB_ts = getsampleusingtime(logs.get(sig.IpB).Values, tS0, tS1);
VnB_ts = getsampleusingtime(logs.get(sig.VnB).Values, tS0, tS1);
InB_ts = getsampleusingtime(logs.get(sig.InB).Values, tS0, tS1);

n = min([numel(VpA_ts.Time), numel(IpA_ts.Time), numel(VnA_ts.Time), numel(InA_ts.Time), ...
         numel(VpB_ts.Time), numel(IpB_ts.Time), numel(VnB_ts.Time), numel(InB_ts.Time)]);

if n < 50
    t_onset = t_fault;
    t0 = t_onset - win_pre;
    t1 = t_onset + win_post;
    X = extractWindow8_by_time_robust(logs, sig, t0, t1, ds);
    return
end

t = VpA_ts.Time(1:n);
VpA = VpA_ts.Data(1:n); IpA = IpA_ts.Data(1:n);
VnA = VnA_ts.Data(1:n); InA = InA_ts.Data(1:n);
VpB = VpB_ts.Data(1:n); IpB = IpB_ts.Data(1:n);
VnB = VnB_ts.Data(1:n); InB = InB_ts.Data(1:n);

E = abs(diff(VpA)) + abs(diff(IpA)) + abs(diff(VnA)) + abs(diff(InA)) + ...
    abs(diff(VpB)) + abs(diff(IpB)) + abs(diff(VnB)) + abs(diff(InB));
tE = t(2:end);

preMask = (tE < (t_fault - 0.1e-3));
if nnz(preMask) < 30
    preMask = false(size(tE));
    preMask(1:max(10, floor(0.2*numel(tE)))) = true;
end

base = E(preMask);
medE = median(base);
madE = median(abs(base - medE));
sigma = 1.4826 * madE;
if sigma <= 0
    sigma = std(base);
    if sigma <= 0; sigma = 1; end
end

thr = medE + z_k * sigma;
idx = find_consecutive(E > thr, min_hold);

if isempty(idx)
    t_onset = t_fault;
else
    t_onset = tE(idx);
end

t0 = t_onset - win_pre;
t1 = t_onset + win_post;
X = extractWindow8_by_time_robust(logs, sig, t0, t1, ds);
end

function idx = find_consecutive(mask, k)
idx = [];
if k <= 1
    f = find(mask,1,"first");
    if ~isempty(f), idx = f; end
    return;
end
m = double(mask(:));
run = conv(m, ones(k,1), "valid");
f = find(run >= k, 1, "first");
if ~isempty(f), idx = f; end
end

function Xn = addMeasurementNoise(X, classLabel, cfg)
Xn = X;
classLabel = string(classLabel);
sigma_scale = cfg.sigma_scale_fault;
if classLabel == "none"; sigma_scale = cfg.sigma_scale_none; end

for c = 1:size(X,2)
    s = X(:,c);
    sig = std(s);
    if sig == 0; sig = max(abs(s)); end
    if sig == 0; sig = 1; end
    sigma = cfg.sigma_base * sigma_scale * sig;
    Xn(:,c) = s + sigma*randn(size(s)) + cfg.bias_frac*sig*(2*rand - 1);
end
end