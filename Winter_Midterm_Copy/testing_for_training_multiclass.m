%% train_hvdc_loc_3view_cnn_from_Xloc.m
% Trains a fault-location regressor using your dataset format:
%   Xloc: cell{N,1} each [T x 8] single (time x channels), variable T
%   Yloc_norm: [N x 1] (0..1)
%   (optional) meta with total_line_km, tag, etc.
%
% Looks ONE DIRECTORY FORWARD for batch files:
%   <pwd>\dataset_out\dsTransient_*.mat
%
% Builds 12 channels by adding physics-inspired differences:
%   dVA=VpA-VnA, dIA=IpA-InA, dVB=VpB-VnB, dIB=IpB-InB
%
% Then builds 3 "views" (depth slices) per sample:
%   view1 = standardized signals
%   view2 = time-derivative of standardized signals
%   view3 = smoothed standardized signals (moving average)
%
% Final input per sample: [Tfixed x 12 x 3]
% Network: 2D CNN over (time x features) with 3 slices as channels.

clear; clc;
rng(1);

%% =========================
% SETTINGS (EDIT THESE)
%% =========================
total_line_km = 546;        % line length (km) - used only for reporting km error

% Choose a fixed length (samples). If empty => auto median length across all samples.
Tfixed = [];

% How to handle variable-length sequences:
cropMode = "crop_center";   % "crop_center" or "crop_start"

% Train/val/test split
trainFrac = 0.70;
valFrac   = 0.15;

% Training hyperparameters
maxEpochs     = 40;
miniBatchSize = 32;
initialLR     = 1e-3;

% View construction params
movMeanWin     = 9;         % smoothing window (samples)
useAbsForDeriv = false;     % abs(d/dt) if true

% Which fault types to include (set empty to include all)
useTags = ["pn","pg","ng"]; % uses meta.tag; set to [] to ignore filtering

%% =========================
% FIND DATA FILES (ONE DIR FORWARD)
%% =========================
dataFiles = dir(fullfile(pwd, "dataset_out", "dsTransient_*.mat"));
if isempty(dataFiles)
    error("No dsTransient_*.mat found in: %s", fullfile(pwd,"dataset_out"));
end
fprintf("Found %d batch files.\n", numel(dataFiles));

%% =========================
% LOAD + CONCAT ALL SAMPLES
%% =========================
X = {};          % cell of [8 x T] (features x time)
Y = [];          % normalized labels [0..1]
tags = strings(0,1);

for i = 1:numel(dataFiles)
    fpath = fullfile(dataFiles(i).folder, dataFiles(i).name);

    S = load(fpath, "Xloc", "Yloc_norm", "meta");  % matches your generator

    if ~isfield(S,"Xloc") || isempty(S.Xloc)
        warning("Skipping %s (missing Xloc).", dataFiles(i).name);
        continue;
    end
    if ~isfield(S,"Yloc_norm") || isempty(S.Yloc_norm)
        warning("Skipping %s (missing Yloc_norm).", dataFiles(i).name);
        continue;
    end

    Xloc = S.Xloc;
    Yloc = double(S.Yloc_norm(:));

    % Optional tag filtering
    if isfield(S,"meta") && isfield(S.meta,"tag")
        tag = string(S.meta.tag);
    else
        tag = "";
    end

    % If we filter by tag and this file doesn't match, skip
    if ~isempty(useTags) && tag ~= "" && ~any(tag == useTags)
        continue;
    end

    % Remove empties
    keep = ~cellfun(@isempty, Xloc) & ~isnan(Yloc);
    Xloc = Xloc(keep);
    Yloc = Yloc(keep);

    % Convert each sample to [8 x T] (features x time)
    for k = 1:numel(Xloc)
        xk = Xloc{k};                % [T x 8] expected
        if size(xk,2) == 8
            xk = xk.';               % -> [8 x T]
        elseif size(xk,1) == 8
            % already [8 x T]
        else
            warning("Skipping a sample in %s due to unexpected size %dx%d", dataFiles(i).name, size(xk,1), size(xk,2));
            continue;
        end
        X{end+1,1} = single(xk); %#ok<AGROW>
    end

    Y = [Y; Yloc]; %#ok<AGROW>

    if tag == ""
        tags = [tags; repmat("unknown", numel(Yloc), 1)]; %#ok<AGROW>
    else
        tags = [tags; repmat(tag, numel(Yloc), 1)]; %#ok<AGROW>
    end

    fprintf("Loaded %s | kept %d samples | tag=%s\n", dataFiles(i).name, numel(Xloc), tag);
end

N = numel(X);
if N == 0
    error("Loaded 0 usable samples. Check that dsTransient_*.mat contains Xloc and Yloc_norm.");
end
fprintf("\nTotal samples loaded: %d\n", N);

%% =========================
% CHOOSE Tfixed (MEDIAN) IF NOT SET
%% =========================
lens = cellfun(@(a) size(a,2), X); % T per sample
if isempty(Tfixed)
    Tfixed = round(median(lens));
    fprintf("Auto Tfixed = %d (median length)\n", Tfixed);
else
    fprintf("Using Tfixed = %d\n", Tfixed);
end

%% =========================
% CROP/PAD -> ALL [8 x Tfixed]
%% =========================
X = cellfun(@(a) cropOrPadFeatTime(a, Tfixed, cropMode), X, "UniformOutput", false);

%% =========================
% ADD DIFF CHANNELS -> [12 x Tfixed]
%% =========================
X = cellfun(@addDiffChannels_12, X, "UniformOutput", false);

%% =========================
% SPLIT TRAIN/VAL/TEST
%% =========================
idx = randperm(N);

nTrain = max(1, round(trainFrac*N));
nVal   = max(1, round(valFrac*N));
nTest  = N - nTrain - nVal;
if nTest < 1
    nTest = 1;
    nVal = max(1, nVal-1);
    nTrain = N - nVal - nTest;
end

iTrain = idx(1:nTrain);
iVal   = idx(nTrain+1:nTrain+nVal);
iTest  = idx(nTrain+nVal+1:end);

XTrain = X(iTrain); YTrain = Y(iTrain);
XVal   = X(iVal);   YVal   = Y(iVal);
XTest  = X(iTest);  YTest  = Y(iTest);

fprintf("Split: train=%d val=%d test=%d\n", numel(XTrain), numel(XVal), numel(XTest));

%% =========================
% STANDARDIZE (TRAIN ONLY) - STREAMING
%% =========================
[xMean, xStd] = computeMeanStdStreaming_12(XTrain);
xStd = xStd + 1e-6;
standardize = @(x) single((x - xMean) ./ xStd);

XTrain = cellfun(standardize, XTrain, "UniformOutput", false);
XVal   = cellfun(standardize, XVal,   "UniformOutput", false);
XTest  = cellfun(standardize, XTest,  "UniformOutput", false);

%% =========================
% BUILD 3-VIEW TENSOR: [Tfixed x 12 x 3 x N]
%% =========================
XTrain4 = build3ViewTensor(XTrain, Tfixed, movMeanWin, useAbsForDeriv);
XVal4   = build3ViewTensor(XVal,   Tfixed, movMeanWin, useAbsForDeriv);
XTest4  = build3ViewTensor(XTest,  Tfixed, movMeanWin, useAbsForDeriv);

fprintf("XTrain tensor size = [%d %d %d %d]\n", size(XTrain4,1), size(XTrain4,2), size(XTrain4,3), size(XTrain4,4));

%% =========================
% NETWORK (2D CNN on time x features; 3 views are channels)
%% =========================
layers = [
    imageInputLayer([Tfixed 12 3], "Name","in", "Normalization","none")

    convolution2dLayer([9 3], 32, "Padding","same", "Name","conv1")
    batchNormalizationLayer("Name","bn1")
    reluLayer("Name","relu1")
    maxPooling2dLayer([2 2], "Stride",[2 2], "Name","pool1")

    convolution2dLayer([7 3], 64, "Padding","same", "Name","conv2")
    batchNormalizationLayer("Name","bn2")
    reluLayer("Name","relu2")
    maxPooling2dLayer([2 2], "Stride",[2 2], "Name","pool2")

    convolution2dLayer([5 3], 128, "Padding","same", "Name","conv3")
    batchNormalizationLayer("Name","bn3")
    reluLayer("Name","relu3")

    globalAveragePooling2dLayer("Name","gap")

    fullyConnectedLayer(64, "Name","fc1")
    reluLayer("Name","relu4")
    dropoutLayer(0.2, "Name","drop")

    fullyConnectedLayer(1, "Name","fc_out")
    regressionLayer("Name","reg")
];

opts = trainingOptions("adam", ...
    "MaxEpochs", maxEpochs, ...
    "MiniBatchSize", miniBatchSize, ...
    "InitialLearnRate", initialLR, ...
    "Shuffle","every-epoch", ...
    "ValidationData",{XVal4, YVal}, ...
    "ValidationFrequency", 25, ...
    "Plots","training-progress", ...
    "Verbose", true);

%% =========================
% TRAIN
%% =========================
net = trainNetwork(XTrain4, YTrain, layers, opts);

%% =========================
% TEST (norm -> km)
%% =========================
Ypred_norm = predict(net, XTest4);

Ypred_km = Ypred_norm(:) * total_line_km;
Ytrue_km = YTest(:)      * total_line_km;

mae  = mean(abs(Ypred_km - Ytrue_km));
rmse = sqrt(mean((Ypred_km - Ytrue_km).^2));

fprintf("\n=== TEST RESULTS (km) ===\n");
fprintf("MAE:  %.2f km\n", mae);
fprintf("RMSE: %.2f km\n", rmse);

figure;
scatter(Ytrue_km, Ypred_km, 25, "filled"); grid on;
xlabel("True location (km)");
ylabel("Predicted location (km)");
title(sprintf("Fault location regressor | MAE=%.2f km RMSE=%.2f km", mae, rmse));
refline(1,0);

%% =========================
% SAVE MODEL (IN dataset_out/trained_models)
%% =========================
modelDir = fullfile(pwd, "dataset_out", "trained_models");
if ~exist(modelDir,"dir"), mkdir(modelDir); end
stamp = datestr(now,"yyyymmdd_HHMMSS");
modelFile = fullfile(modelDir, "hvdc_loc_3view_cnn_" + stamp + ".mat");

model.net = net;
model.xMean = xMean;   % [12 x 1]
model.xStd  = xStd;    % [12 x 1]
model.Tfixed = Tfixed;
model.total_line_km = total_line_km;
model.cropMode = cropMode;
model.movMeanWin = movMeanWin;
model.useAbsForDeriv = useAbsForDeriv;
model.created = datetime("now");
model.channelOrder12 = ["VpA","IpA","VnA","InA","VpB","IpB","VnB","InB","dVA","dIA","dVB","dIB"];

save(modelFile, "model");
fprintf("Saved model: %s\n", modelFile);

%% =======================
% LOCAL HELPERS
%% =======================

function x = cropOrPadFeatTime(x, Tfixed, mode)
% x: [F x T]
F = size(x,1);
T = size(x,2);

if T == Tfixed
    return;
elseif T > Tfixed
    switch mode
        case "crop_center"
            s = floor((T - Tfixed)/2) + 1;
            e = s + Tfixed - 1;
            x = x(:, s:e);
        case "crop_start"
            x = x(:, 1:Tfixed);
        otherwise
            error("Unknown cropMode: %s", mode);
    end
else
    xpad = zeros(F, Tfixed, "like", x);
    xpad(:, 1:T) = x;
    x = xpad;
end
end

function x = addDiffChannels_12(x)
% x: [8 x T] -> [12 x T]
VpA = x(1,:); IpA = x(2,:); VnA = x(3,:); InA = x(4,:);
VpB = x(5,:); IpB = x(6,:); VnB = x(7,:); InB = x(8,:);

dVA = VpA - VnA;
dIA = IpA - InA;
dVB = VpB - VnB;
dIB = IpB - InB;

x = [x; dVA; dIA; dVB; dIB];
end

function [xMean, xStd] = computeMeanStdStreaming_12(Xcell)
% Xcell: each [12 x T]
sumX = zeros(12,1);
sumX2 = zeros(12,1);
count = 0;

for i = 1:numel(Xcell)
    x = double(Xcell{i});
    sumX  = sumX  + sum(x,2);
    sumX2 = sumX2 + sum(x.^2,2);
    count = count + size(x,2);
end

xMean = sumX / count;
varX  = sumX2 / count - xMean.^2;
xStd  = sqrt(max(varX, 1e-12));
end

function X4 = build3ViewTensor(Xcell12, Tfixed, movMeanWin, useAbsForDeriv)
% Returns X4: [Tfixed x 12 x 3 x N]
N = numel(Xcell12);
X4 = zeros(Tfixed, 12, 3, N, "single");

for i = 1:N
    x = Xcell12{i}; % [12 x Tfixed]

    v1 = x; % standardized

    dv = [zeros(12,1,"like",x), diff(x,1,2)];
    if useAbsForDeriv
        dv = abs(dv);
    end

    v3 = movmean(x, movMeanWin, 2);

    X4(:,:,1,i) = v1.'; % [T x 12]
    X4(:,:,2,i) = dv.'; % [T x 12]
    X4(:,:,3,i) = v3.'; % [T x 12]
end
end