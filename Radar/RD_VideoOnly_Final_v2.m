% This script generates a frame-by-frame Range-Doppler video 
% (for a FMCW radar) to match against an actual dash-cam recorded video

clear; clc; close all;

%% -------------------- USER SETTINGS --------------------
adcFilename   = 'C:\Users\sahmed7\Prof. Davi\TI Radar\EV Data\03_22_26\Mesa8 _Raw_0.bin'; % path to raw ADC binary file

numADCBits    = 16;   % ADC bit depth
numLanes      = 1;    % number of LVDS lanes
numADCSamples = 256;  % ADC samples per chirp
numRx         = 1;    % number of receive antennas
numChirps     = 128;  % chirps per frame
numFrames     = 4500; % total number of frames to process

Tchirp = 90e-6;       % chirp repetition interval (s)
Tramp  = 60e-6;       % ramp duration (s)
slope  = 8.35e12;     % FMCW frequency slope (Hz/s)
fs     = 1e7;         % ADC sampling rate (Hz)
fc     = 77.2505e9;   % carrier frequency (Hz)
c      = 3e8;         % speed of light (m/s)

framePeriod_sec = 0.04; % wall-clock duration of one frame (s); 25 Hz
NfftR           = 1024; % range FFT size (zero-padded)

%% -------------------- ROI --------------------
useROI  = true;  % enable region-of-interest masking
RminROI = 2;     % minimum range to display (m)
VminROI = [];    % minimum velocity (empty = use full axis)
VmaxROI = [];    % maximum velocity (empty = use full axis)

%% -------------------- VIDEO --------------------
outVideoName = 'RD_VideoOnly_Mesa8.mp4'; % output video filename
vidFPS       = round(1/framePeriod_sec); % video frame rate (fps)
frameSkip    = 1;                        % process every Nth frame (1 = all frames)
useDbScale   = true;                     % display power in dB
dBfloor      = -25;                      % minimum dB value shown on colormap
dynRangeDB   = 25;                       % colormap dynamic range per frame (dB)

%% =========================================================
%% 0) Derived axes
%% =========================================================
lambda = c/fc;          % wavelength (m)
B      = slope*Tramp;   % sweep bandwidth (Hz)

freq_axis     = (0:(NfftR/2-1))*(fs/NfftR);          % range-bin frequencies (Hz)
rangeAxis_new = (c*freq_axis(:))/(2*slope);           % range axis (m)
dopplerAxis   = (lambda/2)*(-numChirps/2:numChirps/2-1)*(1/(numChirps*Tchirp)); % velocity axis (m/s)
dopplerAxis   = dopplerAxis(:).';

Nr = NfftR/2;    % number of range bins
Nd = numChirps;  % number of Doppler bins

fprintf('RangeRes=%.3f m | Rmax=%.2f m | vmax=%.2f m/s\n', ...
    c/(2*B), (fs*c)/(4*slope), lambda/(4*Tchirp));

if useROI
    RmaxROI = min(80,rangeAxis_new(end)); % maximum range to display (m)
    if isempty(VminROI), VminROI=min(dopplerAxis); end
    if isempty(VmaxROI), VmaxROI=max(dopplerAxis); end
end

%% =========================================================
%% 1) ADC load
%% =========================================================
adcRaw  = readDCA100(adcFilename,numADCBits,numLanes,numADCSamples,numRx,numChirps*numFrames); % raw interleaved ADC samples
adc2D   = adcRaw.';                                                % [totalChirps x samples]
assert(size(adc2D,2)==numADCSamples);
assert(size(adc2D,1)>=numChirps*numFrames);
adcData = reshape(adc2D(1:numChirps*numFrames,:).', numADCSamples, numChirps, numFrames); % [samples x chirps x frames]
clear adcRaw adc2D;
fprintf('ADC: [%d x %d x %d]\n', size(adcData,1), size(adcData,2), size(adcData,3));

w_fast = hann(numADCSamples,'periodic'); % Hann window applied along fast-time (range) dimension

%% =========================================================
%% 2) ROI masks + time vector
%% =========================================================
rROI = true(Nr,1);   % range bin mask (logical index into rangeAxis_new)
dROI = true(1,Nd);   % Doppler bin mask (logical index into dopplerAxis)
if useROI
    rROI = (rangeAxis_new>=RminROI) & (rangeAxis_new<=RmaxROI);
    dROI = (dopplerAxis  >=VminROI) & (dopplerAxis  <=VmaxROI);
end

startTime = datetime(2026,03,22,12,52,54); % recording start time
timeVec   = startTime + (0:numFrames-1)*seconds(framePeriod_sec); % per-frame timestamps

%% =========================================================
%% 3) Figure + video init
%% =========================================================
vw = VideoWriter(outVideoName,'MPEG-4'); % video writer object
vw.FrameRate = vidFPS;  open(vw);
cleanupObj = onCleanup(@() safeCloseVideo(vw)); % closes video safely on crash or Ctrl-C

figW=1200; figH=880; % figure width and height (pixels)
hFig = figure('Color','w','Position',[100 100 figW figH], ...
    'MenuBar','none','ToolBar','none','Resize','off','InvertHardcopy','off');

hAx = axes('Parent',hFig,'Units','normalized','Position',[0.11 0.11 0.67 0.81], ...
    'LineWidth',1.2,'FontSize',12,'Box','on','Layer','top','Color','w'); % main plot axes
hold(hAx,'on'); grid(hAx,'on'); colormap(hAx,'hot');

% Compute first-frame RD map to seed the image object
fw0 = adcData(:,:,1).*w_fast;                          % windowed ADC frame 1
rf0 = fft(fw0,NfftR,1);  rf0=rf0(1:NfftR/2,:);        % range FFT (one-sided)
RD0 = abs(fftshift(fft(rf0,numChirps,2),2)).^2;        % Doppler FFT -> power
RD0(~rROI,:)=0; RD0(:,~dROI)=0;
hIm = imagesc(hAx,dopplerAxis,rangeAxis_new,toDisplay(RD0,useDbScale,dBfloor)); % image handle
axis(hAx,'xy'); clear fw0 rf0 RD0;

xlabel(hAx,'Velocity (m/s)','FontSize',13,'FontWeight','bold','Color','k');
ylabel(hAx,'Range (m)',     'FontSize',13,'FontWeight','bold','Color','k');
title(hAx, 'Range-Doppler Map','FontSize',14,'FontWeight','bold','Color','k');

hCb=colorbar(hAx); hCb.Units='normalized'; hCb.Position=[0.82 0.11 0.028 0.81]; % colorbar handle
hCb.Color='k'; hCb.Label.String='Power (dB)'; hCb.Label.Color='k';
hCb.Label.FontSize=12; hCb.FontSize=11;

if useROI, xlim(hAx,[VminROI VmaxROI]); ylim(hAx,[RminROI RmaxROI]); end

drawnow;
fr0  = getframe(hFig);           % capture frame to establish video dimensions
vidH = size(fr0.cdata,1);        % video frame height (pixels)
vidW = size(fr0.cdata,2);        % video frame width (pixels)
clear fr0;

%% =========================================================
%% 4) Frame loop
%% =========================================================
for j = 1:frameSkip:numFrames

    % --- Per-frame Range-Doppler map ---
    frameW = adcData(:,:,j).*w_fast;                        % windowed chirp matrix for frame j
    rFFT   = fft(frameW,NfftR,1); rFFT=rFFT(1:NfftR/2,:); % range FFT
    RDroi  = abs(fftshift(fft(rFFT,numChirps,2),2)).^2;    % Doppler FFT -> power map
    clear frameW rFFT;
    RDroi(~rROI,:)=0; RDroi(:,~dROI)=0; % apply ROI mask

    if ishandle(hFig)
        RDdisp = toDisplay(RDroi,useDbScale,dBfloor); % dB-scaled display matrix
        set(hIm,'CData',RDdisp);
        clim(hAx,[max(RDdisp(:))-dynRangeDB, max(RDdisp(:))]); % per-frame dynamic colour limits

        title(hAx,sprintf('Range-Doppler (Frame %d/%d)  %s', ...
            j,numFrames,datestr(timeVec(j),'HH:MM:SS.FFF')), ...
            'FontSize',14,'FontWeight','bold','Color','k');

        drawnow limitrate; % flush display; skip if screen refresh not due yet
        fr  = getframe(hFig); img=fr.cdata; % captured video frame
        if size(img,1)~=vidH||size(img,2)~=vidW, img=imresize(img,[vidH vidW]); end
        writeVideo(vw,img); clear fr img;
    end
end

clear cleanupObj;
if isvalid(vw), close(vw); end
fprintf('Saved: %s\n',outVideoName);

%% ============================ LOCAL FUNCTIONS ============================

function safeCloseVideo(vw)
% Closes the VideoWriter object safely; called by onCleanup on any exit.
    try, if isvalid(vw), close(vw); end; catch, end
end

function RDdisp = toDisplay(RD,useDbScale,dBfloor)
% Converts a linear power map to a dB-scaled display matrix clipped at dBfloor.
    if useDbScale, RDdisp=max(10*log10(RD+1e-12),dBfloor);
    else,          RDdisp=RD; end
end
