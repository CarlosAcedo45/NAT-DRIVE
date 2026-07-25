% This script                                                           displays a frame-by-frame Range-Doppler heatmap. On designated frames,
% execution pauses so the user can left-click targets to extract and save
% their Range, Velocity, and Power to a CSV file.

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
makeVideo    = true;                      % write output video to disk
outVideoName = 'RD_ClickExport_Mesa8.mp4'; % output video filename
vidFPS       = round(1/framePeriod_sec);  % video frame rate (fps)
frameSkip    = 1;                         % process every Nth frame (1 = all frames)
useDbScale   = true;                      % display power in dB
dBfloor      = -25;                       % minimum dB value shown on colormap
dynRangeDB   = 25;                        % colormap dynamic range per frame (dB)

%% -------------------- INTERACTIVE CLICK --------------------
interactiveMode = true; % enable interactive clicking

% Frames on which execution pauses for manual clicking.
%   Specific frames : [100, 200, 500]
%   Every Nth frame : 1:25:4500
%   ALL frames      : []
clickFrames = [];   % <-- EDIT THIS

if ~isempty(clickFrames)
    clickFrames = unique(clickFrames(:));
    clickFrames = clickFrames(clickFrames>=1 & clickFrames<=numFrames);
end
clickedCSV = 'ManualClicks_Mesa8.csv'; % output CSV filename for recorded clicks

%% =========================================================
%% 0) Derived axes
%% =========================================================
lambda = c/fc;        % wavelength (m)
B      = slope*Tramp; % sweep bandwidth (Hz)

freq_axis     = (0:(NfftR/2-1))*(fs/NfftR);          % range-bin frequencies (Hz)
rangeAxis_new = (c*freq_axis(:))/(2*slope);           % range axis (m)
dopplerAxis   = ((lambda/2)*(-numChirps/2:numChirps/2-1)*(1/(numChirps*Tchirp))).'; % velocity axis (m/s)
dopplerAxis   = dopplerAxis.';

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
%% 2) ROI masks
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
%% 3) Click storage + emergency save
%% =========================================================
manualClicks = zeros(0,4); % stores [frameIdx, Range_m, Velocity_ms, Power_dB] per click
clickCount   = 0;          % running count of recorded clicks
cleanupObj   = onCleanup(@() emergencySave(manualClicks, clickedCSV)); % saves clicks on crash or Ctrl-C

%% =========================================================
%% 4) Figure + video
%% =========================================================
if makeVideo
    vw = VideoWriter(outVideoName,'MPEG-4'); % video writer object
    vw.FrameRate = vidFPS;  open(vw);

    figW=1200; figH=880; % figure width and height (pixels)
    hFig = figure('Color','w','Position',[100 100 figW figH], ...
        'MenuBar','none','ToolBar','none','Resize','off','InvertHardcopy','off', ...
        'Name','RD Viewer  |  Left-click=read   Right-click/key=next frame');

    hInfoBox = uicontrol(hFig,'Style','text', ...
        'Units','normalized','Position',[0.02 0.005 0.96 0.038], ...
        'FontSize',10,'FontWeight','bold','HorizontalAlignment','left', ...
        'BackgroundColor','w','ForegroundColor',[0 0.45 0],'String','Initialising...'); % status bar at bottom

    hAx = axes('Parent',hFig,'Units','normalized','Position',[0.11 0.11 0.67 0.81], ...
        'LineWidth',1.2,'FontSize',12,'Box','on','Layer','top','Color','w'); % main plot axes
    hold(hAx,'on'); grid(hAx,'on'); colormap(hAx,'hot');

    % Compute first-frame RD map to seed the image object
    fw0  = adcData(:,:,1).*w_fast;                         % windowed ADC frame 1
    rf0  = fft(fw0,NfftR,1);  rf0=rf0(1:NfftR/2,:);       % range FFT (one-sided)
    RD0  = abs(fftshift(fft(rf0,numChirps,2),2)).^2;       % Doppler FFT -> power
    RD0(~rROI,:)=0; RD0(:,~dROI)=0;
    hIm  = imagesc(hAx,dopplerAxis,rangeAxis_new,toDisplay(RD0,useDbScale,dBfloor)); % image handle
    axis(hAx,'xy');  clear fw0 rf0 RD0;

    xlabel(hAx,'Velocity (m/s)','FontSize',13,'FontWeight','bold','Color','k');
    ylabel(hAx,'Range (m)',     'FontSize',13,'FontWeight','bold','Color','k');
    title(hAx, 'Range-Doppler Map','FontSize',14,'FontWeight','bold','Color','k');

    hCb=colorbar(hAx); hCb.Units='normalized'; hCb.Position=[0.82 0.11 0.028 0.81]; % colorbar handle
    hCb.Color='k'; hCb.Label.String='Power (dB)'; hCb.Label.Color='k';
    hCb.Label.FontSize=12; hCb.FontSize=11;

    if useROI, xlim(hAx,[VminROI VmaxROI]); ylim(hAx,[RminROI RmaxROI]); end

    hClickMark = plot(hAx,nan,nan,'g+','MarkerSize',22,'LineWidth',2.8); % crosshair marker at clicked point
    hClickText = text(hAx,nan,nan,'','Color',[0 0.75 0],'FontSize',10,'FontWeight','bold', ...
        'VerticalAlignment','top','HorizontalAlignment','left'); % readout label at clicked point

    drawnow;
    fr0  = getframe(hFig);    % capture frame to establish video dimensions
    vidH = size(fr0.cdata,1); % video frame height (pixels)
    vidW = size(fr0.cdata,2); % video frame width (pixels)
    clear fr0;

    if interactiveMode
        if isempty(clickFrames), fprintf('Pausing on EVERY frame.\n');
        else, fprintf('Pausing on %d frame(s).\n',numel(clickFrames)); end
    end
end

%% =========================================================
%% 5) Frame loop
%% =========================================================
for j = 1:frameSkip:numFrames

    % --- Per-frame Range-Doppler map ---
    frameW = adcData(:,:,j).*w_fast;                        % windowed chirp matrix for frame j
    rFFT   = fft(frameW,NfftR,1);  rFFT=rFFT(1:NfftR/2,:); % range FFT
    RDroi  = abs(fftshift(fft(rFFT,numChirps,2),2)).^2;    % Doppler FFT -> power map
    clear frameW rFFT;
    RDroi(~rROI,:)=0; RDroi(:,~dROI)=0; % apply ROI mask

    % Determine whether to pause on this frame for clicking
    pauseThisFrame = interactiveMode && (isempty(clickFrames)||ismember(j,clickFrames)); % true if frame is in click list

    if makeVideo && ishandle(hFig)
        RDdisp = toDisplay(RDroi,useDbScale,dBfloor); % dB-scaled display matrix
        set(hIm,'CData',RDdisp);
        rdMax = max(RDdisp(:));                        % peak power in current frame (dB)
        clim(hAx,[rdMax-dynRangeDB, rdMax]);           % per-frame dynamic colour limits

        if pauseThisFrame
            title(hAx, sprintf('Frame %d/%d   %s   [ PAUSED ]', ...
                j,numFrames,datestr(timeVec(j),'HH:MM:SS.FFF')), ...
                'FontSize',14,'FontWeight','bold','Color','k');
            set(hInfoBox,'String', ...
                sprintf('PAUSED  Frame %d/%d  |  LEFT-CLICK to read   |   RIGHT-CLICK or KEY → next frame',j,numFrames), ...
                'ForegroundColor',[0 0.45 0]);
        else
            title(hAx, sprintf('Frame %d/%d   %s', ...
                j,numFrames,datestr(timeVec(j),'HH:MM:SS.FFF')), ...
                'FontSize',14,'FontWeight','bold','Color','k');
            if ~isempty(clickFrames)
                nxt=clickFrames(find(clickFrames>j,1)); % next upcoming pause frame
                if ~isempty(nxt), msg=sprintf('Playing...  Frame %d/%d  |  Next pause: frame %d',j,numFrames,nxt);
                else,             msg=sprintf('Playing...  Frame %d/%d  |  No more pause frames',j,numFrames); end
            else
                msg=sprintf('Playing...  Frame %d/%d',j,numFrames);
            end
            set(hInfoBox,'String',msg,'ForegroundColor',[0.4 0.4 0.4]);
        end

        set(hClickMark,'XData',nan,'YData',nan);         % reset crosshair at start of each frame
        set(hClickText,'Position',[nan nan 0],'String',''); % reset readout label

        if pauseThisFrame
            drawnow;
            xlims=xlim(hAx); ylims=ylim(hAx); % current axis limits used to reject out-of-bounds clicks
            while ishandle(hFig)
                result = waitforbuttonpress; % 0 = mouse click, 1 = key press
                if ~ishandle(hFig), break; end
                if result==1, break; end                                            % any key -> next frame
                if ~strcmp(get(hFig,'SelectionType'),'normal'), break; end          % right-click -> next frame

                cp = get(hAx,'CurrentPoint');  % cursor position in axes data coordinates
                xC = cp(1,1);                  % clicked velocity (m/s)
                yC = cp(1,2);                  % clicked range (m)
                if xC<xlims(1)||xC>xlims(2)||yC<ylims(1)||yC>ylims(2), continue; end % ignore out-of-axes clicks

                [~,rIdx] = min(abs(rangeAxis_new-yC)); % nearest range bin index
                [~,dIdx] = min(abs(dopplerAxis  -xC)); % nearest Doppler bin index
                R_c = rangeAxis_new(rIdx); % snapped range value (m)
                V_c = dopplerAxis(dIdx);   % snapped velocity value (m/s)
                P_c = 10*log10(RDroi(rIdx,dIdx)+1e-12); % power at clicked cell (dB)
                clickCount = clickCount+1;

                set(hClickMark,'XData',V_c,'YData',R_c);
                set(hClickText,'Position',[V_c,R_c,0], ...
                    'String',sprintf('  R=%.2fm\n  V=%.2fm/s\n  P=%.1fdB',R_c,V_c,P_c));
                set(hInfoBox,'String', ...
                    sprintf('Click #%d | Frame %d | R: %.2f m  V: %.2f m/s  P: %.1f dB  |  RIGHT-CLICK/KEY → next', ...
                        clickCount,j,R_c,V_c,P_c),'ForegroundColor',[0.75 0.1 0.0]);
                fprintf('[Frame %4d]  Click #%3d  ->  R = %6.2f m  |  V = %7.2f m/s  |  P = %6.1f dB\n', ...
                    j,clickCount,R_c,V_c,P_c);
                manualClicks(end+1,:) = [j,R_c,V_c,P_c]; %#ok<SAGROW>
                drawnow;
            end
            if ~ishandle(hFig), break; end
        else
            drawnow limitrate; % flush display; skip if screen refresh not due yet
        end

        fr  = getframe(hFig); img=fr.cdata; % captured video frame
        if size(img,1)~=vidH||size(img,2)~=vidW, img=imresize(img,[vidH vidW]); end
        writeVideo(vw,img); clear fr img;
    end
end

if makeVideo && isvalid(vw), close(vw); fprintf('Saved: %s\n',outVideoName); end
clear cleanupObj;

%% =========================================================
%% 6) Save clicks to CSV
%% =========================================================
if ~isempty(manualClicks)
    T = table((1:size(manualClicks,1)).',manualClicks(:,1),manualClicks(:,2), ...
              manualClicks(:,3),manualClicks(:,4), ...
        'VariableNames',{'ClickIndex','FrameIdx','Range_m','Velocity_ms','Power_dB'});
    writetable(T,clickedCSV);
    fprintf('Saved %d click(s) to: %s\n',size(manualClicks,1),clickedCSV);
else
    disp('No clicks recorded.');
end

%% ============================ LOCAL FUNCTIONS ============================

function emergencySave(clicks,csvName)
% Saves recorded clicks to an emergency CSV if the script exits unexpectedly.
    if isempty(clicks), return; end
    f = strrep(csvName,'.csv','_EMERGENCY.csv');
    try
        T = table((1:size(clicks,1)).',clicks(:,1),clicks(:,2),clicks(:,3),clicks(:,4), ...
            'VariableNames',{'ClickIndex','FrameIdx','Range_m','Velocity_ms','Power_dB'});
        writetable(T,f);
        fprintf('[EMERGENCY SAVE] %s\n',f);
    catch, fprintf('[EMERGENCY SAVE FAILED]\n'); end
end

function RDdisp = toDisplay(RD,useDbScale,dBfloor)
% Converts a linear power map to a dB-scaled display matrix clipped at dBfloor.
    if useDbScale, RDdisp=max(10*log10(RD+1e-12),dBfloor);
    else,          RDdisp=RD; end
end
