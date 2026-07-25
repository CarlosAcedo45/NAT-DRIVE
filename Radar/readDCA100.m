function [adcData] = readDCA100(filename, numADCBits, numLanes, numADCSamples, numRx, numChirps)
% READDCA1000 Reads binary ADC data captured by DCA1000EVM
% Inputs:
%   - filename: path to the .bin file
%   - numADCBits: ADC resolution (usually 12, 14 or 16)
%   - numLanes: Number of LVDS lanes (usually 4)
%   - numADCSamples: samples per chirp per receiver (e.g., 256)
%   - numRx: number of RX antennas (e.g., 4)
%   - numChirps: total chirps captured (e.g., 128)
%
% Output:
%   adcData: [numADCSamples x numChirps x numRx] complex matrix

    %% Read binary file
    fid = fopen(filename, 'r');
    if fid < 0
        error(['Cannot open file: ', filename]);
    end
    rawData = fread(fid, 'int16');
    fclose(fid);

    %% Adjust if ADC bits < 16
    if numADCBits ~= 16
        l_max = 2^(numADCBits-1) - 1;
        rawData(rawData > l_max) = rawData(rawData > l_max) - 2^numADCBits;
    end

    %% Interpret as complex samples
    rawData = reshape(rawData, 2, []); % [I; Q]
    complexData = complex(rawData(1,:), rawData(2,:)); % I + jQ

    %% Reshape based on expected data layout
    totalSamples = numADCSamples * numChirps * numRx;
    if length(complexData) < totalSamples
        error('Not enough data in binary file!');
    end

    % Reshape into [numADCSamples x numChirps x numRx]
    adcData = reshape(complexData(1:totalSamples), numADCSamples, numChirps, numRx);
end