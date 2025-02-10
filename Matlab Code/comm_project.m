%% Analog Modulation - Communication Project
% Super-Heterodyne Receiver Simulation
%% Clearing the workspace
clear; clc; close all;
%% i : Loading signals, Converting from stereo to mono & Padding signals
% Using mean provides a normalized mono representation
[stereo1, fs1] = audioread("C:\Users\Spectra\Downloads\drive-download-20241221T181444Z-001\Short_QuranPalestine.wav");% Load the first audio signal
mono1 = mean(stereo1, 2); % Convert first signal from stereo to mono
[stereo2, fs2] = audioread("C:\Users\Spectra\Downloads\drive-download-20241221T181444Z-001\Short_BBCArabic2.wav");% Load the second audio signal
mono2 = mean(stereo2, 2); % Convert second signal from stereo to mono
% Ensure both signals have the same sampling frequency
if fs1 ~= fs2
    error('Sampling frequencies of the two audio files does not match.');
end
fs = fs1;
% Pad the shorter signal to match the length of the longer signal
maxlength = max(length(mono1), length(mono2));
mono1 = [mono1; zeros(maxlength - length(mono1), 1)];
mono2 = [mono2; zeros(maxlength - length(mono2), 1)];
% Plotting signals in time before modulation
figure ;
% Plot the first signal in time domain
t1 = (0:maxlength-1)'/fs;
subplot(2, 1, 1);
plot(t1, mono1);
xlabel('Time (sec)');
ylabel('Amplitude');
title('Time domain of first signal');
grid on;
% Plot the second signal in time domain before modulation
t2 = (0:maxlength-1)'/fs;
subplot(2, 1, 2);
plot(t2, mono2);
xlabel('Time (sec)');
ylabel('Amplitude');
title('Time domain of second signal');
grid on;
% Plot signals in frequency domain 
figure ;
% Plot the first signal in frequency domain
N1 = length(mono1);
f1 = (-N1/2:N1/2-1) * (fs/N1);
spectrum1 = fftshift(abs(fft(mono1)));
subplot(2, 1, 1);
plot(f1, spectrum1);
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Frequency Domain of first signal');
grid on;
% Plot the second signal in frequency domain
N2 = length(mono2);
f2 = (-N2/2:N2/2-1) * (fs/N2);
spectrum2 = fftshift(abs(fft(mono2)));
subplot(2,1, 2);
plot(f2, spectrum2);
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Frequency Domain of second signal');
grid on;
%% ii : AM Modulation (DSB-SC)
% Defining parameters
fc1 = 100e3; % Carrier frequency for first signal in Hz
df =50e3; % Frequency increment for subsequent signals
bandwidth = 25e3; % bandwidth in Hz 
filterorder = 10;
f_IF = 25e3; % Intermediate Frequency (IF)
wc1 = fc1 + f_IF; % Carrier frequency 𝜔𝑐 + 𝜔𝐼𝐹, where 𝜔𝑐 = 𝜔𝑛 + 𝑛Δ𝐹.
wc2 = fc1+ f_IF + df;
fsmultiplier = 20;
fsnew= fs * fsmultiplier; % New sampling frequency
ts = 1 / fsnew; % Smpling rate
% Resample the signals to increase Fs
mono1_interp = interp(mono1, fsmultiplier);
mono2_interp = interp(mono2, fsmultiplier);
% Generating carriers
N = max(length(mono1_interp), length(mono2_interp));% No need for comparison as they are the same lenght after padding
t = (0:N-1)' * ts;
carrier1 = cos(2 * pi * fc1 * t);
carrier2 = cos(2 * pi * (fc1 + df) * t);
% Modulating signals (DSB-SC)
modulated1 = (mono1_interp .* carrier1);
modulated2 = (mono2_interp .* carrier2);
% Plotting signals in time after modulation
figure ;
% Plot the first signal in time domain
subplot(2, 1, 1);
plot(t, modulated1);
xlabel('Time (sec)');
ylabel('Amplitude');
title('Time domain of first signal after modulation');
grid on;
% Plot the second signal in time domain after modulation
subplot(2, 1, 2);
plot(t, modulated2);
xlabel('Time (sec)');
ylabel('Amplitude');
title('Time domain of second signal after modulation');
grid on;
% Plot signals in frequency domain after modulation
figure ;
% Plot the first signal in frequency domain
modulatedspectrum1 = fftshift(abs(fft(modulated1)));
fmod1 = linspace(-fsnew/2, fsnew/2, length(modulated1)) / 1e3; 
subplot(2, 1, 1);
plot(fmod1, modulatedspectrum1);
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Frequency domain of first modulated signal');
grid on;
% Plot the second signal in frequency domain
modulatedspectrum2 = fftshift(abs(fft(modulated2)));
fmod2 = linspace(-fsnew/2, fsnew/2, length(modulated2)) / 1e3; 
subplot(2,1, 2);
plot(fmod2, modulatedspectrum2);
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Frequency domain of second modulated signal');
grid on;
% Combine modulated signals (FDM)
FDMsignal = modulated1 + modulated2;
% Plot the FDM signal in the time domain
figure;
subplot(2, 1, 1);
plot(t, FDMsignal);
xlabel('Time (sec)');
ylabel('Amplitude');
title('Time domain of FDM signal');
grid on;
% Plot the FDM signal spectrum
FDMspectrum = fftshift(fft(FDMsignal));
frequencies = linspace(-fsnew/2, fsnew/2, length(FDMsignal)) / 1e3; % in kHz
subplot(2, 1, 2);
plot(frequencies, abs(FDMspectrum));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('Frequency domain of FDM signal');
grid on;
%% iii : Wireless Channel & RF stage
% Band-Pass Filter for the first modulated signal
fpass1 = [fc1 - bandwidth/2, fc1 + bandwidth/2]; % Passband range for Signal 1
BPF_RF1 = designfilt('bandpassiir', 'FilterOrder', filterorder, ...
                  'HalfPowerFrequency1', fpass1(1), 'HalfPowerFrequency2', fpass1(2), ...
                  'SampleRate', fsnew);
filteredsignal1 = filter(BPF_RF1, FDMsignal);% Applying BPF to the first modulated signal
filteredsignal1 = filteredsignal1 / max(abs(filteredsignal1));% Normalizing filtered signals
snr1 = -15; % Signal-to-Noise Ratio (SNR) for first signal in dB
noisy_RF1 = awgn(filteredsignal1, snr1, 'measured'); % Add Gaussian noise to Signal 1
%filteredsignal1 = FDMsignal;
% Band-Pass Filter for the second modulated signal
fpass2 = [fc1 + df - bandwidth/2, fc1 + df + bandwidth/2]; % Passband range for Signal 2
BPF_RF2 = designfilt('bandpassiir', 'FilterOrder', filterorder, ...
                  'HalfPowerFrequency1', fpass2(1), 'HalfPowerFrequency2', fpass2(2), ...
                  'SampleRate', fsnew);
filteredsignal2 = filter(BPF_RF2, FDMsignal);% Apply BPF to the second modulated signal
filteredsignal2 = filteredsignal2 / max(abs(filteredsignal2));% Normalize filtered signals
snr2 = 15; % Signal-to-Noise Ratio (SNR) for second signal in dB
noisy_RF2 = awgn(filteredsignal2, snr2, 'measured');% Add Gaussian noise to Signal 2
%filteredsignal2 = FDMsignal;
% Plot signals in time domain after RF-Stage
% Plot the first filtered signal in time domain
figure;
subplot(2, 1, 1);
plot(t, filteredsignal1);
xlabel('Time (sec)');
ylabel('Amplitude');
title('RF-Time domain of first filtered modulated signal');
grid on;
% Plot the second filtered signal in time domain
subplot(2, 1, 2);
plot(t, filteredsignal2);
xlabel('Time (sec)');
ylabel('Amplitude');
title('RF-Time domain of second filtered modulated signal');
grid on;
% Plot signals in frequency domain after RF-Stage
% Plot the spectrum of the first filtered signal
figure;
filteredspectrum1 = fftshift(fft(filteredsignal1));
freqfiltered = linspace(-fsnew/2, fsnew/2, length(filteredsignal1)) / 1e3; % Convert to kHz
subplot(2, 1, 1);
plot(freqfiltered, abs(filteredspectrum1));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('RF-Frequency domain of first filtered modulated signal');
grid on;
% Plot the spectrum of the second filtered signal
filteredspectrum2 = fftshift(fft(filteredsignal2));
freqfiltered = linspace(-fsnew/2, fsnew/2, length(filteredsignal2)) / 1e3; % Convert to kHz
subplot(2, 1, 2);
plot(freqfiltered, abs(filteredspectrum2));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('RF-Frequency domain of second filtered modulated Signal');
grid on;
%% iv : Mixer Stage & IF Stage
% Mixer for the first filtered signal
ifcarrier1 = cos(2 * pi * (wc1 + 1200 )* t);% Remove 1200 to remove offset
ifsignal1 = filteredsignal1 .* ifcarrier1;%shift to IF frequency
% Plot IF first signal before band-pass filter in time domain
figure;
subplot(2, 1, 1);
plot(t, ifsignal1);
xlabel('Time (sec)');
ylabel('Amplitude');
title('IF-Time domain of first signal before BPF');
grid on;
% Plot IF first signal before filtering in frequency domain
IFspectrum1before = fftshift(fft(ifsignal1));
subplot(2, 1, 2);
plot(frequencies, abs(IFspectrum1before));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('IF-Frequency domain of first signal before BPF');
grid on;
ifbw1 = 10e3; % Bandwidth of the IF filter in Hz
% Design the IF Band-Pass Filter
BPF_IF1 = designfilt('bandpassiir', ...
                     'FilterOrder', 6, ... % Example filter order
                     'HalfPowerFrequency1', f_IF - ifbw1/2, ...
                     'HalfPowerFrequency2', f_IF + ifbw1/2, ...
                     'SampleRate', fsnew);
% Apply IF Band-Pass Filter to the first mixed signal
IF_filtered_signal1 = filter(BPF_IF1, ifsignal1);
% Plot IF first signal after band-pass filtering in time domain
figure;
subplot(2, 1, 1);
plot(t, IF_filtered_signal1);
xlabel('Time (sec)');
ylabel('Amplitude');
title('IF-Time domain of first signal after BPF');
grid on;
% Plot IF first signal after filtering in frequency domain
IFspectrum1after = fftshift(fft(IF_filtered_signal1));
subplot(2, 1, 2);
plot(frequencies, abs(IFspectrum1after));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('IF-Frequency domain of first signal after BPF');
grid on;
% Mixer for the second filtered signal 
ifcarrier2 = cos(2 * pi * (wc2 + 200)* t);% Remove 200 to remove offset
IF_signal2 = filteredsignal2 .* ifcarrier2;%shift to IF frequency
% Plot IF Signal 2 before band-pass filtering in time domain
figure;
subplot(2, 1, 1);
plot(t, IF_signal2);
xlabel('Time (sec)');
ylabel('Amplitude');
title('IF-Time domain of second signal before BPF');
grid on;
% Plot IF Signal 2 before band-pass filtering in frequency domain
IF_spectrum2_before = fftshift(fft(IF_signal2));
subplot(2, 1, 2);
plot(frequencies, abs(IF_spectrum2_before));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('IF-Frequency domain of second signal before BPF');
grid on;
% Define parameters for the second IF Band-Pass Filter
ifbw2 = 20e3; % Bandwidth of the IF filter in Hz
% Design the IF Band-Pass Filter for the second signal
BPF_IF2 = designfilt('bandpassiir', ...
                     'FilterOrder', 6, ... % Example filter order
                     'HalfPowerFrequency1', f_IF - ifbw2/2, ...
                     'HalfPowerFrequency2', f_IF + ifbw2/2, ...
                     'SampleRate', fsnew);
% Apply IF Band-Pass Filter to the second mixed signal
IF_filteredsignal2 = filter(BPF_IF2, IF_signal2);
% Plot IF second Signal after band-pass filtering in time domain
figure;
subplot(2, 1, 1);
plot(t, IF_filteredsignal2);
xlabel('Time (sec)');
ylabel('Amplitude');
title('IF-Time domain of second signal after BPF');
grid on;
% Plot IF second Signal after band-pass filtering in frequency domain
IF_spectrum2_after = fftshift(fft(IF_filteredsignal2));
subplot(2, 1, 2);
plot(frequencies, abs(IF_spectrum2_after));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('IF-Frequency domain of second signal after BPF');
grid on;
%% Baseband Detection
% Define Intermediate Frequency (IF) for downconversion
f_IF = 25e3; % Example IF frequency for the downconversion
% Mixer for baseband detection for first signal
baseband_carrier1 = cos(2 * pi * (f_IF) * t); % Carrier for Signal 1
baseband_signal1 = IF_filtered_signal1 .* baseband_carrier1; % Mixing to baseband
% Low-Pass Filter for first signal
LPF1 = designfilt('lowpassiir', ...
                  'filterorder', 8, ...
                  'HalfPowerFrequency', fs / 2, ... % Nyquist frequency of original sampling rate
                  'SampleRate', fsnew);
% Apply LPF to extract baseband signal & normalize the first signal              
baseband_signal1_filtered = filter(LPF1, baseband_signal1);
baseband_signal1_filtered = baseband_signal1_filtered / max(abs(baseband_signal1_filtered));
% Mixer for Baseband Detection for second signal
baseband_carrier2 = cos(2 * pi * (f_IF) * t); % Carrier for Signal 2
baseband_signal2 = IF_filteredsignal2 .* baseband_carrier2; % Mixing to baseband
% Low-Pass Filter for Signal 2
LPF2 = designfilt('lowpassiir', ...
                  'filterorder', 8, ...
                  'HalfPowerFrequency', fs / 2, ... % Nyquist frequency of original sampling rate
                  'SampleRate', fsnew);
% Apply LPF to extract baseband signal & normalize the first signal              
baseband_signal2_filtered = filter(LPF2, baseband_signal2);
baseband_signal2_filtered = baseband_signal2_filtered / max(abs(baseband_signal2_filtered));
% Plot first signal in Baseband in Time Domain
figure;
subplot(2, 1, 1);
x = (0:length(baseband_signal1_filtered)-1)' / fs; % Time vector for original sampling rate
plot(x, baseband_signal1_filtered);
xlabel('Time (sec)');
ylabel('Amplitude');
title('First signal in baseband in time domain');
grid on;
% Plot first signal in Baseband in Frequency Domain
baseband_FDomain1 = fftshift(fft(baseband_signal1_filtered));
freq_base1 = linspace(-fs/2, fs/2, length(baseband_signal1_filtered)) / 1e3; % in kHz
subplot(2, 1, 2);
plot(freq_base1, abs(baseband_FDomain1));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('First signal in baseband in frequency domain');
grid on;
% Plot second signal in Baseband in Time Domain
figure;
subplot(2, 1, 1);
plot(x, baseband_signal2_filtered);
xlabel('Time (sec)');
ylabel('Amplitude');
title('Second signal in baseband in time domain');
grid on;
% Plot Second signal in Baseband in Frequency Domain
baseband_FDomain2 = fftshift(fft(baseband_signal2_filtered));
freq_base2 = linspace(-fs/2, fs/2, length(baseband_signal2_filtered)) / 1e3; % in kHz
subplot(2, 1, 2);
plot(freq_base2, abs(baseband_FDomain2));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('Second signal in baseband in frequency domain');
grid on;
% Define Downsampling Factor
down_sampling_factor = fsmultiplier; % Ratio of original to new sampling frequency
% Anti-Aliasing LPF Design
anti_aliasing_filter = designfilt('lowpassiir', ...
                             'Filterorder', 8, ...
                             'HalfPowerFrequency', fs / 2, ... % Nyquist frequency of original sampling rate
                             'SampleRate', fsnew);
% Downsample Signals 
anti_aliased_signal1 = filter(anti_aliasing_filter, baseband_signal1_filtered); % Apply anti-aliasing LPF
anti_aliased_signal2 = filter(anti_aliasing_filter, baseband_signal2_filtered); % Apply anti-aliasing LPF
down_sampled_signal1 = downsample(anti_aliased_signal1, down_sampling_factor);% Downsample
down_sampled_signal2 = downsample(anti_aliased_signal2, down_sampling_factor);% Downsample
% Time Vector for Downsampled Signals
t_down_sampled = (0:length(down_sampled_signal1)-1)' / fs; % Time vector for downsampled signals
fs_down_sampled = 1 / t_down_sampled;
% Plot first signal after downsampling in Time Domain
figure;
subplot(2, 1, 1);
plot(t_down_sampled, down_sampled_signal1);
xlabel('Time (sec)');
ylabel('Amplitude');
title('First signal after downsampling in Time Domain');
grid on;
% Plot first signal after downsampling in frequency Domain
down_sampled_FDomain1 = fftshift(fft(down_sampled_signal1));
frequ_down_sampling1 = linspace(-fs/2, fs/2, length(down_sampled_signal1)) / 1e3; % in kHz
subplot(2, 1, 2);
plot(frequ_down_sampling1, abs(down_sampled_FDomain1));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('First signal after downsampling in frequency domain');
grid on;
% Plot second signal after downsampling in Time Domain
figure;
subplot(2, 1, 1);
plot(t_down_sampled, down_sampled_signal2);
xlabel('Time (sec)');
ylabel('Amplitude');
title('Second signal after downsampling in time domain');
grid on;
% Plot second signal after downsampling in frequency Domain
down_sampled_FDomain2 = fftshift(fft(down_sampled_signal2));
freq_down_sampling2 = linspace(-fs/2, fs/2, length(down_sampled_signal2)) / 1e3; % in kHz
subplot(2, 1, 2);
plot(freq_down_sampling2, abs(down_sampled_FDomain2));
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('Second signal after downsampling in frequency domain');
grid on;
%% Comparison between original and recieved signals
sound(mono1, fs);% Play the first original signal 
pause(20);% Wait 20 seconds to let first signal to finish
sound(down_sampled_signal1,fs);% Play the first signal after downsampling
pause(20);% Wait 20 seconds to let first dwnsampled signal to finish
sound(mono2, fs);% Play the second original signal 
pause(20);% Wait 20 seconds to let second original signal to finish
sound(down_sampled_signal2, fs);% Play the second signal after downsampling