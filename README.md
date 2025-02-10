# Super-Heterodyne Receiver Simulation in MATLAB

## Project Overview
This project simulates a **super-heterodyne receiver** using MATLAB. The system includes an **AM modulator** and a **receiver** with multiple stages such as RF, IF, and baseband detection. The goal is to process two audio signals, modulate them using DSB-SC AM, and then demodulate them using a super-heterodyne receiver. The project also explores the effects of noise, RF filter removal, and oscillator frequency offsets on the received signal.

---

## Project Structure
The project is divided into the following blocks:
1. **Transmitter:**
   - **Preprocessing Block:** Loads and prepares audio signals for modulation.
   - **Modulator Block:** Implements DSB-SC AM modulation and combines signals using Frequency-Division Multiplexing (FDM).
2. **Wireless Channel:** Simulates a noise-free channel (noise is added later for analysis).
3. **Receiver:**
   - **RF Stage:** Filters the desired channel using a tunable band-pass filter (BPF).
   - **Mixer Block:** Shifts the RF signal to an intermediate frequency (IF).
   - **IF Stage:** Filters the IF signal using another BPF.
   - **Baseband Detection:** Recovers the original signal by downconverting and low-pass filtering.

---

## MATLAB Implementation
The MATLAB code implements the following steps:
1. **Loading and Preprocessing Signals:**
   - Audio signals are loaded using `audioread`.
   - Stereo signals are converted to mono.
   - Signals are padded with zeros to equalize their lengths.
2. **AM Modulation:**
   - DSB-SC AM modulation is applied using carrier frequencies of 100 kHz and 150 kHz.
   - The modulated signals are combined into an FDM signal.
3. **RF Stage:**
   - Tunable BPFs are applied to isolate the desired channels.
4. **Mixer and IF Stage:**
   - The RF signal is mixed with a local oscillator to shift it to the IF (25 kHz).
   - An IF BPF is applied to isolate the desired signal.
5. **Baseband Detection:**
   - The IF signal is downconverted to baseband using a local oscillator.
   - A low-pass filter (LPF) is applied to recover the original signal.
6. **Noise and Frequency Offset Analysis:**
   - Noise is added to the signal using `awgn`.
   - The effects of removing the RF BPF and oscillator frequency offsets (0.2 kHz and 1.2 kHz) are analyzed.

---

## Key Features
- **Modulation:** DSB-SC AM with carrier frequencies of 100 kHz and 150 kHz.
- **FDM:** Frequency-Division Multiplexing is used to combine the modulated signals.
- **RF and IF Filtering:** Tunable BPFs are used to isolate the desired channels.
- **Baseband Recovery:** Downconversion and LPF are used to recover the original signal.
- **Noise Analysis:** The impact of noise on signal quality is analyzed.
- **Frequency Offset Analysis:** The effects of oscillator frequency offsets on demodulation are studied.

---

## MATLAB Code Highlights
- **Signal Loading and Preprocessing:**
  ```matlab
  [stereo1, fs1] = audioread("Short_QuranPalestine.wav");
  mono1 = mean(stereo1, 2); % Convert to mono
