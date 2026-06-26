import numpy as np
import sounddevice as sd
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Audio Calibration Script')
    parser.add_argument('phase', type=float, default=0.0, help='Phase of the calibration signal in radians')
    parser.add_argument('--duration', type=float, default=10.0, help='Duration of the calibration signal in seconds')
    parser.add_argument('--frequency', type=float, default=440.0, help='Frequency of the calibration signal in Hz')
    parser.add_argument('--sample_rate', type=int, default=48000, help='Sample rate in Hz')
    parser.add_argument("--master-gain", type=float, default=0.10, help="Global attenuation [0.01..1.0] applied to all tones (default: 0.10)")
    return parser.parse_args()

def generate_sine_wave(phase, duration, frequency, sample_rate, master_gain):
    phase_increment = 2 * np.pi * frequency / sample_rate
    idx = np.arange(int(duration * sample_rate))
    wave = np.sin(phase + idx * phase_increment)
    return wave * master_gain

def main():
    args = parse_args()
    signal = generate_sine_wave(args.phase, args.duration, args.frequency, args.sample_rate, args.master_gain)
    sd.play(signal, samplerate=args.sample_rate)
    sd.wait()

if __name__ == "__main__":
    main()
