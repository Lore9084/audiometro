import numpy as np
import sounddevice as sd
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Audio Calibration Script')
    parser.add_argument('--phase', type=float, default=0.0, help='Phase of the calibration signal in radians')
    parser.add_argument('--duration', type=float, default=10.0, help='Duration of the calibration signal in seconds')
    parser.add_argument('--frequency', type=float, default=1000.0, help='Frequency of the calibration signal in Hz')
    parser.add_argument('--sample_rate', type=int, default=48000, help='Sample rate in Hz')
    #gain consigliato: 1 -20dBFS , 0.316 -30dbFS , 0.10 -40dBFS , 0.0316 -50dBFS
    #parser.add_argument('--gain', type=float, default=0.10, help='Gain of the calibration signal')
    return parser.parse_args()

def generate_sine_wave(phase, duration, frequency, sample_rate, gain, master_gain):
    phase_increment = 2 * np.pi * frequency / sample_rate
    idx = np.arange(int(duration * sample_rate))
    wave = np.sin(phase + idx * phase_increment)
    return wave * gain * master_gain

def main():
    args = parse_args()
    master_gain = 0.10
    gain = [1.0, 0.316, 0.10, 0.0316]
    dbfs = [20 * np.log10(g * master_gain) for g in gain]
    db = []
    for i in range(4):
        print(f"Segnale {i+1}/4: "f"gain={gain[i]} "f"→ {dbfs[i]:.1f} dBFS")
        signal = generate_sine_wave(args.phase, args.duration, args.frequency, args.sample_rate, gain[i], master_gain)
        sd.play(signal, samplerate=args.sample_rate)
        sd.wait()
        db.append(float(input(f"Inserisci il livello di pressione sonora (SPL) misurato in dB per il segnale {i+1}: ")))
    offset = [db[i] - dbfs[i] for i in range(4)]
    media_offset = sum(offset) / len(offset)
    std = np.std(offset)
    print(f"Offset calcolato: {offset}")
    print(f"Media offset: {media_offset}")
    print(f"Deviazione standard: ±{std:.2f} dB (accettabile se < 5 dB)")

#offset = SPLmisurato - dBFSriprodotto
#la formula è db = 20 * log10(ampiezza del segnale / ampiezza di riferimento (1))
#la formula classica per calcolare il livello di pressione sonora (SPL) in decibel (dB) è:
#db = 10 * log10(P / P0)
#esempio: SPLmisurato = 80dB , dBFSriprodotto = -40dBFS , offset = 80 - (-40) = 120dB
if __name__ == "__main__":
    main()
