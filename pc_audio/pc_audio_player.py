"""Player audio lato PC per l'audiometro.

Questo script legge i comandi inviati dalla scheda STM32 via seriale,
riproduce il tono richiesto con la scheda audio del computer e, alla fine,
salva e interpreta i risultati in formato JSON.

Modalita operative principali:
- audio guidato dalla scheda STM32 (comandi UART)
- salvataggio automatico dei risultati finali
- modalita `--listen-only` per usare il PC solo come terminale seriale
"""

import argparse
import json
import re
import threading
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
import sounddevice as sd


# La scheda invia righe del tipo: "Freq 1000 Hz → -60.0 dBFS".
# Questa regex estrae frequenza e soglia da ciascuna riga finale.
RESULT_LINE_RE = re.compile(r"^Freq\s+(\d+)\s+Hz\s+→\s+(-?\d+(?:\.\d+)?)\s+dBFS$")


class ResultCollector:
    """Raccoglie, interpreta e salva i risultati finali arrivati via UART."""

    def __init__(self, output_dir: str, expected_ears=None, expected_freq_count: int = 11):
        # Cartella in cui salvare il JSON finale.
        self.output_dir = Path(output_dir)
        # In questo progetto ci aspettiamo due orecchi: sinistro e destro.
        self.expected_ears = expected_ears or ("L", "R")
        # Numero di frequenze attese per ciascun orecchio.
        self.expected_freq_count = expected_freq_count
        self.reset()

    def reset(self):
        # Stato interno del parser:
        # - in_results_block: siamo dentro il blocco finale dei risultati?
        # - current_ear: orecchio attualmente in lettura
        # - results: dizionario {"L": {freq: dBFS}, "R": {...}}
        self.in_results_block = False
        self.current_ear = None
        self.results = {ear: {} for ear in self.expected_ears}

    def feed_line(self, line: str):
        """Analizza una singola riga ricevuta da UART.

        Ritorna una coppia (payload, path) quando il test è completo e il JSON
        è stato salvato; altrimenti ritorna None.
        """
        text = line.strip()

        # Questa riga marca l'inizio del blocco di report finale.
        if text == "=== Risultati Audiometria ===":
            self.in_results_block = True
            self.current_ear = None
            return None

        # Fuori dal blocco finale non facciamo parsing dei risultati.
        if not self.in_results_block:
            return None

        # La scheda stampa un'intestazione per ogni orecchio.
        if text.startswith("Orecchio "):
            ear = text.split()[-1].upper()
            if ear not in self.results:
                self.results[ear] = {}
            self.current_ear = ear
            return None

        # Ogni riga di frequenza viene salvata nel dizionario dell'orecchio.
        match = RESULT_LINE_RE.match(text)
        if match and self.current_ear is not None:
            freq_hz = int(match.group(1))
            dbfs = float(match.group(2))
            self.results[self.current_ear][freq_hz] = dbfs

            # Quando abbiamo tutte le frequenze richieste per entrambi gli orecchi,
            # costruiamo il payload finale e lo salviamo su disco.
            if self._is_complete():
                payload = self._build_payload()
                paths = self._save(payload)
                self.reset()
                return payload, paths

        return None

    def _is_complete(self) -> bool:
        # Il test è concluso solo quando ogni orecchio ha tutte le frequenze.
        for ear in self.expected_ears:
            if len(self.results.get(ear, {})) < self.expected_freq_count:
                return False
        return True

    def _build_payload(self):
        # Dati grezzi dei due orecchi.
        left = self.results.get("L", {})
        right = self.results.get("R", {})

        # Frequenze presenti su entrambi i lati: servono per confronti diretti.
        common_freq = sorted(set(left.keys()) & set(right.keys()))

        if common_freq:
            # Differenza assoluta L/R per ogni frequenza condivisa.
            lr_diff = [abs(left[f] - right[f]) for f in common_freq]
            mean_lr_diff = sum(lr_diff) / len(lr_diff)
            max_lr_diff = max(lr_diff)
        else:
            mean_lr_diff = 0.0
            max_lr_diff = 0.0

        # Piccole funzioni locali per statistiche elementari.
        def mean(values):
            if not values:
                return 0.0
            return sum(values) / len(values)

        def std_dev(values):
            if len(values) < 2:
                return 0.0
            m = mean(values)
            var = sum((v - m) ** 2 for v in values) / len(values)
            return var ** 0.5

        def slope_db_per_khz(values_by_freq):
            points = sorted(values_by_freq.items())
            if len(points) < 2:
                return 0.0
            x = [p[0] / 1000.0 for p in points]
            y = [p[1] for p in points]
            x_mean = mean(x)
            y_mean = mean(y)
            num = sum((xi - x_mean) * (yi - y_mean) for xi, yi in zip(x, y))
            den = sum((xi - x_mean) ** 2 for xi in x)
            if den == 0.0:
                return 0.0
            return num / den

        def classify_symmetry(mean_abs_diff):
            # Classificazione semplice della simmetria tra i due orecchi.
            if mean_abs_diff <= 2.0:
                return "ottima"
            if mean_abs_diff <= 4.0:
                return "buona"
            if mean_abs_diff <= 6.0:
                return "discreta"
            return "asimmetria significativa"

        def classify_low_vs_high(delta_db):
            # delta > 0: basse frequenze richiedono piu livello (atteso con cuffie consumer)
            if delta_db >= 12.0:
                return "forte penalizzazione basse frequenze"
            if delta_db >= 6.0:
                return "moderata penalizzazione basse frequenze"
            if delta_db <= -6.0:
                return "medie/alte piu penalizzate"
            return "profilo abbastanza uniforme"

        summary = {
            # Media complessiva per orecchio e scarto medio assoluto L/R.
            "mean_dbfs": {
                "L": mean(list(left.values())),
                "R": mean(list(right.values())),
            },
            "mean_abs_lr_diff_db": mean_lr_diff,
            "max_abs_lr_diff_db": max_lr_diff,
        }

        # Confronto tra basse e medio-alte frequenze: utile per capire la forma
        # generale della curva uditiva.
        low_band = [125, 250, 500]
        high_band = [1000, 1500, 2000, 3000, 4000, 6000, 8000]

        low_vals = [0.5 * (left[f] + right[f]) for f in low_band if f in left and f in right]
        high_vals = [0.5 * (left[f] + right[f]) for f in high_band if f in left and f in right]
        low_minus_high = mean(low_vals) - mean(high_vals) if low_vals and high_vals else 0.0

        freq_delta = {}
        for f in common_freq:
            freq_delta[str(f)] = abs(left[f] - right[f])

        interpretation = {
            "symmetry": {
                "class": classify_symmetry(mean_lr_diff),
                "mean_abs_lr_diff_db": mean_lr_diff,
                "max_abs_lr_diff_db": max_lr_diff,
                "flagged_freq_abs_diff_gt_4db": [
                    int(f) for f, d in freq_delta.items() if d > 4.0
                ],
            },
            "spectral_profile": {
                "low_band_mean_dbfs": mean(low_vals) if low_vals else 0.0,
                "high_band_mean_dbfs": mean(high_vals) if high_vals else 0.0,
                "low_minus_high_db": low_minus_high,
                "class": classify_low_vs_high(low_minus_high),
            },
            "stability": {
                # Deviazione standard: misura quanto i punti sono sparsi.
                "std_db": {
                    "L": std_dev(list(left.values())),
                    "R": std_dev(list(right.values())),
                },
                # Pendenza rispetto alla frequenza: aiuta a capire se la curva
                # sale o scende con l'aumentare della frequenza.
                "slope_db_per_khz": {
                    "L": slope_db_per_khz(left),
                    "R": slope_db_per_khz(right),
                },
            },
            "verdict": (
                "Profilo complessivamente buono e simmetrico"
                if mean_lr_diff <= 4.0
                else "Profilo con asimmetrie da ricontrollare"
            ),
            "notes": [
                "Valori in dBFS: non equivalgono a dB HL clinici.",
                "Interpretazione influenzata da cuffie, ambiente e calibrazione master-gain.",
            ],
        }

        # Il payload finale contiene sia i dati grezzi sia l'interpretazione.
        return {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "results": {
                "L": {str(k): v for k, v in sorted(left.items())},
                "R": {str(k): v for k, v in sorted(right.items())},
            },
            "summary": summary,
            "interpretation": interpretation,
        }

    def _save(self, payload):
        # Salviamo un solo file JSON con risultati + interpretazione.
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        json_path = self.output_dir / f"audiometry_{stamp}.json"

        with json_path.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)

        return json_path


class TonePlayer:
    """Genera il tono sul PC quando la scheda invia i comandi UART."""

    def __init__(self, samplerate: int, master_gain: float):
        # Il master gain attenua il volume globale per evitare ascolti troppo forti.
        self.samplerate = samplerate
        self.master_gain = max(0.01, min(1.0, float(master_gain)))

        # Stato corrente del tono riprodotto.
        self.phase = 0.0
        self.freq = 440.0
        self.gain = 0.0
        self.ear = "L"
        self.playing = False
        self.lock = threading.Lock()

        # Stream audio stereo a bassa latenza.
        self.stream = sd.OutputStream(
            samplerate=self.samplerate,
            channels=2,
            dtype="float32",
            callback=self._callback,
            blocksize=0,
        )
        self.stream.start()

    def _callback(self, outdata, frames, _time, status):
        # Callback audio richiamata dal driver: qui costruiamo i campioni.
        if status:
            print(f"[audio] {status}")

        with self.lock:
            playing = self.playing
            freq = self.freq
            gain = self.gain
            ear = self.ear
            phase = self.phase

        if not playing or gain <= 0.0:
            outdata.fill(0)
            return

        # Generazione di una sinusoide tramite accumulo di fase.
        phase_inc = 2.0 * np.pi * freq / self.samplerate
        idx = np.arange(frames, dtype=np.float32)
        samples = np.sin(phase + phase_inc * idx) * (gain * self.master_gain)

        # Aggiorniamo la fase per il blocco successivo, evitando salti udibili.
        phase = (phase + phase_inc * frames) % (2.0 * np.pi)
        with self.lock:
            self.phase = phase

        # Costruiamo lo stereo: sinistro, destro o entrambi in base all'orecchio.
        stereo = np.zeros((frames, 2), dtype=np.float32)
        if ear == "R":
            stereo[:, 1] = samples.astype(np.float32)
        elif ear == "B":
            stereo[:, 0] = samples.astype(np.float32)
            stereo[:, 1] = samples.astype(np.float32)
        else:
            stereo[:, 0] = samples.astype(np.float32)

        outdata[:] = stereo

    def start(self, ear: str, freq: float, gain: float):
        # Aggiorna i parametri del tono e attiva la riproduzione.
        with self.lock:
            self.ear = (ear or "L").upper()
            self.freq = max(20.0, float(freq))
            self.gain = max(0.0, min(1.0, float(gain)))
            self.playing = True
        effective = self.gain * self.master_gain
        print(f"[tone] START ear={self.ear} freq={self.freq:.1f}Hz gain={self.gain:.3f} effective={effective:.3f}")

    def set_gain(self, ear: str, gain: float):
        # La scheda aumenta il livello a passi: qui aggiorniamo solo l'ampiezza.
        with self.lock:
            self.ear = (ear or self.ear or "L").upper()
            self.gain = max(0.0, min(1.0, float(gain)))
        effective = self.gain * self.master_gain
        print(f"[tone] GAIN ear={self.ear} {self.gain:.3f} effective={effective:.3f}")

    def stop(self):
        # Spegne il tono quando la scheda invia STOP o DONE.
        with self.lock:
            self.playing = False
            self.gain = 0.0
        print("[tone] STOP")

    def close(self):
        self.stream.stop()
        self.stream.close()


def parse_args():
    """Definisce i parametri da riga di comando."""
    parser = argparse.ArgumentParser(description="STM32 audiometer PC audio player")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--samplerate", type=int, default=48000, help="Audio sample rate")
    parser.add_argument(
        "--master-gain",
        type=float,
        default=0.10,
        help="Global attenuation [0.01..1.0] applied to all tones (default: 0.10)",
    )
    parser.add_argument(
        "--results-dir",
        default="pc_audio/results",
        help="Directory where end-of-test JSON files are stored",
    )
    parser.add_argument(
        "--listen-only",
        action="store_true",
        help="Do not open PC audio output; only read UART and save/interpret results",
    )
    return parser.parse_args()


def main():
    # Leggiamo gli argomenti scelti dall'utente.
    args = parse_args()

    # In modalita listen-only il PC non apre l'uscita audio: resta solo il monitoraggio seriale.
    player = None if args.listen_only else TonePlayer(args.samplerate, args.master_gain)
    collector = ResultCollector(args.results_dir)

    print(f"[serial] Opening {args.port} @ {args.baud}")
    if player is not None:
        print(f"[audio] master_gain={player.master_gain:.2f}")
    else:
        print("[audio] listen-only mode: PC audio output disabled")
    ser = serial.Serial(args.port, args.baud, timeout=1)

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            # Decodifica robusta: ignoriamo eventuali byte non validi.
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            print(f"[rx] {line}")

            # Se la riga appartiene al blocco finale, aggiorniamo il collector.
            collected = collector.feed_line(line)
            if collected is not None:
                payload, json_path = collected
                summary = payload["summary"]
                interpretation = payload["interpretation"]
                print(
                    "[results] Salvato JSON: "
                    f"{json_path} | "
                    f"mean L={summary['mean_dbfs']['L']:.2f} dBFS "
                    f"mean R={summary['mean_dbfs']['R']:.2f} dBFS "
                    f"mean |L-R|={summary['mean_abs_lr_diff_db']:.2f} dB | "
                    f"verdetto={interpretation['verdict']}"
                )

            # I comandi AUDIO servono solo quando il PC deve riprodurre il suono.
            parts = line.split()
            if len(parts) < 2 or parts[0] != "AUDIO":
                continue

            cmd = parts[1].upper()
            try:
                if player is None:
                    # In listen-only ignoriamo i comandi audio ma continuiamo a leggere la seriale.
                    continue
                if cmd == "START":
                    if len(parts) >= 5:
                        ear = parts[2]
                        freq = float(parts[3])
                        gain = float(parts[4])
                        player.start(ear, freq, gain)
                    elif len(parts) >= 4:
                        freq = float(parts[2])
                        gain = float(parts[3])
                        player.start("L", freq, gain)
                elif cmd == "GAIN":
                    if len(parts) >= 4:
                        ear = parts[2]
                        gain = float(parts[3])
                        player.set_gain(ear, gain)
                    elif len(parts) >= 3:
                        gain = float(parts[2])
                        player.set_gain("L", gain)
                elif cmd in {"STOP", "DONE"}:
                    # DONE viene trattato come STOP del suono e fine del test.
                    player.stop()
            except ValueError:
                print(f"[warn] Messaggio non valido: {line}")
    except KeyboardInterrupt:
        print("\n[exit] Interrotto da tastiera")
    finally:
        # Chiusura ordinata delle risorse.
        if player is not None:
            player.close()
        ser.close()


if __name__ == "__main__":
    main()
