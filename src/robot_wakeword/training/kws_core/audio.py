"""WAV I/O, microphone capture, windowing, and training-only augmentation."""
from __future__ import annotations

import subprocess
import wave
from pathlib import Path

from .constants import SAMPLE_RATE, WINDOW


def read_wav(path):
    """Read PCM S16 mono 16 kHz; reject silent/empty files and unconverted input."""
    import numpy as np
    with wave.open(str(path), "rb") as wav:
        if (wav.getnchannels(), wav.getsampwidth(), wav.getframerate()) != (1, 2, SAMPLE_RATE):
            raise ValueError(f"Expected 16 kHz mono S16 PCM WAV: {path}")
        raw = wav.readframes(wav.getnframes())
    audio = np.frombuffer(raw, dtype="<i2").copy()
    if len(audio) < SAMPLE_RATE // 4:
        raise ValueError(f"Audio shorter than 0.25 s: {path}")
    if not np.any(audio):
        raise ValueError(f"Silent/empty audio file: {path}")
    return audio

def write_wav(path, audio):
    import numpy as np
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(SAMPLE_RATE)
        wav.writeframes(np.asarray(audio, dtype="<i2").tobytes())

def record(args):
    """Collect a sample without calling any speech recognizer."""
    duration = float(args.duration)
    if not 0.5 <= duration <= 120:
        raise ValueError("Recording duration must be 0.5–120 seconds")
    cmd = ["parec", "-d", args.source, "--rate=16000", "--channels=1",
           "--format=s16le", "--raw"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        raw = proc.stdout.read(round(duration * SAMPLE_RATE) * 2)
    finally:
        proc.terminate()
        try:
            proc.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
    if len(raw) < SAMPLE_RATE // 2:
        raise RuntimeError("Could not capture microphone audio; inspect pactl list short sources")
    import numpy as np
    write_wav(args.output, np.frombuffer(raw, dtype="<i2"))
    print(f"Saved: {args.output} ({len(raw) / 32000:.2f} seconds)")

def window_audio(audio, rng, shift=False, random_crop=False):
    """Make a 2 s window centered on an utterance; random crop for negative recordings."""
    import numpy as np
    n = len(audio)
    if n >= WINDOW:
        if random_crop:
            start = int(rng.integers(0, n - WINDOW + 1))
        else:
            start = (n - WINDOW) // 2
            if shift:
                start += int(rng.integers(-2000, 2001))
                start = max(0, min(n - WINDOW, start))
        return audio[start:start + WINDOW].copy()
    out = np.zeros(WINDOW, dtype=np.int16)
    start = (WINDOW - n) // 2
    if shift:
        start += int(rng.integers(-2000, 2001))
        start = max(0, min(WINDOW - n, start))
    out[start:start + n] = audio
    return out

def augment_pcm(audio, rng, noise_pool):
    """Apply mild gain and optionally additive room noise at a sampled SNR."""
    import numpy as np
    x = audio.astype(np.float32) / 32768
    gain = float(rng.uniform(0.65, 1.4))
    x *= gain
    if noise_pool and rng.random() < 0.75:
        noise = window_audio(noise_pool[int(rng.integers(0, len(noise_pool)))],
                             rng, random_crop=True).astype(np.float32) / 32768
        p_signal = float(np.mean(x * x))
        p_noise = float(np.mean(noise * noise))
        if p_signal > 1e-7 and p_noise > 1e-9:
            snr_db = float(rng.uniform(5, 25))
            noise *= (p_signal / (p_noise * (10 ** (snr_db / 10)))) ** 0.5
            x += noise
    return np.clip(x * 32768, -32768, 32767).astype(np.int16)
