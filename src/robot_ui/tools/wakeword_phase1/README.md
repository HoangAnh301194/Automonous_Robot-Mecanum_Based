# Wake-word phase 1: pretrained, no training

This is an **isolated experiment**, not production Kiosk integration. It does not
modify ROS topics, navigation, VoiceController, microphone icons or LLM/TTS.
It uses 3 small pretrained **Vosk ASR** models to look for 3 phrases in offline
speech recognition results. This is **not** a specialized wake-word KWS model,
so its false-activation rate and proper-name recognition may be unsuitable.

| ID | Target | Model |
| --- | --- | --- |
| vi_sota_oi | "Sota ơi" | vosk-model-small-vn-0.4 |
| en_hey_souta | "Hey Souta" | vosk-model-small-en-us-0.15 |
| ja_soutakun | "Soutakun" / そうたくん | vosk-model-small-ja-0.22 |

Official pretrained model downloads:
- https://alphacephei.com/vosk/models/vosk-model-small-vn-0.4.zip
- https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
- https://alphacephei.com/vosk/models/vosk-model-small-ja-0.22.zip
- Model index and license notes: https://alphacephei.com/vosk/models

## 1. Prepare an isolated environment on Orin (Ubuntu 22.04)

Use the system Python version that has a compatible vosk wheel. Do not
override ROS 2's system dependencies. From the repo root:

    mkdir -p ~/.venvs
    python3 -m venv ~/.venvs/souta-wakeword
    source ~/.venvs/souta-wakeword/bin/activate
    python -m pip install vosk

Download three model ZIP files from the official URLs above, then extract them
into ~/vosk-models/, resulting in:

    ~/vosk-models/vosk-model-small-vn-0.4/
    ~/vosk-models/vosk-model-small-en-us-0.15/
    ~/vosk-models/vosk-model-small-ja-0.22/

Never commit downloaded models, recordings or personal voice data.

Do **not** run the live probe concurrently with the production kiosk during
phase 1. This baseline must not compete with the existing parec capture.

## 2. Collect recordings

Run from repo root:

    cd src/robot_ui/tools/wakeword_phase1
    python probe.py record --output samples/vi_01.wav --duration 4
    python probe.py record --output samples/en_01.wav --duration 4
    python probe.py record --output samples/ja_01.wav --duration 4
    python probe.py record --output samples/negative_01.wav --duration 10

Optional mic source:

    pactl list short sources
    python probe.py record --source 'alsa_input.usb-...mono-fallback' \
      --output samples/vi_02.wav --duration 4

Use 16-kHz mono 16-bit PCM WAV (the recorder does this). For external WAV files:

    ffmpeg -i input.wav -ac 1 -ar 16000 -c:a pcm_s16le output.wav

manifest.csv (create locally, not committed) must contain:

    file,expected
    samples/vi_01.wav,vi_sota_oi
    samples/en_01.wav,en_hey_souta
    samples/ja_01.wav,ja_soutakun
    samples/negative_01.wav,none

Recommended **pilot** dataset: at least 5 speakers x 4 attempts x 3 keywords
(60 positive samples), with varied distances/noise, plus 30 minutes of negative
speech/background. For production-level false-activation claims, collect a much
longer negative set; 30 minutes cannot establish a low false accepts/hour rate.

Collect negatives that include ordinary words resembling Souta, conversations
in all three languages, TTS played from robot speakers, and ambient lobby noise.

## 3. Offline evaluation

    python probe.py evaluate --manifest manifest.csv \
      --models-dir ~/vosk-models --report results.json

The evaluation loads 3 models once and decodes every WAV with each model,
so memory/CPU are intentionally stressed. The report includes recognized text;
**do not commit or share raw recordings/transcripts without consent**.

Outputs: recall, false accepts, negative audio hours, false accepts/hour (when
there is enough negative audio), per-keyword counts, decoding real-time factor
and peak resident memory. Match aliases are deliberately narrow.

If the ASR mishears a proper name, inspect transcripts and add only evidenced
aliases to a local JSON file. Example:

    {
      "vi_sota_oi": ["sota ơi", "souta ơi"],
      "en_hey_souta": ["hey souta", "hey sota"],
      "ja_soutakun": ["そうたくん", "そうた君", "ソウタクン"]
    }

Then rerun with --aliases aliases.json. Avoid overly broad aliases, which
increase false activations. Preserve the before/after results separately.

## 4. Live standalone probe (NOT connected to Kiosk)

    python probe.py live --models-dir ~/vosk-models
    python probe.py live --models-dir ~/vosk-models --source '@DEFAULT_SOURCE@'

If no wake events appear, turn on diagnostics to examine what each model
**actually recognizes**. Pause briefly after each phrase; Vosk finalizes
segments at speech boundaries. Use --debug-partials to inspect interim
recognition when final transcripts do not appear:

    python probe.py live --models-dir ~/vosk-models --debug-transcripts --debug-partials

In a second terminal, verify the active PulseAudio microphone:

    pactl get-default-source
    pactl list short sources

Debug transcripts can contain private speech: do not upload or publish raw
logs without consent. Remove --debug-transcripts and --debug-partials
after troubleshooting.

One parec process captures PCM and feeds three pretrained recognizers.
Events are written to stdout in JSON with keyword IDs; **no LLM/STT/TTS is
called**. Detectors use completed ASR segments, so latency may be larger than
a purpose-trained keyword spotter. The benchmark prints processing time/audio
time ratio and peak RSS to stderr.

Stop with Ctrl+C. This experiment does not automatically activate the mic,
take over ChatPage, or modify robot behavior.

## 5. Stage-1 decision criteria

Measure separately for quiet room and real lobby:
- Per-keyword recall = correct detections / positive utterances.
- False activations/hour on negative recordings.
- Detection delay from keyword end (measure with live timestamps/video if needed).
- CPU %, RSS and decoding real-time factor on the actual Orin.
- Failure cases: accent, distance, background speech, robot's own speaker.

Initial exploratory targets (not production guarantees):
- At least ~80% recall per keyword in the quiet pilot.
- Decode wall/audio ratio consistently under 1.0.
- No false activation in the short negative pilot; then **longer** negative
  trials are required before production integration.

If any model consistently fails on "Souta" (out-of-vocabulary name), document
it rather than introducing a large list of speculative phonetic aliases.

## 6. Next decision

Only after measuring the phase-1 baseline should we decide whether to:
- Run this ASR-matching approach with VAD gating and a shared microphone
  capture manager;
- Try a dedicated pretrained KWS engine when it covers the language;
- Use a custom wake-word model (phase 2) for problematic keywords.

A successful phase-1 trial does **not** authorize automatically enabling the
wake detector in production. Do not run two unrelated microphone capture
processes inside the Kiosk; refactor into one audio source before integration.
