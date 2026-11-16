# Souta KWS – GUI thu âm có nhãn (Ubuntu)

Standalone Tkinter utility; operates exclusively in `src/robot_wakeword/training/`.
It does not modify the ROS 2 nodes, Kiosk UI, legacy Vosk or Nav2.

## Start

With your `.venv` active, from the training directory:

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH python recorder_gui.py
```

If `ModuleNotFoundError: No module named 'tkinter'`, install Ubuntu's system Tk binding:

```bash
sudo apt install python3-tk
```

PulseAudio's `parec` and `paplay` are used. The default microphone is
`@DEFAULT_SOURCE@`, configurable in the GUI.

## Workflow

1. Choose the **label** first: `positive`, `negative`, `hard_negative`, or `ambient`.
2. **Đăng ký mới**: input a human-friendly speaker name, or a name for an
   ambient recording session. The name stays private on your computer and a
   stable pseudonymous ID (`speaker_001`, `session_001`, ...) is generated.
   Choose an existing name in subsequent recordings.
3. Select the duration (3, 5, 10, 30 or 60 seconds) and click **Bắt đầu ghi**.
   Recording stops automatically or via **Kết thúc sớm** (after at least 0.25 s).
   **Hủy ghi** interrupts and throws away the capture.
4. After recording, click **Nghe lại**. If wrong/noisy or mislabeled, choose
   **Bỏ bản thu / Không lưu** to discard, then record again. No training file
   has been created. To fix a selected label, change the label/person
   before saving and inspect the displayed pending label.
5. Only **Lưu vào dataset** writes a WAV to `data/raw/<label>/<speaker_or_session_id>/<NNN>.wav`.
   Filenames are numbered automatically and never overwritten. The GUI is
   ready for another recording immediately.

For `positive`, say "Sô ta" or "Sô ta ơi" once in a 3-second clip, with some
silence on either side. For `hard_negative`, record similar-sounding phrases,
including "sổ tay". For `negative`, record natural speech without the phrase.
For `ambient`, record continuous room/robot noise in longer sessions.

The WAV format is **mono, 16 kHz, signed 16-bit PCM**, matching `souta_kws.py prepare`.
The app reports RMS and peaks and warns about possibly weak/clipped recordings;
listening remains the final quality check.

Metadata mapping real names to IDs is in `data/recorder_registry.json`.
All recordings, registry metadata, and temporary previews are under `data/`,
which is ignored by Git. Do not commit real voice data to a public repository.
Do not use a separate ID for the same individual across positive and negative
recordings: train/validation/test splits use speaker identity to prevent leakage.
`ambient` uses session IDs instead.

## Headless tests

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH python -m unittest -v test_recorder_gui.py
```

For the rest of the pipeline, continue with `python souta_kws.py prepare` after
collecting **at least three independent speakers for positive and negative**.
