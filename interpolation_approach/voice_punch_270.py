"""
Boxing Robot Arm (270-degree build) - Voice-Cued Tilt Selection
-------------------------------------------------------------------
Say a NUMBER before throwing a punch (e.g. "ninety") - not a punch name
like the original voice_punch.py used. The number is snapped to whichever
of the 4 real punch_angle_270.PUNCHES tilt values it's closest to and
logged as "next punch." When punch_classifier's speed/travel gate then
detects a real strike, track_interpolation_270.py checks for a logged
tilt FIRST - if one's pending, it wins and that punch's full (pan, tilt,
yaw, elbow) position plays instead of guessing from the classifier.
ONLY when nothing is logged does the classifier's own type guess fall
back to triggering an animation (hook -> PUNCHES["4"], uppercut ->
PUNCHES["1"] - both wired in track_interpolation_270.py, not here).

Valid numbers to say - these are the only 4 real positions that exist
(see punch_angle_270.PUNCHES):
    220 = Uppercut        65 = High Overhand
     90 = Overhand        120 = Hook

Reuses the exact offline speech stack and number-parsing from the
original voice_punch.py (Vosk/pyaudio, words_to_number/_extract_angle) -
see that file's docstring for the full reasoning on why. The punch-NAME
recognition and the PUNCH_TILTS/YAW_END import are dropped here since this
build keys off a spoken number instead of a spoken name.

Runs in its own background thread so it never blocks the camera loop -
call start() once, then pop_next_tilt() from the main tracking loop each
frame (only meaningful right when a strike is actually detected).

Standalone test (no camera/Arduino needed):
    python voice_punch_270.py
    Say (or type, in text mode) one of the 4 numbers above. Ctrl+C to quit.
"""

import json
import re
import threading
import time
import winsound
import zipfile
from datetime import datetime
from pathlib import Path

import pyaudio
import requests
from vosk import KaldiRecognizer, Model

from punch_angle_270 import PUNCHES  # noqa: E402 - single source of truth for the 4 real tilts

# ==================== TUNABLE SETTINGS ====================

MODEL_NAME = "vosk-model-en-us-0.22"
BASE_DIR = Path(__file__).resolve().parent

# Every non-empty final transcription gets appended here - what Vosk
# actually heard, what number (if any) got parsed out of it, and what
# happened as a result (armed/rejected/no-number). Added specifically to
# turn "the voice stuff doesn't transcribe very well" from an impression
# into something with real examples to look at - same reasoning as every
# other "measure, don't guess" tool in this project (capture_point.py,
# punch_dataset.py). Appends, never overwrites, so it accumulates across
# runs - delete it manually to start fresh.
TRANSCRIPT_LOG_PATH = BASE_DIR / "voice_transcripts.log"

# Reuse the model already downloaded for BackTPal rather than pulling
# another ~1.8GB copy. Falls back to downloading a local copy into this
# folder if that project ever moves or gets deleted.
_BACKTPAL_MODEL_DIR = Path(r"C:\Users\Mekan\Desktop\BackTPal") / MODEL_NAME / MODEL_NAME
_LOCAL_MODEL_DIR = BASE_DIR / MODEL_NAME
_MODEL_URL = f"https://alphacephei.com/vosk/models/{MODEL_NAME}.zip"

# {tilt: (key, label)} - derived from punch_angle_270.PUNCHES, not
# hardcoded again, so this can't drift out of sync with the real data.
VALID_TILTS = {pos[1]: (key, label) for key, (label, pos) in PUNCHES.items()}

# A spoken number only counts if it's within this many degrees of one of
# the 4 real tilts above - otherwise it's rejected rather than silently
# snapped to something far off (e.g. mishearing "90" as "19" shouldn't
# quietly become Overhand). The 4 real tilts are 65/90/120/220 - the
# tightest real gap between any two is 25 degrees (65-90), so 10 leaves
# real margin against a misheard neighbor while still forgiving normal
# speech-recognition rounding.
SNAP_TOLERANCE_DEG = 10

# A pending tilt expires if never consumed by a real detected strike
# within this long - without this, saying a number and then forgetting,
# then later throwing an unrelated ordinary punch that happens to pass the
# speed/travel gate, would silently play that animation for it.
PENDING_EXPIRY_SEC = 8.0

# "voice" (microphone, Vosk) or "text" (type a line + Enter) - text mode
# exists for testing without speaking out loud. Both expose the exact same
# pending-command interface, so nothing downstream cares which is active.
INPUT_MODE = "voice"

# Windows' system default input device is NOT necessarily the mic you're
# actually sitting in front of - found via direct device enumeration that
# this machine's default was a separate USB audio device, not the C922
# webcam (the same one MediaPipe is already pointed at you with), which
# would silently explain voice "not working at all" - it was listening to
# the wrong physical device the whole time, not failing to recognize
# speech. Pick by name instead of trusting the system default, so this
# can't drift again if Windows' default ever changes (e.g. plugging in
# headphones). Substring match, case-insensitive - adjust if the actual
# mic you want is named differently (run this file with no mic connected
# removed/changed to see the full device list print at startup).
PREFERRED_MIC_NAME_CONTAINS = "C922"

# ============================================================


def words_to_number(text: str):
    """Convert spoken number words into a numeric value. Copied verbatim
    from BackTPal/transcribe_live.py (and voice_punch.py) - same problem
    (Vosk transcribes numbers as words, e.g. "ninety"), no need to
    re-solve it."""
    units = {
        "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
        "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10,
        "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14,
        "fifteen": 15, "sixteen": 16, "seventeen": 17, "eighteen": 18,
        "nineteen": 19,
    }
    tens = {
        "twenty": 20, "thirty": 30, "forty": 40, "fifty": 50,
        "sixty": 60, "seventy": 70, "eighty": 80, "ninety": 90,
    }
    scales = {"hundred": 100, "thousand": 1000, "million": 1000000}

    tokens = [t for t in text.lower().replace("-", " ").split() if t]
    if not tokens:
        return None

    sign = 1
    if tokens and tokens[0] in {"minus", "negative"}:
        sign = -1
        tokens = tokens[1:]

    if tokens and all(token in units and units[token] < 10 for token in tokens):
        return float(sign * int("".join(str(units[token]) for token in tokens)))

    if len(tokens) == 3 and tokens[0] in units and units[tokens[0]] < 10:
        first = units[tokens[0]]
        if tokens[1] in tens and tokens[2] in units and units[tokens[2]] < 10:
            tail = tens[tokens[1]] + units[tokens[2]]
            return float(sign * (first * 100 + tail))

    total = 0
    current = 0
    used = False
    i = 0
    while i < len(tokens):
        token = tokens[i]
        if token == "and":
            i += 1
            continue
        if token in units:
            current += units[token]
            used = True
            i += 1
            continue
        if token in tens:
            current += tens[token]
            used = True
            i += 1
            continue
        if token in scales:
            scale = scales[token]
            if token == "hundred":
                current = (current or 1) * scale
            else:
                total += (current or 1) * scale
                current = 0
            used = True
            i += 1
            continue
        if token.isdigit():
            current += int(token)
            used = True
            i += 1
            continue
        return None

    if not used:
        return None
    return float(sign * (total + current))


_NUMBER_WORDS = {
    "zero", "one", "two", "three", "four", "five", "six", "seven", "eight",
    "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
    "sixteen", "seventeen", "eighteen", "nineteen", "twenty", "thirty",
    "forty", "fifty", "sixty", "seventy", "eighty", "ninety", "hundred",
    "and", "minus", "negative",
}


def _extract_angle(text: str):
    """Plain digits first (Vosk sometimes emits '90' directly rather than
    the word form), then spoken number words as a fallback - filtered to
    just the number-word vocabulary first, since words_to_number() aborts
    on the first unrecognized token and a real utterance is often "ninety
    degrees", not just "ninety"."""
    match = re.search(r"-?\d+(?:\.\d+)?", text)
    if match:
        return float(match.group(0))
    filtered = " ".join(t for t in text.lower().split() if t in _NUMBER_WORDS)
    if not filtered:
        return None
    return words_to_number(filtered)


def _snap_to_valid_tilt(value):
    """Returns (tilt, key, label) for the closest VALID_TILTS entry within
    SNAP_TOLERANCE_DEG, or None if nothing's close enough - see
    SNAP_TOLERANCE_DEG's comment for why this doesn't just always snap to
    the nearest one regardless of distance."""
    closest_tilt = min(VALID_TILTS, key=lambda t: abs(t - value))
    if abs(closest_tilt - value) > SNAP_TOLERANCE_DEG:
        return None
    key, label = VALID_TILTS[closest_tilt]
    return closest_tilt, key, label


def _beep_ready():
    winsound.Beep(880, 120)


def _beep_done():
    winsound.Beep(660, 80)
    time.sleep(0.05)
    winsound.Beep(880, 120)


def _beep_error():
    winsound.Beep(300, 200)


def _log_transcript(heard, parsed, outcome):
    """Appends one line to TRANSCRIPT_LOG_PATH - heard is the raw Vosk
    text, parsed is the extracted number (or None), outcome is a short
    human-readable result ("armed: Hook (tilt=120)", "rejected: no number
    found", "rejected: 19 not close enough to a real tilt"). A write
    failure here should never take down voice recognition itself, so it's
    swallowed rather than raised."""
    try:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        parsed_str = f"{parsed:.0f}" if parsed is not None else "none"
        with open(TRANSCRIPT_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(f"{timestamp} | heard: {heard!r:30} | parsed: {parsed_str:>5} | {outcome}\n")
    except OSError as e:
        print(f"[voice] WARNING: could not write to transcript log: {e}")


def _download_model(out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    zip_path = out_dir / f"{MODEL_NAME}.zip"
    print(f"[voice] Downloading {MODEL_NAME} (~1.8GB, one-time)...")
    with requests.get(_MODEL_URL, stream=True, timeout=300) as response:
        response.raise_for_status()
        with zip_path.open("wb") as f:
            for chunk in response.iter_content(chunk_size=1024 * 1024):
                if chunk:
                    f.write(chunk)
    print("[voice] Extracting model...")
    with zipfile.ZipFile(zip_path, "r") as zf:
        zf.extractall(out_dir)
    zip_path.unlink(missing_ok=True)
    return out_dir / MODEL_NAME


def resolve_model_path() -> Path:
    if _BACKTPAL_MODEL_DIR.exists():
        return _BACKTPAL_MODEL_DIR
    if _LOCAL_MODEL_DIR.exists():
        return _LOCAL_MODEL_DIR
    return _download_model(BASE_DIR)


def select_input_device(audio):
    """Returns the PyAudio device info dict to use - prefers a device
    whose name contains PREFERRED_MIC_NAME_CONTAINS over whatever Windows
    currently has set as the system default (see that constant's comment
    for why). Falls back to the system default, with a clear warning, if
    no matching device is found - e.g. the webcam got unplugged or its
    device name changed."""
    for i in range(audio.get_device_count()):
        info = audio.get_device_info_by_index(i)
        if info["maxInputChannels"] > 0 and PREFERRED_MIC_NAME_CONTAINS.lower() in info["name"].lower():
            return info
    print(f"[voice] WARNING: no input device matching {PREFERRED_MIC_NAME_CONTAINS!r} found - "
          f"falling back to the system default, which may not be the right physical mic.")
    return audio.get_default_input_device_info()


class _PendingTiltListener:
    """Shared pending-command state for any input method - subclasses only
    need to implement _listen_loop() and call _set_pending() from it.
    pop_next_tilt()/peek_next_tilt() are the only methods
    track_interpolation_270.py actually calls."""

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = None  # {"tilt", "key", "label", "armed_at"} or None
        self._thread = None
        self._running = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def pop_next_tilt(self):
        """Returns the pending {"tilt", "key", "label"} dict and clears it,
        or None if nothing's pending or the pending value is too stale
        (see PENDING_EXPIRY_SEC) - consumed exactly once, so an old
        command can't silently reapply to a later, unrelated motion."""
        with self._lock:
            pending = self._pending
            self._pending = None
        if pending is None:
            return None
        if time.monotonic() - pending["armed_at"] > PENDING_EXPIRY_SEC:
            print(f"[input] pending tilt={pending['tilt']:.0f} ({pending['label']}) expired unused")
            return None
        return {"tilt": pending["tilt"], "key": pending["key"], "label": pending["label"]}

    def peek_next_tilt(self):
        """Non-consuming read, for status display only."""
        with self._lock:
            return None if self._pending is None else dict(self._pending)

    def _set_pending(self, tilt, key, label):
        with self._lock:
            self._pending = {"tilt": tilt, "key": key, "label": label, "armed_at": time.monotonic()}

    def _listen_loop(self):
        raise NotImplementedError


class VoiceTiltListener(_PendingTiltListener):
    """Microphone input via Vosk - see module docstring."""

    def _listen_loop(self):
        model_path = resolve_model_path()
        print(f"[voice] Loading Vosk model from {model_path}...")
        model = Model(str(model_path))

        audio = pyaudio.PyAudio()
        device = select_input_device(audio)
        sample_rate = int(device["defaultSampleRate"])
        print(f"[voice] Using input device: {device['name']} (index {device['index']}) @ {sample_rate}Hz")

        recognizer = KaldiRecognizer(model, sample_rate)
        stream = audio.open(
            format=pyaudio.paInt16, channels=1, rate=sample_rate,
            input=True, input_device_index=device["index"], frames_per_buffer=8000,
        )

        tilts_str = ", ".join(f"{t:.0f}={label}" for t, (key, label) in sorted(VALID_TILTS.items()))
        print(f"[voice] Ready. Say a tilt number: {tilts_str}")

        try:
            while self._running:
                data = stream.read(4000, exception_on_overflow=False)
                if not recognizer.AcceptWaveform(data):
                    # Diagnostic only - partial (in-progress) results print
                    # live so it's visible whether Vosk is hearing ANYTHING
                    # at all, not just whether it ever reaches a finished
                    # "final" result. If this never shows anything while
                    # you're talking, the problem is upstream of
                    # recognition entirely (mic not actually capturing),
                    # not transcription quality.
                    partial = json.loads(recognizer.PartialResult()).get("partial", "").strip()
                    if partial:
                        print(f"\r[voice] (listening) {partial:<60}", end="", flush=True)
                    continue
                text = json.loads(recognizer.Result()).get("text", "").strip()
                if not text:
                    continue
                print(f"\n[voice] heard: {text}")

                value = _extract_angle(text)
                if value is None:
                    _log_transcript(text, None, "rejected: no number found in text")
                    _beep_error()
                    continue
                snapped = _snap_to_valid_tilt(value)
                if snapped is None:
                    print(f"[voice] heard {value:.0f}, not close enough to a real tilt - ignored")
                    _log_transcript(text, value, "rejected: not close enough to a real tilt")
                    _beep_error()
                    continue
                tilt, key, label = snapped
                self._set_pending(tilt, key, label)
                _beep_done()
                print(f"[voice] next punch armed: tilt={tilt:.0f} ({label})")
                _log_transcript(text, value, f"armed: {label} (tilt={tilt:.0f})")
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()


class TextTiltListener(_PendingTiltListener):
    """Keyboard input alternative to VoiceTiltListener - same
    pending-command interface, so track_interpolation_270.py doesn't need
    to know or care which one it's using. Type a number, Enter to arm it."""

    def _listen_loop(self):
        tilts_str = ", ".join(f"{t:.0f}={label}" for t, (key, label) in sorted(VALID_TILTS.items()))
        print(f"[text] Type a tilt number, Enter to arm it: {tilts_str}. Ctrl+C to stop.")
        while self._running:
            try:
                line = input("> ")
            except EOFError:
                break
            value = _extract_angle(line)
            if value is None:
                print("[text] couldn't parse that - try one of the numbers above")
                continue
            snapped = _snap_to_valid_tilt(value)
            if snapped is None:
                print(f"[text] heard {value:.0f}, not close enough to a real tilt - try one of the numbers above")
                continue
            tilt, key, label = snapped
            self._set_pending(tilt, key, label)
            print(f"[text] next punch armed: tilt={tilt:.0f} ({label})")


def create_listener():
    """Returns the listener matching INPUT_MODE above - the only thing
    that needs to know which concrete class is active."""
    if INPUT_MODE == "text":
        return TextTiltListener()
    return VoiceTiltListener()


def _standalone_test():
    listener = create_listener()
    listener.start()
    print(f"Input mode: {INPUT_MODE}. Arm a tilt (camera/Arduino not needed for this test). Ctrl+C to quit.")
    try:
        while True:
            pending = listener.pop_next_tilt()
            if pending:
                print(f">>> ARMED: {pending}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        listener.stop()
        print("\nStopped.")


if __name__ == "__main__":
    _standalone_test()
