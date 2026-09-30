"""
Boxing Robot Arm - Voice-Cued Punch Selection
------------------------------------------------
Say a punch name before throwing it (e.g. "hook", or "high overhand") -
this sidesteps the hardest unsolved problem in punch_classifier.py
(classifying punch TYPE from ambiguous vision alone - see its docstring
and CLAUDE.md's "Current direction" section) instead of continuing to
chase it. The speed/travel gate in PunchClassifier.update() already
reliably detects THAT a strike happened; this module just tells the main
loop WHICH of the punch_angle_test.py family it's about to be, declared in
advance by the person throwing it.

PUNCH_TILTS is imported directly from punch_angle_test.py - that's the
single source of truth for which punches exist and what tilt each one
uses (hand-tuned from real testing: Uppercut/High Overhand/Overhand/Hook),
so the keyboard legend there and the voice commands here can't drift out
of sync. Each named punch already has its own pre-tuned tilt + yaw target
built in - just naming one is enough. An optional trailing number still
overrides that punch's default yaw depth for this one throw (e.g. "hook
forty five"), without changing which tilt it uses.

Reuses the exact offline speech stack from a separate project (BackTPal):
Vosk (KaldiRecognizer) over a live PyAudio stream, fully offline - no
network dependency or per-request latency during a live demo, which
matters more here than in BackTPal since this has to keep up with a real
punch. Defaults to BackTPal's already-downloaded model directory to avoid
pulling a redundant ~1.8GB copy; falls back to downloading its own into
this folder if that path doesn't exist (e.g. on a different machine).
words_to_number() below is copied verbatim from BackTPal's
transcribe_live.py - same reasoning: it already turns spoken numbers like
"forty five" into 45.0, no need to re-solve that.

Runs in its own background thread so it never blocks the camera loop -
call start() once, then pop_next_punch() from the main tracking loop each
frame (only meaningful right when a strike is actually detected - see
track_interpolation.py).

Standalone test (no camera/Arduino needed):
    python voice_punch.py
    Say (or type, in text mode) a punch name. Ctrl+C to quit.
"""

import json
import re
import threading
import time
import winsound
import zipfile
from pathlib import Path

import pyaudio
import requests
from vosk import KaldiRecognizer, Model

from punch_angle_test import PUNCH_TILTS, YAW_END  # noqa: E402 - single source of truth, see above

# ==================== TUNABLE SETTINGS ====================

MODEL_NAME = "vosk-model-en-us-0.22"
BASE_DIR = Path(__file__).resolve().parent

# Reuse the model already downloaded for BackTPal rather than pulling
# another ~1.8GB copy. Falls back to downloading a local copy into this
# folder if that project ever moves or gets deleted.
_BACKTPAL_MODEL_DIR = Path(r"C:\Users\Mekan\Desktop\BackTPal") / MODEL_NAME / MODEL_NAME
_LOCAL_MODEL_DIR = BASE_DIR / MODEL_NAME
_MODEL_URL = f"https://alphacephei.com/vosk/models/{MODEL_NAME}.zip"

# A pending {"type", "tilt", "yaw"} expires if never consumed by a real
# detected strike within this long - without this, naming a punch and then
# forgetting, then later throwing an unrelated ordinary punch that happens
# to pass the speed/travel gate, would silently play that animation for it.
PENDING_EXPIRY_SEC = 8.0

# "voice" (microphone, Vosk) or "text" (type a line + Enter, e.g. "hook") -
# text mode exists for testing without speaking out loud (e.g. late at
# night with someone asleep nearby). Both expose the exact same
# pending-command interface (start/stop/pop_next_punch/peek_next_punch),
# so create_listener() is the only thing that needs to know which is
# active - track_interpolation.py doesn't care.
INPUT_MODE = "text"

# ============================================================


def words_to_number(text: str):
    """Convert spoken number words into a numeric value. Copied verbatim
    from BackTPal/transcribe_live.py - same problem (Vosk transcribes
    numbers as words, e.g. "forty five"), no need to re-solve it."""
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
    """Plain digits first (Vosk sometimes emits '45' directly rather than
    the word form), then spoken number words as a fallback - filtered to
    just the number-word vocabulary first (same approach as BackTPal's
    parse_spoken_profit), since words_to_number() aborts on the first
    unrecognized token and a real utterance is often "forty five degrees",
    not just "forty five"."""
    match = re.search(r"-?\d+(?:\.\d+)?", text)
    if match:
        return float(match.group(0))
    filtered = " ".join(t for t in text.lower().split() if t in _NUMBER_WORDS)
    if not filtered:
        return None
    return words_to_number(filtered)


def _beep_ready():
    winsound.Beep(880, 120)


def _beep_done():
    winsound.Beep(660, 80)
    time.sleep(0.05)
    winsound.Beep(880, 120)


def _beep_error():
    winsound.Beep(300, 200)


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


# Punch names ordered longest-first ("high overhand" before "overhand") so
# a substring check can't match the shorter name first just because it
# happens to be contained inside the longer one.
_PUNCH_NAMES_BY_LENGTH = sorted(PUNCH_TILTS, key=len, reverse=True)


def _parse_punch(text: str):
    """Given a recognized/typed utterance, return (punch_type, tilt, yaw)
    if it names one of PUNCH_TILTS's punches, else None. Naming the punch
    alone is enough - it already has its own pre-tuned tilt and yaw
    target. An optional trailing number overrides just the yaw depth for
    this one throw (e.g. "hook forty five" keeps hook's tilt, strikes
    deeper/shallower than its default)."""
    lowered = text.lower()
    punch_type = None
    for candidate in _PUNCH_NAMES_BY_LENGTH:
        if candidate in lowered:
            punch_type = candidate
            break
    if punch_type is None:
        return None
    angle = _extract_angle(lowered)
    yaw = angle if angle is not None else YAW_END
    return punch_type, PUNCH_TILTS[punch_type], yaw


class _PendingPunchListener:
    """Shared pending-command state for any input method - subclasses only
    need to implement _listen_loop() and call _set_pending() from it.
    pop_next_punch()/peek_next_punch() are the only methods
    track_interpolation.py actually calls, so any subclass here is a
    drop-in replacement for another (see create_listener())."""

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = None  # {"type", "tilt", "yaw", "armed_at"} or None
        self._thread = None
        self._running = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def pop_next_punch(self):
        """Returns the pending {"type", "tilt", "yaw"} dict and clears it,
        or None if nothing's pending or the pending command is too stale
        (see PENDING_EXPIRY_SEC) - consumed exactly once, so an old
        command can't silently reapply to a later, unrelated motion."""
        with self._lock:
            pending = self._pending
            self._pending = None
        if pending is None:
            return None
        if time.monotonic() - pending["armed_at"] > PENDING_EXPIRY_SEC:
            print(f"[input] pending {pending['type'].upper()} expired unused")
            return None
        return {"type": pending["type"], "tilt": pending["tilt"], "yaw": pending["yaw"]}

    def peek_next_punch(self):
        """Non-consuming read, for status display only."""
        with self._lock:
            return None if self._pending is None else dict(self._pending)

    def _set_pending(self, punch_type, tilt, yaw):
        with self._lock:
            self._pending = {"type": punch_type, "tilt": tilt, "yaw": yaw, "armed_at": time.monotonic()}

    def _listen_loop(self):
        raise NotImplementedError


class VoicePunchListener(_PendingPunchListener):
    """Microphone input via Vosk - see module docstring."""

    def _listen_loop(self):
        model_path = resolve_model_path()
        print(f"[voice] Loading Vosk model from {model_path}...")
        model = Model(str(model_path))

        audio = pyaudio.PyAudio()
        default_input = audio.get_default_input_device_info()
        sample_rate = int(default_input["defaultSampleRate"])
        print(f"[voice] Using input device: {default_input['name']} @ {sample_rate}Hz")

        recognizer = KaldiRecognizer(model, sample_rate)
        stream = audio.open(
            format=pyaudio.paInt16, channels=1, rate=sample_rate,
            input=True, frames_per_buffer=8000,
        )

        print(f"[voice] Ready. Say a punch name: {', '.join(PUNCH_TILTS)} "
              f"(optionally add a number to override its default depth).")

        try:
            while self._running:
                data = stream.read(4000, exception_on_overflow=False)
                if not recognizer.AcceptWaveform(data):
                    continue
                text = json.loads(recognizer.Result()).get("text", "").strip()
                if not text:
                    continue
                print(f"[voice] heard: {text}")

                parsed = _parse_punch(text)
                if parsed is None:
                    _beep_error()
                    continue
                punch_type, tilt, yaw = parsed
                self._set_pending(punch_type, tilt, yaw)
                _beep_done()
                print(f"[voice] next punch armed: {punch_type.upper()} (tilt={tilt:.0f}, yaw={yaw:.0f})")
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()


class TextPunchListener(_PendingPunchListener):
    """Keyboard input alternative to VoicePunchListener - same
    pending-command interface, so track_interpolation.py doesn't need to
    know or care which one it's using. Type a punch name, Enter to arm it
    - e.g. "hook", "high overhand 30" to override the depth for that
    throw only."""

    def _listen_loop(self):
        print(f"[text] Type a punch name, Enter to arm it: {', '.join(PUNCH_TILTS)}. "
              f"Add a number to override its default depth. Ctrl+C to stop.")
        while self._running:
            try:
                line = input("> ")
            except EOFError:
                break
            parsed = _parse_punch(line)
            if parsed is None:
                print(f"[text] couldn't parse that - try one of: {', '.join(PUNCH_TILTS)}")
                continue
            punch_type, tilt, yaw = parsed
            self._set_pending(punch_type, tilt, yaw)
            print(f"[text] next punch armed: {punch_type.upper()} (tilt={tilt:.0f}, yaw={yaw:.0f})")


def create_listener():
    """Returns the listener matching INPUT_MODE above - the only thing
    that needs to know which concrete class is active."""
    if INPUT_MODE == "text":
        return TextPunchListener()
    return VoicePunchListener()


def _standalone_test():
    listener = create_listener()
    listener.start()
    print(f"Input mode: {INPUT_MODE}. Arm a punch command (camera/Arduino not needed for this test). Ctrl+C to quit.")
    try:
        while True:
            punch = listener.pop_next_punch()
            if punch:
                print(f">>> ARMED: {punch}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        listener.stop()
        print("\nStopped.")


if __name__ == "__main__":
    _standalone_test()
