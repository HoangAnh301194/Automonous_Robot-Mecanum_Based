"""
Temporal smoothing state machine for wave detection.

Prevents flickering by requiring a minimum number of *consecutive*
frames before transitioning between IDLE ↔ WAVE states.

State diagram
─────────────
  ┌─────────┐  raised ≥ raise_frames   ┌──────────┐
  │  IDLE   │ ──────────────────────▶  │  WAVING  │
  └─────────┘                          └──────────┘
       ▲   not-raised ≥ lower_frames       │
       └───────────────────────────────────┘
"""

from enum import Enum, auto
from typing import Tuple


class WaveState(Enum):
    """Two-state machine: the person is either waving or idle."""
    IDLE   = auto()
    WAVING = auto()


class WaveStateMachine:
    """
    Tracks *one* person's waving status across frames.

    Parameters
    ----------
    raise_frames : int
        Number of consecutive "hand raised" frames to trigger WAVING.
    lower_frames : int
        Number of consecutive "hand NOT raised" frames to return to IDLE.
    """

    def __init__(self, raise_frames: int = 5, lower_frames: int = 8) -> None:
        self.raise_frames = raise_frames
        self.lower_frames = lower_frames

        self._state: WaveState = WaveState.IDLE
        self._raised_counter: int = 0
        self._idle_counter: int = 0

        # Which hand(s) caused the current wave
        self._active_hand: str = ""   # "LEFT", "RIGHT", "BOTH", or ""

    # ─────────────────────────────────────────────────────────────────
    def update(self, left_raised: bool, right_raised: bool) -> None:
        """
        Feed the per-frame raise result into the state machine.

        Parameters
        ----------
        left_raised : bool
            Is the left wrist above the head this frame?
        right_raised : bool
            Is the right wrist above the head this frame?
        """
        any_raised = left_raised or right_raised

        if any_raised:
            self._raised_counter += 1
            self._idle_counter = 0       # reset the idle streak

            # Record which hand(s)
            if left_raised and right_raised:
                self._active_hand = "BOTH"
            elif left_raised:
                self._active_hand = "LEFT"
            else:
                self._active_hand = "RIGHT"
        else:
            self._idle_counter += 1
            self._raised_counter = 0     # reset the raise streak

        # ── State transitions ────────────────────────────────────────
        if self._state == WaveState.IDLE:
            if self._raised_counter >= self.raise_frames:
                self._state = WaveState.WAVING

        elif self._state == WaveState.WAVING:
            if self._idle_counter >= self.lower_frames:
                self._state = WaveState.IDLE
                self._active_hand = ""

    # ─────────────────────────────────────────────────────────────────
    @property
    def is_waving(self) -> bool:
        """True while the state machine is in WAVING state."""
        return self._state == WaveState.WAVING

    @property
    def active_hand(self) -> str:
        """The hand(s) that triggered the wave: LEFT / RIGHT / BOTH / ''."""
        return self._active_hand

    @property
    def state(self) -> WaveState:
        return self._state

    # ─────────────────────────────────────────────────────────────────
    def reset(self) -> None:
        """Reset the machine to IDLE."""
        self._state = WaveState.IDLE
        self._raised_counter = 0
        self._idle_counter = 0
        self._active_hand = ""
