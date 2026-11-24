"""Mic -> FFT -> one drawing speed.

Records a few seconds from the microphone, finds the dominant frequency with
an FFT, and maps it to a single speed used for the whole drawing:
low pitch = slower, high pitch = faster.

Usage from a shape module (e.g. square.py):

    from .audio_speed import draw_audio
    draw_audio("square_audio", make_waypoints, "square")
"""
import numpy as np
import sounddevice as sd

import rclpy
from moveit_msgs.msg import MoveItErrorCodes

from .drawing import Drawer

RECORD_SECONDS = 3.0
SAMPLE_RATE = 44100
MIN_SPEED = 0.10     # fraction of planned speed at FREQ_LO
MAX_SPEED = 0.40     # fraction of planned speed at FREQ_HI
FREQ_LO = 100.0      # Hz
FREQ_HI = 2000.0     # Hz


def listen_for_speed(log) -> float:
    """Record from the mic and return a speed in [MIN_SPEED, MAX_SPEED]."""
    log.info(f"Listening for {RECORD_SECONDS:.0f}s...")
    audio = sd.rec(int(RECORD_SECONDS * SAMPLE_RATE), samplerate=SAMPLE_RATE,
                   channels=1, dtype="float64")
    sd.wait()
    audio = audio[:, 0]

    mag = np.abs(np.fft.rfft(audio * np.hanning(len(audio))))
    freqs = np.fft.rfftfreq(len(audio), d=1.0 / SAMPLE_RATE)

    # Only look in the band we care about; ignore rumble and hiss.
    band = (freqs >= FREQ_LO) & (freqs <= FREQ_HI)
    if not band.any() or mag[band].max() < 1e-6:
        log.warn("No sound detected, using minimum speed.")
        return MIN_SPEED

    peak = freqs[band][np.argmax(mag[band])]
    norm = np.log(peak / FREQ_LO) / np.log(FREQ_HI / FREQ_LO)   # 0..1
    speed = MIN_SPEED + norm * (MAX_SPEED - MIN_SPEED)
    log.info(f"Dominant frequency {peak:.0f} Hz -> speed {speed:.2f}")
    return speed


def draw_audio(node_name: str, make_waypoints, label: str):
    """Like drawing.draw(), but speed comes from the mic."""
    rclpy.init()
    node = Drawer(node_name)
    log = node.get_logger()
    try:
        speed = listen_for_speed(log)

        node.add_table()
        log.info("Moving to ready pose...")
        code = node.move_to_ready()
        if code != MoveItErrorCodes.SUCCESS:
            log.error(f"Could not reach ready pose, MoveIt error code {code}")
            return
        node.settle()

        start = node.current_pose()
        response = node.plan(make_waypoints(start))
        log.info(f"Planned fraction: {response.fraction:.3f}")
        if response.fraction < 0.999:
            log.error(f"Could not plan the full {label}.")
            return

        code = node.execute(node.slow_down(response.solution, speed))
        if code == MoveItErrorCodes.SUCCESS:
            log.info(f"{label.capitalize()} complete.")
        else:
            log.error(f"Execution failed, MoveIt error code {code}")
    finally:
        node.destroy_node()
        rclpy.shutdown()