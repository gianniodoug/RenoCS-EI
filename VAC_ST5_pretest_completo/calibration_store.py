"""Strict calibration file validation. No defaults, never guesses geometry."""
import json
from pathlib import Path
from encoder_odometry import EncoderCalibration

REQUIRED = ('ticks_per_wheel_rev','wheel_diameter_m','forward_signs','effective_track_width_m')

def load_calibration(path):
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    missing=[k for k in REQUIRED if k not in data or data[k] is None]
    if missing: raise ValueError('Missing physical calibration: '+', '.join(missing))
    if not data.get('verified_on_robot', False):
        raise ValueError('Calibration has not been confirmed on real robot')
    return EncoderCalibration(*(data[k] for k in REQUIRED))
