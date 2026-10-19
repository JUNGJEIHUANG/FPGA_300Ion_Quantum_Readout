#!/usr/bin/env python3
"""P95 automatic exposure controller for GSENSE6504BSI STD mode.

The controller is hardware-I/O independent: one call consumes one uint16 numpy
frame and returns the next exposure/gain state plus the corresponding register
fields.  The caller is responsible for applying updates between frames.

STD external-trigger controls used here
---------------------------------------
Exposure (FPGA register bus, 16-bit words):
  0x0103  inte_linetime[15:0]
  0x0104  inte_linetime[31:16]
Sensor timing (fixed during AE):
  0x0070/0x0071 LINE_TIME = 325, hence TLINE = 6.5 us at CLK_SEQ=100 MHz.
Analog gain (sensor byte register bus; read-modify-write fields):
  0x00A0[0]   COARSE_GAIN: 1=x1, 0=x5.6
  0x00A1[4:0] HIGH_GAIN code (STD image gain)
No digital gain is used. FPGA 0x1002 image_gain is not analog gain.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

import numpy as np

# GSENSE6504BSI datasheet Table 35, HIGH_GAIN code 0..29.
HIGH_GAIN_LUT = np.asarray([
    0.53, 0.57, 0.63, 0.68, 0.74, 0.80, 0.88, 0.95,
    1.04, 1.12, 1.22, 1.32, 1.45, 1.57, 1.71, 1.85,
    2.04, 2.23, 2.45, 2.69, 3.02, 3.35, 3.78, 4.26,
    5.01, 5.78, 6.87, 8.23, 10.50, 13.43,
], dtype=np.float64)


@dataclass(frozen=True)
class GainSetting:
    analog_gain: float
    coarse_gain_bit: int  # Sensor 0xA0[0]: 1=x1, 0=x5.6
    high_gain_code: int   # Sensor 0xA1[4:0]


def supported_gain_settings(min_gain: float, max_gain: float) -> tuple[GainSetting, ...]:
    """Return monotonic, hardware-realizable STD gains inside user limits."""
    if not (0 < min_gain <= max_gain):
        raise ValueError("require 0 < min_analog_gain <= max_analog_gain")
    settings = []
    for coarse_bit, multiplier in ((1, 1.0), (0, 5.6)):
        for code, high_gain in enumerate(HIGH_GAIN_LUT):
            gain = float(high_gain * multiplier)
            # Datasheet specifies the maximum combined STD gain nominally as
            # x75.2; 13.43 and 5.6 are rounded component values.
            if coarse_bit == 0 and code == 29:
                gain = 75.2
            if min_gain - 1e-12 <= gain <= max_gain + 1e-12:
                settings.append(GainSetting(gain, coarse_bit, code))
    if not settings:
        raise ValueError("analog-gain limits contain no realizable sensor setting")
    # Prefer coarse x1 when two encodings are effectively equivalent.
    settings.sort(key=lambda item: (item.analog_gain, -item.coarse_gain_bit))
    return tuple(settings)


def select_gain(settings: tuple[GainSetting, ...], target: float,
                direction: Literal["nearest", "up", "down"] = "nearest") -> GainSetting:
    """Quantize a requested analog gain to the sensor LUT."""
    if direction == "up":
        return next((item for item in settings if item.analog_gain >= target - 1e-12), settings[-1])
    if direction == "down":
        return next((item for item in reversed(settings) if item.analog_gain <= target + 1e-12), settings[0])
    # Relative/log distance is appropriate because brightness is multiplicative.
    return min(settings, key=lambda item: abs(math.log(item.analog_gain / target)))


@dataclass(frozen=True)
class AEConfig:
    # Hardware/application limits must be supplied explicitly; they are not
    # universal sensor constants.
    min_exposure_us: float
    max_exposure_us: float
    min_analog_gain: float
    max_analog_gain: float

    full_scale: int = 4095
    target_p95: float = 2500.0
    p95_high_thresh: float = 3300.0
    p95_low_thresh: float = 800.0
    saturation_pixel_threshold: int = 500

    # Damped multiplicative update: factor = ratio**damping_alpha, then clamped.
    # A 0.70 exponent remains a damped step (never the full ratio), while
    # allowing a five-frame hardware loop to recover from very dark scenes.
    damping_alpha: float = 0.70
    max_brighten_factor_per_frame: float = 6.00
    min_darken_factor_per_frame: float = 0.67
    saturation_exposure_factor: float = 0.46

    # Official Normal STD 300-fps setting. Keep fixed during AE.
    line_time_count: int = 325
    frame_line_count: int = 676
    fpga_tick_us: float = 0.020  # software/HDL timing unit = 20 ns

    def __post_init__(self) -> None:
        if not (0 < self.min_exposure_us <= self.max_exposure_us):
            raise ValueError("invalid exposure limits")
        if self.min_exposure_us > self.max_external_exposure_us:
            raise ValueError("minimum exposure exceeds fixed STD frame-time limit")
        if not (0 < self.min_analog_gain <= self.max_analog_gain):
            raise ValueError("invalid analog-gain limits")
        if not (0 < self.p95_low_thresh < self.target_p95 < self.p95_high_thresh <= self.full_scale):
            raise ValueError("require low_thresh < target < high_thresh <= full_scale")
        if not (0 < self.damping_alpha <= 1):
            raise ValueError("damping_alpha must be in (0, 1]")
        if not (self.max_brighten_factor_per_frame >= 1):
            raise ValueError("max_brighten_factor_per_frame must be >= 1")
        if not (0 < self.min_darken_factor_per_frame <= 1):
            raise ValueError("min_darken_factor_per_frame must be in (0, 1]")
        if not (0 < self.saturation_exposure_factor < 1):
            raise ValueError("saturation_exposure_factor must be in (0, 1)")

    @property
    def line_time_us(self) -> float:
        # For STD, TLINE = LINE_TIME * 20 ns = 2*LINE_TIME*TCLK_SEQ.
        return self.line_time_count * self.fpga_tick_us

    @property
    def max_external_exposure_us(self) -> float:
        # trigger_gen.v subtracts integration time from frame time.  Retain a
        # one-line margin so fixed-frame STD operation cannot underflow and
        # lose VSYNC.  AE intentionally does not change frame-line registers.
        return (self.frame_line_count - 1) * self.line_time_us


@dataclass(frozen=True)
class AEState:
    exposure_us: float
    analog_gain: float


@dataclass(frozen=True)
class AEResult:
    exposure_us: float
    analog_gain: float
    p95: float
    saturated_pixels: int
    raw_ratio: float
    applied_ratio: float
    reason: str
    exposure_limited: bool
    gain_limited: bool
    gain_setting: GainSetting
    line_time_us: float

    @property
    def fpga_exposure_registers(self) -> tuple[tuple[int, int], ...]:
        """The only two FPGA exposure writes used by AE."""
        return encode_exposure_registers(self.exposure_us, self.line_time_us)

    @property
    def sensor_gain_fields(self) -> tuple[tuple[int, int, int, int], ...]:
        """(address, bit_offset, bit_width, value); apply as read-modify-write."""
        return gain_setting_fields(self.gain_setting)


def encode_exposure_registers(exposure_us: float, line_time_us: float = 6.5) -> tuple[tuple[int, int], ...]:
    """Encode a whole-TLINE exposure using only FPGA 0x0103 and 0x0104."""
    line_count = int(round(exposure_us / line_time_us))
    if not 1 <= line_count <= 0xFFFFFFFF:
        raise ValueError("exposure line count must fit in 32 bits and be nonzero")
    return ((0x0103, line_count & 0xFFFF),
            (0x0104, (line_count >> 16) & 0xFFFF))


def gain_setting_fields(setting: GainSetting) -> tuple[tuple[int, int, int, int], ...]:
    """Return the only two sensor gain fields adjusted by STD AE."""
    return ((0x00A0, 0, 1, setting.coarse_gain_bit),
            (0x00A1, 0, 5, setting.high_gain_code))


def _quantize_exposure(exposure_us: float, cfg: AEConfig,
                       direction: Literal["nearest", "up", "down"] = "nearest") -> float:
    """Quantize external-trigger width to a whole TLINE as required by datasheet."""
    lo = max(1, math.ceil(cfg.min_exposure_us / cfg.line_time_us - 1e-12))
    hardware_max = min(cfg.max_exposure_us, cfg.max_external_exposure_us)
    hi = max(lo, math.floor(hardware_max / cfg.line_time_us + 1e-12))
    value = exposure_us / cfg.line_time_us
    if direction == "up":
        lines = math.ceil(value - 1e-12)
    elif direction == "down":
        lines = math.floor(value + 1e-12)
    else:
        lines = round(value)
    return min(max(lines, lo), hi) * cfg.line_time_us


def _damped_ratio(raw_ratio: float, cfg: AEConfig) -> float:
    if not math.isfinite(raw_ratio):
        return cfg.max_brighten_factor_per_frame
    factor = raw_ratio ** cfg.damping_alpha
    return min(cfg.max_brighten_factor_per_frame,
               max(cfg.min_darken_factor_per_frame, factor))


def _validate_image(image: np.ndarray, full_scale: int) -> np.ndarray:
    if not isinstance(image, np.ndarray) or image.ndim != 2 or image.size == 0:
        raise ValueError("image must be a non-empty 2-D numpy array")
    if not np.issubdtype(image.dtype, np.integer):
        raise TypeError("raw 12-bit image must use an integer numpy dtype")
    if int(image.min()) < 0 or int(image.max()) > full_scale:
        raise ValueError(f"image contains values outside 0..{full_scale}")
    return image


def update_ae(image: np.ndarray, state: AEState, cfg: AEConfig) -> AEResult:
    """Run one P95 AE iteration and return hardware-realizable next settings."""
    image = _validate_image(image, cfg.full_scale)
    settings = supported_gain_settings(cfg.min_analog_gain, cfg.max_analog_gain)

    exposure = _quantize_exposure(state.exposure_us, cfg)
    gain_setting = select_gain(settings, state.analog_gain)
    gain = gain_setting.analog_gain
    p95 = float(np.percentile(image, 95))
    saturated = int(np.count_nonzero(image == cfg.full_scale))
    raw_ratio = math.inf if p95 <= 0 else cfg.target_p95 / p95

    min_exp = _quantize_exposure(cfg.min_exposure_us, cfg, "up")
    max_exp = _quantize_exposure(cfg.max_exposure_us, cfg, "down")

    if saturated > cfg.saturation_pixel_threshold:
        # Saturation has priority over the P95 loop. Exposure is reduced first;
        # if already at its minimum, analog gain is reduced as the only remaining
        # non-digital brightness control.
        factor = cfg.saturation_exposure_factor
        new_exposure = _quantize_exposure(exposure * factor, cfg, "down")
        if new_exposure < exposure:
            exposure = new_exposure
            reason = "saturation: reduce exposure"
        else:
            gain_setting = select_gain(settings, gain * factor, "down")
            gain = gain_setting.analog_gain
            reason = "saturation: exposure at minimum, reduce analog gain"
        applied = factor
    else:
        applied = _damped_ratio(raw_ratio, cfg)
        if applied > 1.0 + 1e-12:
            # Brighten: spend the permitted factor on exposure first, then gain.
            old_exposure = exposure
            exposure = _quantize_exposure(min(max_exp, exposure * applied), cfg, "up")
            used = exposure / old_exposure
            residual = max(1.0, applied / used)
            if exposure >= max_exp - 1e-12 and residual > 1.0 + 1e-12:
                # Quantize to the closest physical LUT step.  Always rounding
                # upward can skip a near-target code (for example 23.856x) and
                # jump directly to a saturating 28.056x setting.
                gain_setting = select_gain(settings, gain * residual, "nearest")
                gain = gain_setting.analog_gain
            reason = "brighten: exposure first" if p95 >= cfg.p95_low_thresh else "dark: increase exposure first"
        elif applied < 1.0 - 1e-12:
            # Darken: shorten exposure first, then reduce analog gain.
            old_exposure = exposure
            exposure = _quantize_exposure(max(min_exp, exposure * applied), cfg, "down")
            used = exposure / old_exposure
            residual = min(1.0, applied / used)
            if exposure <= min_exp + 1e-12 and residual < 1.0 - 1e-12:
                gain_setting = select_gain(settings, gain * residual, "down")
                gain = gain_setting.analog_gain
            reason = "very bright: reduce exposure first" if p95 > cfg.p95_high_thresh else "reduce exposure first"
        else:
            reason = "hold"

    exposure_limited = exposure >= max_exp - 1e-12 if raw_ratio > 1 else exposure <= min_exp + 1e-12
    gain_limited = gain_setting == settings[-1] if raw_ratio > 1 else gain_setting == settings[0]
    return AEResult(
        exposure_us=exposure,
        analog_gain=gain,
        p95=p95,
        saturated_pixels=saturated,
        raw_ratio=raw_ratio,
        applied_ratio=applied,
        reason=reason,
        exposure_limited=exposure_limited,
        gain_limited=gain_limited,
        gain_setting=gain_setting,
        line_time_us=cfg.line_time_us,
    )


def build_sensor_gain_write_sequence(result: AEResult, current_reg04: int,
                                     current_reg_a0: int, current_reg_a1: int,
                                     effective_frames: int = 1) -> tuple[tuple[int, int], ...]:
    """Build synchronized byte writes for category-A STD gain registers.

    The caller must first read 0x04, 0xA0 and 0xA1. This function preserves all
    unrelated bits. It asserts GRP_CFG_HOLD, writes gain, then creates the
    required falling edge. Apply the sequence while following frame/IDLE timing.
    """
    return build_sensor_gain_sequence_from_setting(
        result.gain_setting, current_reg04, current_reg_a0, current_reg_a1,
        effective_frames)


def build_sensor_gain_sequence_from_setting(setting: GainSetting, current_reg04: int,
                                            current_reg_a0: int, current_reg_a1: int,
                                            effective_frames: int = 1) -> tuple[tuple[int, int], ...]:
    """Build the group-hold sequence for a selected STD analog gain."""
    if not 1 <= effective_frames <= 15:
        raise ValueError("effective_frames must be 1..15")
    held04 = (current_reg04 & 0x0E) | (effective_frames << 4) | 0x05
    released04 = held04 & ~0x01
    new_a0 = (current_reg_a0 & ~0x01) | setting.coarse_gain_bit
    new_a1 = (current_reg_a1 & ~0x1F) | setting.high_gain_code
    return ((0x0004, held04), (0x00A0, new_a0), (0x00A1, new_a1), (0x0004, released04))


if __name__ == "__main__":
    # Standalone example only. Replace the four limits with application-approved
    # hardware limits before connecting the returned register values to USB I/O.
    config = AEConfig(
        min_exposure_us=13.0,
        max_exposure_us=20_000.0,
        min_analog_gain=1.04,
        max_analog_gain=75.2,
    )
    frame = np.full((2048, 2176), 1000, dtype=np.uint16)
    current = AEState(exposure_us=2002.0, analog_gain=1.04)
    result = update_ae(frame, current, config)
    print(result)
    print("FPGA exposure writes:", result.fpga_exposure_registers)
    print("Sensor gain fields:", result.sensor_gain_fields)
