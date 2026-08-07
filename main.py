from __future__ import annotations

import argparse
import copy
import glob
import json
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from dynamixel_sdk import COMM_SUCCESS, GroupSyncWrite, PacketHandler, PortHandler


@dataclass(frozen=True)
class ControlTable:
    id: int = 7
    operating_mode: int = 11
    torque_enable: int = 64
    led: int = 65
    goal_position: int = 116
    present_position: int = 132
    moving: int = 122


DEFAULT_BAUDRATE = 1_000_000
DEFAULT_PROTOCOL = 2.0
DEFAULT_TIMEOUT_SECONDS = 5.0
DEFAULT_DISCOVERY_BAUDRATES = (1_000_000, 57_600, 115_200, 2_000_000, 3_000_000, 4_000_000, 9_600)
DEFAULT_DISCOVERY_PROTOCOLS = (2.0, 1.0)
DEFAULT_POSITION_MIN = 0
DEFAULT_POSITION_MAX = 4095
DEFAULT_TEST_DELTA = 20
DEFAULT_CONFIG_PATH = "robot_config.json"
WIRELESS_RETRY_ATTEMPTS = 5
WIRELESS_RETRY_DELAY_SECONDS = 0.01
WIRELESS_MINIMUM_PACKET_TIMEOUT_MS = 120.0
POSITION_CONTROL_MODE = 3
TABLE = ControlTable()

SERVO_COUNT = 28
PARAM_COUNT = 12  # v1 gait model; see GaitModel.param_count for per-model counts
LEG_COUNT = 8
BODY_COUNT = 3
PI_F = math.pi
TWO_PI_F = 2.0 * math.pi
ACTION_FILTER_TAU = 0.12
MAX_DT = 0.05
CONTROL_PERIOD_SECONDS = 0.02
INITIAL_POSITION_DURATION_SECONDS = 2.0
BODY_SLIDER_SEND_INTERVAL_MS = 20
GAIT_TEST_MODES = ("full", "lift", "ground")
# A 57600 bps link carries 5760 bytes/s; one sync write is ~119 bytes, so 40 Hz
# already spends 83% of the link and starves reads. 30 Hz leaves room.
DEFAULT_WIRELESS_CONTROL_HZ = 30.0
DEFAULT_WIRED_CONTROL_HZ = 50.0
TICKS_PER_RAD = 4096.0 / TWO_PI_F

DEFAULT_ROBOT_IDS = [
    1, 2, 3, 4, 5, 6, 7,
    8, 9, 10, 11, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28,
]
JOINT_LABELS = [
    "body 1",
    "body 2",
    "body 3",
    "leg 4 left yaw",
    "leg 4 left lift",
    "leg 4 left knee",
    "leg 4 right yaw",
    "leg 4 right lift",
    "leg 4 right knee",
    "leg 3 left yaw",
    "leg 3 left lift",
    "leg 3 left knee",
    "leg 3 right yaw",
    "leg 3 right lift",
    "leg 3 right knee",
    "leg 2 left yaw",
    "leg 2 left lift",
    "leg 2 left knee",
    "leg 2 right yaw",
    "leg 2 right lift",
    "leg 2 right knee",
    "tail/body joint",
    "leg 1 left yaw",
    "leg 1 left lift",
    "leg 1 left knee",
    "leg 1 right yaw",
    "leg 1 right lift",
    "leg 1 right knee",
]
LEG4_INDICES = [3, 4, 5, 6, 7, 8]
TAIL_BODY_INDEX = 21
LIFT_KNEE_INDICES = [4, 5, 7, 8, 10, 11, 13, 14, 16, 17, 19, 20, 23, 24, 26, 27]
FULL_ENABLED_INDICES = list(range(SERVO_COUNT))
THREE_SEGMENT_ENABLED_INDICES = [
    index
    for index in FULL_ENABLED_INDICES
    if index not in LEG4_INDICES and index != TAIL_BODY_INDEX
]
DEFAULT_ZERO_TICKS = [2048] * SERVO_COUNT
DEFAULT_DIRECTIONS = [
    -1 if index in LIFT_KNEE_INDICES else 1
    for index in range(SERVO_COUNT)
]
JOINT_LOWER = [
    -0.436332, -0.436332, -0.436332,
    -1.570796, -1.570796, -1.570796,
    -1.570796, -1.570796, -1.570796,
    -1.570796, -1.570796, -1.570796,
    -1.570796, -1.570796, -1.570796,
    -1.570796, -1.570796, -1.570796,
    -1.570796, -1.570796, -1.570796,
    -0.785398,
    -1.570796, -1.570796, -1.570796,
    -1.570796, -1.570796, -1.570796,
]
JOINT_UPPER = [
    0.436332, 0.436332, 0.436332,
    1.570796, 1.570796, 1.570796,
    1.570796, 1.570796, 1.570796,
    1.570796, 1.570796, 1.570796,
    1.570796, 1.570796, 1.570796,
    1.570796, 1.570796, 1.570796,
    1.570796, 1.570796, 1.570796,
    0.785398,
    1.570796, 1.570796, 1.570796,
    1.570796, 1.570796, 1.570796,
]
BODY_JOINT_INDEX = [0, 1, 2]
FORWARD_GAIT_PARAMS = [
    0.601504,
    -0.427146,
    -0.115748,
    1.000000,
    -0.331842,
    1.000000,
    -0.268078,
    0.014573,
    -0.123508,
    0.061576,
    0.000000,
    0.306038,
]
SAFE_GAIT_PARAMS = [
    -0.800000,
    -0.700000,
    -0.650000,
    0.200000,
    -0.750000,
    1.000000,
    -0.268078,
    0.000000,
    -0.100000,
    0.000000,
    0.000000,
    0.306038,
]
DEFAULT_GAIT_PARAMS = SAFE_GAIT_PARAMS

GAIT_PARAM_SPECS = [
    ("Frequency", "Hz", 0.25, 1.20),
    ("Leg sweep", "", 0.08, 0.55),
    ("Leg lift", "", 0.08, 0.58),
    ("Knee fold", "", -0.50, 0.50),
    ("Body wave", "", 0.00, 0.38),
    ("Segment phase", "rad", -PI_F, PI_F),
    ("Left/right phase", "rad", 0.45 * PI_F, 1.35 * PI_F),
    ("Sweep bias", "", -0.20, 0.20),
    ("Lift bias", "", -0.18, 0.18),
    ("Knee bias", "", -0.18, 0.18),
    ("Body turn", "", -0.35, 0.35),
    ("Body phase", "rad", -PI_F, PI_F),
]
GAIT_PARAM_DESCRIPTIONS = [
    "歩行周期の速さ。全関節のCPG位相が進む速度を変える。",
    "各脚のyaw前後振幅。主に歩幅と接地中の蹴り量を変える。",
    "遊脚中のlift振幅。足先の持ち上げ高さを変える。",
    "遊脚中の膝曲げ量。符号で折り曲げ方向が変わる。",
    "胴体関節IDs 1〜3の周期的な曲げ振幅。",
    "前後セグメント間の脚位相差。π付近では隣接脚群が交互になる。",
    "左脚群と右脚群の位相差。π付近では左右が交互になる。",
    "yawの固定オフセット。足を置く前後中心位置をずらす。",
    "liftの固定オフセット。基本姿勢の高さ・接地圧をずらす。",
    "膝の固定オフセット。立脚時を含む基本の膝曲げ量を変える。",
    "胴体の固定曲げ。旋回方向または左右の姿勢偏りを作る。",
    "胴体関節間の位相差。胴体を伝わる波の向きと形を変える。",
]

# v2 drives yaw and lift from one shared fore-aft stroke, and couples the knee
# against the hip so the foot keeps its angle to the ground while the leg swings.
GAIT_PARAM_SPECS_V2 = [
    ("Frequency", "Hz", 0.25, 1.20),
    ("Stride", "", -0.55, 0.55),
    ("Stride bias", "", -0.20, 0.20),
    ("Leg swing", "", -0.55, 0.55),
    ("Foot level", "", -1.50, 1.50),
    ("Foot clearance", "", -0.50, 0.50),
    ("Stance duty", "", 0.35, 0.90),
    ("Knee bias", "", -0.35, 0.35),
    ("Lift bias", "", -0.35, 0.35),
    ("Turn", "", -1.00, 1.00),
    ("Segment phase", "rad", -PI_F, PI_F),
    ("Left/right phase", "rad", 0.45 * PI_F, 1.35 * PI_F),
    ("Body wave", "", 0.00, 0.38),
    ("Body phase", "rad", -PI_F, PI_F),
    ("Body turn", "", -0.35, 0.35),
    ("Knee phase", "rad", -PI_F, PI_F),
    ("Body rate", "x", 0.25, 4.00),
]
GAIT_PARAM_DESCRIPTIONS_V2 = [
    "歩行周期の速さ。全関節のCPG位相が進む速度を変える。",
    "yawの前後振幅。水平面内での歩幅を決める。負にするとlift/kneeに対して前後が反転する。",
    "yawの固定オフセット。足を置く前後中心位置をずらす。",
    "liftの前後振幅。yawと同位相で脚全体を前後に振る量。負でlift/kneeがまとめて反転する。",
    "膝とliftの連動係数。1.0でliftの回転を膝が打ち消し、足裏が地面と平行に保たれる。",
    "遊脚中だけliftに加算される持ち上げ量。接地を避けるクリアランス。持ち上げ向きが逆なら負にする。",
    "1周期に占める接地期の割合。接地と遊脚の速度比を決める。0.9で遊脚が9倍速い。",
    "膝の固定オフセット。立脚時を含む基本の膝曲げ量を変える。",
    "liftの固定オフセット。基本姿勢の高さ・接地圧をずらす。",
    "左右のStride差。正で左脚の歩幅が伸び、右へ旋回する。",
    "前後セグメント間の脚位相差。π付近では隣接脚群が交互になる。",
    "左脚群と右脚群の位相差。π付近では左右が交互になる。",
    "胴体関節ID 1〜3の周期的な曲げ振幅。",
    "胴体関節間の位相差。胴体を伝わる波の向きと形を変える。",
    "胴体関節ID 1〜3を同じ向きに曲げる固定量。旋回に使う。",
    "膝がliftから遅れる位相。足裏が地面と平行にならないときの追従タイミング補正。",
    "胴体波の周波数倍率。1.0で脚と同じ周期、2.0で脚1周期あたり胴体2周期。",
]
SAFE_GAIT_PARAMS_V2 = [
    -0.789474,
    0.181818,
    0.000000,
    0.181818,
    0.666667,
    0.200000,
    -0.272727,
    0.000000,
    0.000000,
    0.000000,
    1.000000,
    0.222222,
    -0.736842,
    0.652535,
    0.000000,
    0.000000,
    -0.600000,
]
FORWARD_GAIT_PARAMS_V2 = [
    -0.052632,
    0.545455,
    0.000000,
    0.509091,
    0.666667,
    0.440000,
    -0.090909,
    0.000000,
    0.000000,
    0.000000,
    1.000000,
    0.222222,
    -0.210526,
    0.652535,
    0.000000,
    0.000000,
    -0.600000,
]
@dataclass(frozen=True)
class GaitModel:
    name: str
    specs: list
    descriptions: list
    safe_params: list
    forward_params: list
    config_key: str

    @property
    def param_count(self) -> int:
        return len(self.specs)


GAIT_MODELS = {
    "v1": GaitModel(
        name="v1",
        specs=GAIT_PARAM_SPECS,
        descriptions=GAIT_PARAM_DESCRIPTIONS,
        safe_params=SAFE_GAIT_PARAMS,
        forward_params=FORWARD_GAIT_PARAMS,
        config_key="gait_params",
    ),
    "v2": GaitModel(
        name="v2",
        specs=GAIT_PARAM_SPECS_V2,
        descriptions=GAIT_PARAM_DESCRIPTIONS_V2,
        safe_params=SAFE_GAIT_PARAMS_V2,
        forward_params=FORWARD_GAIT_PARAMS_V2,
        config_key="gait_params_v2",
    ),
}
DEFAULT_GAIT_MODEL = "v1"


@dataclass(frozen=True)
class LegMap:
    segment_index: int
    side_index: int
    side_sign: float
    yaw_index: int
    lift_index: int
    knee_index: int


LEGS = [
    LegMap(0, 0, 1.0, 22, 23, 24),
    LegMap(0, 1, -1.0, 25, 26, 27),
    LegMap(1, 0, 1.0, 15, 16, 17),
    LegMap(1, 1, -1.0, 18, 19, 20),
    LegMap(2, 0, 1.0, 9, 10, 11),
    LegMap(2, 1, -1.0, 12, 13, 14),
    LegMap(3, 0, 1.0, 3, 4, 5),
    LegMap(3, 1, -1.0, 6, 7, 8),
]


@dataclass
class RobotConfig:
    ids: list[int]
    zero_ticks: list[int]
    directions: list[int]
    joint_lower: list[float]
    joint_upper: list[float]
    gait_params: list[float]
    gait_params_v2: list[float]
    enabled_indices: list[int]
    reverse_legs: bool
    sweep_phase_offset_rad: float

    def params_for(self, gait_model: str) -> list[float]:
        """Return the live parameter list for one gait model; mutations stick."""
        return self.gait_params_v2 if gait_model == "v2" else self.gait_params


MOTION_SEQUENCE_FORMAT = "hanachan-motion-sequence"
MOTION_SEQUENCE_VERSION = 1
GAIT_PRESET_FORMAT = "hanachan-gait-presets"
GAIT_PRESET_VERSION = 1
LEG_MOTION_FORMAT = "hanachan-leg-motion-designs"
LEG_MOTION_VERSION = 1


@dataclass
class MotionFrame:
    name: str
    duration_seconds: float
    positions: dict[int, int]


@dataclass
class MotionSequence:
    servo_ids: list[int]
    frames: list[MotionFrame]


@dataclass
class GaitPreset:
    name: str
    gait_params: list[float]
    sweep_phase_offset_rad: float = 0.0
    reverse_legs: bool = False
    # Untagged presets predate the v2 model, so they can only be v1 values.
    gait_model: str = "v1"


@dataclass
class LegMotionKeyframe:
    phase: float
    yaw: float
    lift: float
    knee: float
    duration_seconds: float = 0.25


@dataclass
class LegMotionDesign:
    name: str
    frequency_hz: float
    keyframes: list[LegMotionKeyframe]
    phase_offsets: list[float]


def describe_ports() -> dict[str, str]:
    """Map serial device name to a human readable description when available."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return {}
    return {port.device: (port.description or "").strip() for port in list_ports.comports()}


def candidate_ports() -> list[str]:
    if sys.platform.startswith("win"):
        # COM ports are not filesystem entries, so globbing never finds them.
        return sorted(describe_ports())

    patterns = (
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/tty.usbmodem*",
        "/dev/tty.usbserial*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    )
    ports: list[str] = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


def parse_ids(value: str) -> list[int]:
    ids: set[int] = set()
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = [int(item) for item in part.split("-", maxsplit=1)]
            if start > end:
                raise argparse.ArgumentTypeError(f"invalid id range: {part}")
            ids.update(range(start, end + 1))
        else:
            ids.add(int(part))

    if not ids:
        raise argparse.ArgumentTypeError("at least one id is required")
    for dxl_id in ids:
        if dxl_id < 0 or dxl_id > 252:
            raise argparse.ArgumentTypeError(f"DYNAMIXEL id out of range: {dxl_id}")
    return sorted(ids)


def parse_int_list(value: str) -> list[int]:
    numbers: list[int] = []
    for part in value.split(","):
        part = part.strip()
        if part:
            numbers.append(int(part))
    if not numbers:
        raise argparse.ArgumentTypeError("at least one value is required")
    return numbers


def parse_robot_ids(value: str) -> list[int]:
    ids = parse_int_list(value)
    for dxl_id in ids:
        if dxl_id < 0 or dxl_id > 252:
            raise argparse.ArgumentTypeError(f"DYNAMIXEL id out of range: {dxl_id}")
    return ids


def parse_float_list(value: str) -> list[float]:
    numbers: list[float] = []
    for part in value.split(","):
        part = part.strip()
        if part:
            numbers.append(float(part))
    if not numbers:
        raise argparse.ArgumentTypeError("at least one value is required")
    return numbers


def parse_signed_int_list(value: str) -> list[int]:
    numbers = parse_int_list(value)
    for number in numbers:
        if number not in (-1, 1):
            raise argparse.ArgumentTypeError("directions must be 1 or -1")
    return numbers


def parse_int_overrides(value: str) -> dict[int, int]:
    overrides: dict[int, int] = {}
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" not in part:
            raise argparse.ArgumentTypeError("overrides must use id:value format")
        key, raw_value = part.split(":", maxsplit=1)
        overrides[int(key)] = int(raw_value)
    return overrides


def parse_float_params(value: str) -> list[float]:
    # The gait model is not known at parse time, so accept any model's length
    # here; validate_robot_config rejects a count that does not match the model.
    values = parse_float_list(value)
    allowed = sorted({model.param_count for model in GAIT_MODELS.values()})
    if len(values) not in allowed:
        expected = " or ".join(str(count) for count in allowed)
        raise argparse.ArgumentTypeError(f"gait params must contain {expected} values")
    return values


def require_length(name: str, values: list, expected: int) -> None:
    if len(values) != expected:
        raise ValueError(f"{name} must contain {expected} values, got {len(values)}")


def enabled_indices_for_layout(layout: str) -> list[int]:
    if layout == "full":
        return [index for index in FULL_ENABLED_INDICES if index != TAIL_BODY_INDEX]
    if layout == "three-segment":
        return list(THREE_SEGMENT_ENABLED_INDICES)
    raise ValueError(f"unknown robot layout: {layout}")


def default_robot_config(layout: str = "three-segment") -> RobotConfig:
    return RobotConfig(
        ids=list(DEFAULT_ROBOT_IDS),
        zero_ticks=list(DEFAULT_ZERO_TICKS),
        directions=list(DEFAULT_DIRECTIONS),
        joint_lower=list(JOINT_LOWER),
        joint_upper=list(JOINT_UPPER),
        gait_params=list(DEFAULT_GAIT_PARAMS),
        gait_params_v2=list(SAFE_GAIT_PARAMS_V2),
        enabled_indices=enabled_indices_for_layout(layout),
        reverse_legs=False,
        sweep_phase_offset_rad=0.0,
    )


def load_robot_config(path: str | None, layout: str = "three-segment") -> RobotConfig:
    config = default_robot_config(layout)
    if path is None:
        return config
    if path == DEFAULT_CONFIG_PATH and not Path(path).exists():
        # The default config is optional; an explicit --config path is not.
        return config

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if "ids" in data:
        config.ids = [int(value) for value in data["ids"]]
    if "zero_ticks" in data:
        config.zero_ticks = [int(value) for value in data["zero_ticks"]]
    if "directions" in data:
        config.directions = [int(value) for value in data["directions"]]
    if "joint_lower" in data:
        config.joint_lower = [float(value) for value in data["joint_lower"]]
    if "joint_upper" in data:
        config.joint_upper = [float(value) for value in data["joint_upper"]]
    if "gait_params" in data:
        config.gait_params = fit_gait_params(
            [float(value) for value in data["gait_params"]], "v1"
        )
    if "gait_params_v2" in data:
        config.gait_params_v2 = fit_gait_params(
            [float(value) for value in data["gait_params_v2"]], "v2"
        )
    if "enabled_indices" in data:
        config.enabled_indices = [int(value) for value in data["enabled_indices"]]
    if "reverse_legs" in data:
        config.reverse_legs = data["reverse_legs"]
    if "sweep_phase_offset_rad" in data:
        config.sweep_phase_offset_rad = float(data["sweep_phase_offset_rad"])

    validate_robot_config(config)
    return config


def validate_robot_config(config: RobotConfig) -> None:
    require_length("ids", config.ids, SERVO_COUNT)
    require_length("zero_ticks", config.zero_ticks, SERVO_COUNT)
    require_length("directions", config.directions, SERVO_COUNT)
    require_length("joint_lower", config.joint_lower, SERVO_COUNT)
    require_length("joint_upper", config.joint_upper, SERVO_COUNT)
    require_length("gait_params", config.gait_params, GAIT_MODELS["v1"].param_count)
    require_length("gait_params_v2", config.gait_params_v2, GAIT_MODELS["v2"].param_count)
    if not isinstance(config.reverse_legs, bool):
        raise ValueError("reverse_legs must be true or false")
    if (
        not math.isfinite(config.sweep_phase_offset_rad)
        or config.sweep_phase_offset_rad < -PI_F
        or config.sweep_phase_offset_rad > PI_F
    ):
        raise ValueError("sweep_phase_offset_rad must be between -pi and pi")
    for index in config.enabled_indices:
        if index < 0 or index >= SERVO_COUNT:
            raise ValueError(f"enabled index out of range: {index}")
    for direction in config.directions:
        if direction not in (-1, 1):
            raise ValueError("directions must contain only 1 or -1")


def validate_motion_sequence(sequence: MotionSequence, expected_ids: Iterable[int] | None = None) -> None:
    if not sequence.servo_ids:
        raise ValueError("motion sequence must contain at least one servo id")
    if len(sequence.servo_ids) != len(set(sequence.servo_ids)):
        raise ValueError("motion sequence contains duplicate servo ids")
    for dxl_id in sequence.servo_ids:
        if dxl_id < 0 or dxl_id > 252:
            raise ValueError(f"motion sequence servo id out of range: {dxl_id}")

    expected_set = set(expected_ids) if expected_ids is not None else None
    if expected_set is not None and set(sequence.servo_ids) != expected_set:
        missing = sorted(expected_set - set(sequence.servo_ids))
        extra = sorted(set(sequence.servo_ids) - expected_set)
        raise ValueError(f"motion sequence servo ids do not match config; missing={missing}, extra={extra}")

    servo_id_set = set(sequence.servo_ids)
    for frame_index, frame in enumerate(sequence.frames, start=1):
        if not frame.name.strip():
            raise ValueError(f"frame {frame_index} has an empty name")
        if not math.isfinite(frame.duration_seconds) or frame.duration_seconds <= 0:
            raise ValueError(f"frame {frame_index} duration must be greater than 0")
        if set(frame.positions) != servo_id_set:
            raise ValueError(f"frame {frame_index} positions do not match sequence servo ids")
        for dxl_id, position in frame.positions.items():
            if isinstance(position, bool) or not isinstance(position, int):
                raise ValueError(f"frame {frame_index} position for id={dxl_id} must be an integer")
            if position < DEFAULT_POSITION_MIN or position > DEFAULT_POSITION_MAX:
                raise ValueError(
                    f"frame {frame_index} position for id={dxl_id} is outside "
                    f"{DEFAULT_POSITION_MIN}..{DEFAULT_POSITION_MAX}: {position}"
                )


def motion_sequence_to_data(sequence: MotionSequence) -> dict:
    validate_motion_sequence(sequence)
    return {
        "format": MOTION_SEQUENCE_FORMAT,
        "version": MOTION_SEQUENCE_VERSION,
        "servo_ids": sequence.servo_ids,
        "frames": [
            {
                "name": frame.name,
                "duration_seconds": frame.duration_seconds,
                "positions": [frame.positions[dxl_id] for dxl_id in sequence.servo_ids],
            }
            for frame in sequence.frames
        ],
    }


def save_motion_sequence(path: str, sequence: MotionSequence) -> None:
    data = motion_sequence_to_data(sequence)
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")


def load_motion_sequence(path: str) -> MotionSequence:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError("motion sequence root must be an object")
    if data.get("format") != MOTION_SEQUENCE_FORMAT:
        raise ValueError(f"unsupported motion sequence format: {data.get('format')!r}")
    if data.get("version") != MOTION_SEQUENCE_VERSION:
        raise ValueError(f"unsupported motion sequence version: {data.get('version')!r}")

    raw_servo_ids = data.get("servo_ids")
    raw_frames = data.get("frames")
    if not isinstance(raw_servo_ids, list) or not all(
        isinstance(value, int) and not isinstance(value, bool) for value in raw_servo_ids
    ):
        raise ValueError("motion sequence servo_ids must be an integer array")
    if not isinstance(raw_frames, list):
        raise ValueError("motion sequence frames must be an array")

    servo_ids = list(raw_servo_ids)
    frames: list[MotionFrame] = []
    for frame_index, raw_frame in enumerate(raw_frames, start=1):
        if not isinstance(raw_frame, dict):
            raise ValueError(f"frame {frame_index} must be an object")
        name = raw_frame.get("name")
        duration = raw_frame.get("duration_seconds")
        raw_positions = raw_frame.get("positions")
        if not isinstance(name, str):
            raise ValueError(f"frame {frame_index} name must be a string")
        if isinstance(duration, bool) or not isinstance(duration, (int, float)):
            raise ValueError(f"frame {frame_index} duration_seconds must be a number")
        if not isinstance(raw_positions, list) or len(raw_positions) != len(servo_ids):
            raise ValueError(
                f"frame {frame_index} positions must contain {len(servo_ids)} values"
            )
        if not all(isinstance(value, int) and not isinstance(value, bool) for value in raw_positions):
            raise ValueError(f"frame {frame_index} positions must be integers")
        frames.append(
            MotionFrame(
                name=name,
                duration_seconds=float(duration),
                positions=dict(zip(servo_ids, raw_positions)),
            )
        )

    sequence = MotionSequence(servo_ids=servo_ids, frames=frames)
    validate_motion_sequence(sequence)
    return sequence


def interpolate_positions(
    start: dict[int, int],
    target: dict[int, int],
    progress: float,
) -> dict[int, int]:
    if set(start) != set(target):
        raise ValueError("start and target positions must contain the same servo ids")
    clipped_progress = max(0.0, min(1.0, progress))
    return {
        dxl_id: rounded(start[dxl_id] + (target[dxl_id] - start[dxl_id]) * clipped_progress)
        for dxl_id in start
    }


def validate_gait_preset(preset: GaitPreset) -> None:
    if not preset.name.strip():
        raise ValueError("gait preset name must not be empty")
    if preset.gait_model not in GAIT_MODELS:
        raise ValueError(f"unknown gait preset model: {preset.gait_model}")
    require_length(
        "gait preset params",
        preset.gait_params,
        GAIT_MODELS[preset.gait_model].param_count,
    )
    for value in preset.gait_params:
        if not math.isfinite(value) or value < -1.0 or value > 1.0:
            raise ValueError("gait preset params must be finite values between -1 and 1")
    if (
        not math.isfinite(preset.sweep_phase_offset_rad)
        or preset.sweep_phase_offset_rad < -PI_F
        or preset.sweep_phase_offset_rad > PI_F
    ):
        raise ValueError("gait preset sweep phase must be between -pi and pi")
    if not isinstance(preset.reverse_legs, bool):
        raise ValueError("gait preset reverse_legs must be true or false")


def load_gait_presets(path: str) -> list[GaitPreset]:
    preset_path = Path(path)
    if not preset_path.exists():
        return []
    with preset_path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict) or data.get("format") != GAIT_PRESET_FORMAT:
        raise ValueError(f"unsupported gait preset file: {path}")
    if data.get("version") != GAIT_PRESET_VERSION:
        raise ValueError(f"unsupported gait preset version: {data.get('version')!r}")
    raw_presets = data.get("presets")
    if not isinstance(raw_presets, list):
        raise ValueError("gait preset file presets must be an array")

    presets: list[GaitPreset] = []
    seen_names: set[tuple[str, str]] = set()
    for raw in raw_presets:
        if not isinstance(raw, dict):
            raise ValueError("each gait preset must be an object")
        name = raw.get("name")
        params = raw.get("gait_params")
        if not isinstance(name, str) or not isinstance(params, list):
            raise ValueError("each gait preset requires name and gait_params")
        gait_model = str(raw.get("gait_model", "v1"))
        if gait_model not in GAIT_MODELS:
            raise ValueError(f"unknown gait preset model: {gait_model}")
        preset = GaitPreset(
            name=name.strip(),
            gait_params=fit_gait_params([float(value) for value in params], gait_model),
            sweep_phase_offset_rad=float(raw.get("sweep_phase_offset_rad", 0.0)),
            reverse_legs=raw.get("reverse_legs", False),
            gait_model=gait_model,
        )
        validate_gait_preset(preset)
        key = (preset.name, preset.gait_model)
        if key in seen_names:
            raise ValueError(
                f"duplicate gait preset name for {preset.gait_model}: {preset.name}"
            )
        seen_names.add(key)
        presets.append(preset)
    return presets


def save_gait_presets(path: str, presets: list[GaitPreset]) -> None:
    # A name only has to be unique within its gait model, since the parameters
    # mean different things across models.
    seen_names: set[tuple[str, str]] = set()
    for preset in presets:
        validate_gait_preset(preset)
        key = (preset.name, preset.gait_model)
        if key in seen_names:
            raise ValueError(
                f"duplicate gait preset name for {preset.gait_model}: {preset.name}"
            )
        seen_names.add(key)
    data = {
        "format": GAIT_PRESET_FORMAT,
        "version": GAIT_PRESET_VERSION,
        "presets": [
            {
                "name": preset.name,
                "gait_params": preset.gait_params,
                "sweep_phase_offset_rad": preset.sweep_phase_offset_rad,
                "reverse_legs": preset.reverse_legs,
                "gait_model": preset.gait_model,
            }
            for preset in presets
        ],
    }
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(f"{output_path.name}.tmp")
    with temporary_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")
    temporary_path.replace(output_path)


def default_leg_motion_design(name: str = "New leg motion") -> LegMotionDesign:
    return LegMotionDesign(
        name=name,
        frequency_hz=0.25,
        keyframes=[
            LegMotionKeyframe(0.00, 0.00, 0.00, 0.00, 2.0),
            LegMotionKeyframe(0.50, 0.00, 0.00, 0.00, 2.0),
        ],
        phase_offsets=[0.00, 0.50, 0.50, 0.00, 0.00, 0.50, 0.50, 0.00],
    )


def update_leg_motion_timing(design: LegMotionDesign) -> None:
    total_seconds = sum(keyframe.duration_seconds for keyframe in design.keyframes)
    if total_seconds <= 0.0:
        raise ValueError("leg motion cycle duration must be greater than zero")
    elapsed = 0.0
    for keyframe in design.keyframes:
        keyframe.phase = elapsed / total_seconds
        elapsed += keyframe.duration_seconds
    design.frequency_hz = 1.0 / total_seconds


def batch_edit_leg_motion_keyframes(
    design: LegMotionDesign,
    start_index: int,
    end_index: int,
    joint: str,
    operation: str,
    value_a: float,
    value_b: float | None = None,
) -> None:
    if joint not in {"yaw", "lift", "knee", "duration_seconds"}:
        raise ValueError(f"unknown leg joint column: {joint}")
    if start_index < 0 or end_index >= len(design.keyframes) or start_index > end_index:
        raise ValueError("invalid leg frame range")
    if operation not in {"set", "add", "multiply", "ramp"}:
        raise ValueError(f"unknown batch edit operation: {operation}")
    if not math.isfinite(value_a) or (
        value_b is not None and not math.isfinite(value_b)
    ):
        raise ValueError("batch edit values must be finite")
    if operation == "ramp" and value_b is None:
        raise ValueError("linear ramp requires both start and end values")

    frame_count = end_index - start_index + 1
    edited_values: list[float] = []
    for offset, index in enumerate(range(start_index, end_index + 1)):
        current = getattr(design.keyframes[index], joint)
        if operation == "set":
            edited = value_a
        elif operation == "add":
            edited = current + value_a
        elif operation == "multiply":
            edited = current * value_a
        else:
            progress = 0.0 if frame_count == 1 else offset / (frame_count - 1)
            edited = value_a + (value_b - value_a) * progress
        if joint == "duration_seconds":
            edited_values.append(max(0.02, min(20.0, edited)))
        else:
            edited_values.append(clip_unit(edited))

    for index, edited in zip(
        range(start_index, end_index + 1),
        edited_values,
    ):
        setattr(design.keyframes[index], joint, edited)
    if joint == "duration_seconds":
        update_leg_motion_timing(design)


def delete_leg_motion_keyframe_range(
    design: LegMotionDesign,
    start_index: int,
    end_index: int,
) -> int:
    if start_index < 0 or end_index >= len(design.keyframes) or start_index > end_index:
        raise ValueError("invalid leg frame range")
    delete_count = end_index - start_index + 1
    if len(design.keyframes) - delete_count < 2:
        raise ValueError("at least two leg frames must remain")
    del design.keyframes[start_index : end_index + 1]
    update_leg_motion_timing(design)
    return delete_count


def validate_leg_motion_design(design: LegMotionDesign) -> None:
    if not design.name.strip():
        raise ValueError("leg motion design name must not be empty")
    if not math.isfinite(design.frequency_hz) or not 0.05 <= design.frequency_hz <= 2.0:
        raise ValueError("leg motion frequency must be between 0.05 and 2.0 Hz")
    require_length("leg motion phase offsets", design.phase_offsets, LEG_COUNT)
    for offset in design.phase_offsets:
        if not math.isfinite(offset) or offset < 0.0 or offset >= 1.0:
            raise ValueError("leg phase offsets must be between 0.0 and less than 1.0")
    if len(design.keyframes) < 2:
        raise ValueError("leg motion design requires at least two keyframes")
    phases: set[float] = set()
    for keyframe in design.keyframes:
        if not math.isfinite(keyframe.phase) or keyframe.phase < 0.0 or keyframe.phase >= 1.0:
            raise ValueError("leg keyframe phase must be between 0.0 and less than 1.0")
        rounded_phase = round(keyframe.phase, 6)
        if rounded_phase in phases:
            raise ValueError(f"duplicate leg keyframe phase: {keyframe.phase:.3f}")
        phases.add(rounded_phase)
        for value in (keyframe.yaw, keyframe.lift, keyframe.knee):
            if not math.isfinite(value) or value < -1.0 or value > 1.0:
                raise ValueError("leg keyframe joint values must be between -1 and 1")
        if (
            not math.isfinite(keyframe.duration_seconds)
            or keyframe.duration_seconds < 0.02
            or keyframe.duration_seconds > 20.0
        ):
            raise ValueError(
                "time to the next leg frame must be between 0.02 and 20 seconds"
            )


def evaluate_leg_motion(design: LegMotionDesign, phase: float) -> tuple[float, float, float]:
    normalized_phase = phase % 1.0
    keyframes = sorted(design.keyframes, key=lambda item: item.phase)
    extended = [
        LegMotionKeyframe(
            keyframes[-1].phase - 1.0,
            keyframes[-1].yaw,
            keyframes[-1].lift,
            keyframes[-1].knee,
        ),
        *keyframes,
        LegMotionKeyframe(
            keyframes[0].phase + 1.0,
            keyframes[0].yaw,
            keyframes[0].lift,
            keyframes[0].knee,
        ),
    ]
    start = extended[0]
    end = extended[1]
    for candidate_start, candidate_end in zip(extended, extended[1:]):
        if candidate_start.phase <= normalized_phase <= candidate_end.phase:
            start, end = candidate_start, candidate_end
            break
    span = end.phase - start.phase
    progress = 0.0 if span <= 0.0 else (normalized_phase - start.phase) / span
    eased = 0.5 - 0.5 * math.cos(PI_F * max(0.0, min(1.0, progress)))
    return tuple(
        clip_unit(start_value + (end_value - start_value) * eased)
        for start_value, end_value in (
            (start.yaw, end.yaw),
            (start.lift, end.lift),
            (start.knee, end.knee),
        )
    )


def leg_motion_zero_columns(
    design: LegMotionDesign,
    tolerance: float = 1e-6,
) -> set[str]:
    return {
        joint
        for joint in ("yaw", "lift", "knee")
        if all(
            abs(getattr(keyframe, joint)) <= tolerance
            for keyframe in design.keyframes
        )
    }


def load_leg_motion_designs(path: str) -> list[LegMotionDesign]:
    design_path = Path(path)
    if not design_path.exists():
        return []
    with design_path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict) or data.get("format") != LEG_MOTION_FORMAT:
        raise ValueError(f"unsupported leg motion design file: {path}")
    if data.get("version") != LEG_MOTION_VERSION:
        raise ValueError(f"unsupported leg motion design version: {data.get('version')!r}")
    raw_designs = data.get("designs")
    if not isinstance(raw_designs, list):
        raise ValueError("leg motion design file designs must be an array")

    designs: list[LegMotionDesign] = []
    names: set[str] = set()
    for raw_design in raw_designs:
        if not isinstance(raw_design, dict):
            raise ValueError("each leg motion design must be an object")
        raw_keyframes = raw_design.get("keyframes")
        raw_offsets = raw_design.get("phase_offsets")
        if (
            not isinstance(raw_design.get("name"), str)
            or not isinstance(raw_keyframes, list)
            or not isinstance(raw_offsets, list)
        ):
            raise ValueError("leg motion design requires name, keyframes, and phase_offsets")
        if not all(isinstance(raw, dict) for raw in raw_keyframes):
            raise ValueError("each leg keyframe must be an object")
        if len(raw_keyframes) < 2:
            raise ValueError("leg motion design requires at least two keyframes")
        raw_phases = [
            float(raw["phase"])
            for raw in raw_keyframes
        ]
        frequency_hz = float(raw_design.get("frequency_hz", 0.25))
        keyframes = []
        for index, raw in enumerate(raw_keyframes):
            next_phase = raw_phases[(index + 1) % len(raw_phases)]
            phase_span = (next_phase - raw_phases[index]) % 1.0
            if phase_span <= 0.0:
                phase_span = 1.0 / max(1, len(raw_phases))
            keyframes.append(
                LegMotionKeyframe(
                    phase=raw_phases[index],
                    yaw=float(raw["yaw"]),
                    lift=float(raw["lift"]),
                    knee=float(raw["knee"]),
                    duration_seconds=float(
                        raw.get(
                            "duration_seconds",
                            phase_span / frequency_hz,
                        )
                    ),
                )
            )
        design = LegMotionDesign(
            name=raw_design["name"].strip(),
            frequency_hz=frequency_hz,
            keyframes=keyframes,
            phase_offsets=[float(value) for value in raw_offsets],
        )
        update_leg_motion_timing(design)
        validate_leg_motion_design(design)
        if design.name in names:
            raise ValueError(f"duplicate leg motion design name: {design.name}")
        names.add(design.name)
        designs.append(design)
    return designs


def save_leg_motion_designs(path: str, designs: list[LegMotionDesign]) -> None:
    names: set[str] = set()
    for design in designs:
        update_leg_motion_timing(design)
        validate_leg_motion_design(design)
        if design.name in names:
            raise ValueError(f"duplicate leg motion design name: {design.name}")
        names.add(design.name)
    data = {
        "format": LEG_MOTION_FORMAT,
        "version": LEG_MOTION_VERSION,
        "designs": [
            {
                "name": design.name,
                "frequency_hz": design.frequency_hz,
                "keyframes": [
                    {
                        "phase": keyframe.phase,
                        "yaw": keyframe.yaw,
                        "lift": keyframe.lift,
                        "knee": keyframe.knee,
                        "duration_seconds": keyframe.duration_seconds,
                    }
                    for keyframe in design.keyframes
                ],
                "phase_offsets": design.phase_offsets,
            }
            for design in designs
        ],
    }
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(f"{output_path.name}.tmp")
    with temporary_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")
    temporary_path.replace(output_path)


def apply_robot_overrides(
    config: RobotConfig,
    ids: list[int] | None = None,
    zero_ticks: list[int] | None = None,
    directions: list[int] | None = None,
    gait_params: list[float] | None = None,
    zero_overrides: dict[int, int] | None = None,
    direction_overrides: dict[int, int] | None = None,
    gait_model: str = DEFAULT_GAIT_MODEL,
) -> RobotConfig:
    if ids is not None:
        config.ids = ids
    if zero_ticks is not None:
        config.zero_ticks = zero_ticks
    if directions is not None:
        config.directions = directions
    if gait_params is not None:
        config.params_for(gait_model)[:] = gait_params
    if zero_overrides:
        id_to_index = {dxl_id: index for index, dxl_id in enumerate(config.ids)}
        for dxl_id, zero_tick in zero_overrides.items():
            if dxl_id not in id_to_index:
                raise ValueError(f"zero override id={dxl_id} is not in robot ids")
            config.zero_ticks[id_to_index[dxl_id]] = zero_tick
    if direction_overrides:
        id_to_index = {dxl_id: index for index, dxl_id in enumerate(config.ids)}
        for dxl_id, direction in direction_overrides.items():
            if dxl_id not in id_to_index:
                raise ValueError(f"direction override id={dxl_id} is not in robot ids")
            config.directions[id_to_index[dxl_id]] = direction
    validate_robot_config(config)
    return config


def fit_gait_params(values: list[float], gait_model: str) -> list[float]:
    """Pad a saved parameter list that predates parameters added to the model.

    New parameters are only ever appended, and their defaults are neutral, so an
    older file keeps the gait it was tuned for. A longer list is a real mismatch.
    """
    model = GAIT_MODELS[gait_model]
    if len(values) > model.param_count:
        raise ValueError(
            f"{gait_model} gait params must contain at most {model.param_count} "
            f"values, got {len(values)}"
        )
    return list(values) + list(model.safe_params[len(values):])


def gait_stroke(cycle: float, stance_duty: float) -> tuple[float, float]:
    """Fore-aft stroke and foot rise at one point of the v2 cycle.

    The stroke runs +1 (front) to -1 (rear) at constant speed while the foot is
    on the ground, then eases back to the front. Foot rise is zero through the
    whole stance so the foot is only lifted on the return.
    """
    cycle %= 1.0
    if cycle < stance_duty:
        progress = cycle / max(1e-6, stance_duty)
        return 1.0 - 2.0 * progress, 0.0
    progress = (cycle - stance_duty) / max(1e-6, 1.0 - stance_duty)
    return -math.cos(PI_F * progress), math.sin(PI_F * progress)


def clip_unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


def map_range(value: float, low: float, high: float) -> float:
    return low + 0.5 * (value + 1.0) * (high - low)


def rounded(value: float) -> int:
    return int(math.floor(value + 0.5)) if value >= 0 else int(math.ceil(value - 0.5))


def resolve_control_period(baudrate: int, requested_hz: float | None) -> float:
    if requested_hz is not None:
        if not math.isfinite(requested_hz) or requested_hz <= 0:
            raise ValueError("control frequency must be greater than 0")
        return 1.0 / requested_hz
    if baudrate <= 115_200:
        return 1.0 / DEFAULT_WIRELESS_CONTROL_HZ
    return 1.0 / DEFAULT_WIRED_CONTROL_HZ


def require_sdk_success(packet: PacketHandler, comm_result: int, error: int, action: str) -> None:
    if comm_result != COMM_SUCCESS:
        raise RuntimeError(f"{action}: {packet.getTxRxResult(comm_result)}")
    if error:
        raise RuntimeError(f"{action}: {packet.getRxPacketError(error)}")


class RadioTolerantPortHandler(PortHandler):
    def __init__(self, port_name: str, minimum_timeout_ms: float = 0.0) -> None:
        super().__init__(port_name)
        self.minimum_timeout_ms = minimum_timeout_ms

    def setPacketTimeout(self, packet_length: int) -> None:
        super().setPacketTimeout(packet_length)
        self.packet_timeout = max(self.packet_timeout, self.minimum_timeout_ms)


class DynamixelBus:
    def __init__(self, device: str, baudrate: int, protocol: float) -> None:
        wireless_link = baudrate <= 115_200
        minimum_timeout = WIRELESS_MINIMUM_PACKET_TIMEOUT_MS if wireless_link else 0.0
        self.port = RadioTolerantPortHandler(device, minimum_timeout)
        self.packet = PacketHandler(protocol)
        self.device = device
        self.baudrate = baudrate
        self.wireless_link = wireless_link
        self.retry_attempts = WIRELESS_RETRY_ATTEMPTS if wireless_link else 1

    def __enter__(self) -> DynamixelBus:
        if not self.port.openPort():
            raise RuntimeError(f"failed to open serial port: {self.device}")
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f"failed to set baudrate: {self.baudrate}")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.port.closePort()

    def execute_packet(self, action: str, operation, attempts: int | None = None):
        attempt_count = self.retry_attempts if attempts is None else attempts
        last_result = None
        for attempt in range(attempt_count):
            result = operation()
            last_result = result
            comm_result, error = result[-2], result[-1]
            if comm_result == COMM_SUCCESS and not error:
                return result
            if error:
                break
            if attempt + 1 < attempt_count:
                time.sleep(WIRELESS_RETRY_DELAY_SECONDS)

        if last_result is None:
            raise RuntimeError(f"{action}: operation was not attempted")
        require_sdk_success(self.packet, last_result[-2], last_result[-1], action)
        raise AssertionError("unreachable")

    def ping(self, dxl_id: int) -> tuple[int, int] | None:
        for attempt in range(self.retry_attempts):
            model_number, comm_result, error = self.packet.ping(self.port, dxl_id)
            if comm_result == COMM_SUCCESS and not error:
                return dxl_id, model_number
            if error:
                return None
            if attempt + 1 < self.retry_attempts:
                time.sleep(WIRELESS_RETRY_DELAY_SECONDS)
        return None

    def scan(self, ids: Iterable[int]) -> list[tuple[int, int]]:
        found: list[tuple[int, int]] = []
        for dxl_id in ids:
            result = self.ping(dxl_id)
            if result is not None:
                found.append(result)
        return found

    def set_torque(self, dxl_id: int, enabled: bool) -> None:
        self.execute_packet(
            f"set torque id={dxl_id}",
            lambda: self.packet.write1ByteTxRx(
                self.port,
                dxl_id,
                TABLE.torque_enable,
                1 if enabled else 0,
            ),
        )

    def change_id(self, current_id: int, new_id: int) -> None:
        if new_id < 0 or new_id > 252:
            raise ValueError(f"new DYNAMIXEL id out of range: {new_id}")
        self.set_torque(current_id, False)
        comm_result, error = self.execute_packet(
            f"change id {current_id}->{new_id}",
            lambda: self.packet.write1ByteTxRx(
                self.port,
                current_id,
                TABLE.id,
                new_id,
            ),
            attempts=1,
        )

    def set_position_mode(self, dxl_id: int) -> None:
        self.execute_packet(
            f"set position mode id={dxl_id}",
            lambda: self.packet.write1ByteTxRx(
                self.port,
                dxl_id,
                TABLE.operating_mode,
                POSITION_CONTROL_MODE,
            ),
        )

    def set_led(self, dxl_id: int, enabled: bool) -> None:
        self.execute_packet(
            f"set led id={dxl_id}",
            lambda: self.packet.write1ByteTxRx(
                self.port,
                dxl_id,
                TABLE.led,
                1 if enabled else 0,
            ),
        )

    def read_position(self, dxl_id: int) -> int:
        position, _comm_result, _error = self.execute_packet(
            f"read position id={dxl_id}",
            lambda: self.packet.read4ByteTxRx(
                self.port,
                dxl_id,
                TABLE.present_position,
            ),
        )
        return position

    def read_moving(self, dxl_id: int) -> bool:
        moving, _comm_result, _error = self.execute_packet(
            f"read moving id={dxl_id}",
            lambda: self.packet.read1ByteTxRx(
                self.port,
                dxl_id,
                TABLE.moving,
            ),
        )
        return bool(moving)

    def move_to(self, dxl_id: int, position: int) -> None:
        self.execute_packet(
            f"move id={dxl_id}",
            lambda: self.packet.write4ByteTxRx(
                self.port,
                dxl_id,
                TABLE.goal_position,
                position,
            ),
        )

    def sync_move_to(self, targets: dict[int, int]) -> None:
        # One sync write beats per-servo unicast on a slow link: 21 joints fit in
        # a single ~119 byte packet instead of 21 packets totalling ~336 bytes.
        # Measured over a 57600 bps radio, that is 20 ms per update instead of
        # 122 ms, which is the difference between 8 Hz and 40 Hz control.
        group = GroupSyncWrite(self.port, self.packet, TABLE.goal_position, 4)
        for dxl_id, position in targets.items():
            raw = int(position)
            param = [
                raw & 0xFF,
                (raw >> 8) & 0xFF,
                (raw >> 16) & 0xFF,
                (raw >> 24) & 0xFF,
            ]
            if not group.addParam(dxl_id, param):
                raise RuntimeError(f"sync write addParam failed id={dxl_id}")
        comm_result = group.txPacket()
        group.clearParam()
        if comm_result != COMM_SUCCESS:
            raise RuntimeError(f"sync move: {self.packet.getTxRxResult(comm_result)}")


class WirelessCPGClient:
    def __init__(self, device: str, baudrate: int) -> None:
        self.device = device
        self.baudrate = baudrate
        self.serial = None

    def __enter__(self) -> WirelessCPGClient:
        import serial

        self.serial = serial.Serial(
            port=self.device,
            baudrate=self.baudrate,
            timeout=0.2,
            write_timeout=1.0,
        )
        self.serial.reset_input_buffer()
        response = self.command("HELLO", attempts=5, timeout=1.0)
        if "HANA_CPG_1" not in response:
            raise RuntimeError(f"unexpected OpenRB CPG response: {response}")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def command(self, command: str, attempts: int = 3, timeout: float = 1.0) -> str:
        if self.serial is None:
            raise RuntimeError("wireless CPG serial port is not open")
        if "\n" in command or "\r" in command:
            raise ValueError("wireless CPG command must be a single line")

        packet = f"@{command}\n".encode("ascii")
        last_error = "timeout"
        for _attempt in range(attempts):
            self.serial.reset_input_buffer()
            self.serial.write(packet)
            self.serial.flush()
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                raw_line = self.serial.readline()
                if not raw_line:
                    continue
                line = raw_line.decode("ascii", errors="replace").strip()
                if line.startswith("@OK"):
                    return line
                if line.startswith("@ERR"):
                    raise RuntimeError(f"OpenRB CPG error: {line}")
                last_error = f"unexpected response: {line!r}"
            time.sleep(WIRELESS_RETRY_DELAY_SECONDS)
        raise RuntimeError(f"OpenRB CPG command {command.split(',', 1)[0]} failed: {last_error}")


class OnboardHanachanCPGController:
    def __init__(self, client: WirelessCPGClient, config: RobotConfig) -> None:
        self.client = client
        self.config = config
        self.motion_enabled = False
        self.last_params: tuple[float, ...] | None = None
        self.last_reverse_legs: bool | None = None
        self.last_sweep_phase_offset_rad: float | None = None
        self.gait_test_mode = "full"
        self.last_gait_test_mode: str | None = None

    def send_params(self, force: bool = False) -> None:
        params = tuple(clip_unit(value) for value in self.config.gait_params)
        if force or params != self.last_params:
            payload = "PARAMS," + ",".join(f"{value:.6f}" for value in params)
            self.client.command(payload)
            self.last_params = params

        reverse_legs = self.config.reverse_legs
        if force or reverse_legs != self.last_reverse_legs:
            self.client.command(f"REVERSE,{1 if reverse_legs else 0}")
            self.last_reverse_legs = reverse_legs

        sweep_phase = self.config.sweep_phase_offset_rad
        if force or sweep_phase != self.last_sweep_phase_offset_rad:
            self.client.command(f"SWEEP_PHASE,{sweep_phase:.6f}")
            self.last_sweep_phase_offset_rad = sweep_phase

        if force or self.gait_test_mode != self.last_gait_test_mode:
            self.client.command(f"MODE,{self.gait_test_mode.upper()}")
            self.last_gait_test_mode = self.gait_test_mode

    def set_gait_test_mode(self, mode: str) -> None:
        if mode not in GAIT_TEST_MODES:
            raise ValueError(f"unknown gait test mode: {mode}")
        self.gait_test_mode = mode
        self.send_params()

    def initialize_servos(self, set_position_mode: bool = True) -> None:
        del set_position_mode
        self.client.command("INIT", attempts=3, timeout=8.0)
        self.send_params(force=True)

    def start_motion(self) -> None:
        self.send_params(force=True)
        self.client.command("START")
        self.motion_enabled = True

    def stop_motion_neutral(self) -> None:
        self.client.command("STOP")
        self.motion_enabled = False

    def set_torque(self, enabled: bool) -> None:
        self.client.command(f"TORQUE,{1 if enabled else 0}", timeout=3.0)
        if not enabled:
            self.motion_enabled = False

    def write_all_2048(self) -> None:
        self.client.command("ALL2048")
        self.motion_enabled = False

    def start_initial_position(self, duration_seconds: float) -> None:
        duration_ms = max(100, min(10_000, rounded(duration_seconds * 1000.0)))
        self.client.command(f"INITIAL,{duration_ms}", timeout=4.0)
        self.motion_enabled = False

    def set_body_tick(self, index: int, tick: int) -> None:
        if index not in BODY_JOINT_INDEX:
            raise ValueError(f"index {index} is not a body joint")
        if index not in self.config.enabled_indices:
            raise ValueError(f"{JOINT_LABELS[index]} is disabled")
        self.client.command(f"BODY,{index},{max(0, min(4095, int(tick)))}")

    def clear_body_tick_overrides(self) -> None:
        self.client.command("BODY_CLEAR")

    def update(self, dt: float, expected_period: float = CONTROL_PERIOD_SECONDS) -> None:
        del dt, expected_period
        self.send_params()


class HanachanCPGController:
    def __init__(
        self,
        bus: DynamixelBus,
        config: RobotConfig,
        gait_model: str = DEFAULT_GAIT_MODEL,
    ) -> None:
        self.bus = bus
        self.config = config
        self.gait_model = gait_model
        self.gait_params = config.params_for(gait_model)
        self.filtered_params = [0.0] * GAIT_MODELS[gait_model].param_count
        self.servo_action = [0.0] * SERVO_COUNT
        self.phase_rad = 0.0
        # Separate accumulator: scaling phase_rad would jump at every leg wrap
        # unless the body rate happened to be a whole number.
        self.body_phase_rad = 0.0
        self.motion_enabled = False
        self.body_tick_overrides: dict[int, int] = {}
        self.gait_test_mode = "full"
        self.motion_source = "cpg"
        self.leg_motion_design: LegMotionDesign | None = None
        self.leg_motion_preview_index: int | None = None
        self.leg_motion_phase = 0.0
        self.leg_motion_controlled_indices: set[int] = set()
        self.leg_motion_hold_targets: dict[int, int] = {}

    def set_gait_test_mode(self, mode: str) -> None:
        if mode not in GAIT_TEST_MODES:
            raise ValueError(f"unknown gait test mode: {mode}")
        self.gait_test_mode = mode

    def normalized_to_rad(self, index: int, value: float) -> float:
        clipped = clip_unit(value)
        low = self.config.joint_lower[index]
        high = self.config.joint_upper[index]
        return low + (clipped + 1.0) * 0.5 * (high - low)

    def rad_to_raw_tick(self, index: int, radians: float) -> int:
        zero = self.config.zero_ticks[index]
        direction = self.config.directions[index]
        tick = zero + rounded(direction * radians * TICKS_PER_RAD)
        low_tick = zero + math.floor(direction * self.config.joint_lower[index] * TICKS_PER_RAD)
        high_tick = zero + math.ceil(direction * self.config.joint_upper[index] * TICKS_PER_RAD)
        min_tick = min(low_tick, high_tick)
        max_tick = max(low_tick, high_tick)
        return max(min_tick, min(max_tick, tick))

    def raw_tick_to_normalized(self, index: int, tick: int) -> float:
        direction = self.config.directions[index]
        radians = (int(tick) - self.config.zero_ticks[index]) / (
            direction * TICKS_PER_RAD
        )
        low = self.config.joint_lower[index]
        high = self.config.joint_upper[index]
        if high <= low:
            raise ValueError(f"invalid joint limits for {JOINT_LABELS[index]}")
        return clip_unit(2.0 * (radians - low) / (high - low) - 1.0)

    def leg_is_enabled(self, leg_index: int) -> bool:
        leg = LEGS[leg_index]
        enabled = set(self.config.enabled_indices)
        return all(
            index in enabled
            for index in (leg.yaw_index, leg.lift_index, leg.knee_index)
        )

    def read_leg_semantic_pose(self, leg_index: int) -> tuple[float, float, float]:
        if not self.leg_is_enabled(leg_index):
            raise ValueError(f"leg {leg_index + 1} is disabled")
        leg = LEGS[leg_index]
        yaw_action = self.raw_tick_to_normalized(
            leg.yaw_index,
            self.bus.read_position(self.config.ids[leg.yaw_index]),
        )
        lift_action = self.raw_tick_to_normalized(
            leg.lift_index,
            self.bus.read_position(self.config.ids[leg.lift_index]),
        )
        knee_action = self.raw_tick_to_normalized(
            leg.knee_index,
            self.bus.read_position(self.config.ids[leg.knee_index]),
        )
        return (
            clip_unit(yaw_action / leg.side_sign),
            clip_unit(lift_action / (-leg.side_sign)),
            clip_unit(knee_action),
        )

    def set_leg_torque(self, leg_index: int, enabled: bool) -> None:
        if not self.leg_is_enabled(leg_index):
            raise ValueError(f"leg {leg_index + 1} is disabled")
        leg = LEGS[leg_index]
        for index in (leg.yaw_index, leg.lift_index, leg.knee_index):
            self.bus.set_torque(self.config.ids[index], enabled)

    def enable_leg_torque_at_current_position(self, leg_index: int) -> None:
        if not self.leg_is_enabled(leg_index):
            raise ValueError(f"leg {leg_index + 1} is disabled")
        leg = LEGS[leg_index]
        indices = (leg.yaw_index, leg.lift_index, leg.knee_index)
        current_targets = {
            self.config.ids[index]: self.bus.read_position(self.config.ids[index])
            for index in indices
        }
        self.bus.sync_move_to(current_targets)
        for index in indices:
            self.bus.set_torque(self.config.ids[index], True)

    def filter_params(self, dt: float) -> None:
        alpha = dt / (ACTION_FILTER_TAU + dt)
        for i, value in enumerate(self.gait_params):
            raw = clip_unit(value)
            self.filtered_params[i] += alpha * (raw - self.filtered_params[i])

    def compute_servo_action(self, dt: float) -> None:
        if self.gait_model == "v2":
            self.compute_servo_action_v2(dt)
        else:
            self.compute_servo_action_v1(dt)

    def param(self, index: int) -> float:
        """Filtered parameter mapped through its own spec range, never a copy of it."""
        _name, _unit, low, high = GAIT_MODELS[self.gait_model].specs[index]
        return map_range(self.filtered_params[index], low, high)

    def compute_servo_action_v2(self, dt: float) -> None:
        """Drive yaw and lift from one fore-aft stroke, with the knee counter-coupled.

        The stroke `s` runs +1 (front) to -1 (rear) at constant speed while the foot
        is on the ground, then swings back smoothly. `clearance` lifts the foot only
        during that return, and `foot_level` folds the knee against the hip so the
        foot keeps its angle to the ground through the whole stroke.
        """
        self.servo_action = [0.0] * SERVO_COUNT

        frequency_hz = self.param(0)
        stride = self.param(1)
        stride_bias = self.param(2)
        leg_swing = self.param(3)
        foot_level = self.param(4)
        clearance = self.param(5)
        stance_duty = self.param(6)
        knee_bias = self.param(7)
        lift_bias = self.param(8)
        turn = self.param(9)
        segment_lag = self.param(10)
        side_lag = self.param(11)
        body_amp = self.param(12)
        body_lag = self.param(13)
        body_turn = self.param(14)
        knee_phase = self.param(15)
        body_rate = self.param(16)
        sweep_direction = -1.0 if self.config.reverse_legs else 1.0

        self.phase_rad += TWO_PI_F * frequency_hz * dt
        while self.phase_rad >= TWO_PI_F:
            self.phase_rad -= TWO_PI_F

        # sweep_phase_offset_rad shifts yaw against lift; knee_phase delays the knee
        # against lift, so the foot can be kept level through the whole stroke.
        yaw_offset = self.config.sweep_phase_offset_rad / TWO_PI_F
        knee_offset = knee_phase / TWO_PI_F

        for leg in LEGS:
            leg_phase = self.phase_rad + leg.segment_index * segment_lag + leg.side_index * side_lag
            cycle = (leg_phase / TWO_PI_F) % 1.0

            stroke, foot_rise = gait_stroke(cycle, stance_duty)
            yaw_stroke, _ = gait_stroke(cycle + yaw_offset, stance_duty)
            knee_stroke, knee_rise = gait_stroke(cycle - knee_offset, stance_duty)

            stroke *= sweep_direction
            yaw_stroke *= sweep_direction
            knee_stroke *= sweep_direction
            if self.gait_test_mode == "lift":
                stroke = 0.0
                yaw_stroke = 0.0
                knee_stroke = 0.0
            elif self.gait_test_mode == "ground":
                foot_rise = 0.0
                knee_rise = 0.0

            # Positive turn lengthens the left stride and shortens the right one.
            turn_gain = 1.0 + turn * (1.0 if leg.side_index == 0 else -1.0)
            hip = leg_swing * stroke + clearance * foot_rise

            self.servo_action[leg.yaw_index] = clip_unit(
                leg.side_sign * (stride_bias + stride * turn_gain * yaw_stroke)
            )
            knee_hip = leg_swing * knee_stroke + clearance * knee_rise

            self.servo_action[leg.lift_index] = clip_unit(-leg.side_sign * (lift_bias + hip))
            self.servo_action[leg.knee_index] = clip_unit(
                leg.side_sign * (knee_bias - foot_level * knee_hip)
            )

        # Body turn bends IDs 1-3 the same way to steer; the wave rides on top of it.
        self.body_phase_rad += TWO_PI_F * frequency_hz * body_rate * dt
        while self.body_phase_rad >= TWO_PI_F:
            self.body_phase_rad -= TWO_PI_F
        for i in range(BODY_COUNT):
            body_phase = self.body_phase_rad + i * body_lag
            body_wave = body_amp * math.sin(body_phase) if self.gait_test_mode == "full" else 0.0
            self.servo_action[BODY_JOINT_INDEX[i]] = clip_unit(body_turn + body_wave)

    def compute_servo_action_v1(self, dt: float) -> None:
        self.servo_action = [0.0] * SERVO_COUNT

        frequency_hz = map_range(self.filtered_params[0], 0.25, 1.20)
        sweep_amp = map_range(self.filtered_params[1], 0.08, 0.55)
        lift_amp = map_range(self.filtered_params[2], 0.08, 0.58)
        knee_amp = map_range(self.filtered_params[3], -0.50, 0.50)
        body_amp = map_range(self.filtered_params[4], 0.00, 0.38)
        segment_lag = map_range(self.filtered_params[5], -PI_F, PI_F)
        side_lag = map_range(self.filtered_params[6], 0.45 * PI_F, 1.35 * PI_F)
        sweep_bias = map_range(self.filtered_params[7], -0.20, 0.20)
        lift_bias = map_range(self.filtered_params[8], -0.18, 0.18)
        knee_bias = map_range(self.filtered_params[9], -0.18, 0.18)
        body_turn_bias = map_range(self.filtered_params[10], -0.35, 0.35)
        body_lag = map_range(self.filtered_params[11], -PI_F, PI_F)
        sweep_direction = -1.0 if self.config.reverse_legs else 1.0

        self.phase_rad += TWO_PI_F * frequency_hz * dt
        while self.phase_rad >= TWO_PI_F:
            self.phase_rad -= TWO_PI_F

        for leg in LEGS:
            leg_phase = self.phase_rad + leg.segment_index * segment_lag + leg.side_index * side_lag
            lift_wave = math.sin(leg_phase)
            sweep_wave = math.sin(leg_phase + self.config.sweep_phase_offset_rad)
            swing_lift = max(0.0, lift_wave)
            swing_fold = max(0.0, math.sin(leg_phase + 0.25 * PI_F))

            if self.gait_test_mode == "lift":
                yaw_action = leg.side_sign * sweep_bias
            else:
                yaw_action = leg.side_sign * (
                    sweep_bias + sweep_direction * sweep_amp * sweep_wave
                )
            self.servo_action[leg.yaw_index] = clip_unit(yaw_action)

            if self.gait_test_mode == "ground":
                lift_action = -leg.side_sign * lift_bias
                knee_action = leg.side_sign * knee_bias
            else:
                lift_action = -leg.side_sign * (lift_bias + lift_amp * swing_lift)
                knee_action = leg.side_sign * (knee_bias + knee_amp * swing_fold)
            self.servo_action[leg.lift_index] = clip_unit(lift_action)
            self.servo_action[leg.knee_index] = clip_unit(knee_action)

        for i in range(BODY_COUNT):
            body_phase = self.phase_rad + i * body_lag
            body_wave = body_amp * math.sin(body_phase) if self.gait_test_mode == "full" else 0.0
            self.servo_action[BODY_JOINT_INDEX[i]] = clip_unit(body_turn_bias + body_wave)

    def compute_leg_motion_action(self, dt: float) -> None:
        design = self.leg_motion_design
        if design is None:
            raise RuntimeError("leg motion design is not selected")
        self.servo_action = [0.0] * SERVO_COUNT
        self.leg_motion_phase = (
            self.leg_motion_phase + design.frequency_hz * dt
        ) % 1.0
        for leg_index, leg in enumerate(LEGS):
            if not self.leg_is_enabled(leg_index):
                continue
            if (
                self.leg_motion_preview_index is not None
                and leg_index != self.leg_motion_preview_index
            ):
                continue
            phase = (
                self.leg_motion_phase + design.phase_offsets[leg_index]
            ) % 1.0
            yaw, lift, knee = evaluate_leg_motion(design, phase)
            self.servo_action[leg.yaw_index] = clip_unit(leg.side_sign * yaw)
            self.servo_action[leg.lift_index] = clip_unit(-leg.side_sign * lift)
            self.servo_action[leg.knee_index] = clip_unit(leg.side_sign * knee)

    def targets_from_action(self) -> dict[int, int]:
        targets: dict[int, int] = {}
        indices = self.config.enabled_indices
        if self.motion_source == "leg-template":
            indices = [
                index
                for index in self.config.enabled_indices
                if index in self.leg_motion_controlled_indices
            ]
        for index in indices:
            dxl_id = self.config.ids[index]
            if dxl_id in self.leg_motion_hold_targets:
                targets[dxl_id] = self.leg_motion_hold_targets[dxl_id]
            elif index in self.body_tick_overrides:
                targets[dxl_id] = self.body_tick_overrides[index]
            else:
                radians = self.normalized_to_rad(index, self.servo_action[index])
                targets[dxl_id] = self.rad_to_raw_tick(index, radians)
        return targets

    def neutral_targets(self) -> dict[int, int]:
        return {
            dxl_id: self.config.zero_ticks[index]
            for index, dxl_id in enumerate(self.config.ids)
            if index in self.config.enabled_indices
        }

    def write_targets(self) -> None:
        self.bus.sync_move_to(self.targets_from_action())

    def write_neutral_targets(self) -> None:
        self.bus.sync_move_to(self.neutral_targets())

    def read_current_positions(self) -> dict[int, int]:
        return {
            self.config.ids[index]: self.bus.read_position(self.config.ids[index])
            for index in self.config.enabled_indices
        }

    def write_raw_targets(self, targets: dict[int, int]) -> None:
        self.bus.sync_move_to(targets)

    def set_torque(self, enabled: bool) -> None:
        if enabled:
            current_targets = self.read_current_positions()
            self.bus.sync_move_to(current_targets)
        for index in self.config.enabled_indices:
            self.bus.set_torque(self.config.ids[index], enabled)

    def initialize_servos(self, set_position_mode: bool = True) -> None:
        for index in self.config.enabled_indices:
            dxl_id = self.config.ids[index]
            self.bus.set_torque(dxl_id, False)
            if set_position_mode:
                self.bus.set_position_mode(dxl_id)
        current_targets = self.read_current_positions()
        self.bus.sync_move_to(current_targets)
        for index in self.config.enabled_indices:
            dxl_id = self.config.ids[index]
            self.bus.set_torque(dxl_id, True)

    def start_motion(self) -> None:
        self.motion_source = "cpg"
        self.leg_motion_controlled_indices.clear()
        self.leg_motion_hold_targets.clear()
        self.motion_enabled = True
        self.phase_rad = 0.0
        self.body_phase_rad = 0.0
        self.filtered_params = [clip_unit(value) for value in self.gait_params]

    def start_leg_motion(
        self,
        design: LegMotionDesign,
        preview_leg_index: int | None = None,
    ) -> None:
        update_leg_motion_timing(design)
        validate_leg_motion_design(design)
        if preview_leg_index is not None and not self.leg_is_enabled(preview_leg_index):
            raise ValueError(f"leg {preview_leg_index + 1} is disabled")
        controlled_leg_indices = [
            leg_index
            for leg_index in range(LEG_COUNT)
            if self.leg_is_enabled(leg_index)
            and (
                preview_leg_index is None
                or leg_index == preview_leg_index
            )
        ]
        controlled_indices: set[int] = set()
        hold_targets: dict[int, int] = {}
        zero_columns = leg_motion_zero_columns(design)
        for leg_index in controlled_leg_indices:
            leg = LEGS[leg_index]
            joint_indices = {
                "yaw": leg.yaw_index,
                "lift": leg.lift_index,
                "knee": leg.knee_index,
            }
            controlled_indices.update(joint_indices.values())
            for joint in zero_columns:
                index = joint_indices[joint]
                dxl_id = self.config.ids[index]
                hold_targets[dxl_id] = self.bus.read_position(dxl_id)
        self.motion_source = "leg-template"
        self.leg_motion_design = design
        self.leg_motion_preview_index = preview_leg_index
        self.leg_motion_controlled_indices = controlled_indices
        self.leg_motion_hold_targets = hold_targets
        self.leg_motion_phase = 0.0
        self.motion_enabled = True

    def stop_motion_neutral(self) -> None:
        self.motion_enabled = False
        self.write_neutral_targets()

    def write_all_2048(self) -> None:
        self.motion_enabled = False
        self.body_tick_overrides.clear()
        self.bus.sync_move_to(
            {
                self.config.ids[index]: 2048
                for index in self.config.enabled_indices
            }
        )

    def set_body_tick(self, index: int, tick: int) -> None:
        if index not in BODY_JOINT_INDEX:
            raise ValueError(f"index {index} is not a body joint")
        if index not in self.config.enabled_indices:
            raise ValueError(f"{JOINT_LABELS[index]} is disabled")
        zero_tick = self.config.zero_ticks[index]
        direction = self.config.directions[index]
        low_tick = zero_tick + math.floor(
            direction * self.config.joint_lower[index] * TICKS_PER_RAD
        )
        high_tick = zero_tick + math.ceil(
            direction * self.config.joint_upper[index] * TICKS_PER_RAD
        )
        clipped_tick = max(min(low_tick, high_tick), min(max(low_tick, high_tick), int(tick)))
        self.body_tick_overrides[index] = clipped_tick
        if not self.motion_enabled:
            self.bus.move_to(self.config.ids[index], clipped_tick)

    def clear_body_tick_overrides(self) -> None:
        self.body_tick_overrides.clear()

    def update(self, dt: float, expected_period: float = CONTROL_PERIOD_SECONDS) -> None:
        if dt <= 0.0 or dt > MAX_DT:
            dt = expected_period
        if not self.motion_enabled:
            return
        if self.motion_source == "leg-template":
            self.compute_leg_motion_action(dt)
        else:
            self.filter_params(dt)
            self.compute_servo_action(dt)
        self.write_targets()


class CPGGui:
    def __init__(
        self,
        device: str,
        baudrate: int,
        protocol: float,
        config: RobotConfig,
        output_path: str,
        preset_path: str,
        leg_design_path: str,
        skip_init: bool,
        skip_position_mode: bool,
        torque_on: bool,
        torque_off_exit: bool,
        control_hz: float | None,
        onboard_cpg: bool,
        gait_model: str = DEFAULT_GAIT_MODEL,
    ) -> None:
        import tkinter as tk
        from tkinter import messagebox, ttk

        if onboard_cpg and gait_model != "v1":
            raise RuntimeError(
                "onboard CPG firmware only implements the v1 gait model; "
                "drop --onboard-cpg to run v2 from the PC"
            )

        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.gait_model = gait_model
        self.model = GAIT_MODELS[gait_model]
        self.root = tk.Tk()
        self.root.title(f"Hanachan CPG Tuner ({gait_model})")
        self.root.geometry("980x760")
        self.root.minsize(780, 620)

        self.config = config
        self.onboard_cpg = onboard_cpg
        if onboard_cpg:
            self.bus = WirelessCPGClient(device, baudrate)
            self.controller = OnboardHanachanCPGController(self.bus, config)
        else:
            self.bus = DynamixelBus(device, baudrate, protocol)
            self.controller = HanachanCPGController(self.bus, config, gait_model)
        self.output_path = output_path
        self.preset_path = preset_path
        self.leg_design_path = leg_design_path
        self.preset_load_error: str | None = None
        try:
            self.gait_presets = load_gait_presets(preset_path)
        except Exception as exc:
            self.gait_presets = []
            self.preset_load_error = str(exc)
        self.skip_init = skip_init
        self.skip_position_mode = skip_position_mode
        self.torque_on = torque_on
        self.torque_off_exit = torque_off_exit
        self.control_period = resolve_control_period(baudrate, control_hz)
        self.gait_params = config.params_for(gait_model)
        self.param_vars = [tk.DoubleVar(value=value) for value in self.gait_params]
        self.param_value_vars = [tk.StringVar() for _ in range(self.model.param_count)]
        self.traction_phase_var = tk.DoubleVar(
            value=math.degrees(self.config.sweep_phase_offset_rad)
        )
        self.traction_phase_value_var = tk.StringVar()
        self.gait_test_mode_var = tk.StringVar(value="Full gait")
        self.preset_name_var = tk.StringVar()
        self.preset_combo = None
        self.leg_motion_designer = None
        # The parameter sliders live only in the advanced panel, so show it by default.
        self.show_advanced_var = tk.BooleanVar(value=True)
        self.advanced_container = None
        self.show_body_var = tk.BooleanVar(value=False)
        self.body_container = None
        self.footer = None
        self.body_vars: dict[int, tk.IntVar] = {}
        self.body_value_vars: dict[int, tk.StringVar] = {}
        self.pending_body_after: dict[int, str] = {}
        self.latest_body_ticks: dict[int, int] = {}
        self.suppress_body_slider_events = False
        self.initial_position_after: str | None = None
        self.initial_position_active = False
        self.initial_position_started = 0.0
        self.initial_position_start_targets: dict[int, int] = {}
        self.initial_position_targets: dict[int, int] = {}
        self.status_var = tk.StringVar(value="Opening DYNAMIXEL bus...")
        self.reverse_legs_var = tk.BooleanVar(value=self.config.reverse_legs)
        self.last_time = time.monotonic()
        self.closed = False

    def run(self) -> None:
        try:
            self.bus.__enter__()
            configured_ids = [self.config.ids[index] for index in self.config.enabled_indices]
            if self.onboard_cpg:
                found_ids = set(configured_ids)
            else:
                found = self.bus.scan(sorted(set(configured_ids)))
                found_ids = {dxl_id for dxl_id, _model in found}
                missing = sorted(set(configured_ids) - found_ids)
                if missing:
                    raise RuntimeError(f"configured DYNAMIXEL IDs not found: {missing}")

            if not self.skip_init:
                self.controller.initialize_servos(set_position_mode=not self.skip_position_mode)
            elif self.torque_on:
                self.controller.set_torque(True)

            self.build_widgets(found_ids)
            self.root.protocol("WM_DELETE_WINDOW", self.close)
            self.last_time = time.monotonic()
            self.root.after(round(self.control_period * 1000), self.control_tick)
            self.root.mainloop()
        except Exception as exc:
            self.messagebox.showerror("Hanachan CPG Tuner", str(exc))
            try:
                self.bus.__exit__(None, None, None)
            except Exception:
                pass

    def build_widgets(self, found_ids: set[int]) -> None:
        tk = self.tk
        ttk = self.ttk
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(outer)
        header.pack(fill=tk.X)
        ttk.Label(header, text="Hanachan CPG Tuner", font=("", 18, "bold")).pack(side=tk.LEFT)
        ttk.Label(header, text=f"Detected: {', '.join(map(str, sorted(found_ids)))}").pack(side=tk.RIGHT)

        controls = ttk.Frame(outer)
        controls.pack(fill=tk.X, pady=(12, 8))
        ttk.Button(controls, text="Start", command=self.start_motion).pack(side=tk.LEFT)
        ttk.Button(controls, text="Stop / Neutral", command=self.stop_motion).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(controls, text="Torque On", command=lambda: self.set_torque(True)).pack(side=tk.LEFT, padx=(14, 0))
        ttk.Button(controls, text="Torque Off", command=lambda: self.set_torque(False)).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(controls, text="Save Config", command=self.save_config).pack(side=tk.RIGHT)

        presets = ttk.LabelFrame(outer, text="Presets", padding=8)
        presets.pack(fill=tk.X, pady=(0, 8))
        quick_presets = ttk.Frame(presets)
        quick_presets.pack(fill=tk.X)
        ttk.Button(quick_presets, text="Safe slow", command=lambda: self.apply_gait_preset(self.model.safe_params, "Safe slow")).pack(side=tk.LEFT)
        ttk.Button(quick_presets, text="Forward (design)", command=lambda: self.apply_gait_preset(self.model.forward_params, "Forward")).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            quick_presets,
            text="Initial position (2 s)",
            command=self.move_to_initial_position,
        ).pack(side=tk.LEFT, padx=(18, 0))
        ttk.Button(
            quick_presets,
            text="Leg Motion Designer...",
            command=self.open_leg_motion_designer,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Checkbutton(
            quick_presets,
            text="Show advanced controls",
            variable=self.show_advanced_var,
            command=self.toggle_advanced_controls,
        ).pack(side=tk.RIGHT)

        named_presets = ttk.Frame(presets)
        named_presets.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(named_presets, text="Named preset:").pack(side=tk.LEFT)
        self.preset_combo = ttk.Combobox(
            named_presets,
            textvariable=self.preset_name_var,
            values=[preset.name for preset in self.presets_for_model()],
            width=30,
        )
        self.preset_combo.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 6))
        ttk.Button(named_presets, text="Load", command=self.load_named_preset).pack(side=tk.LEFT)
        ttk.Button(named_presets, text="Save", command=self.save_named_preset).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(named_presets, text="Delete", command=self.delete_named_preset).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        # Every gait parameter has a slider in the advanced panel, so this frame
        # only carries what is not a gait parameter.
        easy = ttk.LabelFrame(outer, text="Timing and tests", padding=10)
        easy.pack(fill=tk.X, pady=(0, 8))
        easy.columnconfigure(1, weight=1)

        ttk.Label(easy, text="Traction timing", width=18).grid(row=0, column=0, sticky="w")
        tk.Scale(
            easy,
            from_=-180,
            to=180,
            orient=tk.HORIZONTAL,
            showvalue=False,
            resolution=5,
            variable=self.traction_phase_var,
            command=self.traction_phase_changed,
        ).grid(row=0, column=1, sticky="ew")
        ttk.Label(
            easy,
            textvariable=self.traction_phase_value_var,
            width=18,
        ).grid(row=0, column=2, sticky="e", padx=(8, 0))
        self.update_traction_phase_label()

        timing_buttons = ttk.Frame(easy)
        timing_buttons.grid(row=1, column=1, sticky="w", pady=(4, 8))
        for degrees in (-90, -45, 0, 45, 90):
            ttk.Button(
                timing_buttons,
                text=f"{degrees:+d}°" if degrees else "0°",
                command=lambda value=degrees: self.set_traction_phase_degrees(value),
            ).pack(side=tk.LEFT, padx=(0, 4))

        test_modes = ttk.Frame(easy)
        test_modes.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(2, 0))
        ttk.Label(test_modes, text="Step test:").pack(side=tk.LEFT)
        ttk.Button(
            test_modes,
            text="1 Lift test",
            command=lambda: self.set_gait_test_mode("lift"),
        ).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(
            test_modes,
            text="2 Ground stroke",
            command=lambda: self.set_gait_test_mode("ground"),
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            test_modes,
            text="3 Full gait",
            command=lambda: self.set_gait_test_mode("full"),
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(test_modes, textvariable=self.gait_test_mode_var).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Checkbutton(
            test_modes,
            text="Reverse propulsion",
            variable=self.reverse_legs_var,
            command=self.reverse_legs_changed,
        ).pack(side=tk.RIGHT)

        self.advanced_container = ttk.Frame(outer)

        body = ttk.LabelFrame(self.advanced_container, text="Body positions", padding=8)
        body.pack(fill=tk.X, pady=(0, 8))
        ttk.Checkbutton(
            body,
            text="Show body position sliders",
            variable=self.show_body_var,
            command=self.toggle_body_positions,
        ).pack(anchor=tk.W)
        # Collapsed by default so the gait parameter list below gets the space.
        self.body_container = ttk.Frame(body)
        for column, index in enumerate(BODY_JOINT_INDEX):
            self.body_container.columnconfigure(column, weight=1)
            cell = ttk.Frame(self.body_container)
            cell.grid(row=0, column=column, sticky="ew", padx=(0 if column == 0 else 8, 0))
            dxl_id = self.config.ids[index]
            zero_tick = self.config.zero_ticks[index]
            value_var = tk.StringVar(value=str(zero_tick))
            tick_var = tk.IntVar(value=zero_tick)
            self.body_value_vars[index] = value_var
            self.body_vars[index] = tick_var
            direction = self.config.directions[index]
            low_tick = zero_tick + math.floor(
                direction * self.config.joint_lower[index] * TICKS_PER_RAD
            )
            high_tick = zero_tick + math.ceil(
                direction * self.config.joint_upper[index] * TICKS_PER_RAD
            )
            ttk.Label(cell, text=f"ID {dxl_id}", font=("", 11, "bold")).pack(anchor=tk.W)
            scale = tk.Scale(
                cell,
                from_=min(low_tick, high_tick),
                to=max(low_tick, high_tick),
                orient=tk.HORIZONTAL,
                showvalue=False,
                resolution=1,
                variable=tick_var,
                command=lambda value, i=index: self.queue_body_tick(i, value),
            )
            scale.pack(fill=tk.X)
            ttk.Label(cell, textvariable=value_var).pack(anchor=tk.E)
        ttk.Button(self.body_container, text="Use CPG body", command=self.use_cpg_body).grid(
            row=1, column=0, columnspan=3, sticky="e", pady=(6, 0)
        )
        self.toggle_body_positions()

        param_container = ttk.LabelFrame(
            self.advanced_container,
            text="Advanced gait parameters",
            padding=8,
        )
        param_container.pack(fill=tk.BOTH, expand=True)
        canvas = tk.Canvas(param_container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(param_container, orient=tk.VERTICAL, command=canvas.yview)
        params = ttk.Frame(canvas)
        params.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        params_window = canvas.create_window((0, 0), window=params, anchor=tk.NW)
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(params_window, width=event.width))
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        params.columnconfigure(1, weight=1)
        for index, (name, _unit, _low, _high) in enumerate(self.model.specs):
            ttk.Label(params, text=name, width=18).grid(row=index, column=0, sticky="w", padx=(0, 8))
            scale = tk.Scale(
                params,
                from_=-1.0,
                to=1.0,
                orient=tk.HORIZONTAL,
                showvalue=False,
                resolution=0.01,
                variable=self.param_vars[index],
                command=lambda _value, i=index: self.param_changed(i),
            )
            scale.grid(row=index, column=1, sticky="ew")
            ttk.Label(params, textvariable=self.param_value_vars[index], width=18).grid(
                row=index, column=2, sticky="e", padx=(8, 0)
            )
            ttk.Label(
                params,
                text=self.model.descriptions[index],
                wraplength=330,
                justify=tk.LEFT,
            ).grid(row=index, column=3, sticky="w", padx=(14, 0))
            self.update_param_label(index)

        self.footer = ttk.Frame(outer)
        self.footer.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(self.footer, textvariable=self.status_var).pack(side=tk.LEFT)
        if self.onboard_cpg:
            rate_text = "OpenRB onboard CPG at 50 Hz"
        else:
            rate_text = f"PC control at {1.0 / self.control_period:.1f} Hz"
        if self.preset_load_error:
            self.status_var.set(f"Preset file error: {self.preset_load_error}")
        else:
            self.status_var.set(
                f"Ready. {rate_text}. Config: {self.output_path}. Presets: {self.preset_path}"
            )

        # Needs the footer, which is why it runs here and not where the panel is built.
        self.toggle_advanced_controls()

    def toggle_advanced_controls(self) -> None:
        if self.advanced_container is None or self.footer is None:
            return
        if self.show_advanced_var.get():
            self.advanced_container.pack(
                fill=self.tk.BOTH,
                expand=True,
                pady=(0, 8),
                before=self.footer,
            )
        else:
            self.advanced_container.pack_forget()

    def toggle_body_positions(self) -> None:
        if self.body_container is None:
            return
        if self.show_body_var.get():
            self.body_container.pack(fill=self.tk.X, pady=(6, 0))
        else:
            self.body_container.pack_forget()

    def open_leg_motion_designer(self) -> None:
        if self.onboard_cpg:
            self.messagebox.showinfo(
                "Leg Motion Designer",
                "Leg Motion Designer is currently available in wired PC control mode.",
            )
            return
        if (
            self.leg_motion_designer is not None
            and self.leg_motion_designer.window.winfo_exists()
        ):
            self.leg_motion_designer.window.lift()
            self.leg_motion_designer.window.focus_force()
            return
        self.leg_motion_designer = LegMotionDesignerWindow(
            owner=self,
            controller=self.controller,
            config=self.config,
            design_path=self.leg_design_path,
        )

    def update_param_label(self, index: int) -> None:
        normalized = self.param_vars[index].get()
        _name, unit, low, high = self.model.specs[index]
        physical = map_range(normalized, low, high)
        suffix = f" {unit}" if unit else ""
        self.param_value_vars[index].set(f"{normalized:+.2f}  ({physical:.3f}{suffix})")

    def param_changed(self, index: int) -> None:
        self.gait_params[index] = clip_unit(self.param_vars[index].get())
        self.update_param_label(index)

    def update_traction_phase_label(self) -> None:
        self.traction_phase_value_var.set(f"{self.traction_phase_var.get():+.0f}°")

    def traction_phase_changed(self, _value: str | None = None) -> None:
        degrees = max(-180.0, min(180.0, self.traction_phase_var.get()))
        self.config.sweep_phase_offset_rad = math.radians(degrees)
        self.update_traction_phase_label()

    def set_traction_phase_degrees(self, degrees: float) -> None:
        self.traction_phase_var.set(degrees)
        self.traction_phase_changed()
        self.status_var.set(f"Traction timing set to {degrees:+.0f}°. Press Start to compare.")

    def set_gait_test_mode(self, mode: str) -> None:
        try:
            self.cancel_initial_position()
            was_running = self.controller.motion_enabled
            if was_running:
                self.controller.stop_motion_neutral()
            self.controller.set_gait_test_mode(mode)
            labels = {
                "lift": "Lift test",
                "ground": "Ground stroke",
                "full": "Full gait",
            }
            self.gait_test_mode_var.set(labels[mode])
            prefix = "Stopped CPG. " if was_running else ""
            self.status_var.set(f"{prefix}{labels[mode]} selected. Press Start to run.")
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")

    def apply_gait_preset(self, values: list[float], name: str) -> None:
        for index, value in enumerate(values):
            self.gait_params[index] = value
            self.param_vars[index].set(value)
            self.update_param_label(index)
        self.status_var.set(f"{name} parameters loaded. Press Start to run.")

    def presets_for_model(self) -> list[GaitPreset]:
        """Presets from another gait model would be read with the wrong meaning."""
        return [preset for preset in self.gait_presets if preset.gait_model == self.gait_model]

    def refresh_named_preset_list(self) -> None:
        if self.preset_combo is not None:
            self.preset_combo.configure(
                values=[preset.name for preset in self.presets_for_model()]
            )

    def selected_named_preset(self) -> GaitPreset:
        name = self.preset_name_var.get().strip()
        if not name:
            raise ValueError("Enter or select a preset name.")
        for preset in self.presets_for_model():
            if preset.name == name:
                return preset
        for preset in self.gait_presets:
            if preset.name == name:
                raise ValueError(
                    f"Preset '{name}' belongs to gait model {preset.gait_model}, not {self.gait_model}"
                )
        raise ValueError(f"Preset not found: {name}")

    def load_named_preset(self) -> None:
        try:
            preset = self.selected_named_preset()
            self.cancel_initial_position()
            was_running = self.controller.motion_enabled
            if was_running:
                self.controller.stop_motion_neutral()
            self.apply_gait_preset(preset.gait_params, preset.name)
            self.config.sweep_phase_offset_rad = preset.sweep_phase_offset_rad
            self.traction_phase_var.set(math.degrees(preset.sweep_phase_offset_rad))
            self.update_traction_phase_label()
            self.config.reverse_legs = preset.reverse_legs
            self.reverse_legs_var.set(preset.reverse_legs)
            prefix = "Stopped CPG. " if was_running else ""
            self.status_var.set(f"{prefix}Loaded preset '{preset.name}'. Press Start to run.")
        except Exception as exc:
            self.messagebox.showerror("Load gait preset", str(exc))

    def save_named_preset(self) -> None:
        try:
            if self.preset_load_error:
                raise RuntimeError(
                    f"Cannot overwrite an unreadable preset file: {self.preset_load_error}"
                )
            name = self.preset_name_var.get().strip()
            if not name:
                raise ValueError("Enter a preset name before saving.")
            preset = GaitPreset(
                name=name,
                gait_params=list(self.gait_params),
                sweep_phase_offset_rad=self.config.sweep_phase_offset_rad,
                reverse_legs=self.config.reverse_legs,
                gait_model=self.gait_model,
            )
            validate_gait_preset(preset)
            # Keyed by model too: the list only shows this model, so matching on
            # the name alone would silently replace another model's preset.
            existing_index = next(
                (
                    index
                    for index, item in enumerate(self.gait_presets)
                    if item.name == name and item.gait_model == self.gait_model
                ),
                None,
            )
            if existing_index is not None:
                if not self.messagebox.askyesno(
                    "Overwrite gait preset",
                    f"Overwrite preset '{name}'?",
                ):
                    return
                self.gait_presets[existing_index] = preset
                action = "Updated"
            else:
                self.gait_presets.append(preset)
                action = "Saved"
            save_gait_presets(self.preset_path, self.gait_presets)
            self.refresh_named_preset_list()
            self.status_var.set(f"{action} preset '{name}' in {self.preset_path}")
        except Exception as exc:
            self.messagebox.showerror("Save gait preset", str(exc))

    def delete_named_preset(self) -> None:
        try:
            preset = self.selected_named_preset()
            if not self.messagebox.askyesno(
                "Delete gait preset",
                f"Delete preset '{preset.name}'?",
            ):
                return
            self.gait_presets = [
                item
                for item in self.gait_presets
                if not (item.name == preset.name and item.gait_model == preset.gait_model)
            ]
            save_gait_presets(self.preset_path, self.gait_presets)
            self.preset_name_var.set("")
            self.refresh_named_preset_list()
            self.status_var.set(f"Deleted preset '{preset.name}'")
        except Exception as exc:
            self.messagebox.showerror("Delete gait preset", str(exc))

    def reverse_legs_changed(self) -> None:
        requested = bool(self.reverse_legs_var.get())
        previous = self.config.reverse_legs
        try:
            self.cancel_initial_position()
            was_running = self.controller.motion_enabled
            if was_running:
                self.controller.stop_motion_neutral()
            self.config.reverse_legs = requested
            mode = "reversed" if requested else "normal"
            prefix = "Stopped CPG. " if was_running else ""
            self.status_var.set(
                f"{prefix}Leg sweep is {mode}; lift, knee, and body timing are unchanged."
            )
        except Exception as exc:
            self.config.reverse_legs = previous
            self.reverse_legs_var.set(previous)
            self.status_var.set(f"Error: {exc}")

    def start_motion(self) -> None:
        try:
            self.cancel_initial_position()
            self.controller.start_motion()
            self.status_var.set("CPG running")
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")

    def stop_motion(self) -> None:
        try:
            self.cancel_initial_position()
            self.controller.stop_motion_neutral()
            self.status_var.set("Stopped at configured neutral positions")
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")

    def set_torque(self, enabled: bool) -> None:
        try:
            self.cancel_initial_position()
            if not enabled:
                self.controller.motion_enabled = False
            self.controller.set_torque(enabled)
            self.status_var.set(f"Torque {'on' if enabled else 'off'}")
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")

    def queue_body_tick(self, index: int, value: str) -> None:
        if self.suppress_body_slider_events:
            return
        tick = int(float(value))
        self.body_value_vars[index].set(str(tick))
        self.latest_body_ticks[index] = tick
        if index not in self.pending_body_after:
            self.pending_body_after[index] = self.root.after(
                BODY_SLIDER_SEND_INTERVAL_MS,
                lambda: self.send_latest_body_tick(index),
            )

    def send_latest_body_tick(self, index: int) -> None:
        self.pending_body_after.pop(index, None)
        tick = self.latest_body_ticks.pop(index, None)
        if tick is None:
            return
        try:
            self.cancel_initial_position()
            self.controller.set_body_tick(index, tick)
            self.status_var.set(f"ID {self.config.ids[index]} body override: {tick}")
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")

    def use_cpg_body(self) -> None:
        self.cancel_pending_body_ticks()
        self.controller.clear_body_tick_overrides()
        self.status_var.set("IDs 1-3 returned to CPG control")

    def cancel_pending_body_ticks(self) -> None:
        for after_id in self.pending_body_after.values():
            self.root.after_cancel(after_id)
        self.pending_body_after.clear()
        self.latest_body_ticks.clear()

    def cancel_initial_position(self) -> None:
        self.initial_position_active = False
        if self.initial_position_after is not None:
            self.root.after_cancel(self.initial_position_after)
            self.initial_position_after = None

    def move_to_initial_position(self) -> None:
        try:
            self.cancel_initial_position()
            self.cancel_pending_body_ticks()
            self.controller.motion_enabled = False
            self.controller.clear_body_tick_overrides()
            if self.onboard_cpg:
                self.controller.start_initial_position(INITIAL_POSITION_DURATION_SECONDS)
                self.initial_position_active = True
                self.status_var.set("Moving slowly to configured initial position (2.0 s)...")
                self.initial_position_after = self.root.after(
                    rounded(INITIAL_POSITION_DURATION_SECONDS * 1000.0),
                    self.finish_initial_position,
                )
                return

            self.initial_position_start_targets = self.controller.read_current_positions()
            self.initial_position_targets = self.controller.neutral_targets()
            self.initial_position_started = time.monotonic()
            self.initial_position_active = True
            self.status_var.set("Moving slowly to configured initial position (2.0 s)...")
            self.initial_position_tick()
        except Exception as exc:
            self.cancel_initial_position()
            self.status_var.set(f"Error: {exc}")

    def initial_position_tick(self) -> None:
        if not self.initial_position_active or self.closed:
            return
        self.initial_position_after = None
        progress = (
            time.monotonic() - self.initial_position_started
        ) / INITIAL_POSITION_DURATION_SECONDS
        eased_progress = max(0.0, min(1.0, progress))
        eased_progress = eased_progress * eased_progress * (3.0 - 2.0 * eased_progress)
        try:
            targets = interpolate_positions(
                self.initial_position_start_targets,
                self.initial_position_targets,
                eased_progress,
            )
            self.controller.write_raw_targets(targets)
            if progress >= 1.0:
                self.finish_initial_position()
                return
            self.initial_position_after = self.root.after(
                round(self.control_period * 1000.0),
                self.initial_position_tick,
            )
        except Exception as exc:
            self.cancel_initial_position()
            self.status_var.set(f"Initial position error: {exc}")

    def finish_initial_position(self) -> None:
        self.initial_position_active = False
        self.initial_position_after = None
        self.suppress_body_slider_events = True
        try:
            for index in BODY_JOINT_INDEX:
                if index in self.body_vars:
                    tick = self.config.zero_ticks[index]
                    self.body_vars[index].set(tick)
                    self.body_value_vars[index].set(str(tick))
        finally:
            self.suppress_body_slider_events = False
        self.status_var.set("Stopped at configured initial position")

    def save_config(self) -> None:
        data = {
            "ids": self.config.ids,
            "zero_ticks": self.config.zero_ticks,
            "directions": self.config.directions,
            "joint_lower": self.config.joint_lower,
            "joint_upper": self.config.joint_upper,
            "gait_params": self.config.gait_params,
            "gait_params_v2": self.config.gait_params_v2,
            "enabled_indices": self.config.enabled_indices,
            "reverse_legs": self.config.reverse_legs,
            "sweep_phase_offset_rad": self.config.sweep_phase_offset_rad,
        }
        try:
            with open(self.output_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
                f.write("\n")
            self.status_var.set(f"Saved config to {self.output_path}")
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")

    def control_tick(self) -> None:
        if self.closed:
            return
        tick_started = time.monotonic()
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        try:
            self.controller.update(dt, self.control_period)
        except Exception as exc:
            self.controller.motion_enabled = False
            self.status_var.set(f"CPG stopped: {exc}")
        elapsed = time.monotonic() - tick_started
        delay_ms = max(1, round(max(0.0, self.control_period - elapsed) * 1000))
        self.root.after(delay_ms, self.control_tick)

    def close(self) -> None:
        self.closed = True
        if self.leg_motion_designer is not None:
            self.leg_motion_designer.close()
        self.cancel_initial_position()
        self.cancel_pending_body_ticks()
        if self.torque_off_exit:
            try:
                self.controller.set_torque(False)
            except Exception:
                pass
        self.bus.__exit__(None, None, None)
        self.root.destroy()


class LegMotionDesignerWindow:
    def __init__(
        self,
        owner: CPGGui,
        controller: HanachanCPGController,
        config: RobotConfig,
        design_path: str,
    ) -> None:
        tk = owner.tk
        ttk = owner.ttk
        self.tk = tk
        self.ttk = ttk
        self.messagebox = owner.messagebox
        self.owner = owner
        self.controller = controller
        self.config = config
        self.design_path = design_path
        self.window = tk.Toplevel(owner.root)
        self.window.title("Hanachan Leg Motion Designer")
        self.window.geometry("1160x860")
        self.window.minsize(960, 700)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.closed = False
        self.manual_torque_off = False
        self.manual_torque_off_leg_index: int | None = None
        self.recording = False
        self.recording_leg_index: int | None = None
        self.recording_started_at = 0.0
        self.recording_samples: list[
            tuple[float, tuple[float, float, float]]
        ] = []
        self.recording_after: str | None = None

        self.design_load_error: str | None = None
        try:
            self.designs = load_leg_motion_designs(design_path)
        except Exception as exc:
            self.designs = []
            self.design_load_error = str(exc)
        self.current_design = copy.deepcopy(
            self.designs[0] if self.designs else default_leg_motion_design()
        )

        self.design_name_var = tk.StringVar(value=self.current_design.name)
        self.frequency_var = tk.DoubleVar(value=self.current_design.frequency_hz)
        self.frequency_label_var = tk.StringVar()
        self.duration_var = tk.DoubleVar(value=0.25)
        self.yaw_var = tk.DoubleVar(value=0.0)
        self.lift_var = tk.DoubleVar(value=0.0)
        self.knee_var = tk.DoubleVar(value=0.0)
        self.frequency_callback_ready = False
        self.batch_start_var = tk.IntVar(value=1)
        self.batch_end_var = tk.IntVar(value=len(self.current_design.keyframes))
        self.batch_joint_var = tk.StringVar(value="Yaw")
        self.batch_operation_var = tk.StringVar(value="Set")
        self.batch_value_a_var = tk.StringVar(value="0.0")
        self.batch_value_b_var = tk.StringVar(value="0.0")
        self.status_var = tk.StringVar()
        self.phase_vars: dict[int, tk.DoubleVar] = {}
        self.phase_value_vars: dict[int, tk.StringVar] = {}
        self.tree = None
        self.design_combo = None

        self.active_leg_indices = [
            index for index in range(LEG_COUNT) if controller.leg_is_enabled(index)
        ]
        if not self.active_leg_indices:
            raise RuntimeError("No complete three-joint legs are enabled.")
        self.leg_labels = {
            index: self.leg_label(index) for index in self.active_leg_indices
        }
        self.source_leg_var = tk.StringVar(
            value=self.leg_labels[self.active_leg_indices[0]]
        )
        self.build_widgets()
        self.load_design_into_widgets()

    @staticmethod
    def leg_label(leg_index: int) -> str:
        leg = LEGS[leg_index]
        side = "Left" if leg.side_index == 0 else "Right"
        return f"Leg {leg.segment_index + 1} {side}"

    def selected_leg_index(self) -> int:
        selected = self.source_leg_var.get()
        for index, label in self.leg_labels.items():
            if label == selected:
                return index
        raise ValueError("Select a leg.")

    def build_widgets(self) -> None:
        tk = self.tk
        ttk = self.ttk
        outer = ttk.Frame(self.window, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        library = ttk.LabelFrame(outer, text="Leg motion design", padding=8)
        library.pack(fill=tk.X)
        ttk.Label(library, text="Name:").pack(side=tk.LEFT)
        self.design_combo = ttk.Combobox(
            library,
            textvariable=self.design_name_var,
            values=[design.name for design in self.designs],
            width=28,
        )
        self.design_combo.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 8))
        ttk.Button(library, text="New", command=self.new_design).pack(side=tk.LEFT)
        ttk.Button(library, text="Load", command=self.load_selected_design).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(library, text="Save", command=self.save_current_design).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(library, text="Delete", command=self.delete_selected_design).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        motion = ttk.LabelFrame(outer, text="Cycle and source leg", padding=8)
        motion.pack(fill=tk.X, pady=(8, 0))
        motion.columnconfigure(1, weight=1)
        ttk.Label(motion, text="Frequency", width=16).grid(row=0, column=0, sticky="w")
        tk.Scale(
            motion,
            from_=0.05,
            to=2.00,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            showvalue=False,
            variable=self.frequency_var,
            command=self.frequency_changed,
        ).grid(row=0, column=1, sticky="ew")
        ttk.Label(motion, textvariable=self.frequency_label_var, width=20).grid(
            row=0, column=2, sticky="e"
        )
        ttk.Label(motion, text="Edit / preview leg", width=16).grid(
            row=1, column=0, sticky="w", pady=(6, 0)
        )
        ttk.Combobox(
            motion,
            textvariable=self.source_leg_var,
            values=[self.leg_labels[index] for index in self.active_leg_indices],
            state="readonly",
            width=20,
        ).grid(row=1, column=1, sticky="w", pady=(6, 0))
        manual = ttk.Frame(motion)
        manual.grid(row=1, column=2, sticky="e", pady=(6, 0))
        ttk.Button(manual, text="Torque Off leg", command=self.torque_off_selected_leg).pack(
            side=tk.LEFT
        )
        ttk.Button(manual, text="Capture pose", command=self.capture_current_pose).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(manual, text="Torque On leg", command=self.torque_on_selected_leg).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        recording = ttk.Frame(motion)
        recording.grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Label(recording, text="Continuous teaching", width=16).pack(side=tk.LEFT)
        ttk.Button(
            recording,
            text="Start recording",
            command=self.start_continuous_recording,
        ).pack(side=tk.LEFT)
        ttk.Button(
            recording,
            text="Finish recording",
            command=self.finish_continuous_recording,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            recording,
            text="Cancel recording",
            command=self.cancel_continuous_recording,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(
            recording,
            text="Move the torque-free leg through exactly one cycle.",
        ).pack(side=tk.LEFT, padx=(12, 0))

        middle = ttk.Panedwindow(outer, orient=tk.HORIZONTAL)
        middle.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        editor = ttk.LabelFrame(middle, text="Single-leg keyframes", padding=8)
        transfer = ttk.LabelFrame(middle, text="Transfer phase per leg", padding=8)
        middle.add(editor, weight=3)
        middle.add(transfer, weight=2)

        columns = ("frame", "phase", "duration", "yaw", "lift", "knee")
        self.tree = ttk.Treeview(editor, columns=columns, show="headings", height=9)
        for column, text, width in (
            ("frame", "Frame", 58),
            ("phase", "Phase", 65),
            ("duration", "To next", 75),
            ("yaw", "Yaw", 65),
            ("lift", "Lift", 65),
            ("knee", "Knee", 65),
        ):
            self.tree.heading(column, text=text)
            self.tree.column(column, width=width, anchor=tk.CENTER)
        self.tree.grid(row=0, column=0, columnspan=4, sticky="nsew")
        self.tree.bind("<<TreeviewSelect>>", self.keyframe_selected)
        editor.rowconfigure(0, weight=1)
        editor.columnconfigure(1, weight=1)

        for row, (label, variable, low, high, resolution) in enumerate(
            (
                ("To next (s)", self.duration_var, 0.02, 5.0, 0.01),
                ("Yaw", self.yaw_var, -1.0, 1.0, 0.01),
                ("Lift", self.lift_var, -1.0, 1.0, 0.01),
                ("Knee", self.knee_var, -1.0, 1.0, 0.01),
            ),
            start=1,
        ):
            ttk.Label(editor, text=label, width=12).grid(row=row, column=0, sticky="w")
            tk.Scale(
                editor,
                from_=low,
                to=high,
                resolution=resolution,
                orient=tk.HORIZONTAL,
                variable=variable,
            ).grid(row=row, column=1, columnspan=3, sticky="ew")

        actions = ttk.Frame(editor)
        actions.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(6, 0))
        ttk.Button(actions, text="Add frame after selected", command=self.add_keyframe).pack(
            side=tk.LEFT
        )
        ttk.Button(actions, text="Update frame / time", command=self.update_keyframe).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(actions, text="Delete frame", command=self.delete_keyframe).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        batch = ttk.LabelFrame(editor, text="Batch edit frame range", padding=6)
        batch.grid(row=6, column=0, columnspan=4, sticky="ew", pady=(8, 0))
        ttk.Label(batch, text="Frames").grid(row=0, column=0, sticky="w")
        tk.Spinbox(
            batch,
            from_=1,
            to=9999,
            width=5,
            textvariable=self.batch_start_var,
        ).grid(row=0, column=1, padx=(4, 0))
        ttk.Label(batch, text="to").grid(row=0, column=2, padx=4)
        tk.Spinbox(
            batch,
            from_=1,
            to=9999,
            width=5,
            textvariable=self.batch_end_var,
        ).grid(row=0, column=3)
        ttk.Button(
            batch,
            text="Use selected rows",
            command=self.use_selected_batch_range,
        ).grid(row=0, column=4, padx=(6, 12))
        ttk.Label(batch, text="Column").grid(row=0, column=5)
        ttk.Combobox(
            batch,
            textvariable=self.batch_joint_var,
            values=("Yaw", "Lift", "Knee", "To next"),
            state="readonly",
            width=7,
        ).grid(row=0, column=6, padx=(4, 0))

        ttk.Label(batch, text="Operation").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Combobox(
            batch,
            textvariable=self.batch_operation_var,
            values=("Set", "Add", "Multiply", "Linear ramp"),
            state="readonly",
            width=12,
        ).grid(row=1, column=1, columnspan=2, sticky="w", padx=(4, 0), pady=(6, 0))
        ttk.Label(batch, text="Value / start").grid(
            row=1, column=3, sticky="e", padx=(8, 4), pady=(6, 0)
        )
        ttk.Entry(batch, textvariable=self.batch_value_a_var, width=8).grid(
            row=1, column=4, sticky="w", pady=(6, 0)
        )
        ttk.Label(batch, text="Ramp end").grid(
            row=1, column=5, sticky="e", padx=(8, 4), pady=(6, 0)
        )
        ttk.Entry(batch, textvariable=self.batch_value_b_var, width=8).grid(
            row=1, column=6, sticky="w", pady=(6, 0)
        )
        ttk.Button(batch, text="Apply batch edit", command=self.apply_batch_edit).grid(
            row=1, column=7, padx=(10, 0), pady=(6, 0)
        )
        ttk.Button(
            batch,
            text="Delete frame range",
            command=self.delete_batch_range,
        ).grid(row=0, column=7, padx=(10, 0))

        transfer.columnconfigure(1, weight=1)
        for row, leg_index in enumerate(self.active_leg_indices):
            ttk.Label(transfer, text=self.leg_labels[leg_index], width=16).grid(
                row=row, column=0, sticky="w"
            )
            variable = tk.DoubleVar()
            value_var = tk.StringVar()
            self.phase_vars[leg_index] = variable
            self.phase_value_vars[leg_index] = value_var
            tk.Scale(
                transfer,
                from_=0.0,
                to=0.99,
                resolution=0.01,
                showvalue=False,
                orient=tk.HORIZONTAL,
                variable=variable,
                command=lambda _value, index=leg_index: self.phase_offset_changed(index),
            ).grid(row=row, column=1, sticky="ew")
            ttk.Label(transfer, textvariable=value_var, width=14).grid(
                row=row, column=2, sticky="e"
            )

        phase_presets = ttk.Frame(transfer)
        phase_presets.grid(
            row=len(self.active_leg_indices),
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(8, 0),
        )
        ttk.Button(
            phase_presets,
            text="Tripod",
            command=lambda: self.apply_phase_preset("tripod"),
        ).pack(side=tk.LEFT)
        ttk.Button(
            phase_presets,
            text="Wave",
            command=lambda: self.apply_phase_preset("wave"),
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            phase_presets,
            text="All together",
            command=lambda: self.apply_phase_preset("together"),
        ).pack(side=tk.LEFT, padx=(6, 0))

        run = ttk.Frame(outer)
        run.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(run, text="Preview selected leg", command=self.preview_selected_leg).pack(
            side=tk.LEFT
        )
        ttk.Button(run, text="Run all legs", command=self.run_all_legs).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(run, text="Stop / Neutral", command=self.stop_motion).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Label(run, textvariable=self.status_var).pack(side=tk.LEFT, padx=(12, 0))

    def frequency_changed(self, _value: str | None = None) -> None:
        requested_frequency = self.frequency_var.get()
        if self.frequency_callback_ready and self.current_design.keyframes:
            current_duration = sum(
                keyframe.duration_seconds
                for keyframe in self.current_design.keyframes
            )
            target_duration = 1.0 / requested_frequency
            scale = target_duration / current_duration
            for keyframe in self.current_design.keyframes:
                keyframe.duration_seconds *= scale
            update_leg_motion_timing(self.current_design)
            self.refresh_keyframes(self.selected_keyframe_index())
        else:
            self.current_design.frequency_hz = requested_frequency
        self.frequency_label_var.set(
            f"{self.current_design.frequency_hz:.2f} Hz / "
            f"{1.0 / self.current_design.frequency_hz:.2f} s"
        )

    def sync_frequency_from_frame_times(self) -> None:
        update_leg_motion_timing(self.current_design)
        self.frequency_callback_ready = False
        self.frequency_var.set(self.current_design.frequency_hz)
        self.frequency_label_var.set(
            f"{self.current_design.frequency_hz:.2f} Hz / "
            f"{1.0 / self.current_design.frequency_hz:.2f} s"
        )
        self.frequency_callback_ready = True

    def refresh_design_names(self) -> None:
        if self.design_combo is not None:
            self.design_combo.configure(values=[design.name for design in self.designs])

    def load_design_into_widgets(self) -> None:
        self.design_name_var.set(self.current_design.name)
        self.sync_frequency_from_frame_times()
        self.batch_start_var.set(1)
        self.batch_end_var.set(len(self.current_design.keyframes))
        for leg_index, variable in self.phase_vars.items():
            variable.set(self.current_design.phase_offsets[leg_index])
            self.update_phase_label(leg_index)
        self.refresh_keyframes()
        if self.design_load_error:
            self.status_var.set(f"File error: {self.design_load_error}")
        else:
            self.status_var.set("Edit one cycle, then preview one leg before running all legs.")

    def refresh_keyframes(self, selected: int | None = None) -> None:
        if self.tree is None:
            return
        self.tree.delete(*self.tree.get_children())
        for index, keyframe in enumerate(self.current_design.keyframes):
            self.tree.insert(
                "",
                self.tk.END,
                iid=str(index),
                values=(
                    index + 1,
                    f"{keyframe.phase:.2f}",
                    f"{keyframe.duration_seconds:.2f} s",
                    f"{keyframe.yaw:+.2f}",
                    f"{keyframe.lift:+.2f}",
                    f"{keyframe.knee:+.2f}",
                ),
            )
        if selected is not None and 0 <= selected < len(self.current_design.keyframes):
            self.tree.selection_set(str(selected))
            self.tree.focus(str(selected))

    def selected_keyframe_index(self) -> int | None:
        if self.tree is None or not self.tree.selection():
            return None
        return int(self.tree.selection()[0])

    def keyframe_from_editor(self) -> LegMotionKeyframe:
        return LegMotionKeyframe(
            phase=0.0,
            yaw=clip_unit(self.yaw_var.get()),
            lift=clip_unit(self.lift_var.get()),
            knee=clip_unit(self.knee_var.get()),
            duration_seconds=max(0.02, min(20.0, self.duration_var.get())),
        )

    def keyframe_selected(self, _event=None) -> None:
        index = self.selected_keyframe_index()
        if index is None:
            return
        keyframe = self.current_design.keyframes[index]
        self.duration_var.set(keyframe.duration_seconds)
        self.yaw_var.set(keyframe.yaw)
        self.lift_var.set(keyframe.lift)
        self.knee_var.set(keyframe.knee)

    def add_keyframe(self) -> None:
        try:
            keyframe = self.keyframe_from_editor()
            selected = self.selected_keyframe_index()
            insert_at = (
                len(self.current_design.keyframes)
                if selected is None
                else selected + 1
            )
            self.current_design.keyframes.insert(insert_at, keyframe)
            self.sync_frequency_from_frame_times()
            validate_leg_motion_design(self.current_design)
            self.refresh_keyframes(insert_at)
            self.status_var.set(
                f"Added frame {insert_at + 1}; "
                f"time to its next frame is {keyframe.duration_seconds:.2f} s."
            )
        except Exception as exc:
            self.messagebox.showerror("Add frame", str(exc))

    def update_keyframe(self) -> None:
        index = self.selected_keyframe_index()
        if index is None:
            self.messagebox.showerror("Update keyframe", "Select a keyframe first.")
            return
        previous = self.current_design.keyframes[index]
        self.current_design.keyframes[index] = self.keyframe_from_editor()
        try:
            self.sync_frequency_from_frame_times()
            validate_leg_motion_design(self.current_design)
            self.refresh_keyframes(index)
            self.status_var.set(
                f"Updated frame {index + 1} and recalculated the cycle timing."
            )
        except Exception as exc:
            self.current_design.keyframes[index] = previous
            self.sync_frequency_from_frame_times()
            self.messagebox.showerror("Update frame", str(exc))

    def delete_keyframe(self) -> None:
        index = self.selected_keyframe_index()
        if index is None:
            return
        if len(self.current_design.keyframes) <= 2:
            self.messagebox.showerror(
                "Delete keyframe",
                "At least two keyframes are required.",
            )
            return
        removed = self.current_design.keyframes.pop(index)
        self.sync_frequency_from_frame_times()
        self.refresh_keyframes()
        self.status_var.set(
            f"Deleted frame {index + 1} "
            f"({removed.duration_seconds:.2f} s to next)."
        )

    def use_selected_batch_range(self) -> None:
        if self.tree is None:
            return
        selected = [int(item) for item in self.tree.selection()]
        if not selected:
            self.messagebox.showerror(
                "Batch edit",
                "Select one or more frame rows first.",
            )
            return
        self.batch_start_var.set(min(selected) + 1)
        self.batch_end_var.set(max(selected) + 1)
        self.status_var.set(
            f"Batch range set to frames {min(selected) + 1}–{max(selected) + 1}."
        )

    def apply_batch_edit(self) -> None:
        if self.recording:
            self.messagebox.showerror(
                "Batch edit",
                "Finish or cancel continuous recording first.",
            )
            return
        if self.controller.motion_enabled:
            self.messagebox.showerror(
                "Batch edit",
                "Stop the motion before applying a batch edit.",
            )
            return
        previous = copy.deepcopy(self.current_design.keyframes)
        try:
            start_index = int(self.batch_start_var.get()) - 1
            end_index = int(self.batch_end_var.get()) - 1
            operation = {
                "Set": "set",
                "Add": "add",
                "Multiply": "multiply",
                "Linear ramp": "ramp",
            }[self.batch_operation_var.get()]
            value_a = float(self.batch_value_a_var.get())
            value_b = (
                float(self.batch_value_b_var.get())
                if operation == "ramp"
                else None
            )
            joint = {
                "Yaw": "yaw",
                "Lift": "lift",
                "Knee": "knee",
                "To next": "duration_seconds",
            }[self.batch_joint_var.get()]
            batch_edit_leg_motion_keyframes(
                self.current_design,
                start_index,
                end_index,
                joint,
                operation,
                value_a,
                value_b,
            )
            validate_leg_motion_design(self.current_design)
            if joint == "duration_seconds":
                self.sync_frequency_from_frame_times()
            self.refresh_keyframes()
            if self.tree is not None:
                self.tree.selection_set(
                    *(str(index) for index in range(start_index, end_index + 1))
                )
            operation_text = self.batch_operation_var.get()
            self.status_var.set(
                f"{operation_text} applied to {self.batch_joint_var.get()} "
                f"in frames {start_index + 1}–{end_index + 1}."
            )
        except Exception as exc:
            self.current_design.keyframes = previous
            self.refresh_keyframes()
            self.messagebox.showerror("Batch edit", str(exc))

    def delete_batch_range(self) -> None:
        if self.recording:
            self.messagebox.showerror(
                "Delete frame range",
                "Finish or cancel continuous recording first.",
            )
            return
        if self.controller.motion_enabled:
            self.messagebox.showerror(
                "Delete frame range",
                "Stop the motion before deleting frames.",
            )
            return
        previous = copy.deepcopy(self.current_design.keyframes)
        try:
            start_index = int(self.batch_start_var.get()) - 1
            end_index = int(self.batch_end_var.get()) - 1
            if (
                start_index < 0
                or end_index >= len(self.current_design.keyframes)
                or start_index > end_index
            ):
                raise ValueError("Select a valid frame range.")
            if not self.messagebox.askyesno(
                "Delete frame range",
                f"Delete frames {start_index + 1}–{end_index + 1}?",
            ):
                return
            deleted = delete_leg_motion_keyframe_range(
                self.current_design,
                start_index,
                end_index,
            )
            validate_leg_motion_design(self.current_design)
            self.sync_frequency_from_frame_times()
            self.refresh_keyframes()
            remaining = len(self.current_design.keyframes)
            next_frame = min(start_index + 1, remaining)
            self.batch_start_var.set(next_frame)
            self.batch_end_var.set(next_frame)
            if self.tree is not None:
                self.tree.selection_set(str(next_frame - 1))
            self.status_var.set(
                f"Deleted {deleted} frames. {remaining} frames remain."
            )
        except Exception as exc:
            self.current_design.keyframes = previous
            self.sync_frequency_from_frame_times()
            self.refresh_keyframes()
            self.messagebox.showerror("Delete frame range", str(exc))

    def phase_offset_changed(self, leg_index: int) -> None:
        value = self.phase_vars[leg_index].get() % 1.0
        self.current_design.phase_offsets[leg_index] = value
        self.update_phase_label(leg_index)

    def update_phase_label(self, leg_index: int) -> None:
        value = self.phase_vars[leg_index].get() % 1.0
        self.phase_value_vars[leg_index].set(f"{value:.2f} / {value * 360:.0f}°")

    def apply_phase_preset(self, name: str) -> None:
        if name == "tripod":
            offsets = [0.00, 0.50, 0.50, 0.00, 0.00, 0.50, 0.50, 0.00]
        elif name == "wave":
            offsets = list(self.current_design.phase_offsets)
            count = len(self.active_leg_indices)
            for order, leg_index in enumerate(self.active_leg_indices):
                offsets[leg_index] = order / count
        elif name == "together":
            offsets = [0.0] * LEG_COUNT
        else:
            raise ValueError(f"unknown phase preset: {name}")
        self.current_design.phase_offsets = offsets
        for leg_index, variable in self.phase_vars.items():
            variable.set(offsets[leg_index])
            self.update_phase_label(leg_index)
        self.status_var.set(f"Applied {name} phase layout; sliders remain editable.")

    def torque_off_selected_leg(self) -> None:
        try:
            if self.recording:
                raise RuntimeError("Finish or cancel the current recording first.")
            leg_index = self.selected_leg_index()
            if (
                self.manual_torque_off_leg_index is not None
                and self.manual_torque_off_leg_index != leg_index
            ):
                raise RuntimeError(
                    "Another leg is already torque-free. Turn its torque ON first."
                )
            self.stop_motion()
            self.controller.set_leg_torque(leg_index, False)
            self.manual_torque_off = True
            self.manual_torque_off_leg_index = leg_index
            self.status_var.set("Selected leg torque is OFF. Support the robot before posing.")
        except Exception as exc:
            self.messagebox.showerror("Torque Off leg", str(exc))

    def torque_on_selected_leg(self) -> None:
        try:
            if self.recording:
                raise RuntimeError("Finish or cancel the current recording first.")
            leg_index = (
                self.manual_torque_off_leg_index
                if self.manual_torque_off_leg_index is not None
                else self.selected_leg_index()
            )
            self.controller.enable_leg_torque_at_current_position(
                leg_index
            )
            self.manual_torque_off = False
            self.manual_torque_off_leg_index = None
            self.status_var.set(
                "Selected leg torque is ON; goals were synchronized to the current pose."
            )
        except Exception as exc:
            self.messagebox.showerror("Torque On leg", str(exc))

    def start_continuous_recording(self) -> None:
        try:
            if self.recording:
                raise RuntimeError("Recording is already running.")
            leg_index = self.selected_leg_index()
            if (
                self.manual_torque_off_leg_index is not None
                and self.manual_torque_off_leg_index != leg_index
            ):
                raise RuntimeError(
                    "Another leg is torque-free. Turn its torque ON before recording."
                )
            self.stop_motion()
            if self.manual_torque_off_leg_index is None:
                self.controller.set_leg_torque(leg_index, False)
                self.manual_torque_off = True
                self.manual_torque_off_leg_index = leg_index
            self.recording = True
            self.recording_leg_index = leg_index
            self.recording_started_at = time.monotonic()
            self.recording_samples = []
            self.record_continuous_sample()
        except Exception as exc:
            self.messagebox.showerror("Start continuous teaching", str(exc))

    def record_continuous_sample(self) -> None:
        if not self.recording or self.recording_leg_index is None or self.closed:
            return
        try:
            elapsed = time.monotonic() - self.recording_started_at
            pose = self.controller.read_leg_semantic_pose(self.recording_leg_index)
            self.recording_samples.append((elapsed, pose))
            self.status_var.set(
                f"Recording one cycle: {elapsed:.2f} s, "
                f"{len(self.recording_samples)} samples"
            )
            self.recording_after = self.window.after(
                50,
                self.record_continuous_sample,
            )
        except Exception as exc:
            self.cancel_continuous_recording(show_status=False)
            self.messagebox.showerror("Continuous teaching", str(exc))

    def stop_recording_timer(self) -> None:
        if self.recording_after is not None:
            try:
                self.window.after_cancel(self.recording_after)
            except Exception:
                pass
            self.recording_after = None

    def finish_continuous_recording(self) -> None:
        if not self.recording:
            self.messagebox.showerror(
                "Finish continuous teaching",
                "Start recording first.",
            )
            return
        previous_keyframes = copy.deepcopy(self.current_design.keyframes)
        previous_frequency = self.current_design.frequency_hz
        try:
            if self.recording_leg_index is None:
                raise RuntimeError("Recording leg is unavailable.")
            duration = time.monotonic() - self.recording_started_at
            final_pose = self.controller.read_leg_semantic_pose(
                self.recording_leg_index
            )
            self.recording_samples.append((duration, final_pose))
            self.recording = False
            self.stop_recording_timer()
            if duration < 0.5 or len(self.recording_samples) < 3:
                raise ValueError(
                    "Record at least 0.5 seconds so one cycle can be reconstructed."
                )

            recorded_frames = self.recording_samples[:-1]
            keyframes = []
            for index, (elapsed, pose) in enumerate(recorded_frames):
                next_elapsed = self.recording_samples[index + 1][0]
                keyframes.append(
                    LegMotionKeyframe(
                        phase=0.0,
                        yaw=pose[0],
                        lift=pose[1],
                        knee=pose[2],
                        duration_seconds=max(0.02, next_elapsed - elapsed),
                    )
                )
            self.current_design.keyframes = keyframes
            self.sync_frequency_from_frame_times()
            validate_leg_motion_design(self.current_design)
            self.refresh_keyframes()
            self.recording_leg_index = None
            self.status_var.set(
                f"Captured your complete {duration:.2f} s cycle as "
                f"{len(keyframes)} keyframes. Turn torque ON, then preview it."
            )
        except Exception as exc:
            self.current_design.keyframes = previous_keyframes
            self.current_design.frequency_hz = previous_frequency
            self.sync_frequency_from_frame_times()
            self.refresh_keyframes()
            self.recording = False
            self.recording_leg_index = None
            self.stop_recording_timer()
            self.messagebox.showerror("Finish continuous teaching", str(exc))

    def cancel_continuous_recording(self, show_status: bool = True) -> None:
        self.recording = False
        self.recording_leg_index = None
        self.recording_samples = []
        self.stop_recording_timer()
        if show_status:
            self.status_var.set(
                "Recording cancelled; the previous keyframes were kept."
            )

    def capture_current_pose(self) -> None:
        try:
            yaw, lift, knee = self.controller.read_leg_semantic_pose(
                self.selected_leg_index()
            )
            self.yaw_var.set(yaw)
            self.lift_var.set(lift)
            self.knee_var.set(knee)
            self.status_var.set("Captured current pose into the keyframe editor.")
        except Exception as exc:
            self.messagebox.showerror("Capture pose", str(exc))

    def require_preview_ready(self) -> None:
        if self.recording:
            raise RuntimeError("Finish or cancel the recording before previewing.")
        if self.manual_torque_off:
            raise RuntimeError("Turn the selected leg torque ON before previewing.")
        self.frequency_changed()
        validate_leg_motion_design(self.current_design)

    def preview_selected_leg(self) -> None:
        try:
            self.require_preview_ready()
            self.owner.cancel_initial_position()
            leg_index = self.selected_leg_index()
            self.controller.start_leg_motion(self.current_design, leg_index)
            self.status_var.set(f"Previewing {self.leg_labels[leg_index]} only.")
            self.owner.status_var.set("Leg template preview running")
        except Exception as exc:
            self.messagebox.showerror("Preview selected leg", str(exc))

    def run_all_legs(self) -> None:
        try:
            self.require_preview_ready()
            self.owner.cancel_initial_position()
            self.controller.start_leg_motion(self.current_design)
            self.status_var.set("Running the template on all enabled legs.")
            self.owner.status_var.set("Transferred leg gait running")
        except Exception as exc:
            self.messagebox.showerror("Run all legs", str(exc))

    def stop_motion(self) -> None:
        self.controller.stop_motion_neutral()
        self.status_var.set("Stopped at configured neutral positions.")
        self.owner.status_var.set("Stopped at configured neutral positions")

    def new_design(self) -> None:
        self.cancel_continuous_recording(show_status=False)
        self.stop_motion()
        self.current_design = default_leg_motion_design("New leg motion")
        self.load_design_into_widgets()

    def selected_saved_design(self) -> LegMotionDesign:
        name = self.design_name_var.get().strip()
        for design in self.designs:
            if design.name == name:
                return design
        raise ValueError(f"Design not found: {name}")

    def load_selected_design(self) -> None:
        try:
            self.cancel_continuous_recording(show_status=False)
            self.stop_motion()
            self.current_design = copy.deepcopy(self.selected_saved_design())
            self.load_design_into_widgets()
            self.status_var.set(f"Loaded '{self.current_design.name}'.")
        except Exception as exc:
            self.messagebox.showerror("Load leg motion design", str(exc))

    def save_current_design(self) -> None:
        try:
            if self.design_load_error:
                raise RuntimeError(
                    f"Cannot overwrite an unreadable design file: {self.design_load_error}"
                )
            self.frequency_changed()
            name = self.design_name_var.get().strip()
            if not name:
                raise ValueError("Enter a design name.")
            self.current_design.name = name
            validate_leg_motion_design(self.current_design)
            existing_index = next(
                (
                    index
                    for index, design in enumerate(self.designs)
                    if design.name == name
                ),
                None,
            )
            if existing_index is not None:
                if not self.messagebox.askyesno(
                    "Overwrite leg motion design",
                    f"Overwrite '{name}'?",
                ):
                    return
                self.designs[existing_index] = copy.deepcopy(self.current_design)
                action = "Updated"
            else:
                self.designs.append(copy.deepcopy(self.current_design))
                action = "Saved"
            save_leg_motion_designs(self.design_path, self.designs)
            self.refresh_design_names()
            self.status_var.set(f"{action} '{name}' in {self.design_path}")
        except Exception as exc:
            self.messagebox.showerror("Save leg motion design", str(exc))

    def delete_selected_design(self) -> None:
        try:
            design = self.selected_saved_design()
            if not self.messagebox.askyesno(
                "Delete leg motion design",
                f"Delete '{design.name}'?",
            ):
                return
            self.designs = [item for item in self.designs if item.name != design.name]
            save_leg_motion_designs(self.design_path, self.designs)
            self.refresh_design_names()
            self.current_design = default_leg_motion_design()
            self.load_design_into_widgets()
            self.status_var.set(f"Deleted '{design.name}'.")
        except Exception as exc:
            self.messagebox.showerror("Delete leg motion design", str(exc))

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.cancel_continuous_recording(show_status=False)
        try:
            self.controller.motion_enabled = False
            if self.manual_torque_off:
                self.controller.set_torque(False)
        except Exception:
            pass
        if self.window.winfo_exists():
            self.window.destroy()


class MotionTeachingGui:
    def __init__(
        self,
        device: str,
        baudrate: int,
        protocol: float,
        config: RobotConfig,
        input_path: str,
        output_path: str,
    ) -> None:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk

        self.tk = tk
        self.ttk = ttk
        self.filedialog = filedialog
        self.messagebox = messagebox
        self.root = tk.Tk()
        self.root.title("Hanachan Motion Teaching")
        self.root.geometry("960x720")
        self.root.minsize(780, 580)

        self.bus = DynamixelBus(device, baudrate, protocol)
        self.config = config
        self.servo_ids = [config.ids[index] for index in config.enabled_indices]
        self.sequence = MotionSequence(servo_ids=list(self.servo_ids), frames=[])
        self.input_path = input_path
        self.output_path = output_path
        self.status_var = tk.StringVar(value="DYNAMIXELバスを開いています...")
        self.name_var = tk.StringVar()
        self.duration_var = tk.StringVar(value="1.0")
        self.detail_var = tk.StringVar(value="姿勢を選択すると指令値を表示します。")
        self.tree = None
        self.playback_after: str | None = None
        self.playback_index = 0
        self.playback_start_time = 0.0
        self.playback_start_positions: dict[int, int] = {}
        self.playing = False
        self.torque_enabled = False
        self.closed = False

    def run(self) -> None:
        try:
            self.bus.__enter__()
            found = self.bus.scan(sorted(set(self.servo_ids)))
            found_ids = {dxl_id for dxl_id, _model in found}
            missing = sorted(set(self.servo_ids) - found_ids)
            if missing:
                raise RuntimeError(f"設定されたDYNAMIXEL IDが見つかりません: {missing}")

            self.set_torque_off()
            if self.input_path:
                self.load_sequence(self.input_path)
            self.build_widgets(found_ids)
            self.root.protocol("WM_DELETE_WINDOW", self.close)
            self.root.mainloop()
        except Exception as exc:
            self.messagebox.showerror("Hanachan Motion Teaching", str(exc))
            try:
                self.set_torque_off()
            except Exception:
                pass
            try:
                self.bus.__exit__(None, None, None)
            except Exception:
                pass

    def build_widgets(self, found_ids: set[int]) -> None:
        tk = self.tk
        ttk = self.ttk
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(outer)
        header.pack(fill=tk.X)
        ttk.Label(header, text="モーション教示", font=("", 18, "bold")).pack(side=tk.LEFT)
        ttk.Label(
            header,
            text=f"対象ID: {', '.join(map(str, self.servo_ids))}",
        ).pack(side=tk.RIGHT)

        safety = ttk.LabelFrame(outer, text="安全状態", padding=10)
        safety.pack(fill=tk.X, pady=(12, 8))
        ttk.Label(
            safety,
            text="通常はトルクOFFです。関節を手で動かしてから姿勢を取り込んでください。",
        ).pack(side=tk.LEFT)
        ttk.Button(safety, text="停止 / トルクOFF", command=self.stop_playback).pack(side=tk.RIGHT)

        editor = ttk.LabelFrame(outer, text="姿勢の取り込み", padding=10)
        editor.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(editor, text="名前").grid(row=0, column=0, sticky="w")
        ttk.Entry(editor, textvariable=self.name_var, width=24).grid(
            row=0, column=1, sticky="ew", padx=(6, 14)
        )
        ttk.Label(editor, text="移動時間 [秒]").grid(row=0, column=2, sticky="w")
        ttk.Entry(editor, textvariable=self.duration_var, width=10).grid(
            row=0, column=3, sticky="w", padx=(6, 14)
        )
        ttk.Button(editor, text="現在姿勢を追加", command=self.capture_pose).grid(
            row=0, column=4, padx=(0, 6)
        )
        ttk.Button(editor, text="選択姿勢を再取込", command=self.update_selected_pose).grid(
            row=0, column=5
        )
        editor.columnconfigure(1, weight=1)

        sequence_frame = ttk.LabelFrame(outer, text="動作シーケンス", padding=8)
        sequence_frame.pack(fill=tk.BOTH, expand=True)
        columns = ("number", "name", "duration")
        self.tree = ttk.Treeview(sequence_frame, columns=columns, show="headings", height=12)
        self.tree.heading("number", text="#")
        self.tree.heading("name", text="名前")
        self.tree.heading("duration", text="移動時間 [秒]")
        self.tree.column("number", width=55, anchor=tk.CENTER, stretch=False)
        self.tree.column("name", width=360)
        self.tree.column("duration", width=130, anchor=tk.E, stretch=False)
        scrollbar = ttk.Scrollbar(sequence_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.grid(row=0, column=0, columnspan=8, sticky="nsew")
        scrollbar.grid(row=0, column=8, sticky="ns")
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)
        sequence_frame.rowconfigure(0, weight=1)
        sequence_frame.columnconfigure(0, weight=1)

        ttk.Button(sequence_frame, text="名前・時間を反映", command=self.apply_selected_metadata).grid(
            row=1, column=0, sticky="w", pady=(8, 0)
        )
        ttk.Button(sequence_frame, text="上へ", command=lambda: self.move_selected(-1)).grid(
            row=1, column=1, padx=(6, 0), pady=(8, 0)
        )
        ttk.Button(sequence_frame, text="下へ", command=lambda: self.move_selected(1)).grid(
            row=1, column=2, padx=(6, 0), pady=(8, 0)
        )
        ttk.Button(sequence_frame, text="削除", command=self.delete_selected).grid(
            row=1, column=3, padx=(6, 0), pady=(8, 0)
        )
        ttk.Button(sequence_frame, text="シーケンス再生", command=self.play_sequence).grid(
            row=1, column=6, padx=(18, 0), pady=(8, 0)
        )
        ttk.Button(sequence_frame, text="停止", command=self.stop_playback).grid(
            row=1, column=7, padx=(6, 0), pady=(8, 0)
        )

        details = ttk.LabelFrame(outer, text="選択姿勢の指令値", padding=8)
        details.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(details, textvariable=self.detail_var, wraplength=900, justify=tk.LEFT).pack(
            fill=tk.X
        )

        footer = ttk.Frame(outer)
        footer.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(footer, textvariable=self.status_var).pack(side=tk.LEFT)
        ttk.Button(footer, text="読込...", command=self.load_from_dialog).pack(side=tk.RIGHT)
        ttk.Button(footer, text="別名で保存...", command=self.save_as).pack(
            side=tk.RIGHT, padx=(0, 6)
        )
        ttk.Button(footer, text="保存", command=self.save_sequence).pack(
            side=tk.RIGHT, padx=(0, 6)
        )
        self.refresh_tree()
        self.status_var.set("トルクOFF。関節を手で動かして現在姿勢を追加してください。")

    def selected_index(self) -> int | None:
        if self.tree is None:
            return None
        selection = self.tree.selection()
        if not selection:
            return None
        return int(selection[0])

    def parse_duration(self) -> float:
        try:
            duration = float(self.duration_var.get())
        except ValueError as exc:
            raise ValueError("移動時間は数値で入力してください") from exc
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("移動時間は0より大きい値にしてください")
        return duration

    def require_teaching_state(self) -> None:
        if self.playing or self.torque_enabled:
            raise RuntimeError("先に停止してトルクをOFFにしてください")

    def read_current_positions(self) -> dict[int, int]:
        positions = {dxl_id: self.bus.read_position(dxl_id) for dxl_id in self.servo_ids}
        for dxl_id, position in positions.items():
            if position < DEFAULT_POSITION_MIN or position > DEFAULT_POSITION_MAX:
                raise RuntimeError(f"ID {dxl_id} の現在位置が範囲外です: {position}")
        return positions

    def capture_pose(self) -> None:
        try:
            self.require_teaching_state()
            duration = self.parse_duration()
            positions = self.read_current_positions()
            name = self.name_var.get().strip() or f"姿勢 {len(self.sequence.frames) + 1}"
            self.sequence.frames.append(
                MotionFrame(name=name, duration_seconds=duration, positions=positions)
            )
            selected = len(self.sequence.frames) - 1
            self.refresh_tree(selected)
            self.name_var.set("")
            self.status_var.set(f"{name} を取り込みました（トルクOFF）")
        except Exception as exc:
            self.messagebox.showerror("姿勢の取り込み", str(exc))

    def update_selected_pose(self) -> None:
        try:
            self.require_teaching_state()
            index = self.selected_index()
            if index is None:
                raise ValueError("再取り込みする姿勢を選択してください")
            frame = self.sequence.frames[index]
            frame.positions = self.read_current_positions()
            self.refresh_tree(index)
            self.status_var.set(f"{frame.name} の指令値を現在姿勢で更新しました")
        except Exception as exc:
            self.messagebox.showerror("姿勢の再取り込み", str(exc))

    def apply_selected_metadata(self) -> None:
        try:
            self.require_teaching_state()
            index = self.selected_index()
            if index is None:
                raise ValueError("編集する姿勢を選択してください")
            name = self.name_var.get().strip()
            if not name:
                raise ValueError("名前を入力してください")
            frame = self.sequence.frames[index]
            frame.name = name
            frame.duration_seconds = self.parse_duration()
            self.refresh_tree(index)
            self.status_var.set(f"{name} の名前と移動時間を更新しました")
        except Exception as exc:
            self.messagebox.showerror("姿勢情報の編集", str(exc))

    def delete_selected(self) -> None:
        try:
            self.require_teaching_state()
            index = self.selected_index()
            if index is None:
                return
            frame = self.sequence.frames.pop(index)
            next_index = min(index, len(self.sequence.frames) - 1) if self.sequence.frames else None
            self.refresh_tree(next_index)
            self.status_var.set(f"{frame.name} を削除しました")
        except Exception as exc:
            self.messagebox.showerror("姿勢の削除", str(exc))

    def move_selected(self, offset: int) -> None:
        try:
            self.require_teaching_state()
            index = self.selected_index()
            if index is None:
                return
            destination = index + offset
            if destination < 0 or destination >= len(self.sequence.frames):
                return
            self.sequence.frames[index], self.sequence.frames[destination] = (
                self.sequence.frames[destination],
                self.sequence.frames[index],
            )
            self.refresh_tree(destination)
        except Exception as exc:
            self.messagebox.showerror("姿勢の並べ替え", str(exc))

    def refresh_tree(self, selected: int | None = None) -> None:
        if self.tree is None:
            return
        self.tree.delete(*self.tree.get_children())
        for index, frame in enumerate(self.sequence.frames):
            self.tree.insert(
                "",
                self.tk.END,
                iid=str(index),
                values=(index + 1, frame.name, f"{frame.duration_seconds:.3f}"),
            )
        if selected is not None and 0 <= selected < len(self.sequence.frames):
            self.tree.selection_set(str(selected))
            self.tree.focus(str(selected))
            self.tree.see(str(selected))
            self.show_frame_details(selected)
        elif not self.sequence.frames:
            self.detail_var.set("姿勢を選択すると指令値を表示します。")

    def on_tree_select(self, _event=None) -> None:
        index = self.selected_index()
        if index is None:
            return
        frame = self.sequence.frames[index]
        self.name_var.set(frame.name)
        self.duration_var.set(f"{frame.duration_seconds:g}")
        self.show_frame_details(index)

    def show_frame_details(self, index: int) -> None:
        frame = self.sequence.frames[index]
        values = "  ".join(
            f"ID {dxl_id}: {frame.positions[dxl_id]}" for dxl_id in self.servo_ids
        )
        self.detail_var.set(values)

    def save_sequence(self) -> None:
        try:
            save_motion_sequence(self.output_path, self.sequence)
            self.status_var.set(f"保存しました: {self.output_path}")
        except Exception as exc:
            self.messagebox.showerror("シーケンス保存", str(exc))

    def save_as(self) -> None:
        path = self.filedialog.asksaveasfilename(
            title="モーションシーケンスを保存",
            defaultextension=".json",
            filetypes=(("JSON", "*.json"), ("All files", "*.*")),
            initialfile=self.output_path,
        )
        if not path:
            return
        self.output_path = path
        self.save_sequence()

    def load_from_dialog(self) -> None:
        try:
            self.require_teaching_state()
        except Exception as exc:
            self.messagebox.showerror("シーケンス読込", str(exc))
            return
        path = self.filedialog.askopenfilename(
            title="モーションシーケンスを開く",
            filetypes=(("JSON", "*.json"), ("All files", "*.*")),
        )
        if path:
            try:
                self.load_sequence(path)
                self.refresh_tree(0 if self.sequence.frames else None)
                self.status_var.set(f"読み込みました: {path}")
            except Exception as exc:
                self.messagebox.showerror("シーケンス読込", str(exc))

    def load_sequence(self, path: str) -> None:
        sequence = load_motion_sequence(path)
        validate_motion_sequence(sequence, expected_ids=self.servo_ids)
        self.sequence = sequence

    def set_torque_off(self) -> None:
        first_error: Exception | None = None
        for dxl_id in self.servo_ids:
            try:
                self.bus.set_torque(dxl_id, False)
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        self.torque_enabled = False
        if first_error is not None:
            raise first_error

    def play_sequence(self) -> None:
        if not self.sequence.frames:
            self.messagebox.showinfo("シーケンス再生", "姿勢を1つ以上取り込んでください")
            return
        if self.playing:
            return
        if not self.messagebox.askyesno(
            "シーケンス再生",
            "再生中だけサーボのトルクをONにします。\n"
            "ロボットの周囲が安全で、非常停止できることを確認しましたか？",
        ):
            return

        try:
            validate_motion_sequence(self.sequence, expected_ids=self.servo_ids)
            current = self.read_current_positions()
            self.set_torque_off()
            for dxl_id in self.servo_ids:
                self.bus.set_position_mode(dxl_id)
            self.bus.sync_move_to(current)
            for dxl_id in self.servo_ids:
                self.bus.set_torque(dxl_id, True)
            self.torque_enabled = True
            self.playing = True
            self.playback_index = 0
            self.playback_start_positions = current
            self.start_playback_frame()
        except Exception as exc:
            self.playing = False
            try:
                self.set_torque_off()
            except Exception:
                pass
            self.messagebox.showerror("シーケンス再生", str(exc))

    def start_playback_frame(self) -> None:
        if not self.playing:
            return
        if self.playback_index >= len(self.sequence.frames):
            self.finish_playback("再生完了。トルクOFFに戻しました。")
            return
        frame = self.sequence.frames[self.playback_index]
        self.playback_start_time = time.monotonic()
        self.status_var.set(
            f"再生中 {self.playback_index + 1}/{len(self.sequence.frames)}: {frame.name}"
        )
        self.playback_tick()

    def playback_tick(self) -> None:
        if not self.playing or self.closed:
            return
        self.playback_after = None
        frame = self.sequence.frames[self.playback_index]
        progress = (time.monotonic() - self.playback_start_time) / frame.duration_seconds
        try:
            targets = interpolate_positions(
                self.playback_start_positions,
                frame.positions,
                progress,
            )
            self.bus.sync_move_to(targets)
            if progress >= 1.0:
                self.playback_start_positions = dict(frame.positions)
                self.playback_index += 1
                self.start_playback_frame()
                return
            self.playback_after = self.root.after(
                round(CONTROL_PERIOD_SECONDS * 1000),
                self.playback_tick,
            )
        except Exception as exc:
            self.finish_playback(f"再生エラー: {exc}")

    def finish_playback(self, status: str) -> None:
        self.playing = False
        if self.playback_after is not None:
            self.root.after_cancel(self.playback_after)
            self.playback_after = None
        try:
            self.set_torque_off()
        except Exception as exc:
            status = f"{status} / トルクOFFエラー: {exc}"
        self.status_var.set(status)

    def stop_playback(self) -> None:
        self.finish_playback("停止しました。トルクOFFです。")

    def close(self) -> None:
        self.closed = True
        self.playing = False
        if self.playback_after is not None:
            self.root.after_cancel(self.playback_after)
            self.playback_after = None
        try:
            self.set_torque_off()
        except Exception:
            pass
        self.bus.__exit__(None, None, None)
        self.root.destroy()


class DynamixelGui:
    def __init__(
        self,
        device: str,
        baudrate: int,
        protocol: float,
        ids: list[int],
        position_min: int,
        position_max: int,
        torque_on_start: bool,
        torque_off_exit: bool,
    ) -> None:
        import tkinter as tk
        from tkinter import messagebox, ttk

        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.root = tk.Tk()
        self.root.title("DYNAMIXEL Controller")
        self.root.geometry("760x520")
        self.root.minsize(620, 360)

        self.bus = DynamixelBus(device, baudrate, protocol)
        self.ids = ids
        self.position_min = position_min
        self.position_max = position_max
        self.torque_on_start = torque_on_start
        self.torque_off_exit = torque_off_exit
        self.command_lock = threading.Lock()
        self.pending_after: dict[int, str] = {}
        self.sliders: dict[int, tk.Scale] = {}
        self.value_labels: dict[int, tk.StringVar] = {}
        self.position_labels: dict[int, tk.StringVar] = {}
        self.found: list[tuple[int, int]] = []

    def run(self) -> None:
        try:
            self.bus.__enter__()
            self.found = self.bus.scan(self.ids)
            if not self.found:
                self.messagebox.showerror(
                    "DYNAMIXEL Controller",
                    "No DYNAMIXEL found. Check power, port, baudrate, protocol, and bridge firmware.",
                )
                self.bus.__exit__(None, None, None)
                return

            if self.torque_on_start:
                for dxl_id, _model_number in self.found:
                    self.bus.set_torque(dxl_id, True)

            self.build_widgets()
            self.root.protocol("WM_DELETE_WINDOW", self.close)
            self.root.mainloop()
        except Exception as exc:
            self.messagebox.showerror("DYNAMIXEL Controller", str(exc))
            try:
                self.bus.__exit__(None, None, None)
            except Exception:
                pass

    def build_widgets(self) -> None:
        tk = self.tk
        ttk = self.ttk

        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(outer)
        header.pack(fill=tk.X)
        ttk.Label(header, text="DYNAMIXEL Controller", font=("", 18, "bold")).pack(side=tk.LEFT)
        ttk.Button(header, text="Refresh Positions", command=self.refresh_positions).pack(side=tk.RIGHT)

        canvas = tk.Canvas(outer, highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        list_frame = ttk.Frame(canvas)
        list_frame.bind(
            "<Configure>",
            lambda _event: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.create_window((0, 0), window=list_frame, anchor=tk.NW)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=(12, 0))
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, pady=(12, 0))

        for row, (dxl_id, model_number) in enumerate(self.found):
            self.add_servo_row(list_frame, row, dxl_id, model_number)

    def add_servo_row(self, parent, row: int, dxl_id: int, model_number: int) -> None:
        ttk = self.ttk
        tk = self.tk

        frame = ttk.Frame(parent, padding=(0, 10))
        frame.grid(row=row, column=0, sticky="ew")
        parent.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        value_var = tk.StringVar()
        position_var = tk.StringVar(value="position: --")
        self.value_labels[dxl_id] = value_var
        self.position_labels[dxl_id] = position_var

        try:
            position = self.bus.read_position(dxl_id)
        except Exception:
            position = (self.position_min + self.position_max) // 2
        position = max(self.position_min, min(self.position_max, position))
        value_var.set(str(position))
        position_var.set(f"position: {position}")

        ttk.Label(frame, text=f"ID {dxl_id}", width=8, font=("", 13, "bold")).grid(
            row=0,
            column=0,
            sticky="w",
        )
        slider = tk.Scale(
            frame,
            from_=self.position_min,
            to=self.position_max,
            orient=tk.HORIZONTAL,
            showvalue=False,
            resolution=1,
            command=lambda value, servo_id=dxl_id: self.queue_move(servo_id, value),
        )
        slider.set(position)
        slider.grid(row=0, column=1, sticky="ew", padx=10)
        self.sliders[dxl_id] = slider

        ttk.Label(frame, textvariable=value_var, width=6).grid(row=0, column=2, sticky="e")
        ttk.Label(frame, text=f"model {model_number}", width=12).grid(row=0, column=3, sticky="e")
        ttk.Label(frame, textvariable=position_var, width=16).grid(row=0, column=4, sticky="e")
        ttk.Button(frame, text="Torque Off", command=lambda servo_id=dxl_id: self.set_torque(servo_id, False)).grid(
            row=0,
            column=5,
            sticky="e",
            padx=(10, 0),
        )
        ttk.Button(frame, text="Torque On", command=lambda servo_id=dxl_id: self.set_torque(servo_id, True)).grid(
            row=0,
            column=6,
            sticky="e",
            padx=(6, 0),
        )

    def queue_move(self, dxl_id: int, value: str) -> None:
        position = int(float(value))
        self.value_labels[dxl_id].set(str(position))
        existing = self.pending_after.get(dxl_id)
        if existing is not None:
            self.root.after_cancel(existing)
        self.pending_after[dxl_id] = self.root.after(40, lambda: self.send_move(dxl_id, position))

    def send_move(self, dxl_id: int, position: int) -> None:
        self.pending_after.pop(dxl_id, None)
        try:
            with self.command_lock:
                self.bus.move_to(dxl_id, position)
        except Exception as exc:
            self.position_labels[dxl_id].set(f"error: {exc}")

    def set_torque(self, dxl_id: int, enabled: bool) -> None:
        try:
            with self.command_lock:
                self.bus.set_torque(dxl_id, enabled)
            self.position_labels[dxl_id].set(f"torque {'on' if enabled else 'off'}")
        except Exception as exc:
            self.position_labels[dxl_id].set(f"error: {exc}")

    def refresh_positions(self) -> None:
        for dxl_id, _model_number in self.found:
            try:
                with self.command_lock:
                    position = self.bus.read_position(dxl_id)
                self.position_labels[dxl_id].set(f"position: {position}")
                if self.position_min <= position <= self.position_max:
                    self.sliders[dxl_id].set(position)
            except Exception as exc:
                self.position_labels[dxl_id].set(f"error: {exc}")

    def close(self) -> None:
        for after_id in self.pending_after.values():
            self.root.after_cancel(after_id)
        if self.torque_off_exit:
            for dxl_id, _model_number in self.found:
                try:
                    self.bus.set_torque(dxl_id, False)
                except Exception:
                    pass
        self.bus.__exit__(None, None, None)
        self.root.destroy()


class RobotMappingGui:
    def __init__(
        self,
        device: str,
        baudrate: int,
        protocol: float,
        ids: list[int],
        config_path: str,
        output_path: str,
        layout: str,
        test_delta: int,
    ) -> None:
        import tkinter as tk
        from tkinter import messagebox, ttk

        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.root = tk.Tk()
        self.root.title("Hanachan Motor Mapping")
        self.root.geometry("1180x760")
        self.root.minsize(980, 560)

        self.bus = DynamixelBus(device, baudrate, protocol)
        self.scan_ids = ids
        self.config_path = config_path
        self.output_path = output_path
        self.layout = layout
        self.test_delta = test_delta
        self.config = load_robot_config(config_path, layout) if config_path else default_robot_config(layout)
        self.found: list[tuple[int, int]] = []
        self.id_vars: list[tk.StringVar] = []
        self.zero_vars: list[tk.StringVar] = []
        self.direction_vars: list[tk.StringVar] = []
        self.enabled_vars: list[tk.BooleanVar] = []
        self.status_var = tk.StringVar(value="Scanning...")

    def run(self) -> None:
        try:
            self.bus.__enter__()
            self.found = self.bus.scan(self.scan_ids)
            if not self.found:
                self.messagebox.showerror("Hanachan Motor Mapping", "No DYNAMIXEL found.")
                self.bus.__exit__(None, None, None)
                return
            self.build_widgets()
            self.root.protocol("WM_DELETE_WINDOW", self.close)
            self.root.mainloop()
        except Exception as exc:
            self.messagebox.showerror("Hanachan Motor Mapping", str(exc))
            try:
                self.bus.__exit__(None, None, None)
            except Exception:
                pass

    def build_widgets(self) -> None:
        tk = self.tk
        ttk = self.ttk
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        detected = ", ".join(str(dxl_id) for dxl_id, _model in self.found)
        header = ttk.Frame(outer)
        header.pack(fill=tk.X)
        ttk.Label(header, text="Hanachan Motor Mapping", font=("", 18, "bold")).pack(side=tk.LEFT)
        ttk.Button(header, text="Save Config", command=self.save_config).pack(side=tk.RIGHT)

        ttk.Label(outer, text=f"Detected IDs: {detected}").pack(anchor=tk.W, pady=(8, 6))

        list_container = ttk.Frame(outer)
        list_container.pack(fill=tk.BOTH, expand=True, pady=(4, 8))

        canvas = tk.Canvas(list_container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(list_container, orient=tk.VERTICAL, command=canvas.yview)
        body = ttk.Frame(canvas)
        body.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        window_id = canvas.create_window((0, 0), window=body, anchor=tk.NW)
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window_id, width=event.width))
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        for col, text, width in (
            (0, "Use", 6),
            (1, "Index", 7),
            (2, "Joint Slot", 24),
            (3, "Motor ID", 10),
            (4, "Zero Tick", 11),
            (5, "Direction", 10),
            (6, f"Test +/-{self.test_delta}", 18),
        ):
            ttk.Label(body, text=text, width=width, font=("", 11, "bold")).grid(
                row=0,
                column=col,
                sticky="w",
                padx=5,
                pady=(0, 6),
            )
        body.columnconfigure(2, weight=1)

        found_ids = [str(dxl_id) for dxl_id, _model in self.found]
        id_choices = [""] + found_ids
        used_indices = set(self.config.enabled_indices)
        for index in range(SERVO_COUNT):
            self.add_mapping_row(body, index + 1, index, id_choices, used_indices)

        footer = ttk.Frame(outer)
        footer.pack(fill=tk.X)
        ttk.Label(footer, textvariable=self.status_var).pack(side=tk.LEFT)
        ttk.Button(footer, text="Torque Off Detected", command=self.torque_off_detected).pack(side=tk.RIGHT)
        ttk.Button(footer, text="Read Zero From Current", command=self.read_selected_zeros).pack(
            side=tk.RIGHT,
            padx=(0, 8),
        )
        self.status_var.set(f"Save target: {self.output_path}")

    def add_mapping_row(self, parent, row: int, index: int, id_choices: list[str], used_indices: set[int]) -> None:
        ttk = self.ttk
        tk = self.tk

        enabled_var = tk.BooleanVar(value=index in used_indices)
        id_var = tk.StringVar(value=str(self.config.ids[index]) if index in used_indices else "")
        zero_var = tk.StringVar(value=str(self.config.zero_ticks[index]))
        direction_var = tk.StringVar(value=str(self.config.directions[index]))
        self.enabled_vars.append(enabled_var)
        self.id_vars.append(id_var)
        self.zero_vars.append(zero_var)
        self.direction_vars.append(direction_var)

        ttk.Checkbutton(parent, variable=enabled_var).grid(row=row, column=0, sticky="w", padx=5, pady=3)
        ttk.Label(parent, text=str(index), width=7).grid(row=row, column=1, sticky="w", padx=5)
        joint_label = JOINT_LABELS[index]
        if index in LEG4_INDICES and self.layout == "three-segment":
            joint_label = f"{joint_label} (not used)"
        elif index == TAIL_BODY_INDEX:
            joint_label = f"{joint_label} (disabled by default)"
        ttk.Label(parent, text=joint_label, width=24).grid(row=row, column=2, sticky="ew", padx=5)
        ttk.Combobox(parent, values=id_choices, textvariable=id_var, width=7, state="readonly").grid(
            row=row,
            column=3,
            sticky="w",
            padx=5,
        )
        ttk.Entry(parent, textvariable=zero_var, width=10).grid(row=row, column=4, sticky="w", padx=5)
        ttk.Combobox(parent, values=["1", "-1"], textvariable=direction_var, width=5, state="readonly").grid(
            row=row,
            column=5,
            sticky="w",
            padx=5,
        )
        buttons = ttk.Frame(parent)
        buttons.grid(
            row=row,
            column=6,
            sticky="w",
            padx=5,
        )
        ttk.Button(buttons, text="-", width=3, command=lambda i=index: self.test_move(i, -1)).pack(side=tk.LEFT)
        ttk.Button(buttons, text="0", width=3, command=lambda i=index: self.move_zero(i)).pack(
            side=tk.LEFT,
            padx=3,
        )
        ttk.Button(buttons, text="+", width=3, command=lambda i=index: self.test_move(i, 1)).pack(side=tk.LEFT)

    def selected_id(self, index: int) -> int | None:
        value = self.id_vars[index].get().strip()
        return int(value) if value else None

    def selected_zero(self, index: int) -> int:
        return int(self.zero_vars[index].get().strip())

    def selected_direction(self, index: int) -> int:
        return int(self.direction_vars[index].get().strip())

    def prepare_servo(self, dxl_id: int) -> None:
        self.bus.set_torque(dxl_id, False)
        self.bus.set_position_mode(dxl_id)
        self.bus.set_torque(dxl_id, True)

    def test_move(self, index: int, sign: int) -> None:
        dxl_id = self.selected_id(index)
        if dxl_id is None:
            self.status_var.set("Select an ID first.")
            return
        try:
            zero = self.selected_zero(index)
            direction = self.selected_direction(index)
            target = zero + sign * direction * self.test_delta
            self.prepare_servo(dxl_id)
            self.bus.move_to(dxl_id, target)
            self.status_var.set(f"{JOINT_LABELS[index]} ID {dxl_id}: moved to {target}")
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")

    def move_zero(self, index: int) -> None:
        dxl_id = self.selected_id(index)
        if dxl_id is None:
            self.status_var.set("Select an ID first.")
            return
        try:
            zero = self.selected_zero(index)
            self.prepare_servo(dxl_id)
            self.bus.move_to(dxl_id, zero)
            self.status_var.set(f"{JOINT_LABELS[index]} ID {dxl_id}: moved to zero {zero}")
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")

    def read_selected_zeros(self) -> None:
        updated = 0
        for index in range(SERVO_COUNT):
            dxl_id = self.selected_id(index)
            if dxl_id is None or not self.enabled_vars[index].get():
                continue
            try:
                self.zero_vars[index].set(str(self.bus.read_position(dxl_id)))
                updated += 1
            except Exception as exc:
                self.status_var.set(f"Error reading ID {dxl_id}: {exc}")
                return
        self.status_var.set(f"Updated zero ticks from {updated} selected motors.")

    def torque_off_detected(self) -> None:
        for dxl_id, _model in self.found:
            try:
                self.bus.set_torque(dxl_id, False)
            except Exception:
                pass
        self.status_var.set("Torque off sent to detected motors.")

    def save_config(self) -> None:
        ids = list(DEFAULT_ROBOT_IDS)
        zero_ticks = list(DEFAULT_ZERO_TICKS)
        directions = list(DEFAULT_DIRECTIONS)
        enabled_indices: list[int] = []
        assigned_ids: set[int] = set()

        try:
            for index in range(SERVO_COUNT):
                if not self.enabled_vars[index].get():
                    continue
                dxl_id = self.selected_id(index)
                if dxl_id is None:
                    raise ValueError(f"{JOINT_LABELS[index]} is enabled but has no ID")
                if dxl_id in assigned_ids:
                    raise ValueError(f"ID {dxl_id} is assigned more than once")
                assigned_ids.add(dxl_id)
                ids[index] = dxl_id
                zero_ticks[index] = self.selected_zero(index)
                directions[index] = self.selected_direction(index)
                enabled_indices.append(index)

            # Only ids, zero_ticks, directions, and enabled_indices are edited here.
            # Everything else is carried over so tuned gait values survive a remap.
            data = {
                "ids": ids,
                "zero_ticks": zero_ticks,
                "directions": directions,
                "joint_lower": list(self.config.joint_lower),
                "joint_upper": list(self.config.joint_upper),
                "gait_params": list(self.config.gait_params),
                "gait_params_v2": list(self.config.gait_params_v2),
                "enabled_indices": enabled_indices,
                "reverse_legs": self.config.reverse_legs,
                "sweep_phase_offset_rad": self.config.sweep_phase_offset_rad,
            }
            with open(self.output_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
                f.write("\n")
            self.status_var.set(f"Saved {len(enabled_indices)} joints to {self.output_path}")
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")

    def close(self) -> None:
        self.bus.__exit__(None, None, None)
        self.root.destroy()


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-d",
        "--device",
        help="serial device connected to OpenRB-150, for example /dev/cu.usbmodemXXXX",
    )
    parser.add_argument("-b", "--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("-p", "--protocol", type=float, default=DEFAULT_PROTOCOL)


def add_robot_config_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--layout",
        choices=("three-segment", "full"),
        default="three-segment",
        help="robot layout used when no config overrides enabled_indices",
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_PATH,
        help="JSON file overriding ids, zero_ticks, directions, joint limits, or gait_params",
    )
    parser.add_argument(
        "--robot-ids",
        type=parse_robot_ids,
        help="28 DYNAMIXEL IDs in robot joint order",
    )
    parser.add_argument(
        "--zero-ticks",
        type=parse_int_list,
        help="28 raw ticks corresponding to joint angle 0 rad",
    )
    parser.add_argument(
        "--directions",
        type=parse_signed_int_list,
        help="28 direction signs in joint order; each value must be 1 or -1",
    )
    parser.add_argument(
        "--zero-overrides",
        type=parse_int_overrides,
        help="ID-based zero tick overrides, for example 5:2060,12:2035",
    )
    parser.add_argument(
        "--direction-overrides",
        type=parse_int_overrides,
        help="ID-based direction overrides, for example 5:-1,12:-1",
    )
    parser.add_argument(
        "--gait-params",
        type=parse_float_params,
        help="12 comma-separated normalized gait parameters",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Control DYNAMIXEL servos connected through an OpenRB-150.",
    )
    add_common_args(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    scan = subparsers.add_parser("scan", help="ping ids and show connected servos")
    scan.add_argument("--ids", type=parse_ids, default=parse_ids("1-20"))

    discover = subparsers.add_parser(
        "discover",
        help="try common baudrates/protocols and show connected servos",
    )
    discover.add_argument("--ids", type=parse_ids, default=parse_ids("1-252"))
    discover.add_argument(
        "--baudrates",
        type=parse_int_list,
        default=list(DEFAULT_DISCOVERY_BAUDRATES),
        help="comma-separated baudrates to try",
    )
    discover.add_argument(
        "--protocols",
        type=parse_float_list,
        default=list(DEFAULT_DISCOVERY_PROTOCOLS),
        help="comma-separated protocols to try",
    )

    torque = subparsers.add_parser("torque", help="enable or disable motor torque")
    torque.add_argument("state", choices=("on", "off"))
    torque.add_argument("--ids", type=parse_ids, required=True)

    led = subparsers.add_parser("led", help="turn servo LED on or off")
    led.add_argument("state", choices=("on", "off"))
    led.add_argument("--ids", type=parse_ids, required=True)

    change_id = subparsers.add_parser("change-id", help="change one isolated DYNAMIXEL id")
    change_id.add_argument("current_id", type=int)
    change_id.add_argument("new_id", type=int)

    read = subparsers.add_parser("read", help="read current position")
    read.add_argument("--ids", type=parse_ids, required=True)

    move = subparsers.add_parser("move", help="move servos to a goal position")
    move.add_argument("--ids", type=parse_ids, required=True)
    move.add_argument("--position", type=int, required=True, help="goal position tick")
    move.add_argument("--wait", action="store_true", help="wait until movement ends")
    move.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_SECONDS)

    gui = subparsers.add_parser("gui", help="scan servos and control them with sliders")
    gui.add_argument("--ids", type=parse_ids, default=parse_ids("1-252"))
    gui.add_argument("--min-position", type=int, default=DEFAULT_POSITION_MIN)
    gui.add_argument("--max-position", type=int, default=DEFAULT_POSITION_MAX)
    gui.add_argument(
        "--no-torque-on-start",
        action="store_true",
        help="do not enable torque automatically after scan",
    )
    gui.add_argument(
        "--torque-off-exit",
        action="store_true",
        help="disable torque for detected servos when closing the GUI",
    )

    mapping = subparsers.add_parser("map", help="assign detected motor IDs to robot joints")
    mapping.add_argument("--ids", type=parse_ids, default=parse_ids("1-252"))
    mapping.add_argument(
        "--config",
        default=DEFAULT_CONFIG_PATH,
        help="existing config to load before editing",
    )
    mapping.add_argument("--output", default=DEFAULT_CONFIG_PATH, help="config JSON to write")
    mapping.add_argument(
        "--layout",
        choices=("three-segment", "full"),
        default="three-segment",
        help="initial robot layout when no config is loaded",
    )
    mapping.add_argument(
        "--test-delta",
        type=int,
        default=DEFAULT_TEST_DELTA,
        help="raw tick delta for +/- test buttons",
    )

    teach = subparsers.add_parser(
        "teach",
        help="capture hand-positioned joints and build a motion sequence",
    )
    teach.add_argument(
        "--layout",
        choices=("three-segment", "full"),
        default="three-segment",
        help="robot layout used when the config does not override enabled_indices",
    )
    teach.add_argument(
        "--config",
        default=DEFAULT_CONFIG_PATH,
        help="JSON file defining robot ids and enabled_indices",
    )
    teach.add_argument(
        "--robot-ids",
        type=parse_robot_ids,
        help="28 DYNAMIXEL IDs in robot joint order",
    )
    teach.set_defaults(
        zero_ticks=None,
        directions=None,
        zero_overrides=None,
        direction_overrides=None,
        gait_params=None,
    )
    teach.add_argument(
        "--input",
        default="",
        help="motion sequence JSON to load when the GUI opens",
    )
    teach.add_argument(
        "--output",
        default="motion_sequence.json",
        help="motion sequence JSON written by the Save button",
    )

    cpg = subparsers.add_parser("cpg", help="run the Hanachan CPG gait ported from OpenCR")
    add_robot_config_args(cpg)
    cpg.add_argument("--start", action="store_true", help="start walking immediately")
    cpg.add_argument("--skip-init", action="store_true", help="do not initialize torque/mode/neutral")
    cpg.add_argument(
        "--skip-position-mode",
        action="store_true",
        help="do not write operating mode during initialization",
    )
    cpg.add_argument(
        "--torque-on",
        action="store_true",
        help="with --skip-init, enable torque before entering the loop",
    )
    cpg.add_argument(
        "--torque-off-exit",
        action="store_true",
        help="disable torque when the program exits",
    )
    cpg.add_argument(
        "--control-hz",
        type=float,
        help="control frequency; defaults to 8 Hz at wireless baudrates and 50 Hz otherwise",
    )

    for command, model_help in (
        ("cpg-gui", "tune and run the Hanachan CPG with sliders (v1 gait model)"),
        ("cpg-gui-v2", "tune and run the v2 gait model with sliders"),
    ):
        add_cpg_gui_args(subparsers.add_parser(command, help=model_help))

    return parser


def add_cpg_gui_args(cpg_gui: argparse.ArgumentParser) -> None:
    add_robot_config_args(cpg_gui)
    cpg_gui.add_argument(
        "--output",
        default=DEFAULT_CONFIG_PATH,
        help="config JSON written by the Save Config button",
    )
    cpg_gui.add_argument(
        "--preset-file",
        default="gait_presets.json",
        help="named gait preset JSON loaded and saved by the GUI",
    )
    cpg_gui.add_argument(
        "--leg-design-file",
        default="leg_motion_designs.json",
        help="single-leg keyframes and per-leg phase offsets used by Leg Motion Designer",
    )
    cpg_gui.add_argument(
        "--initial-preset",
        choices=("safe", "config", "forward"),
        default="safe",
        help="initial GUI gait values; safe is the low-frequency default",
    )
    cpg_gui.add_argument("--skip-init", action="store_true", help="do not initialize torque/mode/neutral")
    cpg_gui.add_argument(
        "--skip-position-mode",
        action="store_true",
        help="do not write operating mode during initialization",
    )
    cpg_gui.add_argument(
        "--torque-on",
        action="store_true",
        help="with --skip-init, enable torque before opening the GUI",
    )
    cpg_gui.add_argument(
        "--torque-off-exit",
        action="store_true",
        help="disable torque when the GUI closes",
    )
    cpg_gui.add_argument(
        "--control-hz",
        type=float,
        help="control frequency; defaults to 8 Hz at wireless baudrates and 50 Hz otherwise",
    )
    cpg_gui.add_argument(
        "--onboard-cpg",
        action="store_true",
        help="run the 50 Hz CPG on OpenRB and send only parameters over wireless serial",
    )


def choose_device(device: str | None) -> str:
    if device:
        return device

    ports = candidate_ports()
    if len(ports) == 1:
        return ports[0]
    if not ports:
        raise RuntimeError(
            "serial device was not found. Reconnect OpenRB-150 or pass --device explicitly."
        )
    descriptions = describe_ports()
    candidates = "\n  ".join(
        f"{port} ({descriptions[port]})" if descriptions.get(port) else port for port in ports
    )
    raise RuntimeError(f"multiple serial devices found. Pass one with --device:\n  {candidates}")


def print_not_found_help() -> None:
    print("No DYNAMIXEL found.")
    print("Hints:")
    print("  - On macOS, try /dev/cu.* instead of /dev/tty.*.")
    print("  - On Windows, pass the COM port, for example --device COM5.")
    print("  - Try: discover --ids 1-252")
    print("  - Check DYNAMIXEL power, ID, baudrate, and protocol.")
    print("  - OpenRB-150 must be running USB-to-DYNAMIXEL bridge firmware for PC SDK access.")


def enqueue_stdin_commands(commands: queue.Queue[str], stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        line = sys.stdin.readline()
        if not line:
            stop_event.set()
            commands.put("q")
            return
        for char in line.strip():
            if char in {"s", "x", "t", "q"}:
                commands.put(char)


def build_robot_config_from_args(
    args: argparse.Namespace,
    gait_model: str = DEFAULT_GAIT_MODEL,
) -> RobotConfig:
    config = load_robot_config(args.config, args.layout)
    return apply_robot_overrides(
        config,
        ids=args.robot_ids,
        zero_ticks=args.zero_ticks,
        directions=args.directions,
        gait_params=args.gait_params,
        zero_overrides=args.zero_overrides,
        direction_overrides=args.direction_overrides,
        gait_model=gait_model,
    )


def run_cpg(args: argparse.Namespace, device: str) -> None:
    config = build_robot_config_from_args(args)
    control_period = resolve_control_period(args.baudrate, args.control_hz)
    commands: queue.Queue[str] = queue.Queue()
    stop_event = threading.Event()
    reader = threading.Thread(
        target=enqueue_stdin_commands,
        args=(commands, stop_event),
        daemon=True,
    )

    with DynamixelBus(device, args.baudrate, args.protocol) as bus:
        controller = HanachanCPGController(bus, config)
        if not args.skip_init:
            controller.initialize_servos(set_position_mode=not args.skip_position_mode)
        elif args.torque_on:
            controller.set_torque(True)

        if args.start:
            controller.start_motion()

        print("Hanachan CPG Python ready")
        print("Commands: s=start, x=stop neutral, t=torque on, q=torque off and quit")
        print("Press a command key then Enter.")
        reader.start()

        last_time = time.monotonic()
        next_control = last_time
        try:
            while not stop_event.is_set():
                try:
                    while True:
                        command = commands.get_nowait()
                        if command == "s":
                            controller.start_motion()
                            print("motion start")
                        elif command == "x":
                            controller.stop_motion_neutral()
                            print("motion stop")
                        elif command == "t":
                            controller.set_torque(True)
                            print("torque on")
                        elif command == "q":
                            controller.motion_enabled = False
                            controller.set_torque(False)
                            print("torque off")
                            stop_event.set()
                            break
                except queue.Empty:
                    pass

                now = time.monotonic()
                if now < next_control:
                    time.sleep(min(0.002, next_control - now))
                    continue

                dt = now - last_time
                last_time = now
                next_control = now + control_period
                controller.update(dt, control_period)
        finally:
            stop_event.set()
            if args.torque_off_exit:
                try:
                    controller.set_torque(False)
                except Exception:
                    pass


def run(args: argparse.Namespace) -> None:
    device = choose_device(args.device)
    if args.command == "map":
        gui = RobotMappingGui(
            device=device,
            baudrate=args.baudrate,
            protocol=args.protocol,
            ids=args.ids,
            config_path=args.config,
            output_path=args.output,
            layout=args.layout,
            test_delta=args.test_delta,
        )
        gui.run()
        return

    if args.command == "teach":
        config = build_robot_config_from_args(args)
        gui = MotionTeachingGui(
            device=device,
            baudrate=args.baudrate,
            protocol=args.protocol,
            config=config,
            input_path=args.input,
            output_path=args.output,
        )
        gui.run()
        return

    if args.command == "cpg":
        run_cpg(args, device)
        return

    if args.command in ("cpg-gui", "cpg-gui-v2"):
        gait_model = "v2" if args.command == "cpg-gui-v2" else "v1"
        model = GAIT_MODELS[gait_model]
        config = build_robot_config_from_args(args, gait_model)
        if args.gait_params is None:
            active = config.params_for(gait_model)
            if args.initial_preset == "safe":
                active[:] = model.safe_params
            elif args.initial_preset == "forward":
                active[:] = model.forward_params
        gui = CPGGui(
            device=device,
            baudrate=args.baudrate,
            protocol=args.protocol,
            config=config,
            output_path=args.output,
            preset_path=args.preset_file,
            leg_design_path=args.leg_design_file,
            skip_init=args.skip_init,
            skip_position_mode=args.skip_position_mode,
            torque_on=args.torque_on,
            torque_off_exit=args.torque_off_exit,
            control_hz=args.control_hz,
            onboard_cpg=args.onboard_cpg,
            gait_model=gait_model,
        )
        gui.run()
        return

    if args.command == "gui":
        gui = DynamixelGui(
            device=device,
            baudrate=args.baudrate,
            protocol=args.protocol,
            ids=args.ids,
            position_min=args.min_position,
            position_max=args.max_position,
            torque_on_start=not args.no_torque_on_start,
            torque_off_exit=args.torque_off_exit,
        )
        gui.run()
        return

    if args.command == "discover":
        found_any = False
        for protocol in args.protocols:
            for baudrate in args.baudrates:
                with DynamixelBus(device, baudrate, protocol) as bus:
                    found = bus.scan(args.ids)
                if found:
                    found_any = True
                    print(f"protocol {protocol:g}, baudrate {baudrate}:")
                    for dxl_id, model_number in found:
                        print(f"  ID {dxl_id}: model {model_number}")
        if not found_any:
            print("No DYNAMIXEL found with the tried protocols/baudrates.")
            print("If the servos work from an OpenRB Arduino sketch, flash USB-to-DYNAMIXEL bridge firmware before using the PC SDK directly.")
        return

    with DynamixelBus(device, args.baudrate, args.protocol) as bus:
        if args.command == "scan":
            found = bus.scan(args.ids)
            if not found:
                print_not_found_help()
                return
            for dxl_id, model_number in found:
                print(f"ID {dxl_id}: model {model_number}")
            return

        if args.command == "torque":
            enabled = args.state == "on"
            for dxl_id in args.ids:
                bus.set_torque(dxl_id, enabled)
                print(f"ID {dxl_id}: torque {'on' if enabled else 'off'}")
            return

        if args.command == "led":
            enabled = args.state == "on"
            for dxl_id in args.ids:
                bus.set_led(dxl_id, enabled)
                print(f"ID {dxl_id}: led {'on' if enabled else 'off'}")
            return

        if args.command == "change-id":
            if args.current_id == args.new_id:
                raise RuntimeError("current_id and new_id are the same")
            if bus.ping(args.new_id) is not None:
                raise RuntimeError(f"new_id {args.new_id} already responds on the bus")
            if bus.ping(args.current_id) is None:
                raise RuntimeError(
                    f"current_id {args.current_id} did not respond cleanly. "
                    "If duplicate IDs are present, isolate one servo before changing its ID."
                )
            bus.change_id(args.current_id, args.new_id)
            time.sleep(0.2)
            if bus.ping(args.new_id) is None:
                raise RuntimeError(f"ID write was sent, but new_id {args.new_id} did not respond")
            print(f"ID {args.current_id} changed to {args.new_id}")
            return

        if args.command == "read":
            for dxl_id in args.ids:
                print(f"ID {dxl_id}: position {bus.read_position(dxl_id)}")
            return

        if args.command == "move":
            for dxl_id in args.ids:
                bus.set_torque(dxl_id, True)
                bus.move_to(dxl_id, args.position)
                print(f"ID {dxl_id}: goal position {args.position}")

            if args.wait:
                deadline = time.monotonic() + args.timeout
                moving_ids = set(args.ids)
                while moving_ids and time.monotonic() < deadline:
                    moving_ids = {dxl_id for dxl_id in moving_ids if bus.read_moving(dxl_id)}
                    time.sleep(0.05)
                for dxl_id in args.ids:
                    print(f"ID {dxl_id}: position {bus.read_position(dxl_id)}")
            return

        raise AssertionError(f"unknown command: {args.command}")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
