"""完整观测事实快照的纯 JSON 协议；不重采样、不分类、不传 native handle。

CellState 编码：-1 UNKNOWN、0 FREE、1 BLOCKED。
与既有C++/ROS类型对齐：revision与epoch为uint64，stamp满足既有ROS Time的非负int32秒范围，
height为有限float32范围；JSON整数不经浮点中转。"""
import json
import math

from .base_map import BaseMapFacts


_SCHEMA = "lunar-observation-facts/v1"
_FLOAT32_MAX = float.fromhex("0x1.fffffep+127")
_ENVELOPE_FIELDS = {"schema", "source_resolution_m", "raw_revision", "frame", "epoch",
                    "complete_snapshot", "observation_fragments", "processed_stamp_ns"}
_FRAGMENT_FIELDS = {"polygon", "height_m", "slope_rad", "relief_m", "state"}


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _fields(value, expected):
    if type(value) is not dict or set(value) != expected:
        raise ValueError("observation facts require exact complete protocol fields")


def _validate(data):
    _fields(data, _ENVELOPE_FIELDS)
    if data["schema"] != _SCHEMA or data["complete_snapshot"] is not True:
        raise ValueError("unsupported schema or incomplete observation snapshot")
    resolution = data["source_resolution_m"]
    if not _finite(resolution) or resolution <= 0:
        raise ValueError("source_resolution_m must be finite and positive")
    if type(data["processed_stamp_ns"]) is not int or not 0 <= data["processed_stamp_ns"] < 2**31 * 10**9:
        raise ValueError("processed_stamp_ns must fit nonnegative ROS Time")
    for key in ("raw_revision", "epoch"):
        if type(data[key]) is not int or not 0 <= data[key] < 2**64:
            raise ValueError(f"{key} must be uint64")
    if type(data["frame"]) is not str or not data["frame"].strip():
        raise ValueError("frame must be a nonempty string")
    fragments = data["observation_fragments"]
    if type(fragments) is not list:
        raise ValueError("observation_fragments must be an array")
    for fragment in fragments:
        _fields(fragment, _FRAGMENT_FIELDS)
        if not _finite(fragment["height_m"]) or abs(fragment["height_m"]) > _FLOAT32_MAX:
            raise ValueError("height_m must be within finite float32 storage range")
        if type(fragment["state"]) is not int or fragment["state"] not in (-1, 0, 1):
            raise ValueError("state must be UNKNOWN(-1), FREE(0), or BLOCKED(1)")
        for key in ("slope_rad", "relief_m"):
            value = fragment[key]
            if value is not None and (not _finite(value) or value < 0):
                raise ValueError(f"{key} must be nonnegative finite or null")
        polygon = fragment["polygon"]
        if type(polygon) is not list or len(polygon) < 3:
            raise ValueError("polygon must contain at least three world-meter points")
        for point in polygon:
            if type(point) is not list or len(point) != 2 or not all(_finite(v) for v in point):
                raise ValueError("polygon points must contain two finite world-meter coordinates")
    return data


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate observation facts field: {key}")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"nonfinite JSON number: {value}")


def decode_observation_facts(payload):
    """验证 v1 完整快照并返回独立的 dict/list 普通值对象。"""
    if not isinstance(payload, str):
        raise ValueError("observation facts payload must be JSON text")
    return _validate(json.loads(payload, object_pairs_hook=_unique_object,
                                parse_constant=_reject_constant))


def encode_observation_facts(facts: BaseMapFacts, *, source_resolution_m,
                             frame_id, epoch, processed_stamp_ns=0) -> str:
    """按原顺序编码 observation_fragments；历史测量不按格去重。"""
    if not isinstance(facts, BaseMapFacts):
        raise ValueError("facts must be BaseMapFacts")
    fragments = []
    for source in facts.sources:
        for fragment in source.observation_fragments:
            measurement = source.measurements[fragment.measurement_index]
            slope = float(measurement["slope_rad"])
            relief = float(measurement["relief_m"])
            fragments.append({
                "polygon": [[x / 100.0, y / 100.0] for x, y in fragment.polygon_cm],
                "height_m": float(measurement["height_m"]),
                "slope_rad": None if math.isnan(slope) else slope,
                "relief_m": None if math.isnan(relief) else relief,
                "state": int(measurement["state"]),
            })
    data = {"schema": _SCHEMA, "source_resolution_m": source_resolution_m,
            "raw_revision": facts.raw_revision, "frame": frame_id, "epoch": epoch,
            "complete_snapshot": True, "observation_fragments": fragments,
            "processed_stamp_ns": processed_stamp_ns}
    _validate(data)
    return json.dumps(data, allow_nan=False, ensure_ascii=False, separators=(",", ":"))
