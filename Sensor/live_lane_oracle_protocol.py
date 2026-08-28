"""LiveLaneOracle.py 의 UDP 와이어 포맷.

`lane_stub_udp`(mgeo_lane_stub)의 `lane_info_protocol.py`와 같은 스타일(헤더 +
data_length + tail)이지만, 좌표계가 다르다 — 저건 자차 좌표(미터), 이건
**픽셀 좌표**다. `GenerateLabels.py`가 학습 라벨을 만들 때 쓰는 것과 똑같은
투영(HD맵 → 자차좌표 → 카메라좌표 → 이미지)을 실시간으로 돌려서, 나중에 실제
학습된 모델이 내놓을 형식(픽셀 기준 3차 다항식)을 그대로 흉내낸 것이다.

패킷 구조::

    header       12B  b"#LaneOracle$"
    data_length   4B  uint32
    sec           4B  uint32 — 이 값을 만든 카메라 프레임의 타임스탬프
    nsec          4B  uint32
    --- 좌측 차선 ---
    left_conf     1B  uint8   0/1 — 0 이면 아래 좌측 필드는 의미 없음
    left_category 1B  int8    0=white_solid 1=white_dashed 2=yellow -1=unknown
    left_broken   1B  uint8   0/1
    left_lower_v  4B  float32 이 곡선이 실제로 관측된 화면 v(세로) 최솟값
    left_upper_v  4B  float32 관측된 v 최댓값 — 이 범위 밖으로 곡선을 믿지 말 것
    left_a/b/c/d 16B  float32 x4 — u = a*v^3 + b*v^2 + c*v + d (픽셀)
    left_num_points 2B uint16
    --- 우측 차선 --- (좌측과 완전히 같은 레이아웃)
    right_...    위와 동일
    --- 정지선 ---
    stop_valid    1B  uint8
    stop_distance_m 4B float32  자차 원점 기준 전방 거리(미터) — 픽셀이 아니라
                                미터인 이유: 정지선은 진행방향과 직각이라
                                x=f(v) 로 표현이 안 되고(BuildPolyTargets.py
                                참고), 제어가 실제로 쓰는 것도 미터 단위 거리다
    --- 가변 길이 ---
    left_points   left_num_points  * 8B  (float32 u, float32 v)
    right_points  right_num_points * 8B
    tail          2B  b"\\r\\n"

좌/우 각각의 (conf, category, broken, lower_v, upper_v, coeffs) 6개 값 +
포인트는 `BuildPolyTargets.fit_slot()`이 만드는 것과 완전히 같은 필드다 —
학습 타깃과 실시간 오라클이 같은 스키마를 쓰게 하려는 의도다.
"""

import struct

PACKET_HEADER = b"#LaneOracle$"
PACKET_TAIL = b"\r\n"

CATEGORY_UNKNOWN = -1
CATEGORY_WHITE_SOLID = 0
CATEGORY_WHITE_DASHED = 1
CATEGORY_YELLOW = 2

_SIDE_FMT = "BbBffffffH"          # conf,category,broken,lower_v,upper_v,a,b,c,d,num_points
_SIDE_SIZE = struct.calcsize("<" + _SIDE_FMT)
_HEAD_FMT = "<II"                 # sec, nsec
_TAIL_FMT = "<Bf"                 # stop_valid, stop_distance_m
_FIXED_FMT = "<II" + _SIDE_FMT + _SIDE_FMT + "Bf"
_FIXED_SIZE = struct.calcsize(_FIXED_FMT)
_POINT_FMT = "<ff"
_POINT_SIZE = struct.calcsize(_POINT_FMT)

_EMPTY_SIDE = {"conf": 0, "category": CATEGORY_UNKNOWN, "broken": False,
              "lower_v": 0.0, "upper_v": 0.0, "coeffs": (0.0, 0.0, 0.0, 0.0),
              "points": ()}


class LaneOraclePacketError(ValueError):
    """수신한 바이트열이 이 와이어 포맷과 안 맞을 때."""


def _pack_side(side):
    side = {**_EMPTY_SIDE, **(side or {})}
    a, b, c, d = side["coeffs"]
    points = list(side["points"])
    fixed = struct.pack(
        "<" + _SIDE_FMT,
        1 if side["conf"] else 0, int(side["category"]),
        1 if side["broken"] else 0,
        float(side["lower_v"]), float(side["upper_v"]),
        float(a), float(b), float(c), float(d), len(points),
    )
    body = b"".join(struct.pack(_POINT_FMT, float(u), float(v)) for u, v in points)
    return fixed, body


def pack_lane_oracle(sec, nsec, left=None, right=None,
                     stopline_valid=False, stopline_distance_m=0.0):
    """`left`/`right`는 dict — `BuildPolyTargets.fit_slot()`이 돌려주는 것과
    같은 키(conf 대신 존재 여부는 dict 자체가 None 인지로 판단해도 됨) +
    `points`(픽셀 (u,v) 목록)를 넣으면 된다. None 이면 '못 찾음'으로 채운다.
    """
    left_fixed, left_body = _pack_side(left)
    right_fixed, right_body = _pack_side(right)
    fixed = (struct.pack(_HEAD_FMT, int(sec) & 0xFFFFFFFF, int(nsec) & 0xFFFFFFFF)
             + left_fixed + right_fixed
             + struct.pack(_TAIL_FMT, 1 if stopline_valid else 0, float(stopline_distance_m)))
    data = fixed + left_body + right_body
    return PACKET_HEADER + struct.pack("<I", len(data)) + data + PACKET_TAIL


def _unpack_side(packet, offset):
    (conf, category, broken, lower_v, upper_v, a, b, c, d,
     num_points) = struct.unpack_from("<" + _SIDE_FMT, packet, offset)
    return {
        "conf": bool(conf), "category": category, "broken": bool(broken),
        "lower_v": lower_v, "upper_v": upper_v, "coeffs": (a, b, c, d),
    }, num_points


def parse_lane_oracle(packet):
    """수신한 UDP 페이로드를 dict 로 판독한다. 형식이 안 맞으면 LaneOraclePacketError."""
    header_len = len(PACKET_HEADER)
    if len(packet) < header_len + 4 + len(PACKET_TAIL):
        raise LaneOraclePacketError(f"packet too short: {len(packet)} bytes")
    if packet[:header_len] != PACKET_HEADER:
        raise LaneOraclePacketError(f"unexpected header {packet[:header_len]!r}")

    (data_length,) = struct.unpack_from("<I", packet, header_len)
    data_start = header_len + 4
    if len(packet) != data_start + data_length + len(PACKET_TAIL):
        raise LaneOraclePacketError(
            f"data_length {data_length} inconsistent with packet size {len(packet)}")
    if packet[-len(PACKET_TAIL):] != PACKET_TAIL:
        raise LaneOraclePacketError(f"unexpected tail {packet[-2:]!r}")
    if data_length < _FIXED_SIZE:
        raise LaneOraclePacketError(f"data_length {data_length} shorter than fixed header")

    sec, nsec = struct.unpack_from(_HEAD_FMT, packet, data_start)
    left, left_n = _unpack_side(packet, data_start + 8)
    right, right_n = _unpack_side(packet, data_start + 8 + _SIDE_SIZE)
    stop_off = data_start + 8 + 2 * _SIDE_SIZE
    stop_valid, stop_distance_m = struct.unpack_from(_TAIL_FMT, packet, stop_off)

    if data_length != _FIXED_SIZE + (left_n + right_n) * _POINT_SIZE:
        raise LaneOraclePacketError(
            f"data_length does not match declared point counts ({left_n}, {right_n})")

    points_off = data_start + _FIXED_SIZE
    left["points"] = [struct.unpack_from(_POINT_FMT, packet, points_off + i * _POINT_SIZE)
                      for i in range(left_n)]
    points_off += left_n * _POINT_SIZE
    right["points"] = [struct.unpack_from(_POINT_FMT, packet, points_off + i * _POINT_SIZE)
                       for i in range(right_n)]

    return {
        "sec": sec, "nsec": nsec, "left": left, "right": right,
        "stopline_valid": bool(stop_valid), "stopline_distance_m": stop_distance_m,
    }
