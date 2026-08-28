"""GenerateLabels.py 의 구조화 라벨(vectors/*.json)을 PolyRegression 학습
타깃으로 변환한다.

    vectors/*.json (차선 ID 포함 폴리라인)
        → 슬롯별 [conf, lower_y, upper_y, a, b, c, d] + 클래스 3-way
        → poly_targets/*.json

--------------------------------------------------------------------------
슬롯을 4개(left_2/ego_left/ego_right/right_2)로 고정한 근거
--------------------------------------------------------------------------
9개 랩(4,080프레임)의 vectors/*.json 을 전수 스캔한 실측:

    lane_index  -1     1      존재율 100% (4080/4080) — 항상 있음
    lane_index  -2     2      존재율 90% / 80%
    lane_index  -3     3      존재율 73% / 58%
    lane_index  -4 ~ -20      점점 줄다가 길게 꼬리를 끈다 (최대 ±20)

±3 밖의 긴 꼬리는 실제 인접 차로가 아니라 **교차로에서 다른 방향 도로의
경계가 섞여 들어온 것**이다(assign_lane_ids 주석 참고: 교차로 한 프레임에
70개까지 잡힘). 그런 것까지 슬롯을 할당하면 낭비이자 노이즈라, ego 바로
양옆 2개씩(±1, ±2)만 고정 슬롯으로 회귀하고 그 밖은 이번 회귀 head의
대상에서 뺀다.

--------------------------------------------------------------------------
정지선은 이 회귀로 표현하지 않는다
--------------------------------------------------------------------------
PolyRegression 은 x = a*y^3 + b*y^2 + c*y + d, 즉 진행방향 곡선 전용이다.
정지선은 진행방향과 직각이라 이 파라미터화가 안 맞는다 — 별도 head 필요
(5클래스 래스터의 stopline 채널에서 라벨을 파생시킬 것).
"""

import argparse
import glob
import json
import os

import numpy as np

from GenerateLabels import CLASS_WHITE_SOLID, CLASS_WHITE_DASHED, CLASS_YELLOW

# 슬롯 순서 고정: index 0=left_2, 1=ego_left, 2=ego_right, 3=right_2
SLOT_LANE_INDEX = [-2, -1, 1, 2]
SLOT_NAMES = ["left_2", "ego_left", "ego_right", "right_2"]
NUM_SLOTS = len(SLOT_LANE_INDEX)
POLY_DEGREE = 3

# extra_outputs 3-way 클래스. 유도선(guide) 등 이 셋에 없는 클래스는 -1(ignore)
# 로 둔다 — 기하(존재·모양)는 그대로 학습시키되 색/스타일 분류 loss에서만 뺀다.
CLASS_TO_CATEGORY = {
    CLASS_WHITE_SOLID: 0,
    CLASS_WHITE_DASHED: 1,
    CLASS_YELLOW: 2,
}


def fit_slot(boundary, width, height):
    """한 슬롯의 폴리라인을 [conf, lower_y, upper_y, a, b, c, d] 로 만든다.

    **`points_uv`는 화면 밖으로 나가는 점을 걸러내지 않은 원본이다.** `near`/
    근평면 클리핑은 월드 좌표(전방·횡거리) 기준이라, 카메라 바로 옆을 스치듯
    지나가는 지점처럼 화면 경계 근처에서 극단적인 픽셀좌표로 투영되는 경우를
    막지 못한다(실측: 149점 중 8점이 화면 밖, v좌표가 이미지 높이의 4배까지
    벗어남). 그대로 피팅하면 lower_y/upper_y 와 계수가 그 이상치에 끌려간다.
    """
    pts = np.asarray(boundary["points_uv"], dtype=np.float64)
    inb = ((pts[:, 0] >= 0) & (pts[:, 0] < width)
           & (pts[:, 1] >= 0) & (pts[:, 1] < height))
    pts = pts[inb]
    if len(pts) == 0:
        return None
    xs, ys = pts[:, 0], pts[:, 1]
    lower_y, upper_y = float(ys.min()), float(ys.max())

    # 점이 적거나(짧게 잘린 폴리라인) 세로 폭이 좁으면 3차 피팅이
    # rank-deficient 해져 계수가 튄다 — 점 개수에 맞춰 차수를 낮추고,
    # 안 쓰는 고차항은 0으로 채운다(계수 배열 길이는 항상 4로 고정).
    deg = min(POLY_DEGREE, len(pts) - 1)
    if deg < 1 or upper_y - lower_y < 5.0:
        coeffs = [0.0, 0.0, 0.0, float(np.median(xs))]
    else:
        fit = np.polyfit(ys, xs, deg)
        coeffs = [0.0] * (POLY_DEGREE - deg) + [float(c) for c in fit]

    category = CLASS_TO_CATEGORY.get(boundary["cls"], -1)
    return {
        "conf": 1,
        "lower_y": lower_y,
        "upper_y": upper_y,
        "coeffs": coeffs,
        "category": category,
        "broken": bool(boundary["broken"]),
        "map_idx": boundary["map_idx"],
    }


EMPTY_SLOT = {
    "conf": 0, "lower_y": 0.0, "upper_y": 0.0,
    "coeffs": [0.0, 0.0, 0.0, 0.0], "category": -1,
    "broken": False, "map_idx": None,
}


def convert_frame(rec):
    """vectors/*.json 레코드 하나 → poly_targets 레코드 하나."""
    width, height = rec["width"], rec["height"]
    by_index = {b["lane_index"]: b for b in rec["boundaries"]
                if b.get("lane_index") in SLOT_LANE_INDEX}
    slots = []
    for li in SLOT_LANE_INDEX:
        b = by_index.get(li)
        fitted = fit_slot(b, width, height) if b is not None else None
        slots.append(fitted if fitted is not None else dict(EMPTY_SLOT))
    return {
        "idx": rec["idx"],
        "image": rec["image"],
        "crop_top": rec["crop_top"],
        "width": rec["width"],
        "height": rec["height"],
        "slot_names": SLOT_NAMES,
        "slots": slots,
    }


def main():
    ap = argparse.ArgumentParser(
        description="vectors/*.json 을 PolyRegression 학습 타깃(poly_targets/*.json)으로 변환")
    ap.add_argument("--recording", required=True,
                    help="vectors/ 가 있는 녹화 폴더 (GenerateLabels.py --save-vectors 로 생성)")
    ap.add_argument("--out", default=None,
                    help="출력 폴더 (기본: <recording>/poly_targets)")
    args = ap.parse_args()

    vec_dir = os.path.join(args.recording, "vectors")
    files = sorted(glob.glob(os.path.join(vec_dir, "*.json")))
    if not files:
        raise SystemExit(f"vectors/*.json 을 찾을 수 없습니다: {vec_dir} "
                          "(GenerateLabels.py --save-vectors 를 먼저 실행하세요)")

    out_dir = args.out or os.path.join(args.recording, "poly_targets")
    os.makedirs(out_dir, exist_ok=True)

    slot_present = [0] * NUM_SLOTS
    for path in files:
        with open(path, encoding="utf-8-sig") as fp:
            rec = json.load(fp)
        target = convert_frame(rec)
        for i, s in enumerate(target["slots"]):
            slot_present[i] += s["conf"]

        out_path = os.path.join(out_dir, os.path.basename(path))
        with open(out_path, "w", encoding="utf-8") as fp:
            json.dump(target, fp, ensure_ascii=False)

    print(f"{len(files)}프레임 변환 완료 → {out_dir}/*.json")
    for name, cnt in zip(SLOT_NAMES, slot_present):
        print(f"  {name:>10}: {cnt:5d}/{len(files)} 프레임에 존재 ({cnt/len(files)*100:.1f}%)")


if __name__ == "__main__":
    main()
