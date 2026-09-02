#!/usr/bin/env python3
"""세그멘테이션 결과를 눈으로 확인한다.

    python seg_predict.py --checkpoint seg_runs/best.pt --lap ../learning/13sunny1 \
        --frames 002703,000836

한 장에 세 칸을 세로로 쌓는다.

    위   원본 + 예측 (색 오버레이)
    가운데 원본 + 정답
    아래  차이 — 초록=맞음, 빨강=놓침(FN), 파랑=잘못 그림(FP), 회색=ignore

회귀 모델과 달리 **곡선을 그리지 않는다.** 차선이 없다고 판단하면 아무 색도
칠하지 않는다 — 세그멘테이션으로 넘어온 이유가 이 "기권 가능" 성질이다.
"""

import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np
import torch

from seg_dataset import (CLASS_NAMES, IGNORE_INDEX, INPUT_H, INPUT_W, NUM_CLASSES,
                         SegDataset)
from seg_model import LaneSegNet

# BGR. 배경은 안 칠한다.
CLASS_COLORS = {
    1: (255, 255, 255),   # 백색실선
    2: (255, 200, 0),     # 백색점선 (하늘)
    3: (0, 200, 255),     # 황색
    4: (0, 0, 255),       # 정지선 (빨강)
}


def build_arg_parser():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="세그멘테이션 결과를 그려 본다")
    ap.add_argument("--checkpoint", default=os.path.join(here, "seg_runs", "best.pt"))
    ap.add_argument("--lap", required=True)
    ap.add_argument("--count", type=int, default=8, help="고르게 뽑을 프레임 수")
    ap.add_argument("--frames", default=None, help="특정 프레임 번호들 (쉼표 구분)")
    ap.add_argument("--out", default=None, help="출력 폴더 (기본: <lap>/seg_check)")
    ap.add_argument("--alpha", type=float, default=0.55, help="오버레이 진하기")
    ap.add_argument("--dilate", type=int, default=1,
                    help="얇은 선이 화면에서 잘 보이도록 굵히는 정도 (0=원본)")
    return ap


def colorize(mask, dilate=1):
    """클래스 마스크를 BGR 색과 알파로 바꾼다."""
    h, w = mask.shape
    color = np.zeros((h, w, 3), np.uint8)
    hit = np.zeros((h, w), bool)
    for c, bgr in CLASS_COLORS.items():
        m = mask == c
        if dilate > 0 and m.any():
            m = cv2.dilate(m.astype(np.uint8), np.ones((2 * dilate + 1,) * 2, np.uint8)) > 0
        color[m] = bgr
        hit |= m
    return color, hit


def overlay(frame, mask, alpha, dilate, label):
    color, hit = colorize(mask, dilate)
    vis = frame.copy()
    vis[hit] = (vis[hit] * (1 - alpha) + color[hit] * alpha).astype(np.uint8)
    cv2.rectangle(vis, (0, 0), (vis.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(vis, label, (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return vis


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ck = torch.load(args.checkpoint, map_location=device)
    model = LaneSegNet(ck["args"].get("backbone", "resnet34"), pretrained=False).to(device)
    model.load_state_dict(ck["model"])
    model.eval()
    print(f"[seg_predict] {args.checkpoint}  epoch {ck['epoch']}")
    print("[seg_predict] 학습 당시: "
          + "  ".join(f"{k} {v:.4f}" for k, v in ck["metrics"].items()))
    if "iou" in ck:
        print("[seg_predict] 클래스별 IoU: "
              + "  ".join(f"{CLASS_NAMES[c]} {ck['iou'][c]:.3f}"
                          for c in range(1, NUM_CLASSES)))

    ds = SegDataset([args.lap], augment=False)
    if args.frames:
        want = {x.strip() for x in args.frames.split(",") if x.strip()}
        picks = [i for i, (m, _) in enumerate(ds.items)
                 if os.path.splitext(os.path.basename(m))[0] in want]
        if not picks:
            raise SystemExit(f"그 프레임을 못 찾았습니다: {sorted(want)}")
    else:
        picks = list(np.linspace(0, len(ds) - 1, args.count).astype(int))

    out_dir = args.out or os.path.join(args.lap, "seg_check")
    os.makedirs(out_dir, exist_ok=True)

    inter = np.zeros(NUM_CLASSES); union = np.zeros(NUM_CLASSES)
    for i in picks:
        mpath, ipath = ds.items[i]
        idx = os.path.splitext(os.path.basename(mpath))[0]
        image, target = ds[i]
        with torch.no_grad():
            pred = model(image.unsqueeze(0).to(device)).argmax(1)[0].cpu().numpy()
        gt = target.numpy()

        frame = cv2.imread(ipath)
        frame = frame[frame.shape[0] - cv2.imread(mpath, 0).shape[0]:]
        frame = cv2.resize(frame, (INPUT_W, INPUT_H))

        valid = gt != IGNORE_INDEX
        for c in range(NUM_CLASSES):
            p, t = (pred == c) & valid, (gt == c) & valid
            inter[c] += (p & t).sum(); union[c] += (p | t).sum()

        # 차이 그림: 초록 맞음 / 빨강 놓침 / 파랑 헛것 / 회색 ignore
        diff = np.zeros((*gt.shape, 3), np.uint8)
        pl, tl = (pred > 0) & valid, (gt > 0) & valid
        diff[pl & tl] = (0, 255, 0)
        diff[~pl & tl] = (0, 0, 255)
        diff[pl & ~tl] = (255, 0, 0)
        diff[~valid] = (90, 90, 90)
        if args.dilate:
            k = np.ones((3, 3), np.uint8)
            diff = cv2.dilate(diff, k)
        cv2.rectangle(diff, (0, 0), (diff.shape[1], 22), (0, 0, 0), -1)
        cv2.putText(diff, "diff  green=hit  red=missed  blue=false  gray=ignore",
                    (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        stack = np.vstack([
            overlay(frame, pred, args.alpha, args.dilate, f"idx={idx}  PRED"),
            overlay(frame, gt, args.alpha, args.dilate, "GT"),
            diff])
        cv2.imwrite(os.path.join(out_dir, f"seg_{idx}.png"), stack)

    print(f"\n{len(picks)}장 저장: {out_dir}")
    print("이 표본에서의 IoU:")
    for c in range(1, NUM_CLASSES):
        v = inter[c] / union[c] if union[c] else float("nan")
        print(f"  {CLASS_NAMES[c]:>12}: {v:.3f}" + ("   (표본에 없음)" if not union[c] else ""))


if __name__ == "__main__":
    main()
