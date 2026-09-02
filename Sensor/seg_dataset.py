"""세그멘테이션용 데이터셋 — frames/*.png + masks/*.png.

--------------------------------------------------------------------------
왜 회귀에서 넘어왔는가
--------------------------------------------------------------------------
앵커 회귀(dataset.py)는 슬롯마다 **반드시** 32개 앵커를 채워야 한다. 차선이
잘 안 보이는 구간에서도 뭔가를 뱉어야 하고, 관측 구간 밖 앵커는 손실에서
빠지므로(`lane_xs_valid`) **감독을 전혀 못 받은 값이 후처리로 흘러들어가**
화면을 가로지르는 곡선이 됐다. 2슬롯 축소 + ignore + min_range 안전장치까지
넣어도 전 프레임에서 남았다.

세그멘테이션은 **모든 출력 픽셀이 감독을 받는다.** 차선이 없으면 배경으로
학습되고, 애매하면 255(ignore)로 손실에서 빠진다. "모를 때 아무거나 뱉는"
구조적 여지가 없다.

--------------------------------------------------------------------------
마스크 규격 (GenerateLabels.TRAIN_CLASS_MAPS["lane5"])
--------------------------------------------------------------------------
    0 배경   1 백색실선   2 백색점선   3 황색   4 정지선   255 ignore

유도선/안전지대/횡단보도는 이미 255 로 접혀 있다. 보닛과 오브젝트 가려짐도
255 다 — "불확실하면 ignore" 원칙.

마스크는 **이미 crop 이 적용된 1280x460** 이고 프레임은 원본 1280x720 이다.
crop_top 은 둘의 높이 차이로 구한다(=260). poly_targets 를 안 읽어도 되도록
일부러 이렇게 유도한다.

--------------------------------------------------------------------------
클래스 불균형 (12랩 543장 실측)
--------------------------------------------------------------------------
    배경 76.90%   백색실선 1.186%   백색점선 0.131%
                  황색     0.560%   정지선   0.063%   ignore 21.16%

차선 전체가 1.9% 뿐이라 **가중치 없이 학습하면 전부 배경으로 수렴한다.**
"""

import glob
import os

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

NUM_CLASSES = 5
IGNORE_INDEX = 255
CLASS_NAMES = ["background", "white_solid", "white_dashed", "yellow", "stopline"]

# 모델 입력 크기 (w, h). 원본 크롭본이 1280x460 이라 세로가 조금 눌리지만,
# ResNet 계열은 32 의 배수여야 다운샘플이 깔끔하다. 차선은 얇은 구조라
# 이보다 더 줄이면 선이 1~2px 이 되어 학습이 어려워진다.
INPUT_W, INPUT_H = 640, 256

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

# ENet 방식 w = 1/ln(1.02 + freq). 역빈도(1/freq)를 그대로 쓰면 정지선이
# 배경의 1200배가 되어 학습이 발산한다. 로그를 씌워 상한을 둔다.
# 위 실측 분포에서 계산한 값이다 (ignore 제외 재정규화 후).
CLASS_WEIGHTS = [1.45, 29.04, 46.67, 37.40, 48.58]


def _resolve_crop_top(frame_h, mask_h):
    """마스크는 crop 된 것이고 프레임은 원본이다. 차이가 crop_top."""
    top = frame_h - mask_h
    if top < 0:
        raise RuntimeError(f"마스크({mask_h})가 프레임({frame_h})보다 큽니다")
    return top


class SegDataset(Dataset):
    """한 항목 = (이미지 텐서 [3,H,W], 라벨 텐서 [H,W] int64).

    라벨의 255 는 그대로 둔다 — CrossEntropyLoss(ignore_index=255) 가 뺀다.
    """

    def __init__(self, recordings, augment=False):
        self.items = []
        for rec in recordings:
            for mpath in sorted(glob.glob(os.path.join(rec, "masks", "*.png"))):
                idx = os.path.splitext(os.path.basename(mpath))[0]
                img = os.path.join(rec, "frames", f"{idx}.png")
                if os.path.isfile(img):
                    self.items.append((mpath, img))
        self.augment = augment
        if not self.items:
            raise SystemExit(f"학습 항목이 없습니다: {recordings}")

    def __len__(self):
        return len(self.items)

    def __getitem__(self, i):
        mpath, ipath = self.items[i]
        mask = cv2.imread(mpath, cv2.IMREAD_GRAYSCALE)
        img = cv2.imread(ipath)
        if mask is None or img is None:
            raise RuntimeError(f"읽지 못했습니다: {mpath} / {ipath}")

        img = img[_resolve_crop_top(img.shape[0], mask.shape[0]):]

        # 라벨은 **반드시 최근접 보간**이다. 선형 보간을 쓰면 클래스 번호가
        # 섞여 없는 클래스가 생기고, 255(ignore)가 차선 위로 번진다.
        mask = cv2.resize(mask, (INPUT_W, INPUT_H), interpolation=cv2.INTER_NEAREST)
        img = cv2.resize(img, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)

        if self.augment:
            img = self._augment_photometric(img)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = (img - IMAGENET_MEAN) / IMAGENET_STD
        return (torch.from_numpy(img.transpose(2, 0, 1).copy()),
                torch.from_numpy(mask.astype(np.int64)))

    @staticmethod
    def _augment_photometric(img):
        """밝기/대비만 흔든다. **좌우 반전은 쓰면 안 된다** — 중앙선(황색)이
        항상 왼쪽에 오는 좌우 비대칭 구조라 반전하면 라벨이 거짓이 된다."""
        a = 1.0 + np.random.uniform(-0.25, 0.25)
        b = np.random.uniform(-25, 25)
        return np.clip(img.astype(np.float32) * a + b, 0, 255).astype(np.uint8)


def split_by_recording(learning_dir, val_laps, train_laps=None):
    """**랩 단위로 나눈다.** 인접 프레임은 0.n초 간격이라 거의 같은 장면이고,
    프레임 단위 랜덤 분할은 사실상 본 장면으로 평가하게 된다(leakage).
    """
    laps = sorted(d for d in os.listdir(learning_dir)
                  if os.path.isdir(os.path.join(learning_dir, d, "masks")))
    missing = set(val_laps) - set(laps)
    if missing:
        raise SystemExit(f"val 랩을 찾을 수 없습니다: {sorted(missing)} (가능: {laps})")

    val = [l for l in laps if l in val_laps]
    if train_laps:
        missing = set(train_laps) - set(laps)
        if missing:
            raise SystemExit(f"train 랩을 찾을 수 없습니다: {sorted(missing)} (가능: {laps})")
        overlap = set(train_laps) & set(val_laps)
        if overlap:
            raise SystemExit(f"train 과 val 에 같은 랩이 있습니다: {sorted(overlap)}")
        train = [l for l in laps if l in train_laps]
    else:
        train = [l for l in laps if l not in val_laps]
    if not train:
        raise SystemExit("학습할 랩이 없습니다")

    to_path = lambda names: [os.path.join(learning_dir, n) for n in names]
    return to_path(train), to_path(val), train, val
