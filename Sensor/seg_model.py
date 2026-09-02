"""차선 세그멘테이션 모델 — ResNet 인코더 + U-Net 디코더.

--------------------------------------------------------------------------
왜 U-Net 형태인가
--------------------------------------------------------------------------
차선은 **얇고 긴 구조**다. 인코더만 쓰고 1/32 특징을 곧장 확대하면 폭 3~7px
짜리 선이 뭉개진다. 스킵 연결로 1/4·1/8 의 고해상 특징을 되살려야 선의
위치가 픽셀 단위로 살아남는다.

--------------------------------------------------------------------------
손실: 가중 CE + Dice
--------------------------------------------------------------------------
차선 픽셀이 전체의 1.9% 뿐이라(seg_dataset 실측) 그냥 CE 로는 전부 배경으로
수렴한다. 두 가지를 같이 쓴다.

    가중 CE   픽셀 단위. 희소 클래스에 큰 가중치(ENet 방식, 상한 있음).
    Dice      영역 겹침 단위. **클래스 크기에 자동으로 정규화**되므로 정지선
              (0.063%)처럼 극단적으로 희소한 클래스에서 CE 보다 안정적이다.

CE 만 쓰면 가중치를 아무리 올려도 얇은 클래스의 경계가 흐려지고, Dice 만
쓰면 초반 수렴이 불안정하다. 둘을 더하는 게 세그멘테이션의 표준 조합이다.

255(ignore) 는 **두 손실 모두에서** 빠져야 한다. CE 는 ignore_index 로,
Dice 는 마스크를 곱해서 뺀다.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18, resnet34, resnet50

from seg_dataset import CLASS_WEIGHTS, IGNORE_INDEX, NUM_CLASSES

_BACKBONES = {"resnet18": resnet18, "resnet34": resnet34, "resnet50": resnet50}
# layer1..layer4 의 출력 채널 수
_CHANNELS = {"resnet18": (64, 128, 256, 512), "resnet34": (64, 128, 256, 512),
             "resnet50": (256, 512, 1024, 2048)}


class _Up(nn.Module):
    """2배 확대 후 스킵과 이어붙이고 conv 두 번."""

    def __init__(self, in_ch, skip_ch, out_ch):
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_ch + skip_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch), nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch), nn.ReLU(inplace=True))

    def forward(self, x, skip=None):
        x = F.interpolate(x, scale_factor=2, mode="nearest")
        if skip is not None:
            # 입력 크기가 32 의 배수가 아니면 1px 어긋날 수 있다
            if x.shape[-2:] != skip.shape[-2:]:
                x = F.interpolate(x, size=skip.shape[-2:], mode="nearest")
            x = torch.cat([x, skip], dim=1)
        return self.block(x)


class LaneSegNet(nn.Module):
    def __init__(self, backbone="resnet34", pretrained=True, num_classes=NUM_CLASSES):
        super().__init__()
        if backbone not in _BACKBONES:
            raise ValueError(f"모르는 백본: {backbone} (가능: {sorted(_BACKBONES)})")
        net = _BACKBONES[backbone](weights="DEFAULT" if pretrained else None)
        c1, c2, c3, c4 = _CHANNELS[backbone]

        self.stem = nn.Sequential(net.conv1, net.bn1, net.relu)   # 1/2, 64ch
        self.pool = net.maxpool                                    # 1/4
        self.layer1, self.layer2 = net.layer1, net.layer2          # 1/4, 1/8
        self.layer3, self.layer4 = net.layer3, net.layer4          # 1/16, 1/32

        self.up4 = _Up(c4, c3, 256)     # 1/16
        self.up3 = _Up(256, c2, 128)    # 1/8
        self.up2 = _Up(128, c1, 64)     # 1/4
        self.up1 = _Up(64, 64, 32)      # 1/2  (stem 과 이어붙임)
        self.up0 = _Up(32, 0, 16)       # 1/1
        self.head = nn.Conv2d(16, num_classes, 1)

    def forward(self, x):
        s = self.stem(x)                # 1/2
        f1 = self.layer1(self.pool(s))  # 1/4
        f2 = self.layer2(f1)            # 1/8
        f3 = self.layer3(f2)            # 1/16
        f4 = self.layer4(f3)            # 1/32
        d = self.up4(f4, f3)
        d = self.up3(d, f2)
        d = self.up2(d, f1)
        d = self.up1(d, s)
        d = self.up0(d)
        return self.head(d)             # [B, C, H, W] 로짓


class SegLoss(nn.Module):
    def __init__(self, weights=None, w_ce=1.0, w_dice=1.0):
        super().__init__()
        w = torch.tensor(weights if weights is not None else CLASS_WEIGHTS,
                         dtype=torch.float32)
        self.register_buffer("class_weight", w)
        self.w_ce, self.w_dice = w_ce, w_dice

    def forward(self, logits, target):
        ce = F.cross_entropy(logits, target, weight=self.class_weight,
                             ignore_index=IGNORE_INDEX)

        # Dice 는 ignore 픽셀을 통째로 빼고 계산한다.
        valid = target != IGNORE_INDEX
        if not valid.any():
            return self.w_ce * ce, {"ce": float(ce.detach()), "dice": 0.0}
        t = torch.where(valid, target, torch.zeros_like(target))
        onehot = F.one_hot(t, logits.shape[1]).permute(0, 3, 1, 2).float()
        m = valid.unsqueeze(1).float()
        prob = logits.softmax(dim=1) * m
        onehot = onehot * m
        dims = (0, 2, 3)
        inter = (prob * onehot).sum(dims)
        denom = prob.sum(dims) + onehot.sum(dims)
        # 그 배치에 아예 없는 클래스는 평균에서 뺀다 (0 으로 끌어내리지 않게)
        present = onehot.sum(dims) > 0
        dice_c = 1.0 - (2 * inter + 1.0) / (denom + 1.0)
        dice = dice_c[present].mean() if present.any() else dice_c.mean() * 0

        total = self.w_ce * ce + self.w_dice * dice
        return total, {"ce": float(ce.detach()), "dice": float(dice.detach())}


@torch.no_grad()
def confusion_update(conf, logits, target, num_classes=NUM_CLASSES):
    """혼동행렬 누적. IoU 는 여기서 파생한다.

    **정확도(accuracy)를 지표로 쓰면 안 된다** — 배경이 97% 라 전부 배경이라고
    찍어도 97% 가 나온다. 클래스별 IoU 를 봐야 한다.
    """
    pred = logits.argmax(1)
    valid = target != IGNORE_INDEX
    p, t = pred[valid].view(-1), target[valid].view(-1)
    idx = t * num_classes + p
    conf += torch.bincount(idx, minlength=num_classes ** 2).view(
        num_classes, num_classes).to(conf.dtype)
    return conf


def iou_from_confusion(conf):
    """클래스별 IoU = TP / (TP + FP + FN). 그 split 에 없는 클래스는 nan."""
    conf = conf.double()
    tp = conf.diag()
    fp = conf.sum(0) - tp
    fn = conf.sum(1) - tp
    denom = tp + fp + fn
    iou = torch.where(denom > 0, tp / denom.clamp(min=1),
                      torch.full_like(denom, float("nan")))
    return iou
