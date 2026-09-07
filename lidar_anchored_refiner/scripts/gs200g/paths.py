"""Data path discovery for the GS-200G 二楼室外 project.

All paths are resolved via glob so that non-ASCII directory names never need
to be typed literally (the surrounding shell mangles them).
"""
from __future__ import annotations

import glob
import os

ROOT = "E:/StoneRecord/ws-Work/02_Image2Image"

# Both "20260807_ImageData" and the Chinese-named precision-test dataset match
# "20260807_*"; the one we want is the non-ASCII (Chinese) directory.
_CANDIDATES = [d for d in glob.glob(os.path.join(ROOT, "20260807_*"))
               if any(ord(ch) > 0x80 for ch in os.path.basename(d))]
assert len(_CANDIDATES) == 1, f"expected one Chinese-named 20260807_* dir, got {_CANDIDATES}"
DATASET_DIR = _CANDIDATES[0]

_PROJECT_DIRS = [d for d in glob.glob(os.path.join(DATASET_DIR, "Project_*")) if os.path.isdir(d)]
assert len(_PROJECT_DIRS) == 1, f"expected exactly one Project_* dir, got {_PROJECT_DIRS}"
PROJECT_DIR = _PROJECT_DIRS[0]

RESULT_OUT = os.path.join(PROJECT_DIR, "ResultOut")
PARA_DIR = os.path.join(PROJECT_DIR, "Para")
IMAGE_DIR = os.path.join(PROJECT_DIR, "Image")
CAMERAS = ("L", "M", "R")

LAS_ORIGINAL = os.path.join(RESULT_OUT, "二楼室外.las")
POS_ECEF = os.path.join(PROJECT_DIR, "二楼室外_pos.txt")
POS_ENH = os.path.join(PROJECT_DIR, "二楼室外_pos_ENH.txt")

# Outputs of the M1 pipeline live next to this package.
WORK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "work"))
os.makedirs(WORK_DIR, exist_ok=True)


def intrinsic_yaml(cam: str) -> str:
    return os.path.join(PARA_DIR, f"{cam}_Intrinsic.yaml")


def extrinsic_yaml(cam: str) -> str:
    return os.path.join(PARA_DIR, f"{cam}_CamToMotor.yaml")


def image_dir(cam: str) -> str:
    return os.path.join(IMAGE_DIR, cam)


def list_images(cam: str) -> list[str]:
    files = sorted(glob.glob(os.path.join(image_dir(cam), "*-M.JPG" if cam == "M" else f"*-{cam}.JPG")))
    return files


if __name__ == "__main__":
    print("PROJECT_DIR =", PROJECT_DIR)
    print("LAS_ORIGINAL exists:", os.path.exists(LAS_ORIGINAL))
    print("POS_ECEF exists:", os.path.exists(POS_ECEF))
    print("POS_ENH exists:", os.path.exists(POS_ENH))
    for c in CAMERAS:
        imgs = list_images(c)
        print(f"{c}: {len(imgs)} images", imgs[0] if imgs else "-", "..", imgs[-1] if imgs else "-")
