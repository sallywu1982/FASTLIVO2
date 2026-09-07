# -*- coding: utf-8 -*-
"""在第19页后插入“视觉点是什么”概念页：克隆第21页的卡片版式，重写文字。"""
import copy
from pptx import Presentation

F = "00_FAST-LIVO2 全景解析（原风格修订版）.pptx"
prs = Presentation(F)
S = prs.slides

TEMPLATE = S[20]  # VISUAL POINT MANAGEMENT 卡片页


def set_text(sh, lines):
    paras = sh.text_frame.paragraphs
    for i, p in enumerate(paras):
        line = lines[i] if i < len(lines) else ""
        if p.runs:
            p.runs[0].text = line
            for r in p.runs[1:]:
                r.text = ""
        else:
            p.text = line


# 1) 克隆模板页到末尾
new = prs.slides.add_slide(TEMPLATE.slide_layout)
for sh in list(new.shapes):
    sh._element.getparent().remove(sh._element)
for sh in TEMPLATE.shapes:
    new.shapes._spTree.append(copy.deepcopy(sh._element))

# 2) 重写文字（形状顺序与模板一致：0 tag / 1 title / 4,5 卡1 / 7,8 卡2 / 10,11 卡3 / 13,14 底部 / 15 页脚）
texts = {
    0: ["VISUAL MAP POINT · 视觉地图点（论文 V-A / V-C）"],
    1: ["视觉点是什么：激光给位置，相机给外观"],
    4: ["它是什么"],
    5: ["视觉点 = 挂在激光点上的图像块",
        "· 3D 位置：来自 LiDAR 点云",
        "  （毫米级、无尺度漂移、无需三角化）",
        "· 外观：参考 Patch（8×8）+ 历史观测块",
        "  + 每次观测时的曝光时间",
        "· 归属：挂在体素地图的叶节点上"],
    7: ["它怎么来（V-C）"],
    8: ["激光先建好体素平面；",
        "视觉更新后，在 30×30 像素网格的空格中，",
        "挑灰度梯度最高、视角合规的候选激光点，",
        "生成视觉点并附加当前帧 Patch、",
        "估计状态（位姿+曝光）与平面法向；",
        "已有点：超 20 帧或偏移超 40px 时追加新块"],
    10: ["它怎么用"],
    11: ["· LIO 配准：用它的坐标参与点到平面残差",
         "· VIO 对齐：用它的参考 Patch 参与光度残差",
         "  （式 21-22）",
         "· 仿射扭曲：用它所在平面的法向 n 构造",
         "  单应矩阵（式 13），视角变化仍能对齐",
         "· 同一个点，两套传感器各取所需"],
    13: ["一句话理解"],
    14: ["视觉点 = 激光测准了位置的 3D 点，被相机反复观察并记下了“长什么样”。",
         "它不是纯激光点（原始点配准后不需要外观），也不是纯视觉特征点（位置不靠三角化）；",
         "它是从激光点云中选出“相机也能持续看到”的点升级成的混合实体，",
         "与所在体素的平面（几何层）共享同一空间索引——这就是“统一体素地图”的含义。"],
    15: ["论文对应：Section V-A, V-C, VII-A | 代码：voxel_map.h (VisualPoint), vio.cpp generateVisualMapPoints / updateVisualMapPoints / updateReferencePatch"],
}
for j, lines in texts.items():
    set_text(new.shapes[j], lines)

# 3) 移动到第 19 页之后（新下标 19）
xml_slides = prs.slides._sldIdLst
ids = list(xml_slides)
last = ids[-1]
xml_slides.remove(last)
xml_slides.insert(19, last)

prs.save(F)
print("inserted, total slides:", len(prs.slides._sldIdLst))
