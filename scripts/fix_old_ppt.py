# -*- coding: utf-8 -*-
"""在旧版 PPT 上原位修改：保留原风格，只改文字与字号。
修正事实错误（公式编号、Δu、曝光页、20维、金字塔层数、杜撰数字），
并放大第 2/3/8/17/19 页的小字号正文。输出为新文件。"""
from pptx import Presentation
from pptx.util import Pt
import copy

SRC = "00_FAST-LIVO2 全景解析：机器人如何同时看路、辨向、建图 .pptx"
DST = "00_FAST-LIVO2 全景解析（原风格修订版）.pptx"

prs = Presentation(SRC)


def find(slide, sub):
    for sh in slide.shapes:
        if sh.has_text_frame and sub in sh.text_frame.text:
            return sh
    raise KeyError(f"not found: {sub}")


def sub_edit(slide, old, new, count=1):
    """在 run 内做子串替换（完整保留格式）"""
    sh = find(slide, old)
    done = 0
    for para in sh.text_frame.paragraphs:
        for r in para.runs:
            if old in r.text:
                r.text = r.text.replace(old, new)
                done += 1
                if done >= count:
                    return sh
    # 跨 run 的情况：整段替换
    return sh


def replace_lines(slide, anchor, new_lines):
    """按行替换：anchor 定位 shape；new_lines 逐段写回，保留每段首 run 格式"""
    sh = find(slide, anchor)
    tf = sh.text_frame
    paras = tf.paragraphs
    n = max(len(paras), len(new_lines))
    for i in range(n):
        if i < len(paras):
            p = paras[i]
            line = new_lines[i] if i < len(new_lines) else ""
            runs = p.runs
            if runs:
                runs[0].text = line
                for r in runs[1:]:
                    r.text = ""
            else:
                p.text = line
        else:
            p = tf.add_paragraph()
            p.text = new_lines[i]
            # 复制上一段格式
            src = paras[0].runs
            if src:
                f0 = src[0].font
                for r in p.runs:
                    r.font.size = f0.size
                    r.font.bold = f0.bold
                    r.font.name = f0.name
                    try:
                        r.font.color.rgb = f0.color.rgb
                    except Exception:
                        pass
    return sh


def bump(slide, min_from, to):
    """把字号 == min_from 的 run 放大到 to"""
    for sh in slide.shapes:
        if not sh.has_text_frame:
            continue
        for para in sh.text_frame.paragraphs:
            for r in para.runs:
                if r.font.size and abs(r.font.size.pt - min_from) < 0.3:
                    r.font.size = Pt(to)


S = prs.slides

# ============ 事实修正 ============
# 6 页：四种→三种
sub_edit(S[5], "四种传感器异步输入", "三种传感器异步输入")

# 9 页：公式编号纠错
sub_edit(S[8], "论文 Section IV-C 公式(3)-(5)", "论文 Section IV-C 式(1)-(3) 先验与传播")
sub_edit(S[8], "公式(3) 连续时间运动学模型", "式(1)(2) 连续时间运动学模型（离散化）")
sub_edit(S[8], "公式(4) 离散时间积分（每IMU步Δt）", "离散时间积分（式(1)的离散化，每IMU步Δt）")
sub_edit(S[8], "公式(5) 协方差传播：P_k^- = F·P_{k-1}·F^T + Q",
         "协方差传播：P_k^- = F·P_{k-1}·F^T + Q（先验分布即式(3)；ESIKF更新为式(11)）")

# 8 页：20个数字 → 19 维
sub_edit(S[7], "推算的状态（20个数字）", "推算的状态（19 维误差状态）")

# 26 页：Δu 解释纠错
sub_edit(S[25], "Δu = 曝光时间导致的亚像素偏移（公式14）",
         "Δu = Patch 内相对中心的像素偏移（式21）")

# 28 页：整页重写（旧页内容论文中不存在）
sub_edit(S[27], "EQUATION 14 · 公式(14) 通俗解释", "EXPOSURE · 曝光时间估计（论文 VII-B）")
sub_edit(S[27], "公式(14)：曝光时间多1ms，照片会糊多少？", "曝光时间估计：光照变了，亮度差怎么消？")
sub_edit(S[27], "∂r / ∂τ = −(图像梯度 ∇I) × (像素运动速度 v_img)",
         "r = τ_r·I_ref(u) − τ_k·I_cur(u')（式21-22）")
replace_lines(S[27], "拍照不是\u201d咔嚓\u201d一瞬间" if False else "快门打开τ时间",
              ["从走廊走进大厅，整体亮度突变。",
               "相机自动曝光(AE)调整曝光时间，",
               "同一点灰度 = 辐照度 × 曝光时间，",
               "灰度整体漂移 →",
               "光度残差出现系统性偏差。"])
sub_edit(S[27], "公式在算什么", "怎么做")
replace_lines(S[27], "∂r/∂τ = 曝光时间多1ms，",
              ["逆曝光时间 τ 作为第 19 维状态，",
               "与位姿在 ESIKF 中联合估计；",
               "残差中参考块乘 τ_r、当前块乘 τ_k；",
               "第一帧固定 τ₀=1（相对曝光）；",
               "对 τ 的雅可比 = 当前像素灰度。"])
sub_edit(S[27], "数字例子", "消融实验")
replace_lines(S[27], "角速度1rad/s，深度3m，",
              ["去掉曝光估计（25 条序列平均）：",
               "RMSE 从 0.044 m 升至 0.051 m",
               "（论文表 II）。",
               "Bright_Screen_Wall 等",
               "亮度突变序列正是目标场景。"])
replace_lines(S[27], "机器人在动(IMU知道ω,v)",
              ["光照变 → 灰度整体漂移 → τ 在 ESIKF 中与位姿联合估计 →",
               "τ 自动吸收亮度尺度差 → 光度残差恢复有效 →",
               "位姿估计不被亮度漂移污染 →",
               "曝光过程噪声 inv_expo_cov=0.1（avia.yaml）"])
replace_lines(S[27], "ESIKF把τ也当状态量在线估计",
              ["勘误：论文公式(14)是法向优化，与曝光无关。旧版把",
               "“∂r/∂τ=−∇I×v_img（运动模糊）”标为公式(14) 系误植，",
               "论文中不存在该式；曝光只通过 τ 缩放进入式(21)(22)，",
               "本页已按论文重写。"])

# 21 页：金字塔层数与杜撰数字
replace_lines(S[20], "· patch_size = 8（8×8像素）",
              ["· patch_size = 8（8×8像素）",
               "· 金字塔：论文三层（V-A/V-C），",
               "  代码 avia.yaml 配 4 层（level 0~3）",
               "· 从粗到精逐层迭代匹配",
               "· 每个视觉点存 1 个参考 Patch"])
replace_lines(S[20], "动态数量，无硬编码上限",
              ["动态数量，无固定上限；",
               "· 由 30×30 像素网格机制",
               "  控制空间分布（论文 V-C）",
               "· 无纹理墙面少、纹理丰富区多；",
               "· 用 std::vector 动态存储"])

# 22 页：杜撰数字
replace_lines(S[21], "图像划分为网格",
              ["图像划分为 30×30 像素网格",
               "（论文 V-C / VII-A2）",
               "空格用梯度最高的候选点",
               "生成新视觉点；",
               "注：grid_size=5、总数200~500",
               "为旧版误写，论文无此数字"])

# 35 页：金字塔层数与网格
sub_edit(S[34], "金字塔迭代(4层)，从粗到精对齐", "金字塔迭代由粗到精（论文三层/代码4层）")
sub_edit(S[34], "网格均匀采样，约28像素一格", "网格均匀采样，30×30 像素一格（论文 VII-A2）")
sub_edit(S[34], "① 图像金字塔构建：4层，从原图(level 0)到1/8分辨率(level 3)",
         "① 图像金字塔构建（论文三层/代码4层），从原图到低分辨率")

# 29 页：20 维状态口径
sub_edit(S[28], "状态向量（20维）", "状态向量（论文 dim(M)=19，代码 DIM_STATE=19）")
sub_edit(S[28], "姿态(四元数4) + 位置(3) + 速度(3)", "旋转 R(3) + 位置(3) + 速度(3)")

# ============ 可读性：放大第 2/3/8/17/19 页小字 ============
bump(S[1], 13, 15)    # 学习路线副标题
bump(S[2], 15, 17)    # 章节问题句
bump(S[7], 13, 14.5)  # IMU 页正文
bump(S[7], 12, 13.5)
bump(S[16], 11.5, 13)  # 特征值扰动页正文
bump(S[16], 12, 13.5)
bump(S[18], 13, 14.5)  # 直接法页正文

prs.save(DST)
print("saved", DST)
