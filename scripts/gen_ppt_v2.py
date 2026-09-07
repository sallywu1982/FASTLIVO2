# -*- coding: utf-8 -*-
"""生成《FAST-LIVO2 全景解析（修订版）》PPT：修正旧版公式编号等错误，结构更清晰。"""
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR

NAVY = RGBColor(0x1F, 0x38, 0x64)
BLUE = RGBColor(0x2E, 0x75, 0xB6)
GRAY = RGBColor(0x50, 0x50, 0x50)
LIGHT = RGBColor(0xF2, 0xF5, 0xFA)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
RED = RGBColor(0xC0, 0x39, 0x2B)

prs = Presentation()
prs.slide_width = Inches(13.333)
prs.slide_height = Inches(7.5)
BLANK = prs.slide_layouts[6]
SW, SH = prs.slide_width, prs.slide_height


def box(slide, x, y, w, h, fill=None):
    from pptx.enum.shapes import MSO_SHAPE
    sh = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, x, y, w, h)
    sh.shadow.inherit = False
    if fill is None:
        sh.fill.background()
    else:
        sh.fill.solid(); sh.fill.fore_color.rgb = fill
    sh.line.fill.background()
    return sh


def tx(slide, x, y, w, h, runs, size=16, color=GRAY, bold=False,
       anchor=MSO_ANCHOR.TOP, spacing=1.0):
    tb = slide.shapes.add_textbox(x, y, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    if isinstance(runs, str):
        runs = [[(runs, {})]]
    elif runs and isinstance(runs[0], tuple):
        runs = [runs]
    first = True
    for para in runs:
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        p.line_spacing = spacing
        if isinstance(para, str):
            para = [(para, {})]
        for t, st in para:
            r = p.add_run(); r.text = t
            r.font.size = Pt(st.get("size", size))
            r.font.bold = st.get("bold", bold)
            r.font.color.rgb = st.get("color", color)
            r.font.name = "Microsoft YaHei"
    return tb


def header(slide, tag, title):
    box(slide, 0, 0, SW, Inches(1.15), fill=NAVY)
    tx(slide, Inches(0.5), Inches(0.08), Inches(11), Inches(0.4), tag.upper(), size=12,
       color=RGBColor(0x9D, 0xC3, 0xE6), bold=True)
    tx(slide, Inches(0.5), Inches(0.38), Inches(12.3), Inches(0.7), title, size=26,
       color=WHITE, bold=True)


def footer(slide, text):
    tx(slide, Inches(0.5), Inches(7.08), Inches(12.3), Inches(0.35), text, size=10,
       color=RGBColor(0xAA, 0xAA, 0xAA))


def content(tag, title, paras, note=None):
    s = prs.slides.add_slide(BLANK)
    header(s, tag, title)
    tx(s, Inches(0.6), Inches(1.45), Inches(12.1), Inches(5.4), paras, spacing=1.15)
    if note:
        footer(s, note)
    return s


def cards(tag, title, items, note=None, h=1.5):
    s = prs.slides.add_slide(BLANK)
    header(s, tag, title)
    y = Inches(1.45)
    for t, b, c in items:
        box(s, Inches(0.5), y, Inches(0.12), h, fill=c)
        box(s, Inches(0.62), y, Inches(12.2), h, fill=LIGHT)
        tx(s, Inches(0.9), y + Inches(0.06), Inches(11.7), Inches(0.45), t, size=19,
           color=NAVY, bold=True)
        tx(s, Inches(0.9), y + Inches(0.52), Inches(11.7), h - Inches(0.55), b, size=15,
           spacing=1.1)
        y += h + Inches(0.16)
    if note:
        footer(s, note)
    return s


B = {"bold": True, "color": NAVY, "size": 17}
E = {"color": RED}

# 1 封面
s = prs.slides.add_slide(BLANK)
box(s, 0, 0, SW, SH, fill=NAVY)
box(s, 0, Inches(4.9), SW, Inches(0.06), fill=BLUE)
tx(s, Inches(1), Inches(1.8), Inches(11.3), Inches(0.5),
   "TECHNICAL DEEP DIVE · 修订版（公式编号与事实已对照论文原文及开源代码核验）",
   size=14, color=RGBColor(0x9D, 0xC3, 0xE6))
tx(s, Inches(1), Inches(2.3), Inches(11.3), Inches(1.2), "FAST-LIVO2 全景解析",
   size=48, color=WHITE, bold=True)
tx(s, Inches(1), Inches(3.6), Inches(11.3), Inches(0.6),
   "机器人如何同时看路、辨向、建图 —— 激光雷达 · IMU · 相机的紧耦合融合",
   size=20, color=RGBColor(0xBD, 0xD7, 0xEE))
tx(s, Inches(1), Inches(5.2), Inches(11.3), Inches(0.5),
   "论文：arXiv 2408.14035v2　|　代码：github.com/hku-mars/FAST-LIVO2（港大 MaRS 实验室）",
   size=14, color=RGBColor(0x9D, 0xC3, 0xE6))

# 2 学习路线
cards("Learning Roadmap", "沿着数据流走一遍：五站旅程",
      [("① 数据入场", "扫描重组对齐三传感器（IV-B）；IMU 前向传播给初值、反向传播去畸变（IV-C）", BLUE),
       ("② 激光更新（粗修正）", "原始点直接找地图平面，算点到平面残差（VI）；ESIKF 迭代到收敛（IV-D）", NAVY),
       ("③ 视觉更新（精修）", "直接法比亮度：平面先验扭曲参考块 → 光度残差 + 曝光估计（VII）", BLUE),
       ("④ 地图维护", "统一体素地图：几何层（平面）+ 视觉层（参考块）（V）", NAVY)],
      h=1.15)

# 3 传感器
cards("Sensors", "三个队员：各有所长，也各有短板",
      [("激光雷达 LiDAR · 10~100Hz", "擅长：精确测距，厘米级\n短板：无颜色；长廊、隧道等几何约束不足时退化", NAVY),
       ("IMU · 100~400Hz", "擅长：高频反应快，两次更新间推算位姿\n短板：积分漂移，越走越偏", BLUE),
       ("相机 Camera · 10~40Hz", "擅长：纹理与颜色信息密度高\n短板：单张图无距离；低光照、运动模糊下失效", NAVY)],
      h=1.55,
      note="三种传感器（注意：不是四种）。频率范围参考论文 Fig.2 与各数据集配置。")

# 4 总体框架
content("Framework", "一图看懂工作流（论文 Fig.1, Section III）",
        [[("三种传感器异步输入", B)],
         [("→ 扫描重组：以图像时刻切分激光点（IV-B；代码 LIVMapper.cpp sync_packages）", {})],
         [("→ IMU 前向传播给出位姿先验 + 反向传播去畸变（IV-C；IMU_Processing.cpp Process2）", {})],
         [("→ 激光更新：点到平面残差，ESIKF 迭代（VI；voxel_map.cpp StateEstimation）", {})],
         [("→ 视觉更新：稀疏直接光度对齐（VII；vio.cpp processFrame）", {})],
         [("→ 统一体素地图同时管理几何与视觉信息（V）", {})],
         [("", {})],
         [("关键架构", B)],
         [("• 单一 19 维状态向量（论文 dim(M)=19；代码 DIM_STATE=19），激光与视觉共用", {})],
         [("• 顺序更新：先激光后视觉，串行执行、共享协方差 P —— 论文式 (5)-(8) 的直接体现", {})],
         [("• 代码主线：LIVMapper.cpp:267 按 lio_vio_flg 状态机交替分发 handleLIO / handleVIO", {})]],
        note="修订：旧版“四种传感器”系笔误；状态维度以论文 dim(M)=19 为准。")

# 5 扫描重组
content("Step 1 · 数据入场", "扫描重组：把三种数据对齐到同一时刻（论文 IV-B）",
        [[("问题", B)],
         [("• LiDAR 一帧内每个点采集时刻都不同（扫描需要时间）；相机是曝光时段的积分而非时刻快照", {})],
         [("• IMU 200Hz、LiDAR 10Hz、相机 10Hz，心跳完全不同", {})],
         [("", {})],
         [("做法", B)],
         [("• 以图像时刻为切割点，把跨越该时刻的激光扫描“切一刀”：前半归当前组，后半留给下一帧", {})],
         [("• IMU 数据按时间区间提取，用于该组的传播", {})],
         [("• 效果：相机与激光同频（如 10Hz），每次 ESIKF 更新对应严格同一时刻", {})],
         [("", {})],
         [("代码", B)],
         [("LIVMapper.cpp:884-1119 sync_packages()：状态机在 LIO/VIO 间交替产出测量组（:946-1075）", {})]])

# 6 IMU 传播
content("Step 2 · IMU 预测", "前向传播猜位姿，反向传播去抖动（论文 IV-C）",
        [[("前向传播（状态 + 协方差）", B)],
         [("• 式 (1)：x = x ⊞ (Δt·f(x,u,w))，传播时置噪声 w=0，得预测 x̂ 与 P̂ 作为更新先验（式 (3)）", {})],
         [("• 离散积分：R←R·Exp((ω−b_g)Δt)；v、p 逐步积分；协方差按 F、Q 传播", {})],
         [("• 代码：IMU_Processing.cpp:334-341（中值积分）、:382-401（协方差，曝光噪声在 cov_w(6,6)）", {})],
         [("", {})],
         [("反向传播（点云去畸变）", B)],
         [("• 扫描期间机器人在动 → 各点在不同位姿下采集，直接拼接会“糊”", {})],
         [("• 从帧尾反向遍历每个点的采集时刻，用 IMU 位姿把点搬到帧尾位姿下 → 等效同一瞬间拍摄", {})],
         [("• 代码：IMU_Processing.cpp UndistortPcl（~:490-535）", {})],
         [("", {})],
         [("修订提示", {"bold": True, "color": RED})],
         [("旧版把运动学/离散积分/协方差传播标为公式 (3)(4)(5) 有误；论文 (3) 是先验分布、(4) 是测量模型、(5) 是顺序更新分解，ESIKF 更新是 (11)。", E)]])

# 7 数学工具
content("Math Toolkit", "SO(3) 旋转计算的两个标配（论文 IV-A 基础）",
        [[("反对称矩阵 ⌊v⌋×", B)],
         [("把叉乘写成矩阵乘法 a×b = ⌊a⌋×·b，使叉乘可求导、可泰勒展开", {})],
         [("", {})],
         [("指数映射 Exp(φ) = 罗德里格斯公式", B)],
         [("φ 的方向 = 旋转轴，模长 = 旋转角；Exp(φ) = I + (sinθ/θ)⌊φ⌋× + ((1−cosθ)/θ²)⌊φ⌋×²", {})],
         [("旋转矩阵不能直接加（破坏正交性）：R_new = R_old · Exp(ω·Δt) 才合法", {})],
         [("", {})],
         [("IMU 旋转积分完整链路", B)],
         [("ω_m → 扣偏置 → ⌊ωΔt⌋× → Exp → 右乘更新 R", {})],
         [("", {})],
         [("代码：common_lib.h Exp()；DIM_STATE = 19（rot3 + pos3 + τ1 + vel3 + b_g3 + b_a3 + g3）", {})]])

# 8 体素地图
cards("Map · 体素地图", "哈希表 + 八叉树：把环境切成数字积木（论文 V-A/V-B）",
      [("结构", "哈希表管理 0.5m 根体素，O(1) 查找、无需预设地图边界\n根内八叉树自适应细分，叶体素存局部平面 {q, n, Σ_nq} + 激光原始点", NAVY),
       ("平面拟合与判据", "法向 = 协方差最小特征值的特征向量；最小特征值 < 0.0025（avia.yaml）判为平面\n否则八分细分递归；平面“成熟”后冻结，留最近 50 点作视觉点候选", BLUE),
       ("滑窗", "只维护当前位置为中心的固定大小局部地图（Fig.4 ring-buffer）\n超出即回收内存，保证长序列内存恒定", NAVY)],
      h=1.6,
      note="论文 IX-A 文字说八叉树最大层为 3，官方 avia.yaml 实配 max_layer: 2 —— 论文-代码差异，以 yaml 为准。")

# 9 LIO 更新
content("Step 3 · 激光更新", "点到平面直接配准（论文 VI，式 (17)-(20)）",
        [[("流程（每帧）", B)],
         [("① 去畸变点按当前状态投影到全局系（式 17）→ ② 哈希定位所在体素及邻域，取平面", {})],
         [("③ 残差 z = nᵀ(ᴳT·p − q)（式 18-19），量测噪声 = 点噪声 + 平面协方差", {})],
         [("④ ESIKF 迭代更新（式 11，最多 5 次），每次迭代重新配准；收敛后更新体素地图", {})],
         [("", {})],
         [("亮点：方向相关的测量噪声（式 20）", B)],
         [("• 激光束有发散角 θ：入射角 φ 越大，测距不确定 δd 越大 → 地面/斜面点自动降权", {})],
         [("", {})],
         [("判据（代码实现）", B)],
         [("• 残差马氏 3σ 门限剔除外点（voxel_map.cpp:737，sigma_num=3）", {})],
         [("• 收敛：旋转 < 0.01° 且平移 < 0.015cm（voxel_map.cpp:477；代码值，论文只写 ‖Δx‖<ε）", {})],
         [("• 修订：论文所述截断最小二乘，代码实际实现为 3σ 剔除 + R 按方差自适应加权", E)]])

# 10 ESIKF
content("Fusion · 融合核心", "ESIKF 顺序更新（论文 IV-D，式 (5)-(11)）",
        [[("为什么顺序更新？", B)],
         [("• 激光（数百点）与图像（数万像素残差）维度不匹配；图像还可分层融合", {})],
         [("• 式 (5)：p(x|y_l,y_c) ∝ p(y_c|x)·p(y_l|x)·p(x) → 拆成两次贝叶斯更新（式 6-7）", {})],
         [("• 在测量噪声统计独立假设下，数学上等价于联合更新", {})],
         [("", {})],
         [("更新公式（式 11）", B)],
         [("K = (HᵀR⁻¹H + P̂⁻¹)⁻¹HᵀR⁻¹　；　x̂ₖ₊₁ = x̂ₖ ⊞ (−Kz − (I−KH)(x̂ₖ⊟x̂))", {})],
         [("", {})],
         [("执行顺序（Algorithm 1）", B)],
         [("① IMU 传播得先验 → ② 激光更新迭代至收敛，P̂←(I−KH)P̂ → ③ 以②为先验做视觉更新（逐金字塔层）", {})],
         [("④ 最终状态更新地图 → 代码：LIVMapper.cpp:267 stateEstimationAndMapping", {})]])

# 11 视觉点
content("Visual 01 · 视觉点", "什么是视觉地图点？（论文 V-C，统一地图的核心）",
        [[("视觉点 = 激光测准位置的 3D 点 + 相机记下的外观", {"bold": True, "color": NAVY, "size": 18})],
         [("", {})],
         [("• 3D 位置与法向来自激光（体素平面），毫米级、无尺度漂移、无需三角化", {})],
         [("• 外观是若干历史观测的图像块（参考块 + 可见块），挂在体素叶节点上", {})],
         [("• 激光和视觉共用同一张地图：LIO 用它的坐标，VIO 用它的块", {})],
         [("", {})],
         [("生成与更新（V-C）", B)],
         [("• 候选 = 当前帧可见 + 图像灰度梯度显著；图像划 30×30 像素网格，空格用梯度最高者生成新点", {})],
         [("• 已有点：距上次加块超 20 帧，或像素位置偏移超 40px 时追加新块 → 观测视角均匀分布", {})],
         [("", {})],
         [("修订", {"bold": True, "color": RED})],
         [("旧版“每叶节点 1~3 个点、总数 200~500”等数字论文/代码均无出处，已删除；论文只规定 30×30 网格机制。", E)]])

# 12 视觉点检索
content("Visual 02 · 视觉点检索", "找出这一帧能看见的点（论文 VII-A）",
        [[("① 可见体素查询", B)],
         [("查当前激光扫描命中的体素 + 上一帧可见点所在体素（相邻帧 FoV 重叠大），再做 FoV 检查", {})],
         [("", {})],
         [("② 按需射线投射（On-demand Raycasting）", B)],
         [("• 激光近距盲区 / 相机 FoV 超出激光 FoV 时，图像按 30×30 网格，空格沿中心像素后向投射射线", {})],
         [("• 深度方向 d_min→d_max 均匀采样，命中含地图点的体素即纳入；相机系预计算省算力", {})],
         [("", {})],
         [("③ 外点剔除（三类判据）", B)],
         [("• 遮挡：每 30×30 网格只留最低深度点；与当前激光扫描投影的深度图 9×9 邻域比较，剔除遮挡/深度不连续", {})],
         [("• 视角过大：参考块或当前块视角（法向与视线夹角）> 80° 剔除", {})],
         [("• 代码：vio.cpp retrieveFromVisualSparseMap（:353-780）", {})]])

# 13 仿射扭曲
content("Visual 03 · 仿射扭曲", "平面先验算仿射矩阵（论文 V-E1，式 (13)）",
        [[("问题：相机移动后同一平面在图像上不仅平移，还近大远小、倾斜剪切，简单平移对不上", {})],
         [("", {})],
         [("公式 (13)：A = P ( R + t·nᵀ / (nᵀ·p) ) P⁻¹", {"bold": True, "color": NAVY, "size": 20})],
         [("P=相机投影矩阵；R,t=参考帧→当前帧位姿；n=激光平面法向；p=3D 点（参考帧系）", {})],
         [("", {})],
         [("为什么准", B)],
         [("• 恒定深度假设：patch 内深度都等于中心深度 → 只有平移+均匀缩放，近距离/斜平面误差大", {})],
         [("• 平面先验：patch 内像素深度按平面各不相同 → 仿射含剪切与非均匀缩放，贴合真实透视形变", {})],
         [("", {})],
         [("金字塔层选择", B)],
         [("仿射行列式 det(A)>3 时升层（上限层 2），防止单层畸变过大（vio.cpp:321）", {})],
         [("代码：vio.cpp getWarpMatrixAffineHomography（:253）、warpAffine（:293）", {})]])

# 14 光度残差
content("Visual 04 · 光度残差", "用像素亮度量出位姿偏差（论文 VII-B，式 (21)-(23)）",
        [[("式 (21)：0 = τ_k·I_k(u_i + Δu) − τ_r·I_r(u'_i + A·Δu)", {"bold": True, "color": NAVY, "size": 20})],
         [("τ = 逆曝光时间；Δu = patch 内相对中心的像素偏移（旧版误写为“曝光偏移”，已修正）", E)],
         [("位姿准 → 投影对准同一表面 → 亮度一致 → 残差为 0", {})],
         [("", {})],
         [("逆组合公式（式 23）：u'_i 在迭代中不变 → 雅可比只需算一次，每层迭代从 10 次降到 3 次", {})],
         [("", {})],
         [("由粗到精：从最粗金字塔层开始，收敛后进更细层（Algorithm 1）", B)],
         [("", {})],
         [("判据", B)],
         [("• 外点：|残差| > outlier_threshold×patch 像素数（avia.yaml 1000，vio.cpp:764）", {})],
         [("• 收敛：旋转 < 0.001°、平移 < 0.001cm（代码值，比激光更严格）", {})],
         [("• 论文-代码差异：论文说 patch 三层金字塔、更新到 level 2；avia.yaml 配 4 层且代码从 level 3→0", E)]])

# 15 曝光估计
content("Visual 05 · 曝光估计", "在线估计逆曝光时间（论文 VII-B 末段）",
        [[("问题：光照突变 + 自动曝光 → 同一点灰度整体漂移，光度一致性被破坏", {})],
         [("", {})],
         [("做法", B)],
         [("• 逆曝光时间 τ 作为状态第 19 维（dim(M)=19 的最后一维），与位姿联合估计", {})],
         [("• 残差中参考块乘 τ_r、当前块乘 τ_k（式 21-22），消除亮度尺度差", {})],
         [("• 第一帧固定 τ₀ = 1 以消除全零退化 → 估计的是相对首帧的曝光", {})],
         [("• 对 τ 的雅可比 = 当前像素灰度，计算零成本；过程噪声 inv_expo_cov = 0.1", {})],
         [("", {})],
         [("消融实验：去掉曝光估计，平均 RMSE 从 0.044 m 升至 0.051 m（论文表 II）", {"bold": True})],
         [("", {})],
         [("修订", {"bold": True, "color": RED})],
         [("旧版“公式(14) = ∂r/∂τ = −∇I×v_img（运动模糊推导）”在论文中不存在（论文 (14) 是法向优化）；曝光只通过 τ 缩放进入 (21)(22)，本页为正确表述。", E)]])

# 16 参考块与法向
content("Visual 06 · 参考块更新与法向细化", "选最靠谱的模板，把平面法向磨得更准（论文 V-D/V-E）",
        [[("参考块更新（V-D，式 12）", B)],
         [("• score S = (1−ω₁)·平均NCC + ω₁·视角余弦 c，其中 ω₁ = 1/(1+e^{tr(Σ_n)})", {})],
         [("• NCC 高 = 外观与多数历史观测相似（避开动态物体）；c 高 = 正对平面（纹理清晰）", {})],
         [("• 得分最高的块成为参考块；前代 FAST-LIVO 选视角最近的块 → 对当前位姿约束弱", {})],
         [("", {})],
         [("法向细化（V-E，式 14-16）", B)],
         [("• 式 (14)：在最高分辨率层最小化参考块与其余块的光度误差，优化平面法向", {})],
         [("• 式 (15)(16)：重参数化到 2 维无约束优化；论文称在独立线程执行；收敛后固定法向、删除多余块", {})],
         [("", {})],
         [("消融：去掉参考块更新策略，平均 RMSE 从 0.044 m 升至 0.089 m —— 单项贡献最大（表 II）", {"bold": True})]])

# 17 五大创新
cards("Innovations", "相对前代 FAST-LIVO 的五大改进（论文摘要 + 实验表 II）",
      [("① ESIKF 顺序更新", "单一 19 维状态先激光后视觉，解决维度不匹配，数学等价联合更新（IV-D）", NAVY),
       ("② 平面先验 + 法向细化", "激光法向做仿射扭曲与细化，替代恒定深度假设（V-E）", BLUE),
       ("③ 参考块动态更新", "NCC + 视角余弦评分选最优参考块；消融贡献最大（V-D）", NAVY),
       ("④ 在线曝光估计", "逆曝光时间入状态联合估计，抵消光照变化（VII-B）", BLUE),
       ("⑤ 按需射线投射", "激光盲区/视野外后向投射找回视觉点（VII-A2）", NAVY)],
      h=0.95,
      note="消融（25 条序列平均 RMSE）：完整版 0.044 m；去参考块更新 0.089 m；去曝光估计 0.051 m（论文表 II）。")

# 18 实验
content("Experiments", "基准对比与运行效率（论文 IX，表 II / 表 III）",
        [[("精度（绝对平移误差 RMSE，25 条序列平均）", B)],
         [("• FAST-LIVO2：0.044 m —— 第二名 FAST-LIVO（0.137 m）的 3 倍精度", {})],
         [("• 对比：R3LIVE 7.416 / LVI-SAM 1.928（9 条失败）/ FAST-LIO2 0.151 / FAST-LIVO 0.137 m", {})],
         [("", {})],
         [("运行时间（i7-10700K）", B)],
         [("• 平均 30.03 ms/帧 = 激光 17.13 ms + 图像 12.90 ms，10Hz 实时；ARM 平台 78.44 ms 仍实时", {})],
         [("• 对比：FAST-LIVO 41.43 / R3LIVE 108.36 / LVI-SAM 108.45 ms；逆组合公式使每层迭代从 10 次降到 3 次", {})],
         [("", {})],
         [("极端场景（IX-C）", B)],
         [("• 激光退化（大屏幕/横幅墙、隧道）与视觉挑战（明暗剧变、过曝）下，端到端误差仍 < 0.01 m", {})],
         [("• FAST-LIVO / R3LIVE 在多条此类序列（如 CBD Building 02/03、Mining Tunnel）完全失败", {})]])

# 19 应用
cards("Applications", "三个真实应用（论文 X）",
      [("全机载无人机自主导航", "位姿（10Hz）+ 轨迹规划 + 控制全部跑在机载电脑\n激光退化区与避障场景演示；首个用于真实自主飞行的激光-惯性-视觉系统", NAVY),
       ("航空测绘", "MARS-LVIG 高空大场景（丛林/山地/岛屿），像素级彩色建图\n滑窗机制保证长序列内存恒定", BLUE),
       ("3D 渲染", "稠密点云直接用于 mesh 与 NeRF 渲染\n统一地图使几何与颜色天然对齐，无需后处理配准", NAVY)],
      h=1.55)

# 20 总结
content("Summary", "一个处理周期的七步回顾",
        [[("① 时间同步 — 扫描重组，以图像时刻切分（IV-B）", {})],
         [("② IMU 前向传播 — 积分推算位姿与协方差先验（IV-C）", {})],
         [("③ 点云去畸变 — 反向传播统一到帧尾（IV-C）", {})],
         [("④ 激光更新 — 点到平面残差 + ESIKF 迭代（VI，式 17-20）", {})],
         [("⑤ 视觉点检索 — 可见体素查询 + 按需 raycast + 外点剔除（VII-A）", {})],
         [("⑥ 视觉更新 — 光度残差 + 曝光估计 + 由粗到精（VII-B，式 21-23）", {})],
         [("⑦ 地图更新 — 新点入图 + 参考块更新 +（可选）法向细化（V）", {})],
         [("", {})],
         [("一句话：IMU 高频“猜”位置，激光“摸”出几何做粗修正，相机“看”纹理做精修；", {"bold": True, "color": NAVY, "size": 18})],
         [("三者共用同一张体素地图与同一个 19 维状态，经 ESIKF 顺序更新紧耦合，实时输出位姿与彩色点云地图。", {"bold": True, "color": NAVY, "size": 18})]])

# 21 勘误页
content("Errata", "本版相对旧版 PPT 的主要修正（核验自论文原文与代码）",
        [[("① “四种传感器” → 三种；② “20 维状态” → 论文 dim(M)=19、代码 DIM_STATE=19", E)],
         [("③ 公式编号纠错：传播是式 (1)-(3)，测量模型 (4)，顺序更新 (5)-(8)，ESIKF 更新 (11)；旧版 (3)(4)(5) 标注全错", E)],
         [("④ Δu 是 patch 内相对像素偏移，不是“曝光导致的亚像素偏移”；公式 (14) 是法向优化", E)],
         [("⑤ 删除论文中不存在的“∂r/∂τ = −∇I×v_img 运动模糊公式”页；曝光仅通过 τ 缩放进入式 (21)(22)，τ₀ 固定为 1", E)],
         [("⑥ 金字塔层数统一说明：论文三层/level≤2，代码 avia.yaml 四层/level 3→0 —— 属论文-代码差异", E)],
         [("⑦ 删除无出处的“grid_size=5、每格 1~2 点、视觉点总数 200~500”等杜撰数字（论文为 30×30 像素网格）", E)],
         [("⑧ 明确标注收敛阈值 0.01°/0.015cm（激光）与 0.001°/0.001cm（视觉）为代码实现值，论文正文未给出具体数值", E)]])

# 22 致谢
s = prs.slides.add_slide(BLANK)
box(s, 0, 0, SW, SH, fill=NAVY)
tx(s, Inches(1), Inches(2.6), Inches(11.3), Inches(0.8), "感谢聆听", size=40, color=WHITE, bold=True)
tx(s, Inches(1), Inches(3.7), Inches(11.3), Inches(2.2),
   [[("论文：arXiv 2408.14035 — FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry", {})],
    [("代码/数据集：github.com/hku-mars/FAST-LIVO2（含 LIV_handhold 硬件方案）", {})],
    [("配套材料：《FAST-LIVO2论文精读翻译与代码实现说明（修订版）.md》", {})],
    [("　　　　　《FAST-LIVO2论文代码对照-数据处理流程详解.md》", {})],
    [("", {})],
    [("本 PPT 所有公式编号、参数与结论均已对照论文 PDF 原文与开源代码核验；修订处以红色标注。", {"color": RGBColor(0x9D, 0xC3, 0xE6)})]],
   size=16, color=RGBColor(0xBD, 0xD7, 0xEE))

OUT = "FAST-LIVO2 全景解析（修订版）.pptx"
prs.save(OUT)
print("saved", OUT, "slides:", len(prs.slides._sldIdLst))
