# FAST-LIVO2 论文全文中文翻译

> **原文标题**：FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry
> **作者**：Chunran Zheng, Wei Xu, Zuhao Zou, Tong Hua, Chongjian Yuan, Dongjiao He, Bingyang Zhou, Zheng Liu, Jiarong Lin, Fangcheng Zhu, Yunfan Ren, Rong Wang, Fanle Meng, Fu Zhang（通讯作者）
> **单位**：香港大学机械工程系 MaRS 实验室；中国电子科技集团公司信息科学研究院
> **发表**：IEEE Transactions on Robotics (T-RO), 2024
> **arXiv**：2408.14035v2
> **代码**：https://github.com/hku-mars/FAST-LIVO2

---

## 摘要

本文提出 FAST-LIVO2：一种快速、直接的激光-惯性-视觉里程计（LiDAR-Inertial-Visual Odometry, LIVO）框架，旨在同时定位与建图（SLAM）任务中实现精确、鲁棒的状态估计，并为实时机载机器人应用提供巨大潜力。

FAST-LIVO2 通过误差状态迭代卡尔曼滤波器（ESIKF）高效融合 IMU、LiDAR 和图像测量。为解决异构 LiDAR 与图像测量之间的维度不匹配问题，我们在卡尔曼滤波器中采用**顺序更新策略**。为提升效率，视觉和 LiDAR 融合均采用**直接法**：LiDAR 模块直接配准原始点云，不提取边缘或平面特征；视觉模块直接最小化光度误差，不提取 ORB 或 FAST 角点特征。

视觉与 LiDAR 测量的融合基于**单一统一体素地图**：LiDAR 模块构建几何结构用于配准新的 LiDAR 扫描，视觉模块将图像块附加到 LiDAR 点上（即视觉地图点），实现新图像的对齐。为提升图像对齐精度，我们使用体素地图中 LiDAR 点提供的**平面先验**（甚至在对齐过程中进一步细化平面先验），并在新图像对齐后**动态更新参考块**。此外，为增强图像对齐的鲁棒性，FAST-LIVO2 采用**按需射线投射**操作，并**实时估计图像曝光时间**。

我们在基准数据集和私有数据集上进行了大量实验，证明所提系统在精度、鲁棒性和计算效率方面显著优于其他最先进的里程计系统。各关键模块的有效性也得到了验证。最后，我们详细介绍了 FAST-LIVO2 的三个应用：无人机机载导航（展示系统实时机载导航的计算效率）、机载建图（展示系统建图精度）、3D 模型渲染（基于网格和 NeRF，强调我们重建的稠密地图适用于后续渲染任务）。我们已在 GitHub 上开源了本工作的代码、数据集和应用。

**关键词**：同时定位与建图（SLAM）、传感器融合、三维重建、空中导航

---

## I. 引言

近年来，同时定位与建图（SLAM）技术取得了显著进步，尤其是在未知环境中的实时三维重建与定位方面。由于 SLAM 能够实时估计位姿并重建地图，它已成为各种机器人导航任务不可或缺的技术。定位过程为机器人机载控制器提供关键的状态反馈，而稠密三维地图则提供关键的环境信息（如自由空间和障碍物），这对于有效的轨迹规划至关重要。彩色地图还承载着丰富的语义信息，能够生动地再现真实世界，为虚拟现实/增强现实、三维建模和人机交互等应用开辟了广阔空间。

目前，已有多种基于单一测量传感器的 SLAM 框架成功实现，主要是相机[1-4]或 LiDAR[5-7]。尽管视觉 SLAM 和 LiDAR SLAM 在各自领域展现出良好前景，但它们都有固有的局限性，制约了在各种场景下的性能。

**视觉 SLAM** 利用低成本的 CMOS 传感器和镜头，能够建立精确的数据关联，从而达到一定的定位精度。丰富的颜色信息进一步增强了语义感知。借助深度学习方法，可以实现鲁棒的特征提取和动态物体滤波。然而，视觉 SLAM 缺乏直接深度测量，需要通过三角化或深度滤波等操作同时优化地图点，这引入了显著的计算开销，往往限制了地图精度和密度。视觉 SLAM 还面临许多其他局限，如不同尺度下测量噪声变化、对光照变化敏感、以及无纹理环境对数据关联的影响。

**LiDAR SLAM** 利用 LiDAR 传感器直接获得精确的深度测量，在定位和建图任务中相比视觉 SLAM 具有更高的精度和效率。尽管有这些优势，LiDAR SLAM 仍存在几个显著缺点：一方面，它重建的点云地图虽然详细，但缺乏颜色信息，降低了信息维度；另一方面，在几何约束不足的环境（如狭窄隧道、单一长墙等）中，LiDAR SLAM 性能往往下降。

随着智能机器人在真实世界中运行需求的增长，尤其是在经常缺乏结构或纹理的环境中，仅依赖单一传感器的现有系统显然无法提供所需的精确鲁棒位姿估计。为解决这一问题，LiDAR、相机和 IMU 等常用传感器的融合日益受到关注。该策略不仅结合了各传感器的优势以提升位姿估计，还有助于构建精确、稠密、彩色的点云地图，即使在单个传感器性能下降的环境中也是如此。

高效精确的激光-惯性-视觉里程计（LIVO）与建图仍然是具有挑战性的问题：

1. **数据量巨大**：整个 LIVO 系统需要处理每秒数百到数千个点的 LiDAR 测量，以及高帧率、高分辨率图像。在机载资源有限的情况下充分利用如此海量数据，需要极高的计算效率。
2. **特征提取瓶颈**：许多现有系统通常包含一个激光-惯性里程计（LIO）子系统和一个视觉-惯性里程计（VIO）子系统，各自需要从视觉和 LiDAR 数据中提取特征以降低计算量。在缺乏结构或纹理的环境中，这种提取过程往往产生有限的特征点。此外，为优化特征提取，需要大量工程适配以适应 LiDAR 扫描模式和点密度的变化。
3. **统一地图设计困难**：为降低计算需求并实现相机与 LiDAR 测量之间更紧密的集成，需要一个统一地图同时管理稀疏点和观测到的高分辨率图像测量。然而，考虑到 LiDAR 和相机的异构测量，设计和维护这样的地图特别具有挑战性。
4. **像素级精度要求**：为确保重建彩色点云的精度，位姿估计需要达到像素级精度。达到这一标准面临相当大的挑战：适当的硬件同步、LiDAR 与相机之间外参的严格预标定、曝光时间的精确恢复，以及能够实时达到像素级精度的融合策略。

受这些问题的启发，我们提出 FAST-LIVO2，一种通过顺序更新的误差状态迭代卡尔曼滤波器（ESIKF）紧密集成 LiDAR、图像和 IMU 测量的高效 LIVO 系统。借助 IMU 传播的先验，系统状态被顺序更新：**先由 LiDAR 测量更新，再由图像测量更新**，两者均基于单一统一体素地图使用直接法。

具体而言，在 LiDAR 更新中，系统将原始点配准到地图以构建和更新其几何结构；在视觉更新中，系统**直接复用 LiDAR 地图点作为视觉地图点**，无需从图像中提取、三角化或优化任何视觉特征。地图中选定的视觉地图点附加有先前观测到的参考图像块，然后投影到当前图像，通过最小化直接光度误差（即稀疏图像对齐）来对齐位姿。

为提高图像对齐精度，FAST-LIVO2 动态更新参考块，并使用从 LiDAR 点获得的平面先验。为提升计算效率，FAST-LIVO2 使用 LiDAR 点识别当前图像可见的视觉地图点，并在没有 LiDAR 点时进行按需体素射线投射。FAST-LIVO2 还实时估计曝光时间以应对光照变化。

FAST-LIVO2 是在我们先前工作 FAST-LIVO[8]的基础上开发的。与 FAST-LIVO 相比，新贡献如下：

1. **高效 ESIKF 顺序更新框架**：解决 LiDAR 与视觉测量之间的维度不匹配问题，提升了 FAST-LIVO 使用异步更新的鲁棒性。
2. **平面先验（及细化）**：使用（甚至细化）来自 LiDAR 点的平面先验以提高精度。相比之下，FAST-LIVO 假设一个块内所有像素共享相同深度，这一大胆假设显著降低了图像对齐中仿射变换的精度。
3. **参考块更新策略**：通过选择具有大视差和充足纹理细节的高质量内点参考块来提高图像对齐精度。FAST-LIVO 基于与当前视图的接近程度选择参考块，往往导致低质量参考块降低精度。
4. **在线曝光时间估计**：处理环境光照变化。FAST-LIVO 未解决此问题，导致在显著光照变化下图像对齐收敛不良。
5. **按需体素射线投射**：增强系统在 LiDAR 近距盲区导致无 LiDAR 点测量情况下的鲁棒性，这是 FAST-LIVO 未考虑的问题。

上述每个贡献都在全面的消融实验中进行了评估以验证其有效性。我们将所提系统实现为实用的开源软件，针对 Intel 和 ARM 处理器上的实时运行进行了精心优化。系统具有通用性，支持多线旋转 LiDAR、具有非传统扫描模式的新兴固态 LiDAR，以及针孔相机和各种鱼眼相机。

此外，我们在公开数据集的 25 个序列（即 Hilti 和 NTU-VIRAL 数据集）以及各种代表性私有数据集上进行了大量实验，与其他最先进的 SLAM 系统（如 R3LIVE、LVI-SAM、FAST-LIO2 等）进行比较。定性和定量结果均表明，所提系统在精度和鲁棒性方面显著优于其他同类系统，且计算成本更低。

为进一步强调系统在真实世界中的适用性和多功能性，我们部署了三个独特的应用：首先，全机载自主无人机导航，展示系统的实时能力，标志着将激光-惯性-视觉系统用于真实世界自主无人机飞行的开创性实例；其次，机载建图展示了系统在无结构环境下的像素级精度；最后，高质量的网格生成、纹理映射和 NeRF 模型生成强调了系统适用于渲染任务。我们已在 GitHub 上公开代码和数据集。

---

## II. 相关工作

### II-A. 直接法

直接法是视觉和 LiDAR SLAM 中用于快速位姿估计的一种重要方法。与基于特征的方法[9,10,5,6]（需要提取显著特征点（如图像中的角点和边缘像素；LiDAR 扫描中的平面和边缘点）并生成鲁棒描述子进行匹配）不同，直接法直接利用原始测量优化传感器位姿[11]，通过最小化基于光度误差或点到平面残差的误差函数[3,12-14]。通过消除耗时的特征提取和匹配，直接法提供快速的位姿估计。然而，缺乏特征匹配需要相当准确的状态先验估计以避免局部最小值。

视觉 SLAM 中的直接法可大致分为**稠密直接法**、**半稠密直接法**和**稀疏直接法**：
- **稠密直接法**主要用于具有完整深度测量的 RGB-D 相机[15-17]，应用图像到模型的对齐进行位姿估计。
- **半稠密直接法**[18,3]利用具有显著灰度梯度的像素进行直接图像对齐。
- **稀疏直接法**[12,2]仅通过少量精心选择的原始块提供精确的状态估计，从而进一步降低计算负担。

与直接视觉 SLAM 方法不同，直接 LiDAR SLAM 系统[13,14,19,20]不区分稠密和稀疏方法，通常在每次扫描中使用空间下采样或时间下采样的原始点构建位姿优化约束。

在我们的工作中，我们将直接法原理用于 LiDAR 和视觉模块。系统的 LiDAR 模块改编自 VoxelMap[14]，视觉模型基于稀疏直接法的变体[12]。虽然受[12]中稀疏直接图像对齐的启发，但我们的视觉模块的不同之处在于**复用 LiDAR 点作为视觉地图点**，从而减轻了密集的后端计算（即特征对齐、滑动窗口优化和/或深度滤波）。

### II-B. 激光-视觉（-惯性）SLAM

在激光-视觉-惯性 SLAM 中集成多个传感器使系统能够处理各种挑战性环境，特别是当一个传感器发生故障或部分退化时。受此启发，研究界涌现出各种激光-视觉-惯性 SLAM 系统。现有方法通常可分为两类：**松耦合**和**紧耦合**。分类可从两个角度确定：状态估计层面和原始测量层面。在状态估计层面，关键在于一个传感器的估计是否作为另一个传感器模型中的优化目标。在原始测量层面，涉及不同传感器的原始数据是否被组合。

Zhang 等人提出了一种激光-视觉-惯性 SLAM 系统[21]，在状态估计层面是松耦合的。在该系统中，VIO 子系统仅为 LIO 子系统中的扫描配准提供初始位姿，而不与扫描配准联合优化。VIL-SLAM[22]采用类似的松耦合方法，不使用 LiDAR、相机和 IMU 测量的联合优化。

一些系统（如 DEMO[23]、LIMO[24]、CamVox[25,26]）使用 3D LiDAR 点为视觉模块提供深度测量[1,27,4]。虽然这些系统表现出测量层面的紧耦合，但在状态估计层面仍是松耦合的，主要因为在状态估计时缺乏直接源自 LiDAR 测量的约束。另一个问题是，由于分辨率不匹配，3D LiDAR 点与 2D 图像特征点和/或线之间不存在一一对应关系。这种不匹配需要在深度关联中进行插值，引入潜在误差。为解决此问题，DVL-SLAM[28]采用直接法进行视觉跟踪，其中 LiDAR 点直接投影到图像中以确定对应像素位置的深度。

上述工作尚未在状态估计层面实现紧耦合。为追求更高的精度和鲁棒性，许多近期研究以紧耦合方式联合优化传感器数据。例如：
- **LIC-Fusion**[29]基于 MSCKF[30]框架紧密融合 IMU 测量、稀疏视觉特征以及 LiDAR 平面和边缘特征。后续的 LIC-Fusion2.0[31]通过在滑动窗口内实现平面特征跟踪来增强 LiDAR 位姿估计。
- **VILENS**[32]通过统一因子图提供视觉、LiDAR 和惯性数据的联合优化，依赖固定滞后平滑。
- **R2LIVE**[33]在流形迭代卡尔曼滤波器[34]中紧密融合 LiDAR、相机和 IMU 测量。对于 R2LIVE 中的 VIO 子系统，使用滑动窗口优化来三角化地图中视觉特征的位置。

一些系统在测量和状态估计层面都实现了完全紧耦合：
- **LVI-SAM**[35]在紧耦合的平滑与建图框架中融合 LiDAR、视觉和惯性传感器，该框架构建于因子图之上。VIO 子系统执行视觉特征跟踪并使用 LiDAR 扫描提取特征深度。
- **R3LIVE**[36]通过 LIO 构建全局地图的几何结构，通过 VIO 渲染地图纹理。这两个子系统通过将各自的 LiDAR 或视觉数据与 IMU 融合来联合估计系统状态。高级版本 R3LIVE++[37]实时估计曝光时间并预先进行光度标定[38]，使系统能够恢复地图点的辐射度。

与大多数前述依赖基于特征方法进行 LIO 和 VIO 子系统的激光-惯性-视觉系统不同，R3LIVE 系列[36,37]对两者都采用直接法而不进行特征提取，使其即使在无纹理或无结构场景中也能捕捉细微的环境特征。

我们的系统也使用 LiDAR、图像和 IMU 数据联合估计状态，并在测量层面维护紧耦合的体素地图。此外，我们的系统使用直接法，利用原始 LiDAR 点进行 LiDAR 扫描配准，使用原始图像块进行视觉跟踪。我们的系统与 R3LIVE（或 R3LIVE++）的关键区别在于：**R3LIVE（和 R3LIVE++）在 VIO 中以单个像素级别运行，而我们的系统以图像块级别运行**。这一差异赋予我们的系统显著优势：

1. **鲁棒性**：我们的方法使用简化的单步帧到地图稀疏图像对齐进行位姿估计，减轻了对 R3LIVE 中必须通过帧到帧光流获得的精确初始状态的严重依赖。因此，我们的系统简化并改进了 R3LIVE 中的两阶段帧到帧和帧到地图操作。
2. **计算效率**：R3LIVE 中的 VIO 主要采用计算昂贵的稠密直接法，需要大量点进行残差构建和渲染。相比之下，我们的稀疏直接法提供了更高的计算效率。
3. **分辨率**：我们的系统利用原始图像块分辨率的信息，而 R3LIVE 受限于其点地图的分辨率。

我们系统的视觉模块与 DV-LOAM[39]、SDV-LOAM[40]和 LVIO-Fusion[41]最为相似，它们将附加有块的 LiDAR 点投影到新图像中，并通过最小化直接光度误差跟踪图像。然而，它们有几个关键区别，例如视觉和 LiDAR 使用独立地图、视觉模块中块扭曲依赖恒定深度假设、状态估计层面的松耦合、以及图像对齐采用两阶段帧到帧和帧到关键帧方式。相比之下，我们的系统在迭代卡尔曼滤波器中紧密集成帧到地图图像对齐、LiDAR 扫描配准和 IMU 测量。此外，由于 LiDAR 和视觉模块使用单一统一地图，我们的系统可以直接利用 LiDAR 点提供的平面先验来加速图像对齐。

---

## III. 系统概述

我们系统的概述如图 1 所示，包含四个部分：ESIKF（第 IV 节）、局部建图（第 V 节）、LiDAR 测量模型（第 VI 节）和视觉测量模型（第 VII 节）。

异步采样的 LiDAR 点首先通过**扫描重组**在相机采样时刻重组为扫描。然后，我们通过具有顺序状态更新的 ESIKF 紧密耦合 LiDAR、图像和惯性测量，其中系统状态被顺序更新：**先由 LiDAR 测量更新，再由图像测量更新**，两者均基于单一统一体素地图使用直接法（第 IV 节）。

为在 ESIKF 更新中构建 LiDAR 测量模型（第 VI 节），我们计算帧到地图的点到平面残差。为建立视觉测量模型（第 VII 节），我们利用可见体素查询和按需射线投射从地图中提取当前视场内的视觉地图点；提取后，识别并丢弃外点视觉地图点（如被遮挡或表现出深度不连续的点）；然后计算帧到地图图像光度误差用于视觉更新。

用于视觉和 LiDAR 更新的局部地图是体素地图结构（第 V 节）：LiDAR 点构建和更新地图的几何结构，而视觉图像将图像块附加到选定的地图点（即视觉地图点）并动态更新参考块。更新后的参考块的法向量在**独立线程**中进一步细化。

---

## IV. 基于顺序状态更新的误差状态迭代卡尔曼滤波器

本节概述系统架构，基于顺序更新的误差状态迭代卡尔曼滤波器（ESIKF）框架。

### IV-A. 符号与状态转移模型

在我们的系统中，假设三个传感器（LiDAR、IMU 和相机）之间的时间偏移已知，可以预先标定或同步。我们以 IMU 坐标系（记为 $I$）作为本体坐标系，以第一个本体坐标系作为全局坐标系（记为 $G$）。此外，假设三个传感器刚性连接，外参（定义见表 I）已预先标定。

则第 $i$ 次 IMU 测量时的离散状态转移模型为：

$$\mathbf{x}_{i+1} = \mathbf{x}_i \boxplus \left( \Delta t \, \mathbf{f}(\mathbf{x}_i, \mathbf{u}_i, \mathbf{w}_i) \right) \tag{1}$$

其中 $\Delta t$ 是 IMU 采样周期，状态 $\mathbf{x}$、输入 $\mathbf{u}$、过程噪声 $\mathbf{w}$ 和函数 $\mathbf{f}$ 定义如下：

状态流形 $\mathcal{M} \triangleq SO(3) \times \mathbb{R}^{16}, \dim(\mathcal{M}) = 19$

$$\mathbf{x} \triangleq \begin{bmatrix} {}^{G}\mathbf{R}_{I}^{T} & {}^{G}\mathbf{p}_{I}^{T} & {}^{G}\mathbf{v}_{I}^{T} & \mathbf{b}_{g}^{T} & \mathbf{b}_{a}^{T} & {}^{G}\mathbf{g}^{T} & \tau \end{bmatrix}^{T} \in \mathcal{M}$$

$$\mathbf{u} \triangleq \begin{bmatrix} \bm{\omega}_{m}^{T} & \mathbf{a}_{m}^{T} \end{bmatrix}^{T}, \quad \mathbf{w} \triangleq \begin{bmatrix} \mathbf{n}_{g}^{T} & \mathbf{n}_{a}^{T} & \mathbf{n}_{\mathbf{b}_g}^{T} & \mathbf{n}_{\mathbf{b}_a}^{T} & n_{\tau} \end{bmatrix}^{T}$$

$$\mathbf{f}(\mathbf{x},\mathbf{u},\mathbf{w}) = \begin{bmatrix} \bm{\omega}_m - \mathbf{b}_g - \mathbf{n}_g \\ {}^{G}\mathbf{v}_I + \frac{1}{2}({}^{G}\mathbf{R}_I(\mathbf{a}_m - \mathbf{b}_a - \mathbf{n}_a) + {}^{G}\mathbf{g})\Delta t \\ {}^{G}\mathbf{R}_I(\mathbf{a}_m - \mathbf{b}_a - \mathbf{n}_a) + {}^{G}\mathbf{g} \\ \mathbf{n}_{\mathbf{b}_g} \\ \mathbf{n}_{\mathbf{b}_a} \\ \mathbf{0}_{3\times 1} \\ n_{\tau} \end{bmatrix}$$

其中 ${}^{G}\mathbf{R}_{I}$、${}^{G}\mathbf{p}_{I}$ 和 ${}^{G}\mathbf{v}_{I}$ 分别表示 IMU 在全局坐标系下的姿态、位置和速度；${}^{G}\mathbf{g}$ 是全局坐标系下的重力向量；$\tau$ 是相对于第一帧的相机逆曝光时间；$n_{\tau}$ 是将 $\tau$ 建模为随机游走的高斯噪声；$\bm{\omega}_m$ 和 $\mathbf{a}_m$ 是原始 IMU 测量；$\mathbf{n}_g$ 和 $\mathbf{n}_a$ 是 $\bm{\omega}_m$ 和 $\mathbf{a}_m$ 中的测量噪声；$\mathbf{b}_a$ 和 $\mathbf{b}_g$ 是 IMU 零偏，分别建模为由高斯噪声 $\mathbf{n}_{\mathbf{b}_g}$ 和 $\mathbf{n}_{\mathbf{b}_a}$ 驱动的随机游走。

### IV-B. 扫描重组

我们采用扫描重组将高频、顺序采样的 LiDAR 原始点在相机采样时刻分割为不同的 LiDAR 扫描，如图 2 所示。这确保相机和 LiDAR 数据在相同频率（如 10 Hz）下同步，允许在同一时刻更新状态。

### IV-C. 传播

在 ESIKF 框架中，状态和协方差从接收到上一帧 LiDAR 扫描和图像帧的时刻 $t_{k-1}$ 传播到接收到当前 LiDAR 扫描和图像帧的时刻 $t_k$。这种**前向传播**通过将式(1)中的过程噪声 $\mathbf{w}_i$ 设为零，在 $t_{k-1}$ 和 $t_k$ 期间对每个 IMU 输入 $\mathbf{u}_i$ 预测状态。记传播后的状态为 $\widehat{\mathbf{x}}$，协方差为 $\widehat{\mathbf{P}}$，它们将作为第 IV-D 节后续更新的先验分布。

此外，为补偿运动畸变，我们如[42]中那样进行**后向传播**，确保 LiDAR 扫描中的点在扫描结束时刻 $t_k$ "被测量"。注意为简化符号，我们在所有状态向量中省略下标 $k$。

### IV-D. 顺序更新

IMU 传播的状态 $\widehat{\mathbf{x}}$ 和协方差 $\widehat{\mathbf{P}}$ 为时刻 $t_k$ 的系统状态施加先验分布：

$$\mathbf{x} \boxminus \widehat{\mathbf{x}} \sim \mathcal{N}(\mathbf{0}, \widehat{\mathbf{P}}) \tag{3}$$

我们将上述先验分布记为 $p(\mathbf{x})$，LiDAR 和相机的测量模型为：

$$\begin{bmatrix} \mathbf{y}_l \\ \mathbf{y}_c \end{bmatrix} = \begin{bmatrix} \mathbf{h}_l(\mathbf{x}, \mathbf{v}_l) \\ \mathbf{h}_c(\mathbf{x}, \mathbf{v}_c) \end{bmatrix} \tag{4}$$

其中 $\mathbf{v}_l \sim \mathcal{N}(\mathbf{0}, \bm{\Sigma}_{\mathbf{v}_l})$ 和 $\mathbf{v}_c \sim \mathcal{N}(\mathbf{0}, \bm{\Sigma}_{\mathbf{v}_c})$ 分别表示 LiDAR 和相机的测量噪声。

标准 ESIKF[43]会使用所有当前测量（包括 LiDAR 测量 $\mathbf{y}_l$ 和图像测量 $\mathbf{y}_c$）更新状态 $\mathbf{x}$。然而，LiDAR 和图像测量是两种不同的传感模式，其数据维度不匹配。此外，图像测量的融合可能在图像金字塔的不同层级进行。为解决维度不匹配并为每个模块提供更大灵活性，我们提出**顺序更新策略**。假设在给定状态向量 $\mathbf{x}$ 的情况下 LiDAR 测量 $\mathbf{y}_l$ 和图像测量 $\mathbf{y}_c$ 统计独立（即测量被统计独立的噪声污染），该策略在理论上等价于使用所有测量的标准更新。

为引入顺序更新，我们将当前状态 $\mathbf{x}$ 的总条件分布重写为：

$$\begin{aligned} p(\mathbf{x} \mid \mathbf{y}_l, \mathbf{y}_c) &\propto p(\mathbf{x}, \mathbf{y}_l, \mathbf{y}_c) = p(\mathbf{y}_c \mid \mathbf{x}, \mathbf{y}_l) p(\mathbf{x}, \mathbf{y}_l) \\ &= p(\mathbf{y}_c \mid \mathbf{x}) \underbrace{p(\mathbf{y}_l \mid \mathbf{x}) p(\mathbf{x})}_{\propto p(\mathbf{x} \mid \mathbf{y}_l)} \end{aligned} \tag{5}$$

式(5)表明总条件分布 $p(\mathbf{x} \mid \mathbf{y}_l, \mathbf{y}_c)$ 可以通过两次顺序贝叶斯更新获得。第一步仅将 LiDAR 测量 $\mathbf{y}_l$ 与 IMU 传播的先验分布 $p(\mathbf{x})$ 融合，获得分布 $p(\mathbf{x} \mid \mathbf{y}_l)$：

$$p(\mathbf{x} \mid \mathbf{y}_l) \propto p(\mathbf{y}_l \mid \mathbf{x}) p(\mathbf{x}) \tag{6}$$

第二步然后将相机测量 $\mathbf{y}_c$ 与 $p(\mathbf{x} \mid \mathbf{y}_l)$ 融合，获得 $\mathbf{x}$ 的最终后验分布：

$$p(\mathbf{x} \mid \mathbf{y}_l, \mathbf{y}_c) \propto p(\mathbf{y}_c \mid \mathbf{x}) p(\mathbf{x} \mid \mathbf{y}_l) \tag{7}$$

有趣的是，式(6)和(7)中的两次融合遵循相同形式：

$$q(\mathbf{x} \mid \mathbf{y}) \propto q(\mathbf{y} \mid \mathbf{x}) q(\mathbf{x}) \tag{8}$$

为对 LiDAR 或图像测量执行式(8)中的融合，我们详细说明先验分布 $q(\mathbf{x})$ 和测量模型 $q(\mathbf{y} \mid \mathbf{x})$ 如下。

对于先验分布 $q(\mathbf{x})$，记为 $\mathbf{x} = \widehat{\mathbf{x}} \boxplus \bm{\delta}\mathbf{x}$，其中 $\bm{\delta}\mathbf{x} \sim \mathcal{N}(\mathbf{0}, \widehat{\mathbf{P}})$。在 LiDAR 更新（即第一步）的情况下，$(\widehat{\mathbf{x}}, \widehat{\mathbf{P}})$ 是从传播步骤获得的状态和协方差。在视觉更新（即第二步）的情况下，$(\widehat{\mathbf{x}}, \widehat{\mathbf{P}})$ 是从 LiDAR 更新获得的收敛状态和协方差。

为获得测量模型分布 $q(\mathbf{y} \mid \mathbf{x})$，记第 $\kappa$ 次迭代估计的状态为 $\widehat{\mathbf{x}}^{\kappa}$，其中 $\widehat{\mathbf{x}}^{0} = \widehat{\mathbf{x}}$。通过在 $\widehat{\mathbf{x}}^{\kappa}$ 处对测量模型(4)（LiDAR 或相机测量）进行一阶泰勒展开，得到：

$$\mathbf{y} \mid \mathbf{x} \simeq \underbrace{\mathbf{h}(\widehat{\mathbf{x}}^{\kappa}, \mathbf{0})}_{\mathbf{z}^{\kappa}} + \mathbf{H}^{\kappa} \bm{\delta}\mathbf{x}^{\kappa} + \mathbf{L}^{\kappa} \mathbf{v} \tag{9}$$

$$q(\mathbf{y} \mid \mathbf{x}) \simeq \mathcal{N}(\mathbf{h}(\widehat{\mathbf{x}}^{\kappa}, \mathbf{0}) + \mathbf{H}^{\kappa} \bm{\delta}\mathbf{x}^{\kappa}, \mathbf{R}) \tag{10}$$

其中 $\bm{\delta}\mathbf{x}^{\kappa} = \mathbf{x} \boxminus \widehat{\mathbf{x}}^{\kappa}$，$\mathbf{z}^{\kappa}$ 是残差，$\mathbf{L}^{\kappa}\mathbf{v} \sim \mathcal{N}(\mathbf{0}, \mathbf{R})$ 是集总测量噪声，$\mathbf{H}^{\kappa}$ 和 $\mathbf{L}^{\kappa}$ 分别是 $\mathbf{h}(\widehat{\mathbf{x}}^{\kappa} \boxplus \bm{\delta}\mathbf{x}^{\kappa}, \mathbf{v})$ 对 $\bm{\delta}\mathbf{x}^{\kappa}$ 和 $\mathbf{v}$ 的雅可比矩阵（在零处求值）。

然后，将先验分布 $q(\mathbf{x})$ 和式(10)中的测量分布 $q(\mathbf{y} \mid \mathbf{x})$ 代入后验分布(8)并执行最大似然估计（MLE），我们可以从 ESIKF 框架[43]中的标准更新步骤获得 $\bm{\delta}\mathbf{x}^{\kappa}$（以及 $\mathbf{x}^{\kappa}$）的最大后验估计（MAP）：

$$\begin{aligned} \mathbf{K} &= \left( (\mathbf{H}^{\kappa})^{T} \mathbf{R}^{-1} \mathbf{H}^{\kappa} + \widehat{\mathbf{P}}^{-1} \right)^{-1} (\mathbf{H}^{\kappa})^{T} \mathbf{R}^{-1}, \\ \widehat{\mathbf{x}}^{\kappa+1} &= \widehat{\mathbf{x}}^{\kappa} \boxplus \left( -\mathbf{K} \mathbf{z}^{\kappa} - (\mathbf{I} - \mathbf{K}\mathbf{H}^{\kappa}) (\widehat{\mathbf{x}}^{\kappa} \boxminus \widehat{\mathbf{x}}) \right) \end{aligned} \tag{11}$$

收敛的状态和协方差矩阵然后构成后验分布 $q(\mathbf{x} \mid \mathbf{y})$ 的均值和协方差。

具有顺序更新的卡尔曼滤波器已在文献中研究，如[44,45]。本文将此方法用于 LiDAR 和相机系统的 ESIKF。具有顺序更新的 ESIKF 的实现在算法 1 中详述。

在第一步（第 6-10 行）中，误差状态从 LiDAR 测量（第 VI-A 节）迭代更新直到收敛。收敛的状态和协方差估计（再次记为 $\widehat{\mathbf{x}}$ 和 $\widehat{\mathbf{P}}$）用于更新地图的几何结构（第 V-B 节），随后在第二步视觉更新（第 13-23 行）中在图像金字塔的每一层（第 VII-B 节）上进行细化直到收敛。最优状态和协方差（记为 $\bar{\mathbf{x}}$ 和 $\bar{\mathbf{P}}$）用于传播传入的 IMU 测量（第 IV-C 节）并更新地图的视觉结构（第 V-D 和 V-E 节）。

---

## V. 局部建图

### V-A. 地图结构

我们的地图采用[14]中提出的自适应体素结构，由哈希表和每个哈希条目的八叉树组织（图 1）。哈希表管理根体素，每个根体素的固定尺寸为 $0.5 \times 0.5 \times 0.5$ 米。每个根体素封装一个八叉树结构以进一步组织不同尺寸的叶体素。叶体素表示一个局部平面，存储平面特征（即平面中心、法向量和不确定性）以及位于该平面上的一组 LiDAR 原始点。

这些点中的一些附加有三层图像块（$8 \times 8$ 块大小），我们称之为**视觉地图点**。收敛的视觉地图点仅附加参考块，而非收敛的视觉地图点附加参考块和其他可见块（见第 V-E 节）。叶体素的可变尺寸使其能够表示不同尺度的局部平面，从而适应具有不同结构的环境[14]。

为防止地图大小无限制增长，我们仅在 LiDAR 当前位置周围长度为 $L$ 的大局部区域内维护局部地图，如图 3 的二维示例所示。最初，地图是以 LiDAR 起始位置 $\mathbf{p}_0$ 为中心的立方体。LiDAR 的检测区域可视化为以其当前位置为中心、半径由 LiDAR 检测范围定义的球体。当 LiDAR 移动到新位置 $\mathbf{p}_1$ 且检测区域触及地图边界时，我们将地图从边界移开距离 $d$。随着地图移动，存储移出局部地图片区的内存将被重置以存储移入局部地图的新片区。这种**环形缓冲区方法**确保我们的局部地图维持在固定大小的内存中。环形缓冲区哈希映射的实现在[46]中有详述。地图移动检查在每次 ESIKF 更新步骤后执行。

### V-B. 几何构建与更新

地图的几何结构由 LiDAR 点测量构建和更新。具体而言，在 ESIKF 中的 LiDAR 更新（第 IV 节）之后，我们将 LiDAR 扫描中的所有点配准到全局坐标系。对于每个配准后的 LiDAR 点，我们确定其在哈希映射中所在的根体素。如果该体素不存在，我们用新点初始化体素并将其索引到哈希映射中。如果确定的体素已存在于地图中，我们将该点追加到现有体素。

在扫描中的所有点被分配后，我们按如下方式进行几何构建和更新：

对于新创建的体素，我们基于奇异值分解确定所有包含的点是否位于一个平面上。如果是，我们计算平面的中心点 $\mathbf{q} = \bar{\mathbf{p}}$、平面法向 $\mathbf{n}$ 以及 $(\mathbf{q}, \mathbf{n})$ 的协方差矩阵，记为 $\bm{\Sigma}_{\mathbf{n},\mathbf{q}}$。$\bm{\Sigma}_{\mathbf{n},\mathbf{q}}$ 用于表征平面不确定性，该不确定性源自位姿估计不确定性和点测量噪声。详细的平面判据以及平面参数和不确定性的计算可参考我们先前的工作[14]。

如果包含的点不位于一个平面上，体素被持续细分为八个更小的八分体，直到子体素中的点被确定形成一个平面或达到最大层数（例如 3 层）。在后一种情况下，叶体素中的点将被丢弃。因此，地图仅包含被识别为平面的体素（根体素或子体素）。

对于追加了新点的现有体素，我们评估新点是否仍与根体素或子体素中的现有点形成平面。如果不是，我们按上述方式进行体素细分。如果是，我们同样更新平面参数（$\mathbf{q}$, $\mathbf{n}$）和协方差 $\bm{\Sigma}_{\mathbf{n},\mathbf{q}}$。

一旦平面参数收敛（见[14]），该平面将被视为**成熟**，该平面上的新点将被丢弃。此外，成熟平面的估计平面参数（$\mathbf{q}$, $\mathbf{n}$）和协方差 $\bm{\Sigma}_{\mathbf{n},\mathbf{q}}$ 将被固定。

平面上的 LiDAR 点（在根体素或子体素中）将用于在下一节中生成视觉地图点。对于成熟平面，最近的 50 个 LiDAR 点是视觉地图点生成的候选，而对于未成熟平面，所有 LiDAR 点都是候选。视觉地图点生成过程将识别这些候选点中的一些作为视觉地图点，并为它们附加图像块以进行图像对齐。

### V-C. 视觉地图点生成与更新

为生成和更新视觉地图点，我们选择地图中满足以下条件的候选 LiDAR 点：(1) 从当前帧可见（详见第 VII-A 节），(2) 在当前图像中表现出显著的灰度梯度。

我们在视觉更新（第 IV-D 节）之后将这些候选点投影到当前图像，并在每个体素的局部平面中保留深度最小的候选点。然后，我们将当前图像划分为均匀的网格单元，每个单元为 $30 \times 30$ 像素。如果一个网格单元不包含任何投影到此处的视觉地图点，我们使用灰度梯度最高的候选点生成一个新的视觉地图点，并将其与当前图像块、估计的当前状态（即帧位姿和曝光时间）以及从上一节中 LiDAR 点计算的平面法向相关联。

附加到视觉地图点的块具有三层相同大小（例如 $11 \times 11$ 像素），每一层是上一层的半采样，形成块金字塔。

如果一个网格单元包含投影到此处的视觉地图点，我们在以下条件之一满足时向现有视觉地图点添加新块（金字塔的所有三层）：(1) 自上次添加块以来已超过 20 帧，或 (2) 其在当前帧中的像素位置与上次添加块时的位置偏差超过 40 像素。因此，地图点将可能具有视角均匀分布的有效块。伴随块金字塔，我们还将估计的当前状态（即位姿和曝光时间）附加到地图点。

### V-D. 参考块更新

一个视觉地图点可能由于添加新块而拥有多个块。我们需要为视觉更新中的图像对齐选择一个参考块。具体而言，我们基于光度相似性和视角对每个块 $\mathbf{f}$ 评分如下：

$$\text{NCC}(\mathbf{f}, \mathbf{g}) = \frac{\sum_{x,y} [\mathbf{f}(x,y) - \bar{\mathbf{f}}][\mathbf{g}(x,y) - \bar{\mathbf{g}}]}{\sqrt{\sum_{x,y} [\mathbf{f}(x,y) - \bar{\mathbf{f}}]^2 \sum_{x,y} [\mathbf{g}(x,y) - \bar{\mathbf{g}}]^2}}$$

$$c = \frac{\mathbf{n} \cdot \mathbf{p}}{\|\mathbf{p}\|}, \quad \omega_1 = \frac{1}{1 + e^{\text{tr}(\bm{\Sigma}_{\mathbf{n}})}}$$

$$S = (1 - \omega_1) \cdot \frac{1}{n} \sum_{i=1}^{n} \text{NCC}(\mathbf{f}, \mathbf{g}_i) + \omega_1 \cdot c \tag{12}$$

其中 $\text{NCC}(\mathbf{f}, \mathbf{g})$ 表示归一化互相关（NCC），用于测量块 $\mathbf{f}$ 和 $\mathbf{g}$ 在两个块的第 0 层金字塔（最高分辨率层）之间的相似性，两个块均进行了均值减法；$c$ 表示法向量 $\mathbf{n}$ 与被评估块 $\mathbf{f}$ 的视角方向 $\mathbf{p}/\|\mathbf{p}\|$ 之间的余弦相似性。当块直接面向地图点所在平面时，$c$ 的值为 1。

总分 $S$ 通过加权 NCC 和 $c$ 求和计算，其中前者表示被评估块 $\mathbf{f}$ 与所有其他块 $\mathbf{g}_i$ 之间的平均相似性，$\text{tr}(\bm{\Sigma}_{\mathbf{n}})$ 表示法向量协方差矩阵的迹。

在附加到视觉地图点的所有块中，得分最高的块被更新为参考块。上述评分机制倾向于选择满足以下条件的参考块：(1) 外观（在 NCC 方面）与大多数其他块相似，这是 MVS[47]使用的一种技术，用于避免动态物体上的块；(2) 视角方向与平面正交，从而以高分辨率保持纹理细节。

相比之下，我们先前工作 FAST-LIVO[8]和现有技术[4]中的参考块更新策略直接选择与当前帧视角方向差异最小的块，导致选定的参考块非常接近当前帧，从而对当前位姿更新施加弱约束。

### V-E. 法向细化

每个视觉地图点假设位于一个小的局部平面上。现有工作[4,2,8]假设一个块中的所有像素具有相同深度，这一大胆假设通常不成立。我们使用第 V-B 节中详述的从 LiDAR 点计算的平面参数以实现更高精度。该平面法向对于在视觉更新过程中执行图像对齐的仿射变换至关重要。

为进一步增强仿射变换的精度，平面法向可以从附加到视觉地图点的块中进一步细化。具体而言，我们通过最小化参考块与附加到视觉地图点的其他块之间的光度误差来细化参考块中的平面法向。

#### V-E1. 仿射变换

仿射变换用于将块像素从参考帧（即源块）变换到其余帧（即目标块）中的块像素，如图 4(a)所示。设 $\mathbf{u}^{j}_{r}$ 为源块中的第 $j$ 个像素坐标，$\mathbf{u}^{j}_{i}$ 为第 $i$ 个目标块中的第 $j$ 个像素坐标。假设块中的所有像素位于一个具有法向 ${}^{I_r}\mathbf{n}$ 和视觉地图点位置 ${}^{I_r}\mathbf{p}$（对应于源块和目标块的中心像素）的局部平面上，两者均在源块坐标系中表示，我们有：

$$\mathbf{u}^{j}_{i} = \mathbf{A}^{i}_{r} \mathbf{u}^{j}_{r} \tag{13}$$

$$\mathbf{A}^{i}_{r} = \mathbf{P} \left( {}^{I_i}\mathbf{R}_{I_r} + {}^{I_i}\mathbf{t}_{I_r} \frac{1}{{}^{I_r}\mathbf{n}^{T} \cdot {}^{I_r}\mathbf{p}} {}^{I_r}\mathbf{n}^{T} \right) \mathbf{P}^{-1}$$

其中 $\mathbf{A}^{i}_{r}$ 表示将像素坐标从源（或参考）块变换到第 $i$ 个目标块的仿射变换矩阵；${}^{I_i}\mathbf{R}_{I_r}$ 和 ${}^{I_i}\mathbf{t}_{I_r}$ 表示参考帧 $I_r$ 相对于目标帧 $I_i$ 的相对位姿。

为直接使用鱼眼图像而不将其校正为针孔图像，我们基于不同的相机模型实现投影矩阵 $\mathbf{P}$ 和反投影矩阵 $\mathbf{P}^{-1}$（例如，对于针孔相机模型，$\mathbf{P}$ 是相机内参矩阵）。

#### V-E2. 法向优化

为细化平面法向 ${}^{I_r}\mathbf{n}$，我们在第 0 层金字塔（即最高分辨率层）最小化参考块与其他图像块之间的光度误差：

$${}^{I_r}\mathbf{n}^{*} = \arg\min_{{}^{I_r}\mathbf{n} \in \mathbb{S}^2} \sum_{i \in S} \sum_{j=1}^{N^2} \left\| \tau_i \mathbf{I}_i(\mathbf{A}^{i}_{r} \mathbf{u}^{j}_{r}) - \tau_r \mathbf{I}_r(\mathbf{u}^{j}_{r}) \right\|_2 \tag{14}$$

其中 $N$ 是块大小，$\tau_r$ 和 $\tau_i$ 分别是参考帧和第 $i$ 个目标帧的逆曝光时间；$\mathbf{I}_r(\mathbf{u}^{j}_{r})$ 表示参考帧中的第 $j$ 个块像素；$\mathbf{I}_i(\mathbf{A}^{i}_{r} \mathbf{u}^{j}_{r})$ 表示第 $i$ 个目标帧中的第 $j$ 个块像素；$S$ 是所有目标帧的集合。

#### V-E3. 优化变量变换

为提高计算效率，我们对式(14)中的最小二乘问题进行重参数化。注意到优化变量 ${}^{I_r}\mathbf{n}$ 仅出现在式(13)中的 $\mathbf{M} \triangleq \frac{1}{{}^{I_r}\mathbf{n}^{T} \cdot {}^{I_r}\mathbf{p}} {}^{I_r}\mathbf{n} \in \mathbb{R}^3$ 中，对 ${}^{I_r}\mathbf{n}$ 的优化可以在 $\mathbf{M}$ 上进行。此外，向量 $\mathbf{M}$ 受约束 ${}^{I_r}\mathbf{p} \cdot \mathbf{M} = 1$，这意味着 $\mathbf{M}$ 可以参数化为：

$$\mathbf{M} = \begin{bmatrix} \mathbf{M}_x \\ \mathbf{M}_y \\ \frac{1}{{}^{I_r}\mathbf{p}_z} - \frac{{}^{I_r}\mathbf{p}_x}{{}^{I_r}\mathbf{p}_z} \mathbf{M}_x - \frac{{}^{I_r}\mathbf{p}_y}{{}^{I_r}\mathbf{p}_z} \mathbf{M}_y \end{bmatrix} = \mathbf{B}\mathbf{m} + \mathbf{b} \tag{15}$$

$$\mathbf{B} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \\ -\frac{{}^{I_r}\mathbf{p}_x}{{}^{I_r}\mathbf{p}_z} & -\frac{{}^{I_r}\mathbf{p}_y}{{}^{I_r}\mathbf{p}_z} \end{bmatrix}, \quad \mathbf{b} = \begin{bmatrix} 0 \\ 0 \\ \frac{1}{{}^{I_r}\mathbf{p}_z} \end{bmatrix}, \quad \mathbf{m} = \begin{bmatrix} \mathbf{M}_x \\ \mathbf{M}_y \end{bmatrix} \in \mathbb{R}^2$$

其中 ${}^{I_r}\mathbf{p}_z \neq 0$，因为不会为视觉地图点选择这样的参考块。${}^{I_r}\mathbf{n}$、$\mathbf{M}$ 和 $\mathbf{m}$ 之间的关系如图 4(b)所示。

最后，式(14)中的优化在无约束的向量 $\mathbf{m} \in \mathbb{R}^2$ 上进行。该优化可以在**独立线程**中执行以避免阻塞主里程计线程。优化后的参数 $\mathbf{m}^{*}$ 然后可用于恢复最优法向量 ${}^{I_r}\mathbf{n}^{*}$：

$${}^{I_r}\mathbf{n}^{*} = \frac{\mathbf{M}^{*}}{\|\mathbf{M}^{*}\|}, \quad \mathbf{M}^{*} = \mathbf{B}\mathbf{m}^{*} + \mathbf{b} \tag{16}$$

一旦平面法向收敛，该视觉地图点的参考块和法向量被固定而不再细化，所有其他块被删除。

---

## VI. LiDAR 测量模型

本节详述在第 IV-D 节 ESIKF 的 LiDAR 更新中使用的 LiDAR 测量模型 $\mathbf{y}_l = \mathbf{h}_l(\mathbf{x}, \mathbf{v}_l)$。

### VI-A. 点到平面 LiDAR 测量模型

在获得扫描中去畸变后的点 $\{{}^{L}\mathbf{p}_j\}$ 后，我们使用 LiDAR 更新第 $\kappa$ 次迭代时的估计状态 $\widehat{\mathbf{x}}^{\kappa}$ 将它们投影到全局坐标系：

$${}^{G}\widehat{\mathbf{p}}^{\kappa}_{j} = {}^{G}\widehat{\mathbf{T}}^{\kappa}_{I} {}^{I}\mathbf{T}_{L} {}^{L}\mathbf{p}_{j} \tag{17}$$

然后我们确定 ${}^{G}\widehat{\mathbf{p}}^{\kappa}_{j}$ 在哈希映射中所在的根体素或子体素。如果未找到体素或体素不包含平面，则该点被丢弃。否则，我们使用体素中的平面为 LiDAR 点建立测量方程。

具体而言，我们假设真实 LiDAR 点 ${}^{L}\mathbf{p}^{gt}_{j}$ 在精确 LiDAR 位姿 ${}^{G}\mathbf{T}_{I}$ 下应位于体素中具有法向 $\mathbf{n}^{gt}_{j}$ 和中心点 $\mathbf{q}^{gt}_{j}$ 的平面上，即：

$$\mathbf{0} = (\mathbf{n}^{gt}_{j})^{T} ({}^{G}\mathbf{T}_{I} {}^{I}\mathbf{T}_{L} {}^{L}\mathbf{p}^{gt}_{j} - \mathbf{q}^{gt}_{j}) \tag{18}$$

由于真实点 ${}^{L}\mathbf{p}^{gt}_{j}$ 被测量为 ${}^{L}\mathbf{p}_{j}$，带有测距和方位噪声 $\bm{\delta}{}^{L}\mathbf{p}_{j}$，我们有 ${}^{L}\mathbf{p}^{gt}_{j} = {}^{L}\mathbf{p}_{j} - \bm{\delta}{}^{L}\mathbf{p}_{j}$。同样，平面参数 $(\mathbf{n}^{gt}_{j}, \mathbf{q}^{gt}_{j})$ 被估计为 $(\mathbf{n}_{j}, \mathbf{q}_{j})$，带有协方差 $\bm{\Sigma}_{\mathbf{n},\mathbf{q}}$（第 V-B 节），因此我们有：$\mathbf{n}^{gt}_{j} = \mathbf{n}_{j} \boxminus \bm{\delta}\mathbf{n}_{j}$，$\mathbf{q}^{gt}_{j} = \mathbf{q}_{j} - \bm{\delta}\mathbf{q}_{j}$。

因此：

$$\underbrace{\mathbf{0}}_{\mathbf{y}_l} = \underbrace{(\mathbf{n}_{j} \boxminus \bm{\delta}\mathbf{n}_{j})^{T} ({}^{G}\mathbf{T}_{I} {}^{I}\mathbf{T}_{L} ({}^{L}\mathbf{p}_{j} - \bm{\delta}{}^{L}\mathbf{p}_{j}) - (\mathbf{q}_{j} - \bm{\delta}\mathbf{q}_{j}))}_{\mathbf{h}_l(\mathbf{x}, \mathbf{v}_l)} \tag{19}$$

其中测量噪声 $\mathbf{v}_l = (\bm{\delta}{}^{L}\mathbf{p}_{j}, \bm{\delta}\mathbf{n}_{j}, \bm{\delta}\mathbf{q}_{j})$ 由与 LiDAR 点、法向量和平面中心相关的噪声组成。

### VI-B. 考虑光束发散角的 LiDAR 测量噪声

局部 LiDAR 坐标系中 LiDAR 点的不确定性 $\bm{\delta}{}^{L}\mathbf{p}_{j}$ 在[14]中被分解为两个分量：由激光飞行时间（TOF）引起的测距不确定性 $\delta d$，以及源自编码器的方位方向不确定性 $\bm{\delta}\bm{\omega}$。除了这些不确定性，我们还考虑由激光光束发散角 $\theta$ 引起的不确定性，如图 5 所示。随着方位方向与法向量之间的角度 $\varphi$ 增大，LiDAR 点的测距不确定性显著增加，而方位方向不确定性不受影响。

由激光光束发散角引起的 $\delta d$ 可以建模为：

$$\delta d = L_2 - L_1 = d \left( \frac{\cos\varphi}{\cos(\theta+\varphi)} - \frac{\cos\varphi}{\cos(\theta-\varphi)} \right) \tag{20}$$

考虑受 TOF 和激光光束发散影响的 $\delta d$，当我们的系统从地面或墙壁选择更多点时（见图 5(c,d)），它比不考虑这种效应的系统实现更精确的位姿估计。

---

## VII. 视觉测量模型

本节详述在第 IV-D 节 ESIKF 的视觉更新中使用的视觉测量模型 $\mathbf{y}_c = \mathbf{h}_c(\mathbf{x}, \mathbf{v}_c)$。

### VII-A. 视觉地图点选择

为在视觉更新中执行稀疏图像对齐，我们首先选择适当的视觉地图点。我们首先使用体素和射线投射查询提取当前相机视场内可见的地图点集合（称为视觉子地图）。然后，从该子地图中选择视觉地图点并剔除外点。此过程产生一组精炼的视觉地图点，准备在视觉测量模型中构建视觉光度误差。

#### VII-A1. 可见体素查询

由于地图中体素数量庞大，识别当前帧视场内的地图体素具有挑战性。为解决此问题，我们查询当前扫描中 LiDAR 点击中的体素。这可以通过使用测量点位置查询体素哈希表高效完成。如果相机视场与 LiDAR 视场大部分重叠，则相机视场中的地图点很可能也位于这些体素中。

我们还查询上一帧图像中被识别为可见（通过相同的体素查询和射线投射）的地图点所击中的体素，假设两个连续图像帧具有大的视场重叠。最后，当前视觉子地图可以作为包含在这两类体素中的地图点，然后进行视场检查获得。

#### VII-A2. 按需射线投射

在大多数情况下，视觉子地图可以通过上述体素查询获得。然而，当 LiDAR 传感器离物体太近时可能返回无点（称为**近距盲区**）。此外，相机视场可能未被 LiDAR 视场完全覆盖。为在这些情况下召回更多视觉地图点，我们采用如图 6 所示的射线投射策略。

我们将图像划分为均匀的网格单元，每个单元为 $30 \times 30$ 像素，并将从体素查询获得的视觉地图点投影到网格单元上。对于每个未被这些视觉地图点占据的图像网格单元，沿中心像素向后投射一条射线，其中采样点沿射线在深度方向从 $d_{\min}$ 到 $d_{\max}$ 均匀分布。为减少计算负载，每条射线上采样点在相机本体坐标系中的位置是预计算的。

对于每个采样点，我们评估对应体素的状态：如果体素包含投影后位于该网格单元的地图点，我们将这些地图点纳入视觉子地图并停止该射线。否则，我们继续到射线上的下一个采样点，直到达到最大深度 $d_{\max}$。在通过射线投射处理所有未占据的图像网格单元后，我们获得一组分布在整个图像中的视觉地图点。

#### VII-A3. 外点剔除

在体素查询和射线投射之后，我们获得当前帧视场内的所有视觉地图点。然而，这些视觉地图点可能在当前帧中被遮挡、具有不连续深度、其参考块在大视角下拍摄，或在当前帧中具有大视角，所有这些都会严重降低图像对齐精度。

为解决第一个问题，我们使用 LiDAR 更新后的位姿将子地图中的所有视觉地图点投影到当前帧，并在每个 $30 \times 30$ 像素的网格单元中保留深度最小的点。

为解决第二个问题，我们将当前 LiDAR 扫描中的 LiDAR 点投影到当前帧生成深度图。通过将视觉地图点的深度与其在深度图中的 $9 \times 9$ 邻域进行比较，我们确定它们的遮挡和深度变化。被遮挡和深度不连续的地图点被剔除（见图 7）。

为解决第三和第四个问题，我们移除参考块或当前块的视角（即法向量与从视觉地图点到块光学中心的方向之间的角度）过大（例如超过 80°）的点。

剩余的视觉地图点将用于对齐当前图像。

### VII-B. 稀疏直接视觉测量模型

上述提取的视觉地图点 $\{{}^{G}\mathbf{p}_i\}$ 用于构建视觉测量模型。基本原理是，当使用真实状态（即位姿）$\mathbf{x}_k$ 将地图点 ${}^{G}\mathbf{p}_i$ 变换到当前图像 $\mathbf{I}_k(\cdot)$ 时，参考块与当前块之间的光度误差应为零：

$$\begin{aligned} \mathbf{0} &= \tau_k \mathbf{I}_{k}^{gt}(\underbrace{\bm{\pi}({}^{C}\mathbf{T}_{I} ({}^{G}\mathbf{T}_{I})^{-1} {}^{G}\mathbf{p}_{i})}_{\mathbf{u}_i} + \Delta\mathbf{u}) \\ &\quad\quad\quad - \tau_r \mathbf{I}_{r}^{gt}(\underbrace{\bm{\pi}({}^{C_r}\mathbf{T}_{G} {}^{G}\mathbf{p}_{i})}_{\mathbf{u}'_i} + \mathbf{A}^{r}_{i} \Delta\mathbf{u}) \end{aligned} \tag{21}$$

其中 $\bm{\pi}(\cdot)$ 是通用相机投影模型（即针孔、MEI、ATAN、Scaramuzza、等距）；${}^{C_r}\mathbf{T}_{G}$ 是全局坐标系 $G$ 相对于参考帧 $C_r$ 的位姿，在接收和融合参考帧时已估计；$\mathbf{A}^{r}_{i}$ 是将像素从第 $i$ 个当前块变换到参考块的仿射变换矩阵；$\Delta\mathbf{u}$ 是当前块内相对于中心 $\mathbf{u}_i$ 的相对像素位置；$\mathbf{I}_{k}^{gt}, \mathbf{I}_{r}^{gt}$ 分别表示参考帧和当前帧的真实像素值。

它们被测量为实际图像像素值 $\mathbf{I}_k, \mathbf{I}_r$，带有测量噪声 $\mathbf{v}_c = (\bm{\delta}\mathbf{I}_k, \bm{\delta}\mathbf{I}_r)$，这些噪声源自各种来源（例如散粒噪声和相机 CMOS 的模数转换（ADC）噪声）。因此：

$$\underbrace{\mathbf{0}}_{\mathbf{y}_c} = \underbrace{\tau_k (\mathbf{I}_k(\mathbf{u}_i + \Delta\mathbf{u}) - \bm{\delta}\mathbf{I}_k) - \tau_r (\mathbf{I}_r(\mathbf{u}'_i + \mathbf{A}^{r}_{i} \Delta\mathbf{u}) - \bm{\delta}\mathbf{I}_r)}_{\mathbf{h}_c(\mathbf{x}, \mathbf{v}_c)} \tag{22}$$

为提高计算效率，我们采用逆组合公式[48,4]，其中参数化 ${}^{G}\mathbf{T}_{I} = {}^{G}\widehat{\mathbf{T}}^{\kappa}_{I} \text{Exp}(\bm{\delta}\mathbf{T})$ 的位姿增量 $\bm{\delta}\mathbf{T} \in \mathbb{R}^6$ 在 $\mathbf{u}_i$ 中（见式(21)），被从 $\mathbf{u}_i$ 移到 $\mathbf{u}'_i$，如下：

$$\begin{aligned} \mathbf{u}_i &= \bm{\pi}({}^{C}\mathbf{T}_{I} ({}^{G}\widehat{\mathbf{T}}^{\kappa}_{I})^{-1} {}^{G}\mathbf{p}_i) \\ \mathbf{u}'_i &= \bm{\pi}({}^{C_r}\mathbf{T}_{G} \, \text{Exp}(\bm{\delta}\mathbf{T}) {}^{G}\mathbf{p}_i) \end{aligned} \tag{23}$$

由于参考帧中的 $\mathbf{u}'_i$ 在每次迭代期间保持不变，我们只需要一次性计算相对于 $\bm{\delta}\mathbf{T}$ 的雅可比矩阵，而不是为每次迭代重新计算。

为从测量方程(22)估计逆曝光时间 $\tau_k$，我们固定初始逆曝光时间 $\tau_0 = 1$，以消除当所有逆曝光时间为零时方程(22)的退化。因此，后续帧估计的逆曝光时间是相对于第一帧的曝光时间。

式(22)在三个层级上用于视觉更新步骤（见算法 1）；视觉更新从最粗层开始，在一层收敛后，进入下一更细层。估计的状态然后用于生成视觉地图点（第 V-C 节）和更新参考块（第 V-D 节）。

---

## VIII. 评估数据集

本节介绍用于性能评估的数据集，包括公开数据集 NTU-VIRAL[49]、Hilti'22[50]、Hilti'23[51]和 MARS-LVIG[52]，以及我们自采集的 FAST-LIVO2 私有数据集。具体而言，NTU-VIRAL 和 Hilti 数据集用于对我们的系统与其他最先进（SOTA）SLAM 系统进行定量基准比较（第 IX-B 节）。FAST-LIVO2 私有数据集主要用于评估我们的系统在各种极端挑战性场景下的表现（第 IX-C 节），展示其高精度建图能力（第 IX-D 节），并验证系统内各个模块的功能（补充材料第 I-A 至 I-D 节）。MARS-LVIG 数据集用于应用演示（第 X 节）和消融研究（补充材料第 I-E 节）。

### VIII-A. NTU-VIRAL、Hilti 和 MARS-LVIG 数据集

**NTU-VIRAL 数据集**在南洋理工大学校园使用空中平台采集，呈现了体现独特空中操作挑战的多样化场景。具体而言，"sbs"序列只能从远处物体提供有噪声的视觉特征。"nya"序列由于半透明表面对 LiDAR SLAM 提出挑战，由于复杂的飞行动力学和低光照条件对视觉 SLAM 提出挑战。该数据集配备 16 通道 OS1 gen1 LiDAR（10 Hz 采样）和内置 IMU（100 Hz），以及两个同步针孔相机（10 Hz 触发）。评估使用左相机。

**Hilti'22 和 Hilti'23 数据集**由手持和机器人设备采集，涵盖来自建筑工地、办公室、实验室和停车场等环境的室内和室外序列。这些序列引入了来自长廊、地下室和楼梯的众多挑战，具有无纹理特征、变化的光照条件和不足的 LiDAR 平面约束。手持序列使用 Hesai PandarXT-32 LiDAR（10 Hz）、五个广角相机（40 Hz，下采样到 10 Hz）和外部 Bosch BMI085 IMU（400 Hz）。同时，机器人安装序列配备 Robosense BPearl LiDAR（10 Hz）、八个全向相机（10 Hz）和 Xsens MTi-670 IMU（200 Hz）。在两种情况下，所有评估系统都使用前向相机。每个序列提供通过运动捕捉系统（MoCap）或全站仪[54]获得的毫米级真值。注意 Hilti 数据集的真值不开源；因此，这些数据集上的算法结果通过 Hilti 官方网站评估。由于 Hilti'23 中的"Site 3"不提供深入分析图（如 RMSE），我们排除了这四个序列，但我们对这些序列的评分结果仍可在其官方网站上找到。NTU-VIRAL 和 Hilti 共贡献 25 个序列。

**MARS-LVIG 数据集**提供高空、面向地面的建图数据，涵盖丛林、山脉和岛屿等多样化的非结构化地形。该数据集通过 DJI M300 RTK 四旋翼无人机采集，配备 Livox Avia LiDAR（内置 BMI088 IMU）和高分辨率全局快门相机，均以 10 Hz 触发。这与前述使用 $752 \times 480$ 灰度图像的 NTU-VIRAL 和 Hilti 数据集明显不同，MARS 数据集使用 $2448 \times 2048$ RGB 图像，从而有助于生成清晰、稠密的彩色点云。因此，我们利用该公开数据集验证我们在高空机载建图应用中的能力。

### VIII-B. FAST-LIVO2 私有数据集

为验证系统在更极端条件下（例如 LiDAR 退化、低光照、剧烈曝光变化以及无 LiDAR 测量的情况）的性能，我们制作了一个名为 FAST-LIVO2 私有数据集的新数据集。该数据集、硬件设备和硬件同步方案随本工作的代码一起发布，以方便复现我们的工作。

#### VIII-B1. 平台

我们的数据采集平台（图 8）配备工业相机（MV-CA013-21UC）、Livox Avia LiDAR 和 DJI manifold-2c（Intel i7-8550u CPU 和 8 GB RAM）作为机载计算机。相机视场为 $70.6^\circ \times 68.5^\circ$，LiDAR 视场为 $70.4^\circ \times 77.2^\circ$。所有传感器通过 STM32 同步定时器生成的 10 Hz 触发信号进行硬件同步。

#### VIII-B2. 序列描述

如补充材料表 S1 所总结，FAST-LIVO2 私有数据集包含 20 个序列，涵盖各种场景（如校园建筑、走廊、地下室、采矿隧道等），特征为无结构、杂乱、昏暗、可变光照和弱纹理环境，总时长为 66.9 分钟。大多数序列表现出视觉和/或 LiDAR 退化，例如面向单一和/或无纹理平面、穿越极其狭窄和/或黑暗隧道、以及经历从室内到室外的变化光照条件（见补充材料图 S7）。为保证相机与 LiDAR 之间增强的同步数据采集，我们在大多数场景中将相机配置为固定曝光时间但自动增益模式。对于其余具有自动曝光的序列，我们记录其真值曝光时间。在所有序列中，平台返回起始点，从而能够进行漂移评估。

---

## IX. 实验结果

### IX-A. 实现与系统配置

我们以 C++ 和机器人操作系统（ROS）实现了所提的 FAST-LIVO2 系统。在默认配置中，启用曝光时间估计，而关闭法向量细化。扫描中的 LiDAR 点以 1:3 的比例进行时间下采样。体素地图的根体素大小设为 0.5 m，内部八叉树的最大层数为 3。图像块大小对于图像对齐为 $8 \times 8$，对于法向细化为 $11 \times 11$。

在顺序 ESIKF 设置中，对于所有实验，相机光度噪声设为恒定值 100。LiDAR 深度误差和方位角误差对于 Livox Avia LiDAR 和 OS1-16 调整为 0.02 m 和 0.05°，对于 PandarXT-32 调整为 0.001 m 和 0.001°，对于 Robosense BPearl LiDAR 调整为 0.008 m 和 0.01°。激光光束发散角对于 Livox Avia LiDAR 和 OS1-16 设为 0.15°，对于 PandarXT-32 和 Robosense BPearl LiDAR 设为 0.001°。我们的系统在具有相同传感器设置的所有数据集的所有序列中使用相同参数。

所有实验的计算平台是配备 Intel i7-10700K CPU 和 32 GB RAM 的台式 PC。对于 FAST-LIVO2，我们还在常用于嵌入式系统的 ARM 处理器上进行了测试，具有降低的功耗和成本。ARM 平台是 RB5，配备 Qualcomm Kryo585 CPU 和 8 GB RAM。我们将基于 ARM 平台的 FAST-LIVO2 实现称为"FAST-LIVO2 (ARM)"。

### IX-B. 基准实验

在本实验中，我们对来自 NTU-VIRAL、Hilti'22 和 Hilti'23 公开数据集的 25 个序列进行定量评估。我们的方法与几个最先进的开源里程计系统进行基准比较，包括 R3LIVE[36]（稠密直接激光-惯性-视觉里程计系统）、FAST-LIO2[13]（直接激光-惯性里程计系统）、SDV-LOAM[40]（半直接激光-视觉里程计系统）、LVI-SAM[35]（基于特征的激光-惯性-视觉 SLAM 系统）以及我们先前的工作 FAST-LIVO[8]。

所有方法的结果如表 II 所示。可以看出，我们的方法在所有序列中实现了最高的整体精度，平均 RMSE 为 **0.044 m**，比第二名 FAST-LIVO 的 0.137 m 精确三倍。我们的系统在大多数序列中取得了最佳结果，除了"Outside Building"和"Large Room (dark)"，在这些序列中我们的系统与纯激光-惯性里程计 FAST-LIO2 相比表现出略微（毫米级）更高的误差。这种差异可归因于这些序列具有丰富的结构特征但光照条件差，导致图像昏暗模糊。因此，融合这些低质量图像并不能提升里程计精度。

排除这两个序列后，我们利用紧密耦合的 LiDAR、惯性和视觉信息的方法显著优于 FAST-LIO2（我们的 LIO 子系统）和纯激光-视觉里程计 SDV-LOAM。值得注意的是，SDV-LOAM 在 Hilti 数据集上表现特别差，因为它缺乏与 IMU 测量的紧密集成，导致 LO 子系统漂移。此外，LiDAR 与视觉观测之间的松耦合，以及 VO 的不良初始值，往往导致局部最优甚至负优化。

我们的 LIO 子系统通常优于 FAST-LIO2，因为我们对每个 LiDAR 点进行了更精确的噪声建模。在 FAST-LIO2 略微优于的少数序列中，差异是毫米级的，可以忽略不计。

此外，我们系统的精度在所有序列中显著超过其他紧密耦合的激光-惯性-视觉系统。其中，LVI-SAM 在九个序列中失败，主要因为其基于特征的 LIO 和 VIO 子系统未充分利用原始测量，这降低了其在具有细微几何或纹理特征环境中的鲁棒性。R3LIVE 通常表现良好，但在"Construction Stairs"、"Cupola"和"Attic to Upper Gallery"序列中表现挣扎，其性能甚至比 FAST-LIO2 更差。这是因为在无结构楼梯处的剧烈旋转导致位姿先验不足，在将彩色地图点与当前帧对齐时引起局部最优，最终导致负优化。FAST-LIVO 和 FAST-LIVO2 通过基于块的图像对齐克服了这些序列中的挑战。

另一方面，FAST-LIVO 在 NTU-VIRAL 数据集上被 R3LIVE 和 FAST-LIVO2 超越，特别是在"nya"序列等非结构化场景中，其中基于恒定深度假设的仿射变换效果不准确。相比之下，R3LIVE 的像素级对齐和 FAST-LIVO2 的平面先验（或细化）不会遇到此类问题。

比较 FAST-LIVO2 的不同变体，我们观察到没有实时曝光时间估计的平均精度比默认值降低 6 mm，因为曝光时间估计可以主动补偿环境中的光照变化。另一方面，没有参考块更新的平均精度比默认值降低 44 mm，因为参考块更新策略有效地选择了更高分辨率的块并避免选择外点块。最后，法向细化将平均精度提高了 1 mm，且精度提升在所有序列中不一致。有限的提升主要是因为法向量细化仅在具有良好图像观测的简单结构化场景中产生正向优化。在 NTU-VIRAL 数据集中，"eee"和"nya"序列的图像极其昏暗模糊，负优化特别严重。

### IX-C. LiDAR 退化与视觉挑战性环境

在本实验中，我们评估系统在经历 LiDAR 退化和/或视觉挑战的环境下的鲁棒性，在 8 个序列中与 FAST-LIVO 和 R3LIVE 的定性建图结果进行比较，如图 9 和 10 所示。

图 9 展示了 LiDAR 退化序列，其中 LiDAR 面向一面大墙同时从一侧向另一侧沿墙移动。由于仅观察到一个墙平面而缺乏几何约束，LIO 方法会失败。值得一提的是，"HIT Graffiti Wall"序列跨越近 800 米，LiDAR 持续面向墙壁，导致相当大的退化。在所有序列中，FAST-LIVO2 明显展示了其即使在长期退化下的鲁棒性，以及提供高精度彩色点云地图的能力。相比之下，FAST-LIVO 设法获得了几何结构但纹理完全模糊。R3LIVE 在几何结构和纹理清晰度方面都表现挣扎。

图 10 展示了在更复杂场景中的测试，其中 LiDAR 和/或相机偶尔都会退化。退化方向由相应的箭头指示。"HKU Cultural Center"（图 10(a)）展示了 FAST-LIVO2、R3LIVE 和 FAST-LIVO 的建图结果。可以看出，R3LIVE 和 FAST-LIVO 的点云地图扭曲、纹理模糊且漂移超过 1 m。相比之下，FAST-LIVO2 成功返回起始点，实现了令人印象深刻的小于 0.01 m 的端到端误差，同时获得了具有清晰纹理的一致点云地图。"CBD Building 03"（图 10(b)）和"Mining Tunnel"（图 10(c)）仅展示了 FAST-LIVO2 的结果，因为 R3LIVE 和 FAST-LIVO 失败了。在图 10(b)中，蓝色箭头表示朝向纯黑屏的移动，指示同时发生 LiDAR 和相机退化。在图 10(c1)和(c2)中，红点表示该位置的 LiDAR 扫描，说明由于观察到单一平面导致的 LiDAR 退化区域。此外，"Mining Tunnel"在整个序列中表现出非常昏暗的光照，伴随着频繁的视觉和 LiDAR 退化。尽管有这些挑战，FAST-LIVO2 在两个序列中仍以小于 0.01 m 的端到端误差返回起始点。

### IX-D. 高精度建图

在本实验中，我们验证系统的高精度建图能力。为探索不同算法的建图精度并确保公平性，我们在以丰富纹理和结构化环境为特征的场景中将我们的系统与 FAST-LIO2、R3LIVE 和 FAST-LIVO 进行比较。我们以"SYSU 01"、"HKU Landmark"和"CBD Building 01"序列为例。

补充材料图 S9 展示了这些序列实时重建的彩色点云地图。我们可以清楚地观察到，FAST-LIVO2 生成的点云地图在所有系统中保留了最精细的细节，彩色点云地图的放大视图类似于实际 RGB 图像中的视图。在"SYSU 01"序列中，我们的算法在招牌上产生了更少的白色噪点，因为我们在着色之前使用恢复的曝光时间将图像颜色归一化到合理的曝光时间，从而产生很少过曝的彩色点云地图。"CBD Building 01"中人和摩托车的重建也例证了我们重建非结构化物体细节的能力。在所有序列中，估计的最终位置以小于 0.01 m 的端到端误差返回起始点。

### IX-E. 运行时间分析

在本节中，我们评估所提系统每次 LiDAR 扫描和图像帧的平均计算时间，在配备 Intel i7-10700K CPU 和 32 GB RAM 的台式 PC 上测试。我们的评估涵盖公开数据集（包括 Hilti'22、Hilti'23 和 NTU-VIRAL）和我们的私有数据集。

如表 III 所示，我们的系统在所有序列中表现出最低的处理时间。在 Intel i7 处理器上的平均计算时间消耗仅为 **30.03 ms**（每次 LiDAR 扫描 17.13 ms，每图像帧 12.90 ms），满足 10 Hz 的实时运行。此外，我们的系统甚至可以在 ARM 处理器上实时运行，每帧平均处理时间仅为 78.44 ms。

LVI-SAM 的 LIO 和 VIO 中的 LiDAR 和视觉特征提取模块耗时。除了 LIO 和 VIO 消耗的时间外，LVI-SAM 在因子图中集成了 IMU 预积分约束、视觉里程计约束和 LiDAR 里程计约束，进一步增加了整体处理时间。

对于 R3LIVE，虽然也采用直接法，但其像素级图像对齐需要使用大量视觉地图点。相比之下，我们的方法使用带有参考块的稀疏点，实现高效对齐。此外，R3LIVE 维护一个经过贝叶斯更新的彩色地图，随着地图分辨率增加显著增加计算负载。

对于 FAST-LIO2，每帧的平均处理时间（补充材料表 S3）比 FAST-LIVO2 少约 10.35 ms，因为不处理额外的图像测量。

FAST-LIVO2 相比前身 FAST-LIVO 也表现出显著提升。主要增强源于我们在稀疏图像对齐中应用了逆组合公式。采用基于 LiDAR 点平面先验的仿射变换进一步提高了我们方法的收敛效率。因此，FAST-LIVO2 将每个金字塔层级的迭代次数从 10 减少到 3，同时仍实现了更高的精度。

---

## X. 应用

为展示 FAST-LIVO2 在真实世界应用中的卓越性能和多功能性，我们开发了多种解决方案，包括全机载自主无人机导航、机载建图、带纹理的网格生成，以及用于 3D 场景表示的 3D 高斯溅射重建。

### X-A. 全机载自主无人机导航

鉴于 FAST-LIVO2 的高精度和鲁棒定位性能以及其实时能力，我们进行了闭环自主无人机飞行。

#### X-A1. 系统配置

硬件和软件设置如图 11 所示。在硬件方面，我们使用 NUC（Intel i7-1360P CPU 和 32 GB RAM）作为机载计算机。在软件方面，定位组件由 FAST-LIVO2 提供支持，以 10 Hz 提供位置反馈。定位结果被馈送到飞行控制器以实现 200 Hz 的位置、速度和姿态反馈。除了定位，FAST-LIVO2 还向规划模块 Bubble planner[55]提供稠密配准点云，该模块规划平滑轨迹，然后由流形模型预测控制（MPC）[56]跟踪。MPC 计算所需的角速度和推力，由飞行控制器上运行的相应低级角速度控制器跟踪。重要的是，MPC、规划器和 FAST-LIVO2 全部在机载计算机上实时运行。

#### X-A2. 无人机自主导航

我们进行了 4 次全机载自主无人机导航实验："Basement"、"Woods"、"Narrow Opening"和"SYSU Campus"（补充材料表 S2）。"Basement"和"Woods"实验是包含所有规划、MPC 和 FAST-LIVO2 模块的完全自主飞行，而"Narrow Opening"和"SYSU Campus"是仅使用 MPC 和 FAST-LIVO2（无规划组件）的手动飞行。

可以看出，"Basement"和"Woods"展示了无人机成功的自主导航和避障。在"Narrow Opening"中，无人机被命令贴近墙壁飞行，导致 LiDAR 点测量很少。尽管如此，射线投射模块召回了更多视觉地图点，为定位提供了充足约束，从而实现稳定定位。此外，"Basement"和"Narrow Opening"经历了 LiDAR 退化，仅观察到单一墙壁（见图(e1)和(e4)、图12(b1-b4)），伴随着显著的曝光变化（见图(e5-e6)）。尽管有这些挑战，我们的无人机系统表现异常出色。

"Woods"涉及无人机以高达 3 m/s 的高速移动，要求整个无人机系统快速响应（见图12(a1-a4)）。"SYSU Campus"是一个非退化场景，主要展示机载高精度建图能力（见补充材料图 S14）。

最后，值得一提的是，在所有这四次无人机飞行中都发生了剧烈的光照变化。FAST-LIVO2 能够估计紧跟真值的曝光时间（见补充材料图 S15）。

关于机载计算时间，需要在机载计算机上运行 MPC（100 Hz）和规划（10 Hz）消耗了计算资源和内存，限制了 FAST-LIVO2 可用的计算资源。尽管控制和规划并发执行，如图 13 所示，FAST-LIVO2 每次 LiDAR 扫描和图像帧的平均机载处理时间约为 53.47 ms，仍远低于帧周期 100 ms。规划和 MPC 的平均处理时间分别为 8.43 ms 和 18.5 ms。总平均处理时间 80.4 ms 很好地满足了机载操作的实时要求。

### X-B. 机载建图

机载建图是测绘应用中的一项关键任务。为评估 FAST-LIVO2 对此应用的适用性，我们使用公开数据集 MARS-LVIG[52]（其硬件配置详见第 VIII-A 节）进行了一次机载建图实验。我们评估了两个序列"HKairport01"和"HKisland01"，其实时建图结果如图(a-c)所示，其中(a)和(c)对应"HKisland01"，(b)描绘"HKairport01"。

结果证明了 FAST-LIVO2 在森林和岛屿等非结构化环境中的有效性。系统成功捕捉了许多精细结构和锐利的着色效果，包括建筑物、道路上的车道标记、路缘、树冠和岩石，所有这些都清晰可见。这些序列的 APE（RMSE）对于 FAST-LIVO2 分别为 0.64 m 和 0.27 m，相比之下 R3LIVE 为 2.76 m 和 0.52 m。在台式 PC（第 IX-A 节）上的平均处理时间分别约为 25.2 ms 和 21.8 ms，相比之下 R3LIVE 为 110.5 ms 和 100.2 ms。

### X-C. 支持 3D 场景应用：网格生成、纹理映射和高斯溅射

利用从 FAST-LIVO2 获得的高精度传感器定位和稠密 3D 彩色点云地图，我们开发了用于渲染管线的软件应用，包括网格化和纹理映射，以及新兴的类 NeRF 渲染管线如 3D 高斯溅射（3DGS）。

对于网格化，我们在"CBD Building 01"中使用基于截断符号距离函数（TSDF）的 VDBFusion[57]，如图 14(a)所示。柱子上的锐利边缘和屋顶的独特结构清晰可见，展示了网格的高质量。这种细节水平的实现归功于 FAST-LIVO2 点云的高密度和结构重建的卓越精度。

网格构建后，我们使用 OpenMVS[58]在"CBD Building 01"和"Retail Street"中使用估计的相机位姿执行纹理映射，如图 14(b-c)所示。在图 14(c1-c2)中，应用在三角面片上的纹理图像无缝且精确对齐，产生了高度清晰精确的纹理映射。这归功于 FAST-LIVO2 实现的像素级图像对齐。

来自 FAST-LIVO2 的稠密彩色点云也可以直接作为 3DGS 的输入。我们在"CBD Building 01"序列上进行了测试，使用了总共 1180 张图像中的 300 帧。结果如图 15 所示。与 COLMAP[59]相比，我们的方法将获得稠密点云和位姿所需的时间从 9 小时显著减少到 21 秒。然而，训练时间从 10 分 59 秒增加到 15 分 30 秒。这种增加归因于更稠密的点云（下采样到 5 cm），引入了更多需要优化的参数。尽管如此，我们点云增加的密度和精度导致与从 COLMAP 输入获得的 PSNR 相比，峰值信噪比（PSNR）略高。

---

## XI. 结论与未来工作

本文提出了 FAST-LIVO2，一种直接 LIVO 框架，实现快速、精确和鲁棒的状态估计，同时实时重建地图。FAST-LIVO2 可以实现高定位精度，同时对严重的 LiDAR 和/或视觉退化具有鲁棒性。

速度的提升归因于在高效的 ESIKF 顺序更新框架中使用原始 LiDAR、惯性和相机测量。在图像更新中，进一步采用逆组合公式以及基于稀疏块的图像对齐来提升效率。

精度的提升归因于使用（甚至细化）来自 LiDAR 点的平面先验以增强图像对齐精度。此外，使用单一统一体素地图同时管理地图点和观测到的高分辨率图像测量。体素地图结构支持几何构建与更新、视觉地图点生成与更新以及参考块更新，已得到开发和验证。

鲁棒性的提升归因于实时估计曝光时间（有效处理环境光照变化）和按需体素射线投射（应对 LiDAR 的近距盲区）。

FAST-LIVO2 的效率和精度在大量公开数据集上进行了评估，而每个系统模块的鲁棒性和有效性在私有数据集上进行了评估。FAST-LIVO2 在真实世界机器人应用中的应用也得到了展示。

作为一种里程计，FAST-LIVO2 在长距离上可能存在漂移。未来，我们可以将回环检测和滑动窗口优化集成到 FAST-LIVO2 中以减轻这种长期漂移。此外，精确稠密的彩色点云地图可用于提取语义信息以进行物体级语义建图。

---

## 参考文献

[1] R. Mur-Artal and J. D. Tardós, "Orb-slam2: An open-source slam system for monocular, stereo, and rgb-d cameras," IEEE transactions on robotics, vol. 33, no. 5, pp. 1255–1262, 2017.
[2] J. Engel, V. Koltun, and D. Cremers, "Direct sparse odometry," IEEE transactions on pattern analysis and machine intelligence, vol. 40, no. 3, pp. 611–625, 2017.
[3] J. Engel, T. Schöps, and D. Cremers, "Lsd-slam: Large-scale direct monocular slam," in European conference on computer vision. Springer, 2014, pp. 834–849.
[4] C. Forster, Z. Zhang, M. Gassner, M. Werlberger, and D. Scaramuzza, "Svo: Semidirect visual odometry for monocular and multicamera systems," IEEE Transactions on Robotics, vol. 33, no. 2, pp. 249–265, 2016.
[5] J. Zhang and S. Singh, "Loam: Lidar odometry and mapping in real-time." in Robotics: Science and Systems, vol. 2, no. 9, 2014.
[6] J. Lin and F. Zhang, "Loam livox: A fast, robust, high-precision lidar odometry and mapping package for lidars of small fov," in 2020 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2020, pp. 3126–3131.
[7] T. Shan and B. Englot, "Lego-loam: Lightweight and ground-optimized lidar odometry and mapping on variable terrain," in 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018, pp. 4758–4765.
[8] C. Zheng, Q. Zhu, W. Xu, X. Liu, Q. Guo, and F. Zhang, "Fast-livo: Fast and tightly-coupled sparse-direct lidar-inertial-visual odometry," in 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2022, pp. 4003–4009.
[9] T. Qin, P. Li, and S. Shen, "Vins-mono: A robust and versatile monocular visual-inertial state estimator," IEEE Transactions on Robotics, vol. 34, no. 4, pp. 1004–1020, 2018.
[10] R. Mur-Artal, J. M. M. Montiel, and J. D. Tardos, "Orb-slam: a versatile and accurate monocular slam system," IEEE transactions on robotics, vol. 31, no. 5, pp. 1147–1163, 2015.
[11] M. Irani and P. Anandan, "All about direct methods," in Proc. Workshop Vis. Algorithms, Theory Pract, 1999, pp. 267–277.
[12] C. Forster, M. Pizzoli, and D. Scaramuzza, "Svo: Fast semi-direct monocular visual odometry," in 2014 IEEE international conference on robotics and automation (ICRA). IEEE, 2014, pp. 15–22.
[13] W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang, "Fast-lio2: Fast direct lidar-inertial odometry," IEEE Transactions on Robotics, pp. 1–21, 2022.
[14] C. Yuan, W. Xu, X. Liu, X. Hong, and F. Zhang, "Efficient and probabilistic adaptive voxel mapping for accurate online lidar odometry," IEEE Robotics and Automation Letters, vol. 7, no. 3, pp. 8518–8525, 2022.
[15]-[59] （其余参考文献略，共59篇，详见原文）

---

## 补充材料

### I. 系统模块验证

本节验证我们系统的关键模块，包括仿射变换、法向细化、参考块更新、按需射线投射查询、曝光时间估计和 ESIKF 顺序更新，利用 FAST-LIVO2 私有数据集和 MARS-LVIG 数据集。

#### A. 仿射变换评估

在本实验中，我们旨在全面评估基于恒定深度假设（半稠密方法中常用的技术）、来自点云的平面先验以及我们所提系统的细化平面法向（分别表示为"恒定深度"、"平面先验"和"平面法向细化"）的各种仿射变换效果。为此，我们在"CBD Building 02"和"Office Building Wall"上比较了三种方法的建图结果和漂移指标。

如图 S2 所示，"平面法向细化"提供了最清晰精确的建图结果，其次是"平面先验"。值得注意的是，"平面法向细化"以卓越的清晰度渲染了地面和墙壁上的文字和图案以及车道标记。此外，"平面先验"和"平面法向细化"的漂移保持在 0.01 m 以下，而"恒定深度"未能返回起始位置，经历了 0.22 m 的漂移。这些结果证实了基于平面先验的仿射变换的增强性能以及平面法向细化的提升作用。

此外，我们在"CBD Building 02"和"Office Building Wall"序列上比较了基于"恒定深度"和"平面先验"的扭曲投影效果。我们从这两个序列中随机选择几个图像帧进行定性分析。对于每个帧，我们将附加到帧中可见视觉地图点的参考块投影到当前帧的空白图像上。此过程产生一张新的 RGB 图像。如果仿射变换和位姿估计都执行良好，具有块投影的区域将产生无缝且最小扭曲的外观，与原始 RGB 图像非常相似。扭曲块的比较如图 S1 所示。结果表明，"平面先验"下的位姿精度和扭曲性能显著优于"恒定深度"下的性能。

#### B. 参考块更新与法向收敛评估

在本实验中，我们在"HIT Graffiti Wall"和"HKU Centennial Garden"上验证参考块更新策略和法向收敛的效果。如图 S3 所示，(a)和(b)是这两个序列的重建点云。在右侧，对于从 A 到 H 的每个区域，我们分别展示了在不同位姿下捕获的五个块观测，每个块大小为 $40 \times 40$ 像素用于可视化。这些块在左侧对应编号的相机帧中被观测到。可以看出，我们的参考路径更新策略倾向于选择沿平面法向面向平面的高分辨率参考块。还注意到，尽管视觉地图点和块在非平面位置（例如树叶、树干和灯柱）生成，整体建图质量仍然很高。

我们还评估了我们提出的法向估计在 A 到 H 区域块上的收敛性。每个块的大小为 $11 \times 11$ 像素。初始法向量从 LiDAR 点估计。收敛曲线表示每次迭代时初始和优化法向量之间的角度变化，如图 S3 所示。区域 A、C、D 和 E 是结构化区域，而区域 B、F、G 和 H 是非结构化区域。可以观察到，结构化区域的法向量收敛更快（6 次迭代内）且法向细化较小（2 到 4 度），因为点云提供的初始法向相对准确。在非结构化区域，如灌木和树叶（即 B 和 F），法向细化显著（高达 9 度）且需要 9 次迭代才能收敛。总体而言，这 8 个区域的法向细化展示了良好的收敛特性。

#### C. 按需射线投射评估

在本实验中，我们在当前和近期 LiDAR 扫描由于 LiDAR 近距盲区而具有很少甚至没有点的极端条件下评估按需射线投射模块的性能。我们使用"Narrow Corridor"序列进行深入分析，如图 S4 所示。在该序列中，我们穿越一个约 1.9 m 宽的极其狭窄的隧道，并转向面向一侧的弱纹理墙壁。由于面向墙壁时 LiDAR 扫描中的点有限，我们只能通过体素查询获得很少的视觉地图点（图 S4(b)中的黄点）。在这种情况下，射线投射提供了足够的视觉约束以减轻退化（图 S4(b)中的蓝点）。可视化结果表明，按需射线投射模块在 LiDAR 扫描中点很少的挑战性条件下工作良好。

#### D. 曝光时间估计评估

在本实验中，我们分两部分验证曝光时间估计模块：
1. 对于具有固定曝光和增益的序列，我们将接收的原始图像的每个像素乘以随时间正弦变化的曝光因子。我们通过将估计的曝光时间与应用的正弦函数进行比较来验证估计的有效性。
2. 对于具有自动曝光以及固定或自动增益设置的序列，我们通过将估计的曝光时间与从相机 API 检索的真值进行比较来评估其精度。

在第一部分中，我们在"Retail Street"序列上进行测试，对具有固定曝光和增益的图像应用曝光因子。如图 S5 所示，估计的相对逆曝光时间与真实值非常吻合，证明了我们的曝光估计在合成条件下的收敛性。

在第二部分中，我们使用"HKU Centennial Garden"、"HKU Cultural Garden"和"HKU Main Building"序列进行测试，这些序列具有显著的曝光时间变化。我们通过第一帧缩放估计的相对曝光时间以恢复每帧的实际曝光时间（ms）。如图 S5 所示，估计的曝光时间紧跟真值，验证了我们曝光时间估计模块的有效性。偶尔的不匹配可能是由于未建模的响应函数和渐晕因子[38]造成的。

#### E. ESIKF 顺序更新评估

在本实验中，我们评估 LiDAR 和相机状态的不同 ESIKF 更新策略。我们比较异步与同步更新，以及标准与顺序更新。具体而言，我们评估三种策略：
- "异步（标准更新）"：相机和 LiDAR 状态在各自的采样时间更新，不进行扫描重组；
- "同步（标准更新）"：LiDAR 扫描被重组以与相机图像同步，状态在标准 ESIKF 内同时使用 LiDAR 和相机测量更新；
- "同步（顺序更新）"：LiDAR 和相机同步，但状态先由 LiDAR 测量更新，然后由相机测量更新。

这些策略使用 MARS-LVIG 数据集的"AMvalley03"序列在精度、鲁棒性和效率方面进行评估。我们选择此序列有几个关键原因：(1) 该序列包含导致 LiDAR 和视觉退化的斜坡，使其成为具有挑战性的测试用例；(2) 该序列代表极其大规模的场景（约 $901 \text{ m} \times 500 \text{ m} \times 130 \text{ m}$），具有长期高速数据采集（以 12 m/s 的速度覆盖 600 s），其中位姿偏差容易发生（由于长期和高速条件），甚至轻微的漂移也会导致彩色点云中的显著模糊（由于大规模），从而产生更明显的比较结果；(3) 该序列提供 RTK 真值数据，允许更精确的定量比较。

我们比较了三种更新策略的定性建图结果、定量 APE 和平均处理时间。"AMvalley03"序列的 APE（RMSE）指标对于"异步（标准更新）"、"同步（标准更新）"和"同步（顺序更新）"分别为 3.12 m、2.45 m 和 0.68 m。在台式 PC 上的平均处理时间分别约为 27.6 ms、49.9 ms 和 23.1 ms。我们提出的"同步（顺序更新）"实现了最高的效率和精度，而"异步（标准更新）"精度最低。"同步（标准更新）"最耗时，主要因为它需要在图像金字塔的每一层融合所有 LiDAR 测量。

总体而言，我们提出的"同步（顺序更新）"提供了卓越的精度和效率，而"异步（标准更新）"精度最低，"同步（标准更新）"最耗时。

---

> **翻译说明**：本文档为 FAST-LIVO2 论文（arXiv:2408.14035v2）的完整中文翻译。公式保留原文 LaTeX 格式，专业术语首次出现时附英文原文。图表编号与原文一致。参考文献[15]-[59]因篇幅省略，详见原文。翻译仅供学习参考，如有歧义以原文为准。

---

## 附录：代码实现说明（论文 vs 开源代码差异对照）

> 本附录基于官方仓库 https://github.com/hku-mars/FAST-LIVO2 （commit 时的 main 分支）逐文件核对，标注论文描述与开源代码实现/配置之间的差异。所有差异均不影响翻译的准确性——翻译忠实于论文原文，此处仅为工程实现层面的补充说明。

### 一、核心源文件与代码量

| 文件 | 行数 | 核心职责 |
|------|------|---------|
| `src/LIVMapper.cpp` | 1370 | 主调度：传感器回调、IMU传播、LIO/VIO分支切换、建图发布 |
| `src/vio.cpp` | 1875 | 视觉测量核心：稀疏点检索、光度误差、ESIKF更新、参考块管理、法向细化 |
| `src/voxel_map.cpp` | 970 | 体素地图：八叉树构建、平面拟合、LiDAR点到平面残差、ESIKF更新 |
| `src/IMU_Processing.cpp` | 587 | IMU预积分、去畸变、扫描重组 |
| `src/preprocess.cpp` | 1125 | LiDAR点云预处理、去畸变、时间戳对齐 |
| `include/common_lib.h` | 243 | 状态向量定义（DIM_STATE=19）、StatesGroup、点结构体 |
| `include/voxel_map.h` | 258 | 体素八叉树、VoxelPlane、VoxelMapManager类声明 |
| `include/vio.h` | 186 | VIOManager类声明、SubSparseMap、Warp结构体 |
| `include/visual_point.h` | 48 | VisualPoint类：3D点位置、法向、观测块列表 |

### 二、论文与代码差异对照表

| 序号 | 论文章节/公式 | 论文描述 | 代码实现/配置 | 差异类型 |
|------|-------------|---------|-------------|---------|
| 1 | IX-A 实现细节 | 八叉树最大层数为 **3** | `config/avia.yaml`: `max_layer: 2` | 参数值差异 |
| 2 | VII-B 公式(23) | 采用**逆组合公式**提升效率 | `avia.yaml`: `inverse_composition_en: false`（默认走正向组合`updateState()`） | 功能已实现但默认关闭 |
| 3 | VII-A2 按需射线投射 | 详细描述射线投射模块 | `avia.yaml`: `raycast_en: false` | 功能已实现但默认关闭 |
| 4 | VI-B 公式(20) | 考虑**光束发散角**的测距误差修正 δd = d·(cosφ/cos(θ+φ) − cosφ/cos(θ−φ)) | `voxel_map.cpp` `calcBodyCov()` 仅实现基础测距+方位角噪声，**未实现**公式(20)的入射角修正 | 论文提出但代码未实现 |
| 5 | VII-A3 外点剔除 | 剔除视角超过 **80°** 的参考块/当前块 | `vio.cpp` 视角检查代码被注释：`// if (dir.dot(norm_vec) <= 0.17) continue;` | 功能已实现但被注释 |

### 三、差异项详细说明

#### 差异1：八叉树最大层数

- **论文原文**（IX-A）："the maximum number of layers of the inner octree is 3"
- **代码配置**（`config/avia.yaml` 第57行）：`max_layer: 2`
- **代码逻辑**（`voxel_map.cpp` `cut_octo_tree()`）：`if (layer_ >= max_layer_) { octo_state_ = 0; return; }`，达到最大层数后不再细分
- **说明**：不同传感器配置文件可能不同。`avia.yaml`（Livox Avia）设为2，`HILTI22.yaml`、`MARS_LVIG.yaml`、`NTU_VIRAL.yaml` 需单独检查。论文给出的是通用默认值，实际部署时根据传感器点密度调整。

#### 差异2：逆组合公式默认关闭

- **论文原文**（VII-B）："we adopt the inverse compositional formulation, where the pose increment δT parameterizing ... is moved from u_i to u'_i"（公式23）
- **代码配置**（`avia.yaml` 第44行）：`inverse_composition_en: false`
- **代码实现**：
  - `vio.cpp` `updateStateInverse()`（约第12000-12500 token处）：完整实现逆组合——`precomputeReferencePatches()` 预计算参考块雅可比存入 `H_sub_inv`，每次迭代只计算残差 `z`，不重新计算图像梯度雅可比
  - `vio.cpp` `updateState()`：正向组合——每次迭代都重新计算 `Jimg`（图像梯度）、`Jdpi`（投影雅可比）、链式法则雅可比
- **调用逻辑**（`computeJacobianAndUpdateEKF`）：
  ```cpp
  if (inverse_composition_en) {
      has_ref_patch_cache = false;
      updateStateInverse(img, level);
  } else
      updateState(img, level);
  ```
- **说明**：逆组合理论上更快（雅可比只算一次），但实际测试中正向组合可能因每次重新计算梯度而对光照变化更鲁棒。代码保留了两种实现供切换。

#### 差异3：射线投射默认关闭

- **论文原文**（VII-A2）：详细描述了对未被视觉地图点占据的图像网格单元，沿中心像素向后投射射线，采样点从 d_min 到 d_max 均匀分布
- **代码配置**（`avia.yaml` 第43行）：`raycast_en: false`
- **代码实现**：
  - `vio.cpp` `initializeVIO()`：预计算每个网格的射线采样点，`d_min=0.1, d_max=3.0, step=0.2`（即每条射线15个采样点）
  - `vio.cpp` `retrieveFromVisualSparseMap()`：对 `grid_num[i] != TYPE_MAP` 的网格执行射线投射，命中体素后提取视觉地图点
- **说明**：射线投射主要用于 LiDAR 近距盲区（物体太近LiDAR无返回点）的场景。一般环境下体素查询已能获取足够视觉点，故默认关闭以节省计算。在狭窄走廊、近距离贴墙等场景建议开启。

#### 差异4：光束发散角噪声公式(20)未实现

- **论文原文**（VI-B 公式20）：
  $$\delta d = L_2 - L_1 = d \left( \frac{\cos\varphi}{\cos(\theta+\varphi)} - \frac{\cos\varphi}{\cos(\theta-\varphi)} \right)$$
  其中 θ 为激光光束发散角，φ 为方位方向与平面法向的夹角。该修正使大入射角下的测距不确定性增大。
- **代码实现**（`voxel_map.cpp` `calcBodyCov()`）：
  ```cpp
  float range = sqrt(pb[0]^2 + pb[1]^2 + pb[2]^2);
  float range_var = range_inc * range_inc;  // 仅TOF测距噪声
  direction_var << sin(DEG2RAD(degree_inc))^2, 0, 0, sin(DEG2RAD(degree_inc))^2;  // 仅方位角噪声
  // 没有 cosφ/cos(θ±φ) 的发散角修正
  ```
- **配置参数**：`beam_err_`（默认0.05度）在代码中用作方位角误差 `degree_inc`，而非论文中的光束发散角 θ
- **说明**：这是论文提出但开源代码未落地的算法。论文实验中可能通过增大 `dept_err_`（测距误差）经验性补偿了大入射角效应，或该修正在内部版本中实现但未开源。

#### 差异5：视角外点剔除被注释

- **论文原文**（VII-A3）："we remove points whose viewing angle of the reference patch or current patch is too large (e.g. over 80°)"
- **代码实现**（`vio.cpp` `retrieveFromVisualSparseMap`）：
  ```cpp
  // if (dir.dot(norm_vec) <= 0.17) continue; // 0.34 70度 0.17 80度 0.08 85度
  ```
  该检查在参考块选择和当前块选择两处均被注释
- **说明**：注释原因可能是法向初始化（来自LiDAR点云平面拟合）在非结构化区域不准确，导致误删有效观测点。代码中保留了注释和阈值参数，方便调试时开启。

### 四、代码中论文未明确提及的关键参数

| 参数 | 论文 | 代码默认值（avia.yaml） | 代码位置 |
|------|------|----------------------|---------|
| 图像金字塔层数 | 未明确 | `patch_pyrimid_level: 4` | `vio.cpp` computeJacobianAndUpdateEKF 从 level=3 到 0 |
| Patch大小 | 8×8（对齐）/ 11×11（细化） | `patch_size: 8` | `vio.cpp` patch_size_total=64 |
| 最大迭代次数 | 未明确 | `max_iterations: 5` | LIO和VIO均为5次 |
| 光度外点阈值 | 未明确 | `outlier_threshold: 1000` | 实际判断 error > 1000×64，即每像素平均>15.6灰度 |
| 图像网格大小 | 30×30像素 | `grid_size: 5, grid_n_height: 17` | 实际 grid_size=height/17≈28像素 |
| 深度连续性阈值 | 未明确 | 0.5米 | `vio.cpp`: delta_dist > 0.5 判定为遮挡/不连续 |
| 视觉点协方差 | 未明确 | `img_point_cov: 100` | ESIKF中视觉测量噪声 R=img_point_cov |
| 曝光时间协方差 | 未明确 | `inv_expo_cov: 0.1` | IMU传播中曝光时间随机游走噪声 |
| LiDAR下采样率 | 1:3 | `point_filter_num: 1`（avia） | preprocess.cpp中每N个点取1个 |

### 五、核心算法函数级对应索引

#### ESIKF 状态传播与更新
| 论文 | 代码函数 | 文件:行号范围 |
|------|---------|-------------|
| IV-C 前向传播 | `ImuProcess::Process2()` | `IMU_Processing.cpp` |
| IV-C 后向传播（去畸变） | `ImuProcess::Process2()` 内 undistort 逻辑 | `IMU_Processing.cpp` |
| IV-D LiDAR更新 | `VoxelMapManager::StateEstimation()` | `voxel_map.cpp` 约3000-4000 token |
| IV-D 视觉更新 | `VIOManager::computeJacobianAndUpdateEKF()` → `updateState()`/`updateStateInverse()` | `vio.cpp` |
| IV-D 顺序更新调度 | `LIVMapper::stateEstimationAndMapping()` → `handleLIO()` 先执行 → `handleVIO()` 后执行 | `LIVMapper.cpp` |

#### 视觉测量模型
| 论文 | 代码函数 | 文件:行号范围 |
|------|---------|-------------|
| VII-A1 可见体素查询 | `VIOManager::retrieveFromVisualSparseMap()` 前半段 | `vio.cpp` 约4000-5000 token |
| VII-A2 射线投射 | `VIOManager::retrieveFromVisualSparseMap()` 中 `if (raycast_en)` 块 | `vio.cpp` |
| VII-A3 外点剔除 | `retrieveFromVisualSparseMap()` 中深度连续性+光度误差阈值检查 | `vio.cpp` |
| VII-B 光度残差(21) | `retrieveFromVisualSparseMap()` 中 `error += (ref_inv*warp - cur_inv*patch)^2` | `vio.cpp` |
| VII-B 逆组合(23) | `VIOManager::updateStateInverse()` + `precomputeReferencePatches()` | `vio.cpp` 约12000-13000 token |
| V-C 视觉点生成 | `VIOManager::generateVisualMapPoints()` | `vio.cpp` 约8000-9000 token |
| V-D 参考块更新 | `VIOManager::updateReferencePatch()` | `vio.cpp` 约8000-9000 token |
| V-D 法向细化 | `updateReferencePatch()` 中从体素地图获取平面法向并更新 | `vio.cpp` |
| VII-B 仿射变换(13) | `VIOManager::getWarpMatrixAffineHomography()`（平面先验）/ `getWarpMatrixAffine()`（恒定深度） | `vio.cpp` 约2000-3000 token |
| VII-B 块扭曲采样 | `VIOManager::warpAffine()` | `vio.cpp` |

#### LiDAR测量模型
| 论文 | 代码函数 | 文件:行号范围 |
|------|---------|-------------|
| VI-A 点到平面残差(17-19) | `VoxelMapManager::StateEstimation()` + `BuildResidualListOMP()` + `build_single_residual()` | `voxel_map.cpp` |
| VI-B 光束发散角(20) | **未实现**（见差异4） | — |
| V-A 体素地图构建 | `VoxelMapManager::BuildVoxelMap()` + `VoxelOctoTree::init_octo_tree()` + `init_plane()` | `voxel_map.cpp` |
| V-A 平面协方差 | `VoxelOctoTree::init_plane()` 中特征值分解+雅可比传播 | `voxel_map.cpp` 约0-3000 token |

### 六、线程模型

FAST-LIVO2 为**单线程主循环 + ROS回调**架构，无多线程并行计算（仅OpenMP用于残差构建的点级并行）：

```
main线程 (LIVMapper::run(), 5000Hz轮询)
├── ROS回调 (缓冲区写入)
│   ├── livox_pcl_cbk() / standard_pcl_cbk()  → LiDAR点云缓冲
│   ├── imu_cbk()                             → IMU数据缓冲
│   └── img_cbk()                             → 图像缓冲
├── sync_packages()          → 传感器时间同步，组装MeasureGroup
├── processImu()             → IMU预积分+去畸变+扫描重组
└── stateEstimationAndMapping()
    ├── handleLIO()          → LiDAR ESIKF更新 + 体素地图更新
    └── handleVIO()          → 视觉ESIKF更新 + 视觉点生成/更新
```

- **无独立建图线程**：地图更新（`UpdateVoxelMap`）在LIO分支内同步执行
- **无回环检测线程**：论文XI节明确说未来工作加入回环
- **OpenMP并行**：`BuildResidualListOMP()` 使用 `#pragma omp parallel for` 并行构建每个LiDAR点的平面残差

---

> **附录说明**：本附录基于 2024 年 8 月 arXiv:2408.14035v2 版本论文与对应官方开源代码核对。代码后续更新可能改变上述配置和实现，以最新仓库为准。



