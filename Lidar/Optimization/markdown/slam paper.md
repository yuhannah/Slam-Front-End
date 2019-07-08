# slam paper

## 1990+

### 1992

#### A Method for Registration of 3D Shapes

> The ICP algorithm always converges monotonically to the nearest local minimum of a mean-square distance metric, and experience shows that the rate of convergence is rapid during the first few iterations.

ICP算法总是单调收敛于==均方距离度量的最近局部最小值==，经验表明，该算法在前几次迭代中收敛速度较快。



> The algorithm requires no extracted features, no curve or surface derivatives, and no preprocessing of 3D data, except for the removal of statistical outliers.

该算法不需要提取特征，不需要曲线或曲面导数，不需要对三维数据进行预处理，只需要去除统计离群值。



> The main application of the proposed method as described here is to register digitized data from unfixtured rigid objects with an idealized geometric model prior to shape inspection.

本文所述方法的主要应用是将未固定刚体的数字化数据匹配为理想几何模型，然后再进行形状检测。



> In this section, methods for computing the closest point to a given point on the various geometric representations listed above are described. First, the basic geometric entities are covered, followed by parametric entities, and, finally, implicit entities.

在这一节中，我们将介绍在上面列出的各种几何表示法上计算离给定点最近的点的方法。首先介绍基本的几何实体，然后是参数实体，最后是隐式实体。



> The computations to compute the distance are not closed form and are relatively involved.

距离的计算不是封闭的，而且是相对复杂的。



> Sets of parametric entities are again straightforward once the distance metric for an individual entity is implemented.

一旦实现了单个实体的距离度量，参数实体集也就很简单了。



> For a parametric space curve C, one can compute a polyline L such that the piecewise-linear approximation never deviates from the space curve by more than a prespecified distance delta. By tagging each point of the polyline with its corresponding u argument values of the parametric curve, one can obtain an estimate of the ua argument value of the closest point from the line segment set.

对于参数化空间曲线C，可以计算折线L，使分段线性逼近与空间曲线的偏差不超过预先指定的距离。将折线上的每个点标记为参数曲线上对应的u个参数值，就可以得到离线段集合最近的点的ua个参数值的估计值。



> All closet point (minimum distance) algorithms have been mentioned in forms that generalize to n dimensions. One more necessary procedure for yielding the least squares rotation and translation is reviewed. For our purposes, the quaternion-based algorithm is preferred over the singular value decomposition (SVD) method in two and three dimensions since reflections are not desired. THe SVD approach, based on the cross-covariance matrix of two point distributions and would be our method of choice for n > 3 in any n-dimensional applications. The basic solution of H is described below, although the method of F is equivalent. Our summary stresses the role of the SVD cross-covariance matrix, which is an important relationship not discussed in other work.

所有的最近点(最小距离)算法都以一般化到n维的形式被提及。介绍了产生最小二乘旋转和平移的另一个必要步骤。对于我们的目的，基于四元数的算法在二维和三维相比奇异值分解(SVD)方法是首选的，因为不需要反射。SVD方法，基于两点分布的互协方差矩阵，是任何n维应用中n >  3的选择方法。H的基本解如下所述，虽然F的方法是等价的。我们的总结强调了SVD交叉协方差矩阵的作用，这是其他工作中没有讨论的一个重要关系。



> A convergence theorem for the ICP algorithm is now stated and proved. The key ideas are that 1) least squares registration generically reduces the average distance between corresponding points during each iteration, whereas 2) the closest point determination generically reduces the distance for each point individually. Of course, this individual distance reduction also reduces the average distance because the average of a set of smaller positive numbers is small. We offer a more elaborate explanation in the proof below.

给出并证明了ICP算法的一个收敛定理。其关键思想是:1)最小二乘配准一般减少每次迭代中对应点之间的平均距离，而2)最接近点的确定一般减少每个点的距离。当然，这个距离的减少也减少了平均距离因为一组较小的正数的平均值是小的。我们在下面的证明中给出了更详细的解释。



> Theorem: The iterative closest point algorithm always converges monotonically to a local minimum with respect to the mean-square distance objective function.

定理:迭代最接近点算法总是单调收敛于关于均方距离目标函数的局部最小值。



> The accelerated ICP algorithm uses a minor variation on the basic line search methods of multivariate unconstrained minimization.

加速ICP算法在多变量无约束最小化的基本直线搜索方法上使用了一个小的变化。

### 1997

#### Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans

It is well understood that odometry is not sufficient because of it leads to ==unbounded position error==.<!--无界误差-->$\color{darkCyan}{<无界误差>}$

To ensure convergence, an initial small rotation and translation must be assumed.<!--初始平移和旋转-->$\color{darkCyan}{<初始平移和旋转>}$

odometry information is often imperfect due to ==wheel slippage==.<!--打滑-->$\color{darkCyan}{<打滑>}$

Due to the existence of random sensing noise and self-occlusion, it may be impossible to align two scans perfectly.<!--噪声和遮挡-->$\color{darkCyan}{<噪声和遮挡>}$

- Rotation Search/Least-Squares Matching Algorithm(==tangent-based== matching method<!--基于切线-->$\color{darkCyan}{<基于切线>}$)

  seven steps of the algorithm:

  1. Project the ==reference scan== $S_{ref}$ to the pose $P_{new}'$ so that the two scans are represented in the same coordinate system. Discard those points on $S_{ref}$ which are likely not to be visible from the new pose.<!--投影参考点云从传感器坐标系到世界坐标系中里程计提供的初始新坐标(统一坐标系)，去除遮挡点-->$\color{darkCyan}{<投影参考点云从传感器坐标系到世界坐标系中里程计提供的初始新坐标(统一坐标系)，去除遮挡点>}$
  2. Compute the tangent directions on each scan by fitting lines to a neighborhood of sample points. Discard unreliable tangent lines, such as at corners or depth discontinuities.<!--两个点云都提取切线，局部拟合，去除角点和不连续点-->$\color{darkCyan}{<两个点云都提取切线，局部拟合，去除角点和不连续点>}$
  3. Decide on a trial value of the rotation $\omega$ from a global search procedure.<!--全局搜索寻找旋转角度-->$\color{darkCyan}{<全局搜索寻找旋转角度>}$
  4. For each point on $S_{new}$ , use the rotation angle $\omega$ to define an approximate
     corresponding point on $S_{ref}$ and compute the point through interpolation.
     Check the correspondence against predefined thresholds in order to reject
     outliers.<!--插值简化寻找对应点的过程-->$\color{darkCyan}{<插值简化寻找对应点的过程>}$
  5. Use all the correspondence pairs to construct a least-squares model for $T$
     and find the least-squares solution. Define a matching distance as a function
     of $\omega$ from the least-squares residual and the outlier penalties.
  6. Update the rigid transformation by the least-squares solution of $T$
  7. Repeat steps 3–6 and find the rotation $\omega$ which minimizes the matching distance function. Also obtain the overall translation by integrating the indi-
     vidual updates.

  <!--先预定义旋转角度，再用最小二乘模型寻找平移值-->$\color{darkCyan}{<先预定义旋转角度，再用最小二乘模型寻找平移值>}$

  <!--迭代循环：旋转->平移->旋转->平移-->$\color{darkCyan}{<迭代循环：旋转->平移->旋转->平移>}$

- Iterative Point Correspondence Matching Algorithm (point to point)

  three steps of the algorithm:

  1. For each point $P_i$ on $S_{new}$ , we use a simple rule (independent of the actual rotation and translation) to determine a corresponding point $P_i$  on $S_{ref}$ .
  2. Then from all the correspondence pairs of points, we compute a least-squares solution of the relative rotation and translation. This solution is applied to reduce the pose error between the two scans. 
  3. We repeat this process until it converges.

  <!--定义点到点的对应关系，用最小二乘解算出旋转和平移，并迭代使收敛-->$\color{darkCyan}{<定义点到点的对应关系，用最小二乘解算出旋转和平移，并迭代使收敛>}$

  two rules of finding correspondence:

  1. the closest-point rule (ICP)<!--找最近的点对-->$\color{darkCyan}{<找最近的点对>}$
     - always converges ==monotonically== to a local minimum with respect to the least-squares distance function<!--单调收敛到局部最小-->$\color{darkCyan}{<单调收敛到局部最小>}$
     - good at solving the translation when the rotation is small<!--更适合角度差别小的匹配-->$\color{darkCyan}{<更适合角度差别小的匹配>}$
     - the convergence speed of the algorithm is always very slow when the distance function approachs a local minimum.<!--收敛速度慢-->$\color{darkCyan}{<收敛速度慢>}$
  2. the matching-range-point rule (IMRP)<!--找离传感器距离近似的点对(移动不能太大)-->$\color{darkCyan}{<找离传感器距离近似的点对(移动不能太大)>}$
     - the correspondence of $P$ under a rotation is a point which has the same polar range as that of $P$ , and the polar angles of the corresponding points differ by the rotation angle $\omega$ .
  3. combine two rules: iterative dual correspondence (IDC)<!--结合两种规则-->
     - take the ==translation== component from the ==closest-point rule== solution and the ==rotation== component from the ==matching-range-point== rule solution to form the current solution for the transformation.
     - ensure the stability of the iterations
     - increases the convergence speed significantly
     - insensitive to the choices of parameter

  the ratio of error residuals ($C=e_{i+1}/e_i$)<!--误差残差-->:

  1. ICP has a sublinear convergence rate
  2. IMRP and IDC have a linear convergence rate

### 1998

#### An Experimental Comparison of Localization Methods

In practice, it is too difficult to determine the joint effect of all sensor and position integration readings; instead, a recursive approximation is assumed:

<!--在实践中，很难确定所有传感器和位置积分读数的联合效应; 相反，假设递归近似：-->
$$
p(l|S^n,A^n)=\alpha \int p(l|s_n,a_n,l')p(l'|S^{n-1},A^{n-1})dl'
$$

- How is the prior distribution to be represented?<!--先前的分配如何表示？-->
- How is the posterior distribution $p(l|s_n,a_n,l')$ to be calculated?<!--如何计算后验分布？-->



Markov localization makes the choice of an explicit, discreet representation for the prior probability, using a grid or topological graph to cover the space of robot poses, and keeping a probability for each element of this space. Scan matching, on the other hand, uses a simple Gaussian for the distribution. Given the divergence in representation, it is interesting that these methods both use the same general technique for calculating the posterior:

<!--马尔可夫定位使得可以选择先验概率的明确，谨慎的表示，使用网格或拓扑图来覆盖机器人姿势的空间，并保持该空间的每个元素的概率。另一方面，扫描匹配使用简单的高斯分布进行分配。鉴于表示的差异，有趣的是这些方法都使用相同的通用技术来计算后验：-->

- _Predict_ the new robot pose $l$ and its associated uncertainty from the previous pose $l'$ , given odometric information.<!--在给定的测距信息的情况下，预测新的机器人姿势l及其与先前姿势l'的相关不确定性。-->
- _Update_ the robot pose (and uncertainty) $l$ using sensor information matched against the map.<!--使用与地图匹配的传感器信息更新机器人姿势（和不确定性）l。-->

The first step generally increases the uncertainty in the robot’s pose, while the second generally reduces it.

<!--第一步通常会增加机器人姿势的不确定性，而第二步通常会降低机器人的姿势。-->

The prediction step is modeled by a conditional probability<!--条件概率-->. 

The possible errors of the odometry (i.e., $p(l|a_n,l')$ ) are modeled as normally distributed<!--里程计误差正态分布-->. Upon robot motion, the pose is calculated as:
$$
p(l) \longleftarrow \int p(l_u|a_n,l')p(l')dl'
$$
In the update step, the new robot pose is calculated according to the Bayes formula:
$$
p(l|s_n)=\alpha p(s_n|l)p(l)
$$

- Markov Localization

  The key idea of Markov localization is to compute a discrete approximation of a probability distribution over all possible poses in the environment. This distribution evolves according to Equations 2 and 3. Different variants of Markov localization have been developed [15, 19, 10, 4] and have been shown in experimental results to have several features:

  <!--马尔可夫定位的关键思想是计算环境中所有可能姿态的概率分布的离散近似。这种分布根据方程式2和3进行演化。已经开发了马尔可夫定位的不同变体[15,19,10,4]，并且已经在实验结果中显示具有以下几个特征：-->
  - They are able to localize the robot when its initial pose is unknown. This property is essential for truly autonomous robots as it no longer requires that the initial pose of the robot is entered whenever it is switched on or gets lost.<!--当初始姿势未知时，他们能够定位机器人。这个属性对于真正自主的机器人来说是必不可少的，因为它不再需要在开启或丢失时输入机器人的初始姿势。-->
  - They are able to deal with noisy sensors such as ultrasonic sensors.<!--他们能够处理噪声传感器，如超声波传感器。-->
  - They are able to represent ambiguities and can be extended to actively resolve ambiguities [5].<!--它们能够表示歧义，可以扩展到积极解决歧义[5]。-->
  - Computationally, the technique is dominated by the dimensionality of the grid, and the size of its cells.<!--在计算上，该技术由网格的维数和其单元的大小决定。-->

  The existing methods can be distinguished according to the type of discretization they rely on. While [15, 19, 10, 20] use a topological discretization of the environment and detect landmarks to localize the robot, the system used in this paper computes a fine-grained grid-based approximation of the distribution [4]. To cope with the huge state space this technique includes several optimizations. In practice, usually only a small area around the robot is updated during localization.

  <!--可以根据它们所依赖的离散化类型来区分现有方法。虽然[15,19,10,20]使用环境的拓扑离散化并检测地标以定位机器人，但本文中使用的系统计算了细粒度的基于网格的分布近似[4]。为了应对巨大的状态空间，这种技术包括几个优化。在实践中，通常在定位期间仅更新机器人周围的小区域。-->

  Map information for Markov localization depends on the type of the state space discretization. The topological approaches [15, 19, 10, 20] use landmarks to detect locations. The fine-grained discretization applied in this paper in contrast uses metric maps of the environment. These can be hand-crafted CAD maps consisting of line segments representing vertical surfaces in the indoor environment, or learned occupancy grid maps [14]. In all approaches the map is used to compute what the sensor readings should be from a given cell in the state space. The closeness of the predicted readings to the actual ones give a measure of $p(s_n|l)$ .

  <!--马尔可夫定位的地图信息取决于状态空间离散化的类型。拓扑方法[15,19,10,20]使用地标来检测位置。相比之下，本文中应用的细粒度离散化使用了环境的度量图。这些可以是手工制作的CAD地图，由代表室内环境中垂直表面的线段或学习占用网格图组成[14]。在所有方法中，映射用于计算传感器读数应该来自状态空间中的给定单元的内容。预测读数与实际读数的接近程度给出了p(s_n|l)的度量。-->

  As already mentioned, an important feature of Markov localization techniques is the ability to globally localize the robot within its environment.

  <!--如前所述，马尔可夫定位技术的一个重要特征是能够在其环境中全局定位机器人。-->

- Scan Matching

  Scan matching is the process of translating and rotating a range scan (obtained from a range device such as a laser range finder) in such a way that a maximum overlap between sensor readings and a priori map emerges. For matching a range scan with a map an initial estimate of the robot pose must be known and is usually derived from odometry information.

  <!--扫描匹配是转换和旋转范围扫描（从诸如激光测距仪的范围设备获得）的过程，使得传感器读数和先验图之间出现最大重叠。为了使范围扫描与地图匹配，必须知道机器人姿势的初始估计并且通常从测距信息导出。-->

  The robot pose and its update from scan matching are modeled as single Gaussian distributions. This has the advantage that robot poses can be calculated with high precision, and that an efficient method for computing the update step can be used, namely, Kalman filtering. Scan matching has the following properties:

  <!--机器人姿势及其从扫描匹配的更新被建模为单高斯分布。这具有可以高精度地计算机器人姿势的优点，并且可以使用用于计算更新步骤的有效方法，即卡尔曼滤波。扫描匹配具有以下属性：-->

  - It can localize the robot precisely given good inputs, and in the linear case it is the optimal estimate of location.<!--它可以精确地定位机器人并给出良好的输入，在线性情况下，它是位置的最佳估计。-->
  - It cannot recover from catastrophic failures caused by bad matches or incorrect error models.<!--它无法从错误匹配或错误模型错误导致的灾难性故障中恢复。-->
  - Because its search is confined to small perturbations of the sensor scans, it is computationally efficient.<!--因为它的搜索仅限于传感器扫描的小扰动，所以它在计算上是有效的。-->



  The extended Kalman filter method has the following form.<!--EKF-->

  - time step $t$:

  ​	robot pose: $l(t)=(x(t),y(t),\theta(t))^T$

  ​	error covariance: $\sum_l(t)$

  - time step $t+1$:

    - robot motion: $a=(\delta,\alpha)^T$

      - robot pose: <!--estimated-->
        $$
        l'(t+1)=F(l(t),a)=\begin{pmatrix}x(t)+\delta \cos(\theta(t)) \\ y(t)+\delta \sin(\theta (t)) \\ \theta(t)+\alpha \end{pmatrix}
        $$

      - error covariance: <!--estimated-->
        $$
        {\sum}'_l(t+1)=\nabla F_l \sum_l(t) \nabla F_l^T + \nabla F_a \sum_a \nabla F_a^T
        $$

    - scan matching: robot pose: $l_s$, error covariance: $\sum_s$

      - robot pose:
        $$
        l(t+1)=\frac{{{\sum}'_l}^{-1}(t+1)l'(t+1)+{\sum_s}^{-1}l_s}{{{\sum}'_l}^{-1}(t+1) + {\sum_s}^{-1}}
        $$

      - error covariance:
        $$
        \sum_l(t+1) = \frac{1}{{{\sum}'_l}^{-1}(t+1) + {\sum_s}^{-1}}
        $$




  These equations demonstrate that Kalman filter based selflocalization can be implemented efficiently. As long as the error models are accurate, Kalman filtering will give a reasonable estimate of the robot pose (in the linear case, it will be an optimal estimate).

  <!--这些方程表明可以有效地实现基于卡尔曼滤波器的自定位。 只要误差模型是准确的，卡尔曼滤波将给出机器人姿态的合理估计（在线性情况下，它将是最佳估计）。-->

  The success of the Kalman filter also depends heavily on the ability of scan matching to correct the robot pose. We use two matching methods, described in [9]. The first approach matches sensor readings against the line segments in a hand-crafted CAD map of the environment [7]. It assigns scan points to line segments based on closest neighborhood and then searches for a translation and rotation that minimizes the total squared distance between scan points and their target lines. For reasons of efficiency we modified the approach to extract the line segments from the CAD model that are visible from the current robot-position and discard the non-visible ones. This greatly reduces the number of line segments for the matching process and also avoids nonsensical assignments, e.g. assignments from a scan point to a line that corresponds to the backside of a wall.

  <!--卡尔曼滤波器的成功在很大程度上还取决于扫描匹配纠正机器人姿势的能力。我们使用两种匹配方法，如[9]中所述。第一种方法将传感器读数与手工制作的环境CAD图中的线段相匹配[7]。它根据最近的邻域将扫描点分配给线段，然后搜索平移和旋转，最小化扫描点与其目标线之间的总平方距离。出于效率的原因，我们修改了从CAD模型中提取从当前机器人位置可见的线段并丢弃不可见的线段的方法。这极大地减少了匹配过程的线段数量，并且还避免了无意义的分配，例如，从扫描点到对应于墙背面的线的分配。-->

  Scan matching can also be used with self-learned maps. We use a map of reference scans which have previously been obtained in an exploration run. As the scan positions in this run have been determined by dead-reckoning only and therefore contain errors, all positions have to be corrected first. This is done by the approach proposed in [13, 9] which computes a consistent map of the environment. Figure 2 shows an overlay of a learned and hand-crafted map of the Bonn Computer Science environment. This map was computed using two 180 degree laser-range finders. Obviously, scan matching produces extremely accurate maps.

  <!--扫描匹配也可以与自学习地图一起使用。 我们使用先前在勘探运行中获得的参考扫描图。 由于此次运行中的扫描位置仅由航位推算确定，因此包含错误，因此必须首先纠正所有位置。 这是通过[13,9]中提出的方法来完成的，该方法计算了一致的环境地图。图2显示了波恩计算机科学环境的学习和手工制作地图的叠加图。 该地图使用两个180度激光测距仪计算。显然，扫描匹配产生非常精确的地图。-->

  For computing a position update, a range scan is matched with one of the reference scans, usually the one whose position is closest to the current robot position. For matching we use the approach proposed in [9] which is a combination of the line-segment matching method of the first approach, and a point-to-point match [12].

  <!--为了计算位置更新，范围扫描与参考扫描之一匹配，通常是其位置最接近当前机器人位置的参考扫描。为了匹配，我们使用[9]中提出的方法，它是第一种方法的线段匹配方法和点对点匹配[12]的组合。-->

- conclusion

  This paper empirically compares two different and popular localization techniques for mobile robots: Markov localization, which represents arbitrary probability distributions across a grid of robot poses, and Kalman filtering which uses normal distributions together with scan-matching. Previous work reported in [18, 16, 9] largely focuses on the comparison of different matching strategies for Kalman filter based localization. Our work differs in that it compares different approaches to localization. While the two techniques analyzed here used similar Bayesian foundations, the choice of representation and subsequent algorithms differed significantly in their performance. The results of our empirical evaluation can be summarized broadly as follows.

  <!--本文通过实证比较了移动机器人的两种不同且流行的定位技术：马尔可夫定位，其表示机器人姿势网格上的任意概率分布，以及使用正态分布和扫描匹配的卡尔曼滤波。[18,16,9]中报道的先前工作主要集中在基于卡尔曼滤波器的定位的不同匹配策略的比较上。我们的工作不同，它比较了不同的方法来定位。虽然这里分析的两种技术使用了相似的贝叶斯基础，但表示和后续算法的选择在性能上有很大差异。我们的实证评估结果可概括如下。-->
  - When sufficient information is available from the sensors, scanmatching and Kalman filtering are more accurate, sometimes by an order of magnitude.<!--当从传感器获得足够的信息时，扫描匹配和卡尔曼滤波更准确，有时达到一个数量级。-->

  - Markov localization is more robust, in that it potentially can keep track of the robot’s position in an arbitrary probabilistic configuration. Having this position information is critical when the quality of information received from the sensors is degraded, and the odometry is unreliable.

    <!--马尔可夫定位更加稳健，因为它可以在任意概率配置中跟踪机器人的位置。 当从传感器接收的信息质量下降并且里程计不可靠时，具有该位置信息是至关重要的。-->

  The experimental evidence suggests combining these two techniques to produce a method that inherits the robustness of Markov localization and the efficiency and accuracy of Kalman filtering. Markov localization, at coarse grid spacing, could act as an overall check on the plausibility of scan matching: Whenever the position of the robot is uniquely determined, Kalman filtering is used to accurately estimate the position of the robot. As soon as Markov localization detects multiple positions where the robot is likely to be, Kalman filtering is no longer applied. As the Markov method converges on a single high-probability location, scan-matching could once again be invoked to produce high-accuracy results.

  <!--实验证据表明，结合这两种技术可以产生一种方法，该方法继承了马尔可夫定位的鲁棒性以及卡尔曼滤波的效率和准确性。粗网格间距的马尔可夫定位可以作为扫描匹配合理性的总体检查：每当机器人的位置被唯一确定时，卡尔曼滤波用于准确地估计机器人的位置。一旦马尔可夫定位检测到机器人可能的多个位置，就不再应用卡尔曼滤波。当马尔可夫方法收敛于单个高概率位置时，可再次调用扫描匹配以产生高精度结果。-->





## 2000+

### 2002

#### The Trimmed Iterative Closest Point Algorithm

> Algorithm 1: Trimmed Iterative Closest Point
>
> 1. For each point of P, find the closest point in M and compute the individual distances $d_i^2$.
> 2. Sort $d_i^2$ in ascending order, select the $N_{po}$ least values and calculate their sum $S^{'}_{LTS}$.
> 3. If any of the stopping conditions is satisfied, exit; otherwise, set $S_{LTS}=S^{'}_{LTS}$ and continue.
> 4. Compute for the $N_{po}$ selected pairs the optimal motion (R, t) that minimises $S^{'}_{LTS}$ .
> 5. Transform P according to (R, t) and go to 1.

### 2002

#### An Experimental Comparison of Localization Methods Continued

The key idea of Monte Carlo localization (MCL) is to represent $p(l)$ by sets of $n$ weighted samples $\langle l_i,\omega_i \rangle$ [9]. Each $l_i$ corresponds to a robot position, and the $\omega_i$ are non-negative numerical factors called importance weights, which sum up to one. The prediction and correction update of the sample sets is achieved by a procedure often referred to as sequential importance sampling with resampling [6]. The basic algorithm takes as input a sample set $S$ representing the current position estimate $p(l)$ , a sensor measurement $s_n$ , and action information $a_n$ . Each sample representing the posterior distribution for p ( l ) is generated according to the following three steps:

<!--蒙特卡罗定位（MCL）的关键思想是通过n个加权样本集<li,wi>来表示p(l)[9]。每个li对应于机器人位置，并且wi是称为重要性权重的非负数值因子，其总和为1。样本集的预测和校正更新是通过一个通常被称为重新采样的顺序重要性采样的过程来实现的[6]。基本算法将表示当前位置估计值p(l)的样本集S，传感器测量值sn和动作信息an作为输入。表示p(l))的后验分布的每个样本根据以下三个步骤生成：-->

- **Resampling**: Draw with replacement a random sample $l'$ from the sample set $S$ according to the (discrete) distribution defined through the importance weights $\omega_i$ . This sample corresponds to an instance of $p(l')$ in (2).<!--重采样：根据通过重要性权重wi定义的（离散）分布，从样本集S中替换随机样本l'。该样本对应于（2）中的p（l 0）的实例。-->
- **Sampling**: Use $l'$ and the control information $a_n$ to sample $l$ from the distribution $p(l|a_n,l')$ . This sample represents $p(l)$ on the left side (2).<!--采样：使用l'和控制信息an从分布p(l|an,l')中采样l。该样品代表（2）左侧的p(l))。-->
- **Importance sampling**: Weight the sample $l$ by the importance weight $p(s_n|l)$, the likelihood of the sample $l$ given the measurement $s_{n}$ .<!--重要性采样：通过重要性权重p(sn|l))对样本l进行加权，给出测量值sn的样本的可能性。-->

After $n$ iterations, the importance weights of the newly generated samples are normalized so that they sum up to 1. It can be shown that the sample set consisting of these samples in fact approximates the posterior density for $p(l)$ [6]. While these steps suffice to efficiently track a robot’s position and solve the global localization problem, this basic algorithm is highly inefficient in solving the kidnapped robot problem. Fox and colleagues [9] showed how adding random samples at each iteration allows the algorithm to efficiently recover from localization failures.

<!--在n次迭代之后，将新生成的样本的重要性权重归一化，使得它们总和为1.可以显示由这些样本组成的样本集实际上近似于p(l)的后验密度[6]。虽然这些步骤足以有效地跟踪机器人的位置并解决全局定位问题，但这种基本算法在解决被绑架机器人问题方面效率极低。Fox及其同事[9]展示了如何在每次迭代中添加随机样本，使算法能够有效地从本地化失败中恢复。-->

### 2004

#### Linear Least-Squares Optimization forPoint-to-Plane ICP Surface Registration

<!--没啥用-->



### 2006

#### Mobile Robot Motion Estimation by 2D Scan Matching with Genetic and Iterative Closest Point Algorithms

The application of scan matching methods for motion estimation can be broadly classified into three different categories:<!--扫描匹配的应用-->

1. Feature based techniques that discern distinctive geometrical patterns, such as line segments, corners or edges, from the laser readings. Computation of these features can be a hard burden for ego-motion estimation.<!--基于特征，计算量大-->
2. Compact data methods that extract mathematical properties from range measurements such as histograms, motion fields or principal eigenvectors. These characteristics can be very sensitive to measurement noise and moving objects in the sensed environment.<!--压缩数据，数据分析，噪声敏感-->
3. Point matching techniques that directly establish correspondences between spatial points from two laser scans. These are well suited for motion estimation in unstructured and dynamic environments because they directly use raw laser data. Exact point correspondence is impossible due to sensor limitations, so matching is usually regarded as an optimization problem where the maximum expected precision is intrinsically limited by the working environment and by the rangefinder performance.<!--点匹配，优化问题-->

- Formula annotation:

  {$q_{k+1}$} must be projected onto the $XY_k$ frame, which results in {$\hat{q}_k$ } according to a tentative transformation $T_k$. $T_k$ is composed of the relative displacements ($\Delta x$, $\Delta y$) and the rotation increment $\Delta \phi$ between $XY_k$ and $XY_{k+1}$ .<!--注意图示方向！！！！从k到k+1，平移T(∆x, ∆y)，顺时针旋转∆φ(该角度在直角坐标系中是负数)-->
  $$
  \left[
  \begin{matrix}
  \hat{x}_k(j) \\
  \hat{y}_k(j)
  \end{matrix}
  \right]
  =
  \left[
  \begin{matrix}
  \cos(\Delta \phi) & -\sin(\Delta \phi) \\
  \sin(\Delta \phi) & \cos(\Delta \phi)
  \end{matrix}
  \right]
  \left[
  \begin{matrix}    
  x_{k+1}(j) \\
  y_{k+1}(j)
  \end{matrix}    
  \right]
  +
  \left[
  \begin{matrix}
  \Delta x \\
  \Delta y
  \end{matrix}
  \right]
  $$
  <!--以上为顺时针旋转一个正角度的旋转公式（逆时针则是俯负角度），常见的是逆时针旋转一个正角度的旋转公式，二者是转置关系-->
  $$
  \hat{\alpha}_k(j)=\Delta \phi + \arctan(\frac{y_{k+1}(j)+\Delta y}{x_{k+1}(j)+\Delta x})
  $$
  <!--这个公式莫不是近似值？？？？？？-->
  $$
  \hat{d}_k(j)=\sqrt{(x_{k+1}(j)+\Delta x)^{2}+(y_{k+1}+\Delta y)^{2}}
  $$
  <!--这个也是啥意思？？？-->
  $$
  e_{T_{K}}(j)=e_{T_{k}}[\hat{q}_k(j),\hat{q}_k(J(j)]
  $$
  <!--点云匹配构建的误差函数-->
  $$
  p_{T_{k}}(j)=
  \begin{cases}
  0 & \text{if $|e_{T_{k}}(j)| \geq E$} \\
  1 & \text{otherwise}
  \end{cases}
  $$
  <!--去除外点的布尔函数，阈值E需要根据实验测定-->
  $$
  n_{T_{k}}=\sum_{j=0}^{N} p_{T_{k}}(j)
  $$
  <!--有效匹配点对计算公式，在布尔函数去除外点后，统计实际匹配上的点对-->
  $$
  P_{T_{k}}=\frac{n_{T_{k}}}{N+1}
  $$
  <!--匹配率，或者说重叠率，统计了有效匹配点对占全部点的比例，此处点数从下标0开始统计，到下标N，总数为N+1-->

  Exact correspondence of points from different scans is impossible due to a number of facts: <!--匹配误差来源-->

  - deformation caused by vehicle motion<!--运动畸变-->
  - spurious ranges<!--虚假距离（测距误差？？）-->
  - random noise<!--随机噪声-->
  - terrain unevenness<!--地形不均匀（倾斜的点云，错误的里程计等）-->
  - mixed pixels<!--混合像素？？-->
  - occluded areas<!--遮挡-->
  - discretized angular resolution<!--离散角分辨率-->
  - moving objects<!--动态障碍物-->, etc. 

  Then, scan matching can be thought of as an optimization problem for determining a $2D$ transformation that minimizes a well-grounded matching criterion $I_{T_{k}}$ .
  $$
  I_{T_{k}}=\frac{\sum_{j=0}^{N}[p_{T_{k}}e_{T_{k}}(j)]}{n_{T_{k}}P_{T_{k}}}
  $$
  <!--优化问题，迭代解决-->

- Iterative closest point algorithm (ICP)

  steps:

  - before ICP starts four-step iterations, use odometric motion estimation $T_{k}^{o}$ to initialize $T_{k}$, including projecting<!--里程计估计，点云投影-->
  - compute the squared distances for every possible combination of ${\hat{q}_k}$ and ${q_k}$ points<!--对每个新点云中的点，计算与参考点云中的每个点的欧式距离，供m×n次计算-->
  - ICP calculates the correspondence index function $J(j)$ based on minimum squared distances<!--找到每个新点云中的点的最近的点后，将配对点送入ICP匹配-->
  - calculates match error function<!--计算匹配误差，其中外点已经被排除了-->
  - updates motion parameters by minimizing matching index<!--最小化匹配率，更新运动参数-->

  This optimization can be solved analytically as follows.<!--优化的解析解，详见论文P7-P8-->

  This technique guarantees convergence to a local minimum that is close to the odometric estimation, which is not necessarily the optimal one. Note that the most expensive computation of ICP is to find the closest points at each iteration.<!--局部最小，不一定是全局最优-->

  The value of $E_{max}$ can be characterized from experimental calibration of odometric uncertainty. An upper bound of the expected odometric errors for each parameter of the transformation can be expressed as $b = (b_x , b_y , b_φ )$, which depends on the elapsed navigation time $t$ between two consecutive scans. Then, $E_{max}$ is calculated as a squared distance.<!--阈值E的上限可以通过里程计误差估计，与slam时间间隔t相关，计算t时间内里程计的最大误差-->

  <!--阈值E的下限与激光测距误差相关，并且是高斯分布-->

- Genetic algorithm (GA)

  GAs provide a derivative-free stochastic optimization<!--无导数随机优化--> tool, where each point in a solution space is encoded into a bit string (chromosome) and is associated with a fitness value according to a cost function. Starting from an initial random population, points with better fitness values are used to construct a new population of solutions by means of genetic operators. Basically, these are:

  - selection, that determines which individuals survive to the next generation
  - crossover, that generates new chromosomes by randomly combining parts from good solutions
  - mutation<!--突变-->, that sporadically changes some bits of the new individuals. 

  Thus, after several iterations (generations), the overall fitness value is improved by exploiting the best solutions and exploring new possibilities.



  The matching error function $e_{T_{k}}(j)$ of Eq. (5) is computed as the difference between the actual and the projected ranges:<!--比较投影后，新点云某点的测距结果与参考点云的对应点的测距结果之差的绝对值-->
  $$
  e_{T_{k}}(j)=
  \begin{cases}
  E & \text{if $(J(j)<0$ or $J(j)>N)$ and $\Phi < 360^\circ$} \\
  |d_k(J(j))-\hat{d}_k(j)| & \text{otherwise}
  \end{cases}
  $$


- Hybrid GA-ICP method

  two steps:

  - GA performs a complete but rough search around the odometric estimation $T_k^o$ that avoids local minima. <!--对里程计估计进行完整和粗略的搜索-->This is accomplished by defining a shorter bit string for each gene but maintaining the limits of the problem space. In this way, the size of the solution space is reduced, as well as running time. Hence, the GA search results in a coarse transformation $T_k^{GA}$ .
  - The ICP technique starts from $T_k^{GA}$ instead of $T_k^o$ in order to refine the coarse transformation with a local search.<!--局部搜索细化--> In this case, the maximum expected initial error $E_{max}$ has a lower bound. This reduction of search uncertainty favours fewer iterations for ICP convengence.<!--降低迭代次数-->



### 2007

#### An accurate closed-form estimate of ICP ’s covariance

$\color{red}{<参考文献无法下载！！[1][2]>}$

[Robot localization based on scan-matching—estimating the covariance matrix for the IDC algorithm](https://www.docin.com/p-1389192542.html)

The most rigorous study of the covariance estimation problem has been developed:

- the Hessian method 
  - The closed-form Hessian method over-estimates the covariance in some cases.<!--闭式方法高估了协方差-->
- the Offline method
  - The Offline method gives reasonable results but cannot be used online, as it is based on a computationally expensive procedure.<!--准确，但是计算量非常大-->

Sources of error for ICP:

- wrong convergence<!--错误的收敛-->

  ICP can converge to a local minimum out of the attraction area of the ‘true’ solution. It is essentially unavoidable. In fact, an iterative method cannot be complete, as there are situations in which two equally likely solutions exists. This kind of error is very hard to model, because global convergence results do not exist for ICP .<!--局部最小不可避免-->

- under-constrained situations<!--约束不足-->

  in some environments there is not enough information to estimate the pose of the robot completely. Apart from degenerate situations, such as having only 0, 1, or 2 distinct correspondences, in the bidimensional case the two prototypical under-constrained situations are the corridor and the circular environment .<!--没有足够的信息约束，比如走廊和圆形环境-->

  check for the under-constrained situations by examining the ==Fisher’s information matrix==<!--利用Fisher's information matrix检测约束不足环境-->

- sensor noise

  even though ICP arrives in the attraction area of the ‘true’ solution, the outcome is different because of noise. It should be noted that, in constrained situations, this error is independent of the odometry error. In fact, it is found experimentally that this error exists even if the first guess coincides with the true solution. This error is theoretically justified by the existence of a lower bound for the covariance of any estimator. For the case of localization ($S_{ref}$ is a perfect map), in [9] it is derived the Cramér–Rao lower bound ( CRB ). The ==CRB== gives a good approximation to the covariance of the ICP in localization, but it is optimistic for scan matching.<!--又一个不可避免的误差-->







### 2008

#### An ICP variant using a point-to-line metric

点到线的ICP匹配

> it converges quadratically, and in a finite number of steps<!--二次收敛，迭代次数有限-->

对比了vanilla ICP, IDC (Iterative Dual Correspondences)，和MB ICP (Metric-Based ICP)

> PL ICP is more precise, and requires less iterations. However, it is less robust to very large initial displacement errors<!--PL ICP精度更高，迭代次数更少，但对初始位姿更敏感-->

以下结论被证明：

> - Proposition 1: (Pottman et al) Properties of the ICP algorithm with point-to-point metric:
>
>   - The error metric is always decreasing; therefore, a local minimum is always reached.<!--误差度量总在减少，局部最小值总能获取-->
>
>   - The ICP algorithm, in general, exhibits linear convergence:<!--线性收敛-->
>     $$
>     ||\bold{q}_k-\bold{q}_{\infin}||<c||\bold{q}_{k-1}-\bold{q}_{\infin}||
>     $$
>     for some constant c ∈ (0, 1). The constant c depends on the direction from which the local minimum is reached. The constant can be computed (if one knows the solution), but can also be estimated from the data.
>
> - Proposition 2: (Pottman et al) Properties of the ICP algorithm with point-to-line metric:
>
>   - Minimizing the point-to-line metric is equivalent to a Gauss-Newton iteration.<!--近似高斯-牛顿迭代-->
>
>   - The algorithm converges quadratically in the case of a zero-residual problem, and a good first guess:<!--二次收敛-->
>     $$
>     ||\bold{q}_k-\bold{q}_{\infin}||^2<c||\bold{q}_{k-1}-\bold{q}_{\infin}||^2
>     $$
>
> - Proposition 3: The PL ICP algorithm converges in a finite number of steps to either a fixed point or a loop.<!--有限迭代次数-->



### 2009

#### Real-Time Correlative Scan Matching

- contributions:
  1. We describe the theoretical and practical advantages of a correlative scan matching approach.<!--相关扫描匹配-->
  2. We present a multi-resolution implementation capable of real-time operation on a conventional microprocessor.<!--多分辨率-->
  3. We show how the correlation-based method can be mapped onto a Graphics Processing Unit (GPU), freeing the CPU for other tasks.<!--使用GPU加速-->
  4. We show how covariance estimates can be obtained from the matching operation.<!--获取协方差估计-->
  5. We present a thorough empirical evaluation of our methods versus three different algorithms in common use.



## 2010+

### 2013

#### Comparing ICP Variants on Real-World Data Sets



### 2013

#### ICP-SLAM methods implementation on a bi-steerable mobile robot (*闭式解)

- six steps of ICP:
  1. selection of the set of points
  2. matching the points to the samples
  3. weighting corresponding pairs appropriately
  4. rejecting certain pairs
  5. assigning an error metric
  6. minimizing the error metric

- classical ICP and Boolean ICP

  > In ICP, the transformation between scans is found iteratively by assuming that every point in the first scan corresponds to its closest point in the second scan, and by calculating a closed form solution using these correspondences.<!--假设第一帧中的点和它在第二帧中距离最近的点匹配，用于闭式解-->
  - Step 1: Projection of laser scan
    $$
    P_{2}^{'}=T \times P_{2}
    $$

  - Step2: Data Association

  - Step3: Position estimation

    - 3.1) Scan alignment criteria
      $$
      J=\frac{1}{N} \sum_{i=1}^{N}||S_{ref}(c(i))-S_{new}^{'}(i)||^2
      $$

    - 3.2) Homogeneous transformation estimation

      > The weighting of the associated pair of points reinforces the contribution of correct associations and decreases the effect of false associations during the estimation phase. Two types of weighting are considered [16]: 
      >
      > - A binary weighting (Boolean) where the weight assigned is 1 when the association is considered correct and it takes zero value if the association is considered false. <!--相关=1；不相关=0，闭式求解-->
      > - The second type of weighting doesn’t consider associations as either exclusively "correct" or only "false", but consider the pairs of points between these two categories.<!--仅考虑点对数，非线性方程系统，迭代求解-->

      - a) Boolean weighting estimation<!--不需要初始位姿-->
      - b) Estimate by weighting pairs of points<!--需要初始位姿-->

- 对比
  - In the classical ICP method, the parameters $t_x$ , $t_y$ , and θ are calculated by solving a nonlinear equations system, where a wrong initialization can lead to algorithm divergence. Instead, the Boolean method does not require an initialization step, which betters its convergence.<!--非线性方程系统，错误的初始值会导致发散-->
  - During the map building, Boolean method ignores the encountered outliers while in the classical ICP, all points are considered. Then, the number of points being more important leads to bigger computation time even if both variants are time consuming as it is argued in the literature.
  - Boolean variant provides a better accuracy with more sensor data (smaller laser resolution), even if the computation time is then increased. On the contrary, classical ICP variant converges less when using more points.



### 2015

#### Precise indoor localization for mobile laser scanner

##### 讨论了三种slam算法：Hector SLAM，GMapping，Karto

- Hector SLAM

  - high frequency laser scanners<!--高频激光雷达-->

  - small changes between frames<!--帧间刚体变换很小，实时搜索-->

    > the search space for rigid-body transformation between scans stays small and the transformation can be found in real time

  - optimize scan endpoints with the gridmap<!--优化扫描端点与地图匹配，找到正确的刚体变换-->

    > The correct transformation is found by optimizing the alignment of scan endpoints with the map learned so far

  - use Gauss-Newton method to optimize pose<!--计算网格单元梯度，用高斯牛顿法优化刚体变换-->

    > - Gradients of the grid cells are calculated at scan end points and the pose is optimized using Gauss-Newton method
    > - Hill climbing style optimization methods such as Gauss-Newton are prone to getting stuck at local optima. <!--爬山法或者下降法，容易陷入局部最优-->

  - multiple occupancy grids<!--多分辨率网格地图的使用，确保收敛到全局最优--> 

    > To ensure convergence to global minimum, the algorithm maintains a pyramid of multiple occupancy grids with each having half the resolution of the preceding one. Scan matching is started with the coarsest resolution occupancy grid and then repeated on finer grids using the result of scan matching done at the previous resolution as the starting guess.<!--每个网格地图的分辨率是前一个网格的一半，从分辨率最粗糙的网格-->

  - Bilinear filtering is employed to reach precision greater than available from discrete sized grid cells<!--双线性滤波用于提高精度-->

  - use IMU to project scans to horizontal plane<!--用惯导将点云投影到水平面上，提高精度-->

    > As Hector SLAM only uses the scan endpoints for matching and does not use the structure of a laser scan, the endpoints can be processed in fiexible way. This enables Hector SLAM to take into account attitude information from IMU which is used for projecting the laser scans to horizontal plane. This improves matching precision if the laser scan attitude differs from horizontal.

  - accumulate errors<!--没有闭环，累积误差，位姿缓慢漂移-->

    > Even though the high quality scan matcher gives the algorithm good accuracy, the position slowly drifts away  and during the closing of a loop the lack of optimizing backend can show up as a discrete jump in the trajectory when the pose is recovered. If the accumulated error is large enough, the pose might not recover and the errors continue to accumulate while the mapper overwrites previously built map.

- GMapping

  - Rao-Blackwellized particle filter<!--RBPF-->

    > Particle filter is a non-parametric implementation of a Bayesian Filter where, instead of describing the posterior distribution with a parametric function, it is represented by a set of discrete samples (particles) drawn from this distribution.<!--贝叶斯滤波器的非参数实现，从分布中抽取离散样本-->

  - MCL<!--Monte Carlo Localization蒙特卡罗定位方法-->

    > The localization step of GMapping is similar to the Monte Carlo Localization (MCL) method, where large number of particles $x_t$ , each representing a possible pose (state) of the robot at a specific time is created and maintained.<!--创建并维护大量粒子，每个粒子代表特定时间的可能位姿（状态）--> The localization is an iterative process which requires a set of particles $X_{t-1}$ (pose hypotheses), control $u_t$ , measurement $z_t$ and a map $m$ as an input.<!--定位是一个迭代过程，需要一组粒子（位姿假设），控制，测量和地图作为输入--> As the first step of each iteration of the filter and for each particle, the probability $p(x_t | x_{t-1},u_t)$ which is based on the motion model is sampled to get a new pose hypothesis $x_t$ . A weight $w_t=p(z_t|x_t,m)$ is then calculated based on the measurement model and assigned to the pose hypothesis. These new particles and corresponding weights form a temporary set $\bar{X_t}$ .<!--迭代的第一步对于每一个粒子，对基于运动模型的概率p进行采样以获得新的姿势假设。基于测量模型计算权重，并分配给姿势假设。新的粒子和对应的权重形成了一个临时集合X--> To complete the iteration of the filter, resampling of the particles is done. A resampled set $X_t$ is created by drawing with replacement from $\bar X_t$ with each particle drawn with probability based on its weight. Now even though the sample set was drawn from just the motion model, the resampling based on the measurement model has changed the distribution to reflect the measurements.<!--重采样粒子以完成滤波器迭代。从临时集合X以及权重形成重采样集合X。即使样本集仅来自运动模型，基于测量模型的重采样也改变了分布，以反映测量结果-->

  - first estimate the movement and then estimate the map<!--先估计运动，再估计地图-->

    > During SLAM the algorithm must also estimate the map in addition to the position, each particle also holds a copy of the map generated by it so far.<!--每个粒子保存到目前为止由它生成的地图的副本-->
    >
    > The SLAM problem can be factorized to first estimating the movement and then estimating the map. This type of factoring is called Rao-Blackwellization.<!--这种因式分解成为Rao-Blackwellization-->
    >
    > $p(x_{1:t},m|z_{1:t},u_{1:t})=p(m|x_{1:t},z_{1:t}) \cdot p(x_{1:t}|z_{1:t},u_{1:t}) $

  - inefficient<!--效率低-->

    > As can be seen, the movement estimation step is similar to Monte Carlo localization based on a known map.<!--运动估计步骤类似于基于已知地图的蒙特卡洛定位--> A straight forward way to solve the SLAM problem would be by just adding a step to each iteration where for each particle the estimated pose is used to integrate the current laser scan to the map learned so far.<!--通过在每次迭代中，对于每个粒子，使用估计的姿势将当前点云集成到目前为止学习的地图中，来解决SLAM的问题--> Unfortunately it is inefficient to use as many particles as with MCL because each particle must maintain a map which requires processing power and memory. The ability to use less particles makes the algorithm less robust.<!--使用与MCL一样多的粒子是低效的，使用较少的粒子不够鲁棒-->

  - one improvement<!--使用里程计作为初始猜想的扫描匹配，对周围的区域进行采样和估计，再从该分布而不是运动模型中绘制粒子姿势-->

    > Grisetti et al. proposed an improved method where they use a more accurate $x_t$  proposal sampling process and only resample the particles when needed.<!--用扫描匹配得到的分布替代运动模型得到的分布，更精确--> The improved proposal sampling method is based on the insight that the information carried by the laser scanner measurement is much more accurate than one available from odometry. It employs scan matching, which uses the position from odometry as the initial guess. The area around the pose acquired by scan matcher is then sampled and evaluated to calculate a Gaussian approximation of the proposal distribution. New pose for the particle is then drawn from this distribution instead of the motion model.

  - another improvement<!--仅当粒子重要性权重的方差超过预定水平再进行重采样-->

    > While particle resampling is necessary to replace samples unlikely to represent the correct robot path and map with better ones, it also carries the risk of replacing the correct hypothesis with a worse one as there is random element in the resampling step.<!--粒子重采样目的是用更好的样本替换不太可能代表正确机器人路径的样本，但是无法避免用更差的假设替换正确的假设的风险--> To reduce this risk Grisetti et al. only commit the resampling step when the particle set stops approximating the target posterior well enough i.e. when the variance of particle importance weights rises above a predefined level . With these improvements the number of particles needed for consistent map building and localization by a factor of ten as compared to the method employed by Hähnel, Thurn et al.<!--粒子数减少了十倍-->

  - While the algorithm does not have an optimizing backend, the multiple hypotheses of the map that are carried by the particles help maintain a coherent map even with loop closures.<!--粒子携带的多个地图假设有助于保持连贯映射-->

- Karto

  - Karto uses correlative scan matching algorithm by in its frontend and Sparse Pose Adjustment (SPA) as its backend<!--前端是相关扫描匹配，后端是稀疏姿势调整SPA-->

  - use odo or Hector SLAM as its first guess<!--提供初始猜想-->

  - first evaluate at a coarser scale to find finer search<!--先进行大单元尺寸粗略搜索，再进行精细搜索-->

    > To narrow down the search over the large 3D space ( x, y, θ ) of possible transformations, the area is first evaluated at a coarser scale against a lookup table with larger cell size to find areas of interest for finer search. The use of multiple resolutions enables the algorithm to run in real time.<!--多分辨率使算法实时运行--> 

  - An additional advantage of the method

    > An additional advantage of the method is that, as a large variety of transformation candidates are evaluated, their likelihoods can be used to estimate covariance values for the proposed transformation.<!--优点：当评估多种变换候选者时，它们的可能性可以用于估计所提出的变换的协方差--> This value is important for the backend as a weight for the constraint created from the scan match.<!--此值对于后端很重要，因为它是从扫描匹配创建的约束的权重--> Covariance values calculated from larger set of transformations are less overconfident than the ones calculated from a smaller set of transformations.<!--从较大的变换集合计算的协方差比从较小的变换集合计算的协方差更不自信--> 

  - downside of discrete steps<!--离散步骤的缺点，误差-->

    > The downside of discrete steps is the maximum limit on accuracy they give. This is especially noticeable if every scan from a scanner is input to the algorithm as the displacement between scans can be less than the search resolution and this causes the localization to become unreliable.

  - connected chain between scans, closed loop<!--扫描间的扫描链，闭环检测-->

    > The current scan is matched with a fixed size rolling buffer of previous scans.<!--当前扫描与先前扫描的固定大小的滚动缓冲区匹配--> Old scans are searched after each scan match to see if a suffciently long connected chain falls under predetermined radius from the current location (chains closely connected to the current scan are ignored).<!--每次扫描匹配之后搜索旧扫描，以查看足够长的连接链是否落在距离当前位置的预定半径之下（忽略与当前扫描紧密连接的链）--> If a chain of scans is found a loop closure is attempted by performing scan matching between the current scan and the found chain of previously processed scans, if the scan matching succeeds with match response value larger than predetermined threshold, loop is closed and a constraint between those poses is added to the underlying pose graph.<!--如果发现扫描链，则通过在当前扫描和找到的先前处理的扫描链之间执行扫描匹配来尝试循环闭合，如果扫描匹配成功并且匹配大于阈值，则关闭循环并且将这些姿势之间的约束添加到基础姿势图中-->

  - the SPA based backend<!--基于SPA的后端-->

    > The SPA based backend of the Karto uses a pose graph structure where scanner poses $x_t$  form the nodes and the edges between them are the scan matching results $z_t$  from the frontend.<!--姿势图结构，扫描器构成节点的x，它们之间的边缘来自前端的扫描匹配结果z--> The measurements always carry some errors from sensor noise etc. and as such, there are no poses which would satisfy all the constraints.<!--传感器噪声带来的误差使没有可以满足所有约束的姿势--> Therefore the graph based SLAM problem must be considered as an optimization problem where one seeks to minimize these errors.<!--图优化--> As the number of nodes grows the accompanying non-linear constraints form a large system of equations which would be difficult to solve if it were not for the fact that as nodes are for the most part only connected to nodes nearby, the system is sparse.<!--随着节点数量的增加，伴随的非线性约束形成了一个庞大的方程组，如果不是因为节点大部分只连接到附近的节点，那么系统就很难解决--> By employing solvers specifically tailored for sparse systems such as **sparse Cholesky factorization** <!--稀疏Cholesky分解-->the optimization can be performed quickly enough to be run in real time. The Karto with its SPA backend has been shown to give excellent results when compared to other algorithms.

  - error sensitive<!--错误敏感-->

    > With maps based on occupancy grids the features and corresponding measurements are replaced with a scan matching operation between the poses where the feature was observed. The pose optimization is sensitive to false constraints caused by a failed data association during loop closure or wrong result from scan matching.<!--对于基于占用网格的图，通过在观察到特征的姿势之间的扫描匹配操作来替换特征和相应的测量。姿势优化对于由循环闭合期间的数据关联失败或扫描匹配的错误结果导致的错误约束敏感-->

##### 提出了两种方法修正：Hector SLAM, Karto

- Hector SLAM: movement distortion correction

  > - The movement of the scanner during single scan sweep causes distortion to the scan adding error to scan matching<!---运动产生扫描畸变-->
  > - If the movement is known, it can be compensated by transforming each measurement of the scan according to the scanner pose at the time of measurement<!--如果运动已知，可以通过变换每个测量来补偿-->
  > - the IMU measurements could be used to estimate this movement but to keep the estimates from drifting away, fusion of the IMU and SLAM estimates with help of for example extended Kalman filter (EKF) would be required<!--IMU测量可以用于估计以上运动，并且为了保持估计不会漂移，需要EKF融合IMU和SLAM估计-->
  > - As an intermediate measure only the rotational component of the movement is considered. This is more straightforward as the IMU provides angular velocity measurements and orientation estimates based on them. Even though the absolute heading (rotation around z-axis) value based on the IMU measurements drifts away from the truth, it can be used to estimate the relative rotation over short periods of time accurately.<!--仅考虑运动的旋转分量，即使绝对航向出现了偏差，测量期间的短时间相对旋转也是精确估计的-->
  > - Karto and GMapping algorithms make assumptions of undistorted scan when doing scan matching, which makes implementing movement compensation scheme implemented for Hector SLAM more difficult.<!--Karto和GMapping假设扫描无畸变-->

- Karto: modify the loop closing to use offline with higher accuracy

  > - In its original form a loop is closed immediately when a match between current scan and a chain of nearby processed scans gives a response value greater than a predetermined threshold. <!--原始形式中，当前扫描与附近的扫描链匹配结果大于阈值时，立即闭合循环-->
  > - As processing is done offine, there is no immediate need to close the loops and an approach called delayed loop closure is used instead.<!--在离线处理中，不需要立即关闭循环，而是延迟关闭循环-->
  > - With delayed loop closure when an acceptable loop closure is found, instead of closing it the scan and corresponding scan chain are saved and the processing of subsequent scans is continued. Loop closure attempts with saved scan chain are continued during the processing of the subsequent scans and if a higher response value is attained, the saved scan is updated with the scan with better match. This is continued as long as loop closure is attempted with a member of the saved scan chain. After no more updates to the saved scan and corresponding chain are attempted the loop is closed.<!--在发现可接受的循环闭合时，不要立刻关闭循环，而是保存当前扫描和相应的扫描链，继续后续扫描和循环闭合尝试，当获得更高响应的循环闭合结果时，用更好的匹配来更新保存的扫描，直到不再尝试循环闭合，则关闭循环闭合-->
  > - This is advantageous as a better match should lead to smaller error and the match generally gets better as the scanning locations get closer together. <!--优势：更好的匹配应该导致更小的误差并且匹配通常随着扫描位置变得更靠近而变得更好-->
  > - In the original form, if the threshold is small a suboptimal loop closure is made when the current location is still far away from the chain of scans but if the threshold is too high, some not as good but still useful loop closures might be missed altogether. With the delayed loop closure approach more loop closures can be made while ensuring that they are as good as possible.<!--原始形式中，如果阈值小，则会接受次优的闭环，反之阈值大，则会错过有用的不太好的闭环-->

##### 三种算法之间的关联

- Hector SLAM是唯一不需要里程计初始估计的算法，它全局搜索

  > The Hector SLAM is the only algorithm studied which works without an initial guess of movement from odometry.

- Hector SLAM从获取的点云和IMU数据中得到初始估计，提供给GMapping和Karto

  > - running the laser scan and IMU data through Hector SLAM algorithm to compute an initial trajectory estimate.
  > - The GMapping and Karto algorithms require an initial guess and as there are no wheel encoders to provide odometry in Slammer platform, the Hector trajectory is used instead.

- Hector SLAM提供密集轨迹，但是没有闭环，轨迹会慢慢偏离，足够提供给其他算法作为初始估计

  > - The Hector SLAM naturally provides dense trajectory and in environments with good overall visibility of the area, i.e. no loop closures after long periods exploring new areas, the algorithm could provide a sufficiently accurate trajectory.
  > - the accurate matching provides locally very good estimates of the path taken and as such can be used to generate a sort of accurate fake odometry which can then be employed by the other algorithms.

- Hector SLAM产生的大的不连续位姿在用于其他算法之前用前两次位姿的均值被平滑掉

  > - This is accomplished by checking each transformation between two poses and by comparing it to the preceding ones.
  > - If there is a large change in the transformation length or direction from the preceding ones, the transformation is replaced by an average of the preceding transformations.

- 从Hector SLAM到Karto和GMapping并行处理，并且频率为3Hz

- Karto和GMapping都支持时间选择性扫描匹配，静止时获取最佳匹配

  > - Both algorithms also support the ability to process a new scan only if the scanner has moved predetermined amount since last scan used. 
  > - Time based subset approach was selected as it enables the algorithms to process scans also when not moving. When stationary the scans are undistorted and should provide the best matches.

- 将Karto或GMapping得到的稀疏轨迹投影到Hector SLAM产生的密集轨迹，并插值

  > - After processing with Karto or GMapping the produced sparse trajectory is combined with the dense smoothed Hector trajectory. 
  > - This is accomplished by performing an affine transformation to each subtrajectory which aligns the subtrajectory endpoints with the sparse trajectory. 
  > - Headings of the poses of subtrajectories are interpolated from the sparse trajectory.

- 整合的算法才能提供密集，平滑和全局一致的轨迹

  > - Hector SLAM fills the first two requirements but it has no functionality to reduce the errors in trajectory caused by accumulating drift except for discrete jumps. 
  > - GMapping and Karto both have problems if the displacement between scans is too small and can not provide reliable trajectory if they need to process every subsequent scan.

- Hector SLAM和Karto用更多的时间寻找更好的参数，因此轨迹比GMapping更平滑

  > - The large errors caused by the localization diverging with GMapping when not moving made the RMSE values less valuable for estimating the actual trajectory accuracy. 
  > - The inspection of error time series and trajectories made it clear that Karto provided trajectories which were more coherent and closer to the reference. This inspection also revealed that with Karto the smoothness of the Hector trajectory is better preserved than with GMapping. Of course it must be taken in to account that more time was spent finding good parameters for Karto and Hector than for GMapping. With better parameter values the jitter in the GMapping trajectory might disappear altogether.
  > - GMapping processing also contains a random element in both the resample and the motion model sampling steps, which adds an additional source of uncertainty to the process. Getting a different trajectory every time the same dataset is processed is not desirable but the differences are small and total failure is unlikely. It should be noted that Karto too fails occasionally, either through the error metric of optimization becoming Not A Number (NaN) or with loop closures associating wrong scans together. This can result in a worse trajectory than the initial trajectory given by Hector SLAM without the trajectory being noticeable wrong.



### 2015

#### A Review of Point Cloud Registration Algorithms for Mobile Robotics

##### 阶段算法分类

- reading and reference
  - Sensor types
    - Photometric, time-of-flight, phase-shift, triangulation
  - Applications
    - Scene/object reconstruction, identification, tracking
  - Data acquisition
    - Different viewpoints, different times, different sensors, scene to model
- Initial Transformations
  - Sources
    - External sensors, users, other registration algorithms
  - Types
    - Single hypothesis, multiple hypotheses
  - Systems
    - Iterative, cascade
- Data Filters
  - Goals
    - Enhance discrepancy, reduce time, reduce noise
  - Descriptor invariance
    - Rotation, translation, scale, affine
  - Feature Relationship
    - Unstructured, structured
  - Support
    - Laser intensity, color, geometry
- Association Solvers
  - Types
    - Feature, descriptor, mixed
  - Direction
    - Reading to reference, reference to reading, both
  - Distance metric
    - Euclidean, Mahalanobis, $\chi ^2 $ test statistic, custom
  - Optimization
    - Hashing/indexing, static space partitioning, dynamic space partitioning, feature reduction
- Outlier Filters
  - Outlier sources
    - Partial observations, dynamic elements, sensor
      noises
  - Support
    - Features, descriptors, mixed
  - Assignment
    - Hard, soft, mixed
- Error Minimizations
  - Error
    - Geometric, morphing, entropy
  - Deformation
    - Global, local
  - Minimization schemes
    - Closed form, small angle approximation, voting















