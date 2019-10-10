# A Tutorial on Graph-Based SLAM

## 1. INTRODUCTION

> These approaches can be classified either as filtering or smoothing. Filtering approaches model the problem as an on-line state estimation where the state of the system consists in the current robot position and the map. The estimate is augmented and refined by incorporating the new measurements as they become available. Popular techniques like Kalman and information filters [28], [3], particle filters [22], [12], [9], or information filters [7], [31] fall into this category. To highlight their incremental nature, the filtering approaches are usually referred to as on-line SLAM methods. Conversely, smoothing approaches estimate the full trajectory of the robot from the full set of measurements [21], [5], [27]. These approaches address the so-called full SLAM problem, and they typically rely on least-square error minimization techniques.

这些方法可以分类为滤波或平滑。滤波方法将问题建模为在线状态估计，其中系统状态包括当前机器人位置和地图。当新的测量结果可用时，通过合并新的测量结果来增加和改进估计值。像卡尔曼和信息滤波器[28]、[3]、粒子滤波器[22]、[12]、[9]或信息滤波器[7]、[31]这样的流行技术属于这一类。为了突出它们的增量性质，滤波方法通常被称为on-line SLAM方法。相反，平滑方法根据全套测量结果来估计机器人的完整轨迹[21]、[5]、[27]。这些方法解决了所谓的full SLAM问题，它们通常依赖于**最小二乘误差最小化技术**。

> Recent insights into the structure of the SLAM problem and advancements in the fields of sparse linear algebra resulted in efficient approaches to the optimization problem at hand. Consequently, graph-based SLAM methods have undergone a renaissance and currently belong to the state-of-the-art techniques with respect to speed and accuracy.

最近对SLAM问题的结构和稀疏线性代数领域的进展的深入研究产生了对现有优化问题的有效方法。因此，基于图形的SLAM方法经历了一次复兴，目前在速度和准确性方面属于最先进的技术。

## 2. PROBABILISTIC FORMULATION OF SLAM

> Maps can be parametrized as a set of spatially located landmarks, by dense representations like occupancy grids, surface maps, or by raw sensor measurements. The choice of a particular map representation depends on the sensors used, on the characteristics of the environment, and on the estimation algorithm. Landmark maps [28], [22] are often preferred in environments where locally distinguishable features can be identified and especially when cameras are used. In contrast, dense representations [33], [12], [9] are usually used in conjunction with range sensors. Independently of the type of the representation, the map is defined by the measurements and the locations where these measurements have been acquired [17], [18].

地图可以参数化为一组空间定位的地标，通过密集的表示，如占用网格、表面地图或原始传感器测量。特定地图表示的选择取决于使用的传感器、环境特征和估计算法。地标地图[28]、[22]通常在能够识别本地可识别特征的环境中，尤其是在使用相机时，更是首选。相反，密集表示法[33]、[12]、[9]通常与距离传感器一起使用。与表示类型无关，地图由测量值和获取这些测量值的位置定义[17]，[18]。

> Estimating the posterior given in (1) involves operating in high dimensional state spaces. This would not be tractable if the SLAM problem would not have a well defined structure. This structure arises from certain and commonly done assumptions, namely the static world assumption and the Markov assumption. A convenient way to describe this structure is via the dynamic Bayesian network (DBN) depicted in Figure 4. A Bayesian network is a graphical model that describes a stochastic process as a directed graph. The graph has one node for each random variable in the process, and a directed edge (or arrow) between two nodes models a conditional dependence between them.

估计（1）中给出的后验涉及在高维状态空间中操作。如果冲击问题没有一个明确的结构，这将是不可处理的。这种结构源于某些常见的假设，即静态世界假设和马尔可夫假设。描述这种结构的一种方便方法是通过图4中描述的**动态贝叶斯网络（DBN）**。贝叶斯网络是将随机过程描述为**有向图**的图形模型。图中每个随机变量都有一个节点，两个节点之间的有向边（或箭头）模拟它们之间的条件依赖关系。

> The connectivity of the DBN follows a recurrent pattern characterized by the state transition model and by the observation model.

DBN的连通性遵循以状态转换模型和观测模型为特征的循环模式。

> Expressing SLAM as a DBN highlights its temporal structure, and therefore this formalism is well suited to describe filtering processes that can be used to tackle the SLAM problem.

将SLAM表示为DBN突出了它的时间结构，因此这种形式主义非常适合描述可以用来解决SLAM问题的**滤波过程**。

> An alternative representation to the DBN is via the so-called “graph-based” or “network-based” formulation of the SLAM problem, that highlights the underlying spatial structure. In graph-based SLAM, the poses of the robot are modeled by nodes in a graph and labeled with their position in the environment [21], [18]. Spatial constraints between poses that result from observations or from odometry measurements are encoded in the edges between the nodes. More in detail, a graph-based SLAM algorithm constructs a graph out of the raw sensor measurements. Each node in the graph represents a robot position and a measurement acquired at that position. An edge between two nodes represents a spatial constraint relating the two robot poses. A constraint consists in a probability distribution over the relative transformations between the two poses. These transformations are either odometry measurements between sequential robot positions or are determined by aligning the observations acquired at the two robot locations. Once the graph is constructed one seeks to find the configuration of the robot poses that best satisfies the constraints. Thus, in graph-based SLAM the problem is decoupled in two tasks: constructing the graph from the raw measurements (graph construction), determining the most likely configuration of the poses given the edges of the graph (graph optimization).

DBN的另一种表示方式是通过所谓的“基于图”或“基于网络”的SLAM问题公式，这突出了底层的空间结构。在基于图形的SLAM中，机器人的姿势由图形中的节点建模，并标记其在环境中的位置[21]，[18]。由观测或里程测量得出的姿态之间的空间约束编码在节点之间的边缘。更详细地说，基于图的SLAM算法从原始传感器测量中构造出一个图。图中的每个节点表示一个机器人位置和在该位置获取的测量值。两个节点之间的边表示与两个机器人姿态相关的空间约束。约束包括两个姿态之间相对变换的概率分布。这些转换要么是连续机器人位置之间的里程计测量，要么是通过对齐在两个机器人位置上获得的观测值来确定的。一旦建立了这个图，我们就会寻找最能满足约束条件的机器人姿态的配置。因此，在基于图形的SLAM中，问题在两个任务中分离：从原始测量构造图形**（图形构造）**，确定给定图形边缘的最可能姿势配置**（图形优化）**。 

> The graph construction is usually called front-end and it is heavily sensor dependent, while the second part is called back-end and relies on an abstract representation of the data which is sensor agnostic. 

图形结构通常称为前端，它与传感器有很大的依赖关系，而第二部分称为后端，它依赖于传感器不可知的数据的抽象表示。

## 3. RELATED WORK

> GraphSLAM [32] applies variable elimination techniques to reduce the dimensionality of the optimization problem. The ATLAS framework [2] constructs a two-level hierarchy of graphs and employs a Kalman filter to construct the bottom level. Then, a global optimization approach aligns the local maps at the second level. Similar to ATLAS, Estrada et al. proposed Hierarchical SLAM [6] as a technique for using independent local maps.

GraphSLAM[32]应用变量消除技术来减少优化问题的维数。ATLAS框架[2]构建了一个两级的图层次结构，并使用Kalman过滤器构建底层。然后，全局优化方法在第二层对齐局部映射。与ATLAS相似，Estrada等人提出了分层SLAM[6]作为一种使用**独立局部地图**的技术。

## 4. GRAPH-BASED SLAM

> A graph-based SLAM approach constructs a simplified estimation problem by abstracting the raw sensor measurements. These raw measurements are replaced by the edges in the graph which can then be seen as “virtual measurements”. More in detail an edge between two nodes is labeled with a probability distribution over the relative locations of the two poses, conditioned to their mutual measurements.

基于图的SLAM方法通过提取原始传感器测量值来构造一个简化的估计问题。这些原始测量值被图中的边所取代，这些边被视为“虚拟测量”。更详细地说，两个节点之间的边在两个姿态的相对位置上标记了概率分布，条件是它们的相互测量。

> In general, the observation model is multi-modal and therefore the Gaussian assumption does not hold. This means that a single observation might result in multiple potential edges connecting different poses in the graph and the graph connectivity needs itself to be described as a probability distribution. Directly dealing with this multi-modality in the estimation process would lead to a combinatorial explosion of the complexity. As a result of that, most practical approaches restrict the estimate to the most likely topology.

一般来说，**观测模型是多模态的，因此高斯假设不成立**。这意味着一次观察可能会导致多个潜在的边缘连接图中不同的位置，图的连通性需要被描述为概率分布。在估计过程中直接处理这种多模态将导致复杂度的组合爆炸。因此，大多数实用的方法都将估计限制在最可能的拓扑上。

> Thus, one needs to determine the most likely constraint resulting from an observation. This decision depends on the probability distribution over the robot poses. This problem is known as data association and is usually addressed by the SLAM front-end. To compute the correct data-association, a front-end usually requires a consistent estimate of the conditional prior over the robot trajectory. This requires to interleave the execution of the front-end and of the back-end while the robot explores the environment. Therefore, the accuracy and the efficiency of the back-end is crucial to the design of a good SLAM system. In this tutorial, we will not describe sophisticated approaches to the data association problem. Such methods tackle association by means of spectral clustering [27], joint compatibility branch and bound [23], or backtracking [13]. We rather assume that the given front-end provides consistent estimates.

因此，我们需要确定由观察结果产生的最可能的约束。这个决定取决于机器人姿态的概率分布。这个问题称为**数据关联**，通常由SLAM前端解决。为了计算正确的数据关联，前端通常需要对机器人轨迹上的条件先验进行一致的估计。这需要在机器人探索环境的同时，交错执行前端和后端。因此，后端的精度和效率对于设计一个好的SLAM系统至关重要。在本教程中，我们将不介绍解决数据关联问题的复杂方法。这种方法通过谱聚类[27]、联合相容分支和界[23]或回溯[13]来处理关联。我们宁愿假设给定的前端提供一致的估计。

> If the observations are affected by (locally) Gaussian noise and the data association is known, the goal of a graph-based mapping algorithm is to compute a Gaussian approximation of the posterior over the robot trajectory. This involves computing the mean of this Gaussian as the configuration of the nodes that maximizes the likelihood of the observations. Once this mean is known the information matrix of the Gaussian can be obtained in a straightforward fashion, as explained in Section IV-B. In the following we will characterize the task of finding this maximum as a constraint optimization problem. We will also introduce parts of the notation illustrated in Figure 6.

如果观测值受到（局部）**高斯噪声**的影响，并且已知数据关联，则**基于图的映射算法的目标是计算机器人轨迹上后验点的高斯近似值**。这涉及到计算高斯的平均值作为最大化观测可能性的节点配置。一旦这一平均值已知，高斯的信息矩阵就可以直接获得，如第IV-B节所述。在下文中，我们将把找到这一最大值的任务描述为约束优化问题。我们还将介绍图6中所示的部分符号。

$\bf{x}$为参数向量，${\bf{x}}_i$描述了节点$i$的位姿：
$$
{\bf{x}}=({\bf{x}}_1, \dots , {\bf{x}}_T)^T
$$
${\bf{z}}_{ij}$为节点$i$和节点$j$之间的虚拟测量的平均值（高斯分布的均值）；

${\bf{\Omega}}_{ij}为节点$$i$和节点$j$之间的虚拟测量的信息矩阵（高斯分布的协方差矩阵）；

> This virtual measurement is a transformation that makes the observations acquired from i maximally overlap with the observation acquired from j.

这种虚拟测量是一种变换，使从$i$获得的观测值与从$j$获得的观测值最大程度地重叠。（虚拟测量量=测量量之间的变化量）

$\hat{\bf{z}}_{ij}({\bf{x}}_i, {\bf{x}}_j)$是节点${\bf{x}}_i$和${\bf{x}}_j$的给定配置的虚拟测量的预测；

> Usually this prediction is the relative transformation between the two nodes.

通常这种预测是两个节点之间的相对转换。

虚拟测量${\bf{z}}_{ij}$的log形式的似然${\bf{l}}_{ij}$为（高斯分布的log形式，省略系数）：
$$
{\bf{l}}_{ij} \varpropto [{\bf{z}}_{ij}-\hat{\bf{z}}_{ij}({\bf{x}}_i,{\bf{x}}_j)]^T{\bf{\Omega}}_{ij}[{\bf{z}}_{ij}-\hat{\bf{z}}_{ij}({\bf{x}}_i,{\bf{x}}_j)]
$$
${\bf{e}}({\bf{x}}_i,{\bf{x}}_j,{\bf{z}}_{ij})$为一个计算预期观测$\hat{\bf{z}}_{ij}$和机器人收集到的实际观测${\bf{z}}_{ij}$之间差异的函数。为了便于记法，我们将按照下标把测量编码为误差函数：
$$
{\bf{e}}_{ij}({\bf{x}}_i,{\bf{x}}_j)={\bf{z}}_{ij}-\hat{\bf{z}}_{ij}({\bf{x}}_i,{\bf{x}}_j)
$$

> Figure 7 illustrates the functions and the quantities that play a role in defining an edge of the graph. Let C be the set of pairs of indices for which a constraint (observation) z exists. The goal of a maximum likelihood approach is to find the configuration of the nodes x∗ that minimizes the negative log likelihood F(x) of all the observations.

图7说明了在定义图的边缘时起作用的函数和数据量。设$C$为存在约束（观测）$\bf{z}$的索引对集（比如$({\bf{x}}_i,{\bf{x}}_j)$）。**最大似然法的目标是找到节点${\bf{x}}^*$的配置，使所有观测的负对数似然${\bf{F}}({\bf{x}})$最小化**：
$$
{\bf{F}}({\bf{x}})=\sum_{\langle i,j \rangle \in {\cal{C}}} \underbrace{{\bf{e}}^T_{ij}{\bf{\Omega}}_{ij}{\bf{e}}_{ij}}_{{\bf{F}}_{ij}}
$$
因此，它寻求解决以下方程：
$$
{\bf{x}}^*=\arg \min {\bf{F}}({\bf{x}})
$$

> In the remainder of this section we will describe an approach to solve Eq. 5 and to compute a Gaussian approximation of the posterior over the robot trajectory. Whereas the proposed approach utilizes standard optimization methods, like the Gauss-Newton or the Levenberg-Marquardt algorithms, it is particularly efficient because it effectively exploits the structure of the problem.

在本节的其余部分，我们将描述一种求解等式5的方法，并计算机器人轨迹上后验的高斯近似值。虽然所提出的方法使用了标准的优化方法，如**高斯-牛顿或列文伯格-马夸特算法**，但由于它有效地利用了问题的结构，因此特别有效。

> We first describe a direct implementation of traditional non-linear least-squares optimization. Subsequently, we introduce a workaround that allows to deal with the singularities in the representation of the robot poses in an elegant manner.

我们首先描述了传统非线性最小二乘优化的直接实现。随后，我们介绍了一种解决方法，可以优雅地处理机器人姿态表示中的**奇点**。

### A. Error Minimization via Iterative Local Linearizations

> If a good initial guess of the robot’s poses is known, the numerical solution of Eq. (5) can be obtained by using the popular Gauss-Newton or Levenberg-Marquardt algorithms. The idea is to approximate the error function by its first order Taylor expansion around the current initial guess.

如果对机器人的姿态有一个很好的初始猜测$\check{\bf{x}}$，则可以使用流行的**Gauss-Newton或Levenberg-Marquardt算法**得到方程（5）的**数值解**。其思想是**利用误差函数在当前初始猜测$\check{\bf{x}}$附近的一阶泰勒展开来近误差函数**：
$$
{\bf{e}}_{ij}(\check{\bf x}_i+\Delta {\bf x}_i,\check{\bf x}_j+\Delta {\bf x}_j)={\bf e}_{ij}(\check{\bf x}+\Delta {\bf x}) \simeq {\bf e}_{ij}+{\bf J}_{ij} \Delta {\bf x}
$$
${\bf J}_{ij}$是${\bf e}_{ij}({\bf x})$对$\check{\bf x}$的雅可比矩阵（一阶导数矩阵）。${\bf e}_{ij}={\bf e}_{ij}(\check{\bf x})$。

将误差函数${\bf e}_{ij}$代入负对数似然${\bf F}_{ij}$中：
$$
\begin{align}
{\bf F}_{ij}&(\check{\bf x}+\Delta {\bf x}) \\
 &= {\bf e}_{ij}(\check{\bf x}+\Delta {\bf x})^T{\bf \Omega}_{ij}{\bf e}_{ij}(\check{\bf x}+\Delta {\bf x}) \\
 & \simeq ({\bf e}_{ij}+{\bf J}_{ij} \Delta {\bf x})^T{\bf \Omega}_{ij}({\bf e}_{ij}+{\bf J}_{ij} \Delta {\bf x}) \\
 &= \underbrace{{\bf e}^T_{ij}{\bf \Omega}_{ij}{\bf e}_{ij}}_{{\bf c}_{ij}}+2 \underbrace{{\bf e}^T_{ij}{\bf \Omega}_{ij}{\bf J}_{ij}}_{{\bf b}_{ij}}\Delta {\bf x}+\Delta {\bf x}^T \underbrace{{\bf J}^T_{ij}{\bf \Omega}_{ij}{\bf J}_{ij}}_{{\bf H}_{ij}}\Delta {\bf x} \\
 &= {\bf c}_{ij}+2{\bf b}_{ij}\Delta {\bf x}+\Delta {\bf x}^T{\bf H}_{ij}\Delta {\bf x}
\end{align}
$$
重写公式（4）的似然函数${\bf F}({\bf x})$：
$$
\begin{align}
{\bf F}(\check{\bf x}+\Delta {\bf x}) &= \sum_{\langle i,j \rangle \in {\cal C}}{\bf F}_{ij}(\check{\bf x}+\Delta {\bf x}) \\
& \simeq \sum_{\langle i,j \rangle \in {\cal C}} {\bf c}_{ij}+2{\bf b}_{ij}\Delta {\bf x}+\Delta {\bf x}^T{\bf H}_{ij}\Delta {\bf x} \\
&= {\bf c}+2{\bf b}^T\Delta {\bf x}+\Delta {\bf x}^T{\bf H}\Delta {\bf x}
\end{align}
$$
其中，令${\bf c}=\sum {\bf c}_{ij}$，${\bf b}=\sum {\bf b}_{ij}$，以及${\bf H}=\sum {\bf H}_{ij}$。

对公式（14）求一阶导数，解线性系统，得到似然函数${\bf F}({\bf x})$的最小值，对应的$\Delta {\bf x}$为$\Delta {\bf x}^*$：
$$
{\bf H}\Delta {\bf x}^*=-{\bf b}
$$

> The matrix H is the information matrix of the system, since it is obtained by projecting the measurement error in the space of the trajectories via the Jacobians. It is sparse by construction, having non-zeros between poses connected by a constraint. Its number of non-zero blocks is twice the number of constrains plus the number of nodes. This allows to solve Eq. (15) by sparse Cholesky factorization. An efficient yet compact implementation of sparse Cholesky factorization can be found in the library CSparse [4].

**矩阵${\bf H}$是系统的信息矩阵**，因为它是通过雅可比矩阵在轨迹空间投影测量误差而得到的。构造后它是**稀疏**的，在由约束连接的姿势之间有非零。**它的非零块数是约束数和节点数的两倍**。这允许通过**稀疏Cholesky分解**来求解等式（15）。稀疏Cholesky分解的一个高效而紧凑的实现可以在CSparse[4]库中找到。

然后将计算出的增量与初始猜测相加，得到线性化的解：
$$
{\bf x}^*=\check{\bf x}+\Delta {\bf x}^*
$$

> The popular Gauss-Newton algorithm iterates the linearization in Eq. (14), the solution in Eq. (15), and the update step in Eq. (16). In every iteration, the previous solution is used as the linearization point and the initial guess.

流行的Gauss-Newton算法迭代：1. 式（14）中的线性化；2.式（15）中的解；3.式（16）中的更新步骤。在每次迭代中，前一解作为线性化点和初始猜测。

> The procedure described above is a general approach to multivariate function minimization, here derived for the special case of the SLAM problem. The general approach, however, assumes that the space of parameters x is Euclidean, which is not valid for SLAM and may lead to sub-optimal solutions.

上述步骤是多元函数极小化的一般方法，这里是针对SLAM问题的特殊情况导出的。然而，一般的方法假设参数$\rm{x}$的空间是欧氏空间，这对于SLAM是无效的，并且可能导致次优解。**(欧式空间是二维的，SLAM位姿是三维的)**

### B. Considerations about the Structure of the Linearized System

> According to Eq. (14), the matrix H and the vector b are obtained by summing up a set of matrices and vectors, one for every constraint. Every constraint will contribute to the system with an addend term. The structure of this addend depends on the Jacobian of the error function. Since the error function of a constraint depends only on the values of two nodes, the Jacobian in Eq. (7) has the following form:

根据式（14），通过对一组矩阵和向量求和得到矩阵$\rm{H}$和向量$\rm{b}$，每个约束对应一个。每个约束都有助于系统的加法项。**这个加法器的结构取决于误差函数的雅可比**。**由于约束的误差函数仅取决于两个节点的值**，因此等式（7）中的雅可比具有以下形式：
$$
{\rm J}_{ij}=\left ( 0 \cdots 0 \ \underbrace{{\rm A}_{ij}}_{{\rm node} \ i} \ 0 \cdots 0 \ \underbrace{{\rm B}_{ij}}_{{\rm node} \ j} \ 0 \cdots 0 \right )
$$
这里${\rm A}_{ij}$和${\rm B}_{ij}$是误差函数对${\rm x}_i$和${\rm x}_j$的导数。从式（10）我们得到块矩阵${\rm H}_{ij}$的以下结构：
$$
{\rm H}_{ij}=
\begin{pmatrix}
\ddots \\
\quad & {\rm A}^T_{ij} {\rm \Omega}_{ij} {\rm A}_{ij} & \cdots & {\rm A}^T_{ij} {\rm \Omega}_{ij} {\rm B}_{ij} \\
\quad & \vdots & \ddots & \vdots \\
\quad & {\rm B}^T_{ij} {\rm \Omega}_{ij} {\rm A}_{ij} & \cdots & {\rm B}^T_{ij} {\rm \Omega}_{ij} {\rm B}_{ij} \\
\quad & \quad & \quad & \ddots
\end{pmatrix}
$$

$$
{\rm b}_{ij}=
\begin{pmatrix}
\vdots \\ {\rm A}^T_{ij} {\rm \Omega}_{ij} {\rm e}_{ij} \\ \vdots \\ {\rm B}^T_{ij} {\rm \Omega}_{ij} {\rm e}_{ij} \\ \vdots
\end{pmatrix}
$$

==此处有疑问，为什么${\rm b}_{ij}$的形式不是一维行向量？==
$$
{\rm b}_{ij}=
\begin{pmatrix}
\cdots & {\rm A}^T_{ij} {\rm \Omega}_{ij} {\rm e}_{ij} & \cdots & {\rm B}^T_{ij} {\rm \Omega}_{ij} {\rm e}_{ij} & \cdots
\end{pmatrix}
$$
为了简单起见，我们省略了零块。

> Algorithm 1 summarizes an iterative Gauss-Newton procedure to determine both the mean and the information matrix of the posterior over the robot poses. Since most of the structures in the system are sparse, we recommend to use memory efficient representations to store the Hessian H of the system. Since the structure of the Hessian is known in advance from the connectivity of the graph, we recommend to pre-allocate the Hessian once at the beginning of the iterations and to update it in place by looping over all edges whenever a new linearization is required. Each edge contributes to the blocks H [ii] , H [ij] , H [ji] , and H [jj] and to the blocks b [i] and b [j] of the coefficient vector. An additional optimization is to compute only the upper triangular part of H, since it is symmetric. Note that the error of a constraint e ij depends only on the relative position of the connected poses x i and x j . Accordingly, the error F(x) of a particular configuration of the poses x is invariant under a rigid transformation of all the poses. This results in Eq. 15 being under determined. To numerically solve this system it is therefore common practice to constrain one of the increments ∆x k to be zero. This can be done by adding the identity matrix to the k th diagonal block H[kk]. Without loss of generality in Algorithm 1 we fix the first node x 1 . An alternative way to fix a particular node of the pose-graph consists in suppressing the k th block row and the k th block column of the linear system in Eq. 15.

算法1总结了一种迭代Gauss-Newton算法，用于确定机器人姿态的后验均值${\rm x}$和信息矩阵${\rm H}$。由于系统中的大多数结构都是稀疏的，我们建议使用内存有效的表示来存储系统的Hessian ${\rm H}$。由于Hessian的结构是预先从图的连通性知道的，我们建议在迭代开始时预先分配一次Hessian，并**在需要新的线性化时通过在所有边上循环来更新它。每一条边贡献给块${\rm H}_{ii}$、${\rm H}_{ij}$、${\rm H}_{ji}$和${\rm H}_{jj}$以及系数向量的块${\rm b}_{i}$和${\rm b}_{j}$**。另外一个优化是**只计算${\rm H}$的上三角部分，因为它是对称的**。注意，约束${\rm e}_{ij}$的误差仅取决于连接位姿${\rm x}_i$和${\rm x}_j$的相对位置。因此，==在所有姿势的刚性变换下，位姿${\rm x}$的特定配置的误差${\rm F}({\rm x})$是不变的。这导致公式15被低估。因此，要对该系统进行数值求解，通常将其中一个增量$\Delta {\rm x}_k$限制为零。这可以通过将恒等矩阵添加到第k对角线块${\rm H}[kk]$来实现。==在不丧失算法1通用性的情况下，我们固定了第一个节点${\rm x}_1$。固定姿势图的特定节点的另一种方法包括在等式15中抑制线性系统的第k块行和第k块列。

(对b的形式存在疑问，对固定节点存在疑问)

### C. Least Squares on a Manifold

> A common approach in numeric to deal with non-Euclidean spaces is to perform the optimization on a manifold. A manifold is a mathematical space that is not necessarily Euclidean on a global scale, but can be seen as Euclidean on a local scale [20]. Note that the manifold-based approach described here is similar to the way of minimizing functions in SO(3) as described by Taylor and Kriegman [30].

**数值方法中处理非欧氏空间的一种常用方法是对流形进行优化。**流形是一个数学空间，在全局尺度上不一定是欧几里德空间，但在局部尺度上可以被看作欧几里德空间[20]。注意，这里描述的基于流形的方法类似于Taylor和Kriegman[30]描述的$SO(3)$中函数最小化的方法。

> In the context of the SLAM problem, each parameter block x i consists of a translation vector t i and a rotational component α i . The translation t i clearly forms a Euclidean space, while the rotational components α i span over the non-Euclidean 2D or 3D rotation group SO(2) or SO(3). To avoid singularities, these spaces are usually described in an over-parametrized way, e.g., by rotation matrices or quaternions. Directly applying Eq. (16) to these over-parametrized representations breaks the constraints induced by the over-parametrization. The over-parametrization results in additional degrees of freedom and thus introduces errors in the solution. To overcome this problem, one can use a minimal representation for the rotation (like, e.g., Euler angles in 3D). This, however, is subject to singularities. The singularities in the 2D case can be easily recovered by normalizing the angle, however in 3D this procedure is not straightforward.

在SLAM问题的背景下，每个参数块${\rm x}_i$由平移向量${\rm t}_i$和旋转分量$\alpha_i$组成。平移${\rm t}_i$清楚地形成欧氏空间，而旋转分量$\alpha_i$跨越非欧氏2D或3D旋转群$SO(2)$或$SO(3)$。为了避免奇异性，这些空间通常以**超参数化**的方式来描述，例如通过**旋转矩阵或四元数**来描述。将式（16）直接应用于这些超参数化表示，可以打破由超参数化引起的约束。过度参数化会导致额外的自由度，从而在解决方案中引入错误。为了克服这个问题，可以使用最小的旋转表示（例如，三维中的欧拉角）。然而，这是受奇点影响的。二维情况下的奇异点可以很容易地通过对角度进行归一化来恢复，但是在三维情况下这个过程并不简单。

> An alternative idea is to consider the underlying space as a manifold and to define an operator ⊞ that maps a local variation ∆x in the Euclidean space to a variation on the manifold, ∆x 7→ x ⊞ ∆x. We refer the reader to the work of Hertzberg [14] for the mathematical details. With this operator, a new error function can be defined as

另一种想法是将底层空间视为流形，并定义一个将欧几里德空间中的局部变差$\Delta {\rm x}$映射到流形上的变差$\Delta {\rm x} \mapsto {\rm x} \boxplus \Delta {\rm x}$的算符。我们请读者参阅Hertzberg的工作[14]中的数学细节。使用此运算符，新的错误函数可以定义为：
$$
\begin{align}
\breve{\rm e}_{ij}(\Delta \tilde{\rm x}_i, \Delta \tilde{\rm x}_j) 
&= {\rm e}_{ij}(\breve{\rm x}_i \boxplus \Delta \tilde{\rm x}_i, \breve{\rm x}_j \boxplus \Delta \tilde{\rm x}_j) \\
&= {\rm e}_{ij}(\breve{\rm x} \boxplus \Delta \tilde{\rm x}) \simeq \breve{\rm e}_{ij}+\tilde{\rm J}_{ij} \Delta \tilde{\rm x}
\end{align}
$$

> where x̆ spans over the original over-parametrized space, for instance quaternions. The term ∆x̃ is a small increment around the original position x̆ and is expressed in a minimal representation.

其中$\breve{\rm x}$跨越原始参数化空间，例如四元数。项$\Delta \tilde{\rm x}$是围绕原始位置$\breve{\rm x}$的一个小增量，用最小表示法表示。



123



## 5. PRACTICAL APPLICATIONS

> In this section we describe some applications of the proposed methods. In the first scenario we describe a complete 2D mapping system, and in the second scenario we briefly describe a 3D mapping system and we highlight the advantages of a manifold representation.

在这一节中，我们将描述所提出方法的一些应用。在第一个场景中，我们描述了一个完整的2D建图系统，在第二个场景中，我们简要描述了一个3D建图系统，并强调了流形表示的优点。

### A. 2D Laser Based Mapping

> The graph is constructed in the following way:
>
> - Whenever the robot moves more than 0.5 meters or rotates more than 0.5 radians, the algorithm adds a new vertex to the graph and labels it with the current laser observation.

图的构造方式如下： 

- 当机器人移动超过0.5米或旋转超过0.5弧度时，该算法会在图形中添加一个新的顶点，并用当前的激光观测标记它。 

> - This laser scan is matched with the previously acquired one to improve the odometry estimate and the corresponding edge is added to the graph. We use a variant of the scan-matcher described by Olson [26]. 

- 此激光扫描与先前获得的匹配以改进里程估计，并将相应的边缘添加到图中。我们使用Olson[26]描述的扫描匹配器的变体。

> - When the robot reenters a known area after traveling for a long time in a previously unknown region, the algorithm seeks for matches of the current scan with the past measurements (loop closing). If a matching between the current observation and the observation of another node succeeds, the algorithm adds a new edge to the graph. The edge is labeled with the relative transformation that makes the two scans to overlap best. Matching the current measurement with all previous scans would be extremely inefficient and error prone, since it does not consider the known prior about the robot location. Instead, the algorithm selects the candidate nodes in the past as the ones whose 3σ marginal covariances contains the current robot pose. These covariances can be obtained as the diagonal blocks of the inverse of a reduced Hessian H red . H red is obtained from H by removing rows and the columns of the newly inserted robot pose. H red is the information matrix of all the trajectory when assuming fixed the current position.

- 当机器人在一个先前未知的区域中移动很长时间后重新进入一个已知区域时，该算法寻找当前扫描与过去测量的匹配（环路闭合）。如果当前观测值和另一个节点的观测值之间的匹配成功，则该算法会向图中添加新边。用相对变换标记边缘，使两个扫描重叠得最好。将当前测量值与所有以前的扫描匹配将极为低效且容易出错，因为它不考虑关于机器人位置的已知先验信息。相反，该算法**选择过去的候选节点作为包含当前机器人姿态的$3\sigma$边缘协方差的候选节点**。这些协方差可以作为约化Hessian ${\rm H}_{red}$逆的对角块来获得。${\rm H}_{red}$是通过移除新插入的机器人姿势的行和列从${\rm H}$获得的。${\rm H}_{red}$是假定当前位置不变时所有轨迹的信息矩阵。

> - The algorithm performs the optimization whenever a loop closure is detected.

- 该算法在检测到环路闭合时执行优化。

> At the end of the run, the graph consists of 1, 802 nodes and 3, 546 edges. Even for this relatively large problem the optimization can be carried on in 100 ms on a standard laptop (Intel Core2@2.4 GHz). Since the robot travels at a velocity of around 1 m/s the graph optimization could be executed after adding every node instead of after detecting a loop closure. Figure 9 shows the effect of the optimization process on the trajectory, while Figure 10 illustrates the uncertainty ellipses. The robot is located in the region where the ellipse become small. Note that the poses in SE(2) do not need to be over parameterized, so in this case there is no advantage in utilizing manifolds.

在运行结束时，该图由802个节点和3546条边组成。即使对于这个相对较大的问题，也可以在标准笔记本电脑（英特尔Core2@2.4GHz）上在100毫秒内进行优化。由于机器人以大约1 m/s的速度移动，因此**可以在添加每个节点后执行图形优化，而不是在检测到环路闭合后执行**。图9显示了优化过程对轨迹的影响，而图10显示了不确定性椭圆。机器人位于椭圆变小的区域。请注意，**$SO(2)$中的姿势不需要过度参数化，因此在这种情况下，使用流形没有优势**。

### B. 3D Laser Based Mapping

