# Efficient Sparse Pose Adjustment for 2D Mapping

## 1. INTRODUCTION

> At the heart of the LM method lies the solution of a large sparse linear problem. In this paper, we develop a method to efficiently compute the sparse matrix from the constraint graph, and use direct sparse linear methods to solve it. In analogy to Sparse Bundle Adjustment in the vision literature, we call this method Sparse Pose Adjustment (SPA), since it deals with the restricted case of pose-pose constraints. The combination of an SBA/GraphSLAM optimizer with efficient methods for solving the linear subproblem has the following advantages.

LM方法的核心是一个大型稀疏线性问题的求解。本文提出了一种从约束图中有效计算稀疏矩阵的方法，并用直接稀疏线性方法求解。与视觉文献中的稀疏束平差(SBA)方法类似，我们将此方法称为稀疏位姿平差（SPA），因为它处理位姿约束的受限情况。SBA/GraphSLAM优化器与求解线性子问题的有效方法相结合具有以下优点。

> - It takes the covariance information in the constraints into account which leads to more accurate solutions.
> - SPA is robust and tolerant to initialization, with very low failure rates (getting stuck in local minima) for both incremental and batch processing.
> - Convergence is very fast as it requires only a few iterations of the LM method.
> - Unlike EKF and information filters, SPA is fully non-linear: at every iteration, it linearizes all constraints around their current pose.
> - SPA is efficient in both batch and incremental mode.

- 它考虑了约束条件中的协方差信息，从而得到更精确的解。
- SPA具有很强的鲁棒性和对初始化的耐受性，无论是增量处理还是批处理，故障率都非常低（陷入局部极小值）。
- 收敛速度非常快，因为它只需要LM方法的几次迭代。
- 与EKF和信息滤波器不同，SPA是完全非线性的：在每次迭代中，它都将当前姿态周围的所有约束线性化。
- SPA在批量和增量模式下都很有效。

> One of the benefits of the efficiency of SPA is that a mapping system can continuously optimize its graph, providing the best global estimate of all nodes, with very little computational overhead. Solving the optimization problem for the large map shown in Figure 1 requires only 150 ms from an initial configuration provided by odometry. In the incremental mode, where the graph is optimized after each node is added, it requires less than 15 ms for any node addition.

SPA效率的一个好处是，建图系统可以连续优化其图，提供所有节点的最佳全局估计，而计算开销非常小。解决图1中所示的大型地图的优化问题只需要150毫秒，而初始配置由Odometry提供。在增量模式中，在添加每个节点后对图形进行优化，任何节点添加所需的时间都小于15毫秒。

> Although SPA can be parameterized with 3D poses, for this paper we have restricted it to 2D mapping, which is a well-developed field with several competing optimization techniques. Our intent is to show that a 2D pose-based mapping system can operate on-line using SPA as its optimization engine, even in large-scale environments and with large loop closures, without resorting to submaps or complicated partitioning schemes.

虽然SPA可以用3D位姿参数化，但本文将其局限于2D建图，这是一个发展很好的领域，具有多种相互竞争的优化技术。我们的目的是证明一个基于2D位姿的建图系统可以使用SPA作为其优化引擎在线运行，即使在大规模环境和大循环闭合的情况下，也不需要借助子图或复杂的分区方案。

## 2. RELATED WORK

> More recently, Dellaert and colleagues use bundle adjustment, which they implement using sparse direct linear solvers [3]; they call their system √ SAM [4]). Our approach is similar to √SAM; we differ from their approach mostly in engineering, by efficient construction of the linear subproblem using ordered data structures. We also use LM instead of a standard nonlinear least-square method, thereby increasing robustness. Finally, we introduce a “continuable LM” method for the incremental case, and an initialization method that is a much more robust approach to the batch problem.

最近，Dellaert和同事使用bundle adjustment，他们使用**稀疏直接线性解算器**来实现 [3]；他们称他们的系统为√SAM [4]。我们的方法类似于√SAM；我们不同于他们的方法主要是在工程上，通过使用**有序的数据结构有效地构造线性子问题**。我们也使用LM代替标准的非线性最小二乘法，从而提高了鲁棒性。最后，我们介绍了一种增量情形下的“连续LM”方法，以及一种更稳健的批量问题的初始化方法。

> Kaess et al. [14] introduced a variant of SAM, called iSAM, that performs incremental update of the linear matrix associated with the nonlinear least-squares problem. Relinearization and variable ordering are performed only occasionally, thereby increasing computational efficiency. In our approach, relinearization and matrix construction are very efficient, so such methods become less necessary. Currently we do not have an implementation of either iSAM or √SAM to test against for performance.

Kaess等人 [14] 引入了√SAM的一个变体，称为iSAM，它执行与非线性最小二乘问题相关的线性矩阵的增量更新。重线性化和变量排序只是偶尔执行的，因此提高了计算效率。在我们的方法中，重线性化和矩阵构造是非常有效的，因此这样的方法变得不那么必要。目前我们没有iSAM或√SAM的实现来测试性能。

> Relaxation or least-squares approaches proceed by iteratively refining an initial guess. Conversely, approaches based on stochastic gradient descent are more robust to the initial guess. In the SLAM literature the importance of this initial guess has been often underestimated. The better the initial guess is, the more likely it is for an algorithm to find the correct solution. In this paper, we address this point and evaluate three different strategies for computing the initial guess.

松弛法或最小二乘法通过迭代优化初始猜测来进行。相反，基于随机梯度下降的方法对初始猜测更具鲁棒性。在SLAM文学中，这种最初猜测的重要性常常被低估。初始猜测越好，算法就越有可能找到正确的解。在本文中，我们将讨论这一点，并评估计算初始猜测的三种不同策略。

> In contrast to full nonlinear optimization, several researchers have explored filtering techniques to solve the graphs incrementally, using an information matrix form. The first such approach was proposed by Eustice et al. and denoted Delayed Sparse Information Filter (DSIF) [7]. This technique can be very efficient, because it adds only a small constant number of elements to the system information matrix, even for loop closures. However, recovering the global pose of all nodes requires solving a large sparse linear system; there are faster ways of getting approximate recent poses.

与完全非线性优化相比，一些研究者已经探索了使用信息矩阵形式的过滤技术来增量地求解图。第一种方法是由Eustice等人提出的。并表示延迟稀疏信息滤波器（DSIF）[7]。这种技术非常有效，因为它只在系统信息矩阵中添加少量常量元素，即使对于循环闭包也是如此。然而，恢复所有节点的全局姿态需要求解一个大型稀疏线性系统；有更快的方法获得近似的最近姿态。

> To summarize the paper: we propose an efficient approach for optimizing 2D pose graphs that uses direct sparse Cholesky decomposition to solve the linear system. The linear system is computed in a memory-efficient way that minimizes cache misses and thus significantly improves the performance. We compare our method, in both accuracy and speed, to existing LM and non-LM approaches that are avaiable, and show that SPA outperforms them. Open source implementations are available both in C++ and in matlab/octave.

总结全文：提出了一种利用直接稀疏Cholesky分解求解线性系统的2D位姿图优化方法。线性系统是以一种节省内存的方式计算的，这种方式可以最小化缓存未命中，从而显著提高性能。我们在准确性和速度上比较了我们的方法与现有的LM和非LM方法，并表明SPA优于他们。开源实现在C++和Matlab/Octave中都是可用的。 

> Efficient direct (non-iterative) algorithms to solve sparse systems have become available [3], thus reviving a series of approaches for optimizing the graphs which have been discarded in the past. 

解决稀疏系统的有效直接（非迭代）算法已经变得可用[3]，从而恢复了一系列过去被丢弃的优化图的方法。 

## 3. SYSTEM FORMULATION

> Popular approaches to solve the SLAM problem are the so-called “graph-based” or “network-based” methods. The idea is to represent the history of the robot measurements by a graph. Every node of the graph represents a sensor measurement or a local map and it is labeled with the location at which the measurement was taken. An edge between two nodes encodes the spatial information arising from the alignment of the connected measurements and can be regarded as a spatial constraint between the two nodes.

解决SLAM问题的流行方法是所谓的“基于图”或“基于网络”的方法。其思想是用图形来表示机器人测量的历史。图中的每个节点代表一个传感器测量或一个局部地图，并用测量的位置标记。两个节点之间的边编码了由连续测量的对齐产生的空间信息，可以看作是两个节点之间的空间约束。

> In the context of graph-based SLAM, one typically considers two different problems. The first one is to identify the constraints based on sensor data. This so-called data association problem is typically hard due to potential ambiguities or symmetries in the environment. A solution to this problem is often referred to as the SLAM front-end and it directly deals with the sensor data. The second problem is to correct the poses of the robot to obtain a consistent map of the environment given the constraints. This part of the approach is often referred to as the optimizer or the SLAM back-end. Its task is to seek for a configuration of the nodes that maximizes the likelihood of the measurements encoded in the constraints. An alternative view to this problem is given by the spring-mass model in physics. In this view, the nodes are regarded as masses and the constraints as springs connected to the masses. The minimal energy configuration of the springs and masses describes a solution to the mapping problem.

在基于图的SLAM环境中，通常考虑两个不同的问题。第一种是基于传感器数据识别约束。由于环境中潜在的模糊性或对称性，这种所谓的数据关联问题通常很难解决。这个问题的解决方案通常被称为SLAM前端，它直接处理传感器数据。第二个问题是修正机器人的姿态，以获得给定约束条件下的一致环境地图。这部分方法通常被称为优化器或SLAM后端。它的任务是寻找最大化约束中编码的测量可能性的节点配置。物理中的弹簧-质量模型给出了这个问题的另一种观点。在该视图中，节点被视为质量，约束被视为连接到质量的弹簧。弹簧和质量的最小能量配置描述了映射问题的解决方案。

> During its operation a graph-based SLAM system interleaves the execution of the front-end and of the back-end, as shown in Figure 2. This is required because the front-end needs to operate on a partially optimized map to restrict the search about potential constraints. The more accurate the current estimate is, the more robust the constraints generated by the front-end will be and the faster its operation. Accordingly, the performance of the optimization algorithm, measured in terms of accuracy of the estimate and execution time, has a major impact on the overall mapping system.

在运行过程中，基于图的SLAM系统交叉执行前端和后端，如图2所示。这是必需的，因为前端需要在部分优化的映射上操作，以限制对潜在约束的搜索。当前估计值越精确，前端生成的约束就越健壮，运算速度也越快。因此，优化算法的性能，从估计精度和执行时间的角度来衡量，对整个映射系统有着重大的影响。

> In this paper we describe in detail an efficient and compact optimization approach that operates on 2D graphs. Our algorithm can be coupled with arbitrary front-ends that handle different kinds of sensors. For clarity of presentation we shortly describe a front-end for laser data. However, the general concepts can be straightforwardly applied to different sensors.

本文详细地描述了一种高效、紧凑的2D图优化方法。我们的算法可以与处理不同类型传感器的任意前端耦合。为清晰起见，我们简要介绍了激光数据的前端。然而，一般的概念可以直接应用于不同的传感器。

## 4. SPARSE POSE ADJUSTMENT

> To optimize a set of poses and constraints, we use the well-known Levenberg-Marquardt (LM) method as a framework, with particular implementations that make it efficient for the sparse systems encountered in 2D map building. In analogy to the Sparse Bundle Adjustment of computer vision, which is a similarly efficient implementation of LM for cameras and features, we call our system Sparse Pose Adjustment (SPA).

为了优化一组姿势和约束，我们使用了著名的Levenberg-Marquardt（LM）方法作为框架，通过特定的实现，使其能够有效地用于2D地图构建中遇到的稀疏系统。类似于计算机视觉的稀疏束调整，这是一种对相机和特征同样有效的LM实现，我们称之为稀疏位姿调整（SPA）。

### A. Error Formulation

系统的变量是机器人的全局姿态${\rm c}$的集合，由平移和角度参数化：$c_i=[t_i,\theta_i]=[x_i,y_i,\theta_i]^T$。约束是从另一个节点$c_i$的位置测量一个节点$c_j$。在$c_i$的坐标系中，$c_i$和$c_j$之间的测量偏移量为$\bar{z}_{ij}$，精度矩阵为$\Lambda_{ij}$（协方差的逆$\Sigma^{-1}$）。对于$c_i$和$c_j$的任何实际姿态，其偏移量可以计算为：
$$
h(c_i,c_j) \equiv 
\left\{
\begin{array}{}
\, R^T_i(t_j-t_i) \\ \, \theta_j-\theta_i
\end{array}
\right.
$$
这里，$R_i$是$\theta_i$的2x2旋转矩阵。
$$
R_i(\theta_i)=
\begin{bmatrix}
\cos(\theta_i) & -\sin(\theta_i) \\ \sin(\theta_i) & \cos(\theta_i)
\end{bmatrix}
$$
$h(c_i,c_j)$被称为测量方程。是3x1矩阵。

与约束关联的误差函数和总误差为：
$$
\begin{split}
e_{ij} & \equiv \bar{z}_{ij}-h(c_i,c_j) \\
\chi^2({\rm c},{\rm p}) & \equiv \sum\limits_{ij} e^T_{ij}\Lambda_{ij}e_{ij}
\end{split}
$$
注意，$h(c_i,c_j)$中的角度参数不是唯一的，因为加上或减去2π会得到相同的结果。当角度差出现时，它们总是被标准化为区间（－π，π]。$e_{ij}$是3x1矩阵，$e^T_{ij} \Lambda_{ij}e_{ij}$是一个实数。

### B. Linear System

> The optimal placement of c is found by minimizing the total error in Equation 2. A standard method for solving this problem is Levenberg-Marquardt (LM), iterating a linearized solution around the current values of c. The linear system is formed by stacking the variables c into a vector x, and the error functions into a vector e. Then we define:

通过最小化方程2中的总误差，找到${\rm c}$的最优位置。解决这个问题的一种标准方法是Levenberg Marquardt（LM），它将线性化的解迭代到${\rm c}$的当前值上。线性系统是通过将变量${\rm c}$叠加到向量${\rm x}$中，将误差函数叠加到向量${\rm e}$中而形成的。然后我们定义：
$$
\begin{split}
\Lambda & \equiv \begin{bmatrix}
\Lambda_{ab} & & \\ & \ddots & \\ & & \Lambda_{mn}
\end{bmatrix} \\
{\rm J} & \equiv \frac{\partial {\rm e}}{\partial {\rm x}} \\
{\rm H} & \equiv {\rm J}^T \Lambda {\rm J}
\end{split}
$$
其中，每个$\Lambda_{ij}$块都是3x3的矩阵，每个$e_{ij}$都是3x1的矩阵，每个$x_i=c_i$都是3x1的矩阵，那么每个$J_i=\frac{\partial e_{ij}}{\partial c_i}$都是3x3的矩阵，每个$H_{ii}=J^T_i\Lambda_{ij}J_i$都是3x3的矩阵。

LM系统是：
$$
({\rm H}+\lambda diag {\rm H})\Delta {\rm x}={\rm J}^T \Lambda {\rm e}
$$

> Here λ is a small positive multiplier that transitions between gradient descent and Newton-Euler methods. Gradient descent is more robust and less likely to get stuck in local minima, but converges slowly; Newton-Euler has the opposite behavior.

这里的λ是一个小的正乘子，它在梯度下降法和牛顿-欧拉法之间过渡。梯度下降更稳健，不易陷入局部极小值，但收敛速度慢；牛顿-欧拉有相反的行为。

矩阵${\rm H}$是通过为每个测量$h(c_i,c_j)$添加四个分量而形成的：
$$
\begin{matrix}
\ddots & & & & \\ & J^T_i \Lambda_{ij}J_i & \cdots & J^T_i \Lambda_{ij}J_j & \\ & \vdots & \ddots & \vdots & \\ & J^T_j \Lambda_{ij}J_i & \cdots & J^T_j \Lambda_{ij}J_j & \\ & & & & \ddots
\end{matrix}
$$
只要测量量不同，四个分量就不会与过去的分量重合，都是相互独立的。

这里我们稍微滥用了$J$的符号，$J_i$是$e_{ij}$关于变量$c_i$的Jacobian，$J_i=\frac{\partial e_{ij}}{\partial c_i}$，是3x3的矩阵；同理，$J_j$是$e_{ij}$关于变量$c_j$的Jacobian，$J_j=\frac{\partial e_{ij}}{\partial c_j}$，是3x3的矩阵。每个分量都是3x3的矩阵块。右侧是通过为每个约束添加3x1的矩阵块$J_{c_i}\Lambda_{ij}e_{ij}$和$J_{c_j}\Lambda_{ij}e_{ij}$而形成的。其中，$J_{c_i}$是3x3的矩阵，$\Lambda_{ij}$是3x3的矩阵，$e_{ij}$是3x1的矩阵。

解线性方程得到一个增量$\Delta x$，该增量可按如下方式加到${\rm x}$的当前值中：
$$
\begin{split}
t_i & = t_i + \Delta t_i \\
\theta_i & = \theta_i + \Delta \theta_i
\end{split}
$$

### C. Error Jacobians

测量函数$h$的雅可比出现在正规方程（4）中，我们在这里列出它们：
$$
\begin{split}
c_i & = 
\begin{bmatrix}(t_i)_{2 \times 1} \\ \theta_i \end{bmatrix}_{3 \times 1} = 
\begin{bmatrix} x_i \\ y_i \\ \theta_i \end{bmatrix}_{3 \times 1} \\
R_i(\theta_i) & = \begin{bmatrix}
\cos(\theta_i) & -\sin(\theta_i) \\ \sin(\theta_i) & \cos(\theta_i)
\end{bmatrix}_{2 \times 2} \\
h(c_i,c_j) & \equiv 
\begin{bmatrix} [R^T_i(t_j-t_i)]_{2 \times 1} \\ \theta_j-\theta_i \end{bmatrix}_{3 \times 1} \\
e_{ij} & = \bar{z}_{ij}-h(c_i,c_j) \\
J_i & = \frac{\partial e_{ij}}{\partial c_i} = 
\begin{bmatrix} 
\left(\frac{\partial e_{ij}}{\partial t_i}\right)_{3 \times 2} & \left(\frac{\partial e_{ij}}{\partial \theta_i}\right)_{3 \times 1} \end{bmatrix}_{3 \times 3} \\
& = \begin{bmatrix} 
\left(\frac{\partial [-h(c_i,c_j)]}{\partial t_i}\right)_{3 \times 2} & \left(\frac{\partial [-h(c_i,c_j)]}{\partial \theta_i}\right)_{3 \times 1} \end{bmatrix}_{3 \times 3} \\
& = \begin{bmatrix} 
\left(\frac{-\partial R^T_i(t_j-t_i)}{\partial t_i}\right)_{2 \times 2} & \left(\frac{-\partial R^T_i(t_j-t_i)}{\partial \theta_i}\right)_{2 \times 1}\\
\left(\frac{-\partial (\theta_j-\theta_i)}{\partial t_i} \right)_{1 \times 2} & \left(\frac{-\partial (\theta_j-\theta_i)}{\partial \theta_i} \right)_{1 \times 1}
\end{bmatrix}_{3 \times 3}  \\
& = \begin{bmatrix}
(R^T_i)_{2 \times 2} & \left(-\frac{\partial R^T_i}{\partial \theta_i}(t_j-t_i)\right)_{2 \times 1}\\
(0 \; 0)_{1 \times 2} & (1)_{1 \times 1}
\end{bmatrix}_{3 \times 3} 
\end{split}
$$
==**注意，个人推导结果符号异于论文！！！**==

同理，对$c_j$求偏导，注意$R_i(\theta_i)$是$\theta_i$的旋转矩阵，是没有$\theta_j$分量的，求偏导结果就是$[0 \; 0]^T$：
$$
\begin{split}
J_j & = \frac{\partial e_{ij}}{\partial c_j} = 
\begin{bmatrix} 
\left(\frac{\partial e_{ij}}{\partial t_j}\right)_{3 \times 2} & \left(\frac{\partial e_{ij}}{\partial \theta_j}\right)_{3 \times 1} \end{bmatrix}_{3 \times 3} \\
& = \begin{bmatrix} 
\left(\frac{\partial [-h(c_i,c_j)]}{\partial t_j}\right)_{3 \times 2} & \left(\frac{\partial [-h(c_i,c_j)]}{\partial \theta_j}\right)_{3 \times 1} \end{bmatrix}_{3 \times 3} \\
& = \begin{bmatrix} 
\left(\frac{-\partial R^T_i(t_j-t_i)}{\partial t_j}\right)_{2 \times 2} & \left(\frac{-\partial R^T_i(t_j-t_i)}{\partial \theta_j}\right)_{2 \times 1}\\
\left(\frac{-\partial (\theta_j-\theta_i)}{\partial t_j} \right)_{1 \times 2} & \left(\frac{-\partial (\theta_j-\theta_i)}{\partial \theta_j} \right)_{1 \times 1}
\end{bmatrix}_{3 \times 3}  \\
& = \begin{bmatrix}
(-R^T_i)_{2 \times 2} & (0 \; 0)^T_{2 \times 1}\\
(0 \; 0)_{1 \times 2} & (-1)_{1 \times 1}
\end{bmatrix}_{3 \times 3} 
\end{split}
$$

### D. Sparsity

> We are interested in large systems, where the number of poses ||c|| can be 10k or more (the largest real-world indoor dataset we have been able to find is about 3k poses, but we can generate synthetic datasets of any order). The number of system variables is 3||c||, and the H matrix is ||c|| 2 , or over 10 8 elements. Manipulating such large matrices is expensive. Fortunately, for typical scenarios the number of constraints grows only linearly with the number of poses, so that H is very sparse. We can take advantage of the sparsity to solve the linear problem more efficiently.

我们对大型系统感兴趣，其中位姿$\|{\rm c}\|$的数量可以是10K或更多（我们能够找到的最大的真实室内数据集大约是3K 个位姿，但我们可以生成任意顺序的合成数据集）。系统变量的个数为$3\|{\rm c}\|$，${\rm H}$矩阵为$\|{\rm c}\|^2$，或超过$10^8$个元素。操作如此大的矩阵是昂贵的。幸运的是，对于典型场景，约束的数量只随着姿势的数量线性增长，因此${\rm H}$非常稀疏。利用稀疏性可以更有效地求解线性问题。

> For solving (4) in sparse format, we use the CSparse package [3]. This package has a highly-optimized Cholesky decomposition solver for sparse linear systems. It employs several strategies to decompose H efficiently, including a logical ordering and an approximate minimal degree (AMD) algorithm to reorder variables when H is large.

对于稀疏格式的（4）求解，我们使用CSparse包[3]。这个软件包有一个用于稀疏线性系统的高度优化的Cholesky分解求解器。它采用了多种策略来有效地分解${\rm H}$，包括逻辑排序和在${\rm H}$较大时对变量重新排序的近似最小度（AMD）算法。

> In general the complexity of decomposition will be O(n 3 ) in the number of variables. For sparse matrices, the complexity will depend on the density of the Cholesky factor, which in turn depends on the structure of H and the order of its variables. Mahon et al. [19] have analyzed the behavior of the Cholesky factorization as a function of the loop closures in the SLAM system. If the number of loop closures is constant, then the Cholesky factor density is O(n), and decomposition is O(n). If the number of loop closures grows linearly with the number of variables, then the Cholesky factor density grows as O(n 2 ) and decomposition is O(n 3 ).

一般情况下，分解的复杂度在变量数上为$O(n^3)$）。对于稀疏矩阵，复杂性将取决于Cholesky因子的密度，而Cholesky因子又取决于${\rm H}$的结构及其变量的顺序。Mahon等人[19]分析了Cholesky分解作为SLAM系统中闭环函数的行为。如果环闭数是常数，则Cholesky因子密度为$O(n)$，分解为$O(n)$。如果环闭的数目与变量的数目成线性增长，那么Cholesky因子密度就以$O(n^2)$的形式增长，分解为$O(n^3)$。

### E. Compressed Column Storage

> Each iteration of the LM algorithm has three steps: setting up the linear system, decomposing H, and finding ∆x by back-substitution. Setting up the system is linear in the number of constraints (and hence in the number of variables for most graph-based SLAM systems). In many situations it can be the more costly part of the linear solver. Here we outline an efficient method for setting up the sparse matrix form of H from the constraints generated by Equation (5).

LM算法的每一次迭代都有三个步骤：建立线性系统，分解${\rm H}$，并通过反代换求$\Delta x$。系统的建立在约束的数量上是线性的（因此对于大多数基于图的SLAM系统来说，变量的数量也是线性的）。在许多情况下，它可能是线性解算器中成本更高的部分。本文从方程（5）的约束条件出发，提出了一种建立${\rm H}$的稀疏矩阵形式的有效方法。

> CSparse uses compressed column storage (CCS) format for sparse matrices. The figure below shows the basic idea.

CSparse对稀疏矩阵使用**压缩列存储（CCS）**格式。下图显示了基本思想。(略)

> Each nonzero entry in the array is placed in the val vector. Entries are ordered by column first, and then by row within the column. col ptr has one entry for each column, plus a last entry which is the number of total nonzeros (nnz). The col ptr entry for a column points to the start of the column in the row ind and val variables. Finally, row ind gives the row index of each entry within a column.

数组中的每个非零项都放在val向量中。条目首先按列排序，然后按列中的行排序。col_ptr每列有一个条目，加上最后一个条目，即非零总数（nnz）。列的col_ptr项指向row_ind和val变量中列的开头。最后，row_ind给出列中每个条目的行索引。

> CCS format is storage-efficient, but is difficult to create incrementally, since each new nonzero addition to a column causes a shift in all subsequent entries. The most efficient way would be to create the sparse matrix in column-wise order, which would require cycling through the constraints ||c|| times. Instead, we go through the constraints just once, and store each 3x3 block J i ⊤ Λ ij J i in a special block-oriented data structure that parallels the CCS format. The algorithm is given in Table I. In this algorithm, we make a pass through the constraints to store the 3x3 block matrices into C++ std::map data structures, one for each column. Maps are efficient at ordered insertion based on their keys, which is the row index. Once this data structure is created (step (2)), we use the ordered nature of the maps to create the sparse CCS format of H by looping over each map in the order of its keys, first to create the column and row indices, and then to put in the values. The reason for separating the column/row creation from value insertion is because the former only has to be done once for any set of iterations of LM.

**CCS格式是一种存储效率高的格式，但很难以增量方式创建，因为每向列中添加一个新的非零项都会导致所有后续条目发生移位。**最有效的方法是按列顺序创建稀疏矩阵，这需要循环约束$\|{\rm c}\|$次。相反，我们只经过一次约束，并将每个3x3 块$J^T_i\Lambda_{ij}J_i$存储在一个与CCS格式并行的特殊面向块的数据结构中。该算法在表I中给出，在该算法中，我们通过约束将3x3块矩阵存储到C++ std::map数据结构，每个列一个。映射在基于keys（行索引）的有序插入方面非常有效。一旦创建了这个数据结构（步骤（2）），我们就使用映射的有序性来创建${\rm H}$的稀疏CCS格式，方法是按键的顺序循环每个映射，首先创建列和行索引，然后放入值。将列/行创建与值插入分开的原因是，对于LM的任何一组迭代，前者只需执行一次。

> Note that only the upper triangular elements of H are stored, since the Cholesky solver in CSparse only looks at this part, and assumes the matrix is symmetric.

注意，只存储${\rm H}$的上三角元素，因为CSparse中的Cholesky解算器只查看此部分，并且假设矩阵是对称的。

### F. Continuable LM System

> The LM system algorithm is detailed in Table II. It does one step in the LM algorithm, for a set of nodes c with associated measurements. Running a single iteration allows for incremental operation of LM, so that more nodes can be added between iterations. The algorithm is continuable in that λ is saved between iterations, so that successive iterations can change λ based on their results. The idea is that adding a few nodes and measurements doesn’t change the system that much, so the value of λ has information about the state of gradient descent vs. Euler-Newton methods. When a loop closure occurs, the system can have trouble finding a good minima, and λ will tend to rise over the next few iterations to start the system down a good path.

LM系统算法详见表二。它在LM算法中为一组节点${\rm c}$执行一个步骤，并进行相关测量。运行单个迭代允许LM的增量操作，以便在迭代之间添加更多节点。该算法是连续的，因为在迭代之间保存了λ，因此连续的迭代可以根据结果改变λ。其思想是增加几个节点和测量值并不会对系统造成太大的改变，因此λ的值与Euler-Newton方法相比具有梯度下降状态的信息。当发生循环闭合时，系统可能无法找到一个好的最小值，并且在接下来的几次迭代中，λ将倾向于上升，以使系统沿着一条好的路径启动。

> There are many different ways of adjusting λ; we choose a simple one. The system starts with a small lambda, 10 −4 . If the updated system has a lower error than the original, λ is halved. If the error is the same or larger, λ is doubled. This works quite well in the case of incremental optimization. As long as the error decreases when adding nodes, λ decreases and the system stays in the Newton-Euler region. When a link is added that causes a large distortion that does not get corrected, λ can rise and the system goes back to the more robust gradient descent.

调节λ的方法有很多种，我们选择一种简单的方法。系统从一个$10^{-4}$的小lambda开始。如果更新后的系统误差小于原始系统，则将λ减半。如果误差相同或更大，则λ加倍。这在增量优化的情况下非常有效。**只要增加节点时误差减小，λ减小，系统保持在牛顿-欧拉区域。当一个链接被添加，导致一个大的失真没有得到纠正，λ可以上升，系统回到更稳健的梯度下降。**

## 5. SCAN MATCHING

> SPA requires precision (inverse covariance) estimates from matching of laser scans (or other sensors). Several scan-match algorithms can provide this, for example, Gutmann et al. [11] use point matches to lines extracted in the reference scan, and return a Gaussian estimate of error. More recently, the correlation method of Konolige and Chou [17], extended by Olson [22], provides an efficient method for finding the globally best match within a given range, while returning an accurate covariance. The method allows either a single scan or set of aligned scans to be matched against another single scan or set of aligned scans. This method is used in the SRI’s mapping system Karto 1 for both local matching of sequential scans, and loop-closure matching of sets of scans as in [12]. To generate the real-world datasets for experiments, we ran Karto on 63 stored robot logs of various sizes, using its scan-matching and optimizer to build a map and generate constraints, including loop closures. The graphs were saved and used as input to all methods in the experiments.

SPA需要通过激光扫描（或其他传感器）的匹配进行精确（协方差的逆）估计。一些扫描匹配算法可以提供这一点，例如，Gutmann等人[11]使用点匹配到参考扫描中提取的线，并返回误差的高斯估计值。最近，由Olson[22]扩展的Konolige和Chou[17]的相关方法提供了一种在给定范围内寻找全局最佳匹配，同时返回精确协方差的有效方法。该方法允许一次扫描或一组对齐扫描与另一次扫描或一组对齐扫描匹配。该方法用于SRI的Karto 1映射系统，用于序列扫描的局部匹配和扫描集的循环闭合匹配，如[12]。为了生成用于实验的真实数据集，我们对63个不同大小的存储机器人日志运行Karto，使用它的扫描匹配和优化器构建映射并生成约束，包括循环闭包。实验中所有方法均以图形作为输入。

## 6. EXPERIMENTS

