# g2o: A General Framework for Graph Optimization

## 1. INTRODUCTION

> All these problems can be represented as a graph. Whereas each node of the graph represents a state variable to optimize, each edge between two variables represents a pairwise observation of the two nodes it connects. In the literature, many approaches have been proposed to address this class of problems. A naive implementation using standard methods like Gauss-Newton, Levenberg-Marquardt (LM), Gauss-Seidel relaxation, or variants of gradient descent typically provides acceptable results for most applications. However, to achieve the maximum performance substantial efforts and domain knowledge are required.

所有这些问题都可以用图来表示。**图中的每个节点表示要优化的状态变量，而两个变量之间的每条边表示它连接的两个节点的成对观察。**在文献中，已经提出了许多方法来解决这类问题。使用标准方法（如Gauss-Newton、Levenberg-Marquardt（LM）、Gauss-Seidel松弛或梯度下降变量）的简单实现通常为大多数应用提供可接受的结果。然而，要达到最大的性能，需要大量的努力和领域知识。

> In this paper, we describe a general framework for performing the optimization of nonlinear least squares problems that can be represented as a graph. We call this framework g2o (for “general graph optimization”). Figure 1 gives an overview of the variety of problems that can be solved by using g2o as an optimization back-end. The proposed system achieves a performance that is comparable with implementations of state-of-the-art algorithms, while being able to accept general forms of nonlinear measurements. We achieve efficiency by utilizing algorithms that

本文描述了一个求解非线性最小二乘问题的一般框架，该框架可以表示为一个图。我们称此框架为${\rm g^2o}$（用于“一般图优化”）。图1概述了使用${\rm g^2o}$作为优化后端可以解决的各种问题。该系统在能够接受一般形式的非线性测量的同时，获得了与最新算法实现相当的性能。我们通过使用

> - exploit the sparse connectivity of the graph,
> - take advantage of the special structures of the graph that often occur in the problems mentioned above,
> - use advanced methods to solve sparse linear systems,
> - and utilize the features of modern processors like SIMD instructions and optimize the cache usage.

- 利用图的稀疏连通性，
- 利用在上述问题中经常出现的图的特殊结构，
- 使用先进的方法来解决稀疏线性系统，
- 并利用现代处理器的特点，如SIMD指令和优化缓存使用。

## 2. RELATED WORK

> However, they assume that the covariance is roughly spherical and thus have difficulties in optimizing pose-graphs where some constraints have covariances with null spaces or substantial differences in the eigenvalues.

然而，他们**假设协方差大致是球形的**，因此在优化位姿图时有困难，其中一些约束具有空空间协方差或特征值存在实质性差异。

> Graph optimization can be viewed as a nonlinear least-squares problem, which typically is solved by forming a linear system around the current state, solving, and iterating. One promising technique for solving the linear system is preconditioned conjugate gradient (PCG), which was used by Konolige [17] as well as Montemerlo and Thrun [20] as an efficient solver for large sparse pose constraint systems. Because of its high efficiency on certain problems, g 2 o includes an implementation of a sparse PCG solver which applies a block-Jacobi pre-conditioner [13].

**图优化问题可以看作是一个非线性最小二乘问题，它通常是通过围绕当前状态形成一个线性系统，求解和迭代来求解的。**求解线性系统的一个有前途的技术是预条件共轭梯度（PCG），Konolige[17]和Montemerlo及Thrun[20]将其作为大型稀疏位姿约束系统的有效解算器。由于其在某些问题上的高效性，**${\rm g^2o}$包含了一个应用块Jacobi预处理的稀疏PCG解算器的实现[13]**。

> However, the latter approach is restricted to 2D pose graphs. In g 2 o we share similar ideas with these systems. Our system can be applied to both SLAM and BA optimization problems in all their variants, e.g., 2D SLAM with landmarks, BA using a monocular camera, or BA using stereo vision. However, g 2 o showed a substantially improved performance compared these systems on all the data we used for evaluation purposes.

然而，后一种方法仅限于2D位姿图。在${\rm g^2o}$中，我们与这些系统有相似的想法。我们的系统可以应用于SLAM和BA优化问题的所有变体，例如，具有地标的2D SLAM、使用单目相机的BA或使用立体视觉的BA。然而，与我们用于评估的所有数据相比，${\rm g^2o}$显示出了显著的性能改进。

> In computer vision, Sparse Bundle Adjustment [27] is a nonlinear least-squares method that takes advantage of the sparsity of the Jacobian pattern between points and camera poses. Very recently, there have been several systems [15], [13] that advance similar concepts of sparse linear solvers and efficient calculation of the Schur reduction (see Section III-D) for large systems (∼100M sparse matrix elements). There are also new systems based on nonlinear conjugate gradient that never form the linear system explicitly [1], [2]; these converge more slowly, but can work with extremely large datasets (∼1000M matrix elements). In this paper we compare g 2 o to the SSBA system of [15], which is the best-performing publicly available system to date.

在计算机视觉中，稀疏束平差[27]是一种非线性最小二乘法，它利用了点和相机姿态之间雅可比模式的稀疏性。最近，有几个系统[15]，[13]提出了稀疏线性解算器的类似概念，并对大型系统（\~100M稀疏矩阵元素）的Schur约化进行了有效计算（见第III-D节）。还有一些基于非线性共轭梯度的新系统，它们从不显式地形成线性系统[1]，[2]；这些系统收敛速度较慢，但可以处理非常大的数据集（\~1000M矩阵元素）。在本文中，我们将${\rm g^2o}$与[15]的SSBA系统进行了比较，后者是迄今为止性能最好的公共可用系统。

## 3. NONLINEAR GRAPH OPTIMIZATION USING LEAST-SQUARES

机器人学或计算机视觉中的许多问题都可以通过找到这种形式的函数的最小值来解决：
$$
{\rm F}({\rm x})=\sum_{\langle i,j \rangle \in {\cal C}} \underbrace{{\rm e}({\rm x}_i,{\rm x}_j,{\rm z}_{ij})^T {\rm \Omega}_{ij} {\rm e}({\rm x}_i,{\rm x}_j,{\rm z}_{ij})}_{{\rm F}_{ij}}
$$

$$
{\rm x}^*=\arg \min_{\rm x} {\rm F}({\rm x})
$$

其中，${\rm x}=({\rm x}_1^T, \ldots, {\rm x}_n^T)^T$是参数向量。每个${\rm x}_i$表示一个泛型参数块。

${\rm z}_{ij}$和${\rm \Omega}_{ij}$分别表示与参数${\rm x}_j$和${\rm x}_i$相关的约束的平均值和信息矩阵。

${\rm e}({\rm x}_i,{\rm x}_j,{\rm z}_{ij})$是向量误差函数，用于测量参数块${\rm x}_i$和${\rm x}_j$满足约束${\rm z}_{ij}$的程度。

> Note that each error function, each parameter block, and each error function can span a different space. A problem in this form can be effectively represented by a directed graph. A node i of the graph represents the parameter block x i and an edge between the nodes i and j represents an ordered constraint between the two parameter blocks x i and x j . Figure 2 shows an example of mapping between a graph and an objective function.

请注意，每个误差函数、每个参数块和每个误差函数可以跨越不同的空间。这种形式的问题可以用有向图来有效地表示。图的节点i表示参数块${\rm x}_i$，节点i和j之间的边表示两个参数块${\rm x}_i$和${\rm x}_j$之间的有序约束。图2显示了图和目标函数之间的映射示例。

### A. Least Squares Optimization

略

### B. Alternative Parameterizations

> The procedures described above are general approaches to multivariate function minimization. They assume that the space of parameters x is Euclidean, which is not valid for several problems like SLAM or bundle adjustment. To deal with state variables that span over non-Euclidean space, a common approach is to express the increments ∆x i in a space different from the one of the parameters x i .

上述步骤是多元函数最小化的一般方法。他们**假设参数${\rm x}$的空间是欧氏空间，这对于SLAM或束平差等问题是无效的**。为了处理跨越非欧几里德空间的状态变量，一种常见的方法是在不同于其中一个参数${\rm x}_i$的空间中表示增量$\Delta {\rm x}_i$。

### C. Structure of the Linearized System

略。

### D. Systems Having Special Structure

略。

## 4. IMPLEMENTATION

> Figure 3 depicts the design of our system. Only the boxes in gray need to be defined to address a new optimization problem. Using the provided base class, deriving a new type of node only requires defining the ⊞ operator for applying the increments. An edge connecting two nodes x i and x j requires the definition of the error function e ij (·). The Jacobian J ij is then evaluated numerically, or, for higher efficiency, the user can specify J ij explicitly by overwriting the virtual base-class function. Thus, implementing new types for addressing a new optimization problem or comparing different parameterizations is a matter of writing a few lines of code.

图3描述了系统的设计。只需要定义灰色框就可以解决新的优化问题。使用提供的基类，派生新类型的节点只需要定义用于应用增量的运算符。**连接两个节点${\rm x}_i$和${\rm x}_j$的边需要定义误差函数${\rm e}_{ij}(\cdot)$。然后对Jacobian ${\rm J}_{ij}$进行数值计算，或者为了提高效率，用户可以通过重写虚拟基类函数显式地指定${\rm J}_{ij}$。**因此，实现用于解决新优化问题或比较不同参数化的新类型需要编写几行代码。

> Special care has been taken in implementing matrix multiplications required for the Schur reduction in Eq. (25). The sparse structure of the underlying graph is exploited to only multiply non-zero entries required to form the extra entries of H pp . Additionally, we operate on the block structures of the underlying matrix (see [15]), which results in a cache efficient matrix multiplication compared to a scalar matrix multiplication.

在实施公式（25）中Schur缩减所需的矩阵乘法时，已特别小心。利用底层图的稀疏结构，只需乘以构成${\rm H}_{\rm pp}$的额外项所需的非零项。此外，我们对底层矩阵的块结构进行操作（参见[15]），这将导致与标量矩阵乘法相比，具有高速缓存效率的矩阵乘法。

> Our framework is agnostic with respect to the embedded linear solver, so we can apply appropriate ones for different problems. We have used two solvers for experiments. Since H is positive semi-definite and symmetric, sparse Cholesky decomposition results in a an efficient solver [4], [3]. Note that the non-zero pattern during the least-squares iterations is constant. We therefore are able to reuse a symbolic decomposition computed within the first iteration, which results in a reduced fill-in and reduces the overall computation time in subsequent iterations. Note that this Cholesky decomposition does not take advantage of the block structure of the parameters. The second method is Preconditioned Conjugate Gradient (PCG) with a block Jacobi pre-conditioner [13], which takes advantage of block matrix operations throughout. As PCG itself is an iterative method, solving a linear system requires n iterations for a n×n matrix. Since carrying out n iterations of PCG is typically slower than Cholesky decomposition, we limit the number of iterations based on the relative decrease in the squared residual of PCG. By this we are able to quantify the loss in the accuracy of the solution introduced by terminating PCG early. In the experiments we will compare the different solvers.

我们的框架对于嵌入的线性解算器是不可知的，因此我们可以针对不同的问题应用合适的框架。我们用了两个解算器做实验。由于${\rm H}$是半正定对称的，稀疏Cholesky分解可以得到一个有效的解[4]，[3]。注意，最小二乘迭代期间的非零模式是恒定的。因此，我们能够重用在第一次迭代中计算出的符号分解，从而减少填充，并在随后的迭代中减少总体计算时间。注意，Cholesky分解没有利用参数的块结构。第二种方法是预处理共轭梯度（PCG）和块Jacobi预处理[13]，它充分利用了块矩阵运算。由于PCG本身是一种迭代方法，求解一个线性系统需要对n×n矩阵进行n次迭代。由于PCG的n次迭代通常比Cholesky分解慢，因此我们基于PCG平方残差的相对减少来限制迭代次数。通过这一点，我们能够量化通过提前终止PCG而引入的解决方案的精度损失。在实验中我们将比较不同的解算器。

## 5. EXPERIMENTS

