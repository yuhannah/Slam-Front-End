# slam中的数学

## 矩阵

### Jacob矩阵

- **定义**

  Suppose ${\bf{f}}:{\mathbb{R}}^n \mapsto {\mathbb{R}}^m $ is a function which takes as input the vector ${\bf{x}} \in {\mathbb{R}}^n$ and produces as output the vector ${\bf{f(x)}} \in {\mathbb{R}}^m$. Then the Jacobian matrix $\bf{J}$ of $\bf{f}$ is an $m \times n$ matrix, usually defined and arranged as follows:
  $$
  {\bf{J}}=
  \begin{bmatrix}
  \frac{\partial {\bf{f}}}{\partial x_1} & \dots & \frac{\partial {\bf{f}}}{\partial x_n}
  \end{bmatrix}=
  \begin{bmatrix}
  \frac{\partial f_1}{\partial x_1} & \dots & \frac{\partial f_1}{\partial x_n} \\
  \vdots & \ddots & \vdots \\
  \frac{\partial f_m}{\partial x_1} & \dots & \frac{\partial f_m}{\partial x_n}
  \end{bmatrix}
  $$
  or, component-wise:
  $$
  {\bf{J}}_{ij}=\frac{\partial f_i}{\partial x_j}
  $$

  <table><tr><td bgcolor=orange>雅可比矩阵：
      1) n维空间向m维空间的映射
      2) m关于n的一阶偏导数
      3) 得到的m行n列的雅可比矩阵</td></tr></table>

- 举例

  The Jacobian matrix of the function $\bf{f}:\mathbb{R}^\rm{3} \mapsto \mathbb{R}^\rm{2}$ with components:
  $$
  {\bf{f(x)}}=
  \begin{bmatrix}
  y_1 \\ y_2 \\ y_3 \\ y_4
  \end{bmatrix} = 
  \begin{bmatrix}
  x_1 \\ 5x_3 \\ 4x^2_2-2x_3 \\ x_3 \sin x_1
  \end{bmatrix}
  $$

  $$
  {\bf{J_f}}(x_1,x_2,x_3)=
  \begin{bmatrix}
  \frac{\partial {\bf{f}}}{\partial x_1} & \frac{\partial {\bf{f}}}{\partial x_2} & \frac{\partial {\bf{f}}}{\partial x_3}
  \end{bmatrix}=
  \begin{bmatrix}
  \frac{\partial y_1}{\partial x_1} & \frac{\partial y_1}{\partial x_2} & \frac{\partial y_1}{\partial x_3} \\
  \frac{\partial y_2}{\partial x_1} & \frac{\partial y_2}{\partial x_2} & \frac{\partial y_2}{\partial x_3} \\
  \frac{\partial y_3}{\partial x_1} & \frac{\partial y_3}{\partial x_2} & \frac{\partial y_3}{\partial x_3}
  \end{bmatrix}=
  \begin{bmatrix}
  1 & 0 & 0 \\ 0 & 0 & 5 \\ 0 & 8x_2 & -2 \\ x_3 \cos x_1 & 0 & \sin x_1
  \end{bmatrix}
  $$

  This example shows that the Jacobian need not be a square matrix.**不需要是正方形矩阵**

- **

