---

---

# ICP算法之理论

# ICP算法

**ICP原理**:迭代最近点算法,顾名思义,通过最近邻确定两个点云的匹配情况,然后转换在同一坐标系下,并最小化误差的(或加权)平方和求两个点云的相对变换T,如此反复迭代直至满足退出条件.



**数学描述**:已知参考点云集合**P**~ref~(在机器人局部坐标系下),对应的机器人位姿态为**T**~ref~,待匹配点云集合**P**~new~(在机器人局部坐标系下),对应的机器人位姿是**T**(未知),则需求解最优的**T**,使得两片点云的误差(或加权)平方和最小,这是一个最小二乘问题(理论上若匹配完全无误,则没有误差),给定一个初始的**T**值**T**~0~.则参考点云中任一点i在世界坐标下位置为
$$
\mathtt{S}_{ref}(i)=\mathtt{T}_{ref}*\mathtt{P}_{ref}(i)
$$
待匹配点云中任一点i在世界坐标下的位置
$$
\mathtt{S}_{new}(i)=\mathtt{T}*\mathtt{P}_{new}(i)
$$
假定两片点云的匹配对应关系为c(i),即待匹配点云中第i个点对应这参考点云中的第c(i)个点

则误差平方和:
$$
J=\sum_{i=1}^N\parallel\mathtt{S}_{ref}(c(i))-\mathtt{S}_{new}(i)\parallel^2=\sum_{i=1}^N\parallel\mathtt{T}_{ref}*\mathtt{P}_{ref}(i)-\mathtt{T}*\mathtt{P}_{new}(i)\parallel^2
$$
已知**T**~ref~,**P**~ref~,**P**~new~及**T**~0~,求解**T**使得:
$$
\mathtt{T}=minJ
$$
**ICP算法步骤** 如下:

1.根据**T**~0~,确认两片点云的匹配关系c(i)

2.求解**T**使得误差平方和最小

3.判断**T**是否收敛,若不收敛则令**T**~0~**=**T**,并转到第一步,若收敛则返回**T**

第一步中根据最近邻求匹配关系c(i),这里给出其伪代码,复杂度为O(N^2),用kd-tree数据结构可加速(详见kd-tree)

```pseudocode
for i = 1 : N
	d_min = 无穷大;
	for k = 1 : N
		d_ik=Euclidean(S_new(i),S_ref(k))
		if d_ik < d_min
			d_min = d_ik
			c(i) = k
		end if
	end for
end for
```



第二步中求解均方误差和函数的方法有两种解法:

a.**封闭解析解**(直接求导数)/SVD分解

b.转换为求解**非线性优化**问题,进行**迭代求解**(高斯牛顿,LM,等等)

一般针对2D点中的ICP,因为**T**只有三个独立变量(x,y,theta) ,可采用直接对J求偏导并令其等于0 ,可得解析解,针对3D点中的ICP,**T**有6维,不易求解出偏导数,可采用方法b.

下面针对2D点的ICP匹配给出其解析解推导过程:

对J求偏导数得:
$$
\begin{cases}
\frac{\partial{J}}{\partial{t_x}}=t_x-\overline{X}_{ref}+\overline{X}_{new}*cos\theta-\overline{Y}_{new}*sin\theta\\
\frac{\partial{J}}{\partial{t_y}}=t_y-\overline{Y}_{ref}+\overline{X}_{new}*sin\theta+\overline{Y}_{new}*cos\theta\\
\frac{\partial{J}}{\partial{\theta}}=\theta-atan2\frac{-A_1+t_x*\overline{Y}_{new}-t_y*\overline{X}_{new}}{A_2-t_x*\overline{X}_{new}-t_y*\overline{Y}_{new}}


\end{cases}
$$
令偏导数为0可得:
$$
\begin{cases}
t_x=\overline{X}_{ref}-\overline{X}_{new}*cos\theta+\overline{Y}_{new}*sin\theta\\
t_y=\overline{Y}_{ref}-\overline{X}_{new}*sin\theta-\overline{Y}_{new}*cos\theta\\
\theta=atan2\frac{-A_1+t_x*\overline{Y}_{new}-t_y*\overline{X}_{new}}{A_2-t_x*\overline{X}_{new}-t_y*\overline{Y}_{new}}
\end{cases}
$$
其中:
$$
\mathtt{T}=\begin{bmatrix}cos\theta&sin\theta&t_x\\
						sin\theta&-cos\theta&t_y\\
						0&0&1\end{bmatrix}\\
\mathtt{S}_{ref}(i)=(\mathtt{X}_{ref}(i),\mathtt{Y}_{ref}(i)^{T}\\
\mathtt{S}_{new}(i)=(\mathtt{X}_{new}(i),\mathtt{Y}_{new}(i))^{T}\\
\overline{X}_{ref}=\frac{1}{N}\sum_{i=1}^{N}\mathtt{X_{ref}(c(i))}\\
\overline{Y}_{ref}=\frac{1}{N}\sum_{i=1}^{N}\mathtt{Y_{ref}(c(i)}\\
\overline{X}_{new}=\frac{1}{N}\sum_{i=1}^{N}\mathtt{X_{new}(i)}\\
\overline{Y}_{new}=\frac{1}{N}\sum_{i=1}^{N}\mathtt{Y_{new}(i)}\\
A_1=\frac{1}{N}(\sum_{i=1}^{N}\mathtt{X}_{ref}(c(i))*\mathtt{Y}_{new}(i)-\sum_{i=1}^{N}\mathtt{Y}_{ref}(c(i))*\mathtt{X}_{new}(i))\\
A_2=\frac{1}{N}(\sum_{i=1}^{N}\mathtt{X}_{ref}(c(i))*\mathtt{X}_{new}(i)+\sum_{i=1}^{N}\mathtt{Y}_{ref}(c(i))*\mathtt{Y}_{new}(i))\\
$$
给定一个初始值,固定theta求tx,ty,然后固定tx,ty求theta,逐步迭代直至得到一组收敛的tx,ty,theta

ICP优化策略及变种

Boollean weighting estimation

设置距离阈值E,丢弃超过阈值的匹配点对:
$$
p_T(i)=\begin{cases}0\ \ \ \ \ \ \ \ \ J_b(i)\gt=E\\
					1\ \ \ \ \ \ \ \ \ otherwise\end{cases}\\
					J_b(i)=(\mathtt{X}_{ref}(c(i))-\mathtt{X}_{new}(i))^2+(\mathtt{Y}_{ref}(c(i))-\mathtt{Y}_{new}(i))^2
$$
记匹配点对数为:
$$
n_T=\sum_{i=1}^{N}p_T(i)
$$
匹配率为:
$$
P_T=\frac{n_T}{N}
$$
构造新的加权后误差项:
$$
I_T=\frac{\sum_{i=1}^{N}p_T(i)*J_b(i)}{n_T*P_T}
$$
对其求偏导数可得:













