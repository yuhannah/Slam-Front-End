# ICP算法之理论

## 概述

**ICP(Iterative Closest Point)算法**：迭代最近点算法。通过最近邻确定两个点云的匹配情况，然后转换在同一坐标系下，并最小化误差的(或加权)平方和，求两个点云的相对变换T。如此反复迭代直至满足退出条件。



## 理论推导

参考文献：

> [1] Martínez, J. L., González, J., Morales, J., Mandow, A. and García-Cerezo, A. J. (2006), Mobile robot motion estimation by 2D scan matching with genetic and iterative closest point algorithms. Journal of Field Robotics, 23: 21–34. doi: 10.1002/rob.20104.

理论推导主要参考上述文献，添加详细推导过程。

首先假设不考虑雷达与机器的相对位姿，假设雷达获取的点云数据已经按照相对位姿转换到机器坐标系下。以下提及的点云序号与实际采集点云的顺序无关，仅代表对点云集合的遍历。

首先考虑两帧点云之间的相对位姿变换推导。局部点云与全局点云之间的绝对位姿变换道理上类似，作补充说明。

令第$k$时刻的机器坐标系为$XY_k$，对应获取的雷达点云在机器坐标系下的集合表示为${q_k}$，其中第$j$个点的坐标表示为$(x_k(j), y_k(j))$。当机器运动到第$k+1$时刻，对应获取的雷达点云在机器坐标系$XY_{k+1}$下的集合表示为${q_{k+1}}$。

记第$k$时刻到第$k+1$时刻，机器的位姿变化了$T_k(\Delta x, \Delta y, \Delta \phi)$。自定义任意世界坐标系$XY_o$，下面将这两帧雷达点云放在同一个坐标系中表示。

首先解释一下，“ICP匹配就是根据不同机器坐标系下的两帧点云之间的旋转和平移变换推测出机器的运动位姿变换的”。举个例子，当机器只进行平移运动时：

![ICP匹配示意图-1](ICP算法之理论.assets/ICP匹配示意图-1.png)

从$k$时刻到$k+1$时刻，环境是不变的矩形，机器从圆圈$k$沿机器的$X$轴正方向平移运动到圆圈$k+1$，世界坐标系为自定义的$XY_o$。机器的实际平移运动增量为$T_k$。当机器在$k$时刻时，观察到的周围环境点云用黄色点云表示，坐标系为机器坐标系$XY_k$，当机器在$k+1$时刻时，观察到的周围环境点云用绿色点云表示，坐标系为机器坐标系$XY_{k+1}$。

ICP算法就是找到绿色点云与黄色点云之间的匹配点对，将绿色点云进行平移和旋转，使绿色点云与黄色点云完全重合。此时绿色点云经历的平移和旋转过程，就代表了机器在这段时间内的平移和旋转运动。所以说，ICP匹配是在未知的情况下，将新一帧点云经过某种平移和旋转变换，转换到旧的点云坐标系中，使两个点云完全重叠，对应的平移和旋转变换就代表了机器的运动变化状态。

添加了旋转变换的复杂情况分析是一样的。

### 解析解的推导

继续上面的分析，当机器从$k$时刻到$k+1$时刻，假设位姿变换了$T_k(\Delta x, \Delta y, \Delta \phi)$，用示意图表示为：

![ICP匹配示意图-2](ICP算法之理论.assets/ICP匹配示意图-2.png)

第一步，将在机器坐标系$XY_{k+1}$下的某一点${q_{k+1}(j)}$转换到机器坐标系$XY_k$下，得到转换后的点$\hat{q}_{k}(j)$。初始的估计坐标变换来自于里程计$T_0(\Delta x_o,\Delta y_o,\Delta \phi_o)$：
$$
\begin{bmatrix}
\hat{x}_k(j) \\
\hat{y}_k(j)
\end{bmatrix} =
\begin{bmatrix}
\cos(-\Delta \phi) & \sin(-\Delta \phi) \\
-\sin(-\Delta \phi) & \cos(-\Delta \phi)
\end{bmatrix}
\begin{bmatrix}
x_{k+1}(j) \\
y_{k+1}(j)
\end{bmatrix}+
\begin{bmatrix}
\Delta x \\ \Delta y
\end{bmatrix} \\
=
\begin{bmatrix}
\cos(\Delta \phi) & -\sin(\Delta \phi) \\
\sin(\Delta \phi) & \cos(\Delta \phi)
\end{bmatrix}
\begin{bmatrix}
x_{k+1}(j) \\
y_{k+1}(j)
\end{bmatrix}+
\begin{bmatrix}
\Delta x \\ \Delta y
\end{bmatrix}
$$
注：$\Delta \phi$表示准时针转动。

第二步，遍历新一帧经过坐标转换后的点云$\{\hat{q}_k\}$，对坐标转换后的点云中的每个点$\hat{q}_k(j)$计算其与旧一帧的点云$\{q_k\}$中每个点之间的平方距离，记为$e(i,j)$：
$$
e(i,j)=(x_k(i)-\hat{x}_k(j))^2+(y_k(i)-\hat{y}_k(j))^2
$$
注：其中，$i\in \text{number of } \{q_k\}$，$j \in \text{number of }\{\hat{q}_k\} $。

第三步，对每个经过坐标转换后的点$\hat{q}_k(j)$，都有一帧数量的平方距离，取其中平方距离最小的对应点作为$\hat{q}_k(j)$的匹配点$q_k(J(j))$：
$$
J(j)=m, \text{if } e(m,j)=\min^N_{i=0}[e(i,j)]
$$
注：在经过坐标转换后的帧$\{\hat{q}_k\}$中，第$j$个点在旧一帧$\{q_k\}$中的对应点序号为$J(j)$。

那么，当前猜测位姿变换$T_k$下的匹配误差表示为：
$$
e_{T_k}(j)=e_{T_k}[\hat{q}_k(j),q_k(J(j))] \\
e_{T_k}(j)=(x_k(J(j))-\hat{x}_k(j))^2+(y_k(J(j))-\hat{y}_k(j))^2 \\
=(x_k(J(j))-x_{k+1}(j)\cos(\Delta \phi)+y_{k+1}(j)\sin(\Delta \phi)-\Delta x)^2 \\ 
+(y_k(J(j))-x_{k+1}(j)\sin(\Delta \phi)-y_{k+1}(j)\cos(\Delta \phi)-\Delta y)^2
$$
注：步骤2-3合在一起可以用KD-tree来加速计算，直接在当前猜测位姿变换下寻找当前帧在旧一帧中的最近点，并计算平方距离。

实际并不是所有的点都会参与计算，需要排除一部分外点，比如说在寻找最近点时，能找到的最近点距离很远，很可能是两个不相关的点，这样的点对不在后续计算范围内，容易导致匹配误差。外点通过设定阈值来去除：
$$
p_{T_k}(j)=
\begin{cases}
0 & \text{if } |e_{T_k}(j)| \geq E \\
1 & \text{otherwise}
\end{cases}
$$
注：这个筛选外点的系数为布尔变量，因此classic-ICP又称boolean-ICP。

最后剩余的有效点个数为：
$$
n_{T_k}=\sum^N_{p=0}p_{T_k}(j)
$$
有效点与总点云点数的比例为：
$$
P_{T_k}=\frac{n_{T_k}}{N+1}
$$
这个比例决定了两帧点云在经过了一个猜测的位姿变化转换后是否有重叠部分，重叠部分的比例又是多少。如果完全不重叠，那么这个猜测位姿变换就是不合理的。

**由于车辆运动引起的变形、伪距、随机噪声、地形不平度、混合像素、遮挡区域、离散角度分辨率、运动物体等诸多因素**，使得不同扫描点之间无法精确对应，因此可以认为扫描匹配作为一个确定一个二维转换的优化问题，它最小化了一个很好的匹配标准$I_{T_k}$。
$$
I_{T_k}=\frac{\sum^N_{j=0}[p_{T_k}(j)e_{T_j}(j)]}{n_{T_k}}\frac{1}{P_{T_k}}
$$
注：其中，前一个分数表示误差均值，后一个分数表示惩罚低的匹配率。

最后，当上式取得最小值时，对应的$T_k$就是当前两帧点云之间的位姿变换。对$I_{T_k}$求偏导：
$$
\frac{\partial I_{T_k}}{\partial \Delta x}
=0=\frac{1}{n_{T_k}P_{T_k}}\frac{\partial \sum^N_{j=0}[p_{T_k}(j)e_{T_j}(j)]}{\partial \Delta x} \\
=\frac{1}{n_{T_k}P_{T_k}} \sum^N_{j=0}[p_{T_k}(j) \frac{\partial e_{T_j}(j)}{\partial \Delta x}] \\
=\frac{1}{n_{T_k}P_{T_k}} \sum^N_{j=0}[p_{T_k}(j)(-2(x_k(J(j))-x_{k+1}(j)\cos(\Delta \phi)+y_{k+1}(j)\sin(\Delta \phi)-\Delta x))]


$$
省略不变的系数后，得到：
$$
\Delta x^{new} = \frac{1}{n_{T_k}} \sum^N_{j=0}[p_{T_k}(j) \Delta x] \\
= \frac{1}{n_{T_k}} \sum^N_{j=0}[p_{T_k}(j)(x_k(J(j))-x_{k+1}(j)\cos(\Delta \phi)+y_{k+1}(j)\sin(\Delta \phi))] \\
=\frac{1}{n_{T_k}} \left[ \sum^N_{j=0}[p_{T_k}(j)x_k(J(j))]- \\
\sum^N_{j=0}[p_{T_k}(j)x_{k+1}(j)]\cos(\Delta \phi)+ \\
\sum^N_{j=0}[p_{T_k}(j)y_{k+1}(j)]\sin(\Delta \phi) \right]
$$
同理对$\Delta y$求偏导得到：
$$
\frac{\partial I_{T_k}}{\partial \Delta y}
=0=\frac{1}{n_{T_k}P_{T_k}}\frac{\partial \sum^N_{j=0}[p_{T_k}(j)e_{T_j}(j)]}{\partial \Delta y} \\
=\frac{1}{n_{T_k}P_{T_k}} \sum^N_{j=0}[p_{T_k}(j) \frac{\partial e_{T_j}(j)}{\partial \Delta y}] \\
=\frac{1}{n_{T_k}P_{T_k}} \sum^N_{j=0}[p_{T_k}(j)(-2(y_k(J(j))-x_{k+1}(j)\sin(\Delta \phi)-y_{k+1}(j)\cos(\Delta \phi)-\Delta y))]
$$
简化后得到：
$$
\Delta y^{new} = \frac{1}{n_{T_k}} \sum^N_{j=0}[p_{T_k}(j) \Delta y] \\
= \frac{1}{n_{T_k}} \sum^N_{j=0}[p_{T_k}(j)(y_k(J(j))-x_{k+1}(j)\sin(\Delta \phi)-y_{k+1}(j)\cos(\Delta \phi))] \\
=\frac{1}{n_{T_k}} \left[ \sum^N_{j=0}[p_{T_k}(j)y_k(J(j))]- \\
\sum^N_{j=0}[p_{T_k}(j)x_{k+1}(j)]\sin(\Delta \phi)- \\
\sum^N_{j=0}[p_{T_k}(j)y_{k+1}(j)]\cos(\Delta \phi) \right]
$$
同理对$\Delta \phi$求偏导，得到：
$$
\frac{\partial I_{T_k}}{\partial \Delta \phi}
=0=\frac{1}{n_{T_k}P_{T_k}}\frac{\partial \sum^N_{j=0}[p_{T_k}(j)e_{T_j}(j)]}{\partial \Delta \phi} \\
=\frac{1}{n_{T_k}P_{T_k}} \sum^N_{j=0}[p_{T_k}(j) \frac{\partial e_{T_j}(j)}{\partial \Delta \phi}] \\
=\frac{1}{n_{T_k}P_{T_k}} \sum^N_{j=0}[p_{T_k}(j)(2(x_k(J(j))-x_{k+1}(j)\cos(\Delta \phi)+y_{k+1}(j)\sin(\Delta \phi)-\Delta x)(x_{k+1}(j)\sin(\Delta \phi)+y_{k+1}(j)\cos(\Delta \phi)) \\
+2(y_k(J(j))-x_{k+1}(j)\sin(\Delta \phi)-y_{k+1}(j)\cos(\Delta \phi)-\Delta y)(-x_{k+1}(j)\cos(\Delta \phi)+y_{k+1}(j)\sin(\Delta \phi)))] \\
=\frac{1}{n_{T_k}P_{T_k}} \sum^N_{j=0}\{p_{T_k}(j)[(x_k(J(j))x_{k+1}(j)+y_k(J(j))y_{k+1}(j)-x_{k+1}(j)\Delta x-y_{k+1}(j)\Delta y)\sin(\Delta \phi) \\
-(x_k(J(j))y_{k+1}(j)-y_k(J(j))x_{k+1}(j)-y_{k+1}(j)\Delta x+x_{k+1}(j)\Delta y)\cos(\Delta \phi)]\}
$$
继续简化：
$$
\Delta \phi^{new}=\arctan \left(\frac{\frac{1}{n_{T_k}} \sum^N_{j=0}[p_{T_k}(j)(x_k(J(j))y_{k+1}(j)-y_k(J(j))x_{k+1}(j)-y_{k+1}(j)\Delta x+x_{k+1}(j)\Delta y)]}{\frac{1}{n_{T_k}} \sum^N_{j=0}[p_{T_k}(j)(x_{k+1}(j)\Delta x+y_{k+1}(j)\Delta y-x_k(J(j))x_{k+1}(j)-y_k(J(j))y_{k+1}(j))]} \right)
$$
用下列式子代替上式中的部分，继续简化得到：
$$
S_{x_k}=\sum^N_{j=0}[p_{T_k}(j)x_k(J(j))]
$$

$$
S_{y_k}=\sum^N_{j=0}[p_{T_k}(j)y_k(J(j))]
$$

$$
S_{x_{k+1}}=\sum^N_{j=0}[p_{T_k}(j)x_{k+1}(J(j))]
$$

$$
S_{y_{k+1}}=\sum^N_{j=0}[p_{T_k}(j)y_{k+1}(J(j))]
$$

$$
S_{x_kx_{k+1}}=\sum^N_{j=0}[p_{T_k}(j)x_k(J(j))x_{k+1}(j)]
$$

$$
S_{y_kx_{k+1}}=\sum^N_{j=0}[p_{T_k}(j)y_k(J(j))x_{k+1}(j)]
$$

$$
S_{x_ky_{k+1}}=\sum^N_{j=0}[p_{T_k}(j)x_k(J(j))y_{k+1}(j)]
$$

$$
S_{y_ky_{k+1}}=\sum^N_{j=0}[p_{T_k}(j)y_k(J(j))y_{k+1}(j)]
$$

$$
\Delta x^{new}=\frac{S_{x_k}-S_{x_{k+1}}\cos(\Delta \phi)+S_{y_{k+1}}\sin(\Delta \phi)}{n_{T_k}}
$$

$$
\Delta y^{new}=\frac{S_{y_k}-S_{x_{k+1}}\sin(\Delta \phi)-S_{y_{k+1}}\cos(\Delta \phi)}{n_{T_k}}
$$

$$
\Delta \phi^{new}=\arctan \left(\frac{S_{x_ky_{k+1}}-S_{y_kx_{k+1}}-S_{y_{k+1}}\Delta x+S_{x_{k+1}}\Delta y}{S_{x_{k+1}}\Delta x+S_{y_{k+1}}\Delta y-S_{x_kx_{k+1}}-S_{y_ky_{k+1}}} \right)
$$

多元函数用偏导数求极值不一定能够解算出来。比如上述三个偏导函数分别包含另外的变量。此处将已经简化的$\Delta x^{new}$和$\Delta y^{new}$代入第三个偏导函数中进一步简化公式，得到：
$$
\Delta \phi^{new}=\arctan \left(\frac{n_{T_k}S_{x_ky_{k+1}}-n_{T_k}S_{y_kx_{k+1}}-S_{x_k}S_{y_{k+1}}+S_{x_{k+1}}S_{y_k}-(S_{x_{k+1}}^2+S_{y_{k+1}}^2)\sin(\Delta \phi)}{S_{x_k}S_{x_{k+1}}+S_{y_k}S_{y_{k+1}}-n_{T_k}S_{x_kx_{k+1}}-n_{T_k}S_{y_ky_{k+1}}-(S_{x_{k+1}}^2+S_{y_{k+1}}^2)\cos(\Delta \phi)} \right) \\
\approx \arctan \left(\frac{n_{T_k}S_{x_ky_{k+1}}-n_{T_k}S_{y_kx_{k+1}}-S_{x_k}S_{y_{k+1}}+S_{x_{k+1}}S_{y_k}}{S_{x_k}S_{x_{k+1}}+S_{y_k}S_{y_{k+1}}-n_{T_k}S_{x_kx_{k+1}}-n_{T_k}S_{y_ky_{k+1}}} \right)
$$
注：最后的近似舍去是从结论倒推出来的。

那么$(\Delta x^{new},\Delta y^{new},\Delta \phi^{new})$的解析解就是通过上述最后三个公式表达的。

对应在MRPT中的代码表示为：

```c++
bool  tfest::se2_l2(
	const TMatchingPairList & in_correspondences,
	TPose2D                 & out_transformation,
	CMatrixDouble33         * out_estimateCovariance )
{
	MRPT_START

	const size_t N = in_correspondences.size();

	if (N<2) return false;

	const float N_inv = 1.0f/N;  // For efficiency, keep this value.

	// ----------------------------------------------------------------------
	// Compute the estimated pose. Notation from the paper:
	// "Mobile robot motion estimation by 2d scan matching with genetic and iterative
	// closest point algorithms", J.L. Martinez Rodriguez, A.J. Gonzalez, J. Morales
	// Rodriguez, A. Mandow Andaluz, A. J. Garcia Cerezo,
	// Journal of Field Robotics, 2006.
	// ----------------------------------------------------------------------
	float SumXa=0, SumXb=0, SumYa=0, SumYb=0;
	float Sxx=0, Sxy=0, Syx=0, Syy=0;

	for (TMatchingPairList::const_iterator corrIt=in_correspondences.begin(); corrIt!=in_correspondences.end(); corrIt++)
	{
		// Get the pair of points in the correspondence:
		const float xa = corrIt->this_x;
		const float ya = corrIt->this_y;
		const float xb = corrIt->other_x;
		const float yb = corrIt->other_y;

		// Compute the terms:
		SumXa+=xa;
		SumYa+=ya;

		SumXb += xb;
		SumYb += yb;

		Sxx += xa * xb;
		Sxy += xa * yb;
		Syx += ya * xb;
		Syy += ya * yb;
	}	// End of "for all correspondences"...

	const float	mean_x_a = SumXa * N_inv;
	const float	mean_y_a = SumYa * N_inv;
	const float	mean_x_b = SumXb * N_inv;
	const float	mean_y_b = SumYb * N_inv;

	// Auxiliary variables Ax,Ay:
	const float Ax = N*(Sxx + Syy) - SumXa*SumXb - SumYa*SumYb;
	const float Ay = SumXa * SumYb + N*(Syx-Sxy)- SumXb * SumYa;

	out_transformation.phi = (Ax!=0 || Ay!=0) ? atan2( static_cast<double>(Ay), static_cast<double>(Ax)) : 0.0;

	const double ccos = cos( out_transformation.phi );
	const double csin = sin( out_transformation.phi );

	out_transformation.x = mean_x_a - mean_x_b * ccos + mean_y_b * csin;
	out_transformation.y = mean_y_a - mean_x_b * csin - mean_y_b * ccos;
    
	return true;

	MRPT_END
}
```

一个完整的ICP迭代过程，从最初的假设位姿$T_0(0,0,0)$或者由里程计提供一个初始假设位姿$T_0(\Delta x_0, \Delta y_0,\Delta \phi_0)$，进行一次ICP循环过程，经历上述四步，找到当前假设位姿下的配对点，并用解析公式求得算法认为的位姿增量$T_k(\Delta x_k,\Delta y_k,\Delta \phi_k)$。此时需要判断迭代是否收敛，位姿是否已经达到最优，配对点是否不再发生变化或者变化小于阈值等。如果没有收敛，ICP将用当前计算出的位姿增量$T_k$作为下次迭代的假设位姿，继续四步迭代计算。直到收敛。

**结论：**

- 解析解的推导如上述所示。
- ICP从一系列匹配点对是可以直接解算出当前的位姿变换的。并且==只要匹配点对没有发生变化，解算的结果是一样的==。这也是ICP迭代收敛的条件之一。
- ICP的迭代是没有上限的，需要人为定义收敛条件，比如说当两次配对点对不发生变化了，认为当前位姿已经是收敛的结果了。
- 当两个点云已经很接近重叠了，ICP获取的匹配点对变化也越来越小，整体的误差变化也越来越小，解算出的位姿增量变化也越来越慢。但是只要有一组配对点发生了变化，对ICP来说就没有达到收敛，所以收敛会越来越慢。
- 大旋转角度下，ICP只根据最近的点最为配对点，ICP找到的配对点并不代表真实情况下的配对点，所以这种最近关系对于旋转变化没有什么帮助；反而是当机器发生平移，根据最近点的配对关系，很容易将发生平移的点对配对成功。

最后补充说明，把新一帧点云投影到旧一帧点云坐标系中，得到机器运动位姿的变化量，同理，将新一帧点云投影到全局坐标系中，得到机器运动的绝对位姿。



### LM优化解的推导




