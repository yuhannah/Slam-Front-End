# Karto SLAM代码解析

> K. Konolige, G. Grisetti, R. K¨ummerle, B. Limketkai, R. Vincent, Efficient Sparse Pose Adjustment for 2D Mapping, In Proc. of Int. Conf. on Intelligent Robots and Systems (IROS), Oct. 2010.

## 1. 概况

特征：

- Multi-Search
- Scan-Match：扫描匹配，点云与局部相关地图之间的匹配
- Loop-Closure：闭环检测
- SPA/G2O：图优化算法
- Pose-Map：位姿图，需要根据激光数据和校正后的位姿另外建图

![图优化框架](Karto SLAM代码解析.assets/图优化框架.png)

上图是对Karto SLAM的整体框架的介绍。从2016年SLAM的大综述中可以看到，这其实就是一般SLAM的一个基本框架，这里指的是带优化的框架。作者2010的论文正是图优化刚开始涌入至SLAM的体现。

Karto SLAM基于图优化的思想，用高度优化和非迭代 Cholesky分解进行稀疏系统解耦作为解。图优化方法利用图的均值表示地图，每个节点表示机器人轨迹的一个位置点和传感器测量数据集，每个新节点加入，就会进行计算更新。

Karto SLAM的ROS版本，其中采用的稀疏点调整（the Spare Pose Adjustment(SPA)）与扫描匹配和闭环检测相关。**Landmark越多，内存需求越大**，然而==图优化方式相比其他方法在大环境下制图优势更大，因为他仅包含点的图(robot pose)，得到优化后的位姿后再建立地图Map==。

![Karto整体框架](Karto SLAM代码解析.assets/Karto整体框架.png)

## 2. 基础类解析

代码中所有的配置参数都以封装成类，调用的时候取值`GetValue()`。

### 向量/位姿/矩阵/变换

#### Vector2

表示二维实空间中的向量`(x，y)`。 实例`double/int`。

- 构造函数

  ```c++
  Vector2() // 默认构造(0,0)
  {
      m_Values[0] = 0;
      m_Values[1] = 0;
  }
  
  Vector2(T x, T y) // 自定义构造(x,y)
  {
      m_Values[0] = x;
      m_Values[1] = y;
  }
  ```

- 成员变量

  ```c++
  T m_Values[2]; // 一维数组
  ```

- 成员函数

  ```c++
  inline void operator += (const Vector2& rOther)
  {
      m_Values[0] += rOther.m_Values[0];
      m_Values[1] += rOther.m_Values[1];
  }
  
  inline void operator -= (const Vector2& rOther)
  {
      m_Values[0] -= rOther.m_Values[0];
      m_Values[1] -= rOther.m_Values[1];
  }
  
  inline const Vector2 operator + (const Vector2& rOther) const
  {
      return Vector2(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1]);
  }
  
  inline const Vector2 operator - (const Vector2& rOther) const
  {
      return Vector2(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1]);
  }
  ```

  所有的运算按照相同的坐标轴相加减来实现。注意对`Pose2`类的影响。

#### Pose2

表示二维空间的位置`(x，y)`和朝向。其中，表示位置的`Vector2<T>`实例化为`Vector2<kt_double>`。朝向是额外添加的实数。因此需要注意`Pose2`类的计算方法。

- 构造函数

  ```c++
  Pose2()
      : m_Heading(0.0) // 默认构造(0.0,0.0,0.0)
      {
      }
  
  Pose2(const Vector2<kt_double>& rPosition, kt_double heading)
      : m_Position(rPosition)
          , m_Heading(heading)
      {
      }
  
  Pose2(kt_double x, kt_double y, kt_double heading)
      : m_Position(x, y)
          , m_Heading(heading)
      {
      }
  
  Pose2(const Pose3& rPose); // 包含四元数到欧拉角的转换
  
  Pose2(const Pose2& rOther)
      : m_Position(rOther.m_Position)
          , m_Heading(rOther.m_Heading)
      {
      }
  ```

- 成员变量

  ```c++
  Vector2<kt_double> m_Position; // 二维空间的位置坐标(x,y)
  kt_double m_Heading; // 二维空间的位置坐标的朝向
  ```

- 成员函数

  注意`Vector2`的加减法针对每个坐标系单独计算，此处`Pose2`的加减法也是针对位置和角度单独计算的，没有旋转过程。

  ```c++
  inline void operator += (const Pose2& rOther)
  {
      m_Position += rOther.m_Position;
      m_Heading = math::NormalizeAngle(m_Heading + rOther.m_Heading);
  }
  
  inline Pose2 operator + (const Pose2& rOther) const
  {
      return Pose2(m_Position + rOther.m_Position, math::NormalizeAngle(m_Heading + rOther.m_Heading));
  }
  
  inline Pose2 operator - (const Pose2& rOther) const
  {
      return Pose2(m_Position - rOther.m_Position, math::NormalizeAngle(m_Heading - rOther.m_Heading));
  }
  ```

#### Matrix3

定义3x3的矩阵类。

- 构造函数

  ```c++
  Matrix3()
  {
      Clear();
  }
  
  inline Matrix3(const Matrix3& rOther)
  {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
  }
  ```

- 成员变量

  ```c++
  kt_double m_Matrix[3][3]; // 二维数组
  ```

- 成员函数

  ```c++
  // 3x3矩阵相乘 [3x3 * 3x3 = 3x3]
  Matrix3 operator * (const Matrix3& rOther) const
  {
      Matrix3 product;
      for (size_t row = 0; row < 3; row++)
      {
          for (size_t col = 0; col < 3; col++)
          {
              product.m_Matrix[row][col] = 
                  m_Matrix[row][0]*rOther.m_Matrix[0][col] +
                  m_Matrix[row][1]*rOther.m_Matrix[1][col] +
                  m_Matrix[row][2]*rOther.m_Matrix[2][col];
          }
      }
  
      return product;
  }
  
  // 3x3矩阵与3x1位姿Pose2相乘 - matrix * pose [3x3 * 3x1 = 3x1]
  inline Pose2 operator * (const Pose2& rPose2) const
  {
      Pose2 pose2;
  
      pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] *
                 rPose2.GetY() + m_Matrix[0][2] * rPose2.GetHeading());
      pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] *
                 rPose2.GetY() + m_Matrix[1][2] * rPose2.GetHeading());
      pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() + m_Matrix[2][1] *
                       rPose2.GetY() + m_Matrix[2][2] * rPose2.GetHeading());
  
      return pose2;
  }
  
  // 3x3矩阵相加
  inline void operator += (const Matrix3& rkMatrix)
  {
      for (kt_int32u row = 0; row < 3; row++)
      {
          for (kt_int32u col = 0; col < 3; col++)
          {
              m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
          }
      }
  }
  
  // 获得绕(x,y,z)轴旋转radians角度的旋转矩阵
  // (1,0,0,theta)->[1 0 0;0 cos(theta) -sin(theta);0 sin(theta) cos(theta)]
  // (0,1,0,theta)->[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)]
  // (0,0,1,theta)->[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1]
  void FromAxisAngle(kt_double x, kt_double y, kt_double z, const kt_double radians)
  {
      kt_double cosRadians = cos(radians);
      kt_double sinRadians = sin(radians);
      kt_double oneMinusCos = 1.0 - cosRadians;
  
      kt_double xx = x * x;
      kt_double yy = y * y;
      kt_double zz = z * z;
  
      kt_double xyMCos = x * y * oneMinusCos;
      kt_double xzMCos = x * z * oneMinusCos;
      kt_double yzMCos = y * z * oneMinusCos;
  
      kt_double xSin = x * sinRadians;
      kt_double ySin = y * sinRadians;
      kt_double zSin = z * sinRadians;
  
      m_Matrix[0][0] = xx * oneMinusCos + cosRadians;
      m_Matrix[0][1] = xyMCos - zSin;
      m_Matrix[0][2] = xzMCos + ySin;
  
      m_Matrix[1][0] = xyMCos + zSin;
      m_Matrix[1][1] = yy * oneMinusCos + cosRadians;
      m_Matrix[1][2] = yzMCos - xSin;
  
      m_Matrix[2][0] = xzMCos - ySin;
      m_Matrix[2][1] = yzMCos + xSin;
      m_Matrix[2][2] = zz * oneMinusCos + cosRadians;
  }
  ```

#### Transform

`Pose2`转换的实现。

- 构造函数

  ```c++
  // 
  Transform(const Pose2& rPose)
  {
      SetTransform(Pose2(), rPose);
  }
  
  Transform(const Pose2& rPose1, const Pose2& rPose2)
  {
      SetTransform(rPose1, rPose2);
  }
  ```

- 成员变量

  ```c++
  Pose2 m_Transform; // 3x1变换位姿
  Matrix3 m_Rotation;// 3x3旋转矩阵
  Matrix3 m_InverseRotation;// 3x3旋转矩阵的逆
  ```

- 成员函数

  ```c++
  // 将此设置为从第一个位姿到第二个位姿的转换
  void SetTransform(const Pose2& rPose1, const Pose2& rPose2)
  {
      if (rPose1 == rPose2) // 两个位姿相同，则旋转矩阵为单位矩阵，变换位姿为(0,0,0)
      {
          m_Rotation.SetToIdentity();
          m_InverseRotation.SetToIdentity();
          m_Transform = Pose2();
          return;
      }
  
      // heading transformation
      m_Rotation.FromAxisAngle(0, 0, 1, rPose2.GetHeading() - rPose1.GetHeading());
      m_InverseRotation.FromAxisAngle(0, 0, 1, rPose1.GetHeading() - rPose2.GetHeading());
  
      // position transformation
      Pose2 newPosition;
      if (rPose1.GetX() != 0.0 || rPose1.GetY() != 0.0)
      {
          newPosition = rPose2 - m_Rotation * rPose1;
      }
      else
      {
          newPosition = rPose2;
      }
  
      m_Transform = Pose2(newPosition.GetPosition(), rPose2.GetHeading() - rPose1.GetHeading());
  }
  ```

  `SetTransform()`接口实现了计算从`pose1`到`pose2`的变换位姿`transform`。位姿中位置的变换和朝向的变换是分开计算的。以下按照XY平面进行说明。

  旋转矩阵`m_Rotation `由旋转轴`(0,0,1)`和旋转角度`(heading2-heading1)`产生。将`pose1`中的位置向量逆时针旋转`(heading2-heading1)`角度。但是不改变旋转后的位姿的朝向，即`heading=heading1`。旋转后的位姿表示为`R*pose1`。

  ```c++
  m_Rotation = [cos(theta) -sin(theta) 0;
                sin(theta) cos(theta)  0;
                0          0           1];
  m_Rotation * pose1 = [cos(theta) -sin(theta) 0;   [pose1.x;
                        sin(theta) cos(theta)  0; *  pose1.y;
                        0          0           1]    heading1];
                     = [pose1.x*cos(theta)-pose1.y*sin(theta);
                        pose1.x*sin(theta)+pose1.y*cos(theta);
                        heading1];
  ```

  由于`Pose2`类的加减运算中，会将角度归一化，在计算变换位姿时是不需要归一化角度的，因此对朝向的计算会单独使用`(heading2-heading1)`角度。注意区分不同。

  ```c++
  pose2 - m_Rotation * pose1 = 
      [pose2.x;    [pose1.x*cos(theta)-pose1.y*sin(theta);
       pose2.y;  -  pose1.x*sin(theta)+pose1.y*cos(theta);
       heading2]    heading1];
  = [pose2.x-pose1.x*cos(theta)-pose1.y*sin(theta);
     pose2.y-pose1.x*sin(theta)+pose1.y*cos(theta);
     math::NormalizeAngle(heading2-heading1)];
  m_Transform = 
      [pose2.x-pose1.x*cos(theta)-pose1.y*sin(theta);
       pose2.y-pose1.x*sin(theta)+pose1.y*cos(theta);
       heading2-heading1];
  ```

  变换位姿`m_Transform`的位置分量最终由`pose2`与旋转后的`R*pose1`之差的位置分量决定，朝向分量最终由`pose2`和`pose1`的朝向分量之差决定。

  ![transform-1](Karto SLAM代码解析.assets/transform-1.png)

  ```c++
  // 将输入位姿pose1按照变换位姿transform变换后得到变换后的位姿
  inline Pose2 TransformPose(const Pose2& rSourcePose)
  {
      Pose2 newPosition = m_Transform + m_Rotation * rSourcePose;
      kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() + m_Transform.GetHeading());
  
      return Pose2(newPosition.GetPosition(), angle);
  }
  
  // 将输入位姿pose2按照变换位姿transform逆变换后得到变换前的位姿
  inline Pose2 InverseTransformPose(const Pose2& rSourcePose)
  {
      Pose2 newPosition = m_InverseRotation * (rSourcePose - m_Transform);
      kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() - m_Transform.GetHeading());
  
      // components of transform
      return Pose2(newPosition.GetPosition(), angle);
  }
  ```

### 顶点/边

#### Vertex

Karto SLAM中，顶点类模板`Vertex<T>`的实例是`Vertex<LocalizedRangeScan>`，指的是包含机器位姿等信息的一帧激光数据。

- 构造和析构函数：

  ```c++
  Vertex(T* pObject)
      : m_pObject(pObject)
  {
  }
  
  virtual ~Vertex()
  {
  }
  ```

- 成员变量：

  ```c++
  T* m_pObject; // 指激光数据指针LocalizedRangeScan*
  std::vector<Edge<T>*> m_Edges; // 该顶点的边的列表
  ```

- 成员函数

  ```c++
  inline const std::vector<Edge<T>*>& GetEdges() const // 获取该顶点的边
  inline T* GetObject() const // 获取该顶点的类型
  std::vector<Vertex<T>*> GetAdjacentVertices() const // 获取该顶点的边的另一个顶点的列表
  inline void AddEdge(Edge<T>* pEdge) // 添加边到该顶点的边的列表
  ```

  ![vertex-1](Karto SLAM代码解析.assets/vertex-1.png)

该顶点（激光数据）将会添加与该顶点（激光数据）相连的边`edge`的信息。从而根据相连的边的信息得到相邻的其他顶点（激光数据）的信息。

#### Edge

Karto SLAM中，边类模板`Edge<T>`的实例是`Edge<LocalizedRangeScan>`，指的是包含机器位姿等信息的两帧激光数据、激光数据的传感器位姿以及两帧之间的传感器位姿差和协方差矩阵。

- 构造和析构函数：

  ```c++
  Edge(Vertex<T>* pSource, Vertex<T>* pTarget)
      : m_pSource(pSource) // 边的起点顶点
          , m_pTarget(pTarget) // 边的终点顶点
          , m_pLabel(NULL) // 边的起点顶点和终点顶点的传感器位姿以及位姿差和协方差矩阵等信息
      {
          m_pSource->AddEdge(this); // 将边添加到起点顶点的边的列表中
          m_pTarget->AddEdge(this); // 将边添加到终点顶点的边的列表中
      }
  
  virtual ~Edge()
  {
      m_pSource = NULL;
      m_pTarget = NULL;
  
      if (m_pLabel != NULL)
      {
          delete m_pLabel;
          m_pLabel = NULL;
      }
  }
  ```

  **边在构造时需要两个顶点，并将构造的边分别添加到两个顶点的边的列表中。**

- 成员变量：

  ```c++
  Vertex<T>* m_pSource; // 边的起点顶点（激光数据）
  Vertex<T>* m_pTarget; // 边的终点顶点（激光数据）
  EdgeLabel* m_pLabel; // 边的标签信息（激光数据的传感器位姿和位姿差，协方差矩阵）
  ```

- 成员函数

  ```c++
  inline Vertex<T>* GetSource() const // 获取边的起点顶点
  inline Vertex<T>* GetTarget() const // 获取边的终点顶点
  inline EdgeLabel* GetLabel() // 获取边的标签信息
  inline void SetLabel(EdgeLabel* pLabel) // 设置边的标签信息
  ```

边指的是传感器位姿之间的相互关系。一条边记录的这条边连接的两个顶点的所有信息。

#### LinkInfo

Karto SLAM中，`LinkInfo`指的是边的两个顶点的位姿、位姿差和协方差矩阵。

- 构造和析构函数：

  ```c++
  LinkInfo(const Pose2& rPose1, const Pose2& rPose2, const Matrix3& rCovariance)
  {
      Update(rPose1, rPose2, rCovariance);
  }
  
  virtual ~LinkInfo()
  {
  }
  ```

  **`LinkInfo`在构造时需要边的两个顶点的位姿（Karto SLAM中是传感器位姿），和协方差矩阵，并在构造时更新位姿差。**

- 成员变量：

  ```c++
  Pose2 m_Pose1; // 边的起点顶点的位姿（传感器位姿）
  Pose2 m_Pose2; // 边的终点顶点的位姿（传感器位姿）
  Pose2 m_PoseDifference; // 边的两个顶点的位姿差
  Matrix3 m_Covariance; // 边的两个顶点之间的协方差矩阵
  ```

- 成员函数

  ```c++
  void Update(const Pose2& rPose1, const Pose2& rPose2, const Matrix3& rCovariance) // 根据输入位姿和协方差，更新LinkInfo的位姿差和协方差矩阵信息
  {
      m_Pose1 = rPose1;
      m_Pose2 = rPose2;
  
      // transform second pose into the coordinate system of the first pose
      Transform transform(rPose1, Pose2());
      m_PoseDifference = transform.TransformPose(rPose2);
  
      // transform covariance into reference of first pose
      Matrix3 rotationMatrix;
      rotationMatrix.FromAxisAngle(0, 0, 1, -rPose1.GetHeading());
  
      m_Covariance = 
          rotationMatrix * rCovariance * rotationMatrix.Transpose();
  }
  ```

  将`pose2`转换到`pose1`的坐标系下，计算`pose2`到`pose1`的位姿的变化量，以及相对协方差矩阵。

  ```c++
  inline const Pose2& GetPose1() // 获取起点顶点的位姿（传感器位姿）
  inline const Pose2& GetPose2() // 获取终点顶点的位姿（传感器位姿）
  inline const Pose2& GetPoseDifference() // 获取位姿差
  inline const Matrix3& GetCovariance() // 获取以起点顶点的位姿为参考坐标系的协方差矩阵
  ```

### 激光数据

#### LocalizedRangeScan

类`LocalizedRangeScan`创建激光点云，包含激光点云的所有距离值，以及多个位姿。

- 构造和析构函数

  ```c++
  LocalizedRangeScan(const Name& rSensorName, const RangeReadingsVector& rReadings)
      : LaserRangeScan(rSensorName, rReadings)
          , m_IsDirty(true)
      {
      }
  
  virtual ~LocalizedRangeScan()
  {
  }
  ```

- 成员变量

  ```c++
  Pose2 m_OdometricPose; // 里程计表示的机器位姿
  Pose2 m_CorrectedPose; // 图优化后的机器位姿（若没有优化，数值同里程计位姿）
  Pose2 m_BarycenterPose; // 所有激光测距值的质心表示的机器位姿
  
  PointVectorDouble m_PointReadings; // 过滤后的激光点云的终端的世界坐标集
  PointVectorDouble m_UnfilteredPointReadings; // 未过滤的激光点云的终端的世界坐标集
  BoundingBox2 m_BoundingBox; // 过滤后的激光点云的终端的世界坐标集的最大范围框
  
  kt_bool m_IsDirty; // 更新上述信息的标识位
  ```

- 成员函数

  ```c++
  // 根据输入传感器位姿和传感器相对于机器的位姿，计算当前机器位姿
  void SetSensorPose(const Pose2& rScanPose)
  {
      Pose2 deviceOffsetPose2 = GetLaserRangeFinder()->GetOffsetPose();
      kt_double offsetLength = deviceOffsetPose2.GetPosition().Length();
      kt_double offsetHeading = deviceOffsetPose2.GetHeading();
      kt_double angleoffset = atan2(deviceOffsetPose2.GetY(), deviceOffsetPose2.GetX());
      kt_double correctedHeading = math::NormalizeAngle(rScanPose.GetHeading());
      Pose2 worldSensorOffset = 
          Pose2(offsetLength * cos(correctedHeading + angleoffset - offsetHeading),
                offsetLength * sin(correctedHeading + angleoffset - offsetHeading),
                offsetHeading);
  
      m_CorrectedPose = rScanPose - worldSensorOffset;
  
      Update();
  }
  ```

  `SetSensorPose()`根据传感器相对于机器位姿的位姿，获取当前传感器位姿处的机器位姿。

  ![scan-1](Karto SLAM代码解析.assets/scan-1.png)

  已知传感器相对于机器的位姿，比如，传感器朝向`sHeading=90`相对于机器朝向`rHeading=0`为逆时针90度，传感器位置分量相对于机器位置分量`(0,0)`为`(-1,-1)`。当新的传感器位姿为`(3,1,45度)`时，对应的机器位姿为`(3+1.414,1,-45度)`。推导如下：

  ```c++
  deviceOffsetPose2 = [sPose.x; sPose.y; sHeading] = [-1; -1; 90deg];
  offsetLength = L = 1.414; // 相对位移
  offsetHeading = sHeading = 90deg; // 相对朝向角度
  angleoffset = atan2(deviceOffsetPose2.GetY(), deviceOffsetPose2.GetX())
      = atan2(sPose.y, sPose.x) = -135deg; // 相对位移的朝向
  
  correctedHeading = sHeading' = 45deg; // 新的传感器朝向
  correctedHeading - offsetHeading = sHeading' - offsetHeading 
      = 45deg - 90deg = -45deg// 新的机器朝向
  correctedHeading - offsetHeading + angleoffset = -45deg + (-135deg)
      = -180deg // 新的相对位移的朝向
  worldSensorOffset = Pose2(
      offsetLength * cos(correctedHeading - offsetHeading + angleoffset),
      offsetLength * sin(correctedHeading - offsetHeading + angleoffset),
      offsetHeading)
      = Pose2(
      L * cos(45deg - 90deg + (-135deg)),
      L * sin(45deg - 90deg + (-135deg)),
      90deg) = Pose2(-L, 0, 90deg); // 新的传感器相对于机器的位移
  m_CorrectedPose = rPose' = sPose' - worldSensorOffset 
  	= [3; 1; 45deg] - [-1.414; 0; 90deg] = [3+1.414; 1; -45deg];
  ```

  --

  ```c++
  // 获取当前机器位姿下的传感器位姿
  inline Pose2 GetSensorPose() const
  {
      return GetSensorAt(m_CorrectedPose);
  }
  // 根据输入的机器位姿和传感器相对于机器的位姿，计算当前传感器位姿
  inline Pose2 GetSensorAt(const Pose2& rPose) const
  {
      // transform(rPose)指(0,0,0)相对于rPose的变换位姿
      // TransformPose(offsetPose)传感器相对于机器的位姿经过上述变换后得到的传感器位姿（将机器位姿为(0,0,0)处的传感器位姿变换到机器位姿为rPose处的传感器位姿）
      return Transform(rPose).TransformPose(GetLaserRangeFinder()->GetOffsetPose());
  }
  ```

  `GetSensorPose()`根据传感器相对于机器位姿的位姿，获取当前机器位姿处的传感器位姿。

这里说的里程计位姿、校正后的位姿等都是指机器位姿，而`scanPose`或者`sensorPose`指的是传感器位姿，两者之间由传感器相对机器的偏移位姿`offsetPose`关联起来。在做坐标转换时，经常要考虑当前位姿是说传感器位姿还是机器位姿，并做相应的偏移处理。

### 查表

#### LookupArray

`LookupArray`类创建用于查表的数组指针。

- 构造和析构函数

  ```c++
  LookupArray()
      : m_pArray(NULL)
          , m_Capacity(0)
          , m_Size(0)
      {
      }
  
  virtual ~LookupArray()
  {
      assert(m_pArray != NULL);
  
      delete[] m_pArray;
      m_pArray = NULL;
  }
  ```

- 成员函数

  ```c++
  kt_int32s* m_pArray; // 数组指针
  kt_int32u m_Capacity; // 数组容量
  kt_int32u m_Size; // 数组大小
  ```

#### GridIndexLookup

`GridIndexLookup`类创建一个按照角度分辨率分布的激光点云查找表。

- 构造和析构函数

  ```c++
  GridIndexLookup(Grid<T>* pGrid)
      : m_pGrid(pGrid) // 用局部相关网格地图初始化，用于提供网格地图的尺寸信息
          , m_Capacity(0)
          , m_Size(0)
          , m_ppLookupArray(NULL)
      {
      }
  
  virtual ~GridIndexLookup()
  {
      DestroyArrays();
  }
  ```

- 成员变量

  ```c++
  Grid<T>* m_pGrid; // 指向局部相关网格地图的地图指针
  kt_int32u m_Capacity; // 二维查找表的第一维容量
  kt_int32u m_Size; // 二维查找表的第一维大小
  LookupArray **m_ppLookupArray; // 二维指针，用于给相关地图的每个栅格分配一个查找表
  std::vector<kt_double> m_Angles; // 二维查找表的第一维角度值，由角度偏移总量和角度分辨率共同决定
  ```

- 成员函数

  ```c++
  // 对第angleIndex个角度偏量angle，将rLocalPoints中的一帧局部激光数据旋转angle角度，将旋转后的点云坐标加上当前局部相关网格地图的偏移量，再根据局部网格地图的尺寸，将世界坐标系下的点云坐标转换成地图的index，并记录在对应的查找表中
  void ComputeOffsets(kt_int32u angleIndex, kt_double angle, const Pose2Vector& rLocalPoints, LocalizedRangeScan* pScan)
  // 对给定的角度偏移量，角度分辨率，中心角度值，计算需要细化的角度数量，并将输入一帧激光数据依次旋转不同的角度，计算旋转后的每个点云对应的局部网格地图序号，更新在查找表中
  void ComputeOffsets(LocalizedRangeScan* pScan,
                          kt_double angleCenter,
                          kt_double angleOffset,
                          kt_double angleResolution)    
  ```


该查找表的第一维按照角度分辨率`angleResolution`和角度偏移值`angleOffset`划分`nAngles`个查找表。

```c++
kt_int32u nAngles = static_cast<kt_int32u>(math::Round(angleOffset * 2.0 / angleResolution) + 1);
```

| angle\index |  point1  |  point2  | $\cdots$ |  pointN  |
| :---------: | :------: | :------: | :------: | :------: |
| startAngle  | index11  | index12  | $\cdots$ | index1N  |
|  $\vdots$   | $\vdots$ | $\vdots$ | $\cdots$ | $\vdots$ |
| angleCenter | indexP1  |  indeP2  | $\cdots$ | indexPN  |
|  $\vdots$   | $\vdots$ | $\vdots$ | $\cdots$ | $\vdots$ |
|  endAngle   | indexQ1  | indexQ2  | $\cdots$ | indexQN  |

根据角度中心值`angleCenter`和角度偏移值`angleOffset`得到起始角度`startAngle`，按照`nAngles`个离散角度增量进行循环，最后的终止角度表示为`endAngle`：

```c++
kt_double angle = 0.0;
kt_double startAngle = angleCenter - angleOffset;
for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
{
    angle = startAngle + angleIndex * angleResolution;
    ComputeOffsets(angleIndex, angle, localPoints, pScan);
} // endAngle = angleCenter + angleOffset
```

查找表的第二维记录了旋转后的激光点云的地图坐标。在每个角度值处，将原始的激光点云按照当前角度值进行旋转，得到旋转后的点云世界坐标，并将该世界坐标转换到地图坐标中，记录地图坐标序号到查找表中，得到这个角度处的旋转后的激光点云地图坐标序号的查找表。

![lookup-1](Karto SLAM代码解析.assets/lookup-1.png)

### 地图

#### CoordinateConverter

`CoordinateConverter`类用于在世界坐标和网格坐标之间转换坐标，比如在世界坐标中1米=在网格坐标中1像素。默认值比例为20，即1像素=0.05米。并且只记录以当前传感器坐标为中心的一定感兴趣范围ROI内的地图的信息，称为局部地图，局部地图的偏移量用`offset`表示。

- 构造函数

  ```c++
  CoordinateConverter()
      : m_Scale(20.0)
      {
      }
  ```

- 成员变量

  ```c++
  Size2<kt_int32s> m_Size; // 地图的尺寸height*width
  kt_double m_Scale; // 世界坐标与网格坐标的比例，1米=m_Scale像素，m_Scale=1/resolution
  Vector2<kt_double> m_Offset; // 当前地图的偏移量，地图左下角(0,0)坐标在世界坐标下的起点位置
  ```

- 成员函数

  ```c++
  // 将世界坐标转换成网格坐标
  inline Vector2<kt_int32s> WorldToGrid(const Vector2<kt_double>& rWorld, kt_bool flipY = false) const
  // 将网格坐标转换成世界坐标
  inline Vector2<kt_double> GridToWorld(const Vector2<kt_int32s>& rGrid, kt_bool flipY = false) const
  ```

下图显示了地图偏移量和局部地图尺寸在世界坐标系下的体现。黑色原点是当前传感器位姿，红色矩形为以当前传感器为中心的感兴趣区域ROI，局部地图建立在这个区域基础上。世界坐标系零点为`(0,0)`，局部地图起点在世界坐标系下的坐标为`(offset.x,offset.y)`，在局部地图中，任意一个点云所在的地图坐标转换后表示为`(grid.x,grid.y)`。

![grid-1](Karto SLAM代码解析.assets/grid-1.png)

#### Grid

`Grid`类创建地图，根据输入`width`和`height`以及分辨率`resolution`创建一个局部网格地图。其中地图的`width`值经过了8bit对齐后是`m_widthStep`。

初始化地图网格的数据都是0。起点在左下角，网格坐标为(0,0)。

- 构造和析构函数

  ```c++
  Grid(kt_int32s width, kt_int32s height)
      : m_pData(NULL)
          , m_pCoordinateConverter(NULL)
      {
          Resize(width, height);
      }
  
  virtual ~Grid()
  {
      delete [] m_pData;
      delete m_pCoordinateConverter;
  }
  ```

- 成员变量

  ```c++
  kt_int32s m_Width;       // width of grid
  kt_int32s m_Height;      // height of grid
  kt_int32s m_WidthStep;   // 8 bit aligned width of grid
  T* m_pData;              // grid data
  
  CoordinateConverter* m_pCoordinateConverter; // 世界坐标和网格坐标的转换
  ```

- 成员函数

  ```c++
  // 判断坐标是否有效，在width和height范围内
  inline kt_bool IsValidGridIndex(const Vector2<kt_int32s>& rGrid) const
  // 将给定网格坐标转换成index
  virtual kt_int32s GridIndex(const Vector2<kt_int32s>& rGrid, kt_bool boundaryCheck = true) const
  // 将给定index转换成网格坐标
  Vector2<kt_int32s> IndexToGrid(kt_int32s index) const
  // 将给定世界坐标转换成网格坐标
  inline Vector2<kt_int32s> WorldToGrid(const Vector2<kt_double>& rWorld, kt_bool flipY = false) const
  // 将给定网格坐标转换成世界坐标
  inline Vector2<kt_double> GridToWorld(const Vector2<kt_int32s>& rGrid, kt_bool flipY = false) const
  ```

`Grid`类只记录宽、高和网格数据。`CoordinateConverter`类用于转换世界坐标系和地图坐标系，并记录该地图与世界坐标系的转换参数，如缩放系数`m_scale`(即分辨率倒数`1.0/resolution`)、偏移`m_Offset`，等。

![grid-2](Karto SLAM代码解析.assets/grid-2.png)

上述两个类相互依赖，包含了世界坐标转换地图序号`Index`、地图序号`Index`转换世界坐标等各类坐标转换接口。

地图的状态分为占据、空闲和未知：

```c++
typedef enum
{
    GridStates_Unknown = 0,
    GridStates_Occupied = 100,
    GridStates_Free = 255
} GridStates;
```

此外，`Grid`类有一个光束轨迹累积函数，方法同`Bresenham`算法一样，用于将光束覆盖的网格的数值增加一个常量。

```c++
void TraceLine(kt_int32s x0, kt_int32s y0, kt_int32s x1, kt_int32s y1, Functor* f = NULL)
{
    kt_bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    kt_int32s deltaX = x1 - x0;
    kt_int32s deltaY = abs(y1 - y0);
    kt_int32s error = 0;
    kt_int32s ystep;
    kt_int32s y = y0;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    kt_int32s pointX;
    kt_int32s pointY;
    for (kt_int32s x = x0; x <= x1; x++)
    {
        if (steep)
        {
            pointX = y;
            pointY = x;
        }
        else
        {
            pointX = x;
            pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX)
        {
            y += ystep;
            error -= deltaX;
        }

        Vector2<kt_int32s> gridIndex(pointX, pointY);
        if (IsValidGridIndex(gridIndex))
        {
            kt_int32s index = GridIndex(gridIndex, false);
            T* pGridPointer = GetDataPointer();
            pGridPointer[index]++;

            if (f != NULL)
            {
                (*f)(index);
            }
        }
    }
}
```

#### CorrelationGrid

该`CorrelationGrid`类用于扫描匹配前建立局部相关地图。与`Grid`类相比，`CorrelationGrid`类对原始地图进行了边缘扩充`2×borderSize`，便于根据平滑偏差`m_SmearDeviation`使用核函数`m_pKernel`进行平滑。

- 构造和析构函数

  ```c++
  CorrelationGrid(kt_int32u width, kt_int32u height, kt_int32u borderSize,
                  kt_double resolution, kt_double smearDeviation)
      : Grid<kt_int8u>(width + borderSize * 2, height + borderSize * 2)
          , m_SmearDeviation(smearDeviation)
          , m_pKernel(NULL)
      {
          GetCoordinateConverter()->SetScale(1.0 / resolution);
  
          // setup region of interest
          m_Roi = Rectangle2<kt_int32s>(borderSize, borderSize, width, height);
  
          // calculate kernel
          CalculateKernel();
      }
  
  virtual ~CorrelationGrid()
  {
      delete [] m_pKernel;
  }
  ```

- 成员变量

  ```c++
  kt_double m_SmearDeviation; // 平滑偏差
  kt_int32s m_KernelSize; // 核边长
  kt_int8u* m_pKernel; // 核数组
  Rectangle2<kt_int32s> m_Roi; // 感兴趣区域
  ```

说明一下搜索区域、感兴趣区域`ROI`和最大平滑区域。

搜索区域用`Grid`类创建，指的是可能的传感器位姿的分布范围。在扫描匹配中按照位移偏移量进行匹配时，该区域限制了遍历位移偏移的数量。如下图左图所示。`searchSize`参数来自配置。

感兴趣区域`ROI`用`CorrelationGrid`类创建，指的是局部相关网格地图的范围。是在搜索区域的范围基础上，向外扩充了最大激光测距有效值`margin`的距离，表示当传感器位于搜索区域的边界时，最远的有效点云也在地图范围内。如下图中间所示。

最大平滑区域用`Grid`类创建，指的是当局部相关地图允许平滑时，核在地图边缘时数据不会溢出。是在局部相关地图的范围基础上，向外扩充了一半核窗口`halfKernelSize+1`大小，以防对边缘网格进行平滑时数组溢出。如下图右图所示。

![grid-3](Karto SLAM代码解析.assets/grid-3.png)

说明一下核。在`CorrelationGrid`类中，允许对网格地图进行平滑，平滑参数为`m_SmearDeviation`，由平滑参数和分辨率一起得到核窗口的边长`m_KernelSize`：

```c++
// 获得核窗口边长的一半
static kt_int32s GetHalfKernelSize(kt_double smearDeviation, kt_double resolution)
{
    assert(resolution != 0.0);

    return static_cast<kt_int32s>(math::Round(2.0 * smearDeviation / resolution));
}
// 获得核窗口边长，+1确保是奇数，有中心网格
m_KernelSize = 2 * GetHalfKernelSize(m_SmearDeviation, resolution) + 1;
```

进而得到核数组`m_pKernel`：

```c++
m_pKernel = new kt_int8u[m_KernelSize * m_KernelSize];
```

核数组的数值根据高斯分布来计算：

```c++
kt_int32s halfKernel = m_KernelSize / 2;
for (kt_int32s i = -halfKernel; i <= halfKernel; i++)
{
    for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
    {
        kt_double distanceFromMean = hypot(i * resolution, j * resolution);
        kt_double z = exp(-0.5 * pow(distanceFromMean / m_SmearDeviation, 2));

        kt_int32u kernelValue = static_cast<kt_int32u>(math::Round(z * GridStates_Occupied));
        assert(math::IsUpTo(kernelValue, static_cast<kt_int32u>(255)));

        int kernelArrayIndex = (i + halfKernel) + m_KernelSize * (j + halfKernel);
        m_pKernel[kernelArrayIndex] = static_cast<kt_int8u>(kernelValue);
    }
}
```

举个例子：

令平滑参数`smearDeviation = 0.03`，分辨率`resolution = 0.05`。则核边长为`kernelSize = 2×(int)(2×0.03/0.05)+1 = 3`。一半核边长为`halfKernel = 1`。剩余的核数值见图：

![grid-4](Karto SLAM代码解析.assets/grid-4.png)

将核应用于地图平滑，是指如果某个给定网格点为障碍物，则在以该点为中心的核窗口中，一一比较窗口中的格子的地图值和核数值，如果核数值大于地图值，则将核数值复制给地图的对应格子，达到平滑的效果。

```c++
inline void SmearPoint(const Vector2<kt_int32s>& rGridPoint)
{
    assert(m_pKernel != NULL);

    int gridIndex = GridIndex(rGridPoint);
    if (GetDataPointer()[gridIndex] != GridStates_Occupied)
    {
        return;
    }

    kt_int32s halfKernel = m_KernelSize / 2;

    // apply kernel
    for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
    {
        kt_int8u* pGridAdr = GetDataPointer(Vector2<kt_int32s>(rGridPoint.GetX(), rGridPoint.GetY() + j));

        kt_int32s kernelConstant = (halfKernel) + m_KernelSize * (j + halfKernel);

        // if a point is on the edge of the grid, there is no problem
        // with running over the edge of allowable memory, because
        // the grid has margins to compensate for the kernel size
        for (kt_int32s i = -halfKernel; i <= halfKernel; i++)
        {
            kt_int32s kernelArrayIndex = i + kernelConstant;

            kt_int8u kernelValue = m_pKernel[kernelArrayIndex];
            if (kernelValue > pGridAdr[i])
            {
                // kernel value is greater, so set it to kernel value
                pGridAdr[i] = kernelValue;
            }
        }
    }
}
```

#### OccupancyGrid

`OccupancyGrid`类用于追迹光束，统计光束通过某个网格的次数或者光束终端落在某个网格的次数，根据两者的比例更新网格地图的障碍物关系。

- 构造和析构函数

  ```c++
  OccupancyGrid(kt_int32s width, kt_int32s height, const Vector2<kt_double>& rOffset, kt_double resolution)
      : Grid<kt_int8u>(width, height)
          , m_pCellPassCnt(Grid<kt_int32u>::CreateGrid(0, 0, resolution))
          , m_pCellHitsCnt(Grid<kt_int32u>::CreateGrid(0, 0, resolution))
          , m_pCellUpdater(NULL)
      {
          m_pCellUpdater = new CellUpdater(this);
  
          if (karto::math::DoubleEqual(resolution, 0.0))
          {
              throw Exception("Resolution cannot be 0");
          }
  
          m_pMinPassThrough = new Parameter<kt_int32u>("MinPassThrough", 2);
          m_pOccupancyThreshold = new Parameter<kt_double>("OccupancyThreshold", 0.1);
  
          GetCoordinateConverter()->SetScale(1.0 / resolution);
          GetCoordinateConverter()->SetOffset(rOffset);
      }
  
  virtual ~OccupancyGrid()
  {
      delete m_pCellUpdater;
  
      delete m_pCellPassCnt;
      delete m_pCellHitsCnt;
  
      delete m_pMinPassThrough;
      delete m_pOccupancyThreshold;
  }
  ```

- 成员变量

  ```c++
  //Counters of number of times a beam passed through a cell
  Grid<kt_int32u>* m_pCellPassCnt; // 用另一个Grid类来统计光束通过网格的次数
  
  //Counters of number of times a beam ended at a cell
  Grid<kt_int32u>* m_pCellHitsCnt; // 用另一个Grid类来统计光束终端落于网格的次数
  
  CellUpdater* m_pCellUpdater; // 用于更新该地图的更新器
  
  ////////////////////////////////////////////////////////////
  // NOTE: These two values are dependent on the resolution.  If the resolution is too small, then not many beams will hit the cell!
  
  // Number of beams that must pass through a cell before it will be considered to be occupied or unoccupied.  This prevents stray beams from messing up the map. 直到一定数量的光束通过了该网格，再考虑该网格是障碍物还是空闲区域，防止杂散光涂抹地图
  Parameter<kt_int32u>* m_pMinPassThrough;
  
  // Minimum ratio of beams hitting cell to beams passing through cell for cell to be marked as occupied
  Parameter<kt_double>* m_pOccupancyThreshold;
  ```

- 成员函数

  ```c++
  // 重置网格地图，清除数据
  virtual void Resize(kt_int32s width, kt_int32s height)
  // 更新网格地图
  virtual void Update()
  // 根据m_pCellPassCnt和m_pCellHitsCnt更新单个网格
  virtual void UpdateCell(kt_int8u* pCell, kt_int32u cellPassCnt, kt_int32u cellHitCnt)
  // 光束追迹
  virtual kt_bool RayTrace(const Vector2<kt_double>& rWorldFrom,
                               const Vector2<kt_double>& rWorldTo,
                               kt_bool isEndPointValid,
                               kt_bool doUpdate = false)
  ```

通过`RayTrace()`函数，并内部调用直线转换为网格的`Bresenham`算法，将光束通过的网格`m_pCellPassCnt`的数值+1，对有效光束终端，同时将光束终端所在的网格`m_pCellPassCnt`和`m_pCellHitsCnt`的数值+1。如此，将世界坐标系下光束的分布转换到两个网格地图中网格的访问次数中，进而用于网格地图的更新。

![grid-5](Karto SLAM代码解析.assets/grid-5.png)

继上述光束追迹之后，根据光束通过网格`m_pCellPassCnt`和光束终端网格`m_pCellHitsCnt`的数据，遍历整个网格地图，对每一个网格，首先判断该网格的光束通过次数是否达到阈值，避免杂散光对地图的涂抹。满足上述条件后，再比较光束终端次数和光束通过次数的比例，满足一定阈值则认为该网格是障碍物，反之，该网格是空闲区域。

通过`AddScan()`函数，将包含距离和终点坐标信息的激光点云依次插入到网格地图中。激光点云的距离如果超出最大最小范围，则跳过该光束；激光点云的距离如果超出阈值，则认为光束终端不可靠，不计入地图中，并且将光束范围重置到有效范围内，再插入地图中。

## 3. 代码流程解析

### 入口流程

入口主函数在`slam_karto`中。进入`main()`函数后，创建了`SlamKarto`类。

![main-1](Karto SLAM代码解析.assets/main-1.png)

在`laserCallback()`中，从`msg`中拿到了一帧激光数据和**传感器相对于机器的位姿**，进入`kartoSLAM`并更新地图。

![main-2](Karto SLAM代码解析.assets/main-2.png)

在获取激光雷达帧时，从`msg`中得到了激光雷达的参数信息，包括：传感器相对于机器的偏移位姿、距离最大最小范围、角度最大最小范围、角度分辨率等。

![main-3](Karto SLAM代码解析.assets/main-3.png)

在`addScan()`中，首先通过`getOdomPose()`函数从`msg`中获取里程计估计的机器位姿；然后创建`LocalizedRangeScan`激光数据类，设置这一帧激光数据的里程计估计的机器位姿，和校正后的机器位姿（**由于当前并未经过图优化校正，该位姿初始值与里程计位姿相同**）。随后进入`kartoSLAM`重要的处理过程。如果处理成功，则获取优化后的当前机器位姿。

![main-4](Karto SLAM代码解析.assets/main-4.png)

### 主要处理流程

上述入口流程完成了数据获取和处理的流程分析。现在进入匹配和图优化处理的流程分析。在获取一帧激光数据，以及相应的里程计位姿后，就可以进入`Process()`中进行匹配和校正位姿。

![process-1](Karto SLAM代码解析.assets/process-1.png)

首先，跳过无效帧的处理。

第二步，判断`Mapper`类的成员变量是否经过初始化。需要初始化的成员变量如下：

```c++
// 连续扫描匹配器--用于扫描匹配
ScanMatcher* m_pSequentialScanMatcher;
// 用于记录传感器数据，激光点云和各种位姿
MapperSensorManager* m_pMapperSensorManager;
// 用于图优化
MapperGraph* m_pGraph;
```

其中，`ScanMatcher`类的构造需要几个参数，来自`Mapper`类的加载参数，如下：

```c++
/**
 * The size of the search grid used by the matcher. 匹配时搜索的网格范围
 * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
 */
Parameter<kt_double>* m_pCorrelationSearchSpaceDimension;

/**
 * The resolution (size of a grid cell) of the correlation grid. 相关网格的分辨率
 * Default value is 0.01 meters.
 */
Parameter<kt_double>* m_pCorrelationSearchSpaceResolution;

/**
 * The point readings are smeared by this value in X and Y to create a smoother response. 平滑地图的范围
 * Default value is 0.03 meters.
 */
Parameter<kt_double>* m_pCorrelationSearchSpaceSmearDeviation;
```

`MapperSensorManager`类的构造需要几个参数，来自`Mapper`类的加载参数，如下：

```c++
/**
 * Scan buffer size is the length of the scan chain stored for scan matching. 记录激光数据的最大数量
 * "scanBufferSize" should be set to approximately "scanBufferMaximumScanDistance" / "minimumTravelDistance".
 * The idea is to get an area approximately 20 meters long for scan matching.
 * For example, if we add scans every minimumTravelDistance == 0.3 meters, then "scanBufferSize"
 * should be 20 / 0.3 = 67.)
 * Default value is 67.
 */
Parameter<kt_int32u>* m_pScanBufferSize;

/**
 * Scan buffer maximum scan distance is the maximum distance between the first and last scans 记录激光数据的最长距离
 * in the scan chain stored for matching.
 * Default value is 20.0.
 */
Parameter<kt_double>* m_pScanBufferMaximumScanDistance;
```

第三步，获取上一帧激光数据。如果不存在上一帧激光数据，说明当前激光数据是第一帧。

第四步，如果存在上一帧激光数据，则获取其中记录的从原始里程计位姿到优化校正后的位姿之间的变换位姿。并用该变换位姿初步校正当前帧的里程计位姿。这一步初步校正不一定存在，如果上一帧激光数据并没有经过图优化，则变换位姿为`(0,0,0)`。

第五步，判断机器在获取当前帧激光数据时是否已经走过一段距离或者旋转了一定角度。针对第一帧激光数据则该判断结果为`true`，针对其他帧激光数据，判断条件包含三个：经过一定的时间，旋转了一定的角度，移动了一定的范围。满足以上任一条件则判断结果为`true`，反之判断结果为`false`。该判断降低了激光数据的处理频率，避免环境没有发生明显变化导致的冗余计算。

第六步，进行扫描匹配。首先根据参数和上一帧激光数据是否存在来判断是否可以进行扫描匹配。第一帧激光数据由于缺少参考激光数据，无法进行扫描匹配。非第一帧激光数据的参考激光数据是一串连续的过去的激光数据，存储为`m_RunningScans`，其中保存了从上一帧开始的与上一帧距离在一定范围内的所有激光数据。随着机器位置的变化，该激光数据串也在不断更新，保存最新的激光数据，删除旧的激光数据。`MatchScan()`扫描匹配的详细介绍放在[扫描匹配](#扫描匹配)中。扫描匹配后将更新该帧的传感器位姿。

第七步，`AddScan()`存储当前激光数据。分别存储到`MapperSensorManager`类和`ScanMatcher`类中，并给激光数据加上ID。

第八步，如果允许进行扫描匹配，则给图添加顶点和边，给`m_RunningScans`添加当前帧激光数据，判断是否进行闭环优化。给图添加顶点和边的详细介绍放在[图添加顶点和边](#图添加顶点和边)中。闭环检测及优化的详细介绍放在[回环检测及优化](#回环检测及优化)中。

最后一步，将当前激光数据更新为`lastScan`。

### 扫描匹配

#### MatchScan()

匹配的过程的入口。设置好匹配的范围以及匹配参数，调用`CorrelateScan()`进行匹配，返回匹配率，位姿均值和协方差矩阵。根据参数不同，分为粗匹配和精匹配。粗匹配之后，如果需要还可以进一步扩充角度偏移的范围，非必需。精匹配缩小了位姿的搜索范围，以及角度分辨率，并以粗匹配的结果作为估计值开始匹配。

![scanmatch-1](Karto SLAM代码解析.assets/scanmatch-1.png)

如果当前帧点云的点数为零，则使用里程计的位姿作为匹配后的位姿，返回默认协方差矩阵。

将感兴趣区域地图中心移到当前点云区域。感兴趣区域尺寸是固定的，以当前传感器世界坐标为中心的一个矩形，从而得到局部地图零点的偏移量。将这个偏移量设置给局部地图。

在添加点云之前，重置局部地图`m_pCorrelationGrid->Clear()`。`AddScans()`插入上一步筛选出的周围临近备选激光点云，到局部地图中，将点云终端的位置转换到世界坐标系下，并将地图`Grid`在该点的属性更新为`Occ`。地图状态只有两种：占据、未知。如果允许进行平滑`doSmear`，那么占据格子会以它为中心按照核的分布对周围的未知格子赋值，增加占据属性的分辨率。

![grid-6](Karto SLAM代码解析.assets/grid-6.png)

#### CorrelateScan()

当前激光数据帧与局部相关地图的匹配过程。局部相关地图已经在`MatchScan()`阶段完成了更新。在`CorrelateScan()`阶段则根据给定分辨率通过位移和角度偏移进行扫描匹配。记录每一种位姿下获得的匹配响应结果，累积匹配响应结果最大的位姿，取平均值得到最佳位姿和最优匹配响应结果，再计算协方差矩阵。

![scanmatch-2](Karto SLAM代码解析.assets/scanmatch-2.png)

`ComputeOffsets()`根据输入的中心角度值、角度分辨率和角度偏移量，计算需要细化迭代的角度数量及对应的角度值，并将输入激光数据依次旋转上述角度值，将点云终端对应的局部相关地图的序号记录到查找表中。这些在[GridIndexLookup](#GridIndexLookup)中已经进行过描述。

仅在粗匹配时重置搜索地图的信息，包括偏移量和搜索地图中记录的匹配响应值。

需要进行匹配的次数=X轴网格数\*Y轴网格数\*角度数=`nX*nY*nAngle`。每一个位姿匹配后记录对应的匹配响应值。

![scanmatch-3](Karto SLAM代码解析.assets/scanmatch-3.png)

`GetResponse()`是把所有输入角度对应的`LookupArray`中的网格序号对应的局部相关地图的状态累积，并取平均值作为匹配响应结果。

**注：`ComputeOffsets()`  时，将传感器坐标从`(0,0)`点移动到`gridOffset`处，存储了当前网格下的点的序号；`GetResponse()`时，将取到的每个点的序号进一步移动到`gridPositionIndex`处，即传感器相对于网格的偏移量，得到局部相关网格下的每个点的序号。**

`ComputePositionalCovariance()`和`ComputeAngularCovariance()`分别计算了位姿协方差和角度协方差。再次遍历所有备选匹配点，根据其匹配响应和位置偏移的关系，累计出位姿的协方差和角度的协方差。

![scanmatch-4](Karto SLAM代码解析.assets/scanmatch-4.png)

![scanmatch-5](Karto SLAM代码解析.assets/scanmatch-5.png)

### 图添加顶点和边

#### AddVertex()-AddNode()

图中添加顶点，对于Karto SLAM来说，顶点就是一帧激光数据，其中包含了一组对应的位姿。对于SPA优化来说，顶点也就是节点，就是该激光数据的匹配后的位姿。

![graph-1](Karto SLAM代码解析.assets/graph-1.png)

#### AddEdge()

该接口输入两帧激光数据，转换成图中对应的两个顶点`vSource`和`vTarget`，表示起点顶点和终点顶点。然后获取起点顶点`vSource`的关联的边，遍历每条边，如果边的终点顶点与`vTarget`相同，表示查找到已经存在的边，返回该边。如果没有找到，则根据两个顶点创建新的边，将新的边分别添加到两个顶点中，并将新的边添加到图中。

![graph-4](Karto SLAM代码解析.assets/graph-4.png)

下图坐标是找到已经存在的边，右边是添加新的边。

![graph-5](Karto SLAM代码解析.assets/graph-5.png)

#### AddEdges()

图中添加边，对于Karto SLAM来说，边的关系来自激光数据帧之间的相对位姿和协方差矩阵。当传感器仅有第一帧激光数据时，将从其他传感器中获取激光数据帧，进行匹配和关联，保留匹配率较高的位姿和协方差矩阵信息。当传感器有两帧以上的激光数据时，首先关联上一帧激光数据，进而关联`runningScans`一串连续的激光数据帧。此外，还需要关联邻近的链条。最后将所有位姿和协方差矩阵求平均，计算当前帧的传感器位姿。

![graph-2](Karto SLAM代码解析.assets/graph-2.png)

#### LinkScans()-AddConstraint()

该接口实现了对输入的两帧激光数据以及后一帧的位姿和协方差矩阵，查找是否存在相连的边，如果不存在，则创建新的边，并将边的信息（相对位姿和协方差矩阵）存储到图中，用于SPA优化。

![graph-3](Karto SLAM代码解析.assets/graph-3.png)

#### LinkChainToScan()

该接口实现了对输入的一串激光数据帧和当前激光数据帧以及后一帧的位姿和协方差，从激光数据帧链中寻找距离当前帧最近的一帧激光数据，并对最近的这两帧激光数据进行上述的`LinkScans()`添加边的信息。

这里的最近比较的是两帧激光数据的`referencePose`之间的距离。

![graph-6](Karto SLAM代码解析.assets/graph-6.png)

#### FindNearLinkedScans()

该接口根据输入的激光点云和距离阈值，寻找该距离范围内的相邻的其他激光点云集。内部主要接口为`Traverse()`，使用了深度遍历方法。

**输入：**

```c++
LocalizedRangeScan* pScan // 当前激光点云
kt_double maxDistance // 最大搜索范围
```

**输出：**

```c++
LocalizedRangeScanVector nearLinkedScans // 范围内的所有激光点云的集合链
```

首先从起点顶点开始遍历与之相连的其他顶点（与顶点相连的边的另一个顶点），将满足一定范围的顶点添加到`toVisit`集合中；接着以`toVisit`集合中某个顶点为起点，遍历与该顶点相连的其他顶点，将满足范围的顶点继续添加到`toVisit`集合中。不断扩充遍历范围。直到`toVisit`集合中的所有顶点都遍历到。

第一次遍历：

(0) 将当前scan作为起始顶点，添加到将要访问的顶点队列`toVisit`和已经看到过的顶点集`seenVertices`中

(1) 以将要访问的顶点队列`toVisit`的第一个顶点作为本次循环的起始顶点`pNext`，从队列中删除第一个顶点

(2) 根据预先设定的最大可接受范围，判断该顶点在最大可接受范围内`Visit()`。true->(3)，false->遍历`toVisit`队列的下一个顶点

(3) 将当前顶点`pNext`添加到有效顶点数组`validVertices`中，遍历与当前顶点`pNext`相连的其他顶点（与顶点相连的边的另一个顶点），将相连的其他顶点添加到相邻顶点集`adjacentVertices`中

(4) 遍历相邻顶点集`adjacentVertices`，将其中未看到过的顶点（不在`seenVertices`集合中）添加到将要访问的顶点队列`toVisit`和已经看到过过的顶点集`seenVertices`中

![scan-2](Karto SLAM代码解析.assets/scan-2.png)

第一次循环结果：

```c++
toVisit.size()=4
seenVertices.size()=8
validVertices.size()=1
```

第n次遍历，重复(1) - (4)步骤。

![scan-3](Karto SLAM代码解析.assets/scan-3.png)

第n次循环结果：

```c++
toVisit.size()=6
seenVertices.size()=11
validVertices.size()=2
```

本例的最终效果：

```c++
toVisit.size()=0
seenVertices.size()=13
validVertices.size()=10
```

![scan-4](Karto SLAM代码解析.assets/scan-4.png)

最终得到了由满足距离条件的所有相连的激光点云形成的集合。

#### FindNearChains()

该接口实现了寻找邻近的激光数据链的功能。首先通过`FindNearLinkedScans()`找到与输入帧相邻的所有激光数据帧，遍历相邻的激光数据帧，对每一帧激光数据，根据ID序号先依次递减，遍历当前帧以前的激光数据，根据位姿之间距离的远近，选择保留或者舍弃该激光数据到当前激光数据链中，超出距离范围后，结束当前遍历。一旦在遍历过程中找到输入帧，则认为该激光数据链无效。

完成根据ID序号依次递减的遍历后，将当前激光数据添加到当前激光数据链中。

再根据ID序号依次递增，遍历当前帧以后的激光数据，同样根据位姿之间的距离的远近，旋转保留或者舍弃该激光数据到当前激光数据链中，超出距离范围后，结束当前遍历。一旦在遍历过程中找到输入帧，则认为该激光数据链无效。

在一次外部遍历过程中，一旦认为该激光数据链有效，则存储当前激光数据链。

最后返回所有的有效激光数据链。

**每条激光数据链来自于以某一帧相邻的激光数据为中点，根据ID大小递减和递增后，在一定相对位移范围内的所有激光数据组成。**

![graph-8](Karto SLAM代码解析.assets/graph-8.png)

#### LinkNearChains()

该接口实现了对输入的一帧激光数据以及位姿和协方差，寻找附近的相连的激光数据链，对每条激光数据链，首先进行`MatchScan()`局部地图匹配，如果匹配结果较好，则进行上述的`LinkChainToScan()`从激光数据链与当前激光数据帧中寻找边的信息。

![graph-7](Karto SLAM代码解析.assets/graph-7.png)

### 回环检测及优化

#### TryCloseLoop()

重要的闭环检测入口，反复调用`FindPossibleLoopClosure()`寻找备选激光数据链，调用`MatchScan()`进行粗匹配，根据粗匹配结果决定是否进行精匹配，根据精匹配结果决定是否进行优化`LinkChainToScan()`和`CorrectPoses()`。

![closeloop-1](Karto SLAM代码解析.assets/closeloop-1.png)

**输入：**

```c++
LocalizedRangeScan* pScan // 一帧点云
const Name& rSensorName // 传感器名字
```

**输出：**

```c++
loopClosed = true or false // 是否闭环优化
```

#### FindPossibleLoopClosure()

该接口实现了寻找可能闭环的激光数据链功能。首先找到输入激光数据帧的一定位移范围内的所有相邻激光数据帧。在后续的寻找可能闭环的激光数据链的过程中，要剔除相邻激光数据帧。

![closeloop-2](Karto SLAM代码解析.assets/closeloop-2.png)

从`rStartNum`开始遍历所有的过去的激光数据帧，如果位移在一定范围内，并且不属于相邻帧，则将该帧添加到备选链中。

如果属于相邻帧，则清空当前备选链，继续遍历下一帧。

如果位移超过一定的范围，并且当前备选链大小达到阈值（Karto SLAM中阈值为10），则返回当前备选链，以及对应的遍历到当前帧的序号，便于下一次寻找可能闭环时继续遍历。

如果位移超过一定的范围，且备选链大小小于阈值，则清空备选链，继续遍历下一帧。

下图示例，假设链的最小数量为4，则满足条件的链有两条，见绿色的圈和连线。

![closeloop-3](Karto SLAM代码解析.assets/closeloop-3.png)

`GetReferencePose()`根据输入参数返回位姿，如果使用质心，则返回这一帧激光点云的质心位置；反之，返回传感器在世界坐标系下的位姿。

#### CorrectPoses()

重要的优化入口，调用`Compute()`对已经添加的节点和约束进行SPA优化，调用`GetCorrections()`返回优化后的结果，并将优化后的位姿赋值`SetSensorPose(pose)`给对应的激光点云帧`GetScan(ID)`上，替换原始位姿。

![closeloop-4](Karto SLAM代码解析.assets/closeloop-4.png)

#### Compute()

优化求解的过程调用了`sparse_bundle_adjustment`库的`doSPA()`接口。获取优化后的结果调用了`getNodes()`接口，转换成`vector<pair<ID, pose>>`形式的输出结果。

![closeloop-5](Karto SLAM代码解析.assets/closeloop-5.png)

**参数：**

```c++
typedef std::vector<std::pair<kt_int32s, Pose2>> IdPoseVector;
karto::ScanSolver::IdPoseVector corrections;
```

**输出：**

```c++
vector<pair<ID, pose>>一一对应的位姿集合。
```

**理论：**

整个优化算法使用了LM优化算法。

观测量：
$$
h(c_i,c_j)=\begin{bmatrix} 
R^T_i(t_j-t_i) \\ \theta_j-\theta_i
\end{bmatrix}
$$
优化代价函数：
$$
\begin{split}
e_{ij} &= z_{ij}-h(c_i,c_j) \\
\chi^2 &= \sum \limits_{ij} e^T_{ij}\Lambda_{ij}e_{ij}
\end{split}
$$

其中，信息矩阵$\Lambda$表示为：
$$
\lambda=\begin{bmatrix}
\Lambda_{ab} & & \\ & \ddots & \\ & & \Lambda_{mn}
\end{bmatrix}
$$
代价函数的雅可比矩阵为：
$$
\begin{split}
\frac{\partial e_{ij}}{\partial t_i} &= \begin{bmatrix} 
-R^T_i \\ 0 \; 0 \end{bmatrix} \\
\frac{\partial e_{ij}}{\partial \theta_i} &= \begin{bmatrix}
-\frac{\partial R^T_i}{\partial \theta_i}(t_j-t_i) \\ -1
\end{bmatrix}
\end{split}
$$
LM系统方程：
$$
({\rm H}+\lambda diag {\rm H})\Delta x={\rm J}^T{\Lambda}{\rm e}
$$
其中，${\rm J}=\frac{\partial {\rm e}}{\partial {\rm x}},{\rm H}={\rm J}^T{\rm \Lambda}{\rm J}$。

更新方程为：
$$
\begin{split} 
t_i &= t_i + \Delta t_i \\
\theta_i &= \theta_i + \Delta \theta_i
\end{split}
$$


### 建图

只保存了位姿节点和激光数据，作为`pose-graph`。如果需要看到网格地图，需要将存储的激光数据按照对应的校正后的位姿插入到地图中进行显示。

