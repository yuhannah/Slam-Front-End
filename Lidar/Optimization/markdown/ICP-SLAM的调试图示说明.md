# ICP-SLAM的调试图示说明

部分调试图示处于关闭状态，需要手动打开。

在`macros.h`文件中有个宏，用于部分图示的开关：

```c++
#define DEBUG_SLAM false
```



## 当前帧的点云

文件：`CManager.cpp`

事件：`SLAM_SLAM_SCAN`

变量：`m_lastScan`

画图代码：

```c++
// 局部
->registerHandler(comm::socket::EMessageType::CLOUD_POINT, [this](std::shared_ptr<mrpt::utils::CMessage> message)
            {
                std::lock_guard<mutex> lock{m_mutex};
                message->serializeObject(&m_lastScan);
                return message;
            })
```

图示：红色的点-线，与网格地图的黑色边界重合。不断变化的点云表示点云在实时更新。即使机器静止，环境没有发生变化，每一帧点云也会闪烁轻微变化的。



出错：

1. 点云停止在同一帧，完全没有变化，表示点云没有更新，slam没有发出最新的事件。一般出现在中途。可能是slam计算结果出错，强制停止更新。也可能是雷达没有新的数据，无法进入slam，无法刷新上一次的点云。
2. 完全没有点云，表示没有获取雷达传感器数据。一般出现在刚开机。可能是雷达传感器没有有效范围内的点云。可能是机器发生倾斜，超出阈值，强制无效了所有的点云。
3. 有点云，也在实时更新，但是点云偶尔没有与网格地图重叠，发生了一定的角度偏移。一般出现在所有时期。原因是点云与位姿不匹配。**当一帧点云作为输入进入slam算法，并得到一个位姿计算结果，如果要正确在地图上显示这一帧点云，需要将这一帧点云画在这个位姿计算结果上，才能保证数据同步。**

## 当前网格地图

文件：`CManager.cpp`

事件：`SLAM_MAP`

变量：`m_lastMap`

画图代码：

```c++
// 局部
->registerHandler(comm::socket::EMessageType::MAP, [this](std::shared_ptr<mrpt::utils::CMessage> message)
            {
                std::lock_guard<mutex> lock{m_mutex};
                message->serializeObject(&m_lastMap);
                return message;
            })
```

图示：网格状，每个格子填充了从全白色到全黑色的颜色，中间是过渡的灰白、灰色和灰黑。灰色表示未知的区域，灰白到全白表示可能是空闲的区域，灰黑到黑色表示可能是障碍物。整个图将与当前机器工作的环境相符。格子的颜色在实时变化，可能是颜色的深浅变化（表示概率在增加或者减小），也可能是黑色白色的颜色变化（表示障碍物的出现或者消失）。算法内部对网格地图进行了美化，表现为将粗糙不平的灰黑到黑色的障碍物边缘细化，约束置一格到两格范围内，使网格地图的障碍物边缘看起来更平直细。进而可用于对点云地图进行美化，参考[当前点云地图](#当前点云地图)。



出错：

1. 完全没有网格地图，表示没有获取地图事件。一般出现在刚开机。可能是没有完成一次slam，可能是输入的点云数据出错，原因见上述[当前一帧的点云](#当前一帧的点云)，可能是slam的计算结果出错，无法发布该事件。
2. 其他情况一般是算法本身的问题，而不是网格地图的错误，因为网格地图仅用作slam的后端显示结果，并不参与slam的过程，不会影响slam算法。至于网格地图确实与实际环境不符，导致的其他模块的出错，应当分析导致地图出错的可能原因。

## 当前机器位姿

文件：`CManager.cpp`

事件：`SLAM_SLAM_POSE`

变量：`m_lastPose`

画图代码：

```c++
// 局部
->registerHandler(comm::socket::EMessageType::POSE, [this](std::shared_ptr<mrpt::utils::CMessage> message)
            {
                std::lock_guard<mutex> lock{m_mutex};
                message->serializeObject(&m_lastPose);
                return message;
            })
```

图示：机器位姿本身就是一个二维点和一个方向，为了便于观察，在上位机上当前位姿的地方画了一个图示，放大了点的位置和位姿的方向。当前图示为一个透明的红色三角箭头板子，三角箭头的尾巴凹槽代表位姿的二维点，三角箭头的头部朝向代表位姿的方向。三角箭头将随着机器同步移动。



出错：

1. 三角箭头在起点位置，一直没有移动。一般发生在刚开机。可能是slam没有发布有效位姿的事件，导致没有正确的当前位姿。需要检查slam的计算输入或者结果是否出错。
2. 三角箭头停在某个位置不动，一直不动或者等一会儿突然跳变到另一个位置继续移动。一般发生在机器运动到不平的地面，比如斜坡、爬上了门槛等，此时由于倾斜将无法进行slam，只能根据里程计进行定位，对应的`SLAM_SLAM_POSE`事件由于没有更新数据将暂停发布，等到机器回到平地上，slam恢复运作，对应的`SLAM_SLAM_POSE`事件将继续发布，此时机器有可能已经跑到另一个地方，显示为三角箭头的跳变。
3. 位姿也是slam算法的计算结果的显示，有任何位姿出错的现象都应该返回slam算法检查计算过程。

## 当前机器轨迹

文件：`CManager.cpp`

事件：`SLAM_SLAM_POSE`

变量：`m_lastPose`

画图代码：

```c++
static size_t count = 0;
if(++count % 2 == 0)
{
    GetInstance<CControllerGraph>()->addTrack(pose);
}
```

图示：天蓝色的线段，从机器起点位置起，按照一定采样频率降低位姿数据量，连接出机器的行走轨迹。轨迹的末端表示机器的当前位置。轨迹线只有二维位置值，没有方向信息。轨迹严格反映了slam算法得出的机器位姿，并不完全与机器实际位姿重叠。存在误差，或者计算错误。



出错：

1. 完全没有轨迹。需要检查当前位姿是否正常变化，如果有变化的位姿，即变化的三角箭头，表示slam算法正常，且发出了位姿事件，但是画轨迹的部分出错。 

## 美化后的轨迹

文件：输入`ServiceActor.cpp`输出`CMqttManager.cpp`

事件：输入`ROBOT_POSE_WITH_STATUS`输出`ROBOT_TRACK`

变量：`m_trackPoseList`

画图代码：

```c++
// ServiceActor.cpp美化轨迹
refineTrack(pose, motionType, targetPose, realNP2P);

// CMqttManager.cpp打包发送给手机端
pathUpload = pathCompress(path);
upload_path(&pathUpload[0], pathUpload.size());
```

图示：美化后的轨迹主要用于给手机端显示。轨迹平行且均匀。也可以在上位机上显示。



出错：

1. 美化后的轨迹与真实轨迹不符，表现为美化后的轨迹杂乱。可能是美化轨迹算法中对运动状态的衔接条件缺失。

## 当前点云地图

文件：`SlamActor.cpp`

事件：无，actor内部调试

变量：`slamResult.pointsMapPtr`

画图代码：

```c++
vector<CPose2D> pointsList;
mrpt::maps::CSimplePointsMap pointsMap = *slamResult.pointsMapPtr;
float x, y;
for (size_t i = 0; i < pointsMap.size(); i++)
{
    pointsMap.getPoint(i, x, y);
    pointsList.emplace_back(CPose2D(x, y, 0));
}
GetInstance<CGraph>()->addPointCloud("new points map", pointsList, CRenderUtils::VIOLET, 2, 0.03);
```

图示：当前点云地图是`VIOLET`紫色的很小的点云集。由于点云地图数量较大，一般在100个点以上，因此用很小的点来画图。显示点云地图目的是观察当前点云地图的分布情况，因为点云地图是slam算法真正用于匹配的地图，而网格地图只是用于显示的地图，因此点云地图的粗糙程度影响了slam算法匹配的精确度。算法内部对点云地图进行了美化。在美化后的网格地图的基础上，对网格地图中灰色和白色以及浅灰黑网格上分布的点云进行删除，降低点云数量和杂散点云。在图示上能够持续观察到，点云地图先逐渐分散，再突变去除杂散点，再继续逐渐分散，如此循环。



出错：

1. 点云地图完全是空的，或者一开始有点云地图，一段时间后点云地图消失了。可能是美化参数设置的过于低，将符合条件的所有点云都清空了。
2. 一开始有点云地图，一段时间后点云地图没有进一步扩充，但是机器明显移动到新的环境中了。可能是地图的更新开关`m_enableMapUpdating`参数传递出错，强制关闭了点云地图的更新。目前算法中仅在覆盖时强制关闭了点云地图的更新。
3. 点云地图多用于观察其美化程度，上述两个出错情况可以通过其他log查询到。

## 定位出错提示

文件：`SlamActor.cpp`

事件：无，actor内部调试

变量：`slamResult.goodness`

画图代码：

```c++
if (slamResult.goodness < 0.55 && ++countPoseError >= 5)
{
    SLAM_LOG_WARN << "slam error...error pose(" << countPoseError << ")" << endl;
    if (5 == countPoseError)
    {
        lidarForSlam->drawScan(undistortedScan, m_slamPose, CRenderUtils::BLUEVIOLET, 4, 0.04f);
    }   
}
SLAM_LOG_INFO << "is error pose(" << countPoseError << ")" << endl;
```

图示：定位出错有一定阈值的计数，当达到计数条件后，认为发生定位出错，将此时的雷达点云画在当前错误的输出位姿上，用`BLUEVIOLET`蓝紫色的点表示。这一帧点云将不会与当前网格地图的障碍物边界重合，表示当前位姿出错。如果后续位姿恢复正确，该图示不会被覆盖清除。如果发生第二次定位出错，该图示会被新的定位出错时的雷达点云覆盖。



出错：

1. 可能存在误判。仅用做辅助显示，出现该图示后说明定位发生异常，slam算法出错，一般情况下无法继续运行slam，需要比如重定位的辅助将错误校正。出现该图示后slam有可能自动恢复到正确的位置，有可能永远无法恢复直到引起其他异常。所有的数据失去意义。

## 重定位初始范围

文件：`CRelocalization.cpp`

事件：无，内部调试

变量：初始化范围

画图代码：

```c++
// 局部重定位初始范围
{
    vector<CPose2D> mclRect;
    mclRect.emplace_back(CPose2D(x_min, y_min, 0.0));
    mclRect.emplace_back(CPose2D(x_min, y_max, 0.0));
    mclRect.emplace_back(CPose2D(x_max, y_max, 0.0));
    mclRect.emplace_back(CPose2D(x_max, y_min, 0.0));
    mclRect.emplace_back(mclRect.front());

    GetInstance<CGraph>()->addLineSet("mcl rect", mclRect, CRenderUtils::SALMON, 0.1f);
}
// 全局重定位初始范围
{
    vector<CPose2D> mclRect;
    mclRect.emplace_back(CPose2D(m_initPDFMinX, m_initPDFMinY, 0.0));
    mclRect.emplace_back(CPose2D(m_initPDFMinX, m_initPDFMaxY, 0.0));
    mclRect.emplace_back(CPose2D(m_initPDFMaxX, m_initPDFMaxY, 0.0));
    mclRect.emplace_back(CPose2D(m_initPDFMaxX, m_initPDFMinY, 0.0));
    mclRect.emplace_back(mclRect.front());

    GetInstance<CGraph>()->addLineSet("mcl rect", mclRect, CRenderUtils::SALMON, 0.1f);
}      
```

图示：用`SALMON`皮粉色表示的矩形。矩形范围有两个尺寸，一种是全局尺寸，从配置文件读取的数据范围[-10m, 10m]，以原点(0, 0)为中心；另一种是局部尺寸，从配置文件读取的数据范围[pose.x-1.5m, pose.x+1.5m]，以当前参考位姿pose为中心。完成重定位初始化后能够看到该图示。



出错：

1. 没有该矩形框。可能是没有进入重定位逻辑，可能是没有进行重定位初始化，可能是原本出现过，完成重定位后图示被清除了。
2. 矩形框的尺寸不对，比如原本应该发生局部重定位，但是出现了全局矩形框。可能是传入参数出错，可能是判断条件出错。



## 重定位粒子分布

文件：`SlamActor.cpp`

事件：无，内部调试

变量：粒子（详细概念请另外参考重定位资料）

画图代码：

```c++
GetInstance<CGraph>()->addParticles("particles", m_relocalizationPtr->getMCL(), CRenderUtils::ROSYBROWN, -0.1f);
```

图示：用`ROSYBROWN`棕色的图表示重定位的粒子。在重定位的每次循环中，都会产生一定的粒子。粒子理解为带方向的点，比如点+箭头，表示位姿。类似于点云，粒子有一定的数量，从配置文件中读取的，比如1300个。每一个粒子表示一个位姿，在输入一帧点云的条件下，从所有的粒子中遍历找到能够让这一帧点云尽可能与网格地图重叠的位姿。粒子分布范围初始时均匀分布在上述初始范围[重定位初始范围](#重定位初始范围)中的白色格子里，随着循环变化，逐渐收敛到较小的范围里。



出错： 

1. 粒子初始分布没有按照初始范围均匀分布，可能是配置参数出问题。
2. 粒子分布没有从均匀分布到逐渐收敛，可能需要更多的循环次数，可能一次循环就完成了重定位，打断了收敛的过程。
3. 粒子分布有部分超出了初始范围，正常的，算法中的随机噪声引起的。

## 重定位估计点云

文件：`SlamActor.cpp`

事件：无，内部调试

变量：`undistortedScan`和`mclPose`

画图代码：

```c++
lidarForSlam->drawScan(undistortedScan, mclPose, CRenderUtils::GREENYELLOW, 4, 0.01f);
```

图示：用`GREENYELLOW`的点云表示重定位的每次循环结束时，将当前输入的雷达点云画在由上述[重定位粒子分布](#重定位粒子分布)的粒子中筛选出的最可能的位姿处。这是一个猜测的位姿，所以看到的实际点云有可能与网格地图的障碍物边缘大致重叠，也有可能完全不重叠。



出错：

1. 该图示用于显示猜测的结果，无所谓正确与否。有可能很快就猜到了比较正确的位姿，也有可能完全在乱猜。
2. 图过该图示出现过，但是重定位结束后消失了，可能是被系统清除了。

## 重定位校正点云

文件：`SlamActor.cpp`

事件：无，内部调试

变量：`undistortedScan`和`icpPose`

画图代码：

```c++
lidarForSlam->drawScan(undistortedScan, icpPose, CRenderUtils::GOLD, 4, 0.02f);
```

图示：用`GOLD`的点云表示重定位的每次循环结束时，将当前输入的雷达点云画在由上述[重定位估计点云](#重定位估计点云)中提到的猜测位姿经过一定次数的icp匹配后优化的位姿上。即对上述猜测位姿的进一步优化。如果上述猜测位姿比较接近真实位姿，经过icp匹配优化后的位姿将更接近真实位姿；如果上述猜测位姿距离真实位姿差别很大，经过icp匹配对位姿的优化效果几乎没有。如果最终结果是重定位成功，那么该图示将与网格地图的障碍物边缘完全重叠。反之，如果最终结果是重定位失败，该图示将与网格地图的障碍物边缘完全部相关联。



出错：

1. 可能出现一定的误判情况。在相似环境中，显示该图示与网格地图的另一个区域有很大程度的重叠，但并不是正确的区域，重定位最终结果是成功。
2. 大部分时候正常的情况下，可以直接根据该图示得到重定位的最终结果。少数时候，可以根据该图示得到重定位的误判情况。
3. 如果该图示没有出现过，可能是输入的雷达点云出错，可能是进入icp匹配后没有跳出循环。
4. 图过该图示出现过，但是重定位结束后消失了，可能是被系统清除了。









## 打滑检测

文件：`SlamActor.cpp`

事件：`EXCEPTION_SLIP`

变量：`m_slamPose`

画图代码：

```c++
slipPoints.emplace_back(m_slamPose);
int count = 0;
for (const auto& i : slipPoints)
{
    GetInstance<CGraph>()->addDisk("slip points in slam"+to_string(count++), i, 0.15f, 0.15f + 0.005f, CRenderUtils::ORANGERED, 0.1f);
}
```

图示：用`ORANGERED`的圆环表示发生打滑的那个位姿，圆环中心为发生打滑点。此处的发生打滑是指接收到打滑事件。该图示用于显示打滑事件发出的位置，如果接收到新的打滑事件，将当前机器位姿存储到点集中，再依次画出这些打滑事件发生点处的圆环。表示在地图的这些点集的位置检测到了打滑。用于分析打滑是否正常发生，或者误触发。



出错：

1. 有一定的误判可能。正常情况下，当机器卡在某个位置，轮子不断转动，但是机身没有移动，应当被视为发生了打滑，在图中将对应出现圆环。如果没有出现圆环，需要检查打滑数据，分析是否计数没有达到阈值。如果机器本身正常行走，在图中出现了圆环，说明发生了误判，需要检查打滑数据，分析是否匹配过程产生了误判。
2. 也可以用于分析打滑事件发生后其他处理是否正确及时。

## 走廊检测

文件：`SlamActor.cpp`

事件：无，内部调试

变量：`undistortedScan`和`m_slamPose`

画图代码：

```c++
static vector<pair<mrpt::poses::CPose2D, mrpt::poses::CPose2D>> pointList;
if (undistortedScan.getFrontValidRatio() < 0.015 && undistortedScan.getBackValidRatio() < 0.015)
{
    CPose2D endPose;
    endPose.x() = m_slamPose.x() + 0.1 * cos(m_slamPose.phi());
    endPose.y() = m_slamPose.y() + 0.1 * sin(m_slamPose.phi());
    pointList.emplace_back(make_pair(m_slamPose, endPose));
}
GetInstance<CGraph>()->addArrowSet("front end invalid points", pointList, CRenderUtils::VIOLET, 0.12f);
```

图示：用`VIOLET`的线段表示的箭头表示机器位姿位于走廊区域。根据输入的当前帧的雷达点云，获取内部计算的机器正前方和正后方一定范围内雷达点云的有效比例，当机器正前方和正后方的雷达点云有效比例均小于阈值，将这种情况判断为机器在沿着走廊移动。由于走廊的环境容易引起icp匹配误差，进而导致上述[打滑检测](#打滑检测)误触发，于是首先检测出走廊，然后调整走廊区域的打滑检测的计数阈值，来降低走廊处打滑检测误触发的概率。



出错：

1. 仅用作显示结果，如果发生与实际情况不符的图示，可能是阈值设置的问题。

