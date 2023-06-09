# 华为软挑赛2023复赛-西北赛区第5名

![](https://github.com/leozjr/CodeCraft/blob/master/Docs/banner.png)

- 使用P控制（无积分微分）进行循迹控制
- 使用VO进行机器人间避障（仅初赛）
- 决策算法为贪心算法
- 复赛路径规划为A*，路径规划在初始化5秒内完成，之后不再改动

## 复赛阻塞控制思路

机器人记录自己走过的路(PastRoad)和未来要走的路(FutureRoad)，每走一个点就把FutureRoad中的点转移到PastRoad中，如果两机器人的FutureRoad重合，那么视为阻塞，按照优先级，一方进行回退操作，即调头走回头路，FutureRoad和PastRoad相应做回滚操作，直到找到不和对方路径重合的一块空间进行避让（前往避让路径和返回记为ParkFutureRoad和ParkPastRoad），避让过程中监视对方机器人，如果对方机器人通过阻塞路段，即可继续行驶。

> 注意： 回滚过程可能与其他机器人发生冲突，因此在判断阻塞时，每个机器人的FutureRoad不仅要和其他机器人的FutureRoad对比，还要和避让状态的ParkRoad对比，同时设置合适的优先级，进行多方避让。

## 其他细节
- 机器人靠墙行驶时会卡墙，因此路径周围靠近墙壁时，会对路径做偏移处理，尽量靠近卡口的正中心
- 机器人到达目标工作台前1米会减速，防止撞墙
- 机器人行进过程中，如果前方有人，会刹车保持间距（KeepDistance函数）
- 卡顿检查，如果两机器人距离非常近并且速度为0，说明陷入死锁，需要进行解锁（尚未完善）

## 未实现
- 路径优化：如果两点之间直接可达，则可以直接直线行驶，需要对搜索出来的路径进行直线优化，未做（需要考虑算力问题）
- 动态路径：正确的路径规划方式应该是阻塞后重新搜索可行路径，经过与前几名的交流确认动态路径规划是可行的，是最正确的选择。

## 证书

![](https://github.com/leozjr/CodeCraft/blob/master/Docs/证书.jpg)

## 复赛训练赛地图

![](https://github.com/leozjr/CodeCraft/blob/master/Docs/图1.png)

![](https://github.com/leozjr/CodeCraft/blob/master/Docs/图2.png)

![](https://github.com/leozjr/CodeCraft/blob/master/Docs/图3.png)

![](https://github.com/leozjr/CodeCraft/blob/master/Docs/图4.png)


