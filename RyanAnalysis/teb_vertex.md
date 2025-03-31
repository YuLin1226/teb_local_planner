在圖優化的時候，速度約束會依照 連續兩個 Pose 及相對的時間差來計算速度。
為了滿足這個目的，TEB 的頂點容器設計成： Pose, Time, Pose, Time, Pose, Time, Pose, Time, Pose 這樣交叉的順序。

```cpp=
void TebOptimalPlanner::AddTEBVertices()
{
  // 添加顶点到图中
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // 用于顶点的索引
  obstacles_per_vertex_.resize(teb_.sizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();
  
  
  // pose 頂點的數量 > time_diff 頂點的數量
  // 所以裡面用 if 也先把 time_diff 頂點塞入。Optimizer 的頂點順序就會變成：p, d, p, d, 這樣（p: pose | d: time_diff）
  // 這個頂點順序關係一定是交替的，因為 TEB 後續會根據交替的關係來建立速度約束。
  // 然後在 addPoseAndTimeDiff 的時候，也有說明 要先加入一個 Pose Vertex (通常代表起點)，然後後續再一次加入 1 Pose & 1 TimeDiff 頂點。
  for (int i=0; i<teb_.sizePoses(); ++i)
  {
    teb_.PoseVertex(i)->setId(id_counter++); // 先记录PoseVertex的id
    optimizer_->addVertex(teb_.PoseVertex(i));  // 再添加位姿顶点
    if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
    {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));  // 添加时间差顶点
    }
    iter_obstacle->clear();
    (iter_obstacle++)->reserve(obstacles_->size());
  }
}
```