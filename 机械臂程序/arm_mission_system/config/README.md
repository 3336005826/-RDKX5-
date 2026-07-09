# config

这个目录放机械臂抓取配置。

## grasp_profiles.yaml

当前模型类别已经改成：

- `cardboard`
- `glass`
- `metal`
- `organic`
- `paper`
- `plastic`

这个文件里包含：

- 每一类的抓取高度
- 默认抓取高度
- 标签别名映射
- 默认投放模式

### 最常调的字段

每个类别都可以调：

- `approach_z`
- `grasp_z`

建议你先重点调：

- `plastic`
- `paper`
- `metal`
- `organic`
