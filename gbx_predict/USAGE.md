# 使用指南 - GBX Predict 图像拼接包

## 快速开始

### 1. 编译和安装

```bash
cd ~/predict_ws
colcon build --packages-select gbx_predict
source install/setup.bash
```

### 2. 启动节点

```bash
# 使用测试配置启动
ros2 launch gbx_predict test_image_stitch.launch.py

# 或者使用完整配置启动
ros2 launch gbx_predict image_stitch.launch.py
```

### 3. 检查节点状态

```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看节点参数
ros2 param list /image_stitch_node
```

## 参数调试

### ROI参数调整

ROI（感兴趣区域）参数决定了从输入图像中提取哪个区域进行拼接：

```bash
# 调整ROI位置和大小（值为图像尺寸的比例，0.0-1.0）
ros2 param set /image_stitch_node roi_x_offset_ratio 0.15    # X偏移
ros2 param set /image_stitch_node roi_y_offset_ratio 0.1     # Y偏移  
ros2 param set /image_stitch_node roi_width_ratio 0.6       # 宽度
ros2 param set /image_stitch_node roi_height_ratio 0.7      # 高度
```

### 拼接参数调整

```bash
# 调整拼接方向
ros2 param set /image_stitch_node stitch_along_y true   # true=纵向拼接, false=横向拼接

# 调整位移阈值
ros2 param set /image_stitch_node min_shift 2          # 最小位移（像素）
ros2 param set /image_stitch_node max_shift 150        # 最大位移（像素）

# 限制全景图大小
ros2 param set /image_stitch_node max_width 5000       # 最大宽度（像素）
```

### 特征检测参数

```bash
# 调整ORB特征检测
ros2 param set /image_stitch_node orb_features 1500    # 特征点数量
ros2 param set /image_stitch_node match_ratio 0.8     # 匹配阈值
ros2 param set /image_stitch_node min_matches 6       # 最小匹配点数
```

### 重置全景图

```bash
# 重置当前全景图
ros2 param set /image_stitch_node reset_now true
```

## 可视化调试

### 查看调试图像

```bash
# 查看带ROI标注的原始图像
ros2 run rqt_image_view rqt_image_view /debug_image

# 查看拼接结果
ros2 run rqt_image_view rqt_image_view /stitched_image

# 查看特征匹配可视化
ros2 run rqt_image_view rqt_image_view /debug_image/matches
```

### 使用rqt进行参数调整

```bash
# 启动参数配置工具
ros2 run rqt_reconfigure rqt_reconfigure
```

## 话题重映射

如果您的相机话题名称不同，可以在启动时重映射：

```bash
ros2 launch gbx_predict test_image_stitch.launch.py \
  input_topic:=/your_camera/image_raw \
  output_topic:=/your_stitched_image \
  debug_topic:=/your_debug_image
```

## 常见问题解决

### 1. 无特征点检测
- 检查输入图像是否有足够纹理
- 增加 `orb_features` 参数值
- 调整ROI区域，确保包含有纹理的部分

### 2. 拼接不连续
- 调整 `match_ratio` 参数（降低值使匹配更严格）
- 增加 `min_matches` 参数
- 检查相机运动是否过快

### 3. 全景图过大
- 设置合适的 `max_width` 参数
- 启用 `auto_reset` 自动重置

### 4. 拼接方向错误
- 根据相机运动方向设置 `stitch_along_y` 参数
- true = 纵向拼接（相机上下运动）
- false = 横向拼接（相机左右运动）

## 性能优化

### 减少计算负载
```bash
# 减少特征点数量
ros2 param set /image_stitch_node orb_features 500

# 调整ROI大小（更小的ROI处理更快）
ros2 param set /image_stitch_node roi_width_ratio 0.5
ros2 param set /image_stitch_node roi_height_ratio 0.5
```

### 提高拼接质量
```bash
# 增加特征点数量
ros2 param set /image_stitch_node orb_features 2000

# 提高匹配精度
ros2 param set /image_stitch_node match_ratio 0.7
ros2 param set /image_stitch_node min_matches 8
``` 