# GBX Predict - ROS2 Image Stitching and Detection Package

这是一个ROS2图像拼接和检测包，基于原始的ROS1 foam stitching节点改进而来。

## 功能特性

- **ROI提取**: 从输入图像中提取感兴趣区域（ROI）
- **实时拼接**: 使用ORB特征检测和匹配进行图像拼接
- **动态参数**: 支持运行时调整ROI和拼接参数
- **调试可视化**: 提供多种调试话题，包括ROI标注、特征匹配可视化等
- **双向拼接**: 支持水平和垂直方向的图像拼接

## 节点说明

### image_stitch_node

主要的图像拼接节点，订阅图像话题并进行ROI提取和拼接处理。

#### 订阅话题
- `input_topic` (sensor_msgs/Image): 输入图像话题

#### 发布话题
- `output_topic` (sensor_msgs/Image): 拼接后的全景图像
- `debug_topic` (sensor_msgs/Image): 带ROI标注的调试图像
- `debug_topic/roi` (sensor_msgs/Image): ROI区域图像
- `debug_topic/matches` (sensor_msgs/Image): 特征匹配可视化图像

#### 参数

##### ROI参数
- `roi_x_offset_ratio` (double, 默认: 0.1): X偏移比例
- `roi_y_offset_ratio` (double, 默认: 0.1): Y偏移比例
- `roi_width_ratio` (double, 默认: 0.7): ROI宽度比例
- `roi_height_ratio` (double, 默认: 0.8): ROI高度比例

##### 拼接参数
- `min_shift` (int, 默认: 1): 最小位移阈值（像素）
- `max_shift` (int, 默认: 200): 最大位移阈值（像素）
- `max_width` (int, 默认: 10000000): 最大全景宽度（像素）
- `auto_reset` (bool, 默认: false): 自动重置全景图
- `reset_now` (bool, 默认: false): 立即重置全景图
- `stitch_along_y` (bool, 默认: true): 沿Y轴拼接（纵向）

##### 特征检测参数
- `orb_features` (int, 默认: 1000): ORB特征点数量
- `match_ratio` (double, 默认: 0.75): Lowe比率测试阈值
- `min_matches` (int, 默认: 4): 最小匹配点数量

## 使用方法

### 1. 编译包

```bash
cd ~/your_ros2_ws
colcon build --packages-select gbx_predict
source install/setup.bash
```

### 2. 启动节点

使用默认参数启动：
```bash
ros2 launch gbx_predict image_stitch.launch.py
```

指定输入话题启动：
```bash
ros2 launch gbx_predict image_stitch.launch.py input_topic:=/your_camera/image_raw
```

### 3. 运行时参数调整

使用rqt_reconfigure调整参数：
```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

或者使用命令行设置参数：
```bash
# 调整ROI参数
ros2 param set /image_stitch_node roi_width_ratio 0.8
ros2 param set /image_stitch_node roi_height_ratio 0.6

# 调整拼接参数
ros2 param set /image_stitch_node min_shift 5
ros2 param set /image_stitch_node max_shift 150

# 重置全景图
ros2 param set /image_stitch_node reset_now true
```

### 4. 查看结果

查看拼接结果：
```bash
ros2 run rqt_image_view rqt_image_view /stitched_image
```

查看调试图像：
```bash
ros2 run rqt_image_view rqt_image_view /debug_image
```

查看特征匹配：
```bash
ros2 run rqt_image_view rqt_image_view /debug_image/matches
```

## 配置文件

参数配置文件位于 `config/image_stitch_params.yaml`，可以根据需要修改默认参数值。

## 算法原理

1. **ROI提取**: 根据设定的比例参数从输入图像中提取感兴趣区域
2. **特征检测**: 使用ORB算法检测特征点
3. **特征匹配**: 使用BFMatcher进行特征匹配，并应用Lowe比率测试
4. **变换估计**: 使用RANSAC算法估计仿射变换
5. **图像拼接**: 根据估计的位移量进行图像拼接

## 注意事项

- 确保输入图像质量良好，具有足够的纹理特征
- 根据实际应用场景调整ROI参数
- 拼接方向（水平/垂直）需要根据相机运动方向设置
- 特征检测参数需要根据图像内容进行调优

## 故障排除

1. **无法检测到特征点**: 检查图像质量，增加ORB特征点数量
2. **拼接效果不佳**: 调整匹配阈值和最小匹配点数量
3. **全景图过大**: 设置合适的最大宽度限制
4. **实时性能问题**: 减少特征点数量或降低输入图像分辨率 