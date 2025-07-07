# 参数优先级说明

## ROS2参数优先级顺序

在我们的图像拼接包中，参数的优先级从高到低为：

1. **Launch文件中的parameters字典** (最高优先级)
2. **命令行参数** 
3. **YAML配置文件**
4. **节点代码中的默认值** (最低优先级)

## 当前配置分析

### 配置文件 (config/image_stitch_params.yaml)
```yaml
image_stitch_node:
  ros__parameters:
    input_topic: "/hk_camera/cs050/image_raw"
    output_topic: "/stitched_image"
    debug_topic: "/debug_image"
    # ... 其他参数
```

### Launch文件 (launch/image_stitch.launch.py)
```python
parameters=[
    LaunchConfiguration('config_file'),  # 加载YAML文件
    {
        'input_topic': LaunchConfiguration('input_topic'),    # 会覆盖YAML中的值
        'output_topic': LaunchConfiguration('output_topic'),  # 会覆盖YAML中的值
        'debug_topic': LaunchConfiguration('debug_topic'),    # 会覆盖YAML中的值
    }
]
```

## 实际效果

由于launch文件中的参数字典优先级更高，最终生效的参数值为：

- 如果启动时指定了launch参数：使用launch参数值
- 如果启动时未指定launch参数：使用launch参数的default_value
- YAML文件中的相同参数会被覆盖

## 使用示例

### 1. 使用默认配置启动
```bash
ros2 launch gbx_predict image_stitch.launch.py
```
结果：使用launch文件中的default_value (`/hk_camera/cs050/image_raw`)

### 2. 通过命令行覆盖话题
```bash
ros2 launch gbx_predict image_stitch.launch.py input_topic:=/my_camera/image_raw
```
结果：使用命令行指定的值 (`/my_camera/image_raw`)

### 3. 使用不同的配置文件
```bash
ros2 launch gbx_predict image_stitch.launch.py config_file:=/path/to/my_config.yaml
```
结果：加载指定的YAML文件，但launch参数仍会覆盖相应的值

## 推荐的配置策略

### 策略1：主要使用YAML配置
- 在YAML文件中设置所有参数的默认值
- Launch文件只提供必要的覆盖能力
- 适合：参数相对固定的场景

### 策略2：Launch文件为主
- 在launch文件中设置常用参数的默认值
- YAML文件用于高级参数配置
- 适合：需要频繁更改话题名称的场景

### 策略3：环境特定配置
- 为不同环境创建不同的YAML配置文件
- Launch文件保持通用性
- 适合：多环境部署的场景

## 避免混淆的建议

1. **保持一致性**：在YAML和launch文件中使用相同的默认值
2. **明确文档**：在注释中说明哪些参数会被覆盖
3. **使用命名空间**：为不同的配置组使用不同的命名空间
4. **测试验证**：启动后检查实际生效的参数值

## 检查实际参数值

启动节点后，可以通过以下命令检查实际生效的参数：

```bash
# 查看所有参数
ros2 param list /image_stitch_node

# 查看特定参数的值
ros2 param get /image_stitch_node input_topic

# 查看所有参数的值
ros2 param dump /image_stitch_node
``` 