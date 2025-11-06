# Data Collection Service Package

机器人数据存储包，用于收集和存储来自robot_interface的机器人数据，支持VR按钮控制和多频率分层数据采集。

## 功能特性

- **VR按钮控制**: 通过VR左手柄按钮手动控制数据记录开始/停止和会话数据保存
- **多频率分层采集**: 支持不同频率的数据源同步采集（30Hz主循环，10Hz各数据源）
- **多格式存储支持**: 支持HDF5、CSV、JSON、Pickle等多种存储格式
- **实时数据订阅**: 订阅robot_interface发布的关节角度和笛卡尔位姿数据
- **VR数据集成**: 集成VR左手柄按钮数据，支持实时状态监控
- **灵活数据处理**: 支持数据过滤、平滑、异常值检测等处理功能
- **可配置字段选择**: 通过配置文件指定需要存储的数据字段
- **自动数据管理**: 支持自动文件清理、缓冲区管理等功能
- **性能监控**: 提供详细的数据处理统计信息和实时状态监控
- **数据验证**: 自动验证数据完整性和质量

## 包结构

```
data_collection_service/
├── config/                    # 配置文件
│   └── data_collection_service_config.yaml
├── data_collection_service/             # Python包
│   ├── __init__.py
│   ├── data_subscriber.py    # 数据订阅器
│   ├── data_processor.py     # 数据处理器
│   ├── storage_manager.py    # 存储管理器
│   ├── vr_button_controller.py  # VR按钮控制器
│   └── data_collection_service_node.py  # 主节点
├── launch/                   # 启动文件
│   └── data_collection_service.launch.py
├── scripts/                  # 可执行脚本
│   └── data_collection_service_node
├── else/                     # 额外资源和文档
│   ├── COMPREHENSIVE_INTEGRATION_GUIDE.md  # 综合集成指南
│   ├── VR_BUTTON_INTEGRATION.md           # VR按钮集成说明
│   ├── VR_BUTTON_CONTROL_GUIDE.md         # VR按钮控制指南
│   ├── ROBOT_INTERFACE_INTEGRATION.md     # 机器人接口集成说明
│   ├── scripts/                           # 测试和工具脚本
│   └── examples/                          # 使用示例
├── test/                     # 测试文件
├── package.xml
├── CMakeLists.txt
├── setup.py
└── README.md
```

## 安装依赖

```bash
# 安装Python依赖
pip install h5py numpy

# 如果使用其他存储格式
pip install pandas  # 用于CSV处理

# VR相关依赖（如果使用VR按钮控制）
pip install json  # 用于VR数据文件解析
```

## 配置说明

### 基本配置

配置文件位于 `config/data_collection_service_config.yaml`，主要包含以下配置项：

1. **VR按钮控制配置** (`vr_button_control`):
   - `enable_vr_button_control`: 启用VR按钮控制
   - `vr_data_file`: VR数据文件路径

2. **记录控制配置** (`recording`):
   - `auto_record`: 自动记录模式（VR按钮控制模式下建议设为false）

3. **存储配置** (`storage`):
   - `storage_directory`: 数据存储目录
   - `storage_format`: 存储格式 (hdf5, csv, json, pickle)
   - `buffer_size`: 缓冲区大小
   - `flush_interval`: 缓冲区刷新间隔

4. **数据源配置** (`data_sources`):
   - `joint_angles`: 关节角度数据配置
   - `cartesian_pose`: 笛卡尔位姿数据配置
   - `vr_left_buttons`: VR左手柄按钮数据配置

5. **数据处理配置** (`processing`):
   - 数据过滤设置
   - 异常值检测
   - 数据转换选项

6. **存储格式配置** (`storage_formats`):
   - HDF5压缩设置
   - CSV分隔符设置
   - JSON格式化选项

### VR按钮控制配置示例

```yaml
vr_button_control:
  enable_vr_button_control: true
  vr_data_file: "/home/airspeedbox/code/vr_robot_control_ws/src/vr_teleoperation/vr_data.json"

recording:
  auto_record: false  # VR按钮控制模式下建议设为false
```

## 使用方法

###快启动
cd /home/airspeedbox/code/vr_robot_control_ws/src/data_collection_service
./launch.sh

### 1. 系统启动

在首次使用或修改该包后，请在ROS 2工作空间中构建并加载环境：

```bash
cd /home/airspeedbox/code/vr_robot_control_ws
colcon build --packages-select data_collection_service robot_interface
source install/setup.bash
```

```bash
# 启动数据存储节点
ros2 launch data_collection_service data_storage.launch.py

# 使用默认配置启动
ros2 launch data_collection_service data_collection_service.launch.py

# 指定存储目录
ros2 launch data_collection_service data_collection_service.launch.py storage_directory:=/home/airspeedbox/code/vr_robot_control_ws/src/data_collection_service/data

# 指定配置文件
ros2 launch data_collection_service data_collection_service.launch.py config_file:=/path/to/config.yaml

# 指定存储格式
ros2 launch data_collection_service data_collection_service.launch.py storage_format:=csv
```

### 2. VR按钮控制

#### 手动控制流程
1. **开始记录**：按下VR左手柄第六个按钮
   - 日志显示：`VR Button 6: Recording started`
2. **存储数据**：按下VR左手柄第五个按钮
   - 日志显示：`VR Button 5: Session data saved`
3. **停止记录**：再次按下VR左手柄第六个按钮
   - 日志显示：`VR Button 6: Recording stopped`

#### 按钮功能映射
- **第六个按钮 (Button 6)**：开始/停止记录数据
- **第五个按钮 (Button 5)**：存储当前会话数据

### 3. 命令行控制（备选方案）

```bash
# 开始记录
ros2 topic pub /data_collection_service/control std_msgs/String "data: 'start'"

# 停止记录
ros2 topic pub /data_collection_service/control std_msgs/String "data: 'stop'"

# 暂停记录
ros2 topic pub /data_collection_service/control std_msgs/String "data: 'pause'"

# 恢复记录
ros2 topic pub /data_collection_service/control std_msgs/String "data: 'resume'"

# 刷新缓冲区
ros2 topic pub /data_collection_service/control std_msgs/String "data: 'flush'"

# 查看状态
ros2 topic pub /data_collection_service/control std_msgs/String "data: 'status'"

# 查看统计信息
ros2 topic pub /data_collection_service/control std_msgs/String "data: 'stats'"
```

### 4. 监控系统状态

```bash
# 查看记录状态
ros2 topic echo /data_collection_service/status

# 查看统计信息
ros2 topic echo /data_collection_service/statistics

# 查看节点信息
ros2 node info /data_collection_service_node

# 查看话题频率
ros2 topic hz /data_collection_service/status

# 检查VR按钮状态
python3 src/data_collection_service/else/scripts/check_vr_button_status.py
```

## 数据格式

### 关节角度数据
- **话题**: `/robot_joint_angles`
- **消息类型**: `std_msgs/Float32MultiArray`
- **数据格式**: `[j1, j2, j3, j4, j5, j6]` (度)
- **频率**: 10Hz

### 笛卡尔位姿数据
- **话题**: `/robot_cartesian_pose`
- **消息类型**: `geometry_msgs/PoseStamped`
- **数据格式**: 位置 (x, y, z) 和四元数 (x, y, z, w)
- **频率**: 10Hz

### VR左手柄按钮数据
- **话题**: `/target_left_arm_pose`
- **消息类型**: `geometry_msgs/PoseStamped`
- **数据格式**: 
  ```json
  {
    "button_5": 0,  // 第五个按钮状态 (0/1)
    "button_6": 0   // 第六个按钮状态 (0/1)
  }
  ```
- **频率**: 10Hz

### 多频率分层采集
- **主循环频率**: 30Hz
- **各数据源频率**: 10Hz
- **同步容忍度**: 10ms
- **最大时间差**: 100ms

## 存储格式说明

### HDF5格式
- 支持数据压缩
- 适合大量数据存储
- 支持元数据存储
- 文件扩展名: `.h5`
- 推荐用于机器人数据存储

### CSV格式
- 人类可读格式
- 适合数据分析
- 支持头部信息
- 文件扩展名: `.csv`
- 适合小规模数据导出

### JSON格式
- 结构化数据格式
- 支持嵌套数据
- 易于解析
- 文件扩展名: `.json`
- 推荐用于VR会话数据

### Pickle格式
- Python原生格式
- 支持复杂数据结构
- 二进制格式
- 文件扩展名: `.pkl`
- 适合Python环境下的数据交换

### 存储结构
- **常规数据**: `@data/` (源码目录下的data文件夹)
- **VR会话数据**: `@data/vr_sessions/` (VR按钮控制的会话数据)
- **机器人数据**: `@data/robot_data/` (机器人接口数据)

## 性能优化

1. **缓冲区管理**: 合理设置缓冲区大小和刷新间隔
2. **数据压缩**: 使用HDF5压缩减少存储空间
3. **文件清理**: 定期清理旧文件避免磁盘空间不足
4. **内存管理**: 监控内存使用情况
5. **多频率同步**: 优化数据同步算法减少延迟
6. **VR数据缓存**: 合理设置VR数据读取频率

## 测试功能

### VR按钮控制测试
```bash
# 运行VR按钮控制测试
python3 src/data_collection_service/else/scripts/test_vr_button_control.py

# 运行VR按钮简单测试
python3 src/data_collection_service/else/scripts/test_vr_button_simple.py

# 检查VR按钮状态
python3 src/data_collection_service/else/scripts/check_vr_button_status.py
```

### 机器人接口集成测试
```bash
# 运行集成测试脚本
python3 src/data_collection_service/else/scripts/test_robot_interface_integration.py

# 运行多频率采集测试
python3 src/data_collection_service/else/scripts/test_multi_frequency_collection.py

# 运行完整的多频率测试
./src/data_collection_service/else/scripts/run_multi_frequency_test.sh
```

### 综合功能测试
```bash
# 运行示例程序
python3 src/data_collection_service/else/examples/example_usage.py
```

## 故障排除

### 常见问题

1. **VR按钮无响应**
   - 检查VR数据文件是否存在：`/home/airspeedbox/code/vr_robot_control_ws/src/vr_teleoperation/vr_data.json`
   - 确认VR遥操作接口正在运行
   - 检查配置文件中的VR按钮控制是否启用

2. **机器人数据未接收**
   - 检查robot_interface节点是否运行
   - 验证话题连接：`ros2 topic list`
   - 检查话题数据：`ros2 topic echo /robot_joint_angles`

3. **数据未保存**
   - 确认在记录状态下按下第五个按钮
   - 检查存储目录权限
   - 查看日志中的错误信息

4. **记录状态异常**
   - 重启数据存储节点
   - 检查数据文件格式
   - 验证数据范围

5. **配置文件加载失败**
   - 检查配置文件路径
   - 验证YAML格式是否正确

6. **数据订阅失败**
   - 确认robot_interface节点正在运行
   - 检查话题名称是否正确

7. **存储空间不足**
   - 启用自动文件清理
   - 调整文件大小限制

8. **性能问题**
   - 调整缓冲区大小
   - 优化数据处理配置

### 调试模式

```bash
# 启用详细日志
ros2 launch data_collection_service data_collection_service.launch.py log_level:=debug

# 查看节点日志
ros2 log info /data_collection_service_node

# 查看话题连接
ros2 topic list
ros2 topic info /robot_joint_angles
ros2 topic info /robot_cartesian_pose
ros2 topic info /target_left_arm_pose
```

## 开发说明

### 添加新的存储格式
1. 在 `storage_manager.py` 中添加新的存储方法
2. 在配置文件中添加相应的格式配置
3. 更新文档说明

### 添加新的数据处理功能
1. 在 `data_processor.py` 中添加新的处理方法
2. 在配置文件中添加相应的处理配置
3. 更新文档说明

### 添加新的数据源
1. 在配置文件中添加新的数据源配置
2. 在 `data_subscriber.py` 中添加相应的订阅逻辑
3. 更新数据验证和处理流程

### 扩展VR按钮功能
1. 在 `vr_button_controller.py` 中添加新的按钮映射
2. 更新配置文件中的按钮配置
3. 添加相应的测试脚本

## 最佳实践

### 1. 系统配置
- 根据实际需求调整缓冲区大小
- 选择合适的存储格式
- 定期清理旧数据文件

### 2. 数据质量
- 启用数据验证功能
- 设置合适的异常检测阈值
- 定期检查数据完整性

### 3. 性能优化
- 监控系统资源使用
- 调整刷新间隔
- 优化存储路径

### 4. 故障预防
- 定期备份重要数据
- 监控磁盘空间
- 建立日志轮转机制

## 相关文档

- [综合集成指南](else/COMPREHENSIVE_INTEGRATION_GUIDE.md) - 完整的系统集成和使用指南


## 许可证

Apache-2.0
