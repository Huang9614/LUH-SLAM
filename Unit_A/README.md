# A 02 Motor Control：通过编码器查看轮子的ticks

## `robot4_motors.txt` 以及 `slam_01_a_plot_motor_ticks`
- 第一列为标识符；
- 第二列为时间戳；
- 第三列为左轮编码器，数字表示的是左轮转动产生的ticks的个数，可以换算成转动过的角度；
- 第七列为右轮编码器

## `lego_robot.py` 以及 `slam_01_b_print_motor_increments`，`slam_01_c_plot_motor_increments`
在 `lego_robot.py` 中，Prof. Brenner 写了一个名为 `LegoLogFile` 的类，用来更方便地读取数据：方法 `read()` 和属性 `motor_ticks`


# A 03 + 04 基础版的Motion Model：如何将轮子的Ticks转变成小车的运动模型

`slam_02_a_filter_motor_question` 

## 参数
- `w`：手动测量小车两个轮子中心的宽度；
- `ticks_to_mm`：手动测量，用于将ticks转变成轮子运动的距离，单位为毫米；
- `l`,`r`：左轮和右轮的运动距离
- `θ`：heading direction
- `α`：小车转动的距离
- `R`：转动中心

## 运动模型

### 无转弯

### 转弯


# A 05 优化版本的运动模型：`slam_02_b_filter_motor_file_question`

基础版本的问题在于：小车坐标系和LiDAR坐标系之间存在误差；小车的位姿是由LiDAR得到的，因此在代码中，`pose` 的数据是在LiDAR坐标系中描述；而利用运动模型预测的位姿实在小车坐标系中的；因此需要考虑对 `pose` 进行修正，然后才能代入到motion model中预测。

这样，我们就通过轮子编码器得到的ticks的个数计算出了小车所有的位姿，并且存储在 `poses_from_ticks.txt` 中，共278行。

另外，这里有一个 Prof. Brenner 自己编写的 `logfile_viewer.py` 模块，用来可视化这些位姿，即生成轨迹。【已修改为python3版本】

# A 06 sensor data

为什么需要传感器数据？因为在之前的作业中，我们确实得到了一条光滑的轨迹，但问题在于不一定正确：还是运行 `logfile_viewer.py`，这次除了打开 `poses_from_ticks.txt`，我们还导入真实的轨迹 `robot4_reference.py`。会发现误差非常离谱！【如果误差不大，应该是调过参数了】。这里就体现了运动模型的精度的影响效果，如果修改 `w`，也就是小车的宽度，从150mm到173mm，就会发现，误差变小了！ 但这毕竟不是万全之策，所以我们需要别的方法：使用LiDAR测量场景中的地标 ，从而确定小车位置！

## `robot4_scan.txt` 包含了从LiDAR中得到的所有测量数据

- LiDAR信号共有660个beams；
- 在这个 `robot4_scan.txt` 中，每个扫描位置，都以 `S` 作为标注，共有662个数值，第一个为时间戳，第二个为counter，后面都为测量数值
- 每个地标会引起这些beams中的一个spike
- 在 `03_a_plot_scan.py` 中将这些测量数据画了出来；

### 设计算法，从测量数据中检测出地标cylinders：`slam_03_b_scan_derivative_question` 和 `slam_03_c_find_cylinders_question`

Idea: 计算一个位姿处的scan信号的梯度信息，当梯度超过一个预设的阈值时，标记为cylinder的一边；negative spike标记为cylinder的左边。

如何计算梯度信息？离散 -> phase shift 

如何忽略测量信号中的误差 -> `minimum_valid_distance = 20.0`

# A 07 如何处理多个cylinders重叠的问题：`slam_03_c_find_cylinders_question`

# A 08 将找到的cylinder的信息由 (ray_index, range) 转变成小车坐标系中的位置坐标：`slam_03_d_find_cylinders_cartesian_question`


