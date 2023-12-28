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


# A 05 优化版本的运动模型
