# C 01 利用离散概率分布表示位置误差
- `distribution.py` 中定义了一个`class Distribution`，其中包含有脉冲信号，三角信号和高斯信号，用来表示位置或者控制分布
- `slam_06_a_move_distribution_question`


# C 03 误差传播
- 除了位置误差外，还有需要用离散概率分布表示运动/控制误差；
- 误差的传播变现为 位置的概率分布和运动的概率分布的卷积
  -  Bayes：P(A,B) = P(B|A) · P(A)
  -  全概率：P(Bj) = Σ P(Bj|Ai) · P(Ai)
  -  `slam_06_b_convolve_distribution_question`
 

# C 04 `slam_06_c_multiply_distribution_question` 引入测量误差，考虑将测量误差和位置误差进行融合
当我们将 `slam_06_b_convolve_distribution_question` 中的 `arena` 和 `moves` 增大时，我们在看plot，可以发现：as we move on，
- the distributions lose their triangle shape，变成了bell shape
- 同时，分布也越来越宽，甚至开始overlap

## 引入测量误差
现在，我们有了两个distributions用来描述postision和movement，现在还缺少对measurement的描述。

1. 传感器在出厂前会进行calibrate，也就是说，测量一个calibrated distance，然后将测量结果绘制出来，通常我们会假设是高斯误差，所以测量的结果绘制出来的曲线应该是一条高斯分布；然后出厂的时候，会说这个传感器的精度为±4cm，也就是说这个高斯分布的标准差σ = 4cm，而期望应该为calibrated distance，否则就是有系统误差
2. 等到我们用这个传感器测量一个距离时，我们也需要将传感器的误差考虑进小车的位置误差内
   - 小车的位置误差分布称为prior
   - 传感器的测量误差分布称为mearsurement

## 融合测量误差和位置误差
由 `slam_06_c_multiply_distribution_question.py` 的plot可知：
- 尽管measurement要比prior精度低（`main` 函数中，`measurement_error` 为200，而`position_error` 仅为100），但是，posterior还是要比prior精度要更高：posterior更窄，并且峰值更高


# C 06 引入1D Kalman Filter
继续研究 `slam_06_c_multiply_distribution_question.py`：
- 将 `measurement_value` 设成500；此时posterior向measurement偏，所以其形状也不再是三角形了；
- 将 `measurement_value` 设成550；此时，measurement和prior几乎实在说，机器人不在同一个地方了；但是尽管如此，posterior的peak也比prior和measurement高。也就是说，无论measurement和prior区别多大，只要加上information from laser scanner，我们都可以得到一个 `information gain`

## dynamic Bayes network -> (discrete) Bayes Filter -> Historgram Filter: `slam_06_d_histogram_filter`

- 当我们将 `control` 信号的宽度设置成50；这要比`measurement`的误差大太多了，所以我们可以看到，红色的posterior比蓝色的prior精度高很多；但是再下一回合的move后，这些gained information由于位置分布的精度太低，又丢失了；另外，每次的posterior几乎不变，因为movement的精度太低，此时posterior更加相信measurements
- 当我们将 `measurement` 信号的宽度设置成50；这要比`control`的误差大很多，此时，posterior的peaks要比之前低很多；也就是说，如果我们降低测量信号的精度，那么estimated/posterior的精度也会下降；极端情况下，如果没有measurements，随着movement，位置分布会越来越宽，并且peak会越来越低

## storage efficiency problem： `slam_06_e_histogram_filter_cleanedup`
现在的问题是，在Bayes Filter中，我们用arrays存储的posterior和prior，随着movement，两个分布会变得越来越宽，也就是对应的arrays会变得越来越长；然后我们又要求卷积和乘积，这个的计算量会非常大。所以我们想要用另外一种方法表示prior和posterior，从而使得在卷积的这一步中，我们可以将Σ转变成∫，并且保证prior和posterior都能用这个方式表示

- 我们可以看到，第二个蓝色曲线几乎是一个bell shape，因为两个triangle shapes 卷积后，就会得到一个binomial，而不再是triangle shape；这个看起来几乎和高斯分布一样了，如果我们将 `Dist` 设置成 `Distribution.gaussian`，我们会发现，如果分布假设成高斯分布，那么无论是通过卷积得到的蓝色曲线（prior）还是通过乘积得到的红色曲线（posterior），都将保持为高斯分布，即：
  - 高斯 * 高斯 = 高斯
  - 高斯 · 高斯 = 高斯
- 并且，由于第二个绿色曲线（measurement）的方差比prior大一倍，所以得到的红色曲线（posterior）更靠近prior，也就是更信任prior

# C 08 1D Kalman Filter 推导

## Filter Step

利用高斯分布描述prior；Likelihood（c用于从状态空间转换到测量空间；这样，机器人的位置可以用厘米做单位，而测量信号仍然用毫米做单位）；以及posterior；

现在，我们就不再需要像之前那样，利用arrays存储概率分布，然后再计算乘积，从而近似posterior；而只需要更新posterior对应的正态分布的期望和方差；

### 判断posterior是否仍然满足高斯分布并且得到公式

推导Kalman Gain；以及期望和方差的更新公式

- K = 0：posterior的更新将忽略measurement，直接将predicted/prior作为posterior
- K = 1：posterior的更新将忽略predicted，直接将measurement作为posterior
- 将K变形后，可以看出：如果measurement的误差越大，K就越小，就越忽略measurement，而重视predicted；

## Prediction Step

如何处理卷积（积分）；

这里，同样用高斯分布描述上一步的posterior和下一步的状态量的概率分布；这里用线性仿射变换描述运动模型；

# C 09 1D KF 总结和Implmentation

预测的结果仍然是高斯分布；并且可以计算出来这个分布的期望和方差，至于如何证明，可以查看《概率机器人》

