# 汉诺威大学的SLAM课程

## conda installation
```bash
conda create -n luh-slam -y python=3.9 pip
conda deactivate && conda activate

pip install matplotlib
```

# 课程内容
A. 构建小车运动模型，根据小车控制信号（车轮编码器）重建小车运动轨迹；根据小车上的LiDar信号，构建算法搜索并重建环境中的地标（cylinders）；将检测到的cylinder投影到环境中，并且和ground truth比较

B. 根据相似性，将ground truth中的cylinders和我们检测到的cylinders相匹配，并且基于此，利用least squares estimation对相似性变换进行计算 -> non-iterative solution；由于有时候检测到的cylinder pairs太少，所以导致轨迹呈锯齿状，因此考虑利用iterative cloest points：将场景中边缘上的点匹配给最近的一对cylinder pair，从而计算相似性变换程度

C. 利用概率分布描述机器人的不确定性：movement（增大）以及mearsurements（减小）对不确定性的影响；结合movement/prediction和measurement/correction

D．应用于多维正态分布的KF；总结小车的非线性运动模型；引入EKF，对运动模型近似，然后进行预测，根据预测绘制轨迹，发现小车的位置和heading的误差都unbounded 变大；对测量模型近似，结合两者，得到的轨迹的误差不再是unbounded了

E．利用一组hypothetical states/particles表示分布：粒子滤波；预测步骤中，我们最终会得到一组发散的粒子；在修正步骤中，我们计算了importance vector/weight，并且基于这些weights对粒子进行了importance sampling；这样即使不知道initial state，也可以重建出轨迹

F．现在，不在预先提供map -> SLAM；EKF-SLAM：地标的位置将作为filter state的一部分

G．FASTSLAM/particle-filter SLAM

