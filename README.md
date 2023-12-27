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
