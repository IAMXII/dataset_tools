import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

# 创建随机数据
np.random.seed(42)
x =np.array([4,
1,
0.041666667,
7.98,
0.133333333,
1,
1.666666667])
y = np.array([0.836986969,
0.961941298,
0.692363902,
1,
0.651884667,
0.758068445,
0.836986969])
# sizes = 1000 * np.random.rand(100)
colors = y

# 设置Seaborn样式
sns.set(style="white", context="talk")

# 绘制散点图
plt.figure(figsize=(12, 8))
scatter = plt.scatter(x, y, c=colors, cmap='viridis', alpha=0.7, edgecolor='w', linewidth=0.5)
plt.scatter(x[3], y[3], color='red', s=300, edgecolor='black', label="Key Point", zorder=5)

# 添加颜色条
# 添加颜色条
plt.colorbar(scatter)

# 添加标题和标签
plt.title('Cool Scatter Plot with Seaborn and Matplotlib', fontsize=20, fontweight='bold')
plt.xlabel('X Axis', fontsize=14)
plt.ylabel('Y Axis', fontsize=14)

# 隐藏顶部和右侧边框
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)

# 显示图像
plt.show()