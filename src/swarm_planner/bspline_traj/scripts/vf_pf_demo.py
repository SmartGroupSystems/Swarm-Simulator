#!/usr/bin/env python3
# 
# Guiding Vector Field  (VF-PF)  demo
# 路径：单位圆  x² + y² = 1
# 向量场：F(p) = -k_e φ ∇φ + k_t E ∇φ
#       E = [[0, -1],
#            [1,  0]]
# 依模板来自《Singularity-Free Guiding Vector Field for Robot Navigation》公式 (2)
# 

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# ---------------- 参数 ----------------
k_e = 2.0      # 横向吸引增益（越大收敛越快）
k_t = 1.0      # 切向推进增益（决定沿路速度）

# ---------------- 路径隐函数及梯度 ----------------
def phi(p):
    x, y = p
    return x**2 + y**2 - 1.0           # 单位圆零水平集

def grad_phi(p):
    x, y = p
    return np.array([2.0*x, 2.0*y])    # ∇φ = (2x, 2y)

E = np.array([[0, -1],
              [1,  0]])                # 90° 顺时针旋转矩阵

# ---------------- 向量场 ----------------
def vf(t, p):
    g = grad_phi(p)
    return -k_e * phi(p) * g + k_t * E @ g

# ---------------- 数值积分 ----------------
t_span = (0.0, 20.0)                  # 模拟 0–20 s
y0 = np.array([2.0, 0.0])             # 初始位置 (2, 0) —— 圆外
sol = solve_ivp(vf, t_span, y0,
                max_step=0.05,        # 小步长更平滑
                atol=1e-9, rtol=1e-8) # 高精度避免漂移

# ---------------- 绘图 ----------------
theta = np.linspace(0.0, 2*np.pi, 400)
circle_x, circle_y = np.cos(theta), np.sin(theta)

plt.figure(figsize=(6, 6))
plt.plot(circle_x, circle_y, 'k--', label='期望路径 (unit circle)')
plt.plot(sol.y[0], sol.y[1], label='机器人轨迹')
plt.axis('equal')
plt.xlabel('x')
plt.ylabel('y')
plt.title('VF-PF 导航演示')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
