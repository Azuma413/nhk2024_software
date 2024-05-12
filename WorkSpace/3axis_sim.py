# 3軸アームの先端の座標から各サーボモーターの角度を求めるシミュレーション

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as wg
from matplotlib import patches

# アームの長さ
L0 = 372.0 # mm 地面から第一関節までの高さ
L1 = 160.0 # mm 第一関節から第二関節までの長さ
L2 = 310.0 # mm 第二関節から第三関節までの長さ
L3 = 113.0 # mm 第三関節から吸着部の中心までの長さ
xmax = L1+L2+L3
ymax = 2*L0

# グラフサイズを決める
plt.figure(figsize=(10, 10))
# 左側と下側に余白を作る
plt.subplots_adjust(left=0.15, bottom=0.15)
# グラフの最大値，最小値 [xmin, xmax, ymin, ymax]
plt.axis([-xmax*0.5, xmax, 0.0, xmax*1.5])
# グリッドを表示
plt.grid()
# スケールを合わせる
plt.gca().set_aspect('equal', adjustable='box')

def calc_rad2(r, h):
    """
    座標ベースで計算する
    """
    a0 = (r - L3)**2 + (h - L0)**2
    if a0 > (L1 + L2)**2 or a0 < (L1 - L2)**2:
        print('out of range')
        return np.array([0, 0, 0, 0]), np.array([0, 0, 0, 0]), 0, 0
    a1 = (a0 + L1**2 - L2**2)/2
    a2 = np.sqrt(a0*(L1**2) - a1**2)
    xn = (a1*(r - L3) - (h - L0)*a2)/a0
    yn = (a1*(h - L0) + (r - L3)*a2)/a0
    x0 = 0.0
    y0 = L0
    x1 = xn
    y1 = yn + L0
    x2 = r - L3
    y2 = h
    x3 = r
    y3 = h
    x_array = np.array([x0, x1, x2, x3])
    y_array = np.array([y0, y1, y2, y3])
    theta0 = np.arctan2(yn, xn)
    theta1 = np.arctan2(h - L0 - yn, r - L3 - xn) - theta0
    return x_array, y_array, theta0, theta1

# アームの初期座標
x = xmax/2
y = ymax/2
x_array, y_array, theta0, theta1 = calc_rad2(x, y)
a = plt.text(x_array[0], y_array[0], str(int(theta0*180/np.pi)), fontsize=10, color='r')
b = plt.text(x_array[1], y_array[1], str(int(theta1*180/np.pi)), fontsize=10, color='r')
# ms:マーカーの大きさ lw:線の太さ
l, = plt.plot(x_array, y_array, "ro-", ms=3, lw=2, color='b')
w, = plt.plot(x, y, "o-", ms=5, color='r')
# (0, L0)を中心とする円を描く
circle0 = plt.Circle((0, L0), L1, fill=False, color='g')
plt.gca().add_patch(circle0)
# (x, y)を中心とする円を描く
circle1 = plt.Circle((x-L3, y), L2, fill=False, color='r')
plt.gca().add_patch(circle1)
# ボールを描く
ball_x = 300
circle2 = plt.Circle((ball_x, 190/2), 190/2, fill=True, color='c')
plt.gca().add_patch(circle2)
# ロボットを描く
robot = patches.Rectangle(xy=(-xmax*0.5, 0), width=xmax*0.5, height=L0, color='y')
plt.gca().add_patch(robot)
# 高さ525mmの水平線を描く(サイロ)
plt.hlines(525, -xmax*0.5, xmax, "m")
# x0, y0, width, height
ax_x = plt.axes([0.405, 0.05, 0.49, 0.04])
ax_y = plt.axes([0.05, 0.15, 0.04, 0.735])
sli_x = wg.Slider(ax_x, 'x', 0, xmax, valinit=x, valstep=0.1)
sli_y = wg.Slider(ax_y, 'y', 0, xmax*1.5, orientation='vertical', valinit=y, valstep=0.1)

def update(val):
    sx = sli_x.val
    sy = sli_y.val
    x_array, y_array, theta0, theta1 = calc_rad2(sx, sy)
    l.set_data(x_array, y_array)
    w.set_data(sx, sy)
    a.set_position((x_array[0], y_array[0]))
    b.set_position((x_array[1], y_array[1]))
    # a.set_text(str(int(theta0*180/np.pi)))
    # b.set_text(str(int(theta1*180/np.pi)))
    # (sx, sy)を中心とする円を描く
    circle1.center = (sx - L3, sy)

sli_x.on_changed(update)
sli_y.on_changed(update)
plt.show()