import numpy as np

launch_points = np.array([
    [2625, -2495],
    [3625, -2495],
    [4625, -2495],
])

target_points = np.array([
    [1125, 1219],
])

distance = np.linalg.norm(launch_points - target_points, axis=1)

duty_list = np.array(
    [0.205, 0.225, 0.24]
)

avg = np.average(np.sqrt(distance)/duty_list)

print(avg)

avg = 301.2541395838844
def calc_duty(distance):
    global avg
    x = np.sqrt(distance)/avg
    print("m1:{}, m2:{}, m3:{}".format(x, x, 3*x))
    
for d in distance:
    calc_duty(d)