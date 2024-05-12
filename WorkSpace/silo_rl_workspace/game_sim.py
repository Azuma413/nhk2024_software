import numpy as np
import gymnasium
from gymnasium import spaces
from matplotlib import pyplot as plt

RED = 1 # 自チーム
BLUE = 2 # 相手チーム

class SiloEnv(gymnasium.Env):
    def __init__(self):
        self.silo_env = None
        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(low=0, high=2, shape=(5,3), dtype=np.int32)
        self.reward_range = [-1., 1.]
        self.my_color = RED
        self.enemy_color = BLUE
        self.done = False
        # 相手の強さ
        self.enemy_strength = 1.0 # 1.0だと自分と同じ強さ。0.0だと全く動かない。
        self.enemy_count = 0
        
    def reset(self):
        self.silo_env = Silo(self.my_color)
        self.done = False
        self.enemy_count = 0
        return self.silo_env.silo
    
    def step(self, action):
        old_count = self.enemy_count
        self.enemy_count += self.enemy_strength
        action_num = int(self.enemy_count) - int(old_count)
        for _ in range(action_num):
            self.silo_env.my_algo()
        
        reward = 0
        done = False
        info = {}
        if self.silo_env.put_ball(self.my_color, action):
            if self.silo_env.check_winner() == self.my_color:
                reward = 1
                done = True
            elif self.silo_env.check_winner() == self.enemy_color:
                reward = -1
                done = True
        else:
            reward = -1
            done = True
        obs = self.silo_env.silo
        return obs, reward, done, info
    
    def render(self, mode='rgb_array'):
        image = None
        fig, ax = plt.subplots()
        # サイロの描画
        for i in range(5):
            for j in range(3):
                if self.silo_env.silo[i][j] == RED:
                    ax.plot(i + 0.5, j + 0.5, 'ro', markersize=50)
                elif self.silo_env.silo[i][j] == BLUE:
                    ax.plot(i + 0.5, j + 0.5, 'bo', markersize=50)
                else:
                    ax.plot(i + 0.5, j + 0.5, 'wo', markersize=50)
        # 表示範囲を設定
        ax.set_xlim(0, 6)
        ax.set_ylim(0, 3)
        # 軸のスケールを揃える
        ax.set_aspect('equal')
        # 縦線を引く
        for i in range(1, 6):
            ax.plot([i, i], [0, 3], 'k-')
        fig.canvas.draw()
        image = fig.canvas.tostring_rgb()
        plt.show()
        image = np.frombuffer(image, dtype=np.uint8)
        image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        return image

class Silo:
    def __init__(self, my_color):
        self.silo = np.zeros((5,3))
        self.my_color = my_color
        self.enemy_color = 3 - my_color
        
    def put_ball(self, color, silo_num):
        for i in range(3):
            if self.silo[silo_num][i] != 0:
                continue
            else:
                self.silo[silo_num][i] = color
                return True # ボールを置けた場合
        return False # ボールを置けなかった場合

    def check_winner(self):
        red_count = 0
        blue_count = 0
        for i in range(5):
            num = self.silo[i][0]*self.silo[i][1]*self.silo[i][2]
            if num % 4 == 0 and num != 0:
                blue_count += 1
            elif num != 0:
                red_count += 1
        if red_count >= 3:
            return RED
        elif blue_count >= 3:
            return BLUE
        else:
            return 0 # まだ勝敗がついていない場合
        
    def my_algo(self):
        priority = np.array([0, 0, 0, 0, 0])
        for i in range(self.silo.shape[0]):
            if self.silo[i][0] == 0: # ボールが置かれていないサイロ
                priority[i] = 1
            elif self.silo[i][2] != 0: # 3つのボールが置かれているサイロ
                priority[i] = 0
            elif self.silo[i][1] == self.enemy_color: # 相手チームのボールが二段目に置かれているサイロ
                priority[i] = 4
            elif self.silo[i][1] == self.my_color: # 自チームのボールが二段目に置かれているサイロ
                priority[i] = 5
            elif self.silo[i][0] == self.enemy_color: # 相手チームのボールが一段目に置かれているサイロ
                priority[i] = 2
            elif self.silo[i][0] == self.my_color: # 自チームのボールが一段目に置かれているサイロ
                priority[i] = 3
        max_idx = np.argmax(priority)
        self.put_ball(self.enemy_color, max_idx)
        
if __name__ == "__main__":
    env = SiloEnv()
    obs = env.reset()
    done = False
    while not done:
        env.render()
        obs, reward, done, info = env.step(np.random.choice(5))
        print("done:", done)
        print("reward:", reward)