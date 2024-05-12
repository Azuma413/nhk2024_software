import cv2
import numpy as np
import os

def likelihood(x, y, func, image, w=30, h=30):
  """
  尤度を計算する
  w*hの領域にあるpixelがすべて条件(func)に合致するなら1.0,
  1つも合致しないなら0.0001を返す
  """
  # x, yを中心とするw*hの領域を切り出す
  x1 = int(max(0, x - w / 2))
  y1 = int(max(0, y - h / 2))
  x2 = int(min(image.shape[1], x + w / 2))
  y2 = int(min(image.shape[0], y + h / 2))
  region = image[y1:y2, x1:x2] # 切り出した画像
  # 切り出した領域内の条件(func)に合致するpixelの数をカウント
  count = region[func(region)].size
  return (float(count) / image.size) if count > 0 else 0.0001

def init_particles(func, image):
  """パーティクルを初期化する"""
  # mask = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convert image to grayscale
  mask = image.copy()
  mask[func(mask) == False] = 0 # 条件に合致しない領域を黒にする
  mask[mask != 0] = 255 # 条件に合致する領域を白にする
  contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # 輪郭を抽出
  if len(contours) <= 0:
    return None # 輪郭がない場合はNoneを返す
  max_contour = max(contours, key=cv2.contourArea) # 最大の輪郭を取得
  max_rect = np.array(cv2.boundingRect(max_contour)) # 輪郭を囲む矩形を取得
  max_rect = max_rect[:2] + max_rect[2:] / 2 # 矩形の中心座標を取得
  weight = likelihood(max_rect[0], max_rect[1], func, image) # 尤度を計算
  particles = np.ndarray((500, 3), dtype=np.float32) # パーティクルを格納する配列 500: パーティクルの数 3: x, y, weight
  particles[:] = [max_rect[0], max_rect[1], weight] # パーティクルを初期化 x: 矩形の中心のx座標 y: 矩形の中心のy座標 weight: 尤度
  return particles

def resample(particles): 
  """
  パーティクルをリサンプリングする(尤度の低いパーティクルを置き換える)
  particles: 初期化されたパーティクルの配列
  """
  tmp_particles = particles.copy()
  weights = particles[:, 2].cumsum() # 尤度の累積和 [0, w1, w1+w2, w1+w2+w3, ...]
  last_weight = weights[weights.shape[0] - 1] # すべてのパーティクルの尤度の合計
  for i in range(particles.shape[0]):
    weight = np.random.rand() * last_weight # 0~last_weightの乱数を生成
    particles[i] = tmp_particles[(weights > weight).argmax()] # weightよりも大きい値を持つtmp_particlesの最初の要素をparticles[i]に代入
    particles[i][2] = 1.0 # 重みを1.0にする

def predict(particles, variance=13.0): 
  """
  varianceは目標の動きの激しさに応じて設定
  パーティクルをランダムに動かす
  """
  particles[:, 0] += np.random.randn((particles.shape[0])) * variance
  particles[:, 1] += np.random.randn((particles.shape[0])) * variance

def weight(particles, func, image):
  """パーティクルの重みを計算する"""
  for i in range(particles.shape[0]): 
    particles[i][2] = likelihood(particles[i][0], particles[i][1], func, image) # likelihood関数で重みを計算
  sum_weight = particles[:, 2].sum() # 重みの合計
  particles[:, 2] *= (particles.shape[0] / sum_weight) # 重みを正規化

def measure(particles): 
  """
  成績の良いパーティクルが集中している場所を計算
  """
  # 重み付き平均を計算
  x = (particles[:, 0] * particles[:, 2]).sum()
  y = (particles[:, 1] * particles[:, 2]).sum()
  weight = particles[:, 2].sum()
  return x / weight, y / weight

particle_filter_cur_frame = 0

def particle_filter(particles, func, image_rgb, max_frame=10):
  """
  パーティクルフィルタを実行する
  max_frameの間，対象の色が画像内にない場合はパーティクルを初期化
  """
  # rgbをhsvに変換
  frame_hsv = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2HSV_FULL)
  image = frame_hsv[:, :, 0]
  _, frame_s = cv2.threshold(frame_hsv[:, :, 1], 0, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
  _, frame_v = cv2.threshold(frame_hsv[:, :, 2], 0, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
  image[(frame_s == 0) | (frame_v == 0)] = 0

  global particle_filter_cur_frame # フレーム数をカウントする変数
  if image[func(image)].size <= 0: # 画像内に対象の色がない場合
    if particle_filter_cur_frame >= max_frame: # 一定フレーム数経過した場合はパーティクルを初期化
      return None, -1, -1
    particle_filter_cur_frame = min(particle_filter_cur_frame + 1, max_frame) # フレーム数をカウント
  else: # 画像内に対象の色がある場合
    particle_filter_cur_frame = 0 # フレーム数のカウントをリセット
    if particles is None: # パーティクルが空なら初期化
      particles = init_particles(func, image)

  if particles is None:
    return None, -1, -1

  resample(particles) # パーティクルをリサンプリング
  predict(particles) # パーティクルを予測
  weight(particles, func, image) # パーティクルの重みを計算
  x, y = measure(particles) # 重み付き平均を計算
  return particles, x, y

# 色域を設定
red = [320, 20]
purple = [260, 300]
blue = [180, 260]

def func(color):
  def is_color(region):
    if color[0] < color[1]:
      return (region >= int(color[0]/360*255)) & (region < int(color[1]/360*255)) # 下限より大きく，上限より小さいときTrue
    else:
      return (region >= int(color[0]/360*255)) | (region < int(color[1]/360*255)) # 上限より大きいか，下限より小さいときTrue
  return is_color

relative_path = 'yolo_finetuning\\blue_video\\MOV_0458.mp4' # 動画のパス
# 現在のファイルのディレクトリを取得
current_dir = os.path.dirname(os.path.abspath(__file__))
video_path = os.path.join(current_dir, relative_path)
if not os.path.exists(video_path):
  print("Video file does not exist.")
  exit()

cap = cv2.VideoCapture(video_path)
annotated_frames = [] # フレームを保存するリスト
particles = None # パーティクルを保存する変数(ループの外で宣言する必要がある)
while cap.isOpened(): # フレームがある間繰り返す
  ret, frame = cap.read()
  if ret: # フレームがある場合
    particles, x, y = particle_filter(particles, func(blue), frame) # ここで検出する色を指定
    if particles is not None:
      # フレームのサイズ内のパーティクルのみを取り出す
      valid_particles = particles[(particles[:, 0] >= 0) & (particles[:, 0] < frame.shape[1]) & (particles[:, 1] >= 0) & (particles[:, 1] < frame.shape[0])]
      for i in range(valid_particles.shape[0]):
        # パーティクルを黄色で描画
        frame[int(valid_particles[i][1]), int(valid_particles[i][0])] = [255, 255, 255] # BGR
      p = np.array([x, y], dtype=np.int32)
      # 重心を四角形で囲む
      cv2.rectangle(frame, tuple(p - 30), tuple(p + 30), (0, 0, 255), thickness=2)
      annotated_frame = frame
  else:
    break
  annotated_frames.append(annotated_frame)
# annotated_framesをmp4として保存
height, width, layers = annotated_frames[0].shape
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter('./test.mp4', fourcc, 30, (width, height))
for frame in annotated_frames:
    video.write(frame)
video.release()
cap.release()
cv2.destroyAllWindows()
print('finished')