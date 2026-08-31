import cv2
import os

# 1. 获取当前脚本所在目录 (tools/)
current_dir = os.path.dirname(os.path.abspath(__file__))

# 2. 获取项目根目录 (tools 的上一级)
project_root = os.path.dirname(current_dir)

# 3. 拼接输入图片文件夹的绝对路径 (deploy/io/output/png)
image_folder = os.path.join(project_root, 'deploy', 'io', 'output', 'png')

# 4. 拼接输出视频文件夹及文件的绝对路径 (deploy/io/output/video/output_video.mp4)
output_folder = os.path.join(project_root, 'deploy', 'io', 'output', 'video')
os.makedirs(output_folder, exist_ok=True) # 如果 video 文件夹不存在则自动创建
video_name = os.path.join(output_folder, 'output_video.mp4')

# 配置参数
fps = 2  # 0.5秒一张，所以一秒钟播放2张 (1 / 0.5 = 2)
num_images = 404

# 构建第一张图片的路径以获取视频尺寸
first_image_path = os.path.join(image_folder, 'output_0001.png')
frame = cv2.imread(first_image_path)

if frame is None:
    print(f"错误：无法读取图片 {first_image_path}，请检查该路径下是否有图片。")
    exit()

height, width, layers = frame.shape

# 初始化视频写入器
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter(video_name, fourcc, fps, (width, height))

print(f"开始生成视频，共 {num_images} 张图片，帧率为 {fps} FPS...")
print(f"正在读取: {image_folder}")

# 循环读取 1 到 404 的图片并写入视频
for i in range(1, num_images + 1):
    filename = f"output_{i:04d}.png"
    img_path = os.path.join(image_folder, filename)
    
    img = cv2.imread(img_path)
    
    if img is None:
        print(f"警告：找不到或无法读取 {filename}，已跳过此帧。")
        continue
        
    video.write(img)

# 释放并关闭写入器
video.release()
print(f"视频生成完毕！已保存至: {video_name}")