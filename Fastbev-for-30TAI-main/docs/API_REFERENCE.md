# FastBEV 部署端自定义 API 说明文档

## fastbev_reader.h

### 结构体 FastBEVCamera
* **功能说明**：存储单路相机的详细参数，包括图像路径、时间戳、传感器内外参以及预计算的投影矩阵。
* **成员变量**：
    * `char image_path[256]`：图像文件相对于数据集根目录的路径。
    * `float extrinsic_t[3]` / `float extrinsic_r[4]`：相机相对于自车坐标系的平移向量 [x, y, z] 和旋转四元数 [w, x, y, z]。
    * `float intrinsic[9]`：3x3 相机内参矩阵，行主序排列。
    * `float lidar2img[16]`：4x4 投影矩阵，将 LiDAR 空间点直接投影到增强后的图像坐标系，包含了完整的变换链。
    * `int has_lidar2img`：布尔标识，指示 JSON 配置文件中是否提供了预计算的 `lidar2img` 矩阵。

### 结构体 FastBEVTemporalFrame
* **功能说明**：表示一个历史时刻的帧数据，用于时序特征融合。
* **成员变量**：
    * `int frame_index`：历史帧索引（例如 0 代表 T-1 时刻）。
    * `float affine_params[6]`：用于 BEV 空间对齐的 6 参数仿射变换矩阵 [a00, a01, a02, a10, a11, a12]。
    * `FastBEVCamera cameras[6]`：该时刻对应的 6 路环视相机数据。

### 结构体 FastBEVSample
* **功能说明**：FastBEV 推理的基本数据单元，包含当前时刻数据及关联的历史帧。
* **成员变量**：
    * `char sample_token[33]`：样本的唯一标识符。
    * `FastBEVCamera cameras[6]`：当前时刻的 6 路相机参数。
    * `FastBEVTemporalFrame temporal[3]`：关联的 3 个历史时刻帧数据。

### 结构体 FastBEVDataset
* **功能说明**：完整数据集的内存容器，保存了从 JSON 加载的所有样本信息。
* **成员变量**：
    * `int num_samples`：数据集中包含的总样本数量。
    * `float bev_resolution`：BEV 网格的物理分辨率（米/像素）。
    * `FastBEVSample *samples`：样本动态数组指针。

### 函数 fastbev_load
* **功能说明**：从指定的 JSON 索引文件中加载并解析完整的数据集信息。
* **参数说明**：
    * 输入 `const char *json_path`：`dataset_info.json` 文件的系统路径。
* **返回值**：成功时返回 `FastBEVDataset*` 指针，失败时返回 `NULL`。

### 函数 fastbev_free
* **功能说明**：释放由 `fastbev_load` 分配的数据集内存。
* **参数说明**：
    * 输入 `FastBEVDataset *ds`：需要销毁的数据集对象指针。
* **返回值**：无。

---

## fastbev_export.hpp

### 函数 fastbev_export_camera_params
* **功能说明**：将当前帧的相机参数导出为 `camera_params.txt` 格式，供可视化程序读取。
* **参数说明**：
    * 输入 `const FastBEVSample *sample`：指向当前待导出样本的指针。
    * 输入 `const char *output_path`：目标文本文件的输出路径。
* **返回值**：`0` 表示成功，`-1` 表示文件创建失败。

---

## fastbev_preprocess_cv.hpp

### 结构体 FastBEVFrameInput
* **功能说明**：封装了送入 NPU 或加速模块的单帧推理数据。
* **成员变量**：
    * `float current_tensor[...]`：当前帧 6 路相机的图像张量，布局为 NHWC [6, 256, 704, 3]。
    * `float affine[3][6]`：3 组历史时刻的 BEV 对齐仿射参数。

### 函数 fastbev_preprocess_one_image
* **功能说明**：单路图像预处理流水线，包括解码、色彩转换 (BGR->RGB)、近邻插值缩放、中心裁剪及标准化。
* **参数说明**：
    * 输入 `const char *img_path`：原始图片路径。
    * 输出 `float *out_hwc`：预先分配好的 float 缓冲区，用于存储预处理后的 HWC 图像数据。
* **返回值**：`0` 成功，`-1` 失败（如图像无法读取）。

### 函数 fastbev_prepare_frame_input
* **功能说明**：高层业务接口，一次性完成当前帧 6 路图像的并行化预处理并提取时序参数。
* **参数说明**：
    * 输入 `const FastBEVSample *sample`：原始样本数据。
    * 输出 `FastBEVFrameInput *inp`：填充完毕的推理输入结构体。
* **返回值**：失败的相机路数（`0` 表示全部相机处理成功）。

---

## types.hpp

### 结构体 BoundingBox
* **功能说明**：描述 3D 检测目标的最终结构化数据。
* **成员变量**：
    * `float x, y, z`：目标底面中心点的 3D 物理坐标。
    * `float w, l, h`：物体的尺寸（宽、长、高）。
    * `float yaw`：物体的偏航角（旋转弧度）。
    * `float vx, vy`：目标在 X 和 Y 方向上的预测速度。
    * `float score`：置信度得分。
    * `int id`：类别 ID。

### 结构体 NMSConfig
* **功能说明**：非极大值抑制（NMS）算法的运行配置。
* **成员变量**：
    * `float score_thr`：初步得分过滤阈值。
    * `int max_num`：单帧允许输出的最大目标数量。
    * `std::vector<float> nms_thr_list`：针对不同物体类别设定的 IoU 阈值列表。
    * `float pedestrian_motorcycle_center_distance_m`：相同类别的行人或摩托车中心距离抑制半径；`0` 表示关闭。
    * `float cross_class_iou_threshold`：类别 `0-4、6、7` 之间的跨类别旋转框 IoU 阈值；`0` 表示关闭。

CARLA 服务要求 YAML 显式提供上述两个配置项，并在进入 NMS 前丢弃
`bicycle (class 5)`。`traffic_cone (class 8)` 和 `barrier (class 9)` 不参与
跨类别抑制。

---

## filter.hpp

### 函数 threshold_and_decode
* **功能说明**：将 NPU 输出的原始分类、回归、方向张量，按照预设 Anchor 解码为 3D 候选框集合。
* **参数说明**：
    * 输入 `const float* cls_ptr`：分类得分张量首地址 (NHWC)。
    * 输入 `const float* bbox_ptr`：3D 框回归张量首地址 (NHWC)。
    * 输入 `const float* dir_ptr`：方向分类张量首地址 (NHWC)。
    * 输入 `float score_thr`：候选框初筛得分阈值。
* **返回值**：包含所有超过阈值并完成坐标转换的 `BoundingBox` 向量。

---

## nms.hpp

### 函数 run_multi_class_nms
* **功能说明**：对候选框执行多类别非极大值抑制，消除空间上的冗余重叠检测。
* **参数说明**：
    * 输入/输出 `std::vector<BoundingBox>& candidates`：待处理的候选框列表，函数内部会参与排序。
    * 输入 `const NMSConfig& config`：NMS 相关的算法配置参数。
* **返回值**：经过 NMS 筛选后的最终结果框列表。

---

## visualize.cpp

### 结构体 CameraInfo (内部类)
* **功能说明**：可视化模块专用的相机配置，包含用于投影的各种逆变换矩阵。

### 函数 run (内部方法)
* **功能说明**：可视化主程序逻辑，读取检测结果与相机参数，生成包含环视投影与 BEV 俯视图的合成图。
* **参数说明**：
    * 输入 `const std::string& cams_path`：`camera_params.txt` 路径。
    * 输入 `const std::string& result_path`：检测结果文件路径。
    * 输入 `const std::string& out_path`：输出图片文件路径。
* **返回值**：`0` 成功，`1` 异常。
