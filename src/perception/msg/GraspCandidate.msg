# GraspCandidate.msg - 抓取候选
# 单个抓取位姿候选，包含位置、姿态和置信度
# 坐标系：相机坐标系 (camera_frame)

# 位置（相机坐标系）
geometry_msgs/Point position           # 夹爪中心位置 (x, y, z) 单位：米

# 姿态（相机坐标系）
geometry_msgs/Quaternion orientation   # 夹爪朝向四元数 (x, y, z, w)

# 抓取参数
float32 width                          # 建议夹爪开口宽度（米）
float32 confidence                     # 置信度 (0.0 - 1.0)

# 可选：4x4 变换矩阵（展平为16个元素，行优先）
# 如果提供，可直接使用；否则从 position + orientation 构建
float64[16] transform_matrix           # 4x4 齐次变换矩阵

