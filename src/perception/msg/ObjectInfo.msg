# ObjectInfo.msg - 物体信息
# 用于查询结果返回

string object_id                      # 物体唯一 ID (如 "obj_001")
string label                          # 标签 (如 "cup")
string description                    # 描述 (如 "红色陶瓷杯子")
int64 created_at                      # 创建时间（毫秒时间戳）
int64 updated_at                      # 更新时间（毫秒时间戳）
int32 sample_count                    # 样本数量

