# README

在 publish_target.py 中创建了一个简单的目标点发布节点：

**功能：**

- 发布 `geometry_msgs/Point` 消息到 `/target` 话题
- 可自定义目标坐标和发布频率
- 默认“单次模式”会先等待订阅者，再做短时突发发送，减少单次消息丢失
- `/target` 使用 QoS：`RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`（更适合单次目标）
- 默认参数：`x=1.0, y=2.0, rate=10.0`

**使用方法：**

- 直接运行并传参（单位为米，频率为 Hz）：

```bash
python3 src/utils/publish_target/publish_target.py --x 0.1 --y 0.1
```

- 连续发布：

```bash
python3 src/utils/publish_target/publish_target.py --x 0.1 --y 0.1 --rate 10 --loop
```

- 不传参数时使用默认值：`x=1.0, y=2.0, rate=10.0`

```bash
python3 src/utils/publish_target/publish_target.py
```

说明

- `/target` 话题仍发布 `geometry_msgs/Point`，单位为米
- C++ 侧 `ActionHandler` 已在 `IDLE` 状态中将接收的米转换为毫米后填入串口包

可选参数（单次模式）：

- `--wait-subs-timeout`：等待订阅者超时（默认 `1.0` 秒）
- `--once-burst-duration`：突发发送时长（默认 `0.0` 秒）
- `--once-burst-rate`：突发发送频率（默认 `20` Hz）

---

## 新增程序 1：当前位置 + 增量发布

脚本：`publish_target_from_delta.py`

**功能：**

- 订阅当前位置话题（默认 `/glim_ros/pose_corrected`，`PoseStamped`）
- 读取用户输入的 `dx dy`（单位米）
- 计算 `target = current + delta` 并发布到 `/target`

**用法：**

```bash
python3 src/utils/publish_target/publish_target_from_delta.py
```

可选参数示例：

```bash
python3 src/utils/publish_target/publish_target_from_delta.py \
	--pose-topic /glim_ros/pose_corrected \
	--target-topic /target \
	--publish-count 1
```

---

## 新增程序 2：Point-LIO TF + 增量发布

脚本：`publish_target_from_delta_pointlio_tf.py`

**功能：**

- 从 Point-LIO TF 读取当前位置（默认 `lidar_odom -> lidar_link`）
- 读取用户输入的 `dx dy`（单位米）
- 计算 `target = current + delta` 并发布到 `/target`

**用法：**

```bash
python3 src/utils/publish_target/publish_target_from_delta_pointlio_tf.py
```

可选参数示例：

```bash
python3 src/utils/publish_target/publish_target_from_delta_pointlio_tf.py \
	--tf-parent-frame lidar_odom \
	--tf-child-frame lidar_link \
	--wait-tf-timeout 3.0 \
	--publish-count 1
```
