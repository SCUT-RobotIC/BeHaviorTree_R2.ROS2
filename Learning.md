# 插件结构
## include 
 - 放BTROS节点的声明
## src 
 - register.cpp BT注册文件
```
// factory-节点工厂参数，params-ROS2节点参数
BT_REGISTER_ROS_NODES(factory, params)
{
    // 把你的 C++ 类注册成树里可用的节点类型，XML 里用 TfListen 这个名字就能创建该节点，并自动注入 ROS2 参数。
    factory.registerNodeType<pose_est_bt::PoseSolveNode>("Solve_Pose", params);
}
```
 - package.cpp BT函数实现文件
```
// 在 BehaviorTree.CPP 里，“端口”就是节点的输入/输出接口，负责在黑板（Blackboard）里读写数据。
providedPorts() 只是声明这个节点有哪些输入/输出，它本身不传值。
BT::PortsList TfListenNode::providedPorts()
{
	return {
		BT::InputPort<std::string>("target_frame"),
		BT::InputPort<std::string>("source_frame"),
		BT::InputPort<double>("timeout", 0.0, "Lookup timeout seconds"),
		BT::OutputPort<double>("x"),
		BT::OutputPort<double>("y"),
		BT::OutputPort<double>("rot_z")
	};
}
```
## 黑板的值的来源：
在行为树 XML 里映射端口到黑板键：例如 target_frame="{target}"，表示从黑板键 target 读到输入端口 target_frame；输出端口同理写回黑板。
在代码里创建树时直接给黑板写初始值：blackboard->set("target", "map")。
 - getInput 返回BT::Expected<bool>，它包含两种状态：

1. 有值：need.has_value() 为真，need.value() 是真正的 bool
2. 没有值/出错：need.has_value() 为假，可以用 need.error() 查看错误原因

# 回调存储+循环读取Action节点中实现读取相机数据完成闭环控制
```
ros2 launch r2_decision bt_executor.launch.py
ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: GetBlockPose}"
```
# 给 copilot 进行默认配置
A. 仅此仓库生效

在仓库根目录新建文件：.github/copilot-instructions.md
写入内容（示例）：
保存后重新打开 VS Code 或重载窗口。
B. 全局所有项目生效

打开目录：~/.config/Code/User/prompts/
新建文件：global.instructions.md
写入内容（示例）：
# tick()检查
对，检查发生在每次 tick() 调用时。
行为树不会提前“统一检查”，而是每个节点在自己的 tick() 里做资源检查/使用，比如：

你的 TfListenNode 在 tick() 里执行 lookupTransform，查不到 TF 就直接返回 FAILURE。
PoseSolve 节点也是在 tick() 里检查服务可达性。
所以从原理上说：行为树只在“运行到该节点时”才会检查依赖，而不是在树开始前全局检查。
如果你需要“启动前一次性检查”，要显式加一个“检查节点”，或者在节点里实现等待/重试逻辑。

动作接口：目标Goal 结果Result 反馈Feedback

在 BT XML 里给节点配置 action_name，这个节点就会把 Goal 发送到对应的 Action Server。

self.capture_event.wait(timeout=0.1)

的语法含义：

self.capture_event 是一个 threading.Event 对象
wait() 会阻塞当前线程，直到事件被 set()
timeout=0.1 表示最多等待 0.1 秒
返回值：

如果在 0.1 秒内事件被 set() → 返回 True
如果超时仍未被 set() → 返回 False
```
<SaySomething name="first"   message="hello world" />
<SaySomething name="second" message="{greetings}" />
// 第一种直接传值
// 第二种从黑板里获取值The value of the entry "greetings" can (and probably will) change at run-time.
<SetBlackboard output_key="target_pose" value="3.5;2.0;0.0" />
// 可以直接在xml里面写入
{}表示黑板条目，也就是key
text="{the_answer}中text是端口名称，后面是写入还是输出要看节点怎么定义
```
默认情况：不完全共享。
在 BehaviorTree.CPP 里，子树通常会有自己的黑板作用域，只通过端口映射与父黑板交换数据。

更具体地说：

SubTree（默认）
子树使用“子黑板”，与父黑板不完全共享。
只有在 XML 里显式映射的端口，才会在父子之间传递。

SubTreePlus
提供更“强”的共享方式（更接近同一黑板），但仍建议用端口映射来控制数据流。

结论：
主树和子树不是天然完全共享黑板；是否共享、共享多少，取决于你使用的子树节点类型以及端口映射方式。
```
            <SubTree ID="MoveRobot" target="{move_goal}" 
                                    result="{move_result}" />
									// 前面是子树黑板的key，后边是主树黑板的key
```
registerScriptingEnums registerScriptingEnums 是 BehaviorTree.CPP 里用来把 C++ 枚举类型注册到 BT 的脚本/黑板表达式系统的函数。注册之后，你在 XML 的脚本表达式里就可以直接用枚举名和枚举值，而不需要手动写数字常量。
the methods getInput and setOutput copy the value from/to the blackboard.
  DataHandler();
  DataHandler(const DataHandler& other) = delete;
  DataHandler& operator=(const DataHandler& other) = delete;
  当类持有不可共享的资源时，禁止拷贝能防止双重释放或资源冲突：
这段代码的目的是让 DataHandler 类不可拷贝（non-copyable），常用于：

    单例模式（Singleton）
    管理独占资源（如文件句柄、网络连接、互斥锁）
    防止对象被意外复制导致资源重复释放
	| 场景        | 是否加 `noexcept`           |
| --------- | ------------------------ |
| 析构函数      | 自动隐式 `noexcept`，一般不用写    |
| 移动构造/移动赋值 | **必须加**，否则性能损失           |
| swap 函数   | **必须加**                  |
| 确定不抛异常的函数 | 建议加，帮助优化                 |
| 可能抛异常的函数  | 不加（默认 `noexcept(false)`） |
noexcept 是 C++11 引入的关键字，用于显式声明函数不会抛出异常。
static DataHandler& GetInstance(); 的意思是：这是 DataHandler 类的一个静态成员函数声明，它返回一个 DataHandler 对象的引用。常见用法是实现单例模式：全局只有一个 DataHandler 实例，通过 GetInstance() 获取。

拆解说明

static：属于类本身，不需要先创建对象就能调用。调用方式是 DataHandler::GetInstance()。
返回类型 DataHandler&：返回的是引用，不会复制对象；通常返回类内部保存的那个唯一实例。
名称 GetInstance：约定俗成的单例入口函数名。
为什么要这样写（原理）

在单例模式中，构造函数通常是私有的（这里 DataHandler() 是私有），外部不能 new DataHandler()。
只能通过 GetInstance() 访问唯一实例，保证资源（比如线程、回调、设备通信）只初始化一次。
返回引用避免拷贝，也能阻止误删实例。
一般会一直发 tick，只要你的行为树执行器在运行（比如主循环里固定频率 tick 根节点）。
规则是：

根节点每次 tick，父节点就会按逻辑去 tick 子节点
如果树在运行（root 正在被周期性 tick），父节点通常也会周期性 tick
只有在树停止、暂停，或执行器停止 tick 时，父节点才不会继续发
StatefulActionNode 是 BehaviorTree.CPP 里带“运行状态”的动作节点。
它的核心特点是：可以跨多个 tick 持续执行，并在执行过程中保持内部状态。

为什么需要它
有些行为不是“一次 tick 就能完成”的，比如：

等待订阅消息
等待服务响应
等待机器人移动完成
这些都需要多次 tick 才能完成，所以用 StatefulActionNode。
它的执行流程

第一次被 tick → 调用 onStart()
如果 onStart() 返回 RUNNING → 之后每次 tick 调用 onRunning()
当返回 SUCCESS/FAILURE → 节点结束
如果中途被中断 → 调用 onHalted() 清理资源
一句话理解
StatefulActionNode 就是“会持续运行、能保持内部状态”的叶子节点。
std::shared_ptr 的方法 .reset()
| 方法           | 作用                                   |
| ------------ | ------------------------------------ |
| `reset()`    | 释放 `shared_ptr` 管理的对象（引用计数-1，若为0则销毁） |
| `reset(ptr)` | 接管新的原始指针                             |
sudo chown -R $(whoami):$(whoami) /home/gzic-robotic/Documents/librealsense2
sudo chown -R $(whoami):$(whoami) ~/r2_bht/BeHaviorTree_R2.ROS2/build
sudo chown -R $(whoami):$(whoami) ~/r2_bht/BeHaviorTree_R2.ROS2/install
# 只编译 src/目标文件夹/ 下的所有包
colcon build --base-paths src/目标文件夹
1715  ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: MainTree}"
其中 WrappedResult 是 BT::RosActionNode 内部定义的一个结构体，通常包含 action 的结果、状态等信息，适用于你需要访问完整 action 返回上下文的场景
std::function<void(const std::vector<uint8_t>&)> callback解释语法

std::function<void(const std::vector<uint8_t>&)> callback 的解释：

std::function<...>：类型擦除的可调用对象包装器，可存放任意符合签名的可调用实体（函数指针、lambda、std::bind、函数对象等）。
模板签名 void(const std::vector<uint8_t>&)：表示被包装的可调用对象必须返回 void，并接受一个 const std::vector<uint8_t>& 参数。
callback：变量名，表示一个回调函数句柄，可以被复制和赋值（前提是所存对象可复制）。
const std::vector<uint8_t>&：用常量引用传参，避免拷贝，提高性能，同时保证回调不能修改传入的数据（除非去除 const）。

“类型擦除”（type erasure）意思是把具体类型的信息隐藏起来，只保留一个统一的接口来使用它。调用者不关心背后实际是什么类型，只按约定的签名调用即可。

std::function<R(Args...)> 就是一个类型擦除的可调用对象包装器：它能保存任意返回类型为 R、参数为 Args... 的可调用实体（普通函数指针、lambda、函数对象、std::bind 结果等），并通过统一的调用接口 operator() 调用它们。

工作原理（高层说明）：

std::function 内部保存了一个“调用器”函数指针和一个对实际可调用对象的存储（小对象优化时放在内部缓冲区，否则在堆上分配）。
当你用 f(args...) 调用时，std::function 通过调用器把调用转发给内部存储的具体可调用对象（具体类型对外不可见——被“擦除”了）。
调用方式：把 std::function 当作函数直接用 () 调用，例如 receive_callback_(data);。调用前应检查是否为空：if (receive_callback_) receive_callback_(data);。

推荐线程安全写法（防止回调在另一个线程被清空）：
auto cb = receive_callback_; if (cb) cb(data);

核心场景：何时需要 std::function 回调
1. 异步事件通知（最典型）
当类需要异步通知外部数据到达时，调用方无法阻塞等待：
2. 需要多态 callable（灵活绑定）
std::function 可以接受任意可调用对象：
3. 回调需要带状态（闭包）
4. 回调需要动态替换


实例化SerialPort之后传入符合要求的函数给set_receive_callback就相当于初始化了成员变量Callback callback_;

简短说明 — std::move 不是“搬数据”的函数，而是一个类型转换工具：

作用：将一个左值显式转换为对应的右值引用（rvalue reference），即把 T& 变成 T&&，以便触发移动语义（移动构造/移动赋值）。
本质：只做转换，不做拷贝或移动；真正的移动由被调用的移动构造函数或移动赋值来完成。
典型用途：把临时对象或不再需要的对象“交出”资源以避免拷贝开销，例如 vec2 = std::move(vec1); 会调用 vector 的移动赋值，转移内部缓冲区指针。
serial_protocol 是一个 C++ 命名空间（namespace），定义了串口协议的常量、数据结构与序列化/反序列化工具函数。

它包含的主要项：

协议边界：HEADER = {0xAA,0xAA}、FOOTER = {0xBB,0xBB}
尺寸常量：PACKET_SIZE（接收包长度，例中为 66）等
数据结构：SerialPacket（表示一帧的数据字段）
工具函数：serialize_packet()、deserialize_packet()、format_packet_for_debug() 等，用于打包/解析/调试输出。
作用：统一描述帧格式并提供打包/解析逻辑，供 SerialManager 的 process_buffer() 判断 HEADER/FOOTER、按 PACKET_SIZE 提取完整帧并调用回调。

std::search(first, last, pat_first, pat_last) 只在区间 [first, last) 内查找模式。
若未找到，返回的迭代器等于传入的 last（即 past‑the‑end）。
不能对该迭代器解引用，只能用来比较或计算距离（例如 if (it == v.end()) 或 std::distance(v.begin(), it)）。

解释 erase（以 std::vector 的成员为例，与你代码中 receive_buffer_.erase(...) 对应）：

功能：从容器中移除元素。常用重载：

iterator erase(const_iterator pos); — 删除单个位置，返回指向被删元素之后的迭代器（或 end()）。
iterator erase(const_iterator first, const_iterator last); — 删除区间 [first, last)，返回指向被删区间之后第一个元素的迭代器（或 end()）。
复杂度：对 std::vector，删除会把后面的元素向前移动以填补空位，时间复杂度为 O(N)（N = 被移动的元素数）。删除大量前端元素时代价高（会做大量搬移）。

迭代器/引用失效规则（重要）：

对 std::vector：删除后，指向被删除位置及其之后的所有迭代器、引用和指针都会失效（因为元素被移动）。只有返回的迭代器是新的有效位置。
因此在循环中使用 erase 时要小心不要使用已失效的迭代器。

std::equal 简要说明：

头文件：#include <algorithm>。
功能：比较两个序列的元素是否逐一相等，全部相等返回 true，否则 false。复杂度为线性 O(n)。
常用重载（概念）：

std::equal(first1, last1, first2)
比较区间 [first1, last1) 与从 first2 开始的同长度区间的元素是否一一相等。要求第二区间至少有 (last1-first1) 个元素。
std::equal(first1, last1, first2, pred)
使用自定义二元谓词 pred(a,b) 代替 == 做比较。
要点与注意：

返回值类型为 bool。
不要对 end() 解引用；first2 指向的序列必须足够长。
如果序列长度不匹配但调用的是前三参数版本，会越界读取第二序列，因此调用前务必保证长度或使用明确的两端比较（C++20 有 ranges helpers）。
适用于任意输入迭代器（但效率与迭代器类别有关）。
receive_buffer_.end()它表示“past‑the‑end”就是虽然指向了数组外的地址但是由于不可解引用所以不越界
end() 是 past‑the‑end（指向最后元素之后的位置），本身是合法的迭代器值；把它作为算法的 last 参数传入（例如 std::equal(..., packet_data.end())）是安全的，只要不解引用 end()。
不能解引用或读写 end()（*packet_data.end() 会是未定义行为）。如果要用 packet_data.end() - N，必须先保证 packet_data.size() >= N，否则 end()-N 会越界（非法）。

reset() 是 std::unique_ptr 的成员函数。简要要点：

功能：释放当前持有的指针并销毁所管理的对象（相当于 delete），随后将智能指针置为 nullptr。签名类似 void reset(pointer p = pointer()) noexcept。
行为：如果 unique_ptr 为空则什么也不做；若非空则先调用被管理对象的析构函数（执行清理逻辑），再释放资源。
区别：不要与 release() 混淆——release() 仅放弃所有权并返回裸指针，不会删除对象；reset() 会删除对象。
在你当前代码中的语义：serial_manager_.reset(); 会销毁 SerialManager 实例，从而触发其析构函数（该析构函数把 running_ = false、join 读取线程并关闭串口），这是安全停止后台线程并释放资源的一种正确方式（前提是没有其它线程持有该对象的裸指针）。

注意：在调用 reset() 前确保没有其他线程正在并发访问该对象的成员，否则会造成竞态或悬空访问。

| 内存序                        | 作用                         | 场景                                |
| -------------------------- | -------------------------- | --------------------------------- |
| `memory_order_relaxed`     | 无同步                        | 纯计数器                              |
| `memory_order_acquire`     | 读操作后，后续读写不能重排到此读之前         | `load`                            |
| `memory_order_release`     | 写操作前，先前读写不能重排到此写之后         | `store`                           |
| **`memory_order_acq_rel`** | **同时满足 acquire + release** | `read-modify-write`（如 `exchange`） |
| `memory_order_seq_cst`     | 最强顺序（默认）                   | 简单但慢                              |

	exchange无条件交换：直接写入新值

    重排（Reordering） 是指 编译器或 CPU 改变代码执行顺序 的优化行为，以提高性能。

    绝对名（以 / 开头）：不会被节点的 namespace 隐式前缀化，例如 /ACK 始终解析为 /ACK（除非显式 remap 改写）。

相对名（不以 / 或 ~ 开头）：会被节点的 namespace 前缀化。例如节点在命名空间 /robot1 下：

话题名 ACK → 解析为 /robot1/ACK
私有名（以 ~ 开头）：相对于节点的 namespace 与节点名，例如节点名 node1、namespace /robot1：

~ACK → /robot1/node1/ACK
另外：显式 remap（launch 或 CLI 的 -r）可以把任意名称（包括绝对名）映射到另一个名字。
这里要分 3 个名字：

package="btcpp_ros2_samples"
这是包名。

executable="sample_bt_executor"
这是运行的可执行程序文件名（进程入口）。

name="bt_executor"
这是 ROS 节点名（参数匹配、topic namespace、ros2 node list 看到的名字）。

你问“节点名指哪个”：

对 yaml 顶层键匹配来说，看的是 name="bt_executor" 这个 ROS 节点名。
不是 sample_bt_executor。
“宿主节点是啥”：

宿主节点就是这个 launch 里创建出来的 ROS 节点 bt_executor。
r2_initial_bt 这类 BT 插件是在这个宿主节点进程里被加载和运行的。
所以插件读取到的是 /bt_executor 这个节点的参数空间。
一句话：
插件参数要写到 bt_executor 这个节点名下，而不是写成 sample_bt_executor。
/**: 在 ROS2 参数文件里表示“通配所有节点（全局匹配）”。