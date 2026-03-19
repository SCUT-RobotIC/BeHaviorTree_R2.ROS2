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
