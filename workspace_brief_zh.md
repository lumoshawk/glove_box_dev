# 工作区结构

本工作区把现场所有 EtherCAT 设备集成为 **ROS 2 (Jazzy) + ros2_control** 接口。上层应用只需要通过话题读写,
不必关心 EtherCAT 主站、PDO 映射、CiA402 状态机和 RS485 / CAN 协议。

## 1. 分层

```
上层应用(rclpy / 其他节点)               ← 只用话题:/<ctrl>/gpio_states 读,/<ctrl>/commands 写
        │
GPIO 控制器 (gpio_controllers/GpioCommandController),每个驱动包一个:
  /mdx_gpio  /eyou_gpio  /dh_gpio  /tuven_gpio  
        │
ros2_control_node + ethercat_driver (EthercatDriver) + GenericEcSlave 插件   ← src/ethercat_ros2_driver_mod-main
        │
IgH EtherCAT 主站 (/dev/EtherCAT0,同一时刻只能被一个进程占用)
        │
总线:CU1128 分线器、D1616BP IO 盒、MDX+ 伺服、(EYOU 关节、DH RS485 盒、GCAN CAN 盒 按现场接线)
```

## 2. 目录结构

```
glove_box_dev/
├── src/                         源码(colcon 工作区)
│   ├── ethercat_ros2_driver_mod-main/   上游 EtherCAT 驱动(ethercat_driver、generic_plugins、interface、msgs),基本不改
│   ├── mdx_ethercat/            MDX+_EC 伺服(CiA402):x2、y2
│   ├── eyou_ethercat/           EYOU PHU/RHU 关节模块(CiA402):r1、r2、r3
│   ├── dh_ethercat/             DH EtherCAT Box 4 Port:4 路 RS485 电爪 / 电缸
│   ├── tuven_ethercat/          TUVEN IO 盒 D1616BP(别名 52):换刀传感器 + 锁止执行器
│   ├── gcan_ethercat/           GCAN-8221 EtherCAT↔CAN 网关(别名 51),开发中
│   ├── elmo_ethercat/           Elmo Gold EtherCAT 驱动器(环路位置 5,无法写别名):read / enable 测试,已并入顶层 bringup
│   └── glove_box_bringup/       顶层:一个主站、一个 ros2_control_node,合并启动所有设备
├── ref/                         手册、ESI(xml)、alias_def.md(别名与 IO 逻辑定义)
├── guide.md  cheatsheet.md      使用说明 / 命令速查
└── build/ install/ log/         colcon 产物,不提交
```

每个驱动包结构一致:

```
<pkg>/config/<pkg>_params.yaml      系统参数:周期、master_id、加载哪些设备(motors:)
<pkg>/config/drives/<型号>.yaml      设备类型定义:身份 + PDO 映射 + 接口名(来自 ESI / 手册)
<pkg>/config/gateways/ 或 profiles/  一台设备 / 一个通道一份:总线地址(alias+position)、机械参数、限位、测试参数
<pkg>/<pkg>/system.py                配置 → ros2_control 的 xml、控制器 yaml、从站配置(build() 供 bringup 合并)
<pkg>/<pkg>/*_client.py              封装话题的 Python 客户端(可参考)
<pkg>/<pkg>/*_test.py                read / write 等硬件测试节点
<pkg>/launch/*.launch.py             单包独立启动
<pkg>/esi/  docs/  test/             ESI、说明、pytest(部分包有)
```

## 3. 设备与地址

| 设备 | 包 | 站点别名 | 控制器 | 说明 |
|---|---|---|---|---|
| MDX+_EC 伺服 `mdx_x2` / `mdx_y2` | `mdx_ethercat` | 6 / 7 | `/mdx_gpio` | CiA402,PP / PV / CSP |
| EYOU 关节 `eyou_r1..r3` | `eyou_ethercat` | 8 / 9 / 10 | `/eyou_gpio` | CiA402 |
| GCAN CAN 盒 | `gcan_ethercat` | 51 | `/gcan_gpio` | Retired |
| **TUVEN IO 盒 D1616BP** | `tuven_ethercat` | **52** | `/tuven_gpio` | sensor1/2 = DI1/2,actuator1/2 = DO1/2 |
| DH RS485 盒(4 口电爪) | `dh_ethercat` | 0(按环路位置);`alias_def.md` 规划 53 | `/dh_gpio` | 5 485 divices |
| Elmo Gold 驱动器 `elmo_m1` | `elmo_ethercat` | 无(EEPROM 只读,别名写不进;按环路位置 5 寻址) | `/elmo_gpio` | CiA402;须让 NetworkManager 忽略 `eoe*` 网口(已配置),见包 README |

设备按**别名**寻址(别名写入从站 EEPROM,`ethercat alias -p <ring> <alias>`,断电重启后由硬件加载),不依赖接线顺序;
配置里 `bus: {alias, position: 0}`,`position` 是相对别名的偏移。别名为 0 的设备才按环路位置(`ring`)寻址。

## 4. 上层应用的接口

每个控制器两个话题,消息类型相同 `control_msgs/msg/DynamicInterfaceGroupValues`:

| 话题 | 方向 | 内容 |
|---|---|---|
| `/<ctrl>/gpio_states` | 读 | 全部 state interface,按组(`interface_groups`)给出名字和值 |
| `/<ctrl>/commands` | 写 | command interface,可只写一部分;**控制器锁存**,值保持到下次写入 |

组名 / 接口名约定:

| 包 | 组名 | 接口名 | 例 |
|---|---|---|---|
| mdx / eyou | 电机名 | 字段名 | 组 `mdx_x2`:状态 `position`、`velocity`、`status_word`;命令 `control_word`、`mode`、`target_position`、`target_velocity` |
| dh | 网关 `dh_box1` | `<设备>_<字段>` | `gripper1_position`、`gripper1_target_position` |
| tuven | 网关 `io_box1` | `<设备>_input` / `<设备>_output` | 状态 `sensor1_input`;命令 `actuator1_output`(原始电平 0/1) |

最小示例(解锁执行器 1,传感器从 `/tuven_gpio/gpio_states` 的 `io_box1` 组读 `sensor1_input`,1 = 稳定):

```python
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
msg = DynamicInterfaceGroupValues(interface_groups=['io_box1'], interface_values=[
    InterfaceValue(interface_names=['actuator1_output'], values=[1.0])])   # 1 = 解锁,0 = 锁止
# 发布到 /tuven_gpio/commands
```

注意:

- 状态在从站到 OP 之前是 NaN;应用要先等到有限数值再用。
- MDX / EYOU 的位置 / 速度是驱动原始单位(counts),换算与零点、方向、齿轮比在 profile 的 `mechanics` / `calibration`;
  CiA402 使能、模式切换、PP 目标触发的时序参考各包 `*_client.py` 与测试节点,**不要自己拼控制字**。
- IO 盒无输出回读;输出在主站停止或 SM2 看门狗超时时回到 0(锁止),是安全状态。
- 换刀逻辑(锁止 → 延时 → 解锁 → 传感器稳定 3 s / 5 s → 锁止)已在 `tuven_ethercat` 提供:
  `ros2 run tuven_ethercat get_tool|switch_tool`,时序逻辑在 `tool_logic.py`(可被上层直接复用),参数在 `tuven_params.yaml`。

## 5. 启动方式

```bash
source /opt/ros/jazzy/setup.bash && source ~/glove_box_dev/install/setup.bash
colcon build --packages-select <pkg> --symlink-install      # 改 C++ / 新增文件 / setup.py 才需要重建;yaml 改后重启即可
ros2 launch tuven_ethercat tuven.launch.py                   # 单设备包独立启动
ros2 launch glove_box_bringup all.launch.py                  # 顶层:所有设备同一主站,500 Hz
```

顶层配置 `glove_box_bringup/config/bringup.yaml`:`bus:`(电机,`package/motor/ring`)与 `gateways:`(IO 盒、DH 盒等非电机从站);
`skip_absent: true` 会跳过总线上不存在的电机(缺一个从站会让共享过程数据域不完整,所有设备反馈变 NaN)。
各包保留自己的控制器和话题,顶层启动后单包话题与测试不变。

## 6. 约束与已知问题

1. **同一时刻只能有一个进程占用 EtherCAT 主站**:单包启动与顶层启动二选一。
2. 启动前 `pgrep -af "ros2_control_node|robot_state_publisher"` 必须为空;残留的 `robot_state_publisher` 会发布自己的
   `/robot_description`,`ros2_control_node` 可能取到它的而不是新启动的(已出现过)。停止时先停 `ros2 launch` 再停其 publisher。
3. 设备身份以 SII / ESI 为准(DH 盒的 CoE `0x1018` 与 SII 不同)。新增设备流程:查身份 → 取 ESI → 生成 drive 文件 →
   写 profile → 写 `system.py` → 测试(参考 `src/tuven_ethercat/README.md` 第 1 节)。

## 7. 文档与测试索引

| 内容 | 位置 |
|---|---|
| 命令速查 / 使用说明 | `cheatsheet.md`、`guide.md` |
| 别名与 IO 逻辑定义 | `ref/alias_def.md` |
| 各包说明 | `src/<pkg>/README.md`(mdx、dh、tuven、gcan) |
| IO 盒测试简报(含控制逻辑) | `src/tuven_ethercat/docs/test_brief_zh.md` |
| 离线测试 | `cd src/<pkg> && python3 -m pytest test -q`(dh、gcan、tuven、bringup 有) |
| 硬件测试 | 各包 `read_test` / `write_test` / `pp_test` / `get_tool` 等,退出码 0 = 通过 |
