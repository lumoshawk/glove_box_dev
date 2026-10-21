# TUVEN IO 盒 D1616BP 测试简报

## 1. 测试对象与环境

| 项目 | 内容 |
|---|---|
| 设备 | TUVEN TWEC-D1616BP(16 路 DI PNP/NPN + 16 路 DO PNP,24 V,0.5 A) |
| 从站标识 | vendor `0x345`,product `0x6D`(109),revision `0x20111` |
| 站点别名 | 52(`ethercat alias -p 2 52` 写入 SII,`ethercat slaves` 显示 `52:0`) |
| 软件包 | `tuven_ethercat`(`/tuven_gpio`,IgH 主站 + `GenericEcSlave`) |
| 接线(见 `ref/alias_def.md`) | `sensor1`=DI1,`sensor2`=DI2(接近传感器,高电平 = 稳定);`actuator1`=DO1,`actuator2`=DO2(锁止执行器,高电平 = 解锁,0 = 锁止) |
| 脚本参数(`tuven_params.yaml`) | `delay_s` = 3 s,`get_tool` `stable_s` = 3 s,`switch_tool` `stable_s` = 5 s,`timeout_s` = 60 s |

## 2. 控制逻辑简述

**电平约定**(`ref/alias_def.md`):传感器高电平 = 状态稳定(工具到位);执行器高电平 = 动作 = 解锁,0 = 锁止。输出上电默认 0(锁止),主站停止或 SM2 看门狗超时时输出回到 0,锁自动闭合。

**数据通路**:ESI → `gen_drive` → `d1616bp.yaml`(固定 16 位输入 `0x6000:1..16`、16 位输出 `0x7000:1..16`)→ `system.py` 只为接线的通道生成接口(`sensorN_input` 状态、`actuatorN_output` 命令)→ `/tuven_gpio`(GPIO 控制器,命令保持锁存)→ 脚本经 `IoClient` 读写。

**取工具 `get_tool` / 换工具 `switch_tool`**(同一状态序列,仅 `stable_s` 不同:3 s / 5 s):

```
锁止 actuator ──► 延时 delay_s ──► 解锁 actuator ──► 轮询配对传感器(20 ms)
                                                     │ 稳定:开始/继续计时
                                                     │ 丢失:计时清零
                                                     ├─ 连续稳定 ≥ stable_s ─► 成功(PASS)
                                                     └─ 超过 timeout_s ──────► 失败(FAIL,退出码 1)
                                          无论成功、超时、异常或 Ctrl-C,最后都 ──► 锁止 actuator
```

- 第一步锁止假定换刀机构处于初始状态(取工具时槽位为空,换工具时已装有工具)。
- 每个执行器对应自己的传感器(`actuator1`↔`sensor1`,`actuator2`↔`sensor2`),由 profile 的 `sensor` 字段配对。
- "稳定"指传感器连续保持高电平;任何一次丢失都会使计时从头开始。
- 脚本默认只操作第一个执行器,`-p device:=actuator2` 切换;`delay_s`、`stable_s`、`timeout_s` 可在 `tuven_params.yaml` 或命令行覆盖。
- `write_test` 仅做 锁止 → 解锁 `hold_s` → 锁止,`read_test` 只读不写。

## 3. 测试项目与结果

| # | 测试 | 命令 | 结果 | 说明 |
|---|---|---|---|---|
| 1 | 单元测试(离线 + 在线身份/别名检查) | `python3 -m pytest test -q` | **通过(21 项)** | 覆盖配置、ESI 与驱动文件一致性、PDO 映射、别名 52、锁止/解锁电平、3 s / 5 s 稳定逻辑、传感器抖动重新计时、超时、中断后回锁 |
| 2 | 总线与状态机 | `ethercat slaves` | **通过** | 别名 52 的 D1616BP 由 PREOP 进入 OP;过程数据 4 字节,WC 3/3 |
| 3 | 输出位映射 | `ethercat data` | **通过** | actuator1 → 第 1 位 `0x01`,actuator2 → 第 2 位 `0x02`,同时 `0x03` |
| 4 | 读测试 | `read_test` | **通过** | 两路传感器电平均可读取,不写入任何输出 |
| 5 | 写测试 | `write_test` | **通过** | 锁止 → 解锁 → 锁止,命令电平 0 → 1 → 0,结束时保持锁止 |
| 6 | 取工具 | `get_tool` | **通过** | 见下 |
| 7 | 换工具 | `switch_tool` | **通过** | 见下 |

### 取工具(`get_tool`,actuator1 / sensor1)

| 时间(s) | 动作 |
|---|---|
| 93.24 | 锁止 actuator1(假定工具为空) |
| 96.25 | 延时 3 s 后解锁,等待装上工具 |
| 113.67 | sensor1 变为稳定,开始计时 |
| 116.67 | 连续稳定满 3 s,重新锁止 |

结论:**通过**,总耗时 23.4 s(大部分为人工装工具的等待时间)。

### 换工具(`switch_tool`,actuator1 / sensor1)

| 时间(s) | 动作 |
|---|---|
| 159.74 | 锁止 actuator1(假定已装有工具) |
| 162.74 | 延时 3 s 后解锁,等待换装 |
| 162.74 – 185.44 | sensor1 多次"稳定 → 丢失 → 重新计时"(162.90、163.22、180.28、183.03、183.56、184.60 丢失) |
| 185.44 | sensor1 最后一次变为稳定 |
| 190.45 | 连续稳定满 5 s,重新锁止 |

结论:**通过**,总耗时 30.7 s。传感器每次丢失都会使 5 s 计时清零,最终只有连续 5 s 稳定才锁止,符合设计。

## 4. 观察与说明

1. **输出无回读**:该盒没有输出回读,`write_test` 只能确认命令已下发、结束时回到锁止;解锁/锁止的实际动作需看输出指示灯或执行器本身(已用 `ethercat data` 核对过输出位)。
2. **换工具刚解锁时传感器仍为高**:`switch_tool` 在 162.74 s 解锁瞬间 sensor1 就是"稳定"(旧工具仍在位),随后因工具被取下/晃动而丢失。当前逻辑按规格只要求"稳定满 5 s",不要求先看到传感器变低;若旧工具恰好在位 5 s 不动,会提前锁止。需要时可增加"先等待传感器变低"的步骤(未实现)。
3. **传感器抖动**:换工具过程中多次短时丢失,属于装卸时的机械抖动,脚本已按"重新计时"处理。若现场抖动偏多,可调整传感器安装距离或增大 `stable_s`。
4. **延时参数**:`alias_def.md` 未给出延时,默认 1 s,本次测试按配置使用 3 s。
5. **别名**:别名已写入 SII,断电重启后由从站硬件加载;不再依赖环路位置(当前为 ring 2)。
6. **环境**:测试前已停止历史残留的 `gcan_ethercat` 启动进程及其 `robot_state_publisher`,避免其发布的 `/robot_description` 干扰 `ros2_control_node`。

## 5. 复现

```bash
source /opt/ros/jazzy/setup.bash && source ~/glove_box_dev/install/setup.bash
ros2 launch tuven_ethercat tuven.launch.py        # 终端 1,保持运行
ros2 run tuven_ethercat read_test                 # 终端 2
ros2 run tuven_ethercat write_test                # 锁会动作
ros2 run tuven_ethercat get_tool                  # 脚本退出码 0 = 通过
ros2 run tuven_ethercat switch_tool
cd src/tuven_ethercat && python3 -m pytest test -q
```
