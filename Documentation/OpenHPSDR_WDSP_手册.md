# OpenHPSDR 协议与 WDSP 库调用详解手册

> **版本**：v1.0（2026-10-19）
> **适用代码基**：Thetis 主分支（本仓库）
>
> 本手册基于对仓库源码的逐行核查编写，所有结论均可在下列文件中验证：
> - `Project Files/Source/ChannelMaster/network.c`（1249 行，协议 2 / ETH 实现）
> - `Project Files/Source/ChannelMaster/network.h`（435 行，数据结构与端口定义）
> - `Project Files/Source/ChannelMaster/networkproto1.c`（727 行，协议 1 / USB-Metis 实现）
> - `Project Files/Source/ChannelMaster/netInterface.c`、`cmaster.c`、`cmbuffs.c`、`obbuffs.c`（网络↔DSP 胶水层）
> - `Project Files/Source/wdsp/channel.h/.c`、`iobuffs.h/.c`、`analyzer.h/.c`、`comm.h`（WDSP 引擎本体）
> - `Project Files/Source/Console/dsp.cs`（C# P/Invoke 绑定）、`Console/HPSDR/specHPSDR.cs`、`Console/HPSDR/NetworkIOImports.cs`、`Console/enums.cs`
> - `Documentation/Radio/WDSP Guide, Rev 1.23.pdf`（官方 WDSP 参考文档）

---

## 目录

- [第一部分：OpenHPSDR 网络协议](#第一部分openhpsdr-网络协议)
  - [1. 协议总览与代码组织](#1-协议总览与代码组织)
  - [2. 协议 2（ETH）端口体系全解](#2-协议-2eth端口体系全解)
  - [3. 协议 1（USB / Hermes-I + Metis）深入](#3-协议-1usb--hermes-i--metis深入)
  - [4. 网络抽象公共层与生命周期](#4-网络抽象公共层与生命周期)
- [第二部分：WDSP 库深入讲解](#第二部分wdsp-库深入讲解)
  - [5. 总体架构](#5-wdsp-总体架构)
  - [6. 信道编号与生命周期](#6-信道编号与生命周期openchannelclosechannelsetchannelstate)
  - [7. 数据面核心：fexchange0 逐行剖析](#7-数据面核心fexchange0-逐行剖析)
  - [8. ChannelMaster 如何编排 WDSP](#8-channelmaster-如何编排-wdsp接收发射流水线)
  - [9. 频谱分析器 API](#9-频谱分析器-api-analyzerh--spechsdr-cs)
  - [10. C# P/Invoke 绑定层](#10-c-pinvoke-绑定层console-dspcs)
  - [11. 独立工具块速查](#11-独立工具块速查)
- [第三部分：端到端集成与时序](#第三部分端到端集成与时序)
  - [12. 完整启动/运行/关闭时序](#12-完整启动运行关闭时序协议-2-为例)
  - [13. 工程注意事项汇总](#13-工程注意事项汇总)
  - [14. 附录 A：常用函数签名清单](#14-附录-a常用函数签名清单已从源码核对)
  - [15. 附录 B：术语表](#15-附录-b术语表)

---

# 第一部分：OpenHPSDR 网络协议

## 1. 协议总览与代码组织

OpenHPSDR 协议族定义了 PC 控制软件（Thetis、PowerSDR 等）与 ANAN/Hermes/Orion
系列开源电台（FPGA + Metis/Mercury/Penelope/Apollo 子板）之间的通信规范。
Thetis 中由全局枚举选择协议版本（见 `network.h`）：

```c
enum _RadioProtocol
{
    USB = 0,  // Protocol USB (P1) —— Hermes-I + Metis 桥接器
    ETH = 1   // Protocol ETH (P2) —— Hermes-II/Orion 原生以太网
} RadioProtocol;
```

| 特性 | 协议 1 (USB) | 协议 2 (ETH) |
|---|---|---|
| 传输介质 | USB（经 Metis 桥接为 UDP） | 原生以太网 UDP |
| Socket 模型 | 单一端口复用（全部走 1024） | 多端口分发（1024~1041） |
| 帧格式 | `0xEF 0xFE` 头 + 端点 + 序号 + 1024字节载荷 | 4字节大端序号 + 变长载荷 |
| 乱序处理 | ARQ/重排模块 `xpro`（`pro.c`） | 序列号检测 + 快照日志 |
| C&C 方式 | 控制字节 C0-C4 轮换嵌入数据帧 | 独立的高优先级 UDP 包 |
| DDC 数量 | 由 C4 位段编码（nddc=2/4/5） | Portmap 静态映射 7 路（1035-1041） |
| 实现文件 | `networkproto1.c` | `network.c` |

硬件型号枚举（`network.h` `_HPSDRHW`）：
`Atlas=0, Hermes=1(ANAN-10/100), HermesII=2(ANAN-10E/100B), Angelia=3(ANAN-100D), Orion=4(ANAN-200D), OrionMKII=5(ANAN-7000/8000DLE)`。

## 2. 协议 2（ETH）端口体系全解

所有数据包前 **4 字节为大端序 32-bit 序列号**，收发双方各自维护计数器用于丢包统计
（`radionet` 结构中的 `rx_in_seq_no/rx_in_seq_err` 等字段）。`BUFLEN` 定义为
**1444 字节**（`network.c`），这是 DDC/C&C 的标准帧长。

### 2.1 PC → 电台（发送方向）

| 端口 | 函数（network.c） | 长度 | 功能 |
|---|---|---|---|
| **1024** | `CmdGeneral()` | 60 B | Portmap：FPGA 数据流端口映射 + 宽带参数 + 看门狗 + PA/Alex 使能 |
| **1025** | `CmdRx()` | 1444 B | RX 专用配置：ADC 数、dither/random、DDC 使能位图、各路采样率/位深、sync |
| **1026** | `CmdTx()` | 60 B | TX 专用配置：CW 关键器参数、TX 采样率、相位偏移、麦克风控制、阶梯衰减 |
| **1027** | `CmdHighPriority()` | 1444 B | 高优先级实时 C&C：Run/PTT/CWX/Dot/Dash + 12 路 RX 频率 + TX 频率/驱动电平 + ALEX 滤波字 + 用户输出 |
| **1028** | `WriteUDPFrame(id=0)` | 音频 | RX 音频回传（PCM 模式用） |
| **1029** | `WriteUDPFrame(id=1)` | spp×6 B | TX I&Q 样本（PureSignal 校准环回所需） |

**Portmap 包（1024）关键字节**（逐字节对照 `CmdGeneral()` 源码）：

```
[4]     命令码 0x00
[5:6]   Rx Specific → 端口 1025        [7:8]   Tx Specific → 端口 1026
[9:10]  High Priority from PC → 1027   [11:12] High Priority to PC → 1025
[13:14] Rx Audio → 1028                [15:16] Tx0 I&Q → 1029
[17:18] Rx0 base port（rx_base_port，默认 1035，即 DDC0~DDC6 → 1035~1041）
[19:20] Mic Samples → 1026             [21:22] Wideband ADC base port（默认 1027）
[23]    宽带使能位图 WB0..WB7          [24:25] 宽带每包样本数（512）
[26]    宽带样本位宽（16bit）          [27]    宽带更新率(ms)
[28]    每帧宽带包数（默认32）         [33:36] EER PWM max/min
[37]    0x08 = 发送相位字              [38]    看门狗定时值 wdt（0=禁用）
[58]    bit0=!PA（PA使能反转）+ Apollo/时钟源
[59]    Alex 使能（prbpfilter->enable | prbpfilter2->enable）
```

**高优先级 C&C 包（1027）关键字节**（对照 `CmdHighPriority()`）：

```
[4]       bit0=Run, bit1=PTT[0], bit7=PureSignal
[5]       bit0=CWX, bit1=Dot, bit2=Dash
[9:12]    RX0 频率(Hz, 大端32bit)；当 puresignal_run && PTT 时改写为 TX 频率
[13:16]   RX1 频率 … [53:56] RX11 频率（共12路，每路4字节）
[329:332] TX0 频率                     [345] TX0 drive level（DAC增益）
[1400]    xvtr_enable | (!audioamp_enable)<<1 | atu_tune<<2   // MW0LGE_22b
[1401]    开漏输出 oc_output<<1        [1402] 用户数字输出 DB9 pin1-4
[1403]    Mercury 20dB 衰减器          [1430:1431] Alex2 滤波字
[1432:1435] Alex1 32bit 滤波字（高16位TX低通/带通，低16位RX高通）
[1442:1443] ADC1/ADC0 RX 阶梯衰减(0-31dB)
```

ALEX 带通滤波器的 32 位字在 `network.h` 的 `_rbpfilter` 结构体中按位域精确声明
（bit0=rx_yellow_led、bit1~6=各 HPF、bit8~10=XVTR/EXT1/EXT2 输入选择、
bit12=Bypass、bit13/14=20/10dB 衰减、bit18~23=TX LPF 组、bit24~26=ANT1~3、
bit27=TR 继电器……），发送时拆成 4 个字节写入 `[1432..1435]`。

**RX 专用包（1025）**（对照 `CmdRx()`）：`[4]`=num_adc，`[5]/[6]`=各 ADC
dither/random 位，`[7]`=Rx0~Rx6 使能位图，随后每路 RX 以固定偏移给出 ADC 选择、
16bit 采样率（48/96/192/384k…）、位深（16/24）；`[1363]`=SyncRx0（多样性合成
Rx0+Rxn）。

**TX 专用包（1026）**（对照 `CmdTx()`）：`[5]`=CW mode_control 位域（EER/CW/Reverse/
Iambic/Sidetone/ModeB/StrictSpacing/BreakIn，见 `network.h` `_cw` 结构），`[6]`=侧音电平，
`[7:8]`=侧音频率，`[9]`=键速，`[10]`=重量比，`[11:12]`=hang delay，`[13]`=RF delay，
`[14:15]`=TX 采样率，`[26:27]`=IQ 相位偏移(0-359°)，`[50]`=mic_control 位域
（LineIn/MicBoost/PTT线序/TRS/bias），`[51]`=线路输入增益，`[57:59]`=各 ADC TX 阶梯衰减。

### 2.2 电台 → PC（接收方向）

`ReadThreadMainLoop()` 通过 WSAEvent 异步等待 `listenSock` 上的 `FD_READ`，
收到后由 `ReadUDPFrame()` 解析并按端口分发：

| 端口 | 内容 | 处理方式（源码行为） |
|---|---|---|
| **1025** | 高优先级状态回报 | 见下方字节表，更新 `prn` 状态字段 |
| **1026** | MIC 音频（1440B，16bit@48k） | 转 double 存入 `TxReadBufp` → `Inbound(inid(1,0), mic.spp, …)` |
| **1027~1034** | 宽带 ADC0~7 原始数据 | 在 `ReadUDPFrame` 内直接喂 `Spectrum(disp_id,…)` 给频谱分析器（帧状态机 wb_state 0/1，序号错则补零凑帧） |
| **1035~1041** | DDC0~DDC6 I&Q（1444B/包） | 校验长度→记序号差→24bit 解交织→`xrouter(0,0,rc,spp,RxReadBufp)` 路由到各接收流 |

**1025 状态回报字节表**（对照 `ReadThreadMainLoop` case 1025）：

```
[0]:  bit0=PTT输入  bit1=Dot  bit2=Dash  bit4=PLL锁定(MW0LGE_21d)
[1]:  bit0~7 = ADC0~7 过载位图
[2:3]=激励器功率(12bit符号扩展)   [10:11]=前向功率   [18:19]=反射功率
[26:27]=硬件LED状态               [45:46]=电源电压
[47:48]=UserADC3  [49:50]=UserADC2  [51:52]=UserADC1(AIN4)  [53:54]=UserADC0(AIN3)
[55]: 用户数字输入 IO4/5/6/8/2 位图
```

其中前向/反射功率经 `PeakFwdPower/PeakRevPower` 做峰值保持（指数衰减 `MDECAY`），
`user_adc0` 同步送 `SetAmpProtectADCValue(0,…)` 供功放保护逻辑使用。

**DDC 包的解交织数学**（case 1035~1041）：每复样点占 6 字节（I 3 字节 + Q 3 字节，
24bit 大端左对齐），换算公式为

```
value = (1.0 / 2^31) * (double)(b0<<24 | b1<<16 | b2<<8)
```

即满量程 ±1.0 的定点归一化。238 复样点/包 × 6B + 4B 序号 + 填充 = 1444B。
转换后的双精度交织 I/Q 缓冲交给 `xrouter`，由其按“流”复制到主副接收通路。

### 2.3 发现广播（Discovery）

电台周期性在 **UDP 4991** 端口广播 40 字节发现包，Thetis 解析进
`struct _discovery`（`network.h`）：MAC 地址、BoardType（对应 HPSDRHW 枚举）、
协议版本、固件版本、Mercury 0-3 版本、Penny/Metis 版本、numRxs。
UI 用它决定可用带宽、Alex 组数、是否支持 PureSignal 等能力。

### 2.4 看门狗与失锁（LOS）处理

- `packetbuf[38] = prn->wdt`：非零时电台启动秒级看门狗，PC 必须持续发 1024 包，
  否则 FPGA 停止振荡（防止断网后电台卡发射）。
- `KeepAliveLoop()` 创建 Windows WaitableTimer，周期 **500ms**；仅当
  `run && wdt` 时循环调用 `CmdGeneral()` 保活。
- `ReadThreadMainLoop` 中 `WSAWaitForMultipleEvents` 超时值取
  `prn->wdt ? 3000 : WSA_INFINITE`——一旦 3 秒无包即判定 LOS：置 `HaveSync=0`、
  调 `SendStop()`、并向 RX/TX 入口灌零样本（`memset` + `Inbound`），保证 DSP
  流水线不断流、音频静音而非崩溃。
- 读线程优先级：`AvSetMmThreadCharacteristics("Pro Audio")` +
  `AvSetMmThreadPriority(2)`，失败则退化为 `THREAD_PRIORITY_TIME_CRITICAL`——
  这是低延迟音频路径的关键技巧。

### 2.5 序列号诊断子系统

每个 DDC 维护 `rx_in_seq_delta[40]` 环形缓冲记录“期望序号-实际序号”差值；发生跳变时
`addSnapShot()` 把当前环内容连同时间戳、received/last 序号打包为
`_seqLogSnapshot_t` 链表节点（上限 20 条）。C# 侧通过导出函数 `getOOO()`（乱序计数）
与 `getSeqInDelta(bInit, rx, deltas[], dateTimeStamp, out received, out last)` 读取，
用于“Radio Diagnostics”界面定位网络抖动。

## 3. 协议 1（USB / Hermes-I + Metis）深入

### 3.1 帧结构与握手

所有流量复用单个 UDP socket（端口 1024），帧头 8 字节：

```
[0][1] = 0xEF 0xFE        // 帧标志
[2]    = 方向/类型        // 0x01=数据帧, 0x04=控制帧
[3]    = endpoint         // 2=PC→FPGA, 6=FPGA→PC
[4:7]  = 大端32位序号
[8:…]  = 载荷（1024字节数据帧）
```

**启动握手 `SendStartToMetis()`**（`networkproto1.c`）：先
`create_pro(1, 1024, 16, 5)` 建立 ARQ 重排上下文；随后最多 5 次循环：发
`ForceCandCFrame(1)`（注入 C&C）→ 发控制帧 `{EF FE 04 01}`（endpoint 0x01 = start）
→ `MetisReadDirect()` 探测 → 若 `MetisLastRecvSeq` 变化则成功返回 0，否则
Sleep(10) 重试，最终失败返回 -1。**停止** `SendStopToMetis()` 发
`{EF FE 04 00}` 并等待序号停增，然后 `destroy_pro(prop)`；注意源码注释
（MW0LGE_21g）：因 setup.cs 可能触发不配对的 Stop，`prop=NULL` 防重复销毁。

### 3.2 控制字节 C0-C4 轮换机制

协议 1 没有独立 C&C 通道，控制信息以 5 字节一组嵌入每个 512 字节半帧
（每 1024 帧含 2 个半帧），载荷开头是同步字节 `0x7F 0x7F 0x7F`。
`WriteMainLoop()` 用轮转索引 `out_control_idx` 依次发出不同语义的控制组：

| idx | C0 值 | 含义 |
|---|---|---|
| 0 | 0x00 | 通用设置：C1=采样率2bit码，C2=EER位+开漏输出，C3=ALEX衰减/前端选择/dither/random，C4=天线+dupent位+`(nddc-1)<<3`+diversity<<7 |
| 1 | 0x02 | TX VFO 频率（C1-C4 = Hz 大端 4 字节） |
| 2 | 0x04 | RX1/DDC0 频率（Hermes-II 发射且 PureSignal 运行时改用 TX 频率） |
| 3 | 0x06 | RX2/DDC1 频率（nddc==5 时为 RX0 频率；nddc==2&&PS 时为 TX 频率） |
| 4 | 0x1C | ADC 分配字 P1_adc_cntrl + TX 阶梯衰减 |
| 5 | 0x08 | RX3/DDC2 频率 |
| 6 | 0x0A | RX4/DDC3 频率（恒为 TX 频率，供 PureSignal 反馈） |

T/R 切换瞬间（`XmitBit != PreviousTXBit` 且 nddc==2）强制跳到 idx=2 立刻刷新
DDC0 频率——这是 Hermes-II 只有 2 个物理 VFO 时的补偿逻辑。

### 3.3 上行数据的多样本交织解包

一个 512B 半帧内，nddc 路 DDC 的 I&Q 与 MIC 数据按样点交错排列。解码核心：

```
spr = 504 / (6*nddc + 2)          // 每DDC样点数（nddc=2→63, 4→36, 5→28）
第iddc路第n样点偏移 k = 8 + n*(6*nddc+2) + iddc*6
MIC样点偏移          k = 8 + nddc*6 + n*(2 + nddc*6)   // 再做 mic_decimation_factor 抽取
```

解出的各 DDC 缓冲按 nddc 分派：`nddc==2` 时 `twist(spr,0,1,1035)` 将两路去交织合并；
`nddc==4` 时 DDC0/2 直入 `xrouter(1035/1037)`、DDC1/3 成对 twist 进 1036；
`nddc==5`（Orion）三对组合分别入 1035/1036/1037。下行（TX）方向
`sendProtocol1Samples` 线程在 IQ/LR 两个信号量齐备后组装 endpoint 2 帧发出；
EER 运行时 I&Q 与 L&R 需在 `outIQbufp` 内做 63 样点的重新交织
（`sendOutbound` 中的三段指针搬运）。

### 3.4 协议 1 的下行状态回报

C0 高位段（`ControlBytesIn[0] & 0xF8`）区分 5 种遥测语义（case 0x00~0x20）：
ADC 过载+数字输入、激励/前向功率、反射功率+PA电压、PA电流+电源电压、
三 ADC 过载位图——字段与协议 2 的 1025 包等价，只是分时复用。

## 4. 网络抽象公共层与生命周期

- `nativeInitMetis(netaddr, localaddr, localport, protocol)`（C# 侧
  `NetworkIOImports.cs`）设定 `MetisAddr = inet_addr(...)` 并建 socket；
  `RadioProtocol` 在此处赋值，随后 `UpdateRadioProtocolSampleSize()`
  （netInterface.c）按协议重算 `spp`：协议 2 下 `rx[i].spp=238, tx[0].spp=240,
  mic.spp=640, audio[i].spp=64`；协议 1 下改为 spr 动态值。
- 发送统一走 `sendPacket()`：进入 `sndpkt` 临界区后 `sendto(MetisAddr, port)`，
  保证多线程发包原子性。
- `create_rnet()/destroy_rnet()`（netInterface.c）负责 `radionet` 结构分配、
  缓冲对齐（`_aligned_malloc` 16 字节）、多个 CRITICAL_SECTION 初始化（自旋计数
  2500），并在末尾调用 `SendpOutbound(OutBound)` 把 DSP 出口回调挂到
  ChannelMaster。
- 启动顺序（`StartAudioNative`）：`RadioProtocol==USB` 时先 `SendStartToMetis()`
  握手，成功后 `run=1; CmdHighPriority()`；协议 2 直接 `run=1` 并发 C&C。

---

# 第二部分：WDSP 库深入讲解

## 5. WDSP 总体架构

WDSP（Warren Pratt's DSP library，GPLv2，作者 NR0V）编译为 `wdsp.dll`，
`cdecl` 导出（`channel.h` 中 `#define PORT extern "C" __declspec(dllexport)`）。
内部三大子系统：

1. **信道处理**（`channel.c` / `iobuffs.c` / `RXA.c` / `TXA.c`）：收/发 DSP 流水线
   （滤波、AGC、解调、ALC、压缩、EQ）；
2. **分析器**（`analyzer.c`）：FFT 频谱/瀑布显示引擎；
3. **独立工具块**（`nb.c`,`anf.c`,`anr.c`,`emnr.c`,`eq.c`,`compress.c`,`eer.c`,
   `ilv.c`,`dexp.c`,`txgain.c`,`cfcomp.c`,`amsq.c`,`div.c`,`osctrl.c`,`notchvar.c`…）：
   可脱离信道单独实例化。

Thetis 并不让 C# 直接驱动全部流程，而是把 WDSP 嵌进中间层 **ChannelMaster.dll**
（`cmaster.c` 等），C# 只管理参数与音频设备；两者通过回调相连。
依赖库：FFTW3（wisdom 文件需预加载）、portaudio 无关（音频由 Thetis 自己管）。

## 6. 信道编号与生命周期（OpenChannel/CloseChannel/SetChannelState）

`comm.h` 第 96 行定义 `MAX_CHANNELS = 32`。约定：

```
接收信道编号: 0 .. 31        (type = 0)
发射信道编号: 32 .. 63       (type = 1)   // channel = type * MAX_CHANNELS + number
```

ChannelMaster 层的映射函数 `chid(stream, subrx)`（cmsetup.c）：

```c
case 0 /*rx*/: ch_id = cmSubRCVR * stream + subrx;                 // 接收：每流多个子接收信道
case 1 /*tx*/: ch_id = stream + (cmSubRCVR - 1) * cmRCVR;          // 发射：偏移到高端编号
```

**OpenChannel 完整签名**（`channel.h` 第 62 行，已核对源码）：

```c
PORT void OpenChannel (int channel, int in_size, int dsp_size,
                       int input_samplerate, int dsp_rate, int output_samplerate,
                       int type, int state,
                       double tdelayup, double tslewup,
                       double tdelaydown, double tslewdown,
                       int bfo);
```

Thetis 中的实际调用值（cmaster.c create_rcvr/create_xmtr）：

```c
// 接收信道实例
OpenChannel(chid, xcm_insize[in_id], 4096, xcm_inrate, 48000,
            rcvr[i].ch_outrate, /*type=*/0, /*state=*/0,
            0.010, 0.025, 0.000, 0.010, /*bfo=*/1);
// 发射信道实例
OpenChannel(chid, xcm_insize, 4096, xcm_inrate, 96000,
            xmtr[i].ch_outrate, /*type=*/1, /*state=*/0,
            0.000, 0.010, 0.000, 0.010, /*bfo=*/1);
```

要点：

- `type`: 0=接收、1=发射；`state` 初始 0（关），由
  `SetChannelState(ch, 1, dmode)` 激活，dmode 控制 T/R 切换时是否立即冲刷缓冲。
- `tdelayup/tslewup/tdelaydown/tslewdown` 构成上/下斜波（slew）参数，避免 T/R
  切换爆音；内部由 `create_slews()` 生成线性渐变乘数。
- `bfo=1`（block for output）：`fexchange0` 会阻塞直到该信道的处理后输出就绪——
  Thetis 全程使用 bfo=1，使调用线程天然与 DSP 线程同步节流。
- 三个速率 `in_rate → dsp_rate → out_rate` 各配一个多相重采样器链；改单速率用
  `SetInputSamplerate` 等，同时改三个必须用 **`SetAllRates(channel, in, dsp, out)`**
  （`channel.h` 第 80 行），它在临界区保护下重建全部 FIR 系数，防止中间态撕裂。
- `CloseChannel(int channel)`（`channel.h` 第 64 行）逆序释放 iobuffs、resamplers、
  RXA/TXA 块。

## 7. 数据面核心：fexchange0 逐行剖析

接口声明（`iobuffs.h` 第 90 行，已核对源码）：

```c
void fexchange0 (int channel, double* in, double* out, int* error);
```

每次调用交换**一整块**交织双精度复样点：输入 `in_size` 个复样点（2×in_size 个
double），输出 `out_size` 个复样点。`iobuffs.h` 中的关键字段注释即为契约：

```c
int in_size;    // input number of complex samples in a fexchange call
int out_size;   // output number of complex samples in a fexchange call
```

内部机制（iobuffs.c）：

1. 若 `ch[channel].exchange == 0`（信道停用/正在关闭）直接返回——这就是为什么
   `SetChannelState(0)` 后数据流自动无声中断。
2. 进入 `csEXCH` 临界区，取 exchange 用途的 IOB 副本。
3. **上行**：若处于 up-slew 状态则 `upslew0()` 渐入，否则 `memcpy` 写入输入伪环
   `r1_baseptr`；累计 `r1_unqueuedsamps ≥ r1_outsize` 时按整数倍
   `ReleaseSemaphore(Sem_BuffReady)` 通知 DSP 线程取 `dsp_size` 块处理。
   索引回绕：`r1_inidx += in_size; if == active_buffsize → 0`。
4. **下行**：在 `r2_ControlSection` 中原子判断 `r2_havesamps ≥ out_size`；
   `bfo` 则 `WaitForSingleObject(Sem_OutReady, INFINITE)` 等 DSP 线程产出。
   有数据→（down-slew 或 memcpy 从 r2 环取出）；无数据→`memset(out,0)` 并
   `*error += -2`（欠载错误码；-1 保留给 r1 溢出覆盖）。
5. 下斜波结束时复位 exchange 位并 `ReleaseSemaphore(Sem_Flush)`，配合
   `flushChannel()` 完成安全关闭。

DSP 侧存在独立执行线程，等待 `Sem_BuffReady` 后依次跑
resampler → AGC → 滤波 → 解调（RXA 流水线）并把结果压回 r2 环、释放
`Sem_OutReady`。因此 **fexchange0 是“生产者-消费者”边界**，控制台线程永不长时间
占用 CPU。

同族接口：`fexchange2`（分离 I/Q、float 型，供外部浮点应用，见 `iobuffs.h` 第 93 行）；
Thetis 主要用 `fexchange0`（double 交织，满量程 ±1.0）。

## 8. ChannelMaster 如何编排 WDSP（接收/发射流水线）

**接收流水线**（`xcmaster()` case 0，stream=某 RX 流）：

```
网络线程 Inbound(id,n,RxReadBufp) ──写入──> pcm->pcbuff[id] 输入伪环
   cmb_main 线程: WaitFor(Sem_BuffReady) → cmdata() 取 in_size 块 → xcmaster(id):
       xpipe(stream,0,in)                    // 管道钩子(校准注入等)
       xanb / xnob                           // 模拟噪声消隐 / 脉冲噪声消隐(前置)
       Spectrum0(run_pan, rx, 0, 0, in)      // 原样喂频谱分析器(panadapter)
       for j in subrcvr:
           fexchange0(chid(stream,j), in, rcvr[rx].audio[j], &err)   // ★ DSP 信道
       xpipe(stream,1,audio)
       xMixAudio(...)                        // 混入监听总线 + anti-vox 混音器
   混合输出经 Outbound() 回调 → 声卡播放
```

**发射流水线**（case 1）：

```
声卡采集 → Inbound(mic流) → cmb_main → xcmaster(tx stream):
    xpipe → xdexp(tx)                        // VOX 动态扩展器
    fexchange0(chid, in, xmtr[tx].out[0])    // ★ 发射DSP(滤波/ALC/EQ/压缩)
    xpipe → xMixAudio(监视)
    xtxgain(pgain)                           // Penelope增益 + 功放保护
    xeer(peer)                               // EER 包络分解(可选)
    xilv(pilv, out)                          // 交织 EER/IQ 并调用 OutBound()
OutBound(id, nsamples, in) → obp.pebuff[id] 输出伪环 → obb_main 线程
    → sendOutbound(id,...) → network.c 里量化成 24bit 大端 → WriteUDPFrame(1029)
```

`Inbound()`（cmbuffs.c）与 `OutBound()`（obbuffs.c）互为镜像的伪环写入函数，都带
`accept` 门控位 + 临界区 + 环绕 memcpy（first/second 两段拼接），并用信号量把
“攒够一块”事件传递给处理线程。`SetCMRingOutsize()` 展示了热改块长的标准舞蹈：
关门→停线程→冲刷→改尺寸→重启→开门。

## 9. 频谱分析器 API（analyzer.h / specHPSDR.cs）

**创建**（`analyzer.h` 第 149 行，签名带参数注释，已核对源码）：

```c
void XCreateAnalyzer (int disp,
                      int *success,        // writes '0' to success if all went well, <0 if mem alloc failed
                      int m_size,          // maximum fft size to be used
                      int m_LO,            // maximum number of LO positions per subspan
                      int m_stitch,        // maximum number of subspans to be concatenated
                      char *app_data_path);
```

Thetis 对每个接收流及 TX 流均调用 `(disp=in_id, 262144, 1, 1, "")`
（cmaster.c），256k FFT 上限支撑最大跨度。销毁：`DestroyAnalyzer(int disp)`
（第 158 行）。

**配置**：`SetAnalyzer(disp, n_pixout, n_fft, typ, flp, sz, bf_sz, win_type, pi,
ovrlp, clp, fscLin, fscHin, n_pix, n_stch, calset, fmin, fmax, max_w)` —
一次定义 FFT 尺寸、窗型（Blackman/Harris/Kaiser…）、重叠、边沿裁剪、像素数、
拼接子跨度数、显示频段范围。C# 声明见 specHPSDR.cs，参数逐项注释齐全。

**喂数据**：三种方式——

- `Spectrum0(int run, int disp, int ss, int LO, double* pbuff)`
  （`analyzer.h` 第 180 行）：交织 double I/Q（Thetis 主用；run 用
  `_InterlockedAnd(&pcm->rcvr[rx].run_pan, 0xffffffff)` 原子读，可在不抽数据的
  同时停算以省 CPU）；
- `Spectrum(disp, ss, LO, pI, pQ)`：宽带 ADC 路径在 ReadUDPFrame 内直接调用；
- `OpenBuffer(int disp, int ss, int LO, void **Ipointer, void **Qpointer)`
  （`analyzer.h` 第 168 行）：拿到内部缓冲指针零拷贝写入。

**取像素**（`analyzer.c` 第 1244-1246 行，已核对源码签名）：

```c
PORT void GetPixels (int disp,
                     int pixout,
                     dOUTREAL *pix,   // if new pixel values avail, copies to pix and sets flag = 1
                     int *flag);      // else, returns 0 (try again later)
```

C# 侧以 `ref int flag` 封送。在绘制定时器中周期调用；pixout 允许多路输出
（主显示 + 自动检测/瀑布共用一次 FFT）。显示类参数函数：
`SetDisplayDetectorMode`（峰值/RMS/均值）、`SetDisplayAverageMode`
（log/power/vector/snap）、`SetDisplayNumAverage`、`SetDisplayAvBackmult`、
`SetDisplayNormOneHz`（1Hz 归一化）、`SetDisplaySampleRate`、`GetDisplayENB`
（等效噪声带宽，用于绝对标定）、`SnapSpectrum`（冻结一帧原始谱）。

**幅度标定**（`analyzer.h` 第 161-165 行，已核对源码签名）：

```c
void SetCalibration (int disp,
                     int set_num,                    // identifier for this calibration data set
                     int n_points,                   // number of calibration points in the set
                     double (*cal)[dMAX_M+1]);       // pointer to the calibration table
```

每行一条修正曲线，列 0 为频率轴；支持多套标定数据共存（set_num 区分）。

另有独立噪声消隐封装供 C# 直接持有缓冲使用：`create_nobEXT/xnobEXTF`、
`create_anbEXT/xanbEXTF` 及其 Set* 系列（specHPSDR.cs）。

## 10. C# P/Invoke 绑定层（Console/dsp.cs）

`unsafe class WDSP` 集中了约 300 个
`[DllImport("wdsp.dll", CallingConvention = CallingConvention.Cdecl)]` 声明。模式示例：

```csharp
[DllImport("wdsp.dll", EntryPoint = "fexchange0", CallingConvention = CallingConvention.Cdecl)]
public static extern void fexchange0(int channel, double* Cin, double* Cout, int* error);

[DllImport("wdsp.dll", EntryPoint = "SetRXAMode", CallingConvention = CallingConvention.Cdecl)]
public static extern void SetRXAMode(int channel, DSPMode mode);
```

- 指针参数一律用 `double*`/`float*` + `unsafe` 上下文；批量数组用 `IntPtr`
  （如 `SetCalibration`、`GetPixels`）。
- 模式枚举必须与 wdsp 内部顺序一致（`Console/enums.cs`）：
  `LSB=0, USB=1, DSB=2, CWL=3, CWU=4, FM=5, AM=6, DIGU=7, SPEC=8, DIGL=9,
  SAM=10, DRM=11, AM_LSB=12, AM_USB=13`。任何新增模式都要同步修改两侧，否则串扰。
- AGC 枚举 `AGCMode { FAST, MEDIUM, SLOW, USER, OFF }`；参数族：
  `SetRXAAGCMode/Fixed/Top/Attack/Decay/Hang/Slope/HangThreshold/HangLevel`
  及对应 Get*。
- 控制函数按功能分组：降噪陷波 `SetRXAANF*/SetRXAANR*`（taps/delay/gain/
  leakage/position）、EMNR 语音降噪 `SetRXAEMNR*`、均衡 `SetRXAEQRun/
  SetRXAGrphEQ10`、带宽滤波 `SetRXABandpassFreqs/Window`、静噪 `SetRXAAMSQ*`、
  SAM 解调深度 `SetRXAAMDFadeLevel`、FM 偏差 `SetRXAFMDeviation/SetTXAFMDeviation`、
  发射 ALC `SetTXAALCDecay/MaxGain`、压缩器 `SetTXACompressorRun/Gain`、
  osctrl `SetTXAosctrlRun`。
- 便捷转换器 `GetModeFromString / GetAGCFromString` 把 UI 文本映射到枚举。

## 11. 独立工具块速查

| 模块 | 文件 | 创建/销毁 | 处理函数 | Thetis 用途 |
|---|---|---|---|---|
| 脉冲噪声消隐 | `nb.c` | `create_nob/destroy_nob`(+EXT) | `xnob0/xnobEXTF` | 接收前置 NB |
| 模拟噪声消隐 | `nb.c` | `create_anb/destroy_anb`(+EXT) | `xanb0/xanbEXTF` | ANB |
| 自适应陷波 | `anf.c` | `create_anf/destroy_anf` | `xanf` | ANF |
| 自适应降噪 | `anr.c` | `create_anr/destroy_anr` | `xanr` | ANR |
| EMNR 降噪 | `emnr.c` | `create_emnr/destroy_emnr` | `xemnr` | 语音降噪 |
| 图形均衡 | `eq.c` | `create_eq10ext/...` | `xeqext` | RX/TX EQ |
| 压缩限幅 | `compress.c` | `create_compress/destroy_compress` | `xcompress` | TX 压缩 |
| CF 压缩 | `cfcomp.c` | `create_cfcomp/...` | `xcfcomp` | 峰均比控制 |
| EER 分解 | `eer.c` | `create_eer/destroy_eer` | `xeer` | EER 发射 |
| 交织/分路 | `ilv.c` | `create_ilv/destroy_ilv` | `xilv` | EER+IQ 合成输出 |
| 动态扩展 | `dexp.c` | `create_dexp/destroy_dexp` | `xdexp` | VOX 扩展 |
| TX 增益 | `txgain.c` | `create_txgain/...` | `xtxgain` | Penelope 增益表 |
| 静噪 | `amsq.c` | `create_amsq/...` | `xamsq` | AMSQ |
| 多样性合成 | `div.c` | `create_div/destroy_div` | `xdiv` | Rx0+Rxn 分集 |
| 过采样控制 | `osctrl.c` | `create_osctrl/...` | `xosctrl` | osCTRL |
| 可调陷波 | `notchvar.c` | `create_notchvar/...` | `xnotchvar` | Manual Notch |

共同范式：`create_*(&instance, params...)` 返回句柄 → 每块数据
`x*(instance, buffer, nsamples)` 原地处理 → `destroy_*(instance)` 释放。
EXT 变体（`*_EXT/_F`）允许调用方自己持有工作缓冲，便于 C# 直接 new 托管数组。

---

# 第三部分：端到端集成与时序

## 12. 完整启动/运行/关闭时序（协议 2 为例）

```
① nativeInitMetis(ip,local,port,ETH) → create_rnet() → CreateCMaster()
     （OpenChannel×N + XCreateAnalyzer×N + SendpOutbound(OutBound)）
② UpdateRadioProtocolSampleSize() 定 spp（rx=238, tx=240, mic=640, audio=64）
③ CmdGeneral(); CmdRx(); CmdTx();  然后 StartAudioNative():
     RadioProtocol==USB ? SendStartToMetis() : 直接 run=1; CmdHighPriority()
④ 稳态循环:
   网络读线程: 1025→状态字段; 1026→mic; 1027+→宽带Spectrum; 1035+→xrouter
        →Inbound→cm_main→xcmaster(Spectrum0 → fexchange0 → 混音) → 声卡
   声卡采集: Inbound(mic) → xcmaster(tx) → fexchange0 → xilv → OutBound
        → obb线程 → sendOutbound → WriteUDPFrame(1029)
   保活线程: 每500ms CmdGeneral()(若wdt≠0)
   UI 线程: 频率改变→写 prn->rx[n].frequency→CmdHighPriority();
            定时器→GetPixels→渲染 Panadapter
⑤ 停机: run=0 → CmdHighPriority()(Run=0) → StopAudio/SendStopToMetis(P1)
        → destroy_cmaster(CloseChannel/DestroyAnalyzer) → destroy_rnet()
```

## 13. 工程注意事项汇总

1. **数值约定**：全链路 double 交织 I/Q，满幅 ±1.0；网络 24bit 定点按 `1/2^31`
   缩放，发射量化用 `±8388607`（floor/ceil+0.5 四舍五入），16bit 音频用 `±32767`。
2. **线程纪律**：`fexchange0` 只在 cm_main 线程调用；参数 Set* 任意线程可调
   （内部临界区保护）；改尺寸/速率必须走带“关门-停车-冲刷-重启”流程的封装
   （如 `SetXcmInrate`、`SetCMRingOutsize`）。
3. **速率一致性**：网络侧 `SetDDCRate` 改变 DDC 输出率时，须同步 `SetAllRates`
   对应信道，三者（in/dsp/out）不成比例会导致重采样器滤波系数过期。
4. **协议差异陷阱**：协议 1 的 spp 随 nddc 动态变化（63/36/28），协议 2 固定
   238/240；`lr_audio_swap` 用于修复特定机型 L/R 声道反接；曾出现 1400 字节被
   1025 包读取覆盖 user_dig_in 的历史缺陷（MW0LGE_22b 修复）。
5. **PureSignal 特例**：PTT 且 puresignal_run 时 DDC0/1 频率被强制改写为 TX 频率
   （协议 2 见 1027 包 [9:16]，协议 1 见 case 2/3），以便反馈环采自身发射信号。
6. **bfo 语义**：Thetis 全信道 bfo=1，`fexchange0` 阻塞至输出就绪，天然限速；
   若改 bfo=0 需自行处理 `-2` 欠载错误码（输出已被置零）。
7. **许可合规**：wdsp 为 GPLv2（见 `Project Files/Source/wdsp` 头部版权声明及
   `GNU_GENERAL_PUBLIC_LICENSE.rtf`），衍生作品须开源。
8. **参考资料**：仓内 `Documentation/Radio/WDSP Guide, Rev 1.23.pdf` 提供每个
   导出函数的权威参数说明；各版本 Change Log PDF 记录了协议兼容性变更。

## 14. 附录 A：常用函数签名清单（已从源码核对）

```c
/* ---- channel.h ---- */
PORT void OpenChannel (int channel, int in_size, int dsp_size,
                       int input_samplerate, int dsp_rate, int output_samplerate,
                       int type, int state, double tdelayup, double tslewup,
                       double tdelaydown, double tslewdown, int bfo);   // line 62
PORT void CloseChannel (int channel);                                   // line 64
PORT void SetAllRates (int channel, int in_rate, int dsp_rate, int out_rate); // line 80
PORT int  SetChannelState (int channel, int state, int dmode);         // line 82

/* ---- iobuffs.h ---- */
void fexchange0 (int channel, double* in, double* out, int* error);     // line 90
void fexchange2 (int channel, INREAL *Iin, INREAL *Qin,
                 OUTREAL *Iout, OUTREAL *Qout, int* error);             // line 93

/* ---- analyzer.h / analyzer.c ---- */
void XCreateAnalyzer (int disp, int *success, int m_size,
                      int m_LO, int m_stitch, char *app_data_path);     // line 149
void DestroyAnalyzer (int disp);                                        // line 158
void SetCalibration  (int disp, int set_num, int n_points,
                      double (*cal)[dMAX_M+1]);                         // line 161
void OpenBuffer (int disp, int ss, int LO, void **Ipointer, void **Qpointer); // line 168
void Spectrum0 (int run, int disp, int ss, int LO, double* pbuff);      // line 180
void GetPixels (int disp, int pixout, dOUTREAL *pix, int *flag);        // analyzer.c line 1244

/* ---- RXA.h ---- */
extern __declspec (dllexport) void SetRXAMode (int channel, int mode);  // line 189

/* ---- comm.h ---- */
#define MAX_CHANNELS 32                                                 // line 96
```

## 15. 附录 B：术语表

| 术语 | 含义 |
|---|---|
| DDC | Digital Down Converter，FPGA 内数字下变频器，输出窄带 I&Q |
| DUC | Digital Up Converter，FPGA 内数字上变频器 |
| Metis | Hermes-I 时代的 USB↔UDP 桥接芯片/模块 |
| Mercury | 收发前端板（3 路 ADC/DAC，Hermes-II 起内置） |
| Penelope | 1kW 功放板，参与 EER/PureSignal |
| ALEX | Analog LEX 天线/滤波器切换板 |
| PureSignal | 基于反馈环的 DPD（数字预失真）校正系统 |
| EER | Envelope Elimination and Restoration 发射体制 |
| C&C | Command & Control 命令与控制通道 |
| Portmap | 1024 号包，声明各数据流的 UDP 端口映射 |
| LOS | Loss of Sync/Signal，网络失锁状态 |
| WDT | Watchdog Timer，电台看门狗 |
| panadapter | 频谱瀑布显示（Panoramic Adapter） |
| spp | samples per packet，每包样点数 |
| bfo | block for output，fexchange 是否阻塞等输出 |
| slew | T/R 切换时的振幅渐变斜坡 |
