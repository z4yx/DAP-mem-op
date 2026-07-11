# SVF Generator for ARM Cortex-R52

基于 ARM IHI0031H (ARM Debug Interface Architecture Specification) 的 SVF JTAG 操作序列生成器。

## 安装依赖

无第三方依赖，仅使用 Python 3 标准库。

## 用法

```bash
# 基本用法：将 firmware.bin 下载到地址 0x80000000
python svf_gen.py firmware.bin --addr 0x80000000 -o download.svf

# 指定 MEM-AP 选择索引
python svf_gen.py firmware.bin --addr 0x80000000 --ap 2 -o download.svf

# 16-bit 访问宽度 + 读回校验
python svf_gen.py firmware.bin --addr 0x20000000 --width 16 --verify -o download.svf

# 输出到标准输出
python svf_gen.py firmware.bin --addr 0x80000000

# 详细模式
python svf_gen.py firmware.bin --addr 0x80000000 -o download.svf -V

# JTAG 菊花链支持
python svf_gen.py firmware.bin --addr 0x80000000 --chain chain.json -o download.svf

# ADIv5 协议（Cortex-A9, Cortex-M4 等）
python svf_gen.py firmware.bin --addr 0x00100000 --adi-version 5 --verify -o download.svf
```

## 参数说明

| 参数 | 说明 |
|------|------|
| `binfile` | 要下载的 .bin 文件路径 |
| `--addr, -a` | 目标内存基地址（十六进制，如 0x80000000） |
| `--output, -o` | 输出 SVF 文件路径（默认：stdout） |
| `--ap, -p` | MEM-AP 选择索引 APSEL（默认：0） |
| `--width, -w` | 内存访问宽度：8/16/32 bit（默认：32） |
| `--adi-version, -A` | ADI 协议版本：5 或 6（默认：6） |
| `--verify, -v` | 在 SVF 中包含读回校验序列 |
| `--chain, -c` | JTAG 菊花链配置文件 (JSON) |
| `--idcode` | 期望的 DP IDCODE 值 |
| `--verbose, -V` | 输出详细进度信息 |

## JTAG 菊花链支持

当调试接口上有多个 TAP (Test Access Port) 串联时，需要将无关 TAP 置于 BYPASS 模式，只将 IR/DR 指令发送给目标 DAP。

### 菊花链配置文件格式 (JSON)

```json
{
    "taps": [
        {"name": "ICE",   "irlen": 4},
        {"name": "DAP",   "irlen": 4},
        {"name": "FPGA",  "irlen": 6}
    ],
    "target": "DAP"
}
```

| 字段 | 说明 |
|------|------|
| `taps` | TAP 列表，从前到后对应 TDI→TDO 方向 |
| `taps[].name` | TAP 名称（任意标识字符串） |
| `taps[].irlen` | 该 TAP 的 IR 寄存器长度 (bits) |
| `target` | 目标 DAP 的 TAP 名称（也可用 `target_index` 指定索引） |

### 工作原理

```
TDI → [ICE] → [DAP] → [FPGA] → TDO
      4-bit     4-bit    6-bit
      BYPASS    TARGET   BYPASS
```

1. **SIR**：无关 TAP 的 IR 设置为全 1（BYPASS 指令）
   - 完整 IR = `{FPGA_BYPASS[5:0], DAP_IR[3:0], ICE_BYPASS[3:0]}` = 14 bits

2. **SDR**：无关 TAP 贡献 1 bit BYPASS DR（值为 0）
   - 完整 DR = `{FPGA_BYPASS[1], DAP_DR[34:0], ICE_BYPASS[1]}` = 37 bits

### 示例

```bash
# 使用示例配置文件
python svf_gen.py firmware.bin --addr 0x80000000 --chain chain_example.json -o download.svf
```

## 生成的 SVF 操作序列

```
Phase 1: JTAG Reset → IDLE → 读取 IDCODE
Phase 2: 上电调试域（DP.CTRL/STAT: CDBGPWRUPREQ + CSYSPWRUPREQ）
Phase 3: 选择 MEM-AP（DP.SELECT: APSEL + APBANKSEL）
Phase 4: 配置 MEM-AP（AP.CSW: 32-bit, auto-increment, DeviceEn）
Phase 5: 逐字写入内存（auto-increment 模式）
         ├─ AP.TAR ← 基地址（仅一次）
         └─ 循环: AP.DRW ← 数据（TAR 自动递增）
Phase 6: (可选) 读回校验
Phase 7: JTAG Reset 关闭
```

## JTAG-DP 协议细节

- IR 长度：4 bits
- DPACC: IR=0xA, DR=35 bits `{DATA[31:0], A[3:2], RnW}`
- APACC: IR=0xB, DR=35 bits `{DATA[31:0], A[3:2], RnW}`
- IDCODE: IR=0xE, DR=32 bits
- 读操作采用流水线：数据在下一次 DPACC/APACC 事务的 TDO 上返回
- 使用 DP.RDBUFF 读取上一次 AP 读操作的结果

## 参考

- ARM IHI0031H — ARM Debug Interface Architecture Specification (ADIv6)
- ARM Cortex-R52 Technical Reference Manual
- IEEE 1149.1 — Standard Test Access Port and Boundary-Scan Architecture

## 芯片实例

### Xilinx Zynq-7020 (XC7Z020)

Zynq-7020 内部 JTAG 链包含两个 TAP：

```
     ┌───────────────── Zynq-7020 ─────────────────┐
     │                                              │
TDI ─┼─→ [ARM_DAP] ──→ [PL_TAP] ──→ TDO            │
     │   4-bit IR       6-bit IR                    │
     │   Cortex-A9      Artix-7 FPGA                │
     │   ID:0x4BA00477  ID:0x03722093               │
     └──────────────────────────────────────────────┘
```

| TAP | IR 长度 | 典型 IDCODE | 说明 |
|-----|---------|-------------|------|
| ARM_DAP | 4 bits | 0x4BA00477 | Cortex-A9 MPCore 双核调试接口 |
| PL_TAP | 6 bits | 0x03722093 | 7-Series FPGA 逻辑 (Artix-7) |

**链顺序**：取决于 `DAP_PL_MODE` 引脚/SLCR 配置：
- **DAP 在首位**（大多数开发板默认）→ 使用 `zynq7020_dap_first.json`
- **PL 在首位** → 使用 `zynq7020_pl_first.json`

```bash
# Zynq-7020 典型用法（DAP 在链首）
python svf_gen.py app.bin --addr 0x00100000 \
    --chain zynq7020_dap_first.json -o download.svf
```

> **如何判断链顺序**：用 JTAG 工具扫描链，观察 TDO 上 IDCODE 出现的先后。
> 先出现的 IDCODE 对应链首 TAP（离 TDI 最近）。
> 或在 Xilinx XSCT 中执行 `jtag targets` 查看顺序。

## ADIv5 与 ADIv6 兼容性

APACC 域段定义（DR 格式、CSW、TAR、DRW、ACK OK=0b010）在两个版本中完全一致。
通过 `--adi-version` 参数适配的唯一差异：

| 特性 | ADIv5 | ADIv6 |
|------|-------|-------|
| CSW bit 31 | 保留（不设置） | DBGSWENABLE（设置） |
| 典型 CPU | Cortex-A9, Cortex-M4 | Cortex-R52, Cortex-M55 |

```bash
# ADIv5 示例（Zynq-7020 的 Cortex-A9）
python svf_gen.py app.bin --addr 0x00100000 \
    --chain zynq7020_dap_first.json \
    --adi-version 5 --verify -o download.svf

# ADIv6 示例（Cortex-R52，默认）
python svf_gen.py app.bin --addr 0x80000000 --verify -o download.svf
```
