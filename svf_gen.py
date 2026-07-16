#!/usr/bin/env python3
"""
SVF (Serial Vector Format) Generator for ARM Cortex-R52 Memory Access
=========================================================================
Based on ARM IHI0031H — ARM Debug Interface Architecture Specification (ADIv6)

This tool generates SVF-format JTAG operation sequences for two use cases:

1.  **Binary download** — download a .bin binary file into ARM Cortex-R52
    processor memory via the CoreSight Debug Access Port (DAP).
2.  **Command file** — execute arbitrary memory read/write commands defined
    in a text file, with optional per-read TDO verification.

Architecture Overview
---------------------
    Host (SVF Player) ──JTAG──> JTAG-DP ──> MEM-AP ──> Memory Bus (AHB/AXI)

    JTAG-DP IR (4-bit):
        0xE = IDCODE      — Read device identification
        0xA = DPACC       — Debug Port register access (35-bit DR)
        0xB = APACC       — Access Port register access  (35-bit DR)
        0x8 = ABORT       — Abort operation

    DPACC / APACC DR Scan Chain (35 bits):
        Bits [34:3] = DATA[31:0]   — 32-bit read/write data
        Bits  [2:1] = A[3:2]       — Register address within bank
        Bit    [0]  = RnW          — 1=Read, 0=Write

    Pipelined Reads:
        AP/DP read data is returned in the *next* DPACC/APACC transaction.
        Use a dummy DP-RDBUFF read to retrieve the last read result.

Memory Write Sequence (binary mode)
-----------------------------------
    1. JTAG Reset → Run-Test/Idle
    2. Read IDCODE (verification)
    3. Power-up debug domain (DP.CTRL/STAT)
    4. Select MEM-AP (DP.SELECT)
    5. Configure MEM-AP (AP.CSW: 32-bit, auto-increment)
    6. For each word: write AP.TAR, then write AP.DRW
    7. (Optional) Read-back verification
    8. JTAG Reset

Memory Command Sequence (command-file mode)
--------------------------------------------
    1–4. Same setup as binary mode, but CSW uses ADDRINC_OFF.
    5. For each command:
       - Write: set AP.TAR, write AP.DRW
       - Read:  set AP.TAR, read AP.DRW, read DP.RDBUFF
         (with optional TDO verification if Y flag is set)
    6. JTAG Reset

Command File Format
-------------------
    <addr_hex>  <data_hex>  <W|R>  [Y]
    0  <count>  TCK
    # This is a comment line
    0x80000000  DEADBEEF  W
    0x80000004  12345678  R  Y
    0x80001000  AABBCCDD  R
    0  100  TCK

Usage
-----
    # Binary download
    python svf_gen.py firmware.bin --addr 0x80000000 -o download.svf

    # Command file
    python svf_gen.py --cmds ops.txt -o ops.svf
    python svf_gen.py --cmds ops.txt --width 16 -o ops.svf
"""

import argparse
import json
import os
import struct
import sys
import textwrap
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


# =============================================================================
# Memory Command (for text-based command files)
# =============================================================================


class Op(Enum):
    """Memory command operation type."""

    R = 0  # read
    W = 1  # write
    T = 2  # TCK wait


@dataclass
class MemCmd:
    """A single command parsed from a text command file.

    Fields:
        addr:   Target memory address (32-bit).  Ignored for wait commands.
        data:   Data value to write, expected data on read, or TCK cycle
                count for wait commands.
        op:     Operation type — ``Op.R`` (read), ``Op.W`` (write), or
                ``Op.T`` (TCK wait).
        verify: For reads: whether to compare TDO against *data* (Y/N).
                Only meaningful when *op* is ``Op.R``.
    """

    addr: int
    data: int
    op: Op
    verify: bool = False


def parse_cmd_file(path: str, data_width: int = 32) -> List[MemCmd]:
    """Parse a text file of memory access commands.

    Each non-empty, non-comment line must contain at least three
    whitespace-separated fields::

        <addr_hex>  <data_hex>  <W|R>  [Y]
        0  <count>  TCK

    - *addr_hex*   — 32-bit address in hexadecimal (e.g. ``0x80000000``).
    - *data_hex*   — data word in hex; width is clipped to *data_width* bits.
    - *W* or *R*   — write or read operation.
    - *Y*          — (optional, read-only) verify that TDO matches *data_hex*.

    A special delay/wait form is also supported::

        0  <count>  TCK
        0  100  TCK

    This inserts a RUNTEST wait for *count* TCK cycles; no JTAG scan
    operations are emitted.

    Lines whose first non-whitespace character is ``#`` are treated as
    comments and skipped.  Blank lines are also ignored.
    """
    mask = (1 << data_width) - 1
    cmds: List[MemCmd] = []

    with open(path, "r") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line or line.startswith("#") or line.startswith("//"):
                continue

            parts = line.split()
            if len(parts) < 3:
                raise ValueError(
                    f"{path}:{lineno}: expected at least 3 fields "
                    f"(addr data R|W), got {len(parts)}: {line!r}"
                )

            addr = int(parts[0], 16)
            rw = parts[2].upper()

            # --- TCK wait: "0 <count> TCK" ---
            if addr == 0 and rw == "TCK":
                tck = int(parts[1], 0)
                if tck <= 0:
                    raise ValueError(
                        f"{path}:{lineno}: TCK count must be positive, "
                        f"got {tck}: {line!r}"
                    )
                cmds.append(MemCmd(addr=0, data=tck, op=Op.T))
                continue

            data = int(parts[1], 16) & mask

            ops = {"R": Op.R, "W": Op.W}
            if rw not in ops:
                raise ValueError(
                    f"{path}:{lineno}: third field must be R, W, or TCK, "
                    f"got {rw!r}: {line!r}"
                )

            op = ops[rw]
            verify = False
            if op == Op.R and len(parts) >= 4:
                v = parts[3].upper()
                if v == "Y":
                    verify = True
                elif v != "N":
                    raise ValueError(
                        f"{path}:{lineno}: fourth field must be Y or N "
                        f"(or omitted), got {v!r}: {line!r}"
                    )

            cmds.append(MemCmd(addr=addr, data=data, op=op, verify=verify))

    return cmds


# =============================================================================
# JTAG Daisy-Chain Configuration
# =============================================================================


@dataclass
class JtagChainConfig:
    """Describes a JTAG daisy-chain topology for multi-TAP systems.

    In a daisy chain, TAPs are connected in series:
        TDI → [TAP_0] → [TAP_1] → ... → [TAP_n-1] → TDO

    Only the *target* TAP receives real IR/DR data; all others are placed
    in BYPASS mode (IR = all-1s, 1-bit DR = 0).

    Configuration file format (JSON)::

        {
            "taps": [
                {"name": "ICE",   "irlen": 4},
                {"name": "DAP",   "irlen": 4},
                {"name": "FPGA",  "irlen": 6}
            ],
            "target": "DAP"
        }

    *target* may be a name (string) or *target_index* may be a zero-based
    integer.  *target* takes precedence when both are present.
    """

    taps: List[dict] = field(default_factory=list)
    target_index: int = 0

    # ------------------------------------------------------------------
    # Factory
    # ------------------------------------------------------------------

    @classmethod
    def from_json_file(cls, path: str) -> "JtagChainConfig":
        """Load chain configuration from a JSON file."""
        with open(path, "r") as fh:
            cfg = json.load(fh)

        taps = cfg["taps"]
        if not taps:
            raise ValueError("Chain config: 'taps' list must not be empty")

        # Resolve target
        target_index = cls._resolve_target(cfg, taps)
        if target_index < 0 or target_index >= len(taps):
            raise ValueError(
                f"Chain config: target_index {target_index} out of range "
                f"[0, {len(taps) - 1}]"
            )

        return cls(taps=taps, target_index=target_index)

    @classmethod
    def single_tap(cls) -> "JtagChainConfig":
        """Convenience: a chain consisting of a single DAP TAP (no daisy chain)."""
        return cls(taps=[{"name": "DAP", "irlen": 4}], target_index=0)

    @staticmethod
    def _resolve_target(cfg: dict, taps: list) -> int:
        """Determine target TAP index from 'target' (name) or 'target_index'."""
        if "target" in cfg:
            name = cfg["target"]
            for i, tap in enumerate(taps):
                if tap.get("name") == name:
                    return i
            raise ValueError(f"Chain config: target '{name}' not found in taps list")
        if "target_index" in cfg:
            return int(cfg["target_index"])
        # Default: first tap
        return 0

    # ------------------------------------------------------------------
    # Chain-aware SIR / SDR composition
    # ------------------------------------------------------------------

    def compose_sir(self, target_ir: int, target_ir_bits: int) -> tuple:
        """Given the target TAP's IR value and bit-width, return the
        full daisy-chain SIR as ``(total_bits, tdi_value)``.

        Non-target TAPs receive BYPASS IR (all 1s) of their respective
        IR length.
        """
        n = len(self.taps)
        if n <= 1:
            return (target_ir_bits, target_ir)

        t = self.target_index

        # IR lengths before / after the target
        irlen_before = sum(tap["irlen"] for tap in self.taps[:t])
        irlen_after = sum(tap["irlen"] for tap in self.taps[t + 1 :])

        total_bits = irlen_before + target_ir_bits + irlen_after

        # BYPASS IR = all-1s for the given length
        bypass_before = (1 << irlen_before) - 1 if irlen_before else 0
        bypass_after = (1 << irlen_after) - 1 if irlen_after else 0

        # TDI layout (LSB shifted first in JTAG):
        #   [bypass_after] [target_ir] [bypass_before]
        tdi = (
            (bypass_after << (irlen_before + target_ir_bits))
            | (target_ir << irlen_before)
            | bypass_before
        )

        return (total_bits, tdi)

    def compose_sdr(self, target_dr: int, target_dr_bits: int) -> tuple:
        """Given the target TAP's DR value and bit-width, return the
        full daisy-chain SDR as ``(total_bits, tdi_value)``.

        Non-target TAPs in BYPASS contribute 1 bit each (value = 0).
        """
        n = len(self.taps)
        if n <= 1:
            return (target_dr_bits, target_dr)

        t = self.target_index

        # Each non-target TAP in BYPASS = 1 DR bit
        dr_pre_bits = t  # TAPs before target
        dr_post_bits = n - t - 1  # TAPs after target

        total_bits = dr_pre_bits + target_dr_bits + dr_post_bits

        # BYPASS DR = 0
        tdi = target_dr << dr_pre_bits

        return (total_bits, tdi)

    def compose_tdo(self, target_tdo: int, target_bits: int, target_mask: int) -> tuple:
        """Given the expected target TAP TDO value and bit-width, return the
        full daisy-chain TDO as ``(full_tdo, full_mask)``.

        Only the target TAP's data bits are verified.  Non-target TAPs in
        BYPASS may produce arbitrary TDO values, so their bits are excluded
        from the mask (mask = 0 at those positions).
        """
        n = len(self.taps)
        if n <= 1:
            return (target_tdo, target_mask)

        t = self.target_index
        dr_pre_bits = t

        # Position target TDO after preceding BYPASS bits
        full_tdo = target_tdo << dr_pre_bits

        # Only check the target TAP's bits — ignore bypass TAPs
        full_mask = target_mask << dr_pre_bits

        return (full_tdo, full_mask)

    @property
    def is_daisy_chain(self) -> bool:
        """True when multiple TAPs are configured."""
        return len(self.taps) > 1

    @property
    def target_name(self) -> str:
        """Human-readable name of the target TAP."""
        return self.taps[self.target_index].get("name", f"TAP#{self.target_index}")

    @property
    def summary(self) -> str:
        """One-line chain description for SVF comments."""
        if not self.is_daisy_chain:
            return "Single TAP (no daisy chain)"
        names = [tap.get("name", "?") for tap in self.taps]
        markers = [
            f"[{n}]" if i == self.target_index else f" {n} "
            for i, n in enumerate(names)
        ]
        return f"Daisy chain: {' → '.join(markers)}  (target marked [...])"


# =============================================================================
# SVF Command Generator
# =============================================================================


class SvfGenerator:
    """Generates well-formatted SVF (Serial Vector Format) commands.

    SVF is an industry-standard ASCII format for describing JTAG operations,
    defined in the IEEE 1149.1 boundary-scan standard.  This class emits
    syntactically correct SVF that can be played back by OpenOCD, UrJTAG,
    Xilinx tools, and other JTAG utilities.
    """

    # ---- JTAG Instruction Register codes (4-bit for JTAG-DP) ---------------
    IR_IDCODE = 0xE  # IDCODE register access
    IR_DPACC = 0xA  # Debug Port access
    IR_APACC = 0xB  # Access Port access
    IR_ABORT = 0x8  # Abort register

    # ---- DP register addresses (A[3:2] encoding within DPACC DR) ----------
    DP_IDCODE_ABORT = 0x0  # Read: IDCODE;  Write: ABORT
    DP_CTRL_STAT = 0x1  # Control / Status
    DP_SELECT = 0x2  # AP Select
    DP_RDBUFF = 0x3  # Read Buffer (pipeline flush)

    # ---- AP register addresses (A[3:2] encoding, bank 0) ------------------
    AP_CSW = 0x0  # Control / Status Word
    AP_LTAR = 0x1  # Transfer Address Register (lower 32 bits)
    AP_HTAR = 0x2  # Transfer Address Register (upper 32 bits, 64-bit mode)
    AP_DRW = 0x3  # Data Read / Write

    # ---- DP.CTRL/STAT bit definitions ------------------------------------
    CSYSPWRUPREQ = 1 << 30  # System power-up request
    CDBGPWRUPREQ = 1 << 28  # Debug power-up request
    CSYSPWRUPACK = 1 << 31  # System power-up acknowledge (read-only)
    CDBGPWRUPACK = 1 << 29  # Debug power-up acknowledge  (read-only)
    STICKYERR = 1 << 5  # Sticky error flag
    TRNCNT_MASK = 0xFFF  # Turnaround counter (bits 11:0)
    TRNCNT_VAL = 0x200  # Recommended: 512 TCK cycles

    # ---- AP.CSW bit definitions -------------------------------------------
    CSW_SIZE_8BIT = 0x0
    CSW_SIZE_16BIT = 0x1
    CSW_SIZE_32BIT = 0x2
    CSW_SIZE_64BIT = 0x3
    CSW_ADDRINC_OFF = 0 << 4  # No auto-increment
    CSW_ADDRINC_SINGLE = 1 << 4  # Single increment after access
    CSW_ADDRINC_PACKED = 2 << 4  # Packed transfers
    CSW_DEVICEEN = 1 << 6  # Device enabled
    CSW_DBGSWENABLE = 1 << 31  # Debug software enable (ADIv6)
    CSW_HPROT_DATA = 3 << 24  # AHB HPROT: data access, non-cacheable

    def __init__(
        self, output_file: Optional[str] = None, chain: Optional[JtagChainConfig] = None
    ):
        """Open output stream (file or stdout).

        Args:
            output_file: Path to SVF output file (None = stdout).
            chain: Optional JTAG daisy-chain configuration.  When provided,
                   all SIR/SDR operations are automatically extended to
                   account for non-target TAPs in BYPASS.
        """
        self._f = open(output_file, "w") if output_file else sys.stdout
        self._indent = 0
        self._chain = chain or JtagChainConfig.single_tap()
        self._emit_header()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def comment(self, text: str):
        """Emit a human-readable comment line."""
        for line in text.splitlines():
            self._emit(f"// {line}")

    def blank(self):
        """Emit a blank line for readability."""
        self._emit("")

    def runtest(self, tck_cycles: int):
        """Run-Test/Idle for *tck_cycles* TCK clock periods."""
        if tck_cycles > 0:
            self._emit(f"RUNTEST {tck_cycles} TCK;")

    def jtag_idcode(self):
        """Set IR = IDCODE, then scan 32-bit DR to read device ID."""
        self._sir(4, self.IR_IDCODE)
        self._sdr(32, 0x00000000, comment="Read IDCODE — TDO receives device ID")

    def jtag_dpacc(self):
        """Set IR = DPACC (Debug Port access)."""
        self._sir(4, self.IR_DPACC)

    def jtag_apacc(self):
        """Set IR = APACC (Access Port access)."""
        self._sir(4, self.IR_APACC)

    def dp_write(self, addr: int, data: int, comment: str = ""):
        """Write *data* to DP register at *addr* (A[3:2] = 0-3)."""
        dr = self._make_dp_ap_dr(addr, rnw=0, data=data)
        self.jtag_dpacc()
        self._sdr(35, dr, comment=comment)

    def dp_read(
        self,
        addr: int,
        comment: str = "",
        tdo_expected: Optional[int] = None,
        tdo_mask: Optional[int] = None,
    ):
        """Initiate read from DP register at *addr*.

        The actual read data appears on TDO during the **next** DPACC/APACC
        transaction (pipelined).  Follow with a DP-RDBUFF read to retrieve it.

        Args:
            addr: DP register address (A[3:2] = 0-3).
            comment: Human-readable annotation.
            tdo_expected: Expected 35-bit TDO value for verification (data portion).
            tdo_mask: Bit mask for TDO comparison (1 = check, 0 = ignore).
        """
        dr = self._make_dp_ap_dr(addr, rnw=1, data=0)
        self.jtag_dpacc()
        self._sdr(35, dr, comment=comment, tdo=tdo_expected, tdo_mask=tdo_mask)

    def ap_write(self, addr: int, data: int, comment: str = ""):
        """Write *data* to AP register at *addr* (A[3:2] = 0-3)."""
        dr = self._make_dp_ap_dr(addr, rnw=0, data=data)
        self.jtag_apacc()
        self._sdr(35, dr, comment=comment)

    def ap_read(self, addr: int, comment: str = ""):
        """Initiate read from AP register at *addr*.

        As with DP reads, the data appears in the next DPACC/APACC scan.
        """
        dr = self._make_dp_ap_dr(addr, rnw=1, data=0)
        self.jtag_apacc()
        self._sdr(35, dr, comment=comment)

    def close(self):
        """Flush and close the output stream."""
        if self._f is not sys.stdout:
            self._f.close()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _make_dp_ap_dr(addr: int, rnw: int, data: int) -> int:
        """Build the 35-bit DPACC/APACC DR scan value.

        Layout (LSB shifted first on JTAG TDI):
            Bit   [0]   = RnW        (1=Read, 0=Write)
            Bits [2:1]  = A[3:2]     (register select, 0-3)
            Bits [34:3] = DATA[31:0] (read/write payload)
        """
        return ((data & 0xFFFFFFFF) << 3) | ((addr & 0x3) << 1) | (rnw & 0x1)

    @staticmethod
    def _hex(value: int, bits: int) -> str:
        """Format *value* as zero-padded hex, wide enough for *bits*.

        Examples:
            _hex(0xA, 4)   -> "A"
            _hex(0xA, 8)   -> "0A"
            _hex(0x91A2B3C0, 35) -> "0091A2B3C0"  (9 nibbles for 35 bits)
        """
        nibbles = (bits + 3) // 4  # round up to full nibbles
        mask = (1 << bits) - 1
        return format(value & mask, f"0{nibbles}X")

    @staticmethod
    def _mask(bits: int) -> str:
        """All-ones hex mask for *bits* bits."""
        return SvfGenerator._hex((1 << bits) - 1, bits)

    def _emit(self, line: str):
        """Write a line with optional indentation."""
        prefix = "    " * self._indent
        self._f.write(f"{prefix}{line}\n")

    def _emit_header(self):
        """Emit the SVF preamble that every compliant SVF file requires."""
        chain_desc = self._chain.summary
        self._f.write(
            textwrap.dedent(f"""\
            // ============================================================================
            //  SVF — Serial Vector Format
            //  Target:  ARM Cortex-R52 (ARM IHI0031H / ADIv6)
            //  Chain:   {chain_desc}
            //  Generated by: svf_gen.py
            // ============================================================================
            TRST OFF;
            ENDIR IDLE;
            ENDDR IDLE;
            STATE RESET IDLE;
        """).lstrip()
        )

    def _sir(self, target_bits: int, target_tdi: int):
        """Scan Instruction Register — set target TAP IR to *target_tdi*.

        When a daisy chain is configured, non-target TAPs receive BYPASS IR
        (all-1s) and the SIR length is extended accordingly.
        """
        bits, tdi = self._chain.compose_sir(target_tdi, target_bits)
        self._emit(
            f"SIR {bits} TDI ({self._hex(tdi, bits)}) SMASK ({self._mask(bits)});"
        )

    def _sdr(
        self,
        target_bits: int,
        target_tdi: int,
        comment: str = "",
        tdo: Optional[int] = None,
        tdo_mask: Optional[int] = None,
    ):
        """Scan Data Register — shift *target_tdi* into the target TAP DR.

        When a daisy chain is configured, non-target TAPs in BYPASS add
        1 bit each (value 0) and the SDR length is extended accordingly.

        Args:
            target_bits: Target TAP DR width.
            target_tdi: TDI value for the target TAP.
            comment: Human-readable annotation.
            tdo: Expected TDO value for verification (full chain).
            tdo_mask: TDO comparison mask (1=check, 0=ignore).
        """
        bits, tdi = self._chain.compose_sdr(target_tdi, target_bits)
        parts = f"SDR {bits} TDI ({self._hex(tdi, bits)}) SMASK ({self._mask(bits)})"
        if tdo is not None and tdo_mask is not None:
            full_tdo, full_mask = self._chain.compose_tdo(tdo, target_bits, tdo_mask)
            parts += (
                f" TDO ({self._hex(full_tdo, bits)}) "
                f"MASK ({self._hex(full_mask, bits)})"
            )
        tail = f"  // {comment}" if comment else ""
        self._emit(f"{parts};{tail}")


# =============================================================================
# High-Level Sequence Generator
# =============================================================================


class CortexR52SvfBuilder:
    """Orchestrates the full SVF sequence for Cortex-R52 memory download.

    Supports both ADIv5 (e.g. Cortex-A9, Cortex-M4) and ADIv6 (Cortex-R52)
    DAP protocols.  The APACC field definitions (DR format, CSW, TAR, DRW)
    and ACK OK encoding (0b010) are identical between versions.  The only
    difference is CSW bit 31 (DBGSWENABLE), which exists only in ADIv6.
    """

    # ACK encoding: OK = 0b010 in both ADIv5 and ADIv6.
    ACK_OK = 0x2  # OK response
    ACK_MASK = 0x7  # verify all 3 ACK bits

    # Known DP IDCODE values for common implementations
    # (Actual value depends on silicon revision; override with --idcode)
    CORTEX_R52_BASE_CID = 0x6BA02477  # Cortex-R52 r1p0

    def __init__(
        self,
        bin_path: Optional[str] = None,
        base_addr: Optional[int] = None,
        output_path: Optional[str] = None,
        ap_sel: int = 0,
        data_width: int = 32,
        dp_idcode: Optional[int] = None,
        verify: bool = False,
        verbose: bool = False,
        chain_config_path: Optional[str] = None,
        adi_version: int = 6,
        cmd_list: Optional[List[MemCmd]] = None,
        addr64: bool = False,
    ):
        self.bin_path = bin_path
        self.base_addr = base_addr
        self.output_path = output_path
        self.ap_sel = ap_sel
        self.data_width = data_width
        self.dp_idcode = dp_idcode
        self.verify = verify
        self.verbose = verbose
        self.chain_config_path = chain_config_path
        self.adi_version = adi_version
        self.cmd_list = cmd_list
        self.addr64 = addr64
        # TAR de-duplication: track last-written {lo, hi} to skip redundant writes
        self._last_tar_lo: Optional[int] = None
        self._last_tar_hi: Optional[int] = None

        if data_width not in (8, 16, 32):
            raise ValueError("data_width must be 8, 16, or 32")
        if adi_version not in (5, 6):
            raise ValueError("adi_version must be 5 or 6")

        # Exactly one source must be provided
        has_bin = self.bin_path is not None
        has_cmd = self.cmd_list is not None
        if has_bin and has_cmd:
            raise ValueError("Provide either a .bin file or --cmds, not both.")
        if not has_bin and not has_cmd:
            raise ValueError("Provide either a .bin file or --cmds.")
        if has_bin and self.base_addr is None:
            raise ValueError("--addr is required for .bin file mode.")

    # ------------------------------------------------------------------
    # CSW configuration
    # ------------------------------------------------------------------

    def _csw_size_field(self) -> int:
        """Map data width to CSW Size[2:0] field."""
        return {8: 0, 16: 1, 32: 2}[self.data_width]

    def _csw_value(self, auto_increment: bool = True) -> int:
        """Build the MEM-AP CSW register value.

        ADIv6 adds DBGSWENABLE (bit 31); ADIv5 reserves this bit so we
        omit it for v5 targets.

        Args:
            auto_increment: If True, use ADDRINC_SINGLE (suitable for
                sequential block writes).  If False, use ADDRINC_OFF
                (suitable for random-access command files where TAR is
                set explicitly before each access).
        """
        addrinc = (
            SvfGenerator.CSW_ADDRINC_SINGLE
            if auto_increment
            else SvfGenerator.CSW_ADDRINC_OFF
        )
        csw = (
            SvfGenerator.CSW_DEVICEEN
            | addrinc
            | SvfGenerator.CSW_HPROT_DATA
            | (self._csw_size_field())
        )
        if self.adi_version >= 6:
            csw |= SvfGenerator.CSW_DBGSWENABLE
        return csw

    # ------------------------------------------------------------------
    # TAR address writing (with 64-bit support and de-duplication)
    # ------------------------------------------------------------------

    def _write_tar(self, g: SvfGenerator, addr: int, label: str = ""):
        """Write AP.TAR (and AP.TAR2 for 64-bit) — skip if unchanged.

        Splits *addr* into upper/lower 32-bit halves.  Only emits APACC
        writes for halves that differ from the last written value.

        Args:
            g:     SvfGenerator instance.
            addr:  Full 64-bit (or 32-bit) target address.
            label: Optional progress label (e.g. ``"[3/16]"``).
        """
        label_prefix = f"{label} " if label else ""
        lo = addr & 0xFFFFFFFF
        hi = (addr >> 32) & 0xFFFFFFFF

        if not self.addr64 and hi != 0:
            raise ValueError("Must use --addr64 for addresses above 32 bits")

        if self.addr64 and hi != self._last_tar_hi:
            g.ap_write(
                SvfGenerator.AP_HTAR, hi, f"{label_prefix}TAR2 ← 0x{hi:08X}  (upper)"
            )
            self._last_tar_hi = hi

        if lo != self._last_tar_lo:
            comment = f"{label_prefix}TAR ← 0x{lo:08X}"
            if self.addr64:
                comment += f"  (lower, full=0x{addr:016X})"
            g.ap_write(SvfGenerator.AP_LTAR, lo, comment)
            self._last_tar_lo = lo

    # ------------------------------------------------------------------
    # Shared setup (JTAG reset, power-up, AP selection, CSW)
    # ------------------------------------------------------------------

    def _emit_setup_phases(self, g: SvfGenerator, csw: int, csw_desc: str):
        """Emit SVF phases 1–4 (reset, power-up, AP select, CSW config)."""
        # Phase 1 — Reset & IDCODE
        g.comment("=" * 70)
        g.comment("PHASE 1: JTAG Reset & Device Identification")
        g.comment("=" * 70)
        g.blank()
        g.runtest(10)
        g.jtag_idcode()
        g.runtest(10)

        # Phase 2 — Power-Up
        g.comment("=" * 70)
        g.comment("PHASE 2: Power-Up the Debug & System Domains")
        g.comment("=" * 70)
        g.blank()

        ctrl_stat_write = (
            SvfGenerator.CDBGPWRUPREQ
            | SvfGenerator.CSYSPWRUPREQ
            | SvfGenerator.TRNCNT_VAL
        )
        g.dp_write(
            SvfGenerator.DP_CTRL_STAT,
            ctrl_stat_write,
            "Request CDBGPWRUP + CSYSPWRUP, TRNCNT=512",
        )
        g.runtest(50)

        g.dp_read(
            SvfGenerator.DP_CTRL_STAT, "Read CTRL/STAT (result in next transaction)"
        )
        g.dp_read(
            SvfGenerator.DP_RDBUFF,
            "Read RDBUFF — TDO = CTRL/STAT value; check ACK bits",
        )
        g.runtest(10)

        # Phase 3 — Select MEM-AP
        g.comment("=" * 70)
        g.comment("PHASE 3: Select MEM-AP via DP.SELECT")
        g.comment("=" * 70)
        g.blank()

        select_val = (self.ap_sel & 0xFF) << 24
        g.dp_write(
            SvfGenerator.DP_SELECT,
            select_val,
            f"SELECT APSEL={self.ap_sel}, APBANKSEL=0",
        )
        g.runtest(10)

        # Phase 4 — Configure CSW
        g.comment("=" * 70)
        g.comment("PHASE 4: Configure MEM-AP (CSW)")
        g.comment("=" * 70)
        g.blank()

        g.ap_write(SvfGenerator.AP_CSW, csw, csw_desc)
        g.runtest(10)

    # ------------------------------------------------------------------
    # Main generation
    # ------------------------------------------------------------------

    def generate(self):
        """Read the binary file (or command list) and write the SVF sequence."""
        if self.cmd_list is not None:
            return self._generate_from_cmds()
        return self._generate_from_bin()

    def _generate_from_bin(self):
        """Original binary-download generation path."""

        # --- Read & pad binary -------------------------------------------------
        with open(self.bin_path, "rb") as fh:
            blob = fh.read()

        word_bytes = self.data_width // 8
        remainder = len(blob) % word_bytes
        if remainder:
            pad = word_bytes - remainder
            blob += b"\x00" * pad
            if self.verbose:
                print(
                    f"[INFO] Padded binary with {pad} zero-bytes to "
                    f"{word_bytes}-byte alignment",
                    file=sys.stderr,
                )

        # Unpack into words
        if self.data_width == 32:
            fmt = f"<{len(blob) // 4}I"
        elif self.data_width == 16:
            fmt = f"<{len(blob) // 2}H"
        else:
            fmt = f"<{len(blob)}B"
        words = list(struct.unpack(fmt, blob))

        # --- Build chain configuration -----------------------------------------
        chain = None
        if self.chain_config_path:
            chain = JtagChainConfig.from_json_file(self.chain_config_path)
            if self.verbose:
                print(
                    f"[INFO] JTAG chain config: {self.chain_config_path}",
                    file=sys.stderr,
                )
                print(f"[INFO]   {chain.summary}", file=sys.stderr)

        gen = SvfGenerator(self.output_path, chain=chain)
        g = gen  # shorthand

        # --- Shared setup phases 1–4 -------------------------------------------
        csw = self._csw_value(auto_increment=True)
        size_name = {8: "8-bit", 16: "16-bit", 32: "32-bit"}[self.data_width]
        self._emit_setup_phases(
            g, csw, f"CSW: {size_name}, auto-increment, DeviceEn, DbgSwEnable"
        )

        # ==================================================================
        #  PHASE 5 — Write Binary Data to Memory
        # ==================================================================
        g.comment("=" * 70)
        g.comment(f"PHASE 5: Write {len(blob)} bytes to 0x{self.base_addr:08X}")
        g.comment(f"        Data width: {self.data_width}-bit, Words: {len(words)}")
        g.comment("        Mode: auto-increment (TAR set once)")
        g.comment("=" * 70)
        g.blank()

        word_bytes = self.data_width // 8

        # TAR auto-increments after each DRW access — set it once
        self._write_tar(g, self.base_addr, label="(auto-increment mode)")

        for idx, word in enumerate(words):
            g.ap_write(
                SvfGenerator.AP_DRW,
                word,
                f"DRW ← 0x{word:0{self.data_width // 4}X}  "
                f"→ 0x{self.base_addr + idx * word_bytes:08X}  "
                f"[{idx}/{len(words) - 1}]",
            )

            # Periodic progress annotations
            chunk = max(len(words) // 20, 1)
            if idx > 0 and idx % chunk == 0:
                pct = idx * 100 // len(words)
                g.comment(f"  ... {pct}% complete ({idx}/{len(words)} words)")

        g.comment(f"  ✓ Write complete — {len(words)} words, {len(blob)} bytes total")
        g.blank()

        # ==================================================================
        #  PHASE 6 — Verification (optional)
        # ==================================================================
        if self.verify:
            TDO_MASK = (0xFFFFFFFF << 3) | self.ACK_MASK

            g.comment("=" * 70)
            g.comment("PHASE 6: Read-Back Verification (with TDO check)")
            g.comment(
                f"  Protocol: ADIv{self.adi_version}  "
                f"(ACK OK = 0b{self.ACK_OK:03b}, mask = 0b{self.ACK_MASK:03b})"
            )
            g.comment("  Each SDR includes TDO(expected) MASK — the SVF player")
            g.comment("  compares actual TDO against expected.  Both DATA and ACK")
            g.comment("  bits are verified.  Mismatches are reported immediately.")
            g.comment("=" * 70)
            g.blank()

            # TAR auto-increments after each DRW read — set it once
            # Reset tracking since auto-increment has moved hardware TAR
            self._last_tar_lo = None
            self._write_tar(g, self.base_addr, label="(auto-increment mode, verify)")

            for idx, expected_word in enumerate(words):
                addr = self.base_addr + idx * word_bytes

                # AP read DRW — request; TDO here is the *previous* pipelined
                # result (stale), so we do NOT verify TDO on this transaction.
                g.ap_read(
                    SvfGenerator.AP_DRW,
                    f"Read DRW → 0x{addr:08X}  (request, TDO=stale)",
                )

                # DP read RDBUFF — TDO contains the DRW data from the AP read
                # above.  Verify both data and ACK bits.
                tdo_expected = (expected_word << 3) | self.ACK_OK
                g.dp_read(
                    SvfGenerator.DP_RDBUFF,
                    f"RDBUFF TDO ?= 0x{expected_word:08X} (+ACK=OK)  "
                    f"[{idx}/{len(words) - 1}]",
                    tdo_expected=tdo_expected,
                    tdo_mask=TDO_MASK,
                )

                # Periodic progress annotations
                vchunk = max(len(words) // 20, 1)
                if idx > 0 and idx % vchunk == 0:
                    pct = idx * 100 // len(words)
                    g.comment(f"  ... {pct}% verified ({idx}/{len(words)} words)")

            g.comment("  ✓ Verification sequence complete")

        # ==================================================================
        #  PHASE 7 — Clean Shutdown
        # ==================================================================
        self._emit_shutdown(g)

        g.close()
        return len(blob)

    # ------------------------------------------------------------------
    # Command-file generation
    # ------------------------------------------------------------------

    def _generate_from_cmds(self):
        """Generate SVF from a parsed list of MemCmd entries."""

        if self.verbose:
            reads = sum(1 for c in self.cmd_list if c.op == Op.R)
            writes = sum(1 for c in self.cmd_list if c.op == Op.W)
            waits = sum(1 for c in self.cmd_list if c.op == Op.T)
            print(
                f"[INFO] Command file: {writes} writes, {reads} reads, "
                f"{waits} waits ({len(self.cmd_list)} total)",
                file=sys.stderr,
            )

        # --- Build chain configuration -----------------------------------------
        chain = None
        if self.chain_config_path:
            chain = JtagChainConfig.from_json_file(self.chain_config_path)
            if self.verbose:
                print(
                    f"[INFO] JTAG chain config: {self.chain_config_path}",
                    file=sys.stderr,
                )
                print(f"[INFO]   {chain.summary}", file=sys.stderr)

        gen = SvfGenerator(self.output_path, chain=chain)
        g = gen  # shorthand

        # --- Shared setup phases 1–4 -------------------------------------------
        size_name = {8: "8-bit", 16: "16-bit", 32: "32-bit"}[self.data_width]
        csw = self._csw_value(auto_increment=False)
        self._emit_setup_phases(
            g, csw, f"CSW: {size_name}, ADDRINC_OFF, DeviceEn, DbgSwEnable"
        )

        # ==================================================================
        #  PHASE 5 — Execute Memory Commands
        # ==================================================================
        g.comment("=" * 70)
        waits_count = sum(1 for c in self.cmd_list if c.op == Op.T)
        rw_count = len(self.cmd_list) - waits_count
        g.comment(
            f"PHASE 5: Execute {len(self.cmd_list)} commands "
            f"({rw_count} R/W, {waits_count} wait)"
        )
        g.comment(f"        Data width: {self.data_width}-bit")
        g.comment("        Mode: explicit TAR per access (ADDRINC_OFF)")
        g.comment("=" * 70)
        g.blank()

        TDO_MASK = (0xFFFFFFFF << 3) | self.ACK_MASK
        # Adjust TDO_MASK for non-32-bit widths
        if self.data_width == 16:
            TDO_MASK = (0xFFFF << 3) | self.ACK_MASK
        elif self.data_width == 8:
            TDO_MASK = (0xFF << 3) | self.ACK_MASK

        for idx, cmd in enumerate(self.cmd_list):
            label = f"[{idx + 1}/{len(self.cmd_list)}]"

            if cmd.op == Op.T:
                # --- Wait / delay ---
                g.comment(f"{label} RUNTEST {cmd.data} TCK (wait {cmd.data} cycles)")
                g.runtest(cmd.data)
                g.blank()
                continue

            if cmd.op == Op.R:
                # --- Read operation ---
                self._write_tar(g, cmd.addr, label)

                # Initiate AP DRW read
                g.ap_read(
                    SvfGenerator.AP_DRW,
                    f"{label} Read DRW from 0x{cmd.addr:08X}  (TDO=stale)",
                )

                if cmd.verify:
                    # Verify via pipelined RDBUFF read
                    tdo_expected = (cmd.data << 3) | self.ACK_OK
                    g.dp_read(
                        SvfGenerator.DP_RDBUFF,
                        f"{label} RDBUFF TDO ?= 0x{cmd.data:0{self.data_width // 4}X} "
                        f"(+ACK=OK)",
                        tdo_expected=tdo_expected,
                        tdo_mask=TDO_MASK,
                    )
                else:
                    # Read without verification
                    g.dp_read(
                        SvfGenerator.DP_RDBUFF, f"{label} RDBUFF (TDO not checked)"
                    )
            else:
                # --- Write operation ---
                self._write_tar(g, cmd.addr, label)
                g.ap_write(
                    SvfGenerator.AP_DRW,
                    cmd.data,
                    f"{label} DRW ← 0x{cmd.data:0{self.data_width // 4}X}  "
                    f"→ 0x{cmd.addr:08X}",
                )

            # Periodic progress
            chunk = max(len(self.cmd_list) // 20, 1)
            if (idx + 1) > 0 and (idx + 1) % chunk == 0:
                pct = (idx + 1) * 100 // len(self.cmd_list)
                g.comment(
                    f"  ... {pct}% complete ({idx + 1}/{len(self.cmd_list)} commands)"
                )

        g.comment(f"  ✓ All {len(self.cmd_list)} commands executed")
        g.blank()

        # ==================================================================
        #  PHASE 6 — Clean Shutdown
        # ==================================================================
        self._emit_shutdown(g)

        g.close()
        return len(self.cmd_list)

    # ------------------------------------------------------------------
    # Shared shutdown
    # ------------------------------------------------------------------

    @staticmethod
    def _emit_shutdown(g: SvfGenerator):
        """Emit the final JTAG reset sequence."""
        g.comment("=" * 70)
        g.comment("PHASE 7: JTAG Reset (clean shutdown)")
        g.comment("=" * 70)
        g.blank()
        g.runtest(10)
        g._emit("STATE RESET IDLE;")
        g.blank()
        g.comment("End of SVF — Total TCK cycles include all operations above.")


# =============================================================================
# CLI Entry Point
# =============================================================================


def _parse_address(s: str) -> int:
    """Parse a hex address string like '0x80000000' or '80000000'."""
    try:
        return int(s, 0)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"Invalid address '{s}'.  Use hex (e.g., 0x80000000)."
        )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Generate SVF JTAG sequences for ARM Cortex-R52 memory access "
            "via CoreSight DAP.  Supports .bin file download or text-based "
            "command files with per-access read/write control."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Examples (binary download)
            --------------------------
              %(prog)s firmware.bin --addr 0x80000000 -o download.svf
              %(prog)s app.bin -a 0x20000000 --ap 1 -o flash.svf
              %(prog)s data.bin -a 0x08000000 --width 16 --verify
              %(prog)s fw.bin -a 0x80000000 --chain chain.json -o dl.svf
              %(prog)s fw.bin -a 0x00100000 -A 5 --verify  # ADIv5 (Cortex-A9)

            Examples (command file)
            -----------------------
              %(prog)s --cmds ops.txt -o ops.svf
              %(prog)s --cmds ops.txt -w 16 -o ops.svf
              %(prog)s --cmds ops.txt --chain chain.json -o ops.svf

            Command File Format
            -------------------
            Each non-empty, non-comment line has 3 or 4 whitespace-separated
            fields:
                <addr_hex>  <data_hex>  <W|R>  [Y]
                0  <count>  TCK
            - addr_hex:  memory address in hex (e.g. 0x80000000)
            - data_hex:  data to write, or expected data on read
            - W | R:     write or read operation
            - Y:         (optional, read-only) verify TDO against data_hex
            - 0 <count> TCK:  wait <count> cycles in Run-Test/Idle
            Lines starting with # are comments.

            Chain Config Format (JSON)
            --------------------------
              {
                  "taps": [
                      {"name": "ICE",   "irlen": 4},
                      {"name": "DAP",   "irlen": 4},
                      {"name": "FPGA",  "irlen": 6}
                  ],
                  "target": "DAP"
              }

            Reference
            ---------
              ARM IHI0031H — ARM Debug Interface Architecture Specification
              (ADIv6).  Cortex-R52 Technical Reference Manual.
        """),
    )

    # ---- Source selection (mutually exclusive) --------------------------------
    parser.add_argument(
        "binfile",
        nargs="?",
        default=None,
        help="Path to the .bin (raw binary) file to download.",
    )
    parser.add_argument(
        "--cmds",
        "-C",
        default=None,
        metavar="CMDS.txt",
        help=(
            "Path to a text file listing memory read/write commands.  "
            "Each line: <addr_hex> <data_hex> <W|R> [Y].  "
            "Mutually exclusive with the positional .bin file argument."
        ),
    )

    # ---- Address (required only for .bin mode) --------------------------------
    parser.add_argument(
        "--addr",
        "-a",
        default=None,
        type=_parse_address,
        help="Target memory base address in hex (e.g., 0x80000000).  "
        "Required for .bin file mode, ignored for --cmds mode.",
    )

    # ---- Optional I/O ---------------------------------------------------------
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output SVF file path.  Default: stdout.",
    )
    parser.add_argument(
        "--verbose",
        "-V",
        action="store_true",
        help="Print progress information to stderr.",
    )

    # ---- DAP configuration ----------------------------------------------------
    parser.add_argument(
        "--ap",
        "-p",
        type=int,
        default=0,
        help="MEM-AP selection index (APSEL, default: 0).",
    )
    parser.add_argument(
        "--idcode",
        type=_parse_address,
        default=None,
        help="Expected DP IDCODE for verification (e.g., 0x6BA02477).",
    )
    parser.add_argument(
        "--width",
        "-w",
        type=int,
        choices=[8, 16, 32],
        default=32,
        help="Memory access width in bits (8, 16, or 32).  Default: 32.",
    )
    parser.add_argument(
        "--adi-version",
        "-A",
        type=int,
        choices=[5, 6],
        default=6,
        help=(
            "ARM Debug Interface version: 5 or 6.  Controls CSW bit 31 "
            "(DBGSWENABLE, ADIv6 only).  Default: 6 (Cortex-R52).  "
            "Use 5 for Cortex-A9, Cortex-M4, etc."
        ),
    )

    # ---- JTAG chain -----------------------------------------------------------
    parser.add_argument(
        "--chain",
        "-c",
        default=None,
        metavar="CONFIG.json",
        help=(
            "Path to a JSON file describing the JTAG daisy-chain topology.  "
            "When provided, all SIR/SDR operations are extended to place "
            "non-target TAPs in BYPASS.  See README for the config format."
        ),
    )

    # ---- Features -------------------------------------------------------------
    parser.add_argument(
        "--addr64",
        action="store_true",
        help="Enable 64-bit addressing mode.  Upper 32 bits of the address "
        "are written to AP.TAR2 (register 0x2) before setting AP.TAR.  "
        "Useful for targets with >4 GiB physical address space.",
    )
    parser.add_argument(
        "--verify",
        "-v",
        action="store_true",
        help="Include read-back verification sequence in the SVF output "
        "(.bin mode only; for --cmds mode use the per-command Y flag).",
    )

    args = parser.parse_args()

    # ---- Determine source mode ------------------------------------------------
    has_bin = args.binfile is not None
    has_cmd = args.cmds is not None

    if has_bin and has_cmd:
        print("Error: provide either a .bin file or --cmds, not both.", file=sys.stderr)
        sys.exit(1)
    if not has_bin and not has_cmd:
        print("Error: provide either a .bin file or --cmds.", file=sys.stderr)
        sys.exit(1)

    # ---- Validate .bin mode ---------------------------------------------------
    file_size = 0
    cmd_list = None

    if has_bin:
        if not os.path.isfile(args.binfile):
            print(f"Error: binary file not found — {args.binfile}", file=sys.stderr)
            sys.exit(1)

        file_size = os.path.getsize(args.binfile)
        if file_size == 0:
            print("Error: binary file is empty.", file=sys.stderr)
            sys.exit(1)

        if args.addr is None:
            print("Error: --addr is required for .bin file mode.", file=sys.stderr)
            sys.exit(1)

    # ---- Validate --cmds mode -------------------------------------------------
    if has_cmd:
        if not os.path.isfile(args.cmds):
            print(f"Error: command file not found — {args.cmds}", file=sys.stderr)
            sys.exit(1)

        try:
            cmd_list = parse_cmd_file(args.cmds, data_width=args.width)
        except ValueError as exc:
            print(f"Error parsing command file: {exc}", file=sys.stderr)
            sys.exit(1)

        if not cmd_list:
            print("Error: command file contains no valid commands.", file=sys.stderr)
            sys.exit(1)

        if args.verify:
            print(
                "[WARNING] --verify is ignored in --cmds mode; "
                "use the per-command Y flag for read verification.",
                file=sys.stderr,
            )

    # ---- Validate chain config ------------------------------------------------
    if args.chain and not os.path.isfile(args.chain):
        print(f"Error: chain config file not found — {args.chain}", file=sys.stderr)
        sys.exit(1)

    if args.verbose:
        if has_bin:
            print("[INFO] Mode          : .bin download", file=sys.stderr)
            print(
                f"[INFO] Input file   : {args.binfile} ({file_size} bytes)",
                file=sys.stderr,
            )
            print(f"[INFO] Target addr  : 0x{args.addr:08X}", file=sys.stderr)
        else:
            print("[INFO] Mode          : command file", file=sys.stderr)
            print(
                f"[INFO] Command file : {args.cmds} ({len(cmd_list)} commands)",
                file=sys.stderr,
            )
        print(f"[INFO] ADI version  : v{args.adi_version}", file=sys.stderr)
        print(f"[INFO] Data width   : {args.width}-bit", file=sys.stderr)
        print(f"[INFO] AP selection : APSEL={args.ap}", file=sys.stderr)
        print(
            f"[INFO] JTAG chain   : {args.chain or '(none — single TAP)'}",
            file=sys.stderr,
        )
        print(
            f"[INFO] Verify       : {'Yes' if args.verify else 'No'}", file=sys.stderr
        )
        print(
            f"[INFO] 64-bit addr  : {'Yes' if args.addr64 else 'No'}", file=sys.stderr
        )
        print(f"[INFO] Output       : {args.output or '(stdout)'}", file=sys.stderr)

    # ---- Generate -------------------------------------------------------------
    builder = CortexR52SvfBuilder(
        bin_path=args.binfile,
        base_addr=args.addr,
        output_path=args.output,
        ap_sel=args.ap,
        data_width=args.width,
        dp_idcode=args.idcode,
        verify=args.verify,
        verbose=args.verbose,
        chain_config_path=args.chain,
        adi_version=args.adi_version,
        cmd_list=cmd_list,
        addr64=args.addr64,
    )

    result = builder.generate()

    if args.verbose:
        if has_bin:
            print(
                f"[INFO] SVF generation complete — {result} bytes payload.",
                file=sys.stderr,
            )
        else:
            print(
                f"[INFO] SVF generation complete — {result} commands.", file=sys.stderr
            )


if __name__ == "__main__":
    main()
