#!/usr/bin/env python3
"""
FTDI USB-to-JTAG Adapter
========================

Drives an FTDI USB-to-JTAG bridge (FT2232C/D, FT2232H, FT4232H, FT232H --
e.g. Digilent JTAG-HS1/HS2/SMT1, Olimex ARM-USB-TINY, Amontec JTAGkey, ...)
from the host PC and exposes the raw JTAG shift operations needed to talk to
a Test Access Port (TAP).

Backend selection
-----------------
The FTDI backend library is picked from the host OS:

    ============== ============================ ==================
    Host OS        PyPI distribution            Import package
    ============== ============================ ==================
    Windows        ``pyftdiwin``                ``pyftdi``  (*)
    Linux / macOS  ``pyftdi``                    ``pyftdi``
    ============== ============================ ==================

    (*) ``pyftdiwin`` is a Windows-oriented fork of PyFtdi.  Both
        distributions install the very same top-level ``pyftdi`` package, so
        application code never needs to know which one is present -- this
        module performs the lookup, detects any distribution/platform
        mismatch and reports it through :func:`describe_backend`.

Install the backend with::

    pip install pyftdiwin        # Windows
    pip install pyftdi           # Linux / macOS

Note that both distributions require ``pyusb`` and (on Linux/macOS) a working
``libusb-1.0``:

* Linux: grant access rights to the USB device (udev rule) or run as root,
  and make sure the device is *not* claimed by the ``ftdi_sio`` kernel driver.
* Windows: the device must be bound to a libusb/WinUSB compatible driver
  (e.g. installed with Zadig), not to the FTDI VCP driver.

JTAG pin mapping (fixed by the PyFtdi MPSSE controller, ADBUS)
-------------------------------------------------------------
    ADBUS0 = TCK   ADBUS1 = TDI   ADBUS2 = TDO   ADBUS3 = TMS   ADBUS4 = nTRST

Data representation
-------------------
Instruction Register (IR) and Data Register (DR, a.k.a. TDR -- "Test Data
Register") payloads are plain Python integers.  JTAG shifts the *least*
significant bit first, therefore:

* bit 0 of an integer passed to this module is the first bit clocked into TDI;
* bit 0 of an integer returned by this module is the first bit clocked out of
  TDO.

The IR selects which data register is connected between TDI and TDO; in other
words, an IR value is an instruction that selects a TDR, and the payload is
then shifted through that TDR.

Daisy chains
------------
When several TAPs are wired in series (``TDI -> TAP_0 -> ... -> TAP_n-1 ->
TDO``, as described by ``chain_example.json``), the shift registers of all the
TAPs form one long register.  The first bit clocked into TDI travels through
every TAP and only settles -- after all the shifts -- in the TAP closest to
TDO, while the last bit clocked in stays in the TAP closest to TDI.  The
instruction and data fields of the target TAP therefore start at stream
position ``sum(irlen of the TAPs after it)``, and this adapter composes the
TDI value and extracts the TDO value accordingly.  All the other TAPs are kept
in BYPASS (all-ones IR, 1-bit DR).

Example
-------
::
    from jtag_adapter import JtagAdapter

    with JtagAdapter('ftdi://ftdi:2232h/1', frequency=1.0e6) as jtag:
        # ARM JTAG-DP: IDCODE instruction (IR = 0xE, 4-bit IR), 32-bit DR
        print(hex(jtag.read_idcode()))

        # Shift 32 bits into the TDR selected by IR = 0xB (APACC, 35-bit DR)
        # and print the 35 bits clocked out meanwhile.
        print(hex(jtag.shift_ir_dr(0xB, 0x0000_0000,
                                  ir_bits=4, dr_bits=35)))

Daisy chains (multiple TAPs in series) are supported through the
``chain_example.json`` style configuration shared with ``svf_gen.py``::

    jtag = JtagAdapter(chain='chain_example.json', ir_len=4, dr_len=35)
    jtag.shift_ir_dr(0xA, 0x0, dr_bits=35)   # IR/DR auto-composed for the chain

High-level bus access
---------------------
:class:`BusMaster` and :class:`CfgBusMaster` add 32-bit bus reads and writes on
top of the raw shifts, for FPGA JTAG-to-bus bridges that expose the command,
address and data in a data register.  Two frame formats are supported:

* :class:`BusMaster` -- 34-bit register, ``{command[1:0], addr[31:0]}``::

      read : frame 1 = '10' + addr[31:0]   frame 2 = all zeros -> TDO[31:0]
      write: frame 1 = '11' + addr[31:0]   frame 2 = '01' + data[31:0]

* :class:`CfgBusMaster` -- 55-bit register,
  ``{command[1:0], addr[16:0], be[3:0], data[31:0]}``, the write data
  travelling with the address::

      read : frame 1 = '10' + addr (be = data = 0)
             frame 2 = all zeros -> TDO[31:0] = data
      write: frame 1 = '11' + addr + data   frame 2 = all-zero no-op

Usage::

    with JtagAdapter('ftdi://ftdi:4232:26SG060/1') as jtag:
        bus = BusMaster(jtag, ir=0x00C)          # IR selecting the bridge DR
        bus.write32(0x0000_0100, 0xDEAD_BEEF)
        print(hex(bus.read32(0x0000_0100)))

        cfg = CfgBusMaster(jtag, ir=0x00D)       # wider configuration bridge
        cfg.write32(0x0001_0000, 0x5A5A_5A5A)

Command line
------------
::
    python jtag_adapter.py --list
    python jtag_adapter.py --idcode
    python jtag_adapter.py --ir 0xE --ir-bits 4 --dr-bits 32
    python jtag_adapter.py --ir 0xB --ir-bits 4 --dr 0x1234 --dr-bits 35
    python jtag_adapter.py -c chain_example.json --ir 0xA --ir-bits 4 --dr-bits 35
"""

from __future__ import annotations

import argparse
import importlib
import io
import os
import sys
import types
from dataclasses import dataclass
from typing import Any, List, Mapping, Optional, Tuple

__all__ = [
    "JtagAdapter",
    "JtagAdapterError",
    "FtdiBackendError",
    "FtdiBackend",
    "BusMaster",
    "CfgBusMaster",
    "load_backend",
    "describe_backend",
    "list_devices",
    "default_url",
    "backend_distribution",
    "is_windows",
]


# =============================================================================
# Backend selection -- pyftdiwin on Windows, pyftdi elsewhere
# =============================================================================

WINDOWS_DISTRIBUTION = "pyftdiwin"
POSIX_DISTRIBUTION = "pyftdi"

#: Top-level packages to try when importing the backend.  Both PyPI
#: distributions ship the ``pyftdi`` package; ``pyftdiwin`` is accepted as a
#: fallback for builds that expose their own package name.
PACKAGE_CANDIDATES = ("pyftdi", "pyftdiwin")

#: Default TCK frequency, in Hz.
DEFAULT_FREQUENCY = 1.0e6

#: IDCODE instruction of an ARM JTAG-DP (ADIv5/ADIv6), 4-bit IR.
ARM_IDCODE_IR = 0x0E
ARM_IR_BITS = 4
ARM_IDCODE_DR_BITS = 32


class JtagAdapterError(Exception):
    """Generic error raised by this module."""


class FtdiBackendError(JtagAdapterError):
    """The FTDI backend library is missing or cannot be loaded."""


def is_windows() -> bool:
    """Return True when running on a Windows host."""
    return os.name == "nt" or sys.platform.startswith("win")


def backend_distribution() -> str:
    """Return the PyPI distribution expected on the current host OS."""
    return WINDOWS_DISTRIBUTION if is_windows() else POSIX_DISTRIBUTION


def _distribution_version(distribution: str) -> Optional[str]:
    """Return the installed version of *distribution*, or None."""
    try:
        from importlib.metadata import version  # Python >= 3.8
    except ImportError:  # pragma: no cover - very old interpreters
        return None
    try:
        return version(distribution)
    except Exception:
        return None


def _import_submodule(package: types.ModuleType, name: str) -> types.ModuleType:
    """Import ``<package>.<name>``, falling back to the canonical ``pyftdi``
    namespace, as both distributions share the same package layout."""
    errors = []
    for base in (package.__name__, POSIX_DISTRIBUTION):
        try:
            return importlib.import_module(f"{base}.{name}")
        except ImportError as exc:
            errors.append(exc)
    raise errors[-1]


@dataclass(frozen=True)
class FtdiBackend:
    """A loaded PyFtdi-compatible backend."""

    #: Distribution selected for the current platform (``pyftdi``/``pyftdiwin``).
    distribution: str
    #: Name of the imported top-level package (``pyftdi`` for both distributions).
    package: str
    #: The imported top-level package module.
    module: types.ModuleType
    #: ``pyftdi.ftdi`` module (device handling, ``Ftdi`` class).
    ftdi: types.ModuleType
    #: ``pyftdi.jtag`` module (``JtagEngine``, ``JtagController``).
    jtag: types.ModuleType
    #: ``pyftdi.bits`` module (``BitSequence``).
    bits: types.ModuleType
    #: Version of the selected distribution, when known.
    version: Optional[str] = None
    #: Human readable remarks (distribution/platform mismatch, ...).
    notes: Tuple[str, ...] = ()

    @property
    def Ftdi(self) -> Any:
        """The ``pyftdi.ftdi.Ftdi`` class."""
        return self.ftdi.Ftdi

    @property
    def JtagEngine(self) -> Any:
        """The ``pyftdi.jtag.JtagEngine`` class."""
        return self.jtag.JtagEngine

    @property
    def BitSequence(self) -> Any:
        """The ``pyftdi.bits.BitSequence`` class."""
        return self.bits.BitSequence

    def summary(self) -> str:
        """Return a one-line description of this backend."""
        text = f"{self.distribution} {self.version or '?'}"
        if self.package != self.distribution:
            text += f" (import package '{self.package}')"
        if self.notes:
            text += "  [" + "; ".join(self.notes) + "]"
        return text


def load_backend(distribution: Optional[str] = None) -> FtdiBackend:
    """Import and return the FTDI backend for the current platform.

    On Windows the ``pyftdiwin`` distribution is used, on any other OS the
    ``pyftdi`` distribution.  When the expected distribution is not installed
    the other one is used if available, and the mismatch is reported in
    :attr:`FtdiBackend.notes`.

    :param distribution: optional distribution name override
        (``'pyftdi'`` or ``'pyftdiwin'``)
    :raise FtdiBackendError: when no backend can be loaded
    """
    wanted = distribution or backend_distribution()
    notes: List[str] = []

    installed = [name for name in (WINDOWS_DISTRIBUTION, POSIX_DISTRIBUTION)
                 if _distribution_version(name)]
    if wanted not in installed and installed:
        fallback = installed[0]
        notes.append(
            f"'{wanted}' is not installed; falling back to '{fallback}'"
        )
        wanted = fallback

    if wanted == WINDOWS_DISTRIBUTION and not is_windows():
        notes.append(f"'{WINDOWS_DISTRIBUTION}' targets Windows hosts")
    elif wanted == POSIX_DISTRIBUTION and is_windows():
        notes.append(
            f"on Windows, '{WINDOWS_DISTRIBUTION}' is the recommended backend"
        )

    module = None
    first_error: Optional[ImportError] = None
    for name in PACKAGE_CANDIDATES:
        try:
            module = importlib.import_module(name)
            break
        except ImportError as exc:
            first_error = first_error or exc
    if module is None:
        raise FtdiBackendError(
            f"unable to import the FTDI backend "
            f"{'/'.join(PACKAGE_CANDIDATES)}: {first_error} "
            f"(run 'pip install {wanted}')"
        )

    try:
        ftdi = _import_submodule(module, "ftdi")
        jtag = _import_submodule(module, "jtag")
        bits = _import_submodule(module, "bits")
    except ImportError as exc:
        raise FtdiBackendError(
            f"'{module.__name__}' is installed but its dependencies are "
            f"incomplete: {exc} (pyusb, and libusb-1.0 on Linux/macOS, are "
            f"required)"
        ) from exc

    return FtdiBackend(
        distribution=wanted,
        package=module.__name__,
        module=module,
        ftdi=ftdi,
        jtag=jtag,
        bits=bits,
        version=_distribution_version(wanted)
        or getattr(module, "__version__", None),
        notes=tuple(notes),
    )


def describe_backend(backend: Optional[FtdiBackend] = None) -> str:
    """Return a one-line description of the active FTDI backend.

    Loads the backend when none is given, so this is a convenient way to
    check *which* library would be used on the current host.
    """
    return (backend or load_backend()).summary()


# =============================================================================
# FTDI device enumeration
# =============================================================================


def _build_device_strings(backend: FtdiBackend,
                          devdescs: List[Any]) -> Optional[List[Tuple[str, str]]]:
    """Turn USB descriptors into ``(url, description)`` pairs (best effort)."""
    try:
        usbtools = _import_submodule(backend.module, "usbtools")
        Ftdi = backend.Ftdi
        return [
            (url, description)
            for url, description in usbtools.UsbTools.build_dev_strings(
                "ftdi", Ftdi.VENDOR_IDS, Ftdi.PRODUCT_IDS, devdescs
            )
        ]
    except Exception:
        return None


def _parse_show_devices(text: str) -> List[Tuple[str, str]]:
    """Extract ``(url, description)`` pairs from ``Ftdi.show_devices()`` text."""
    devices = []
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("ftdi://"):
            continue
        parts = line.split(None, 1)
        description = parts[1].strip() if len(parts) > 1 else ""
        devices.append((parts[0], description))
    return devices


def list_devices(url: Optional[str] = None,
                 backend: Optional[FtdiBackend] = None
                 ) -> List[Tuple[str, str]]:
    """List the connected FTDI interfaces.

    :param url: optional URL pattern to restrict the search
    :param backend: optional pre-loaded backend
    :return: a list of ``(url, description)`` pairs, empty when no device was
        found
    """
    backend = backend or load_backend()
    Ftdi = backend.Ftdi
    try:
        devdescs = Ftdi.list_devices(url)
    except Exception as exc:  # USB access error (permissions, driver, ...)
        raise FtdiBackendError(f"unable to enumerate FTDI devices: {exc}") from exc

    if not devdescs:
        return []

    # Recent PyFtdi returns USB descriptors, older ones return URLs.
    if all(isinstance(desc, str) for desc in devdescs):
        return [(desc, "") for desc in devdescs]

    devices = _build_device_strings(backend, devdescs)
    if devices:
        return devices

    buffer = io.StringIO()
    try:
        Ftdi.show_devices(url, out=buffer)
    except Exception:
        pass
    devices = _parse_show_devices(buffer.getvalue())
    if devices:
        return devices

    return [(str(desc), "") for desc in devdescs]


def default_url(backend: Optional[FtdiBackend] = None) -> str:
    """Return the URL of the first FTDI interface found.

    :raise JtagAdapterError: when no FTDI device is connected
    """
    devices = list_devices(backend=backend)
    if not devices:
        raise JtagAdapterError(
            "no FTDI device found: check the USB connection, the driver "
            "binding and (on Linux) the udev permissions"
        )
    return devices[0][0]


# =============================================================================
# Optional daisy-chain support (shared with svf_gen.py)
# =============================================================================


def _import_chain_config() -> Any:
    """Import ``JtagChainConfig`` from the sibling ``svf_gen`` module."""
    try:
        from svf_gen import JtagChainConfig
    except ImportError:
        here = os.path.dirname(os.path.abspath(__file__))
        if here not in sys.path:
            sys.path.insert(0, here)
        try:
            from svf_gen import JtagChainConfig
        except ImportError as exc:
            raise JtagAdapterError(
                f"JTAG daisy-chain support requires svf_gen.py: {exc}"
            ) from exc
    return JtagChainConfig


def _resolve_chain(chain: Any) -> Any:
    """Normalize the *chain* argument into a ``JtagChainConfig``.

    Accepted forms: ``None``, a ``JtagChainConfig`` instance, a path to a JSON
    chain description file, or an already parsed chain description mapping
    (``{"taps": [...], "target": "..."}``).
    """
    if chain is None:
        return None

    JtagChainConfig = _import_chain_config()
    if isinstance(chain, JtagChainConfig):
        return chain
    if isinstance(chain, (str, bytes, os.PathLike)):
        return JtagChainConfig.from_json_file(os.fspath(chain))
    if isinstance(chain, Mapping):
        taps = list(chain.get("taps") or [])
        if not taps:
            raise JtagAdapterError("chain config: 'taps' must not be empty")
        index = chain.get("target_index")
        if chain.get("target") is not None:
            names = [tap.get("name") for tap in taps]
            if chain["target"] not in names:
                raise JtagAdapterError(
                    f"chain config: target '{chain['target']}' not found in "
                    f"taps list {names}"
                )
            index = names.index(chain["target"])
        return JtagChainConfig(taps=taps, target_index=int(index or 0))
    raise JtagAdapterError(
        f"unsupported chain description: {type(chain).__name__}"
    )


# =============================================================================
# JTAG Adapter
# =============================================================================


class JtagAdapter:
    """JTAG access to a TAP through an FTDI USB-to-JTAG bridge.

    The adapter is opened lazily on first use, so it is valid to construct it
    and start shifting immediately; using it as a context manager is
    recommended to make sure the USB device is released::

        with JtagAdapter('ftdi://ftdi:2232h/1', frequency=1.0e6) as jtag:
            idcode = jtag.read_idcode()

    :param url: FTDI URL selector (``'ftdi://ftdi:2232h/1'``, ``'ftdi:///?'
        ``-style pattern, or any URL accepted by PyFtdi).  ``None`` selects the
        first FTDI interface found on the USB bus.
    :param frequency: TCK frequency in Hz; the closest frequency reachable by
        the FTDI clock divider is used.  Keep it below the limit of the
        bridge (6 MHz for FT2232C/D, 30 MHz for FT2232H/FT4232H/FT232H).
    :param trst: drive the optional nTRST line (ADBUS4) during TAP reset.
    :param ir_len: default width, in bits, of the (target) instruction
        register -- e.g. 4 for an ARM JTAG-DP.  When a *chain* description is
        given, its ``irlen`` for the target TAP is used instead.
    :param dr_len: default width, in bits, of the data register -- e.g. 35 for
        an ARM DPACC/APACC transfer, 32 for IDCODE.
    :param chain: optional JTAG daisy-chain description, either a
        ``svf_gen.JtagChainConfig`` instance, a path to a chain JSON file
        (see ``chain_example.json``) or a chain description mapping.  The
        target TAP is then transparently addressed and the other TAPs are kept
        in BYPASS.
    :param backend: optional FTDI backend distribution override
        (``'pyftdi'``/``'pyftdiwin'``).
    :param idle_after: return the TAP to Run-Test/Idle after each shift
        (default).  Debug ports such as the ARM DAP complete a memory access
        while the TAP is in Run-Test/Idle, so this should normally stay
        enabled.
    :param reset_on_open: reset the TAP (Test-Logic-Reset) when the adapter is
        opened.
    """

    def __init__(self,
                 url: Optional[str] = None,
                 *,
                 frequency: float = DEFAULT_FREQUENCY,
                 trst: bool = False,
                 ir_len: Optional[int] = None,
                 dr_len: Optional[int] = None,
                 chain: Any = None,
                 backend: Optional[str] = None,
                 idle_after: bool = True,
                 reset_on_open: bool = True):
        self._url = url
        self._frequency = float(frequency)
        self._trst = bool(trst)
        self._ir_len = ir_len
        self._dr_len = dr_len
        self._chain = _resolve_chain(chain)
        self._backend_override = backend
        self._idle_after = bool(idle_after)
        self._reset_on_open = bool(reset_on_open)
        self._backend: Optional[FtdiBackend] = None
        self._engine: Any = None

    # ------------------------------------------------------------------
    # Session handling
    # ------------------------------------------------------------------

    @property
    def backend(self) -> FtdiBackend:
        """The loaded FTDI backend."""
        if self._backend is None:
            self._backend = load_backend(self._backend_override)
        return self._backend

    @property
    def engine(self) -> Any:
        """The underlying ``pyftdi.jtag.JtagEngine``, or None when closed."""
        return self._engine

    @property
    def url(self) -> Optional[str]:
        """FTDI URL selector of the adapter (resolved when opened)."""
        return self._url

    @property
    def frequency(self) -> float:
        """Configured TCK frequency, in Hz."""
        return self._frequency

    @property
    def ir_len(self) -> Optional[int]:
        """Default instruction register width, in bits."""
        return self._ir_len

    @property
    def dr_len(self) -> Optional[int]:
        """Default data register width, in bits."""
        return self._dr_len

    @property
    def chain(self) -> Any:
        """The ``JtagChainConfig`` in use, or None for a single-TAP chain."""
        return self._chain

    @property
    def is_open(self) -> bool:
        """True when the FTDI device is open."""
        return self._engine is not None

    def open(self, url: Optional[str] = None) -> "JtagAdapter":
        """Open the FTDI device and reset the TAP.

        :param url: optional URL overriding the one given to the constructor
        """
        if self._engine is not None:
            return self

        backend = self.backend
        if url is not None:
            self._url = url
        if self._url is None:
            self._url = default_url(backend)

        engine = self._make_engine(backend, self._trst, self._frequency)
        try:
            configure = getattr(engine, "configure", None)
            if configure is None:
                configure = engine.controller.configure
            configure(self._url)
        except Exception as exc:
            try:
                engine.close(freeze=True)
            except Exception:
                pass
            raise JtagAdapterError(
                f"unable to open FTDI device '{self._url}': {exc}"
            ) from exc

        self._engine = engine
        if self._reset_on_open:
            self.reset()
        return self

    def close(self, freeze: bool = False) -> None:
        """Close the FTDI device.

        :param freeze: keep the FTDI port in its current state instead of
            resetting it to the default configuration.
        """
        engine, self._engine = self._engine, None
        if engine is None:
            return
        try:
            engine.close(freeze=freeze)
        except TypeError:  # very old PyFtdi signature
            engine.close()

    def __enter__(self) -> "JtagAdapter":
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        self.close()
        return False

    @staticmethod
    def _make_engine(backend: FtdiBackend, trst: bool, frequency: float) -> Any:
        """Instantiate the PyFtdi JTAG engine (version tolerant)."""
        try:
            return backend.JtagEngine(trst=trst, frequency=frequency)
        except TypeError:
            # PyFtdi < 0.30: the engine wraps an explicit controller instance
            controller = backend.jtag.JtagController(
                trst=trst, frequency=frequency)
            try:
                return backend.JtagEngine(controller)
            except TypeError as exc:
                raise JtagAdapterError(
                    f"unsupported PyFtdi JTAG engine API: {exc}"
                ) from exc

    def _require_engine(self) -> Any:
        """Return the open engine, opening the adapter if needed."""
        if self._engine is None:
            self.open()
        return self._engine

    # ------------------------------------------------------------------
    # TAP control
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reset the TAP (Test-Logic-Reset).

        Uses the nTRST line when the adapter was created with ``trst=True``,
        and always pulses TMS high for 5 TCK cycles.
        """
        self._require_engine().reset()

    def go_idle(self) -> None:
        """Move the TAP to Run-Test/Idle."""
        self._require_engine().go_idle()

    def runtest(self, cycles: int = 1) -> None:
        """Clock *cycles* TCK periods in Run-Test/Idle.

        Useful for debug ports that need bus time to complete an access.
        """
        engine = self._require_engine()
        engine.go_idle()
        if cycles > 0:
            zeros = self.backend.BitSequence(length=cycles)
            engine.write(zeros, False)
            engine.sync()

    # ------------------------------------------------------------------
    # IR / DR shift primitives
    # ------------------------------------------------------------------

    def shift_ir(self,
                 ir: int,
                 ir_bits: Optional[int] = None,
                 *,
                 idle: Optional[bool] = None) -> int:
        """Load *ir* into the instruction register.

        :param ir: instruction value; bit 0 is shifted in first
        :param ir_bits: IR width in bits, defaults to the adapter ``ir_len``
        :param idle: override the adapter ``idle_after`` setting
        :return: the value clocked out of TDO meanwhile -- i.e. the
            instruction that was loaded before this call (bit 0 shifted out
            first)
        """
        engine = self._require_engine()
        bits = self._resolve_ir_bits(ir, ir_bits)
        total, tdi = self._compose_ir(ir, bits)

        engine.change_state("shift_ir")
        tdo = self._shift_out(engine, tdi, total, name="IR",
                              update_state="update_ir")
        if self._should_idle(idle):
            engine.go_idle()
        return self._extract(tdo, bits, self._ir_field_offset)

    def shift_dr(self,
                 data: int,
                 dr_bits: Optional[int] = None,
                 *,
                 idle: Optional[bool] = None) -> int:
        """Shift *data* through the data register (TDR) selected by the IR.

        The instruction register must already select the wanted data register
        -- see :meth:`shift_ir` or :meth:`shift_ir_dr`.

        :param data: payload shifted into TDI; bit 0 is shifted in first
        :param dr_bits: DR width in bits, defaults to the adapter ``dr_len``,
            and to ``data.bit_length()`` (at least 1 bit) when neither is set
            -- pass an explicit width to read a register with a zero payload
        :param idle: override the adapter ``idle_after`` setting
        :return: the value clocked out of TDO meanwhile, i.e. the previous
            content of the register for a read-only TDR (bit 0 shifted out
            first)
        """
        engine = self._require_engine()
        bits = self._resolve_dr_bits(data, dr_bits)
        total, tdi = self._compose_dr(data, bits)

        engine.change_state("shift_dr")
        tdo = self._shift_out(engine, tdi, total, name="DR",
                              update_state="update_dr")
        if self._should_idle(idle):
            engine.go_idle()
        return self._extract(tdo, bits, self._dr_field_offset)

    def read_dr(self, dr_bits: Optional[int] = None, *,
                idle: Optional[bool] = None) -> int:
        """Shift zeroes through the selected data register and return the
        data clocked out of TDO."""
        bits = self._resolve_dr_bits(0, dr_bits)
        return self.shift_dr(0, bits, idle=idle)

    def shift_ir_dr(self,
                    ir: int,
                    data: int,
                    *,
                    ir_bits: Optional[int] = None,
                    dr_bits: Optional[int] = None,
                    idle: Optional[bool] = None) -> int:
        """Load *ir* into the IR and shift *data* through the TDR it selects.

        This is the core primitive of the adapter: the IR value selects which
        data register is connected between TDI and TDO, *data* is then shifted
        into that register (the first bit of *data* being bit 0), and the value
        that was clocked out of TDO meanwhile is returned -- for a read-only
        or status register this is the captured content, for a writable
        register it is the previous value.

        :param ir: instruction (IR) value selecting the target data register
        :param data: payload shifted into the selected TDR (bit 0 first)
        :param ir_bits: IR width in bits; defaults to the adapter ``ir_len``
        :param dr_bits: DR width in bits; defaults to the adapter ``dr_len``,
            and to ``data.bit_length()`` (at least 1 bit) when neither is set.
            Always pass an explicit width when the payload is 0 (a pure read)
            or when the instruction selects a wide register.
        :param idle: override the adapter ``idle_after`` setting
        :return: the data clocked out of TDO, with bit 0 shifted out first
        :raise JtagAdapterError: when the IR width is unknown or a value does
            not fit in the requested width

        Example -- ARM JTAG-DP IDCODE read (IR = 0xE, 4 bits; DR = 32 bits)::

            idcode = jtag.shift_ir_dr(0x0E, 0, ir_bits=4, dr_bits=32)
        """
        engine = self._require_engine()
        ir_width = self._resolve_ir_bits(ir, ir_bits)
        dr_width = self._resolve_dr_bits(data, dr_bits)

        ir_total, ir_tdi = self._compose_ir(ir, ir_width)
        dr_total, dr_tdi = self._compose_dr(data, dr_width)

        # 1. select the target data register
        engine.change_state("shift_ir")
        self._shift_out(engine, ir_tdi, ir_total, name="IR",
                        update_state="update_ir")

        # 2. shift the payload through that register
        engine.change_state("shift_dr")
        tdo = self._shift_out(engine, dr_tdi, dr_total, name="DR",
                              update_state="update_dr")

        if self._should_idle(idle):
            engine.go_idle()
        return self._extract(tdo, dr_width, self._dr_field_offset)

    def read_idcode(self,
                    *,
                    ir: int = ARM_IDCODE_IR,
                    ir_bits: int = ARM_IR_BITS,
                    dr_bits: int = ARM_IDCODE_DR_BITS,
                    idle: Optional[bool] = None) -> int:
        """Read the 32-bit IDCODE of the target TAP.

        The defaults match an ARM JTAG-DP (IDCODE instruction ``0xE`` on a
        4-bit IR); pass ``ir``/``ir_bits`` for another TAP.

        :return: the 32-bit IDCODE value
        """
        return self.shift_ir_dr(ir, 0, ir_bits=ir_bits, dr_bits=dr_bits,
                                idle=idle)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _shift_out(self, engine: Any, value: int, bits: int, *,
                   name: str, update_state: str) -> int:
        """Shift *bits* bits of *value* out of the current shift state and
        return the raw TDO value (full chain width, bit 0 shifted out first).

        Leaves the TAP in the update state.
        """
        payload = self._to_bits(value, bits, name)
        if bits > 1:
            # shift_and_update_register() defers the last bit into the TMS
            # transition that ends the shift, so it reads all bits at once.
            tdo = engine.shift_and_update_register(payload)
        else:
            # A single-bit register (e.g. BYPASS) leaves nothing to defer:
            # use the plain shift then move to the update state explicitly.
            tdo = engine.shift_register(payload)
            engine.change_state(update_state)
        return int(tdo)

    def _should_idle(self, idle: Optional[bool]) -> bool:
        return self._idle_after if idle is None else bool(idle)

    def _resolve_ir_bits(self, ir: int, ir_bits: Optional[int]) -> int:
        if ir_bits is None:
            ir_bits = self._ir_len
        if ir_bits is None and self._daisy_chain:
            # the chain description knows the target IR width
            ir_bits = self._tap_irlens[self._chain.target_index]
        if ir_bits is None:
            raise JtagAdapterError(
                "IR width is unknown: pass ir_bits=... or set ir_len=... when "
                "creating the adapter"
            )
        if ir_bits < 1:
            raise JtagAdapterError(f"invalid IR width: {ir_bits} bit(s)")
        return int(ir_bits)

    def _resolve_dr_bits(self, data: int, dr_bits: Optional[int]) -> int:
        if dr_bits is None:
            dr_bits = self._dr_len
        if dr_bits is None:
            dr_bits = max(1, int(data).bit_length())
        if dr_bits < 1:
            raise JtagAdapterError(f"invalid DR width: {dr_bits} bit(s)")
        return int(dr_bits)

    def _to_bits(self, value: int, bits: int, name: str) -> Any:
        """Convert *value* into the backend bit sequence of width *bits*."""
        if not isinstance(value, int):
            raise JtagAdapterError(
                f"{name} payload must be an integer, not "
                f"{type(value).__name__}"
            )
        if value < 0:
            raise JtagAdapterError(f"{name} payload must not be negative")
        if value >> bits:
            raise JtagAdapterError(
                f"{name} payload 0x{value:x} does not fit in {bits} bit(s)"
            )
        return self.backend.BitSequence(value, length=bits)

    def _compose_ir(self, ir: int, bits: int) -> Tuple[int, int]:
        """Return ``(total_bits, tdi)`` for an IR shift.

        In a daisy chain the TDI bit stream is shared by every TAP: the first
        bit clocked in travels through all the TAPs and settles in the TAP
        closest to TDO, while the last bit clocked in stays in the TAP closest
        to TDI.  The instruction field of the TAP at *index* therefore sits at
        stream position ``sum(irlen of the TAPs after it)``.
        """
        if not self._daisy_chain:
            return bits, ir

        irlens = self._tap_irlens
        target = self._chain.target_index
        if bits != irlens[target]:
            raise JtagAdapterError(
                f"IR width mismatch for the target TAP "
                f"'{self._chain.target_name}': {bits} bit(s) given, "
                f"{irlens[target]} bit(s) in the chain description"
            )

        tdi = ir << self._ir_field_offset
        for index, irlen in enumerate(irlens):
            if index == target:
                continue
            # every other TAP is kept in BYPASS (IR = all ones)
            bypass = (1 << irlen) - 1
            tdi |= bypass << sum(irlens[index + 1:])
        return sum(irlens), tdi

    def _compose_dr(self, data: int, bits: int) -> Tuple[int, int]:
        """Return ``(total_bits, tdi)`` for a DR shift.

        Non-target TAPs are in BYPASS and contribute a single DR bit each, so
        the full chain is ``bits + (number of TAPs - 1)`` bit long, and the
        target data register sits after the BYPASS bits of the TAPs that are
        closer to TDO.
        """
        if not self._daisy_chain:
            return bits, data
        offset = self._dr_field_offset
        return bits + len(self._chain.taps) - 1, data << offset

    @staticmethod
    def _extract(tdo: int, bits: int, offset: int) -> int:
        """Pick the target TAP's bits out of the full chain TDO value."""
        return (tdo >> offset) & ((1 << bits) - 1)

    @property
    def _daisy_chain(self) -> bool:
        return self._chain is not None and self._chain.is_daisy_chain

    @property
    def _tap_irlens(self) -> List[int]:
        return [int(tap["irlen"]) for tap in self._chain.taps]

    @property
    def _ir_field_offset(self) -> int:
        """Number of IR bits clocked before the target TAP's IR bits."""
        if not self._daisy_chain:
            return 0
        return sum(self._tap_irlens[self._chain.target_index + 1:])

    @property
    def _dr_field_offset(self) -> int:
        """Number of DR bits clocked before the target TAP's DR bits.

        Each non-target TAP left in BYPASS contributes exactly one bit, and
        the bits of the TAPs closer to TDO are clocked out first.
        """
        if not self._daisy_chain:
            return 0
        return len(self._chain.taps) - self._chain.target_index - 1

    def __repr__(self) -> str:
        state = "open" if self.is_open else "closed"
        chain = f", chain={self._chain.summary}" if self._chain else ""
        return (f"JtagAdapter(url={self._url!r}, frequency={self._frequency:g}"
                f", ir_len={self._ir_len}, dr_len={self._dr_len}"
                f"{chain}, {state})")


# =============================================================================
# High-level bus access -- JTAG bridge with a 34-bit data register
# =============================================================================

class BusMaster:
    """Read and write 32-bit bus locations through a JTAG-to-bus bridge.

    The bridge is selected by an instruction register value (*ir*) and exposes
    a 34-bit data register laid out as ``{command, payload[31:0]}``::

        DR[33:32] = 2-bit command      DR[31:0] = address / data payload

    Each access takes two scans:

    ========= ====================================== =========================
    Operation Frame 1 -- address phase               Frame 2 -- data phase
    ========= ====================================== =========================
    read      ``'10'`` (DR[33:32] = 0b10) +          ~~~~ all zeros ~~~~
              ``addr[31:0]``                         (TDO[31:0] = read data)
    write     ``'11'`` (DR[33:32] = 0b11) +          ``'01'`` (0b01) +
              ``addr[31:0]``                         ``data[31:0]``
    ========= ====================================== =========================

    The command sits in the high bits of the register, so it is the *last*
    thing clocked in; the payload is clocked in first, least-significant bit
    first -- the same order this module uses for every IR/DR payload::

        frame value = (command << 32) | payload

    Example::

        with JtagAdapter('ftdi://ftdi:4232:26SG060/1', frequency=1e6) as jtag:
            bus = BusMaster(jtag, ir=0x00C)     # IR selecting the bridge
            bus.write32(0x0000_0100, 0xDEAD_BEEF)
            print(hex(bus.read32(0x0000_0100)))
            print([hex(w) for w in bus.read_words(0x0000_0100, 4)])

    :param adapter: open (or lazily opened) :class:`JtagAdapter`
    :param ir: instruction that selects the bridge data register
    :param ir_bits: IR width in bits, defaults to the adapter ``ir_len``
    :param dr_bits: bridge data register width, defaults to
        :data:`BUS_DR_BITS` (34 bits)
    :param read_wait: TCK cycles clocked in Run-Test/Idle between the two
        scans of a read, for bridges that need bus time before the data is
        valid (``0`` -- the default -- clocks straight through)
    :param idle: override the adapter ``idle_after`` setting for each scan
    """
    #: Width of the bridge data register: 2-bit command + 32-bit payload.
    BUS_DR_BITS = 34

    #: Width of the address / data payload.
    BUS_PAYLOAD_BITS = 32

    #: 2-bit command field, held in the *high* bits of the data register
    #: (``DR[33:32]``), i.e. the register is ``{command, payload[31:0]}``.  The
    #: command is therefore clocked in last, and the values are the binary
    #: literals of the frame description: ``'10'`` = ``0b10`` for a read, etc.
    BUS_CMD_READ = 0b10
    BUS_CMD_WRITE = 0b11
    BUS_CMD_DATA = 0b01

    def __init__(self,
                 adapter: JtagAdapter,
                 ir: int,
                 *,
                 ir_bits: Optional[int] = None,
                 read_wait: int = 0,
                 idle: Optional[bool] = None):
        self._adapter = adapter
        self._ir = ir
        self._ir_bits = ir_bits
        self._dr_bits = self.BUS_DR_BITS
        self._read_wait = read_wait
        self._idle = idle

    # ------------------------------------------------------------------
    # Frame helpers
    # ------------------------------------------------------------------

    @property
    def adapter(self) -> JtagAdapter:
        """The underlying :class:`JtagAdapter`."""
        return self._adapter

    @property
    def ir(self) -> int:
        """Instruction that selects the bridge data register."""
        return self._ir

    @property
    def dr_bits(self) -> int:
        """Bridge data register width, in bits."""
        return self._dr_bits

    @classmethod
    def frame(cls, command: int, payload: int = 0) -> int:
        """Return the data register value for *command* and *payload*.

        The 2-bit command is placed in the high bits of the register, i.e. at
        ``DR[BUS_DR_BITS-1:BUS_DR_BITS-2]``; the payload fills everything
        below it.  Subclasses only need to set :attr:`BUS_DR_BITS` and to
        build their payload in :meth:`setup_frame` / :meth:`action_frame`.
        """
        return ((command & 0b11) << (cls.BUS_DR_BITS - 2)) | payload

    @classmethod
    def setup_frame(cls, command: int, address: int, data: int = 0) -> int:
        """Return the read / write address-phase frame.

        For this format the frame is the 2-bit command followed by the 32-bit
        address; *data* is ignored (it is sent by :meth:`action_frame`), and
        only exists so that subclasses whose address frame also carries the
        write data can be called the same way.
        """
        if not 0 <= address < (1 << cls.BUS_PAYLOAD_BITS):
            raise JtagAdapterError(
                f"bus address 0x{address:x} does not fit in "
                f"{cls.BUS_PAYLOAD_BITS} bit(s)"
            )
        return cls.frame(command, address)

    @classmethod
    def action_frame(cls, data: int) -> int:
        """Return the write data-phase frame (``'01'`` + 32-bit data)."""
        if not 0 <= data < (1 << cls.BUS_PAYLOAD_BITS):
            raise JtagAdapterError(
                f"bus data 0x{data:x} does not fit in "
                f"{cls.BUS_PAYLOAD_BITS} bit(s)"
            )
        return cls.frame(cls.BUS_CMD_DATA, data)

    def _scan(self, value: int) -> int:
        """Shift one frame through the bridge and return the TDO value."""
        return self._adapter.shift_dr(value, self._dr_bits, idle=self._idle)

    def _select(self) -> None:
        """Load the bridge instruction into the IR."""
        self._adapter.shift_ir(self._ir, self._ir_bits,
                               idle=self._idle)

    # ------------------------------------------------------------------
    # Word access
    # ------------------------------------------------------------------

    def read32(self, address: int) -> int:
        """Read the 32-bit word at *address*.

        Frame 1 (``'10'`` + address) requests the read, frame 2 (all zeros)
        clocks the data out in bits [31:0] of the shifted-out value.  When
        ``read_wait`` is set, that many TCK periods are clocked in
        Run-Test/Idle in between.

        :return: the 32-bit data
        """
        self._select()
        self._scan(self.setup_frame(self.BUS_CMD_READ, address))
        if self._read_wait:
            self._adapter.runtest(self._read_wait)
        return self._scan(0) & ((1 << self.BUS_PAYLOAD_BITS) - 1)

    def write32(self, address: int, data: int) -> None:
        """Write the 32-bit *data* to *address*.

        Frame 1 is ``'11'`` + address, frame 2 is ``'01'`` + data.
        """
        self._select()
        self._scan(self.setup_frame(self.BUS_CMD_WRITE, address, data))
        self._scan(self.action_frame(data))

    def read_words(self,
                   address: int,
                   count: int,
                   *,
                   stride: int = 4) -> List[int]:
        """Read *count* consecutive words starting at *address*.

        The bridge only carries one address per access, so the words are read
        with one :meth:`read32` per address.

        :param stride: address increment between two words (default: 4)
        """
        if count < 0:
            raise JtagAdapterError(f"invalid word count: {count}")
        return [self.read32(address + index * stride)
                for index in range(count)]

    def write_words(self,
                    address: int,
                    data: List[int],
                    *,
                    stride: int = 4) -> None:
        """Write *data* to consecutive words starting at *address*."""
        for index, word in enumerate(data):
            self.write32(address + index * stride, word)

    # ------------------------------------------------------------------
    # Raw frame access (bridge bring-up / other protocols)
    # ------------------------------------------------------------------

    def scan(self, frame: int) -> int:
        """Shift an arbitrary 34-bit *frame* into the bridge.

        Useful to drive bridge commands that this class does not model yet:
        the IR is reloaded first and the shifted-out data register value is
        returned.

        :param frame: raw data register value to shift in
        :return: the value clocked out of TDO
        """
        self._select()
        return self._scan(frame)

    def __repr__(self) -> str:
        return (f"BusMaster(ir=0x{self._ir:x}, dr_bits={self._dr_bits}, "
                f"adapter={self._adapter!r})")


class CfgBusMaster(BusMaster):
    """BusMaster for the 55-bit configuration-bus frame.

    Same two-scan protocol as :class:`BusMaster`, but a wider data register
    that carries the address, the write data and a 4-bit byte-enable field in
    a single frame::

        DR[54:53] = 2-bit command      '10' = read, '11' = write
        DR[52:36] = 17-bit address
        DR[35:32] = 4-bit byte enable (reserved, always 0 for now)
        DR[31:0]  = 32-bit data

        frame value = (command << 53) | (address << 36) | (be << 32) | data

    The write data travels *with* the address in frame 1, so the second scan
    of a write is an all-zero no-op frame; a read still leaves its data in
    ``TDO[31:0]`` of the second scan::

        read : frame 1 = '10' + addr + be=0 + data=0
               frame 2 = all zeros                -> TDO[31:0] = read data
        write: frame 1 = '11' + addr + be=0 + data
               frame 2 = all zeros

    Example::

        bus = CfgBusMaster(jtag, ir=0x00C)
        bus.write32(0x0001_2345, 0xDEAD_BEEF)
        print(hex(bus.read32(0x0001_2345)))
    """

    #: Data register: 2-bit command + 17-bit address + 4-bit byte enable
    #: + 32-bit data.
    BUS_DR_BITS = 55

    #: Width of the 32-bit address / data value.
    BUS_PAYLOAD_BITS = 32
    #: Width of the address field.
    BUS_ADDRESS_BITS = 17
    #: Width of the byte-enable field (reserved, currently always 0).
    BUS_STRB_BITS = 4

    #: Position of the address field: above the byte-enable and data fields.
    BUS_ADDR_OFFSET = BUS_PAYLOAD_BITS + BUS_STRB_BITS

    #: 2-bit command field, held in the high bits of the data register
    #: (``DR[54:53]``).
    BUS_CMD_READ = 0b10
    BUS_CMD_WRITE = 0b11

    def __init__(self,
                 adapter: JtagAdapter,
                 ir: int,
                 *,
                 ir_bits: Optional[int] = None,
                 read_wait: int = 0,
                 idle: Optional[bool] = None):
        super().__init__(adapter, ir, ir_bits=ir_bits,
                         read_wait=read_wait, idle=idle)

    @classmethod
    def setup_frame(cls, command: int, address: int, data: int = 0) -> int:
        """Return the address-phase frame of a read or a write.

        The write *data* is carried by this frame (it is placed in the low 32
        bits), so the byte-enable field stays zero.
        """
        if not 0 <= address < (1 << cls.BUS_ADDRESS_BITS):
            raise JtagAdapterError(
                f"bus address 0x{address:x} does not fit in "
                f"{cls.BUS_ADDRESS_BITS} bit(s)"
            )
        if not 0 <= data < (1 << cls.BUS_PAYLOAD_BITS):
            raise JtagAdapterError(
                f"bus data 0x{data:x} does not fit in "
                f"{cls.BUS_PAYLOAD_BITS} bit(s)"
            )
        payload = (address << cls.BUS_ADDR_OFFSET) | data
        return cls.frame(command, payload)

    @classmethod
    def action_frame(cls, data: int) -> int:
        """Return the second scan of a write: an all-zero no-op frame.

        The write data was already sent by :meth:`setup_frame`.
        """
        return cls.frame(0, 0)


# =============================================================================
# Command line interface
# =============================================================================

_EPILOG = """\
examples:
  # list the FTDI interfaces visible on the USB bus
  %(prog)s --list

  # read the IDCODE of an ARM JTAG-DP (IR = 0xE, 4-bit IR, 32-bit DR)
  %(prog)s --idcode
  %(prog)s -f 2e6 --idcode

  # shift 0x1234 into the 35-bit TDR selected by IR = 0xB (APACC)
  %(prog)s --ir 0xB --ir-bits 4 --dr 0x1234 --dr-bits 35

  # same, on the DAP TAP of a 3-TAP daisy chain described by a JSON file
  %(prog)s -c chain_example.json --ir 0xA --ir-bits 4 --dr-bits 35

  # 32-bit bus access through the bridge data register
  %(prog)s --bus-ir 0xC --bus-read 0x00000100
  %(prog)s --bus-ir 0xC --bus-format 55 --bus-write 0x00010000 0xDEADBEEF

Bus frames (the 2-bit command sits in the high bits of the data register):
  --bus-format 34: DR = {cmd[1:0], addr[31:0]}
    read : '10' + addr[31:0], then all zeros -> data in TDO[31:0]
    write: '11' + addr[31:0], then '01' + data[31:0]
  --bus-format 55: DR = {cmd[1:0], addr[16:0], be[3:0], data[31:0]}
    read : '10' + addr (be = data = 0), then all zeros -> data in TDO[31:0]
    write: '11' + addr + data in one frame, then an all-zero no-op frame

The value printed as TDO uses the JTAG bit order: bit 0 is the first bit
clocked out of TDO.
"""


def _auto_int(text: str) -> int:
    """Parse an integer, accepting decimal, 0x/0b/0o prefixes and bare hex."""
    try:
        return int(text, 0)
    except ValueError:
        try:
            return int(text, 16)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"invalid integer value: {text!r}"
            ) from exc


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="jtag_adapter.py",
        description="Shift data through an FTDI-attached JTAG TAP.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_EPILOG,
    )
    parser.add_argument("-u", "--url", default=None,
                        help="FTDI URL selector, e.g. ftdi://ftdi:2232h/1 "
                             "(default: first device found)")
    parser.add_argument("-f", "--frequency", type=float,
                        default=DEFAULT_FREQUENCY, metavar="HZ",
                        help="TCK frequency in Hz (default: 1e6)")
    parser.add_argument("-l", "--list", action="store_true",
                        help="list the available FTDI interfaces and exit")
    parser.add_argument("-c", "--chain", default=None, metavar="JSON",
                        help="JTAG daisy-chain description file")
    parser.add_argument("--trst", action="store_true",
                        help="use the nTRST line for TAP reset")
    parser.add_argument("--no-idle", dest="idle", action="store_false",
                        help="do not return to Run-Test/Idle after the shift")
    parser.add_argument("-q", "--quiet", action="store_true",
                        help="only print the shifted-out value")

    group = parser.add_argument_group("shift options")
    group.add_argument("--ir", type=_auto_int, default=None, metavar="VALUE",
                       help="instruction register value to load")
    group.add_argument("--ir-bits", type=int, default=None, metavar="BITS",
                       help="IR width in bits (required with --ir, unless "
                            "--chain describes the target TAP)")
    group.add_argument("--dr", type=_auto_int, default=None, metavar="VALUE",
                       help="data shifted into the selected TDR "
                            "(default: 0)")
    group.add_argument("--dr-bits", type=int, default=None, metavar="BITS",
                       help="DR width in bits (default: inferred from --dr; "
                            "pass it explicitly to read a wide register)")
    group.add_argument("--runtest", type=int, default=0, metavar="CYCLES",
                       help="clock CYCLES TCK periods in Run-Test/Idle first")

    idcode = parser.add_argument_group("IDCODE shortcut")
    idcode.add_argument("--idcode", action="store_true",
                        help="read the 32-bit IDCODE (see --idcode-ir...)")
    idcode.add_argument("--idcode-ir", type=_auto_int, default=ARM_IDCODE_IR,
                        metavar="VALUE", help="IDCODE instruction "
                                              "(default: 0xE)")
    idcode.add_argument("--idcode-ir-bits", type=int, default=ARM_IR_BITS,
                        metavar="BITS", help="IR width (default: 4)")
    idcode.add_argument("--idcode-dr-bits", type=int,
                        default=ARM_IDCODE_DR_BITS, metavar="BITS",
                        help="IDCODE width (default: 32)")

    bus = parser.add_argument_group("bus access (JTAG bus bridge)")
    bus.add_argument("--bus-ir", type=_auto_int, default=None, metavar="IR",
                     help="instruction selecting the bridge data register "
                          "(required with --bus-read/--bus-write)")
    bus.add_argument("--bus-ir-bits", type=int, default=None, metavar="BITS",
                     help="IR width of the bridge instruction")
    bus.add_argument("--bus-format", type=int, choices=(34, 55), default=34,
                     metavar="{34,55}",
                     help="bridge frame format: 34 = {cmd, addr}, "
                          "55 = {cmd, addr, be, data} (default: 34)")
    bus.add_argument("--bus-wait", type=int, default=0, metavar="CYCLES",
                     help="TCK cycles in Run-Test/Idle between the two scans "
                          "of a read (default: 0)")
    bus.add_argument("--bus-read", type=_auto_int, action="append",
                     default=None, metavar="ADDR",
                     help="read one 32-bit word (repeatable)")
    bus.add_argument("--bus-write", type=_auto_int, nargs=2, action="append",
                     default=None, metavar=("ADDR", "DATA"),
                     help="write one 32-bit word (repeatable)")
    return parser


def _format_value(label: str, value: int, bits: Optional[int]) -> str:
    """Format a shifted-out value with its bit count."""
    if bits:
        text = f"0x{format(value, f'0{(bits + 3) // 4}x')}"
        return f"{label}: {text}  ({value}, {bits} bit(s))"
    return f"{label}: 0x{value:x}  ({value})"


def _bus_requests(args: argparse.Namespace) -> bool:
    """True when the parsed command line asks for bus accesses."""
    return bool(args.bus_read or args.bus_write)


def main(argv: Optional[List[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.list:
        try:
            devices = list_devices()
        except JtagAdapterError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
        if not devices:
            print("no FTDI device found")
            return 1
        if not args.quiet:
            print(f"# backend: {describe_backend()}")
        print("Available FTDI interfaces:")
        for url, description in devices:
            print(f"  {url}   {description}")
        return 0

    if args.ir is None and not args.idcode and not _bus_requests(args):
        parser.error("nothing to do: give --idcode, --ir (with --ir-bits) or "
                     "--bus-read/--bus-write")
    if _bus_requests(args) and args.bus_ir is None:
        parser.error("--bus-read/--bus-write require --bus-ir")

    try:
        with JtagAdapter(args.url,
                         frequency=args.frequency,
                         trst=args.trst,
                         chain=args.chain,
                         idle_after=args.idle) as jtag:
            if not args.quiet:
                print(f"# backend: {describe_backend(jtag.backend)}")
                print(f"# device:  {jtag.url} @ {args.frequency:g} Hz")

            if args.runtest > 0:
                jtag.runtest(args.runtest)

            if args.bus_read or args.bus_write:
                bus_class = (CfgBusMaster if args.bus_format == 55
                             else BusMaster)
                bus = bus_class(jtag, args.bus_ir,
                                ir_bits=args.bus_ir_bits,
                                read_wait=args.bus_wait)
                if not args.quiet:
                    print(f"# bus:     {bus_class.__name__} "
                          f"({bus.dr_bits}-bit frame)")
                for address, data in args.bus_write or ():
                    bus.write32(address, data)
                    print(f"BUS: [0x{address:08x}] <= 0x{data:08x}")
                for address in args.bus_read or ():
                    data = bus.read32(address)
                    print(f"BUS: [0x{address:08x}] => 0x{data:08x}")
            elif args.idcode:
                value = jtag.read_idcode(ir=args.idcode_ir,
                                         ir_bits=args.idcode_ir_bits,
                                         dr_bits=args.idcode_dr_bits)
                print(_format_value("IDCODE", value, args.idcode_dr_bits))
            else:
                value = jtag.shift_ir_dr(args.ir,
                                         0 if args.dr is None else args.dr,
                                         ir_bits=args.ir_bits,
                                         dr_bits=args.dr_bits)
                print(_format_value("TDO", value, args.dr_bits))
    except JtagAdapterError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:  # pragma: no cover
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
