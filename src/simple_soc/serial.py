from amaranth import *
from amaranth.utils import bits_for
from amaranth.lib.fifo import SyncFIFOBuffered
from amaranth.lib import data, wiring
from amaranth.lib.wiring import In, Out, flipped, connect

from amaranth_soc import csr
from amaranth_soc.csr import Field, action

from amaranth_stdio.serial import *


__all__ = ["AsyncSerialRX_Blackbox", "AsyncSerialTX_Blackbox", "AsyncSerial_Blackbox",
           "AsyncSerialPeripheral"]


class AsyncSerialRX_Blackbox(wiring.Component):
    def __init__(self, *, divisor, divisor_bits=None, data_bits=8, parity="none", parent=None):
        if parent is not None and not isinstance(parent, AsyncSerial_Blackbox):
            raise TypeError(f"Parent must be an instance of AsyncSerial_Blackbox, not {parent!r}")
        self._parent = parent
        super().__init__(AsyncSerialRX.Signature(divisor=divisor, divisor_bits=divisor_bits,
                                                 data_bits=data_bits, parity=parity))

    def elaborate(self, platform):
        return Instance("serial_rx",
            p_ID           = hex(id(self._parent) if self._parent else id(self)),
            p_DATA_BITS    = self.signature.data_bits,
            i_clk          = ClockSignal("sync"),
            o_data         = self.data,
            o_err_overflow = self.err.overflow,
            o_err_frame    = self.err.frame,
            o_err_parity   = self.err.parity,
            o_rdy          = self.rdy,
            i_ack          = self.ack,
        )


class AsyncSerialTX_Blackbox(wiring.Component):
    def __init__(self, *, divisor, divisor_bits=None, data_bits=8, parity="none", parent=None):
        if parent is not None and not isinstance(parent, AsyncSerial_Blackbox):
            raise TypeError(f"Parent must be an instance of AsyncSerial_Blackbox, not {parent!r}")
        self._parent = parent
        super().__init__(AsyncSerialTX.Signature(divisor=divisor, divisor_bits=divisor_bits,
                                                 data_bits=data_bits, parity=parity))

    def elaborate(self, platform):
        return Instance("serial_tx",
            p_ID        = hex(id(self._parent) if self._parent else id(self)),
            p_DATA_BITS = self.signature.data_bits,
            i_clk       = ClockSignal("sync"),
            i_data      = self.data,
            o_rdy       = self.rdy,
            i_ack       = self.ack,
        )


class AsyncSerial_Blackbox(wiring.Component):
    def __init__(self, *, divisor, divisor_bits=None, data_bits=8, parity="none", parent=None):
        super().__init__(AsyncSerial.Signature(divisor=divisor, divisor_bits=divisor_bits,
                                               data_bits=data_bits, parity=parity))

    def elaborate(self, platform):
        m = Module()

        rx = AsyncSerialRX_Blackbox(divisor=self.signature.divisor,
                                    divisor_bits=self.signature.divisor_bits,
                                    data_bits=self.signature.data_bits,
                                    parity=self.signature.parity,
                                    parent=self)
        tx = AsyncSerialTX_Blackbox(divisor=self.signature.divisor,
                                    divisor_bits=self.signature.divisor_bits,
                                    data_bits=self.signature.data_bits,
                                    parity=self.signature.parity,
                                    parent=self)

        m.submodules.rx = rx
        m.submodules.tx = tx

        m.d.comb += [
            self.rx.divisor.eq(self.divisor),
            self.tx.divisor.eq(self.divisor),
        ]

        connect(m, flipped(self.rx), rx),
        connect(m, flipped(self.tx), tx),

        return m


class AsyncSerialPeripheral(wiring.Component):
    class Ctrl(csr.Register, access="rw"):
        def __init__(self, divisor, divisor_bits=None):
            if not isinstance(divisor, int) or divisor <= 0:
                raise ValueError(f"Divisor reset must be a positive integer, not {divisor!r}")
            if divisor_bits is None:
                divisor_bits = bits_for(divisor)
            if not isinstance(divisor_bits, int) or divisor_bits <= 0:
                raise ValueError(f"Divisor width must be a positive integer, not {divisor_bits!r}")
            super().__init__({
                "divisor": Field(action.RW, unsigned(divisor_bits), reset=divisor),
            })

    class RxInfo(csr.Register, access="r"):
        def __init__(self):
            super().__init__({
                "rdy": csr.Field(csr.action.R, unsigned(1)),
                "err": {
                    "overflow": Field(csr.action.R, unsigned(1)),
                    "frame":    Field(csr.action.R, unsigned(1)),
                    "parity":   Field(csr.action.R, unsigned(1)),
                },
            })

    class RxData(csr.Register, access="r"):
        def __init__(self, data_bits):
            super().__init__({
                "data": Field(action.R, unsigned(data_bits)),
            })

    class TxInfo(csr.Register, access="r"):
        def __init__(self):
            super().__init__({
                "rdy": Field(action.R, unsigned(1)),
            })

    class TxData(csr.Register, access="w"):
        def __init__(self, data_bits):
            super().__init__({
                "data": Field(action.W, unsigned(data_bits)),
            })

    def __init__(self, *, name, divisor, divisor_bits=None, data_bits=8, parity="none",
                 rx_depth=16, tx_depth=16):
        self._phy = AsyncSerial_Blackbox(divisor=divisor, divisor_bits=divisor_bits,
                                         data_bits=data_bits, parity=parity)
        self._rx_fifo = SyncFIFOBuffered(width=data_bits + len(self._phy.rx.err.as_value()),
                                         depth=rx_depth)
        self._tx_fifo = SyncFIFOBuffered(width=data_bits, depth=tx_depth)

        register_map = csr.RegisterMap()
        rx_cluster   = csr.RegisterMap()
        tx_cluster   = csr.RegisterMap()

        self._ctrl    = register_map.add_register(self.Ctrl(divisor, divisor_bits), name="ctrl")
        self._rx_info = rx_cluster.add_register(self.RxInfo(), name="info")
        self._rx_data = rx_cluster.add_register(self.RxData(data_bits), name="data")
        self._tx_info = tx_cluster.add_register(self.TxInfo(), name="info")
        self._tx_data = tx_cluster.add_register(self.TxData(data_bits), name="data")

        register_map.add_cluster(rx_cluster, name="rx")
        register_map.add_cluster(tx_cluster, name="tx")

        self._bridge = csr.Bridge(register_map, addr_width=8, data_width=8, name=name)
        super().__init__(self._bridge.signature)
        self.bus.memory_map = self._bridge.bus.memory_map

    def elaborate(self, platform):
        m = Module()

        m.submodules.bridge  = self._bridge
        m.submodules.rx_fifo = self._rx_fifo
        m.submodules.tx_fifo = self._tx_fifo
        m.submodules.phy     = self._phy

        connect(m, flipped(self), self._bridge)

        rx_fifo_w_data = Signal(data.StructLayout({
            "data": self._phy.rx.data.shape(),
            "err":  self._phy.rx.err .shape(),
        }))
        rx_fifo_r_data = Signal.like(rx_fifo_w_data)

        m.d.comb += [
            self._phy.divisor.eq(self._ctrl.f.divisor.data),

            rx_fifo_w_data.data.eq(self._phy.rx.data),
            rx_fifo_w_data.err .eq(self._phy.rx.err),

            self._rx_fifo.w_data.eq(rx_fifo_w_data),
            self._rx_fifo.w_en  .eq(self._phy.rx.rdy),
            self._phy.rx.ack.eq(self._rx_fifo.w_rdy),

            rx_fifo_r_data.eq(self._rx_fifo.r_data),

            self._rx_info.f.rdy.r_data.eq(self._rx_fifo.r_rdy),
            self._rx_info.f.err.overflow.r_data.eq(rx_fifo_r_data.err.overflow),
            self._rx_info.f.err.frame   .r_data.eq(rx_fifo_r_data.err.frame),
            self._rx_info.f.err.parity  .r_data.eq(rx_fifo_r_data.err.parity),

            self._rx_data.f.data.r_data.eq(rx_fifo_r_data.data),
            self._rx_fifo.r_en.eq(self._rx_data.element.r_stb),

            self._tx_fifo.w_data.eq(self._tx_data.f.data.w_data),
            self._tx_fifo.w_en  .eq(self._tx_data.element.w_stb),
            self._tx_info.f.rdy.r_data.eq(self._tx_fifo.w_rdy),

            self._phy.tx.data.eq(self._tx_fifo.r_data),
            self._phy.tx.ack .eq(self._tx_fifo.r_rdy),
            self._tx_fifo.r_en.eq(self._phy.tx.rdy),
        ]

        return m
