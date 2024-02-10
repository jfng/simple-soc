from amaranth import *
from amaranth.lib.fifo import SyncFIFOBuffered
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, flipped, connect

from amaranth_soc import csr
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
    class CKD(csr.Register, access="rw"):
        def __init__(self, *, reset):
            super().__init__({
                "divisor": csr.Field(csr.action.RW, unsigned(32), reset=reset),
            })

    class RXS(csr.Register, access="r"):
        rdy: csr.Field(csr.action.R, unsigned(1))

    class RXD(csr.Register, access="r"):
        data: csr.Field(csr.action.R, unsigned(8))
        err:  {"overflow": csr.Field(csr.action.R, unsigned(1)),
               "frame":    csr.Field(csr.action.R, unsigned(1))}

    class TXS(csr.Register, access="r"):
        rdy: csr.Field(csr.action.R, unsigned(1))

    class TXD(csr.Register, access="w"):
        data: csr.Field(csr.action.W, unsigned(8))

    def __init__(self, *, name, divisor, rx_fifo_depth=16, tx_fifo_depth=16):
        super().__init__({"bus": In(csr.Signature(addr_width=8, data_width=8))})

        regs = csr.Builder(addr_width=self.bus.addr_width,
                           data_width=self.bus.data_width,
                           name=name)

        self._ckd = regs.add("CKD", self.CKD(reset=divisor), offset=0x00)
        self._rxs = regs.add("RXS", self.RXS(), offset=0x04)
        self._rxd = regs.add("RXD", self.RXD(), offset=0x08)
        self._txs = regs.add("TXS", self.TXS(), offset=0x0c)
        self._txd = regs.add("TXD", self.TXD(), offset=0x10)

        self.bus.memory_map = regs.as_memory_map()

        self._bridge  = csr.Bridge(self.bus.memory_map)
        self._rx_fifo = SyncFIFOBuffered(width=10, depth=rx_fifo_depth)
        self._tx_fifo = SyncFIFOBuffered(width=8,  depth=tx_fifo_depth)
        self._phy     = AsyncSerial_Blackbox(divisor=divisor, divisor_bits=32)

    def elaborate(self, platform):
        m = Module()

        m.submodules.bridge  = self._bridge
        m.submodules.rx_fifo = self._rx_fifo
        m.submodules.tx_fifo = self._tx_fifo
        m.submodules.phy     = self._phy

        connect(m, flipped(self), self._bridge)

        m.d.comb += [
            # CKD / phy

            self._phy.divisor.eq(self._ckd.f.divisor.data),

            # phy.rx / rx_fifo

            self._rx_fifo.w_data.eq(Cat(self._phy.rx.data,
                                        self._phy.rx.err.overflow,
                                        self._phy.rx.err.frame)),
            self._rx_fifo.w_en.eq(self._phy.rx.rdy),
            self._phy.rx.ack.eq(self._rx_fifo.w_rdy),

            # rx_fifo / RXS, RXD

            self._rxs.f.rdy.r_data.eq(self._rx_fifo.r_rdy),

            Cat(rxd_field.r_data for _, rxd_field in self._rxd).eq(self._rx_fifo.r_data),
            self._rx_fifo.r_en.eq(self._rxd.f.data.r_stb),

            # TXS, TXD / tx_fifo

            self._txs.f.rdy.r_data.eq(self._tx_fifo.w_rdy),

            self._tx_fifo.w_data.eq(self._txd.f.data.w_data),
            self._tx_fifo.w_en.eq(self._txd.f.data.w_stb),

            # tx_fifo / phy.tx

            self._phy.tx.data.eq(self._tx_fifo.r_data),
            self._phy.tx.ack.eq(self._tx_fifo.r_rdy),
            self._tx_fifo.r_en.eq(self._phy.tx.rdy),
        ]

        return m
