from amaranth import *
from amaranth.utils import bits_for
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out

from amaranth_soc import wishbone
from amaranth_soc.memory import MemoryMap


__all__ = ["WishboneMemory"]


class WishboneMemory(wiring.Component):
    def __init__(self, *, name, size, data_width=32, granularity=8, writable=True, init=None):
        if not isinstance(name, str) or not name:
            raise ValueError("Name must be non-empty string, not {name!r}")
        if not isinstance(size, int) or size <= 0 or size & size-1:
            raise ValueError(f"Size must be an integer power of two, not {size!r}")
        if data_width not in (8, 16, 32, 64):
            raise ValueError(f"Data width must be 8, 16, 32 or 64, not {data_width!r}")
        if granularity not in (8, 16, 32, 64):
            raise ValueError(f"Granularity must be 8, 16, 32 or 64, not {granularity!r}")
        if size * granularity < data_width:
            raise ValueError(f"Size {size!r} * granularity {granularity!r} must be greater than "
                             f"or equal to data width {data_width!r}")
        self._name        = name
        self._size        = size
        self._data_width  = data_width
        self._granularity = granularity
        self._writable    = bool(writable)
        self._storage     = Memory(depth=(size * granularity) // data_width,
                                   width=data_width, init=init)
        super().__init__()

    @property
    def name(self):
        return self._name

    @property
    def size(self):
        return self._size

    @property
    def data_width(self):
        return self._data_width

    @property
    def granularity(self):
        return self._granularity

    @property
    def writable(self):
        return self._writable

    @property
    def init(self):
        return self._storage.init

    @property
    def signature(self):
        bus_sig = wishbone.Signature(addr_width=bits_for(self._storage.depth),
                                     data_width=self.data_width,
                                     granularity=self.granularity,
                                     features=("cti", "bte"))
        bus_map = MemoryMap(addr_width=bits_for(self.size), data_width=self.granularity,
                            name=self.name)
        bus_map.add_resource(self._storage, name="storage", size=self.size)
        bus_sig.memory_map = bus_map
        return wiring.Signature({"bus": Out(bus_sig)})

    def elaborate(self, platform):
        m = Module()

        addr_curr = Signal.like(self.bus.adr)
        addr_next = Signal.like(self.bus.adr)
        addr_incr = Signal.like(self.bus.adr)

        m.d.comb += addr_incr.eq(addr_curr + 1)

        with m.If(self.bus.cti == wishbone.CycleType.INCR_BURST):
            with m.Switch(self.bus.bte):
                with m.Case(wishbone.BurstTypeExt.WRAP_4):
                    m.d.comb += addr_next.eq(Cat(addr_incr[:2], addr_curr[2:]))
                with m.Case(wishbone.BurstTypeExt.WRAP_8):
                    m.d.comb += addr_next.eq(Cat(addr_incr[:3], addr_curr[3:]))
                with m.Case(wishbone.BurstTypeExt.WRAP_16):
                    m.d.comb += addr_next.eq(Cat(addr_incr[:4], addr_curr[4:]))
                with m.Default():
                    m.d.comb += addr_next.eq(addr_incr)
        with m.Else():
            m.d.comb += addr_next.eq(addr_curr)

        m.submodules.memory = self._storage

        mem_rp = self._storage.read_port()
        m.d.comb += [
            mem_rp.addr.eq(addr_curr),
            self.bus.dat_r.eq(mem_rp.data),
        ]

        if self.writable:
            mem_wp = self._storage.write_port(granularity=self.granularity)
            m.d.comb += [
                mem_wp.addr.eq(addr_curr),
                mem_wp.data.eq(self.bus.dat_w),
            ]

        with m.FSM():
            with m.State("IDLE"):
                m.d.sync += addr_curr.eq(self.bus.adr)
                with m.If(self.bus.cyc & self.bus.stb):
                    m.next = "CYCLE"

            with m.State("CYCLE"):
                m.d.sync += addr_curr.eq(addr_next)
                with m.If(self.bus.cyc & self.bus.stb):
                    if self.writable:
                        m.d.comb += mem_wp.en.eq(Mux(self.bus.we, self.bus.sel, 0))
                    m.d.sync += self.bus.ack.eq(1)
                with m.Else():
                    m.d.sync += self.bus.ack.eq(0)
                    m.next = "IDLE"

        return m
