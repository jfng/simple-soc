CXX = pdm run python -m ziglang c++ -target x86_64-linux-gnu
CC  = $(CXX)

yosys_files != pdm run python -c \
	"from importlib import resources; print(resources.files('yowasp_yosys'))"

CPPFLAGS = -Iinclude -I$(yosys_files)/share/include -include simple_soc.h -MMD
CXXFLAGS = -std=c++14 -Wall -O3 -mtune=native -flto=thin
LDFLAGS  = -flto=thin
LDLIBS   =

SRC = \
      serial_driver.cc \
      simple_soc.cc \
      simulate.cc \

OBJS = $(SRC:.cc=.o)
DEPS = $(OBJS:.o=.d)

.PHONY: all ./_hello/hello.bin clean

all: simulate

-include DEPS

../_hello/hello.bin:
	make -C ../_hello hello.bin

simple_soc.il: ../_hello/hello.bin
	pdm run python -m simple_soc.cli --rom-init=../_hello/hello.bin generate simple_soc.il

simple_soc.cc simple_soc.h: simple_soc.ys simple_soc.il serial_blackbox.v
	pdm run yowasp-yosys -s simple_soc.ys -q

simulate.o: CPPFLAGS += -DCXXRTL_TOP='cxxrtl_design::p_top'

$(OBJS): simple_soc.h

simulate: $(OBJS)

clean:
	make -C ../_hello clean
	rm -f simple_soc.il
	rm -f simple_soc.cc simple_soc.h
	rm -f $(OBJS) $(DEPS)
	rm -f simulate
