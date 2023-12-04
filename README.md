```bash
% pdm install
% cd src/_cxxrtl
% make
% ./simulate -h
```

Simulate for 1000 cycles and dump a waveform:

```bash
% ./simulate -c 1000 -t dump.vcd
Assigning 'serial phy rx' to /dev/pts/30
Assigning 'serial phy tx' to /dev/pts/30
Press Enter to start simulation...
Running.
Press Ctrl-C to exit simulation.
Exiting.
```

To see the serial output, open a terminal (e.g. `picocom`) on `/dev/pts/XX` before pressing Enter.
