# RadiaCLI

Command-line interface for Radiacode 102.  
Brother of https://github.com/Liptoon/radmon in C++.

## Requirements

* libsimpleble
* dbus-1
* g++

## Build

```bash
make
```

### Live Mode
```bash
./RadiaCLI -m <MAC> -l
```

### Run-once Mode
```bash
./RadiaCLI -m <MAC> -r 1 -i 3
```

## Options

* -m, --mac : Device MAC address.
* -l, --live : Continuous real-time display.
* -r N, --runonce N : Print N measurements and exit.
* -i S, --interval S : Averaging interval in seconds (default 3).

## Output Format

```text
t=<ISO8601> HT=<µSv/h µR/h> CPS=<> CPM=<> Err=±<%> Temp=<°C> Bat=<%>
```