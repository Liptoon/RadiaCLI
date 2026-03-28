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
Live records automatically refreshed.

### Run-once Mode
```bash
./RadiaCLI -m <MAC> -r 1 -i 3
```
Prints given number of lines and exits

### Quiet Mode
```bash
./RadiaCLI -m <MAC> -q
```
Prints only data line and exits.

## Options

* -m, --mac : Device MAC address. Without this flag RadiaCode device will be searched automatically.
* -l, --live : Continuous real-time display.
* -r N, --runonce N : Print N measurements and exit.
* -i S, --interval S : Averaging interval in seconds (default 3).
* -q --quiet: Prints only data line and exits

## Output Format

```text
[HH:MM:SS] HT=µSv/h µR/h HTErr=±% CPS= CPM= CPSErr=±%
```