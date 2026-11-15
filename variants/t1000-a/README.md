# Seeed SenseCAP T1000-A (Goodix GR5515)

Unofficial MeshCore port to the **T1000-A** (not the nRF52-based T1000-E, which is supported upstream).
The T1000-A uses a Goodix GR5515 BLE SoC, a Semtech LR1110 and an Airoha AG3335 GNSS.

This port lives in a fork ([Artemia76/MeshCore](https://github.com/Artemia76/MeshCore), branch
`feature/t1000a-gr5515`) and is **not supported by the MeshCore core team**. It builds on a community Arduino core,
[platform-gr551x](https://github.com/Artemia76/platform-gr551x), which only implements what MeshCore needs.

## Status

| Feature | State |
|---|---|
| LoRa (LR1110) TX/RX, mesh | works, interoperates with stock MeshCore nodes |
| Companion over BLE (MeshCore apps), PIN pairing | works (default PIN `123456`) |
| GNSS (AG3335), battery voltage, buzzer, status LED, button | works |
| Charge indication on the status LED, external power detection | works (see below) |
| Power off (long press 3 s), power on (hold 2 s) | works (see below) |
| Repeater / room server builds | not provided yet |
| Light sleep | partial (CPU idles between loop passes) |

Firmware: `t1000a_companion_radio_ble`.

## Status LED

The T1000-A has a single LED (AON3), dimmable through PWM1.

| State | LED |
|---|---|
| On battery | short blink every 4 s (longer when messages are waiting) |
| Charging (cable on MSIO0, charger status AON4 low) | slow breathing, 4 s period |
| Charged (cable on MSIO0, AON4 high) | steady on |

On the charger, MeshCore activity is shown in negative: the LED goes dark for 20 ms at the top of each 4 s cycle,
200 ms when messages are waiting. While the cable is plugged in the CPU no longer sleeps (smooth animation).

## Power off

Hold the button 3 s (more than 8 s after start-up) to switch off:
- GNSS (with its backup supply), the battery measurement divider and the buzzer are cut, and the LR1110 goes to
  cold sleep;
- the GR5515 enters ultra deep sleep, woken by the button or when charging starts.

To switch on, hold the button about 3 s. Every wake goes through the Seeed bootloader, which blinks the LED twice;
the box then powers on (two more blinks) only if the button is still held, so a shorter press puts it straight
back to sleep. When charging starts while off, the LED shows the charge
(breathing, then steady) without starting MeshCore; hold the button 2 s to switch on. A box that is off and
already full does not wake when plugged in (the charger status does not change). Holding 6 s still restarts into
the DFU bootloader. After a power off, the console's `power` command shows the battery voltage when it went off
and when it came back, and how many wakes went straight back to sleep meanwhile.

## Flashing

The T1000-A has no USB data port: flashing is done over **Bluetooth**, through the stock Seeed/Goodix bootloader,
which is never overwritten. You need a Linux PC with BlueZ, Python 3 and the `bleak` package, plus
[`t1000_ble.py`](https://github.com/Artemia76/platform-gr551x/blob/v0.1.0/tools/t1000_ble.py).

1. Get `firmware_dfu.bin` from the fork's releases, or build it:
   `pio run -e t1000a_companion_radio_ble` → `.pio/build/t1000a_companion_radio_ble/firmware_dfu.bin`.
2. Put the device in the bootloader (it advertises as `SENSECAP_GR_DFU_xxxxxx`):
   - from the stock Seeed firmware: `t1000_ble.py flash firmware_dfu.bin --enter-dfu <serial>-T1000`
     (sends `AT+DFU_ENTER`; press the button long to wake the Seeed firmware's BLE first);
   - from MeshCore: CLI command `start ota`, or hold the button **6 s**;
   - last resort: unplug the magnetic cable, hold the button, plug the cable back while holding.
3. `t1000_ble.py flash firmware_dfu.bin --name SENSECAP_GR_DFU -t 600`

Safety net: after 3 consecutive short boots (crash or watchdog before 2 min of uptime) the firmware restarts into
the bootloader by itself.

## Development console and power log

The firmware exposes a text console on the Goodix UART service (not advertised: connect to the companion's BLE
name), e.g. `t1000_ble.py console --name <name>`. Besides `bat`, `gps`, `beep`, `blelog` and `dfu`:

- `power`: time spent in each state since the last reset — CPU idle, BLE connected / advertising, LoRa transmit
  time and packets, GNSS on — plus the battery voltage.
- `power log`: one CSV line per 10 min interval over the last 24 h (`t_s,dt_s,bat_mv,idle_pm,wakes,conn_s,adv_s,
  gps_s,tx_ms,rx,tx,flags,msio0_mv`). `flags` gives the state when the battery was read: `0x01` GNSS on,
  `0x02` BLE connected, `0x04` reading postponed until a LoRa transmission ended, `0x08`/`0x10` AON4/AON5 (charge
  state candidates), `0x20` GPIO14 (measurement divider), `0x40` buzzer amplifier enabled.
- `power reset`: clears the counters and the log.
- `chg`: raw levels of the charge state candidates (AON4, AON5, MSIO0), to compare with the cable plugged in and out.

Capture from a PC: `t1000_ble.py console --name <name> --cmd power --cmd "power log" > capture.txt`. The log only
holds times: currents per state come from the datasheets, so the power budget is computed off-device.

## Going back to the Seeed firmware

Flash Seeed's official T1000-A application image (`tracker-t1000-ab-app-ble-*.bin`) the same way (step 2 then 3).
The bootloader accepts it unchanged.
