# aioc-slop

Alternative firmware for the AIOC (All-In-One-Cable, rev 1.0, STM32F302CBT6),
written in plain C on CMSIS headers (no HAL, no USB library).

USB functions (VID:PID 1209:0001, pid.codes test PID):

| Interface | What |
|---|---|
| if00 (CDC-ACM) | Configuration: terminal menu, `:` for a command line |
| if02 (CDC-ACM) | UART bridge to the radio (PA9/PA10), line coding from the host; DTR/RTS drive PTT |
| USB audio (UAC1) | Mono 16 bit, 8/11.025/16/22.05/24/32/44.1/48 kHz both ways, synchronous, clocked from USB SOF; ADC free-running at 1.125 MS/s with a rational moving-average decimator (ratio follows the host clock), DAC free-running at 480 kHz with a rational interpolator and first-order sigma-delta (16 -> 12 bit); both ratios follow the host clock (switchable) |
| DFU runtime | `dfu-util` detaches into the STM32 ROM bootloader |

## Build and flash

    cd fw
    make            # arm-none-eabi-gcc
    make flash      # dfu-util; works from aioc-slop or the ROM bootloader

First flash over the stock firmware: `dfu-util -d 1209:7388,0483:df11 -a 0 -s 0x08000000:leave -D build/aioc-slop.bin`.
If nothing responds, short the BOOT0 pads (J2) while plugging in to reach the ROM bootloader.

## Menu

    picocom /dev/serial/by-id/usb-aioc-slop_aioc-slop_<serial>-if00

Arrow keys only: Up/Down select, Right opens a page, runs an action or edits a
value (then Up/Down change it, Left/Right ends editing), Left goes back.
Also: Enter = Right, Esc = Left, +/- quick change, PgUp/PgDn big steps, S save, Ctrl-L redraw, `:` command line (`show`, `get`, `set`, `save`,
`defaults`, `status`, `dfu`, `reset`).

Pages: Audio (levels, mute, DC blocker, buffer/latency (default 1 ms),
oversampling, low-pass 3/4/8/16 kHz for TX and RX), PTT (source per output:
DTR / RTS / DTR&!RTS / RTS&!DTR, VOX, time-out, manual), VOX (threshold, hang),
Serial, System (LED, DFU, reset), Live status.

Default PTT1 = DTR and not RTS, so CHIRP (DTR+RTS) doesn't key the radio.
The resampler and sigma-delta ideas come from rhgndf's AIOC fork (branch
adc_dma, release v1.3.0-oversampling). RX noise
floor with nothing connected: -82.5 dBFS at 48 kHz and -88.5 dBFS at 8 kHz with
oversampling on (-71/-72 dBFS with point sampling). The core never sleeps
(no WFI): sleeping made the supply current step with the interrupt work,
which added 1 kHz lines and cost 5..8 dB.

## License

GPL-3.0 (see `LICENSE`), except for the vendored headers in `fw/cmsis/`:
ARM CMSIS-Core (`fw/cmsis/core/`, Apache-2.0) and the ST device headers
(`fw/cmsis/device/`, BSD-3-Clause), each with their license file.
The schematic `k1-aioc-schematic-rev1.0.pdf` is from the AIOC project
(https://github.com/skuep/AIOC) and is under that project's license.
