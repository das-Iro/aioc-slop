# aioc-slop

Alternative firmware for the AIOC (All-In-One-Cable, rev 1.0, STM32F302CBT6),
written in plain C on CMSIS headers (no HAL, no USB library).

USB functions (VID:PID 1209:0001, pid.codes test PID):

| Interface | What |
|---|---|
| if00 (CDC-ACM) | Configuration: terminal menu, `:` for a command line |
| if02 (CDC-ACM) | UART bridge to the radio (PA9/PA10), line coding from the host; DTR/RTS drive PTT |
| USB audio (UAC1) | Mono 16 bit, 8/11.025/16/22.05/24/32/44.1/48 kHz both ways, synchronous, clocked from USB SOF; ADC free-running at 2.4 MS/s with a rational moving-average decimator (ratio follows the host clock), DAC at ~192 kHz with sample-and-hold (switchable) |
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

Pages: Audio (levels, mute, DC blocker, buffer/latency), PTT (source per output:
DTR / RTS / DTR&!RTS / RTS&!DTR, VOX, time-out, manual), VOX (threshold, hang),
Serial, System (LED, sleep on/off, DFU, reset), Live status.

Default PTT1 = DTR and not RTS, so CHIRP (DTR+RTS) doesn't key the radio.
`sleep off` replaces WFI with a busy idle loop to test supply-ripple noise.
The decimator idea comes from rhgndf's AIOC fork (audio_dsp.c). RX noise
floor with nothing connected: -84 dBFS at 48 kHz and -87 dBFS at 8 kHz with
oversampling on and sleep off (-71/-72 dBFS with point sampling). With sleep
on, CPU current steps add 1 kHz lines and cost 5..8 dB.
