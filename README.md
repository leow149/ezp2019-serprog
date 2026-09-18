# EZP2019+ serprog firmware

Custom **serprog** (flashrom) firmware for the FNIRSI/zhifengsoft-style
**EZP2019+ USB EEPROM/SPI programmer** (WCH CH552G), hardened and cleaned up.

- Full `flashrom` support: `flashrom -p serprog:dev=/dev/ttyACM0:4000000,spispeed=4000000`
- **~205–216 KiB/s** sustained reads (4 MB in ~19–20 s), byte-stable
- Self-healing USB: no permanent wedges on aborted (`SIGKILL`-ed) transfers
- Correct activity LED (P1.1, active-high): **off when idle, on during commands**
- Purpose-built fixes over the upstream `EZP2020_CH552x-FW`, including the big
  one: the SDK's `CfgFsys()` never entered SAFE MODE, so the chip idled at
  ~3–4 MHz instead of 24 MHz — which made SPI ~6× slower and produced the
  infamous "56 KiB/s wall" reported elsewhere.

## Supported hardware

| | |
|---|---|
| MCU | WCH CH552G (chip id `0x5211`) |
| USB | Full-speed CDC-ACM (`1a86:5722`, product "EZP2019+ serprog") |
| SPI | CS=P1.4, MOSI=P1.5, MISO=P1.6, SCK=P1.7 (verified against the OEM image) |
| LED | Green activity on **P1.1, active-high**; P3.4 is only the USB-SOF "connected" pin in stock |
| Socket VCC | hardware auto-select; no firmware GPIO (S_CMD_S_VCC is a no-op) |
| ISP entry | hold P3.6 / USB-D+ high at power-on (`4348:55e0` bootloader) |

## Build

Requires [SDCC](https://sdcc.sourceforge.net/) and `objcopy` (binutils):

```sh
make                          # -> build/ezp2019-serprog.ihx, ezp2019-serprog.bin
```

(Lint-clean: builds with `--std-sdcc11 --Werror`.)

## Flash

1. Enter the CH552 ISP bootloader: **short the BOOT pads (D+/P3.6 → 3.3 V) with
   a ~1 kΩ resistor** while plugging in; the device appears as `4348:55e0`.
2. Flash with [wchisp](https://github.com/ch32-rs/wchisp):

```sh
wchisp flash ezp2019-serprog.bin
```

3. Unplug, remove the bridge, replug → `/dev/ttyACM0`.

> Keep a stock image (`PGMNAME`/descriptors differ) backed up first —
> `wchisp` cannot dump the original 14 KiB image.

## Usage

```sh
# probe / read / write
flashrom -p serprog:dev=/dev/ttyACM0:4000000,spispeed=4000000 -c W25Q32FV -r dump.bin
```

After an aborted run the device self-heals in a few seconds (host USB
autosuspend can stretch that); a retry wrapper `ezp-wait` (in the release
assets / this repo's tools) waits for readiness before re-running flashrom.

## Feature notes

- serprog commands: NOP, Q_IFACE, Q_CMDMAP, Q_PGMNAME, Q_SERBUF, Q_BUSTYPE,
  Q_WRNMAXLEN, Q_RDNMAXLEN, SYNCNOP, S_BUSTYPE, O_SPIOP, S_SPI_FREQ,
  plus custom `S_CMD_S_VCC` (0x1A, no-op on this board).
- SPI clock capped at **4 MHz** (silicon max is 12 MHz, but 12 MHz is marginal
  on the socket wiring), negotiated per-command via `spispeed=`.
- A hardware double-buffered `O_SPIOP` variant was built and measured — it was
  slower and had framing errors, so the pipelined single-buffer path is the
  shipping one.

## Performance summary (measured)

| SPI | read rate |
|---|---|
| 1 MHz | ~123 KiB/s |
| 2 MHz | ~164 KiB/s |
| 4 MHz | ~197–216 KiB/s (USB-bound) |

The limit is the full-speed USB CDC feed, not SPI or the flash.

## Credits

- [@ieiao](https://github.com/ieiao) — [`ieiao/ch554_sdcc`](https://github.com/ieiao/ch554_sdcc), the CH55x SDCC SDK and the original serprog USB example.
- [@iCE-HACK3R](https://github.com/iCE-HACK3R) — [`iCE-HACK3R/EZP2020_CH552x-FW`](https://github.com/iCE-HACK3R/EZP2020_CH552x-FW), the CH552x EZP-programmer adaptation this work is based on.
- [WCH](https://www.wch.cn) — the CH55x SDK/register headers and the [CH552 datasheet](https://www.wch.cn/downloads/CH552DS1_PDF.html).
- See `NOTICE` for the full attribution and license scoping.

## License

MIT — see [LICENSE](LICENSE).