# qibixx MDB-USB ASCII Protocol — Reference

Notes from qibixx's public docs (docs.qibixx.com/mdb-products/mdb-toolchest,
mdb-usb-interface API pages), gathered while building `main/usb-monitor.c`.
Kept here so future protocol work (liveness handshake, write commands,
Toolchest interop) doesn't need to be re-researched from scratch.

Not a copy of qibixx's firmware or SDK — this project does **not** ship or
depend on any qibixx code. This is our own read-only ASCII monitor
(`main/usb-monitor.c`, CDC1) *inspired by* their command shape, kept
deliberately incompatible with their write commands (no `D,REQ`/`D,END`)
so it can't be mistaken for a controllable cashless-master channel.

## Transport

- USB CDC-ACM (virtual COM port), **115200 8N1**.
- Line-based ASCII, commands and responses terminated with `\n` (`\r\n`
  tolerated on input).
- Real qibixx MDB-USB Interface hardware enumerates as **VID `16d0` PID
  `0bd7`**. Our board deliberately does NOT spoof this VID:PID — it uses
  Espressif's own TinyUSB descriptor (`303a:4002` composite), so it never
  gets misidentified as genuine qibixx hardware over USB enumeration.

## Command groups (their protocol)

### General
| Cmd | Response | Meaning |
|---|---|---|
| `V` | `v,<fw_version>,<serial>` | firmware version + device serial |
| `H` | `h,<hw_rev>,<capflags>` | hardware revision + capability flags |
| `F,RESET` | — | reset device |
| `F,REVERT` | — | revert to previous firmware slot |
| `F,UPDATE` | — | enter firmware update mode |
| `L` | — | toggle/query onboard LED |
| `W` | — | watchdog-related |

### Cashless Master mode (device emulates a cashless peripheral toward the
VMC — this is the mode relevant to a controller/target bridge like ours)
| Cmd | Meaning |
|---|---|
| `D,0` / `D,1` / `D,2` | select cashless device mode/slot |
| `D,READER,1` | enable card reader |
| `D,REQ,<amount>,<product>` | request a vend for `<amount>` of `<product>` |
| `D,END[,<id>]` | end current session, optional transaction id |
| `D,STATUS` → `d,STATUS,...` | query session/vend status |

### Generic Master mode (raw MDB passthrough — host app is responsible for
polling; qibixx device is just an MDB↔USB pipe, no protocol logic)
| Cmd | Meaning |
|---|---|
| `M,...` | raw MDB frame in/out; host must poll bus itself |

## Known gap: "MDB-USB alive" liveness check

qibixx's official **MDB Toolchest** GUI (`~/MDB-Toolchest.jar` in this
environment) performs a proprietary handshake — observed in its log as
`"Checking if MDB-USB is alive!"` — before it will treat an attached
serial device as real qibixx hardware. It is **not** one of the documented
`V`/`H`/`D,`/`M,` commands above; exact bytes are unknown (not published in
the docs we found). Our firmware answers `V`/`H`/status queries correctly
but does not implement this handshake, so Toolchest does not fully
recognize the board as genuine MDB-USB. Two ways forward if this matters
later:
1. Sniff Toolchest↔real-dongle traffic (needs actual qibixx hardware) to
   reverse the handshake bytes, or
2. Ignore Toolchest entirely and drive the board with our own tooling —
   `web/dashboard.html` (Web Serial, zero install) or any plain serial
   terminal on the CDC1 monitor port, both already verified working
   end-to-end on real hardware.

## Our port (`main/usb-monitor.c`, CDC1) — for comparison

Read-only monitor, not a qibixx clone. Commands: `V H S C B L U ?`.
See file header comment for the full table — deliberately uses lowercase
single-letter tags disjoint from qibixx's `D,`/`M,` namespace.
