# Metronome II

A hardware metronome built on an ESP32-S3 SuperMini: six COB LED filaments chase the
beat, a passive buzzer plays a kick-under-hihats tock voice, a TM1637 4-digit display
shows tempo and time signature, and a KY-040 encoder sets both. It broadcasts its own
WiFi access point and serves a phone control page whose default view is a swinging
pendulum, phase-locked to the device's real beat clock.

No home network, no cloud, no app. Join the AP, open a browser, play.

## Build photos

Coming soon — breadboard bring-up is complete, sculpture assembly is next.

## Wiring

![Wiring diagram](wiring-metronome-2.png)

The diagram regenerates from `wiring-metronome-2.json` (spec for a pictorial-diagram
generator; kept beside the firmware as the wiring contract).

| Subsystem | Pins | Notes |
|---|---|---|
| COB filaments 1-6 | IO7, IO6, IO5, IO4, IO2, IO1 | each via S8050 NPN + 820R base brick, 5V rail |
| TM1637 display | CLK=IO9, DIO=IO8 | 2-wire bitbang, driver chip multiplexes |
| KY-040 encoder | CLK=IO10, DT=IO11, SW=IO12 | internal pullups, polled — no interrupts |
| Passive buzzer (TMB12A05) | IO13 | 1k into NPN low-side, buzzer to 5V, flyback diode across coil |

GPIO3 is skipped on purpose (strapping pin).

## Phone remote

The S3 runs a standalone WPA2 AP (`Metronome` / `metronome`). `http://192.168.4.1`
serves a single-page UI:

- **Pendulum view (default)** — a full-screen metronome arm whose swing extremes land
  on the actual ticks. The API reports time-to-next-beat and period; the client keeps a
  continuous phase and nudges it by the smallest modular correction each poll, so the
  arm alternates sides correctly and never snaps. The bob rides the arm by tempo like a
  real metronome weight. During tempo adjustment the arm eases to center and relaunches
  on the restart downbeat.
- **Adjust view** — ±1/±10 buttons, type-in tempo, time-signature cycle.

Endpoints: `GET /api/state`, `/api/bpm?d=N|set=N`, `/api/sig`. All controls route
through the same code paths as the physical knob.

## Firmware architecture

FreeRTOS tasks: beat engine (prio 6, one-shot esp_timer against absolute targets,
drift-free), audio (5), UI/encoder/decay (3), TM1637 display (2, pinned to core 0 so
its bitbang busy-waits cannot starve the UI), web server (2, core 0). The encoder is
decoded by **2kHz polled level sampling** with 2-consecutive-sample confirmation — no
interrupts — plus a 120ms holdback that a button press retroactively cancels (pressing
the shaft wiggles CLK/DT hard enough to fake detents *before* the press is detectable).

Turning the knob silences the metronome and fades the lights; 900ms after the last
detent it restarts clean on a downbeat at the new tempo.

Serial hooks (115200 on native USB CDC): `+ - t` tempo, `s` signature, `c a` click
test, `j` jitter reset, `b` beat trace, `?` state, `!` black-box dump, `W` radio
toggle.

## The bug that was five bugs

This firmware ships with an in-RAM flight recorder (`!` dumps it) because the encoder
"phantom detent" hunt turned out to be five real, separate defects. In the order found:

1. **Edge-ISR state desync** — the interrupt decoder's noise filter rejected edges
   without updating its state machine, desyncing it from the pins permanently. Replaced
   interrupts with polled level sampling (self-healing by construction).
2. **Press-wiggle leak** — clicking the encoder faked detents that arrived before the
   press was electrically detectable. Fixed with a holdback window a raw button-low
   retroactively cancels.
3. **Single-sample corruption** — display-bitbang crosstalk could flip one sample;
   2-consecutive-sample confirmation rejects anything shorter than 500µs.
4. **NVS save loop** — the tempo save re-fired every 2s while phantom detents kept the
   state dirty. Saves are now gated on 10s of true quiet, with the encoder muted and
   resynced across the flash write.
5. **The real spine: HWCDC blocking.** With the USB port closed (i.e. normal standalone
   use), `Serial.printf` on native CDC blocks the calling task up to 20 × 100ms = 2000ms
   per print once the TX buffer fills. The UI task froze for 2010ms per tempo print,
   freezing the light-decay PWM at constant duty — a periodic aggressor that aliased
   slow fake quadrature walks onto the adjacent encoder lines. Connecting a serial
   monitor drained the buffer and made the bug vanish, which is why every diagnostic
   capture came back clean. One line ends the class: `Serial.setTxTimeoutMs(0)`.

If your ESP32-S3/C3 project misbehaves only when the USB cable is unplugged from a
computer, start there.

## Build

PlatformIO with the pioarduino ESP32 core (see `platformio.ini`):

```sh
pio run -t upload --upload-port /dev/ttyACM0
```

## License

MIT
