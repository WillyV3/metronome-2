// Metronome II — ESP32-S3 SuperMini (S3FH4R2)
// Pin map: docs/boards/esp32-s3-supermini.md — PROVISIONAL until silkscreen verify.
// Changes vs v1 (metronome-sculpture): S3 has NO DAC, audio is I2S PDM TX (DMA-fed, same
// blocking-write architecture) -> RC -> LM386 brick. Display is a TM1637 4-digit module:
// two wires, driver chip does the multiplexing, colon shows the time signature as "6:8".
// Everything else (beat engine, gated encoder, tock synth, NVS, serial hooks) is v1 verbatim.
// Serial hooks: + - t s c j ? a  ('d' display walk gone - TM1637 needs no mapping walk)
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <driver/i2s_pdm.h>
#include <atomic>

// ---------------- pins (docs/boards/esp32-s3-supermini.md — PROVISIONAL, verify silkscreen) ----
static const int FIL[6] = {7, 6, 5, 4, 2, 1};            // COB channels 1..6 AS WIRED 2026-08-03 (descending, GPIO3 skipped - strapping)
static std::atomic<int> g_filLevel[6];                    // beat task sets, uiTask decays
static const int TM_CLK = 9, TM_DIO = 8;                 // TM1637 AS WIRED 2026-08-03 (Willy landed DIO on 8, CLK on 9)
static const int ENC_CLK = 10, ENC_DT = 11, ENC_SW = 12; // KY-040 (module pullups on CLK/DT)
// Audio backend: 1 = passive buzzer (GPIO -> 1k -> NPN low-side, buzzer to 5V, FLYBACK
// DIODE across it - the coil is inductive). 0 = PDM -> RC -> LM386 speaker brick (v1 tock).
#define AUDIO_BUZZER 1
static const int AUD_PIN = 13;


// ---------------- musical state ----------------
struct TimeSig { uint8_t beats; uint8_t unit; };
static const TimeSig SIGS[] = {{2,4},{3,4},{4,4},{5,4},{6,8},{7,8}};
static const int NSIGS = sizeof SIGS / sizeof *SIGS;

static std::atomic<int> g_bpm{120};       // 30..300, denominator-unit beats per minute
static std::atomic<int> g_sig{2};         // index into SIGS, default 4/4
static std::atomic<int> g_beat{-1};

static int64_t periodUs() {
  // BPM means ticks-per-minute, ALWAYS - /8 signatures no longer double the rate
  // (surprised Willy on the bench; restore * 0.5 for unit==8 if eighth-note feel is wanted).
  return (int64_t)(60000000.0 / g_bpm.load());
}

#if !AUDIO_BUZZER
// ---------------- click synthesis (static buffers, built once at boot) ----------------
static const int SR = 22050;
static const int CLICK_MS = 42;
static const int NSAMP = SR * CLICK_MS / 1000;
static uint8_t clickHi[NSAMP];   // downbeat: brighter strike
static uint8_t clickLo[NSAMP];   // other beats: lower tock

static void synthClicks() {
  // Mechanical "tock", not an electronic beep: a click is NOISE-dominant with a fast decay
  // (tau ~3ms) plus two INHARMONIC damped partials — inharmonic means no musical pitch, which
  // is exactly what makes wood sound like wood. A decaying sine (the old synth) IS a beep.
  // All energy stays above ~1.5kHz where this small cone actually speaks.
  // Voicing knobs: f1/f2 partial freqs, nGain noise level, decays in the exp() terms.
  auto build = [](uint8_t *buf, float f1, float f2, float nGain, float gain) {
    uint32_t rng = 0x12345678;
    float lp = 0;
    for (int i = 0; i < NSAMP; i++) {
      float t = (float)i / SR;
      rng = rng * 1664525u + 1013904223u;
      float w = ((int)(rng >> 24) - 128) / 128.0f;
      lp += 0.55f * (w - lp);                       // tame the harshest highs a little
      float s = lp * nGain * expf(-t * 350.0f)      // the strike: noise burst, tau ~2.9ms
              + sinf(2 * PI * f1 * t) * 0.45f * expf(-t * 260.0f)   // woody body,
              + sinf(2 * PI * f2 * t) * 0.28f * expf(-t * 320.0f);  // inharmonic pair
      s = constrain(s * gain, -1.0f, 1.0f);
      buf[i] = (uint8_t)(128 + s * 120);
    }
    for (int i = NSAMP - 64; i < NSAMP; i++)   // hard-fade tail to midscale: no stop click
      buf[i] = 128 + (int)((buf[i] - 128) * (NSAMP - 1 - i) / 64.0f);
  };
  build(clickHi, 2230.0f, 3170.0f, 1.0f, 1.0f);    // downbeat: brighter block
  build(clickLo, 1560.0f, 2390.0f, 0.9f, 0.85f);   // other beats: lower, softer tock
}

#endif  // !AUDIO_BUZZER

// ---------------- audio ----------------
#if !AUDIO_BUZZER
static i2s_chan_handle_t s_pdm;
#endif
static QueueHandle_t s_clickQ;           // uint8_t: 1 = downbeat, 0 = beat. Depth 2 ON PURPOSE:
                                         // if audio ever stalls, stale clicks drop instead of
                                         // machine-gunning on recovery.
#if AUDIO_BUZZER
// passive buzzer "tock": impacts CHIRP DOWNWARD with a dying envelope — a fixed-pitch
// burst reads as electronic, a falling one reads as a knock. Each 1ms step drops the
// frequency and (every 3rd step) halves the duty. Voicing knobs: F0/F1/STEPS per voice.
static void tock(bool down) {
  // A resonant buzzer answers at its own 2-4kHz voice whatever you play (square-wave
  // harmonics land in the resonance), so PITCH contrast dies. RHYTHM survives:
  // downbeat = DOUBLE strike ("da-DIK"), other beats = one soft short tick.
  auto strike = [](int freq, int ms, int duty) {
    ledcWriteTone(AUD_PIN, freq);
    ledcWrite(AUD_PIN, duty);
    vTaskDelay(pdMS_TO_TICKS(ms));
    ledcWrite(AUD_PIN, 0);
    ledcWriteTone(AUD_PIN, 0);
  };
  if (down) strike(800, 20, 400);      // the ONE: low thud, long — kick drum under hi-hats
  else      strike(1100, 5, 140);      // ticks sit above it, short and soft
}
static void audioTask(void *) {
  uint8_t id;
  for (;;) {
    if (xQueueReceive(s_clickQ, &id, portMAX_DELAY) != pdTRUE) continue;
    tock(id != 0);
  }
}
#else
static void audioTask(void *) {
  uint8_t id;
  for (;;) {
    if (xQueueReceive(s_clickQ, &id, portMAX_DELAY) != pdTRUE) continue;
    const uint8_t *buf = id ? clickHi : clickLo;
    static int16_t pcm[256];
    size_t off = 0;
    while (off < NSAMP) {                     // 8-bit unsigned -> 16-bit signed, chunked
      size_t n = min((size_t)256, (size_t)NSAMP - off), w = 0;
      for (size_t i = 0; i < n; i++) pcm[i] = ((int)buf[off + i] - 128) << 8;
      i2s_channel_write(s_pdm, pcm, n * 2, &w, 200);
      off += n;
    }
  }
}
#endif  // !AUDIO_BUZZER

// ---------------- encoder: POLLED quadrature decoder (1kHz level sampling) ----------------
// Event-driven decoding is history-dependent: one filtered/missed edge desyncs the state
// machine from the physical pins, and every crosstalk burst afterward walks it from a wrong
// state — the "once it fucks up it doesn't stop" latch Willy caught. Sampling LEVELS on a
// 1ms clock is self-healing (state re-reads reality every tick), and microsecond crosstalk
// bursts from the TM1637 bitbang are invisible between samples. 1kHz tracks 250 detents/s;
// a fast human flick is ~20/s.
static std::atomic<int32_t> g_detents{0};
static std::atomic<uint32_t> g_encMuteUntilMs{0};   // NVS-write bracket: see uiTask save
// Detent microstructure log: cyc = rest->rest cycle duration in ms. A mechanical detent
// snap is 2-30ms; fabricated cycles reveal the aggressor's corruption-window width.
struct QEv { uint32_t ms; int16_t cyc; int8_t dir; };
static QEv g_qev[32];
static std::atomic<uint16_t> g_qevN{0};
static const int8_t QTAB[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
static esp_timer_handle_t s_encTimer;

static void encPoll(void *) {
  // 2-consecutive-sample confirmation: a crosstalk spike can corrupt ONE sample, but the
  // pullup restores the line in ~1us, so a spike cannot survive two polls 500us apart.
  // Real detent edges sit >1ms per state even on fast flicks — nothing mechanical is lost
  // at 2kHz. (Belt-and-suspenders: the serial "phantom" measurements that motivated this
  // were later found contaminated by live bench fingers; kept because it costs 1ms latency.)
  static uint8_t qstate = 0b11, cand = 0b11, candN = 2;
  static int8_t qacc = 0;
  uint8_t ab = (uint8_t)((digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT));
  // Muted during + shortly after NVS flash writes: the write's rail transient fabricated
  // detents (black box: phantoms at EXACTLY the 2s save period, each re-dirtying the bpm
  // and arming the next save — the self-sustaining latch). While muted, track the live
  // pins so sampling resumes in sync with reality.
  if ((int32_t)(millis() - g_encMuteUntilMs.load(std::memory_order_relaxed)) < 0) {
    qstate = cand = ab; candN = 2; qacc = 0; return;
  }
  if (ab != cand) { cand = ab; candN = 1; return; }
  if (candN == 1) candN = 2;
  if (cand == qstate) return;
  qacc += QTAB[(qstate << 2) | cand];
  qstate = cand;
  static uint32_t cycT0 = 0;
  if (qacc != 0 && cycT0 == 0) cycT0 = millis();
  if (qstate == 0b11) {                 // back at rest: judge the completed path
    if (qacc >= 4 || qacc <= -4) {
      g_qev[g_qevN.fetch_add(1) % 32] = { millis(), (int16_t)(millis() - cycT0), (int8_t)(qacc > 0 ? 1 : -1) };
      if (qacc > 0) g_detents.fetch_add(1, std::memory_order_relaxed);
      else g_detents.fetch_sub(1, std::memory_order_relaxed);
    }
    qacc = 0; cycT0 = 0;
  }
}

// ---------------- display: TM1637 4-digit module (driver chip does the multiplexing) ----
// Minimal bitbang driver: TM1637 speaks an I2C-like 2-wire protocol, LSB first, no address.
// Segment bit order is gfedcba (bit0=a). Colon = bit7 of digit index 1.
static const uint8_t TM_DIGIT[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
static const uint8_t TM_DASH = 0x40;
static std::atomic<int64_t> g_tempoUntilUs{0};   // encoder turn -> BPM readout window
static std::atomic<int64_t> g_sigUntilUs{0};     // sig click -> "b:u" readout window

static void tmDelay() { delayMicroseconds(3); }
static void tmStart() { digitalWrite(TM_DIO, LOW); tmDelay(); }
static void tmStop()  { digitalWrite(TM_CLK, LOW); tmDelay(); digitalWrite(TM_DIO, LOW); tmDelay();
                        digitalWrite(TM_CLK, HIGH); tmDelay(); digitalWrite(TM_DIO, HIGH); tmDelay(); }
static void tmByte(uint8_t b) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(TM_CLK, LOW); tmDelay();
    digitalWrite(TM_DIO, (b >> i) & 1); tmDelay();
    digitalWrite(TM_CLK, HIGH); tmDelay();
  }
  digitalWrite(TM_CLK, LOW); pinMode(TM_DIO, INPUT_PULLUP); tmDelay();   // ack slot
  digitalWrite(TM_CLK, HIGH); tmDelay(); digitalWrite(TM_CLK, LOW);
  pinMode(TM_DIO, OUTPUT); tmDelay();
}
static void tmShow(const uint8_t seg[4], uint8_t bright /*0-7*/) {
  tmStart(); tmByte(0x40); tmStop();                 // auto-increment mode
  tmStart(); tmByte(0xC0);                           // addr 0
  for (int i = 0; i < 4; i++) tmByte(seg[i]);
  tmStop();
  tmStart(); tmByte(0x88 | bright); tmStop();        // display on + brightness
}

static void glyphs4(uint8_t out[4]) {
  int64_t now = esp_timer_get_time();
  out[0] = out[1] = out[2] = out[3] = 0;
  if (now < g_tempoUntilUs.load()) {                 // " 120" right aligned
    int v = g_bpm.load();
    out[3] = TM_DIGIT[v % 10];
    out[2] = TM_DIGIT[(v / 10) % 10];
    if (v >= 100) out[1] = TM_DIGIT[v / 100];
  } else if (now < g_sigUntilUs.load()) {            // " 6:8" — colon carries the slash
    const TimeSig &ts = SIGS[g_sig.load()];
    out[1] = TM_DIGIT[ts.beats] | 0x80;              // bit7 of digit 1 = colon
    out[2] = TM_DIGIT[ts.unit];
  } else {                                           // counting: beat number, rightmost
    int b = g_beat.load();
    if (b >= 0) out[3] = TM_DIGIT[(b % 9) + 1];
  }
}

// content-diff refresh at 50Hz: the TM1637 multiplexes itself, we only send changes
static void displayTask(void *) {
  uint8_t cur[4] = {0xFF, 0xFF, 0xFF, 0xFF}, next[4];
  for (;;) {
    glyphs4(next);
    if (memcmp(cur, next, 4) != 0) {
      memcpy(cur, next, 4);
      tmShow(cur, 4);
      vTaskDelay(pdMS_TO_TICKS(100));   // redraw cap ~10Hz: less bitbang = less crosstalk energy
      continue;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ---------------- beat engine: one-shot self-scheduling against absolute targets ----------
static TaskHandle_t s_beatTask;
static esp_timer_handle_t s_beatTimer;
static std::atomic<int64_t> g_targetUs{0};     // when the NEXT beat is due (absolute)
static std::atomic<int64_t> g_lastBeatUs{0};   // when the LAST beat fired (phase anchor)
static std::atomic<int64_t> g_maxJitterUs{0};
static std::atomic<bool> g_trace{false};       // serial 'b': per-beat timing trace

static void beatTimerCb(void *) { xTaskNotifyGive(s_beatTask); }

static void scheduleNext(int64_t dueUs) {
  int64_t now = esp_timer_get_time();
  g_targetUs = dueUs;
  esp_timer_stop(s_beatTimer);
  esp_timer_start_once(s_beatTimer, dueUs > now + 50 ? dueUs - now : 50);
}

// Willy's spec (bench, 2026-08-03): while the knob is turning the metronome goes QUIET —
// no beats, lights decay to dark — and 400ms after the last detent it restarts clean on
// a downbeat at the new tempo. (Every fire-immediately variant machine-gunned the bar.)
static std::atomic<int64_t> g_adjustUntilUs{0};
// 900ms: long enough that SLOW deliberate turning (detents ~500ms apart) stays inside one
// quiet session instead of stuttering tick-quiet-tick across the boundary. Every detent
// pushes the restart out, so continuous turning = continuous quiet, any speed.
static const int64_t ADJUST_SETTLE_US = 900000;
static void tempoChanged() {
  int64_t now = esp_timer_get_time();
  if (now >= g_adjustUntilUs.load()) g_beat = -1;   // session start: restart on a downbeat
  g_adjustUntilUs = now + ADJUST_SETTLE_US;
  scheduleNext(now + ADJUST_SETTLE_US);             // each detent pushes the restart out
}

static void beatTask(void *) {
  esp_task_wdt_add(NULL);
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)) == 0) { esp_task_wdt_reset(); continue; }
    esp_task_wdt_reset();
    int64_t now = esp_timer_get_time();
    int64_t target = g_targetUs.load();
    g_lastBeatUs = now;

    int64_t j = now - target; if (j < 0) j = -j;
    int64_t worst = g_maxJitterUs.load();
    if (j > worst) g_maxJitterUs = j;

    const TimeSig &ts = SIGS[g_sig.load()];
    int b = (g_beat.load() + 1) % ts.beats;
    g_beat = b;
    bool down = (b == 0);

    // next beat: absolute target + period = drift-free. If we somehow stalled past a
    // whole beat (flash write burst, etc), resync rather than machine-gun catch-up.
    int64_t p = periodUs();
    int64_t next = target + p;
    if (next <= now + 2000) next = now + p;
    scheduleNext(next);

    uint8_t id = down ? 1 : 0;
    xQueueSend(s_clickQ, &id, 0);
    if (g_trace.load()) {
      static int64_t lastTrace = 0;
      Serial.printf("[beat] b=%d dt=%lldms bpm=%d\n", b, (now - lastTrace) / 1000, g_bpm.load());
      lastTrace = now;
    }

    // lights: beats past filament 6 HOLD on 6 (7/8 must not wrap onto the downbeat lamp).
    // The beat task only SETS levels; decay happens in uiTask with plain ledcWrite.
    // ledcFade is banned: its fade-engine ISR raced per-beat restarts and crashed (IWDT).
    int lit = min(b, 5);
    g_filLevel[lit] = down ? 255 : 150;
    ledcWrite(FIL[lit], g_filLevel[lit].load());
  }
}

// ---------------- shared UI actions (encoder + serial test hooks use the same paths) -----
static Preferences prefs;
static std::atomic<bool> g_dirty{false};
static std::atomic<uint32_t> g_lastChangeMs{0};

// Black box: phantom episodes happen ONLY while the CDC port is closed (probing suppresses
// them), so events are logged to RAM with a source tag and dumped on '?' afterward.
// src: k=knob detent, w=web, s=serial, S/W=sig, F=NVS save, x=press-cancel, T=uiTask stall(ms)
struct Ev { uint32_t ms; char src; int16_t val; };
static Ev g_ev[48];
static std::atomic<uint16_t> g_evN{0};
static void evLog(char src, int16_t val) {
  g_ev[g_evN.fetch_add(1) % 48] = { millis(), src, val };
}

static void bumpTempo(int d, char src = 's') {
  int bpm = constrain(g_bpm.load() + d, 30, 300);
  if (bpm == g_bpm.load()) return;
  g_bpm = bpm;
  g_lastChangeMs = millis();
  evLog(src, (int16_t)bpm);
  tempoChanged();
  g_tempoUntilUs = esp_timer_get_time() + 3000000;   // BPM readout for 3s
  g_dirty = true;
  Serial.printf("tempo %d bpm\n", bpm);
}

static void nextSig(char src = 'S') {
  int s = (g_sig.load() + 1) % NSIGS;
  g_lastChangeMs = millis();
  evLog(src, (int16_t)s);
  g_sig = s;
  g_beat = -1;
  g_sigUntilUs = esp_timer_get_time() + 1500000;   // "b-u" readout for 1.5s
  g_tempoUntilUs = 0;
  scheduleNext(esp_timer_get_time() + 60000);   // fresh downbeat 60ms after the click
  g_dirty = true;
  Serial.printf("time signature %d/%d\n", SIGS[s].beats, SIGS[s].unit);
}

static void uiTask(void *) {
  int swStable = HIGH, swLast = HIGH;
  uint32_t swChangeMs = 0, swReleaseMs = 0;
  // Pending-detent holdback, v2: the ring version warehoused turns and replayed them
  // seconds late (black box: commits with zero detents behind them, 2010ms dribble).
  // This is the simplest structure that still gives press-wiggle retro-cancel: one
  // accumulator, committed 120ms after its FIRST detent, wiped by any raw SW low.
  int pend = 0;
  uint32_t pendSinceMs = 0, swLowRawMs = 0, lastTickMs = 0;
  bool tick = false;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(10));
    tick = !tick;
    if (tick) {
      for (int i = 0; i < 6; i++) {
        int v = g_filLevel[i].load();
        if (v > 0) {
          v = (v * 220) >> 8;
          if (v < 4) v = 0;
          g_filLevel[i] = v;
          ledcWrite(FIL[i], v);
        }
      }
    }
    int sw = digitalRead(ENC_SW);
    uint32_t ms = millis();
    if (lastTickMs && ms - lastTickMs > 50) evLog('T', (int16_t)min<uint32_t>(ms - lastTickMs, 32767));
    lastTickMs = ms;
    if (sw != swLast) { swLast = sw; swChangeMs = ms; }
    if (ms - swChangeMs > 30 && sw != swStable) {
      swStable = sw;
      if (sw == LOW) nextSig();
      else swReleaseMs = ms;
    }
    int d = g_detents.exchange(0);
    if (sw == LOW) swLowRawMs = ms;
    if ((swLowRawMs && ms - swLowRawMs < 200) || swStable == LOW) {
      if (d || pend) evLog('x', (int16_t)(pend + d));   // press-wiggle cancel, on the record
      d = 0; pend = 0;
    }
    if (d) { if (!pend) pendSinceMs = ms; pend += d; }
    if (pend && ms - pendSinceMs >= 120) {
      // half-quadratic acceleration (Willy: full quadratic too hot): +-1 stays precise,
      // spins scale continuously at half the old rate.
      int mag = abs(pend);
      bumpTempo(mag == 1 ? pend : pend * mag / 2, 'k');
      pend = 0;
    }
    // Settle-gated save: the old save-every-2s-while-dirty loop WAS the phantom (see
    // encPoll comment). Save once after 10s of true quiet, encoder muted across the
    // flash write + an 80ms recovery tail, pending detents discarded.
    if (g_dirty.load() && ms - g_lastChangeMs.load() > 10000) {
      g_encMuteUntilMs = ms + 500;
      prefs.putInt("bpm", g_bpm.load());
      prefs.putInt("sig", g_sig.load());
      evLog('F', (int16_t)g_bpm.load());          // saves show up in the black box too
      g_encMuteUntilMs = millis() + 80;
      g_detents.exchange(0);
      g_dirty = false;
    }
  }
}

// serial hooks: exercise every user path with nothing wired. '+'/'-' = ±5bpm, 't' = ±40 sweep,
// 's' = next signature, 'c' = force click, 'j' = reset jitter stat, '?' = state dump,
// 'd' = display mapping walk, 'a' = audio burst (~0.6s, easier to hunt than one click).
static void serialHook() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case '+': bumpTempo(+5); break;
      case '-': bumpTempo(-5); break;
      case 't': bumpTempo((g_bpm.load() < 165) ? +40 : -40); break;
      case 's': nextSig(); break;
      case 'c': { uint8_t one = 1; xQueueSend(s_clickQ, &one, 0); } break;
      case 'a': for (int i = 0; i < 12; i++) { uint8_t one = 1; xQueueSend(s_clickQ, &one, 200); } break;
      case 'j': g_maxJitterUs = 0; Serial.println("jitter reset"); break;
      case 'b': g_trace = !g_trace.load(); Serial.printf("trace %d\n", g_trace.load()); break;
      case 'W': {                       // radio kill-switch: the discriminating experiment
        static bool off = false; off = !off;
        if (off) { WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); Serial.println("wifi OFF"); }
        else { WiFi.mode(WIFI_AP); WiFi.softAP("Metronome", "metronome"); Serial.println("wifi ON"); }
      } break;
      case '!': {                       // black box dump: oldest -> newest
        uint16_t n = g_evN.load();
        uint16_t start = n > 48 ? n - 48 : 0;
        Serial.printf("[blackbox] %u events total, showing %u:\n", n, n - start);
        for (uint16_t i = start; i < n; i++) {
          const Ev &e = g_ev[i % 48];
          Serial.printf("  [ev] %lums %c %d\n", (unsigned long)e.ms, e.src, e.val);
        }
        uint16_t qn = g_qevN.load();
        uint16_t qs = qn > 32 ? qn - 32 : 0;
        Serial.printf("[detents] %u total, showing %u (ms dir cyc):\n", qn, qn - qs);
        for (uint16_t i = qs; i < qn; i++) {
          const QEv &q = g_qev[i % 32];
          Serial.printf("  [dt] %lums %+d cyc=%dms\n", (unsigned long)q.ms, q.dir, q.cyc);
        }
      } break;
      case '?': Serial.printf("state: %d bpm, %d/%d, beat %d, target in %lldus | AP %s, %d client(s)\n",
                    g_bpm.load(), SIGS[g_sig.load()].beats, SIGS[g_sig.load()].unit,
                    g_beat.load(), g_targetUs.load() - esp_timer_get_time(),
                    WiFi.softAPIP().toString().c_str(), WiFi.softAPgetStationNum()); break;
    }
  }
}

// ---------------- phone remote: standalone AP + one-page web UI ----------------
// The metronome TRAVELS (Willy: no home-network reliance) — it broadcasts its own WPA2
// AP. Phone joins "Metronome" / pass "metronome", opens http://192.168.4.1 (AP default
// IP, stable, bookmarkable). All controls route through bumpTempo()/nextSig(), so the
// quiet-adjust sessions, readout windows, and NVS saves behave exactly like the knob.
static WebServer s_web(80);
static const char INDEX_HTML[] = R"HTML(<!doctype html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Metronome</title><style>
body{margin:0;background:#111;color:#eee;font-family:system-ui;overflow:hidden}
#pv{display:flex;height:100vh;flex-direction:column;align-items:center}
#scene{position:relative;width:100vw;height:100vh}
.pyr{position:absolute;left:50%;bottom:0;transform:translateX(-50%);clip-path:polygon(50% 0,100% 100%,0 100%)}
#pyrO{width:66vw;height:82vh;background:#3a3a3a}
#pyrI{width:64vw;height:80.5vh;background:#191919}
#arm{position:absolute;left:50%;bottom:15vh;width:1.4vw;height:60vh;margin-left:-.7vw;background:#c8c8c8;border-radius:1vw;transform-origin:50% 100%}
#bob{position:absolute;left:50%;transform:translateX(-50%);width:8vw;height:5.5vw;background:#2f6b40;border:1px solid #6fbf7f;border-radius:1.2vw}
#pivot{position:absolute;left:50%;bottom:15vh;width:3.6vw;height:3.6vw;margin:0 0 -1.8vw -1.8vw;background:#888;border-radius:50%}
#bpmlbl{position:absolute;left:50%;bottom:5vh;transform:translateX(-50%);font-size:6vw;color:#7c7;font-variant-numeric:tabular-nums}
#gear{position:fixed;right:4vw;bottom:4vh;background:#222;color:#999;border:1px solid #444;border-radius:10px;font-size:4.5vw;padding:2vw 4vw}
#cv{display:none;height:100vh;flex-direction:column;align-items:center;gap:4vh;padding-top:6vh}
#bpm{font-size:26vw;font-weight:700;line-height:1;font-variant-numeric:tabular-nums}
#sig{font-size:9vw;color:#7c7}
.row{display:flex;gap:3vw}
button{background:#222;color:#eee;border:1px solid #444;border-radius:12px;font-size:6.5vw;padding:3vw 5vw}
button:active{background:#274}
#num{background:#222;color:#eee;border:1px solid #444;border-radius:12px;font-size:6.5vw;padding:3vw;width:30vw;text-align:center}
</style></head><body>
<div id=pv><div id=scene>
<div class=pyr id=pyrO></div><div class=pyr id=pyrI></div>
<div id=arm><div id=bob></div></div><div id=pivot></div>
<div id=bpmlbl>--</div>
</div><button id=gear onclick="view(0)">adjust</button></div>
<div id=cv>
<div id=bpm>--</div><div id=sig>-/-</div>
<div class=row>
<button onclick="q('/api/bpm?d=-10')">-10</button><button onclick="q('/api/bpm?d=-1')">-1</button>
<button onclick="q('/api/bpm?d=1')">+1</button><button onclick="q('/api/bpm?d=10')">+10</button>
</div>
<div class=row><input id=num type=number inputmode=numeric min=30 max=300 placeholder=bpm>
<button onclick="setBpm()">set</button></div>
<div class=row><button onclick="q('/api/sig')">time signature</button>
<button onclick="view(1)">done</button></div>
</div>
<script>
// The arm hits its extremes exactly ON the device's beats: /api/state carries nextMs
// (time to next beat) + periodMs. Client keeps a continuous phase t0 (epoch of an
// extreme) and each poll NUDGES t0 by the smallest modular correction, so the swing
// alternates sides properly and never snaps. Quiet-adjust sessions ease the arm to
// center; the restart downbeat re-launches it.
var per=600,t0=0,quiet=false,th=0;
function show(s){
 bpm.textContent=s.bpm;sig.textContent=s.beats+"/"+s.unit;bpmlbl.textContent=s.bpm;
 var tb=performance.now()+s.nextMs;
 quiet=s.nextMs>s.periodMs*1.2;
 if(s.periodMs!=per||!t0){per=s.periodMs;t0=tb}
 else{var e=((tb-t0)%per+per)%per;if(e>per/2)e-=per;t0+=e}
 bob.style.top=(4+(s.bpm-30)/270*60)+"%"}
function loop(t){
 var tgt=quiet?0:26*Math.cos(Math.PI*(t-t0)/per);
 th+=(tgt-th)*(quiet?0.12:0.5);
 arm.style.transform="rotate("+th+"deg)";
 requestAnimationFrame(loop)}
requestAnimationFrame(loop);
function q(u){fetch(u).then(r=>r.json()).then(show)}
function setBpm(){if(num.value){q("/api/bpm?set="+num.value);num.value="";num.blur()}}
num.addEventListener("keydown",function(e){if(e.key=="Enter")setBpm()});
function view(p){pv.style.display=p?"flex":"none";cv.style.display=p?"none":"flex"}
setInterval(function(){q("/api/state")},1000);q("/api/state")
</script></body></html>)HTML";

static void webSendState() {
  char j[128];
  const TimeSig &ts = SIGS[g_sig.load()];
  int64_t nextMs = (g_targetUs.load() - esp_timer_get_time()) / 1000;
  if (nextMs < 0) nextMs = 0;
  snprintf(j, sizeof j, "{\"bpm\":%d,\"beats\":%d,\"unit\":%d,\"beat\":%d,\"nextMs\":%lld,\"periodMs\":%lld}",
           g_bpm.load(), ts.beats, ts.unit, g_beat.load(), nextMs, periodUs() / 1000);
  s_web.send(200, "application/json", j);
}
static void webTask(void *) {
  for (;;) { s_web.handleClient(); vTaskDelay(pdMS_TO_TICKS(10)); }
}
static void webStart() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Metronome", "metronome");   // WPA2 — an open AP would let the whole gig conduct
  s_web.on("/", [] { s_web.send_P(200, "text/html", INDEX_HTML); });
  s_web.on("/api/state", webSendState);
  s_web.on("/api/bpm", [] {
    if (s_web.hasArg("d"))        bumpTempo(s_web.arg("d").toInt(), 'w');
    else if (s_web.hasArg("set")) bumpTempo(s_web.arg("set").toInt() - g_bpm.load(), 'w');
    webSendState();
  });
  s_web.on("/api/sig", [] { nextSig('W'); webSendState(); });
  s_web.begin();
  xTaskCreatePinnedToCore(webTask, "web", 4096, nullptr, 2, nullptr, 0);
  Serial.printf("AP up: SSID Metronome, http://%s\n", WiFi.softAPIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  // Port-closed printf BLOCKS the calling task up to 20x100ms (HWCDC.cpp bounded wait)
  // — the "2010ms" phantom clock. uiTask froze 2s per tempo print whenever no host was
  // attached, freezing the filament-decay PWM at constant duty, which aliased into slow
  // coherent quadrature walks on the adjacent encoder rows. Zero = drop, never block.
  Serial.setTxTimeoutMs(0);
  delay(300);
  Serial.println("\n== metronome sculpture ==");

  prefs.begin("metro");
  g_bpm = constrain(prefs.getInt("bpm", 120), 30, 300);
  g_sig = constrain(prefs.getInt("sig", 2), 0, NSIGS - 1);

#if !AUDIO_BUZZER
  uint32_t t0 = micros();
  synthClicks();
  Serial.printf("click synth: %lu us for %dx2 samples\n", (unsigned long)(micros() - t0), NSAMP);
#endif

  pinMode(TM_CLK, OUTPUT); pinMode(TM_DIO, OUTPUT);
  digitalWrite(TM_CLK, HIGH); digitalWrite(TM_DIO, HIGH);
  g_sigUntilUs = esp_timer_get_time() + 1500000;   // greet with the signature
  for (int p : FIL) { ledcAttach(p, 5000, 8); ledcWrite(p, 0); }

  // S3 has no input-only pins, so internal pullups hold CLK/DT quiet until the KY-040
  // is wired (v1's pins 34/35 couldn't do this - floating noise stormed the tempo).
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  static const esp_timer_create_args_t eargs = {
      .callback = encPoll, .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK, .name = "enc", .skip_unhandled_events = true};
  ESP_ERROR_CHECK(esp_timer_create(&eargs, &s_encTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(s_encTimer, 500));   // 2kHz, 2-sample confirm

#if AUDIO_BUZZER
  ledcAttach(AUD_PIN, 2000, 10);   // tone bursts, freq set per click
#else
  // S3 has no DAC: PDM TX is the DMA-fed analog substitute (1-bit stream, RC recovers audio)
  i2s_chan_config_t ch = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&ch, &s_pdm, NULL));
  i2s_pdm_tx_config_t pdm = {
    .clk_cfg = I2S_PDM_TX_CLK_DEFAULT_CONFIG(SR),
    .slot_cfg = I2S_PDM_TX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = { .clk = I2S_GPIO_UNUSED, .dout = (gpio_num_t)AUD_PIN, .invert_flags = {} },
  };
  ESP_ERROR_CHECK(i2s_channel_init_pdm_tx_mode(s_pdm, &pdm));
  ESP_ERROR_CHECK(i2s_channel_enable(s_pdm));
#endif

  s_clickQ = xQueueCreate(2, sizeof(uint8_t));
  xTaskCreatePinnedToCore(audioTask, "audio", 3072, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(beatTask, "beat", 3072, nullptr, 6, &s_beatTask, 1);
  xTaskCreatePinnedToCore(uiTask, "ui", 3072, nullptr, 3, nullptr, 1);
  // display goes LOW priority on the OTHER core: its bitbang busy-waits were starving
  // uiTask (knob + light decay) during tempo readouts — lights froze bright, detents
  // batched. Least-critical task, isolated where it can't preempt anything that matters.
  xTaskCreatePinnedToCore(displayTask, "disp", 2048, nullptr, 2, nullptr, 0);

  static const esp_timer_create_args_t targs = {
      .callback = beatTimerCb, .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK, .name = "beat", .skip_unhandled_events = true};
  ESP_ERROR_CHECK(esp_timer_create(&targs, &s_beatTimer));
  scheduleNext(esp_timer_get_time() + periodUs());

  webStart();
  Serial.printf("running: %d bpm, %d/%d  (serial hooks: + - t s c j ?)\n",
                g_bpm.load(), SIGS[g_sig.load()].beats, SIGS[g_sig.load()].unit);
}

void loop() {
  serialHook();
  static uint32_t last = 0;
  vTaskDelay(pdMS_TO_TICKS(50));
  if (millis() - last < 15000) return;
  last = millis();
  Serial.printf("[health] jitter max %lldus | heap %lu | stack beat:%u\n",
                g_maxJitterUs.load(), (unsigned long)ESP.getFreeHeap(),
                s_beatTask ? uxTaskGetStackHighWaterMark(s_beatTask) : 0);
  g_maxJitterUs = 0;
}
