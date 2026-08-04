// metronome-2 — ESP32-S3 SuperMini
// 6 COB filaments, TM1637, KY-040, passive buzzer, wifi AP + web remote.
// serial hooks: + - t s c a j b ? ! W
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <driver/i2s_pdm.h>
#include <atomic>

// ---------------- pins ----------------
static const int FIL[6] = {7, 6, 5, 4, 2, 1};            // COB channels 1..6 (GPIO3 skipped - strapping)
static std::atomic<int> g_filLevel[6];                    // beat task sets, uiTask decays
static const int TM_CLK = 9, TM_DIO = 8;                 // TM1637
static const int ENC_CLK = 10, ENC_DT = 11, ENC_SW = 12; // KY-040 (module pullups on CLK/DT)
// 1 = passive buzzer (1k -> NPN low-side, flyback diode across the coil)
// 0 = I2S PDM -> RC -> LM386 speaker
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
  // BPM = ticks per minute regardless of the /8 signatures
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
static QueueHandle_t s_clickQ;           // 1 = downbeat, 0 = beat. depth 2: stale clicks
                                         // drop instead of machine-gunning on recovery.
#if AUDIO_BUZZER
// a resonant buzzer answers at its own voice whatever you play, so pitch contrast is
// weak — the downbeat gets a longer, harder strike instead.
static void tock(bool down) {
  auto strike = [](int freq, int ms, int duty) {
    ledcWriteTone(AUD_PIN, freq);
    ledcWrite(AUD_PIN, duty);
    vTaskDelay(pdMS_TO_TICKS(ms));
    ledcWrite(AUD_PIN, 0);
    ledcWriteTone(AUD_PIN, 0);
  };
  if (down) strike(800, 20, 400);      // downbeat: low long thud
  else      strike(1100, 5, 140);      // ticks: short and soft
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

// ---------------- encoder: polled quadrature, 2kHz ----------------
// no interrupts: edge decoding desyncs on noise, level sampling re-reads the pins every
// tick. 2-consecutive-sample confirm rejects sub-500us glitches.
static std::atomic<int32_t> g_detents{0};
static std::atomic<uint32_t> g_encMuteUntilMs{0};   // NVS-write bracket: see uiTask save
// detent log ('!' dump): cyc = rest->rest ms
struct QEv { uint32_t ms; int16_t cyc; int8_t dir; };
static QEv g_qev[32];
static std::atomic<uint16_t> g_qevN{0};
static const int8_t QTAB[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
static esp_timer_handle_t s_encTimer;

static void encPoll(void *) {
  static uint8_t qstate = 0b11, cand = 0b11, candN = 2;
  static int8_t qacc = 0;
  uint8_t ab = (uint8_t)((digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT));
  // muted around NVS flash writes; track the pins so sampling resumes in sync
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
static TaskHandle_t s_dispTask;
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

// turning = quiet: no beats, lights decay. restarts on a downbeat at the new tempo
// once the knob settles. 900ms so slow turning stays inside one quiet session.
static std::atomic<int64_t> g_adjustUntilUs{0};
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

    // beats past filament 6 hold on 6. beat task sets levels, uiTask decays.
    // no ledcFade: its fade ISR races per-beat restarts and crashes.
    int lit = min(b, 5);
    g_filLevel[lit] = down ? 255 : 150;
    ledcWrite(FIL[lit], g_filLevel[lit].load());
  }
}

// ---------------- shared UI actions (encoder + serial test hooks use the same paths) -----
static Preferences prefs;
static std::atomic<bool> g_dirty{false};
static std::atomic<uint32_t> g_lastChangeMs{0};

// RAM event log ('!' dump). src: k=knob w=web s=serial S/W=sig F=save x=press-cancel T=ui stall
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

// double-tap on the encoder button -> deep sleep; any press wakes (ext0 on GPIO12,
// RTC pullup kept alive). sig is restored to its pre-tap value so the double-tap's
// two nextSig() firings leave no trace after wake.
static void goSleep(int restoreSig) {
  esp_timer_stop(s_beatTimer);
  vTaskSuspend(s_dispTask);                     // stop redraws so SHHH survives
  vTaskDelay(pdMS_TO_TICKS(20));
  static const uint8_t SHHH[4] = {0x6D, 0x76, 0x76, 0x76};
  tmShow(SHHH, 2);   // low brightness; TM1637 multiplexes on its own, so SHHH
                     // stays lit through deep sleep (~2mA vs dark)
  g_sig = restoreSig;
  prefs.putInt("bpm", g_bpm.load());
  prefs.putInt("sig", restoreSig);
  // high-Z LEDC pins float the transistor bases in deep sleep (= all lights ON):
  // detach, drive LOW, and hold-latch every driver pin through sleep.
  for (int pin : FIL) { ledcDetach(pin); pinMode(pin, OUTPUT); digitalWrite(pin, LOW); gpio_hold_en((gpio_num_t)pin); }
  ledcDetach(AUD_PIN); pinMode(AUD_PIN, OUTPUT); digitalWrite(AUD_PIN, LOW); gpio_hold_en((gpio_num_t)AUD_PIN);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("deep sleep (double tap)");
  while (digitalRead(ENC_SW) == LOW) delay(10); // arm only after release, else instant wake
  delay(200);
  // park the TM1637 bus at idle and hold it — floating CLK/DIO in deep sleep feeds
  // the chip noise it can read as commands (SHHH was dying at sleep entry)
  digitalWrite(TM_CLK, HIGH); digitalWrite(TM_DIO, HIGH);
  gpio_hold_en((gpio_num_t)TM_CLK); gpio_hold_en((gpio_num_t)TM_DIO);
  gpio_deep_sleep_hold_en();
  rtc_gpio_pullup_en((gpio_num_t)ENC_SW);
  rtc_gpio_pulldown_dis((gpio_num_t)ENC_SW);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)ENC_SW, 0);
  esp_deep_sleep_start();
}

static void uiTask(void *) {
  int swStable = HIGH, swLast = HIGH;
  uint32_t swChangeMs = 0, swReleaseMs = 0;
  // detent holdback: committed 120ms after the first detent, wiped by any raw SW low
  // (pressing wiggles CLK/DT hard enough to fake detents before the press debounces)
  int pend = 0;
  uint32_t pendSinceMs = 0, swLowRawMs = 0, lastTickMs = 0, lastPressMs = 0;
  int sigBeforeTap = -1;
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
      if (sw == LOW) {
        if (ms - lastPressMs < 400 && sigBeforeTap >= 0) goSleep(sigBeforeTap);
        sigBeforeTap = g_sig.load();
        lastPressMs = ms;
        nextSig();
      } else swReleaseMs = ms;
    }
    int d = g_detents.exchange(0);
    if (sw == LOW) swLowRawMs = ms;
    if ((swLowRawMs && ms - swLowRawMs < 200) || swStable == LOW) {
      if (d || pend) evLog('x', (int16_t)(pend + d));
      d = 0; pend = 0;
    }
    if (d) { if (!pend) pendSinceMs = ms; pend += d; }
    if (pend && ms - pendSinceMs >= 120) {
      // half-quadratic accel: +-1 stays precise, spins scale
      int mag = abs(pend);
      bumpTempo(mag == 1 ? pend : pend * mag / 2, 'k');
      pend = 0;
    }
    // save after 10s of quiet, encoder muted across the flash write
    if (g_dirty.load() && ms - g_lastChangeMs.load() > 10000) {
      g_encMuteUntilMs = ms + 500;
      prefs.putInt("bpm", g_bpm.load());
      prefs.putInt("sig", g_sig.load());
      evLog('F', (int16_t)g_bpm.load());
      g_encMuteUntilMs = millis() + 80;
      g_detents.exchange(0);
      g_dirty = false;
    }
  }
}

// serial hooks: +/- = 5bpm, t = 40 sweep, s = sig, c = click, a = click burst,
// j = jitter reset, b = beat trace, ? = state, ! = event log, W = wifi toggle
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
      case 'W': {
        static bool off = false; off = !off;
        if (off) { WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); Serial.println("wifi OFF"); }
        else { WiFi.mode(WIFI_AP); WiFi.softAP("Metronome", "metronome", 1, 0, 8); Serial.println("wifi ON"); }
      } break;
      case '!': {                       // event log dump, oldest -> newest
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
// join "Metronome" / "metronome", open http://192.168.4.1. controls share the knob's
// code paths so remote changes behave exactly like turning it.
static WebServer s_web(80);
static const char INDEX_HTML[] = R"HTML(<!doctype html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no"><title>Metronome</title><style>
*{margin:0;box-sizing:border-box}
body{height:100vh;overflow:hidden;font-family:Georgia,serif;background:
 radial-gradient(130vw 90vh at 50% 26%,#2a1e11,#0d0906 72%)}
#desk{position:fixed;bottom:0;width:100%;height:22vh;
 background:linear-gradient(#0000 0,#0007 100%),
 repeating-linear-gradient(90deg,#563619 0 4vw,#4c2f16 4vw 9vw,#5c3a1c 9vw 13vw);
 box-shadow:0 -2px 0 #7a5228,0 -6px 18px #0009}
#shadow{position:fixed;bottom:20.5vh;left:50%;transform:translateX(-50%);width:70vw;height:5vh;
 background:radial-gradient(closest-side,#000c,#0000);border-radius:50%}
#plinth{position:fixed;bottom:22vh;left:50%;transform:translateX(-50%);width:82vw;height:9vh;
 background:linear-gradient(#5b3a1c,#3f2810),repeating-linear-gradient(90deg,#0002 0 2px,#0000 2px 7px);
 border-radius:1.6vw 1.6vw 0 0;box-shadow:inset 0 2px 3px #ffffff22,0 4px 14px #000a;
 display:flex;align-items:center;justify-content:center;gap:6vw}
#bezel{width:22vw;height:6.4vh;background:#121418;border-radius:2vw;
 box-shadow:inset 0 0 8px #000,0 1px 0 #ffffff18;display:flex;align-items:center;justify-content:center}
#beatd{font-family:ui-monospace,Menlo,monospace;font-size:4.6vh;color:#dfe8ff;text-shadow:0 0 12px #9fb6ff88}
#plaque{min-width:24vw;padding:.8vh 3vw;text-align:center;border-radius:1vw;
 background:linear-gradient(#d9b35c,#a37c33);color:#33210a;font-size:2.2vh;
 box-shadow:inset 0 1px 0 #ffe9b8,0 2px 5px #0008}
#mbody{position:fixed;bottom:31vh;left:50%;transform:translateX(-50%);width:72vw;height:56vh;
 clip-path:polygon(40% 0,60% 0,93% 100%,7% 100%);
 background:linear-gradient(90deg,#6e4520,#8a5a2b 28%,#7a4e24 52%,#935f2c 78%,#66401d),
 repeating-linear-gradient(92deg,#0002 0 3px,#0000 3px 10px),
 linear-gradient(115deg,#ffffff26 2%,#0000 34%)}
#track{position:fixed;bottom:32vh;left:50%;transform:translateX(-50%);width:72vw;height:52vh;
 clip-path:polygon(46.5% 0,53.5% 0,60% 100%,40% 100%);
 background:linear-gradient(#1c1108,#2a1a0d);box-shadow:inset 0 0 12px #000}
#scale{position:fixed;bottom:34vh;left:50%;transform:translateX(-50%);width:1.2vw;height:46vh;
 background:repeating-linear-gradient(#c9a04e3d 0 .5vh,#0000 .5vh 3.4vh)}
#arm{position:fixed;bottom:33.5vh;left:50%;width:1.6vw;height:44vh;margin-left:-.8vw;
 background:linear-gradient(90deg,#e0bc72,#a8803a);border-radius:1vw;transform-origin:50% 100%}
#bob{position:absolute;left:50%;transform:translateX(-50%);width:9vw;height:5.6vw;
 background:radial-gradient(#ffb35c,#c97a1e);border:1px solid #ffd79a66;border-radius:1.4vw}
#pivot{position:fixed;bottom:33.5vh;left:50%;width:5vw;height:5vw;margin:0 0 -2.5vw -2.5vw;
 background:radial-gradient(circle at 35% 30%,#e8c87e,#8a6526);border-radius:50%;
 box-shadow:0 1px 4px #000c}
#gear{position:fixed;right:4vw;bottom:3vh;background:#1c130b;color:#c9a04e;border:1px solid #c9a04e55;
 border-radius:2vw;font-size:2.1vh;padding:1.2vh 4vw;font-family:Georgia,serif}
#cv{display:none;position:fixed;inset:0;flex-direction:column;align-items:center;gap:3.6vh;padding-top:7vh}
#bpm{font-size:24vw;line-height:1;color:#f0e2c0;font-family:ui-monospace,Menlo,monospace}
#sig{font-size:8vw;color:#c9a04e}
.row{display:flex;gap:3vw}
button{background:#241a10;color:#e8d9b0;border:1px solid #c9a04e66;border-radius:2vw;
 font-size:6vw;padding:2.6vw 5vw;font-family:Georgia,serif}
button:active{background:#3a2a16}
#num{background:#1c130b;color:#e8d9b0;border:1px solid #c9a04e66;border-radius:2vw;
 font-size:6vw;padding:2.6vw;width:30vw;text-align:center}
</style></head><body>
<div id=pv>
<div id=desk></div><div id=shadow></div>
<div id=mbody></div><div id=track></div><div id=scale></div>
<div id=arm><div id=bob></div></div><div id=pivot></div>
<div id=plinth><div id=bezel><span id=beatd></span></div><div id=plaque>--</div></div>
<button id=gear onclick="view(0)">adjust</button>
</div>
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
<div class=row><button onclick="doSleep()">shhh</button></div>
</div>
<script>
// arm extremes land on the device beats: continuous phase t0, nudged by the smallest
// modular correction each poll. beat count advances client-side at each extreme.
var per=600,t0=0,quiet=false,th=0,beat=-1,beats=4,tbeat=0,glow=0;
function show(s){
 bpm.textContent=s.bpm;sig.textContent=s.beats+"/"+s.unit;
 plaque.textContent=s.bpm+" bpm";
 var tb=performance.now()+s.nextMs;
 quiet=s.nextMs>s.periodMs*1.2;
 beat=s.beat;beats=s.beats;tbeat=tb;
 if(s.periodMs!=per||!t0){per=s.periodMs;t0=tb}
 else{var e=((tb-t0)%per+per)%per;if(e>per/2)e-=per;t0+=e}
 var f=(s.bpm-30)/270;bob.style.top=(6+f*58)+"%"}
function loop(t){
 while(tbeat&&t>=tbeat){if(beat>=0){beat=(beat+1)%beats;glow=beat==0?1.5:.8}tbeat+=per}
 beatd.textContent=(quiet||beat<0)?"":beat+1;
 glow*=.9;
 bob.style.boxShadow="0 0 "+(glow*6)+"vw "+(glow*2)+"vw rgba(255,154,42,"+(glow*.5)+")";
 var tgt=quiet?0:26*Math.cos(Math.PI*(t-t0)/per);
 th+=(tgt-th)*(quiet?.12:.5);
 arm.style.transform="rotate("+th+"deg)";
 requestAnimationFrame(loop)}
requestAnimationFrame(loop);
function q(u){fetch(u).then(r=>r.json()).then(show)}
function setBpm(){if(num.value){q("/api/bpm?set="+num.value);num.value="";num.blur()}}
num.addEventListener("keydown",function(e){if(e.key=="Enter")setBpm()});
function view(p){pv.style.display=p?"block":"none";cv.style.display=p?"none":"flex"}
function doSleep(){fetch("/api/sleep");document.body.style.opacity=.25}
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
  WiFi.softAP("Metronome", "metronome", 1, 0, 8);   // 8 clients; an open AP would let the whole gig conduct
  s_web.on("/", [] {
    s_web.sendHeader("Cache-Control", "no-store");   // stale pages hid every UI update
    s_web.send_P(200, "text/html", INDEX_HTML);
  });
  s_web.on("/api/state", webSendState);
  s_web.on("/api/bpm", [] {
    if (s_web.hasArg("d"))        bumpTempo(s_web.arg("d").toInt(), 'w');
    else if (s_web.hasArg("set")) bumpTempo(s_web.arg("set").toInt() - g_bpm.load(), 'w');
    webSendState();
  });
  s_web.on("/api/sig", [] { nextSig('W'); webSendState(); });
  s_web.on("/api/sleep", [] {
    s_web.send(200, "application/json", "{\"ok\":true}");
    delay(150);                    // let the response leave before the radio dies
    goSleep(g_sig.load());         // wake is the physical button only
  });
  s_web.begin();
  xTaskCreatePinnedToCore(webTask, "web", 4096, nullptr, 2, nullptr, 0);
  Serial.printf("AP up: SSID Metronome, http://%s\n", WiFi.softAPIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  // port-closed CDC writes block the caller up to 2s once the TX ring fills — drop instead
  Serial.setTxTimeoutMs(0);
  delay(300);
  Serial.println("\n== metronome-2 ==");

  prefs.begin("metro");
  g_bpm = constrain(prefs.getInt("bpm", 120), 30, 300);
  g_sig = constrain(prefs.getInt("sig", 2), 0, NSIGS - 1);

#if !AUDIO_BUZZER
  uint32_t t0 = micros();
  synthClicks();
  Serial.printf("click synth: %lu us for %dx2 samples\n", (unsigned long)(micros() - t0), NSAMP);
#endif

  gpio_deep_sleep_hold_dis();                    // release sleep-held driver pins on wake
  for (int pin : FIL) gpio_hold_dis((gpio_num_t)pin);
  gpio_hold_dis((gpio_num_t)AUD_PIN);
  gpio_hold_dis((gpio_num_t)TM_CLK); gpio_hold_dis((gpio_num_t)TM_DIO);
  pinMode(TM_CLK, OUTPUT); pinMode(TM_DIO, OUTPUT);
  digitalWrite(TM_CLK, HIGH); digitalWrite(TM_DIO, HIGH);
  g_sigUntilUs = esp_timer_get_time() + 1500000;   // greet with the signature
  for (int p : FIL) { ledcAttach(p, 5000, 8); ledcWrite(p, 0); }

  // internal pullups on all three encoder lines
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
  // display on core 0, low prio: its bitbang busy-waits must not preempt the ui
  xTaskCreatePinnedToCore(displayTask, "disp", 2048, nullptr, 2, &s_dispTask, 0);

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
