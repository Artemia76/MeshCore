// LR1110 probe (debug build only): measures the real chip's behaviour over raw SPI, to keep the digital twin's
// model faithful (emulator/twin/twin/lr1110.py). Console command "lr <test>"; every test ends with a reboot
// so that MeshCore gets a cleanly configured radio again.
//
// Raw access: NSS driven by hand, SPI at 2 MHz mode 0 (as RadioLib), BUSY read on its pin, the IRQ line read on
// AON1 (DIO9 through an inverter, which digitalRead undoes: HIGH = interrupt pending). Times in microseconds (micros()).
#ifdef T1000A_BLE_CONSOLE
#include <Arduino.h>
#include <SPI.h>

namespace {

Print* out;

inline bool busy() { return digitalRead(P_LORA_BUSY) == HIGH; }
inline bool irqLine() { return digitalRead(P_LORA_DIO_1) == HIGH; }  // DIO9 inverted on the board, undone by digitalRead

// Waits for BUSY low; returns the time waited (us), or -1 after timeout_us
long waitBusyLow(unsigned long timeout_us = 200000) {
  unsigned long t0 = micros();
  while (busy()) {
    if (micros() - t0 > timeout_us) return -1;
    if (micros() - t0 > 2000) gr551x_service();   // watchdog during long waits
  }
  return (long)(micros() - t0);
}

// One SPI transaction (NSS low .. high). Returns the BUSY-high time after NSS rises (us), -1 on timeout.
long xfer(const uint8_t* tx, uint8_t* rx, size_t n, bool wait_busy_before = true, bool measure = true) {
  if (wait_busy_before) waitBusyLow();
  static uint8_t dummy[300];                      // MISO bytes of a write (up to WriteBuffer8 255 + 2)
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(P_LORA_NSS, LOW);
  SPI.transfer(tx, rx ? rx : dummy, n);
  digitalWrite(P_LORA_NSS, HIGH);
  SPI.endTransaction();
  return measure ? waitBusyLow() : 0;
}

long cmd(uint16_t op, const uint8_t* p = nullptr, size_t n = 0, bool wait_busy_before = true) {
  uint8_t tx[300];
  tx[0] = op >> 8; tx[1] = op & 0xFF;
  for (size_t i = 0; i < n; i++) tx[2 + i] = p[i];
  return xfer(tx, nullptr, n + 2, wait_busy_before);
}

// Read command: write phase, then NOPs to clock out Stat1 + data (rsp[0] = Stat1)
long readCmd(uint16_t op, const uint8_t* p, size_t n, uint8_t* rsp, size_t rn) {
  long t = cmd(op, p, n);
  uint8_t tx[64] = {0};
  xfer(tx, rsp, rn + 1);
  return t;
}

struct Status { uint8_t s1, s2; uint32_t irq; };
Status getStatus() {
  uint8_t tx[6] = {0x01, 0x00, 0, 0, 0, 0}, rx[6];
  xfer(tx, rx, 6);
  return {rx[0], rx[1], (uint32_t)rx[2] << 24 | (uint32_t)rx[3] << 16 | (uint32_t)rx[4] << 8 | rx[5]};
}

const char* modeName(uint8_t s2) {
  static const char* n[] = {"SLEEP", "STBY_RC", "STBY_XOSC", "FS", "RX", "TX", "GEO", "?"};
  return n[(s2 >> 1) & 7];
}

void printStatus(const char* what, const Status& s) {
  out->printf("  %s: mode=%s cmd=%u irq=%08lx reset=%u boot=%u\r\n", what, modeName(s.s2), (s.s1 >> 1) & 7,
              (unsigned long)s.irq, s.s2 >> 4, s.s2 & 1);
}

void standbyRc() { uint8_t p = 0; cmd(0x011C, &p, 1); }
void clearIrq() { uint8_t p[4] = {0xFF, 0xFF, 0xFF, 0xFF}; cmd(0x0114, p, 4); }

uint8_t packetType() { uint8_t r[2]; readCmd(0x0202, nullptr, 0, r, 1); return r[1]; }

void writeBuf(uint8_t seed, size_t n) {
  uint8_t p[255];
  for (size_t i = 0; i < n; i++) p[i] = seed + i;
  cmd(0x0109, p, n);
}
// Bytes 0..n-1 of the data buffer equal to the pattern written by writeBuf(seed)?
int bufMatches(uint8_t seed, size_t n) {
  uint8_t p[2] = {0, (uint8_t)n}, r[64];
  readCmd(0x010A, p, 2, r, n);
  int ok = 0;
  for (size_t i = 0; i < n; i++) ok += r[1 + i] == (uint8_t)(seed + i);
  return ok;
}

// Records BUSY edges for `ms`: prints the high/low durations (us)
void busyChronogram(unsigned ms, int max_edges = 24) {
  unsigned long t0 = micros(), last = t0;
  bool lvl = busy();
  int edges = 0;
  out->printf("  BUSY start=%d :", lvl);
  while (micros() - t0 < ms * 1000UL && edges < max_edges) {
    bool b = busy();
    if (b != lvl) {
      unsigned long now = micros();
      out->printf(" %s%lu", lvl ? "H" : "L", now - last);
      last = now; lvl = b; edges++;
    }
    if ((edges & 3) == 0) gr551x_service();
  }
  out->print("\r\n");
}

// ------------------------------------------------------------------ tests
void testVersion() {
  uint8_t r[5];
  readCmd(0x0101, nullptr, 0, r, 4);
  out->printf("version: hw=%02x type=%02x fw=%02x.%02x\r\n", r[1], r[2], r[3], r[4]);
  uint8_t e[3];
  readCmd(0x010D, nullptr, 0, e, 2);
  out->printf("errors: %02x%02x\r\n", e[1], e[2]);
  printStatus("status", getStatus());
}

void testBusy() {
  standbyRc();
  out->print("busy (us after NSS high), from STBY_RC:\r\n");
  uint8_t x1 = 1, x0 = 0, pt = 2;
  out->printf("  GetStatus        %ld\r\n", cmd(0x0100));
  out->printf("  SetStandby XOSC  %ld\r\n", cmd(0x011C, &x1, 1));
  out->printf("  SetStandby RC    %ld\r\n", cmd(0x011C, &x0, 1));
  out->printf("  SetPacketType    %ld\r\n", cmd(0x020E, &pt, 1));
  uint8_t mod[4] = {8, 0x03, 1, 0};
  out->printf("  SetModulation    %ld\r\n", cmd(0x020F, mod, 4));
  uint8_t pkt[6] = {0, 32, 0, 0xFF, 1, 0};
  out->printf("  SetPacketParams  %ld\r\n", cmd(0x0210, pkt, 6));
  uint8_t buf[255];
  for (int i = 0; i < 255; i++) buf[i] = i;
  out->printf("  WriteBuffer8 255 %ld\r\n", cmd(0x0109, buf, 255));
  out->printf("  SetFs            %ld\r\n", cmd(0x011D));
  uint8_t rx[3] = {0xFF, 0xFF, 0xFF};
  out->printf("  SetRx continu    %ld\r\n", cmd(0x0209, rx, 3));
  printStatus("apres SetRx", getStatus());
  out->printf("  SetStandby RC    %ld (depuis RX)\r\n", cmd(0x011C, &x0, 1));
  uint8_t img[2] = {0xD7, 0xDB};
  out->printf("  CalibImage       %ld\r\n", cmd(0x0111, img, 2));
  uint8_t cal = 0x3F;
  out->printf("  Calibrate 3F     %ld\r\n", cmd(0x010F, &cal, 1));
  printStatus("fin", getStatus());
}

void testCmdDuringBusy() {
  standbyRc();
  uint8_t img[2] = {0xD7, 0xDB}, x1 = 1;
  uint8_t ci[4] = {0x01, 0x11, img[0], img[1]};
  xfer(ci, nullptr, 4, true, false);               // CalibImage (≈ 6.5 ms), without waiting for BUSY low
  delayMicroseconds(100);
  bool was_busy = busy();
  uint8_t tx[3] = {0x01, 0x1C, x1};
  xfer(tx, nullptr, 3, false, false);              // SetStandby XOSC sent while BUSY is high
  long t = waitBusyLow();
  Status s = getStatus();
  out->printf("cmdbusy: busy_au_moment=%d attente=%ld us -> %s\r\n", was_busy, t,
              ((s.s2 >> 1) & 7) == 2 ? "commande EXECUTEE (STBY_XOSC)" : "commande PERDUE");
  printStatus("status", s);
}

void testSleep(bool warm) {
  standbyRc();
  clearIrq();
  uint8_t pt = 2;
  cmd(0x020E, &pt, 1);
  writeBuf(0x40, 16);
  out->printf("sleep %s: avant: type=%u tampon=%d/16 irq_line=%d\r\n", warm ? "chaud" : "froid", packetType(),
              bufMatches(0x40, 16), irqLine());
  for (int dios = 0; dios < 2; dios++) {
    uint8_t d = dios;
    cmd(0x012A, &d, 1);                            // DriveDiosInSleepMode
    uint8_t p[5] = {(uint8_t)(warm ? 1 : 0), 0, 0, 0, 0};
    uint8_t tx[7] = {0x01, 0x1B, p[0], 0, 0, 0, 0};
    xfer(tx, nullptr, 7, true, false);             // SetSleep: BUSY stays high
    delay(20);
    out->printf("  en sommeil (DriveDios=%d): busy=%d irq_line=%d\r\n", dios, busy(), irqLine());
    // wake with a NOP transaction
    uint8_t nop[2] = {0, 0};
    unsigned long t0 = micros();
    xfer(nop, nullptr, 2, false, false);
    long t = waitBusyLow(100000);
    out->printf("  reveil par NOP: BUSY bas apres %ld us (depuis NSS: %lu)\r\n", t, micros() - t0);
    Status s = getStatus();
    printStatus("apres reveil", s);
    out->printf("  type=%u tampon=%d/16\r\n", packetType(), bufMatches(0x40, 16));
    cmd(0x020E, &pt, 1);
    writeBuf(0x40, 16);
  }
  // wake with a real command: SetStandby XOSC as the waking transaction
  uint8_t tx[7] = {0x01, 0x1B, (uint8_t)(warm ? 1 : 0), 0, 0, 0, 0};
  xfer(tx, nullptr, 7, true, false);
  delay(20);
  uint8_t sx[3] = {0x01, 0x1C, 0x01};
  xfer(sx, nullptr, 3, false, false);
  long t = waitBusyLow(100000);
  Status s = getStatus();
  out->printf("  reveil par SetStandby(XOSC): BUSY bas apres %ld us -> %s\r\n", t,
              ((s.s2 >> 1) & 7) == 2 ? "commande EXECUTEE" : "commande PERDUE");
  printStatus("status", s);
}

void testFallback() {
  for (uint8_t m = 1; m <= 3; m++) {
    standbyRc();
    clearIrq();
    cmd(0x0213, &m, 1);
    uint8_t rx[3] = {0, 0x01, 0x48};               // timeout 328 ticks = 10 ms
    cmd(0x0209, rx, 3);
    delay(40);
    Status s = getStatus();
    out->printf("fallback=%u, apres timeout RX:", m);
    printStatus("", s);
  }
  uint8_t m = 1;
  cmd(0x0213, &m, 1);
}

uint32_t ticks(float ms) { return (uint32_t)(ms * 32.768f + 0.5f); }

void startDuty(float rx_ms, float sl_ms) {
  standbyRc();
  clearIrq();
  uint8_t irq[8] = {0, 0, 0x04, 0xC8, 0, 0, 0, 0};    // RxDone|HeaderErr|CrcErr|Timeout -> DIO9 (not preamble/header)
  cmd(0x0113, irq, 8);
  uint32_t r = ticks(rx_ms), s = ticks(sl_ms);
  uint8_t p[7] = {(uint8_t)(r >> 16), (uint8_t)(r >> 8), (uint8_t)r, (uint8_t)(s >> 16), (uint8_t)(s >> 8), (uint8_t)s, 0};
  cmd(0x0214, p, 7, true);
}

void testDuty(unsigned secs, float rx_ms, float sl_ms) {
  out->printf("duty: rx=%lu us sleep=%lu us, attente d'une trame %u s\r\n", (unsigned long)(rx_ms * 1000), (unsigned long)(sl_ms * 1000), secs);
  startDuty(rx_ms, sl_ms);
  busyChronogram(400);
  unsigned long t0 = millis();
  while (!irqLine()) {
    if (millis() - t0 > secs * 1000UL) { out->print("  aucune trame recue\r\n"); return; }
    delay(1);
  }
  unsigned long t_irq = micros();
  out->printf("  IRQ apres %lu ms; BUSY au moment de l'IRQ=%d\r\n", millis() - t0, busy());
  busyChronogram(200, 12);
  out->printf("  200 ms apres l'IRQ: busy=%d irq_line=%d (dt=%lu us)\r\n", busy(), irqLine(), micros() - t_irq);
  if (busy()) {
    out->print("  BUSY haut : la puce dort -> reveil par NOP\r\n");
    uint8_t nop[2] = {0, 0};
    xfer(nop, nullptr, 2, false, false);
    out->printf("  BUSY bas apres %ld us\r\n", waitBusyLow());
  }
  printStatus("status", getStatus());
  uint8_t r[3];
  readCmd(0x0203, nullptr, 0, r, 2);
  uint8_t ps[4];
  readCmd(0x0204, nullptr, 0, ps, 3);
  out->printf("  rx_len=%u rx_off=%u rssi=-%u/2 snr=%d/4\r\n", r[1], r[2], ps[1], (int8_t)ps[2]);
  uint8_t rb[2] = {r[2], 8}, d1[9], d2[9];
  readCmd(0x010A, rb, 2, d1, 8);
  out->printf("  tampon RX: %02x %02x %02x %02x %02x %02x %02x %02x\r\n", d1[1], d1[2], d1[3], d1[4], d1[5], d1[6], d1[7], d1[8]);
  busyChronogram(300, 12);                          // does the cycle go on after RxDone?
  printStatus("300 ms plus tard", getStatus());
  // does the RX buffer survive a warm sleep (UM §10.3: data buffer cleared in sleep)?
  uint8_t sl[7] = {0x01, 0x1B, 1, 0, 0, 0, 0};
  xfer(sl, nullptr, 7, true, false);
  delay(20);
  uint8_t nop[2] = {0, 0};
  xfer(nop, nullptr, 2, false, false);
  waitBusyLow();
  readCmd(0x010A, rb, 2, d2, 8);
  int same = 0;
  for (int i = 1; i <= 8; i++) same += d1[i] == d2[i];
  out->printf("  tampon RX apres sommeil chaud: %02x %02x %02x %02x ... (%d/8 identiques)\r\n", d2[1], d2[2], d2[3], d2[4], same);
}

void testDutyNss(float rx_ms, float sl_ms) {
  startDuty(rx_ms, sl_ms);
  delay(5);
  unsigned long t0 = millis();
  while (!busy() && millis() - t0 < 500) {}       // wait for a sleep phase
  out->printf("duty+NSS: en sommeil=%d\r\n", busy());
  uint8_t nop[2] = {0, 0};
  xfer(nop, nullptr, 2, false, false);
  out->printf("  NOP pendant le sommeil: BUSY bas apres %ld us\r\n", waitBusyLow());
  printStatus("status", getStatus());
  busyChronogram(300, 12);                          // cycle stopped (BUSY stays low) or still running?
  printStatus("300 ms plus tard", getStatus());
}

void testDutyBuffer(float rx_ms, float sl_ms) {
  startDuty(rx_ms, sl_ms);
  waitBusyLow();                                    // in an RX window
  writeBuf(0x60, 16);
  out->printf("duty+tampon: ecrit pendant une fenetre; relu aussitot=%d/16\r\n", bufMatches(0x60, 16));
  delay(300);                                       // several sleep phases
  waitBusyLow();
  standbyRc();
  out->printf("  apres 300 ms de cycle puis SetStandby: tampon=%d/16 (16 = conserve, 0 = efface)\r\n",
              bufMatches(0x60, 16));
  printStatus("status", getStatus());
}

}  // namespace

void t1000aLrProbe(Print& o, int argc, char** argv) {
  out = &o;
  const char* t = argc > 1 ? argv[1] : "";
  float rx = argc > 3 ? atof(argv[3]) : 36.8f, sl = argc > 4 ? atof(argv[4]) : 59.5f;
  if (!strcmp(t, "ver")) testVersion();
  else if (!strcmp(t, "busy")) testBusy();
  else if (!strcmp(t, "cmdbusy")) testCmdDuringBusy();
  else if (!strcmp(t, "sleep")) testSleep(argc < 3 || strcmp(argv[2], "cold"));
  else if (!strcmp(t, "fallback")) testFallback();
  else if (!strcmp(t, "duty")) testDuty(argc > 2 ? atoi(argv[2]) : 120, rx, sl);
  else if (!strcmp(t, "dutynss")) testDutyNss(argc > 2 ? atof(argv[2]) : 36.8f, argc > 3 ? atof(argv[3]) : 59.5f);
  else if (!strcmp(t, "dutybuf")) testDutyBuffer(argc > 2 ? atof(argv[2]) : 36.8f, argc > 3 ? atof(argv[3]) : 59.5f);
  else {
    o.print("lr ver | busy | cmdbusy | sleep [warm|cold] | fallback | duty [s] [rx_ms sl_ms] | dutynss | dutybuf\r\n");
    return;
  }
  o.print("lr: fin, redemarrage pour rendre la radio a MeshCore\r\n");
  delay(500);
  gr551x_reboot();
}
#endif
