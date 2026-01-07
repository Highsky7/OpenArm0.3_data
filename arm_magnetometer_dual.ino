#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>
// #include <avr/wdt.h>   // (옵션) MEGA에서 워치독 쓸 때 주석 해제

/* ===== I2C addresses ===== */
#define TCA_ADDR       0x70
#define TCA2_ADDR      0x71 // 새 TCA 주소 추가
#define AS5600_ADDR    0x36
#define RAW_ANGLE_REG  0x0C

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 230400   // 가능하면 230400 또는 250000으로 올리면 더 안정
#endif

/* ===== Channels / constants ===== */
const uint8_t kChannels[] = {0,1,2,3,4,5,6,7};
const int NUM_CH = sizeof(kChannels) / sizeof(kChannels[0]);

/* 사용자 플로우 임계값 (반바퀴 근처) */
const int THRESH = 2000;

/* 캘리 키 */
const char KEY_CAPTURE = 'c';
const int SPECIAL_OFFSET_IDX = 3 + NUM_CH;  // CH3 런타임 zero 대상

Servo g_servo;       // 전역
Servo g_servo2;      // 추가
const int SERVO_PIN = 9;
const int SERVO2_PIN = 10;   // 두 번째 서보 핀 번호 추가

int CH7_IDX = -1;    // kChannels[]에서 값이 7인 인덱스(보통 7)
float g_servo_deg = 0.0f;   // 마지막으로 보낸 각도(0~60)
float g_servo2_deg = 0.0f;  // 두 번째 서보 각도 변수 추가
bool  g_servo_has = false;  // 유효값 보낸 적 있는지
bool  g_servo2_has = false; // 두 번째 서보 유효값 변수 추가
// === Servo smoothing & rate-limit settings ===
const uint16_t SERVO_PERIOD_MS = 25;   // 서보 갱신 주기(<= 50Hz). 25ms = 40Hz
const float    SERVO_EMA_ALPHA = 0.20; // (미사용) 기존 상수 유지
const float    SERVO_SLEW_DPS  = 300.0f; // (미사용)
const float    SERVO_EPS_DEG   = 0.2f;   // (미사용)

/* 마지막 보낸 각도(출력용, 소수점 2자리 유지) */
static float sent_deg = 0.0f;
static float sent_deg2 = 0.0f; // 두 번째 서보 출력용 각도 변수 추가

/* ===== Cal data (모든 함수보다 위) ===== */
struct __attribute__((packed)) Cal {
  uint16_t min_raw, max_raw;   // 캘리시에 "원시 RAW"로 캡처
  uint16_t center;             // eff 기준 center (0..4095)
  int16_t  offset_counts;      // 런타임 제로/위상 오프셋(카운트)
  uint8_t  mirrored;           // 0: raw, 1: 4096-raw
  uint8_t  sign_neg;           // 0/1: 증감 방향 반전(출력 부호)
  uint8_t  ready;              // 캘리 완료
  uint8_t  enabled;            // 채널 활성
  uint8_t  case_code;          // 11/12/21/22 디버깅
  uint8_t  tca_addr;           // TCA 주소 저장
};

struct __attribute__((packed)) CalStoreV2 {
  uint16_t center;
  int16_t  offset_counts;
  uint8_t  mirrored;
  uint8_t  enabled_with_sign;  // bit0: enabled, bit1: sign_neg
  uint8_t  tca_addr;           // TCA 주소 저장
};

Cal cal[NUM_CH * 2];  // 2배로 확장
const uint8_t kChannels2[] = {0,1,2,3,4,5,6,7}; // 두 번째 TCA 채널

/* ===== EEPROM ===== */
const uint8_t EEPROM_MAGIC = 0x45;  // 새 매직
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_DATA_ADDR  = EEPROM_MAGIC_ADDR + 1;
const int EEPROM_BYTES_NEEDED = EEPROM_DATA_ADDR + (NUM_CH * 2) * (int)sizeof(CalStoreV2);

/* ===== Helpers ===== */
static inline int16_t nearDiffCounts(int16_t a_minus_b){
  if (a_minus_b >  2048) a_minus_b -= 4096;
  if (a_minus_b <= -2048) a_minus_b += 4096;
  return a_minus_b;
}
static inline uint16_t mirror4096(uint16_t raw){
  uint16_t t = (uint16_t)(4096 - raw);
  return (uint16_t)(t & 0x0FFF);
}
static inline float countsToRad(int32_t c){
  return (2.0f * PI * (float)c) / 4096.0f;
}
static inline float wrapPi(float x){
  while (x >  PI) x -= 2.0f*PI;
  while (x <= -PI) x += 2.0f*PI;
  return x;
}
static inline uint16_t modsub12(uint16_t a, uint16_t b){
  return (uint16_t)((a - b) & 0x0FFF);
}
static inline void unwrap_pair_180(uint16_t a, uint16_t b, uint32_t &a_lin, uint32_t &b_lin){
  uint16_t diff = (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
  a_lin = a; b_lin = b;
  if (diff >= 2048){
    if (a < b) a_lin = (uint32_t)a + 4096u;
    else       b_lin = (uint32_t)b + 4096u;
  }
}
static inline uint32_t unwrap_to_band_180(uint16_t v, uint32_t ref){
  uint32_t vv = v;
  if (ref >= vv){
    if ((ref - vv) >= 2048) vv += 4096u;
  } else {
    if ((vv - ref) >= 2048) vv -= 4096u;
  }
  return vv;
}
static inline int indexOfChannel(uint8_t ch){
  for (int i=0;i<NUM_CH;i++){
    if (kChannels[i] == ch) return i;
  }
  return -1;
}

static inline int indexOfTcaChannel(uint8_t tca_addr, uint8_t ch){
  if (tca_addr == TCA_ADDR){
    for (int i=0;i<NUM_CH;i++){
      if (kChannels[i] == ch) return i;
    }
  } else if (tca_addr == TCA2_ADDR){
    for (int i=0;i<NUM_CH;i++){
      if (kChannels2[i] == ch) return i + NUM_CH;
    }
  }
  return -1;
}

/* ===== I2C init / recover ===== */

// TCA 채널 캐시
static int8_t g_last_tca = -1;
static uint8_t g_last_ch = 0xFF; // TCA 주소와 채널 모두 캐싱

void i2cInit(){
  Wire.begin();
  #if defined(TWBR)
    // AVR(Arduino Mega 등)에서 100kHz
    TWBR = ((F_CPU / 100000L) - 16) / 2;
  #endif
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000 /*us*/, true /*reset on timeout*/);
  #endif
}

void i2cRecover(){
  // TCA off(아무 채널도 선택하지 않음)
  Wire.beginTransmission(TCA_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
  Wire.beginTransmission(TCA2_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();

  delay(2);

  // AVR TWI 하드 리셋
  #if defined(TWCR) && defined(TWEN)
    TWCR &= ~(_BV(TWEN));   // disable
    delayMicroseconds(20);
    TWCR |= _BV(TWEN);      // enable
  #endif

  i2cInit();
  g_last_tca = -1; 
  g_last_ch = 0xFF;
  // 캐시 무효화
}

/* ===== 라인 단위 청크 전송 ===== */
// 최대 budget_us 안에 줄 전체를 못 보내면 false(그 프레임 드롭)
bool writeLineChunked(const char* line, size_t len, uint32_t budget_us = 3000) {
  uint32_t start = micros();
  size_t off = 0;

  while (off < len) {
    int room = Serial.availableForWrite();
    if (room > 0) {
      size_t to_write = (size_t)room;
      if (to_write > (len - off)) to_write = (len - off);
      Serial.write((const uint8_t*)(line + off), to_write);
      off += to_write;
    } else {
      delayMicroseconds(150);
    }
    if ((micros() - start) > budget_us) return false; // 시간 초과 → 드롭
  }
  return true;
}

/* ===== I2C read (TCA 캐시 + 재시도 + 실패 누적 복구) ===== */
static inline void tcaSelect(uint8_t tca_addr, uint8_t ch){
  if (g_last_tca == (int8_t)tca_addr && g_last_ch == ch) return;
  Wire.beginTransmission(tca_addr);
  Wire.write((uint8_t)(1u << ch));
  Wire.endTransmission();
  g_last_tca = (int8_t)tca_addr;
  g_last_ch = ch;
  delayMicroseconds(150);
}

static uint8_t g_i2c_fail_streak = 0;

static inline bool readAS5600Raw(uint16_t &outRaw){
  for (uint8_t attempt=0; attempt<3; ++attempt){
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(RAW_ANGLE_REG);
    if (Wire.endTransmission(false) == 0){
      if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) == 2){
        uint16_t hi = Wire.read(), lo = Wire.read();
        outRaw = (uint16_t)(((hi << 8) | lo) & 0x0FFF);
        g_i2c_fail_streak = 0;
        return true;
      }
    }
    delayMicroseconds(150);
  }
  if (++g_i2c_fail_streak >= 10){
    i2cRecover();
    g_i2c_fail_streak = 0;
  }
  return false;
}

// 2개의 TCA 주소를 인자로 받는 함수로 수정
static inline int readRawOnChannelIndex(int idx){
  uint8_t tca_addr = (idx < NUM_CH) ? TCA_ADDR : TCA2_ADDR;
  uint8_t ch = (idx < NUM_CH) ? kChannels[idx] : kChannels2[idx - NUM_CH];
  tcaSelect(tca_addr, ch);
  uint16_t raw;
  if (!readAS5600Raw(raw)) return -1;
  return (int)raw;
}

/* ===== EEPROM helpers ===== */
void eepromBeginIfNeeded(uint16_t bytes){
#if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(bytes);
#else
  (void)bytes;
#endif
}
void eepromCommitIfNeeded(){
#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif
}
void saveToEEPROM(){
  eepromBeginIfNeeded(EEPROM_BYTES_NEEDED);
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  for (int i=0;i<NUM_CH * 2;i++){ // 2배로 확장
    CalStoreV2 cs;
    cs.center        = cal[i].center;
    cs.offset_counts = cal[i].offset_counts;
    cs.mirrored      = cal[i].mirrored;
    uint8_t b = 0;
    if (cal[i].enabled)  b |= 0x01;
    if (cal[i].sign_neg) b |= 0x02;
    cs.enabled_with_sign = b;
    cs.tca_addr      = cal[i].tca_addr;
    EEPROM.put(EEPROM_DATA_ADDR + i*(int)sizeof(CalStoreV2), cs);
  }
  eepromCommitIfNeeded();
  Serial.println(F("[EEPROM] saved."));
}
bool loadFromEEPROM(){
  eepromBeginIfNeeded(EEPROM_BYTES_NEEDED);
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) return false;
  for (int i=0;i<NUM_CH * 2;i++){ // 2배로 확장
    CalStoreV2 cs;
    EEPROM.get(EEPROM_DATA_ADDR + i*(int)sizeof(CalStoreV2), cs);
    cal[i].center        = cs.center;
    cal[i].offset_counts = cs.offset_counts;
    cal[i].mirrored      = cs.mirrored;
    cal[i].enabled       = (cs.enabled_with_sign & 0x01) ? 1 : 0;
    cal[i].sign_neg      = (cs.enabled_with_sign & 0x02) ? 1 : 0;
    cal[i].ready         = cal[i].enabled;
    cal[i].case_code     = 0;
    cal[i].tca_addr      = cs.tca_addr;
  }
  Serial.println(F("[EEPROM] loaded."));
  return true;
}
void clearEEPROM(){
  eepromBeginIfNeeded(EEPROM_BYTES_NEEDED);
  for (int i=0;i<EEPROM_BYTES_NEEDED;i++) EEPROM.write(i, 0xFF);
  eepromCommitIfNeeded();
  Serial.println(F("[EEPROM] cleared. Power-cycle to recalibrate."));
}

/* ===== Probe channels ===== */
void probeActiveChannels(){
  Serial.println(F("[CAL] Probing channels..."));
  for (int i=0;i<NUM_CH * 2;i++){ // 2배로 확장
    bool ok=false;
    for (uint8_t t=0;t<3;t++){
      int r = readRawOnChannelIndex(i);
      if (r>=0){ ok=true; break; }
      delay(5);
    }
    cal[i].enabled = ok ? 1 : 0;
    cal[i].ready   = 0;
    cal[i].offset_counts = 0;
    cal[i].mirrored = 0;
    cal[i].sign_neg = 0;
    cal[i].case_code = 0;
    cal[i].tca_addr = (i < NUM_CH) ? TCA_ADDR : TCA2_ADDR;
    Serial.print(F(" - CH")); Serial.print( (i<NUM_CH) ? kChannels[i] : kChannels2[i-NUM_CH] );
    Serial.print(F(" (TCA ")); Serial.print(cal[i].tca_addr, HEX); Serial.print(F(")"));
    Serial.println(ok ? F(": ACTIVE") : F(": INACTIVE"));
  }
}

/* ===== 사용자 플로우(11/12/21/22) ===== */
void computeCenter_UserFlow(Cal &c){
  uint16_t minr = c.min_raw;
  uint16_t maxr = c.max_raw;

  if (minr > maxr){
    uint16_t diff = (uint16_t)(minr - maxr);
    if (diff > THRESH){
      // 1-1
      uint32_t min_t = 4096u - (uint32_t)minr;
      uint32_t max_t = 4096u - (uint32_t)maxr;
      uint32_t mid   = (min_t + max_t) >> 1;
      c.center   = (uint16_t)(mid & 0x0FFF);
      c.mirrored = 1;
      c.case_code= 11;
    }else{
      // 1-2
      uint32_t mid = ((uint32_t)minr + ((uint32_t)maxr + 4096u)) >> 1;
      c.center   = (uint16_t)(mid & 0x0FFF);
      c.mirrored = 0;
      c.case_code= 12;
    }
  }else{
    uint16_t diff = (uint16_t)(maxr - minr);
    if (diff > THRESH){
      // 2-1
      uint32_t mid = ((uint32_t)minr + (uint32_t)maxr) >> 1;
      c.center   = (uint16_t)(mid & 0x0FFF);
      c.mirrored = 0;
      c.case_code= 21;
    }else{
      // 2-2
      uint32_t min_t = 4096u - (uint32_t)minr;
      uint32_t max_t = (4096u - (uint32_t)maxr) + 4096u;
      uint32_t mid   = (min_t + max_t) >> 1;
      c.center   = (uint16_t)(mid & 0x0FFF);
      c.mirrored = 1;
      c.case_code= 22;
    }
  }
  c.ready = 1;
}

/* ===== (변경) 캘리: 특정 인덱스만 또는 전체 ===== */
void captureMinMaxInSetup(int only_idx = -1){
  const bool do_all = (only_idx < 0);

  if (do_all) {
    Serial.println(F("\n[CAL] ===== Start (ALL) ====="));
    Serial.println(F("[CAL] 각 채널: (일반) MIN→'c', MAX→'c'  /  (CH6/7) MIN→'c', MAX→'c', ORIGIN→'c'"));
  } else {
    uint8_t print_ch = (only_idx < NUM_CH) ? kChannels[only_idx]
                                           : kChannels2[only_idx - NUM_CH];
    Serial.print(F("\n[CAL] ===== Start (CH"));
    Serial.print(print_ch);
    Serial.println(F(" only) ====="));
    Serial.println(F("[CAL] 대상 채널만: (일반) MIN→'c', MAX→'c'  /  (CH6/7) MIN→'c', MAX→'c', ORIGIN→'c'"));
  }

  for (int i=0;i<NUM_CH * 2;i++){
    if (!do_all && i!=only_idx) continue;

    if (!cal[i].enabled){
      if (do_all) continue;
      Serial.println(F("[CAL] 대상 채널 비활성 상태"));
      break;
    }

    const uint8_t ch = (i<NUM_CH) ? kChannels[i] : kChannels2[i-NUM_CH];
    const uint8_t tca_addr = (i<NUM_CH) ? TCA_ADDR : TCA2_ADDR;

    // ---------- CH6 / CH7: 특수 3-스텝 (MIN→MAX→ORIGIN) ----------
    if (ch == 6 || ch == 7){
      // ===== MIN =====
      Serial.print(F("[CAL] CH")); Serial.print(ch);
      Serial.print(F(" (TCA ")); Serial.print(tca_addr, HEX); Serial.println(F(")"));
      Serial.println(F(" → MIN 위치로 이동 후 'c' 입력"));
      uint16_t min_raw=0, max_raw=0, org_raw=0;
      while(true){
        if (!Serial.available()){ delay(1); continue; }
        char k = (char)Serial.read();
        if (k=='\r'||k=='\n') continue;
        if (k=='R'||k=='r'){ clearEEPROM(); continue; }
        if (k!=KEY_CAPTURE) continue;
        int r = readRawOnChannelIndex(i);
        if (r>=0){ min_raw=(uint16_t)r; Serial.print(F("[CAL] CH")); Serial.print(ch);
          Serial.print(F(" MIN RAW=")); Serial.println(min_raw); break; }
        else { Serial.println(F("[ERR] read fail, press 'c' again")); }
      }

      // ===== MAX =====
      Serial.print(F("[CAL] CH")); Serial.print(ch);
      Serial.println(F(" → MAX 위치로 이동 후 'c' 입력"));
      Serial.print(F(" (TCA ")); Serial.print(tca_addr, HEX); Serial.println(F(")"));
      while(true){
        if (!Serial.available()){ delay(1); continue; }
        char k = (char)Serial.read();
        if (k=='\r'||k=='\n') continue;
        if (k=='R'||k=='r'){ clearEEPROM(); continue; }
        if (k!=KEY_CAPTURE) continue;
        int r = readRawOnChannelIndex(i);
        if (r>=0){ max_raw=(uint16_t)r; Serial.print(F("[CAL] CH")); Serial.print(ch);
          Serial.print(F(" MAX RAW=")); Serial.println(max_raw); break; }
        else { Serial.println(F("[ERR] read fail, press 'c' again")); }
      }

      // --- 언랩(180° 기준) ---
      uint32_t min_lin=0, max_lin=0;
      unwrap_pair_180(min_raw, max_raw, min_lin, max_lin);

      // ===== ORIGIN(원점) =====
      Serial.print(F("[CAL] CH")); Serial.print(ch);
      Serial.print(F(" (TCA ")); Serial.print(tca_addr, HEX); Serial.println(F(")"));
      Serial.println(F(" → ORIGIN(원점) 위치로 이동 후 'c' 입력"));
      while(true){
        if (!Serial.available()){ delay(1); continue; }
        char k = (char)Serial.read();
        if (k=='\r'||k=='\n') continue;
        if (k=='R'||k=='r'){ clearEEPROM(); continue; }
        if (k!=KEY_CAPTURE) continue;
        int r = readRawOnChannelIndex(i);
        if (r>=0){ org_raw=(uint16_t)r; Serial.print(F("[CAL] CH")); Serial.print(ch);
          Serial.print(F(" ORIGIN RAW=")); Serial.println(org_raw); break; }
        else { Serial.println(F("[ERR] read fail, press 'c' again")); }
      }

      // ORIGIN도 같은 밴드로 언랩
      uint32_t mid_lin = (min_lin + max_lin) >> 1;
      uint32_t org_lin = unwrap_to_band_180(org_raw, mid_lin);

      // 방향 판정
      cal[i].sign_neg = (min_lin > max_lin) ? 1 : 0;

      // 6/7에서는 미러 사용 안 함
      cal[i].mirrored = 0;

      // 저장
      cal[i].center        = (uint16_t)(org_lin & 0x0FFF);
      cal[i].offset_counts = 0;
      cal[i].min_raw       = (uint16_t)(min_lin & 0x0FFF);
      cal[i].max_raw       = (uint16_t)(max_lin & 0x0FFF);
      cal[i].case_code     = (ch==6) ? 61 : 71;
      cal[i].ready         = 1;

      // 요약
      uint32_t gap_lin = (min_lin > max_lin) ? (min_lin - max_lin) : (max_lin - min_lin);
      Serial.print(F("[CAL] CH")); Serial.print(ch);
      Serial.print(F(" (TCA ")); Serial.print(tca_addr, HEX); Serial.print(F(")"));
      Serial.print(F(" (6/7 special) case=")); Serial.print(cal[i].case_code);
      Serial.print(F(" center(origin)=")); Serial.print(cal[i].center);
      Serial.print(F(" mirrored=0"));
      Serial.print(F(" sign_neg=")); Serial.print(cal[i].sign_neg ? 1 : 0);
      Serial.print(F(" offset_counts=0"));
      Serial.print(F(" gap_lin=")); Serial.print((uint16_t)(gap_lin & 0x0FFF));
      Serial.println();
    }
    else{
      // ---------- 그 외 채널: 기존 MIN→MAX 후 11/12/21/22 ----------
      // ===== MIN =====
      Serial.print(F("[CAL] CH")); Serial.print(ch);
      Serial.print(F(" (TCA ")); Serial.print(tca_addr, HEX); Serial.println(F(")"));
      Serial.println(F(" → MIN 위치로 이동 후 'c' 입력"));
      while(true){
        if (!Serial.available()){ delay(1); continue; }
        char k = (char)Serial.read();
        if (k=='\r' || k=='\n') continue;
        if (k=='R' || k=='r'){ clearEEPROM(); continue; }
        if (k!=KEY_CAPTURE) continue;

        int raw = readRawOnChannelIndex(i);
        if (raw>=0){
          cal[i].min_raw = (uint16_t)raw;
          Serial.print(F("[CAL] CH")); Serial.print(ch);
          Serial.print(F(" (TCA ")); Serial.print(tca_addr, HEX); Serial.print(F(")"));
          Serial.print(F(" MIN RAW=")); Serial.println(raw);
          break;
        } else {
          Serial.println(F("[ERR] read fail, press 'c' again"));
        }
      }

      // ===== MAX =====
      Serial.print(F("[CAL] CH")); Serial.print(ch);
      Serial.print(F(" (TCA ")); Serial.print(tca_addr, HEX); Serial.println(F(")"));
      Serial.println(F(" → MAX 위치로 이동 후 'c' 입력"));
      while(true){
        if (!Serial.available()){ delay(1); continue; }
        char k = (char)Serial.read();
        if (k=='\r' || k=='\n') continue;
        if (k=='R' || k=='r'){ clearEEPROM(); continue; }
        if (k!=KEY_CAPTURE) continue;

        int raw = readRawOnChannelIndex(i);
        if (raw>=0){
          cal[i].max_raw = (uint16_t)raw;
          Serial.print(F("[CAL] CH")); Serial.print(ch);
          Serial.print(F(" (TCA ")); Serial.print(tca_addr, HEX); Serial.print(F(")"));
          Serial.print(F(" MAX RAW=")); Serial.println(raw);
          break;
        } else {
          Serial.println(F("[ERR] read fail, press 'c' again"));
        }
      }

      // 4케이스 처리
      computeCenter_UserFlow(cal[i]);

      // 사용자 지정 예외 (원래 코드 유지)
      if (tca_addr == TCA_ADDR && ch == 1){ cal[i].sign_neg = 1;
        cal[i].offset_counts = nearDiffCounts((int16_t)(cal[i].offset_counts + 2048)); }
      if (tca_addr == TCA_ADDR && ch == 3){ cal[i].sign_neg = 1;
}

      // 요약
      uint16_t gap_lin = (cal[i].min_raw > cal[i].max_raw)
                        ? (uint16_t)(cal[i].min_raw - cal[i].max_raw)
                        : (uint16_t)(cal[i].max_raw - cal[i].min_raw);
      Serial.print(F("[CAL] CH")); Serial.print(ch);
      Serial.print(F(" (TCA ")); Serial.print(tca_addr, HEX); Serial.print(F(")"));
      Serial.print(F(" case=")); Serial.print(cal[i].case_code);
      Serial.print(F(" center=")); Serial.print(cal[i].center);
      Serial.print(F(" mirrored=")); Serial.print(cal[i].mirrored ? 1 : 0);
      Serial.print(F(" sign_neg=")); Serial.print(cal[i].sign_neg ? 1 : 0);
      Serial.print(F(" offset_counts=")); Serial.print(cal[i].offset_counts);
      Serial.print(F(" gap(|max-min|)=")); Serial.print(gap_lin);
      Serial.println();
    }

    if (!do_all) break;
  }

  // CH3 런타임 제로 단계 — 전체 캘리일 때만 표시
  if (do_all && cal[SPECIAL_OFFSET_IDX].enabled){
    Serial.println(F("\n[CAL] CH3 Zero-Offset: ZERO 포즈 만든 뒤 'z'=저장 / 's'=건너뛰기"));
    while(true){
      if (!Serial.available()){ delay(1); continue; }
      char k=(char)Serial.read();
      if (k=='\r' || k=='\n') continue;
      if (k=='R' || k=='r'){ clearEEPROM(); continue; }

      if (k=='z'){
        int raw = readRawOnChannelIndex(SPECIAL_OFFSET_IDX);
        if (raw>=0){
          uint16_t eff = cal[SPECIAL_OFFSET_IDX].mirrored ? mirror4096((uint16_t)raw) : (uint16_t)raw;
          int16_t d0   = nearDiffCounts((int16_t)eff - (int16_t)cal[SPECIAL_OFFSET_IDX].center);
          if (cal[SPECIAL_OFFSET_IDX].sign_neg) d0 = (int16_t)(-d0);
          cal[SPECIAL_OFFSET_IDX].offset_counts = nearDiffCounts((int16_t)(-d0));
          saveToEEPROM();
          Serial.println(F("[CH3] runtime zero saved"));
        } else {
          Serial.println(F("[ERR] CH3 read fail"));
        }
      } else if (k=='s'){
        Serial.println(F("[CAL] CH3 zero skipped"));
        break;
      }
    }
  }

  saveToEEPROM();
  Serial.println(do_all ? F("[CAL] ===== End (ALL) =====\n")
                        : F("[CAL] ===== End (ONE) =====\n"));
}

/* ===== 런타임 변환 (오프셋 + 부호 반전 포함) ===== */
float countsToRadRuntime(const Cal& c, uint16_t raw_now){
  uint16_t eff = c.mirrored ? mirror4096(raw_now) : raw_now;
  int16_t  d   = nearDiffCounts((int16_t)eff - (int16_t)c.center);
  if (c.sign_neg) d = (int16_t)(-d);
  d = nearDiffCounts((int16_t)(d + c.offset_counts));
  return wrapPi(countsToRad(d));
}

/* ===== runtime state ===== */
int16_t last_dcnt[NUM_CH * 2]; // 2배로 확장
uint16_t has_value_mask = 0;
// ★ 이전 출력 유지용 버퍼(초기 0.0)
float last_out[NUM_CH * 2] = {0.0f}; // 2배로 확장

/* ===== Arduino ===== */
void setup(){
  Serial.begin(SERIAL_BAUD);  // 필요시 230400/250000으로 올려도 OK (수신측도 동일)
  i2cInit();     // <<< 안정화 초기화

  Serial.println(F("@,r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,s1,s2"));
  Serial.println(F("R:clear  C|c[0-7|*|a]: recal  Z:CH3 runtime zero"));

  if (!loadFromEEPROM()){
    probeActiveChannels();
    captureMinMaxInSetup(-1);   // 전체 최초 캘리
  }

  // === Servo attach ===
  g_servo.attach(SERVO_PIN, 500, 2500);
  g_servo.writeMicroseconds(500);
  g_servo2.attach(SERVO2_PIN, 500, 2500);
  g_servo2.writeMicroseconds(500);
  CH7_IDX = indexOfChannel(7);
  Serial.print(F("[SERVO] CH7_IDX=")); Serial.println(CH7_IDX);

  for (int i=0;i<NUM_CH * 2;i++){ last_dcnt[i]=0; }
  has_value_mask = 0;

  // (옵션) 워치독
  // wdt_disable();
  // wdt_enable(WDTO_250MS);
}

void loop(){
  // wdt_reset(); // (옵션) 워치독 사용시 활성화

  /* ===== 명령 처리 ===== */
  while (Serial.available()){
    char c = (char)Serial.read();
    if (c=='\r' || c=='\n') continue;

    if (c=='R' || c=='r'){
      clearEEPROM();
    } else if (c=='C' || c=='c'){
      while (Serial.peek() == ' ') Serial.read();
      int peekc = Serial.peek();

      int only_idx = -1; // 기본 전체
      if (peekc >= '0' && peekc <= '9'){
        int chval = Serial.read() - '0';
        int idx = indexOfTcaChannel(TCA_ADDR, (uint8_t)chval);
        if (idx < 0) idx = indexOfTcaChannel(TCA2_ADDR, (uint8_t)chval);
        if (idx >= 0) only_idx = idx;
        else {
          Serial.print(F("[CAL] Unknown channel value: ")); Serial.println(chval);
        }
      } else if (peekc=='*' || peekc=='a' || peekc=='A'){
        Serial.read(); // consume
        only_idx = -1; // 전체
      }

      if (only_idx >= 0 && !cal[only_idx].enabled){
        Serial.print(F("[CAL] CH")); Serial.print( (only_idx<NUM_CH) ? kChannels[only_idx] : kChannels2[only_idx-NUM_CH] );
        Serial.println(F(" is INACTIVE; probing first..."));
        probeActiveChannels();
        if (!cal[only_idx].enabled){
          Serial.println(F("[CAL] Still inactive. Abort."));
          continue;
        }
      }
      captureMinMaxInSetup(only_idx);
    } else if (c=='Z' || c=='z'){
      if (cal[SPECIAL_OFFSET_IDX].enabled && cal[SPECIAL_OFFSET_IDX].ready){
        int raw = readRawOnChannelIndex(SPECIAL_OFFSET_IDX);
        if (raw>=0){
          uint16_t eff = cal[SPECIAL_OFFSET_IDX].mirrored ? mirror4096((uint16_t)raw) : (uint16_t)raw;
          int16_t d0   = nearDiffCounts((int16_t)eff - (int16_t)cal[SPECIAL_OFFSET_IDX].center);
          if (cal[SPECIAL_OFFSET_IDX].sign_neg) d0 = (int16_t)(-d0);
          cal[SPECIAL_OFFSET_IDX].offset_counts = nearDiffCounts((int16_t)(-d0));
          saveToEEPROM();
          Serial.println(F("[CH3] runtime zero saved"));
        } else {
          Serial.println(F("[ERR] CH3 read fail"));
        }
      } else {
        Serial.println(F("[WARN] CH3 not ready"));
      }
    }
  }

  /* ===== 값 읽기 & 서보 갱신 & 라인 버퍼에 누적 (이전값 유지) ===== */
  char line[320];
  int  n = 0;

  // 헬퍼: 문자열 붙이기
  auto append_str = [&](const char* s){
    size_t L = strlen(s);
    if ((size_t)n + L >= sizeof(line)) L = sizeof(line) - 1 - n;
    memcpy(line + n, s, L);
    n += (int)L;
    line[n] = '\0';
  };

  // 헬퍼: 실수 → 문자열(고정 소수)로 변환하여 붙이기 (dtostrf 사용)
  auto append_float = [&](float v, uint8_t prec){
    char buf[24];
    dtostrf(v, 0, prec, buf);  // width=0, precision=prec
    append_str(buf);
  };

  append_str("@,");

  for (int i=0;i<NUM_CH * 2;i++){ // 2배로 확장
    // 기본값: 이전 출력 유지
    float th = last_out[i];

    if (cal[i].ready && cal[i].enabled){
      int rr = readRawOnChannelIndex(i);
      if (rr >= 0){
        uint16_t raw = (uint16_t)rr;

        // 라디안(로그용)
        th = countsToRadRuntime(cal[i], raw);
        has_value_mask |= (uint16_t)(1u<<i);

        // ---- CH7: min → 60°, max → 0° (단조 + 끝점 클램프) ----
        if (cal[i].tca_addr == TCA_ADDR && ( (i-NUM_CH) < 0 ? kChannels[i] : kChannels2[i-NUM_CH] ) == 7) {
          uint16_t eff  = cal[i].mirrored ? mirror4096(raw) : raw;
          uint16_t minr = cal[i].min_raw;
          uint16_t maxr = cal[i].max_raw;

          // span 계산 (0..4095 모듈러)
          uint16_t span = modsub12(maxr, minr);

          // - 캘리 미완(=0)일 때 임시 시드
          if (span == 0) {
            const uint16_t SEED = 800;
            minr = (uint16_t)((eff + 4096u - SEED) & 0x0FFF);
            maxr = (uint16_t)((eff + SEED) & 0x0FFF);
            cal[i].min_raw = minr;
            cal[i].max_raw = maxr;
            cal[i].center  = eff;
            cal[i].ready   = 1;
            saveToEEPROM();
            span = modsub12(maxr, minr);
          }

          // - 항상 짧은 호( < 180° )가 되도록 정규화
          if (span >= 2048) {
            uint16_t tmp = minr; minr = maxr; maxr = tmp;
            span = modsub12(maxr, minr);
          }

          // min 기준 전방거리
          uint16_t dist = modsub12(eff, minr);

          // --- 호 밖이면 끝점으로 클램프 ---
          float t;
          if (dist <= span) {
            t = (float)dist / (float)span;
          } else {
            uint16_t beyond   = (uint16_t)(dist - span);
            uint16_t to_minBW = (uint16_t)(4096 - dist);
            t = (beyond <= to_minBW) ? 1.0f : 0.0f;
          }

          // min→60°, max→0°
          const float OUT_MIN_DEG = 0.0f, OUT_MAX_DEG = 60.0f;
          float deg_target = OUT_MAX_DEG - t * (OUT_MAX_DEG - OUT_MIN_DEG);

          static unsigned long last_ms_local = 0;
          unsigned long now = millis();
          if (now - last_ms_local >= SERVO_PERIOD_MS) {
            last_ms_local = now;
            // (원본 수식 유지)
            deg_target = 2*(deg_target-30.0);
            deg_target = 60.0 - deg_target;
            if (deg_target < OUT_MIN_DEG) deg_target = OUT_MIN_DEG;
            if (deg_target > 50.0f)       deg_target = 50.0f;
            g_servo.write((int)(deg_target));
            sent_deg = deg_target;  // 마지막 보낸 값 기록(출력용)
          }
        }

        // ---- 두 번째 서보 (TCA2, CH7) 제어 ----
        if (cal[i].tca_addr == TCA2_ADDR && ( (i-NUM_CH) < 0 ? kChannels[i] : kChannels2[i-NUM_CH] ) == 7) {
          uint16_t eff  = cal[i].mirrored ? mirror4096(raw) : raw;
          uint16_t minr = cal[i].min_raw;
          uint16_t maxr = cal[i].max_raw;
          uint16_t span = modsub12(maxr, minr);

          if (span == 0) {
            const uint16_t SEED = 800;
            minr = (uint16_t)((eff + 4096u - SEED) & 0x0FFF);
            maxr = (uint16_t)((eff + SEED) & 0x0FFF);
            cal[i].min_raw = minr;
            cal[i].max_raw = maxr;
            cal[i].center  = eff;
            cal[i].ready   = 1;
            saveToEEPROM();
            span = modsub12(maxr, minr);
          }
          
          if (span >= 2048) {
            uint16_t tmp = minr;
            minr = maxr; maxr = tmp;
            span = modsub12(maxr, minr);
          }
          
          uint16_t dist = modsub12(eff, minr);
          float t;
          if (dist <= span) {
            t = (float)dist / (float)span;
          } else {
            uint16_t beyond   = (uint16_t)(dist - span);
            uint16_t to_minBW = (uint16_t)(4096 - dist);
            t = (beyond <= to_minBW) ? 1.0f : 0.0f;
          }
          
          const float OUT_MIN_DEG = 0.0f, OUT_MAX_DEG = 60.0f;
          float deg_target = OUT_MAX_DEG - t * (OUT_MAX_DEG - OUT_MIN_DEG);

          static unsigned long last_ms_local2 = 0;
          unsigned long now2 = millis();
          if (now2 - last_ms_local2 >= SERVO_PERIOD_MS) {
            last_ms_local2 = now2;
            deg_target = 2*(deg_target-30.0);
            deg_target = 60.0 - deg_target;
            if (deg_target < OUT_MIN_DEG) deg_target = OUT_MIN_DEG;
            if (deg_target > 50.0f)       deg_target = 50.0f;
            g_servo2.write((int)(deg_target));
            sent_deg2 = deg_target;
          }
        }

        // ★ 읽기 성공 시에만 last_out 갱신
        last_out[i] = th;
      }
      // 읽기 실패 시: th는 last_out[i] 그대로(이전값 유지)
    }

    append_float(th, 6);                 // 값
    if ((i + 1) % 8 == 0) append_str("\r\n");  // 8개마다 줄바꿈
    else                  append_str(",");     // 그 외엔 쉼표
}

  // 두 번째 서보 각도 추가
  append_float(sent_deg, 2);  // 마지막 각도 소수 2자리 (원래와 동일)
  append_str(",");
  append_float(sent_deg2, 2);
  append_str("\r\n");

  // ===== 한 방에, 안전하게 쓰기(버퍼 부족 시 이번 줄 드롭) =====
  (void)writeLineChunked(line, (size_t)n);

  // 115200 유지 시 버퍼 여유를 위해 10→15ms 권장(수신속도 맞추면 10ms도 OK)
  delay(10);
}
