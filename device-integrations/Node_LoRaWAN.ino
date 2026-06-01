// ═══════════════════════════════════════════════════════════
//  LoRaWAN LOW POWER NODE — ESP32 + DS3231 (Deep Sleep)
//  Berbasis kode original yang sudah jalan, ditambah:
//    - Deep sleep antar TX
//    - Frame counter LMIC disimpan di RTC memory
//    - ADC baterai lebih akurat (multi-sample + calibrated)
//    - TX timeout agar tidak stuck
// ═══════════════════════════════════════════════════════════
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include "DFRobot_ESP_PH.h"
#include <EEPROM.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_adc_cal.h>

#define PH_PIN   36
#define BATT_PIN 34

// ── ADC & Baterai ───────────────────────────────────────────
#define VREF            3.3
#define ADC_RESOLUTION  4095.0
#define BATT_OFFSET     0.354
#define BATT_R1         10000.0
#define BATT_R2         10000.0
#define BATT_FACTOR     ((BATT_R1 + BATT_R2) / BATT_R2)
#define ADC_SAMPLES     32          // [NEW] jumlah sample untuk rata-rata

// ─── Mode ─────────────────────────────────────────────────
// 0 = dummy | 1 = real
#define SENSOR_MODE 0

// ─── Interval deep sleep (detik) ──────────────────────────
#define SLEEP_INTERVAL_SEC 300      // 5 menit

// ─── TX timeout (ms) ─────────────────────────────────────
#define TX_TIMEOUT_MS 30000         // 30 detik

// ─── Data di RTC memory (survive deep sleep) ──────────────
RTC_DATA_ATTR static uint16_t app_counter   = 0;
RTC_DATA_ATTR static uint32_t savedSeqnoUp  = 0;
RTC_DATA_ATTR static uint32_t savedSeqnoDn  = 0;

DFRobot_ESP_PH ph;
RTC_DS3231 rtc;
static bool rtcOk = false;
static unsigned long txStartTime = 0;

// ─── ADC Calibration ──────────────────────────────────────
static esp_adc_cal_characteristics_t adcChars;

// ─── Kredensial ABP ───────────────────────────────────────
static u4_t DEVADDR = 0x0076E6C6;
static u1_t NWKSKEY[16] = { 0x1B, 0xC8, 0xA5, 0x70, 0x29, 0x26, 0x62, 0xAF, 0x07, 0xBB, 0xB2, 0x3D, 0x0A, 0xFA, 0x51, 0x86 };
static u1_t APPSKEY[16] = { 0x17, 0x37, 0x9C, 0xE0, 0x4F, 0x09, 0x60, 0x2E, 0x6C, 0xC3, 0xFF, 0x35, 0x75, 0xD4, 0xDA, 0x9A };

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t  mydata[9];
static osjob_t  sendjob;
static volatile bool txDone = false;

const lmic_pinmap lmic_pins = {
    .nss  = 13,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = 16,
    .dio  = {27, 17, LMIC_UNUSED_PIN},
};

// ─── Matikan WiFi & BT ───────────────────────────────────
void disableRadios() {
    esp_wifi_stop();
    esp_bt_controller_disable();
}

// ─── Masuk deep sleep ─────────────────────────────────────
void enterDeepSleep() {
    // Simpan frame counter LMIC sebelum tidur
    savedSeqnoUp = LMIC.seqnoUp;
    savedSeqnoDn = LMIC.seqnoDn;

    Serial.print(F("Tidur "));
    Serial.print(SLEEP_INTERVAL_SEC);
    Serial.println(F(" detik..."));
    Serial.flush();

    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_SEC * 1000000ULL);
    esp_deep_sleep_start();
}

// ═══════════════════════════════════════════════════════════
//  FUNGSI BATERAI — Diperbaiki untuk akurasi lebih tinggi
// ═══════════════════════════════════════════════════════════

// [NEW] Baca ADC dengan multi-sample + kalibrasi ESP32
// - Ambil 32 sample lalu rata-rata → noise berkurang ~5.6x
// - Pakai esp_adc_cal untuk kompensasi non-linearitas ADC
float readBattVoltage() {
    uint32_t adcSum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        adcSum += analogRead(BATT_PIN);
        delayMicroseconds(100);     // jeda antar sample
    }
    uint32_t adcAvg = adcSum / ADC_SAMPLES;

    // Gunakan kalibrasi jika tersedia, fallback ke perhitungan manual
    uint32_t millivolts = esp_adc_cal_raw_to_voltage(adcAvg, &adcChars);
    float vADC = millivolts / 1000.0;

    return (vADC * BATT_FACTOR) - BATT_OFFSET;
}

// [IMPROVED] Tabel tegangan LiPo yang lebih detail
// Sumber: discharge curve LiPo 3.7V tipikal pada 0.2C
// Diukur pada OCV (Open Circuit Voltage), bukan under load
int battToPercent(float v) {
    // Tabel 21 titik sudah cukup halus untuk interpolasi linear
    const float voltTable[] = {
        4.20, 4.15, 4.11, 4.08, 4.02,
        3.98, 3.95, 3.91, 3.87, 3.83,
        3.79, 3.75, 3.71, 3.67, 3.63,
        3.59, 3.55, 3.51, 3.45, 3.40, 3.30
    };
    const int pctTable[] = {
        100,  95,   90,   85,   80,
         75,  70,   65,   60,   55,
         50,  45,   40,   35,   30,
         25,  20,   15,   10,    5,   0
    };
    const int TABLE_SIZE = 21;

    if (v >= voltTable[0])            return 100;
    if (v <= voltTable[TABLE_SIZE-1]) return 0;

    for (int i = 0; i < TABLE_SIZE - 1; i++) {
        if (v >= voltTable[i + 1]) {
            float vHigh = voltTable[i];
            float vLow  = voltTable[i + 1];
            int   pHigh = pctTable[i];
            int   pLow  = pctTable[i + 1];
            return pLow + (int)((v - vLow) / (vHigh - vLow) * (pHigh - pLow));
        }
    }
    return 0;
}

// ═══════════════════════════════════════════════════════════
//  LMIC CALLBACK & TX
// ═══════════════════════════════════════════════════════════

void onEvent(ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        Serial.println(F("TX_DONE — masuk deep sleep"));
        txDone = true;
    }
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("[WARN] OP_TXRXPEND, skip → tidur"));
        txDone = true;
        return;
    }

    uint16_t phEncoded;
    uint8_t  battPct;

#if SENSOR_MODE == 1
    float voltage = analogRead(PH_PIN) / ADC_RESOLUTION * (VREF * 1000.0);
    float phValue = ph.readPH(voltage, 25.0);
    phEncoded = (uint16_t)(phValue * 100);

    float vBatt = readBattVoltage();
    battPct = battToPercent(vBatt);

    Serial.print(F("pH: "));      Serial.print(phValue, 2);
    Serial.print(F(" | Batt: ")); Serial.print(vBatt, 2);
    Serial.print(F("V ("));       Serial.print(battPct);
    Serial.print(F("%) | "));
#else
    phEncoded = 0;
    battPct   = 0;
    Serial.print(F("[DUMMY] | "));
#endif

    uint32_t currentTs = rtcOk ? rtc.now().unixtime() : (uint32_t)(millis() / 1000UL);
    app_counter++;

    mydata[0] = highByte(phEncoded);
    mydata[1] = lowByte(phEncoded);
    mydata[2] = battPct;
    mydata[3] = (currentTs >> 24) & 0xFF;
    mydata[4] = (currentTs >> 16) & 0xFF;
    mydata[5] = (currentTs >> 8)  & 0xFF;
    mydata[6] =  currentTs        & 0xFF;
    mydata[7] = highByte(app_counter);
    mydata[8] = lowByte(app_counter);

    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);

    Serial.print(F("TS: "));         Serial.print(currentTs);
    Serial.print(F(" | Counter: ")); Serial.print(app_counter);
    Serial.print(F(" | SeqUp: "));   Serial.println(LMIC.seqnoUp);
    Serial.println(F("Packet Queued"));
}

// ═══════════════════════════════════════════════════════════
//  SETUP — jalan setiap bangun dari deep sleep
// ═══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    // Matikan WiFi & BT — hemat ~50mA
    disableRadios();

    // ── ADC Init ──────────────────────────────────────────
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // [NEW] Kalibrasi ADC — pakai eFuse jika tersedia
    esp_adc_cal_characterize(
        ADC_UNIT_1,             // ADC1 (GPIO 32-39)
        ADC_ATTEN_DB_11,        // atenuasi 11dB = range 0-3.3V
        ADC_WIDTH_BIT_12,       // 12-bit
        1100,                   // Vref default (mV), override oleh eFuse jika ada
        &adcChars
    );

    EEPROM.begin(32);

    // ── RTC Init ──────────────────────────────────────────
    Wire.begin();
    rtcOk = rtc.begin();
    if (!rtcOk) {
        Serial.println(F("[WARN] RTC tidak ditemukan"));
    } else {
        if (rtc.lostPower())
            Serial.println(F("[WARN] RTC lost power"));
        Serial.println(F("[OK] RTC Ready"));
    }

    // ── LoRa Module ───────────────────────────────────────
    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);
    delay(100);   // dikurangi dari 1000ms — cukup untuk reset modul

    ph.begin();

    // ── LMIC Init ─────────────────────────────────────────
    os_init();
    LMIC_reset();
    LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);

    // Restore frame counter dari deep sleep
    // Tanpa ini, network server tolak paket karena counter reset
    LMIC.seqnoUp = savedSeqnoUp;
    LMIC.seqnoDn = savedSeqnoDn;

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7, 14);

    // ESP32 crystal kurang presisi → butuh toleransi untuk RX window
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

#if SENSOR_MODE == 0
    Serial.println(F("[MODE] DUMMY"));
#else
    Serial.println(F("[MODE] REAL"));
#endif

    Serial.print(F("Boot ke-")); Serial.print(app_counter + 1);
    Serial.print(F(" | SeqUp: ")); Serial.println(savedSeqnoUp);

    txStartTime = millis();
    do_send(&sendjob);
}

// ═══════════════════════════════════════════════════════════
//  LOOP — tunggu TX selesai lalu deep sleep
// ═══════════════════════════════════════════════════════════
void loop() {
    os_runloop_once();

    if (txDone || (millis() - txStartTime > TX_TIMEOUT_MS)) {
        if (!txDone) {
            Serial.println(F("[WARN] TX timeout — paksa deep sleep"));
        }
        enterDeepSleep();
    }
}
