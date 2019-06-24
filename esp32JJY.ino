/*
   MIT License: https://opensource.org/licenses/MIT
   Copyright (c) 2019 Fumiyuki Shimizu
   Copyright (c) 2019 Abacus Technologies, Inc.

   ESP32 core for Arduino
     https://github.com/espressif/arduino-esp32
     for boards manager: https://dl.espressif.com/dl/package_esp32_index.json
*/
/*
   NTP function is ripped from example sketch
     ESP8266 -> WiFi -> NtpClient,
   which is in the public domain, and modified. thx.
   original header follows at the end of this file.
*/
//-------------------------------------------------------------------------
// If you encountered errors as
//   No such file "limits,"
// please check that the correct esp32 board is selected.
#include <limits>
#include <bitset>
#include <cassert>
#include <cmath>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/ledc.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define STR1(x) #x
#define STR2(x) STR1(x)

template<class T, size_t N>
constexpr size_t size(T (&)[N]) {
  return N;
}

#define GPIO0   0
#define GPIO2   2 // onboard LED
#define GPIO4   4
#define GPIO12  12
#define GPIO13  13
#define GPIO14  14
#define GPIO15  15
#define GPIO25  25
#define GPIO26  26
#define GPIO27  27
#define GPIO32  32
#define GPIO33  33

namespace abct {

// setting this to other than 0 may make your mom beging mad.
// 60: 1minute ahead, 600:10minutes ahead.
//const auto MY_TIME_ADJUST_IN_SECONDS = 10 * 60;
const auto MY_TIME_ADJUST_IN_SECONDS = 0;

const auto WAKEUP_MINUTES = 59; // every xx:59, 1minutes to oclock. -1: no sleep
const auto FREQ1_DURATION_MINUTES_NORMAL =  7;  // duration in 1st freq.(40000U), including NTP syncing.
const auto FREQ2_DURATION_MINUTES_NORMAL =  5;  // duration in 2st freq.(60000U). sleep after that.
const auto FREQ1_DURATION_MINUTES_LONG   = 12;  // duration in 1st freq.(40000U), including NTP syncing.
const auto FREQ2_DURATION_MINUTES_LONG   = 10;  // duration in 2st freq.(60000U). sleep after that.
static_assert(-1 == WAKEUP_MINUTES || (0 <= WAKEUP_MINUTES && WAKEUP_MINUTES <= 59), "specify correct value.");

#define OUTPIN  GPIO26  // jjy. up to your wiring and the board.
#define SWPIN   GPIO25  // normal -> long1 -> long2 mode sw. up to your wiring and the board.
#define LEDPIN  GPIO2   // led. off: normal, blink: long1, on: long2

// east Japan: fukushima 40kHz
// west Japan: kyushu    60kHz
static const decltype(ledc_timer_config_t::freq_hz) _jjy_freqs[] = { 40000U, 60000U, };

#if 0 /* change to 1 and fill your values below. */
static const char *_ssids[] = {
  "esuesuaidhi-_nomap",
  "essahoisa_nomap",
  "aidhi-_nomap",
};
static const char *_psks[] = {
  "purishea-doki-",
  "puripuri",
  "naisho",
};

static const char *_ntp_hosts[] = {
  "ntp",
  "ntp.local",
  "ntp.jst.mfeed.ad.jp",
};
#else
#  include "__abct_config.h"
#endif
static_assert(sizeof(_ssids) == sizeof(_psks), "specify same number of _ssids as _psks.");

const auto TIMER_JJY_CARRIER = LEDC_TIMER_0;
const auto TIMER_BULSE       = LEDC_TIMER_1;

const auto PWM_SPEED_MODE_JJY_CARRIER = LEDC_HIGH_SPEED_MODE;
const auto PWM_CHANNEL_JJY_CARRIER    = LEDC_CHANNEL_0;

const auto PWM_DUTY_RESOLUTION_JJY_CARRIER = LEDC_TIMER_1_BIT;
const auto PWM_DUTY_MAX_JJY_CARRIER        = (1U << PWM_DUTY_RESOLUTION_JJY_CARRIER); /* 0: 0%, .., MAX-1:<100%, MAX:100% */

class freqsStor {
  private:
    size_t cur;

  public:
    freqsStor() {
      cur = 0;
    }

    decltype(*_jjy_freqs)
    get() const {
      return _jjy_freqs[cur];
    }

    void
    setIndex(size_t i) {
      cur = i % size(_jjy_freqs);
    }
};

class ssidsStor {
  private:
    size_t cur;
    size_t start;

  public:
    ssidsStor() {
      start = cur = 0;
    }

    const char *
    ssid() const {
      return _ssids[cur];
    }

    const char *
    psk() const {
      return _psks[cur];
    }

    void
    begin () {
      start = cur;
    }

    boolean
    next() {
      cur = (1 + cur) % size(_ssids);
      return cur != start;
    }
};

class ntpHostsStor {
  private:
    size_t cur;
    size_t start;

  public:
    ntpHostsStor() {
      start = cur = 0;
    }

    const char *
    get() const {
      return _ntp_hosts[cur];
    }

    void
    begin() {
      start = cur;
    }

    boolean
    next() {
      cur = (1 + cur) % size(_ntp_hosts);
      return cur != start;
    }
};

static ssidsStor ssids;
static ntpHostsStor ntpHosts;

class milliTime_t {
  private:
    unsigned long _secs;
    unsigned int _millis;

  public:
    milliTime_t(unsigned long secs, unsigned int millis) {
      set(secs, millis);
    }

    milliTime_t() {
    }

    milliTime_t&
    set(unsigned long secs, unsigned int millis) {
      _secs = secs;
      setMillis(millis);
      return *this;
    }

    milliTime_t&
    setMillis(unsigned int millis) {
      if (1000 <= millis) {
        _secs += millis / 1000;
        _millis = millis % 1000;
      } else {
        _millis = millis;
      }
      return *this;
    }

    milliTime_t&
    addSeconds(int secs) {
      _secs += secs;
      return *this;
    }

    unsigned long
    getSeconds() const {
      return _secs;
    }

    unsigned int
    getMillis() const {
      return _millis;
    }
};

class wifiProcedures {
  private:
    wifiProcedures(const wifiProcedures&);
    wifiProcedures& operator=(const wifiProcedures&);
    ~wifiProcedures() {}

    wifiProcedures() {
      WiFi.persistent(false);
      stop();
    }

  public:
    static wifiProcedures&
    getInstance() {
      static wifiProcedures rc;
      return rc;
    }

    static boolean
    start(boolean rebootOnFailure = false) {
      byte mac[6];
      ssids.begin();
      for (;;) {
        WiFi.macAddress(mac);
        Serial.printf("\nI'm %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        Serial.print("Connecting to ");
        Serial.println(ssids.ssid());
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssids.ssid(), ssids.psk());
        delay(1000);
        for (int j = 0; j < 60; ++j) {
          if (WiFi.status() == WL_CONNECTED) {
            Serial.println();
            Serial.println("Wi-Fi connected");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
            return 1;
          }
          Serial.print(".");
          delay(500);
        }
        Serial.println(" failed.");
        WiFi.disconnect(true);
        delay(500);

        if (!ssids.next()) {
          break;
        }
      }
      Serial.println("giving up.");

      if (rebootOnFailure) {
        // ESP_ERROR_CHECK(nvs_flash_init());
        // ESP_ERROR_CHECK(nvs_flash_erase());
        WiFi.disconnect(true);
        delay(1000);
        WiFi.disconnect(true);
        delay(1000);
        Serial.println("restarting...");
        ESP.restart();
      }
    }

    static void
    stop() {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      delay(100);
    }

    // XXX Y2038
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    static const auto SECS_PER_70YEARS = 2208988800;

    unsigned long
    ntpClient(milliTime_t& currentUTC, boolean rebootOnFailure = false) {
      static const auto NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message
      static byte udpbuf[NTP_PACKET_SIZE];
      memset(udpbuf, 0, sizeof(udpbuf));
      udpbuf[0] = 0b11100011;   // LI, Version, Mode
      udpbuf[1] = 0;     // Stratum, or type of clock
      udpbuf[2] = 6;     // Polling Interval
      udpbuf[3] = 0xEC;  // Peer Clock Precision
      // 8 bytes of zero for Root Delay & Root Dispersion
      udpbuf[12] = 49;
      udpbuf[13] = 0x4E;
      udpbuf[14] = 49;
      udpbuf[15] = 52;

      unsigned long sentMilli;
      ntpHosts.begin();
      for (;;) {
        static WiFiUDP udp;
        unsigned short localPort = random(40000, 64000);
        udp.begin(localPort);

        Serial.print("Sending udp from ");
        Serial.print(WiFi.localIP());
        Serial.printf(":%u to %s:123\n", localPort, ntpHosts.get());
        if (udp.beginPacket(ntpHosts.get(), 123)) {
          udp.write(udpbuf, sizeof(udpbuf));
          udp.endPacket();
          sentMilli = millis();
          delay(100);

          Serial.print("Waiting for response: ");
          int udp_len;
          for (int i = 0; i < 50 && 1 > (udp_len = udp.parsePacket()); ++i) {
            Serial.print(".");
            delay(100);
          }

          if (NTP_PACKET_SIZE <= udp_len) {
            Serial.printf("got %dbytes.\n", udp_len);
            udp.read(udpbuf, sizeof(udpbuf));
            udp.flush();

            //the timestamp starts at byte 40 of the received packet and is four bytes,
            // or two words, long. First, esxtract the two words:
            uint16_t hiWord = word(udpbuf[40], udpbuf[41]);
            uint16_t loWord = word(udpbuf[42], udpbuf[43]);

            // combine the four bytes (two words) into a long integer
            // this is NTP time (seconds since Jan 1 1900):
            //  uint32_t epoch = secsSince1900 - SECS_PER_70YEARS;
            // XXX Y2038
            // XXX Y2036
            uint32_t utc = (((uint32_t) hiWord << 16) | loWord) - SECS_PER_70YEARS;
            Serial.printf("NTP time in unix time = %lu.", utc);

            hiWord = word(udpbuf[44], udpbuf[45]);
            loWord = word(udpbuf[46], udpbuf[47]);
            uint32_t f = ((uint32_t) hiWord << 16) | loWord;
            unsigned long utcf = (unsigned long)((1000ULL * f) >> 32);
            Serial.printf("%03lu, millis() = %lu\n", utcf, sentMilli);

            currentUTC.set(utc, utcf);
            return sentMilli;
          }

          if (1 > udp_len) {
            Serial.println(" no response.");
          } else {
            Serial.println(" truncated data.");
          }
        }

        Serial.println("failed.");
        if (!ntpHosts.next()) {
          break;
        }
      }
      Serial.println("giving up.");
      if (rebootOnFailure) {
        Serial.println("restarting...");
        ESP.restart();
      }
      return 0;
    }
};

static wifiProcedures& wifi = wifiProcedures::getInstance();

class carrier {
  private:
    ledc_timer_config_t ledc_timer;
    ledc_channel_config_t ledc_channel;
    decltype(ledc_channel.duty) duty;
    freqsStor freqs;

    void
    calcDuty(double duty_percent) {
      static const auto eps = 1e-5;
      duty = PWM_DUTY_MAX_JJY_CARRIER * duty_percent / 100.0;
      assert(0 <= duty);
      assert(PWM_DUTY_MAX_JJY_CARRIER >= duty);
      double diff = abs(PWM_DUTY_MAX_JJY_CARRIER * duty_percent - duty * 100.0);
      //assert(diff < eps);
      if (diff >= eps) {
        if (duty < PWM_DUTY_MAX_JJY_CARRIER && diff > abs(PWM_DUTY_MAX_JJY_CARRIER * duty_percent - (1 + duty) * 100.0)) {
          ++duty;
        }
        Serial.printf("cannot set duty to %lf%%. set to the nearest: %lf%%(%u/%u)\n",
                      duty_percent, 100.0 * duty / PWM_DUTY_MAX_JJY_CARRIER, duty, PWM_DUTY_MAX_JJY_CARRIER);
      }
    }

    void
    initPWM() {
      // https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/ledc.html

      memset(&ledc_timer, 0, sizeof(ledc_timer));
      ledc_timer.duty_resolution = PWM_DUTY_RESOLUTION_JJY_CARRIER; // resolution of PWM duty
      ledc_timer.freq_hz         = freqs.get();                     // frequency of PWM signal
      ledc_timer.speed_mode      = PWM_SPEED_MODE_JJY_CARRIER;      // timer mode
      ledc_timer.timer_num       = TIMER_JJY_CARRIER;               // timer index
      ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

      memset(&ledc_channel, 0, sizeof(ledc_channel));
      ledc_channel.channel    = PWM_CHANNEL_JJY_CARRIER;
      ledc_channel.duty       = 0;
      ledc_channel.gpio_num   = OUTPIN;
      ledc_channel.intr_type  = LEDC_INTR_DISABLE;
      ledc_channel.hpoint     = 0;
      ledc_channel.speed_mode = ledc_timer.speed_mode;
      ledc_channel.timer_sel  = ledc_timer.timer_num;
      ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    carrier(const carrier&);
    carrier& operator=(const carrier&);
    ~carrier() {}

    carrier() {
      calcDuty(50);
      reset();
    }

  public:
    static carrier&
    getInstance() {
      static carrier rc;
      return rc;
    }

    void
    reset() {
      Serial.printf("\nfreq.: %uHz\n", (unsigned int)freqs.get());
      Serial.printf("output pin: " STR2(OUTPIN) "(%d)\n", OUTPIN);
      initPWM();
      setLevelLow();
    }

    void
    setJJYFreq(int i) {
      freqs.setIndex(i);
      if (freqs.get() != currentFreq()) {
        reset();
      }
    }

    decltype(ledc_timer.freq_hz)
    currentFreq() const {
      return ledc_timer.freq_hz;
    }

    void
    setLevelHigh() {
      //ESP_ERROR_CHECK(ledc_set_duty_and_update(ledc_channel.speed_mode, ledc_channel.channel, duty, 0));
      // separated, but set_duty() actually sets duty. so, do not insert lines below.
      ESP_ERROR_CHECK(ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty));
      ESP_ERROR_CHECK(ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel));
    }

    void
    setLevelLow() {
      //ESP_ERROR_CHECK(ledc_set_duty_and_update(ledc_channel.speed_mode, ledc_channel.channel, 0, 0));
      // separated, but set_duty() actually sets duty. so, do not insert lines below.
      ESP_ERROR_CHECK(ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0));
      ESP_ERROR_CHECK(ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel));
    }
};

static carrier& pwm = carrier::getInstance();

// XXX Y2038
// 1970-01-01 thu.  0:sun, 1:mon, 2:tue, 3:wed, 4:thu
#define DOW_EPOCH 4
// XXX DST
// w/o TZDB...
#define SECS_TZ_OFFSET ((MY_TIME_ADJUST_IN_SECONDS) + 9*60*60L)

class japanTime {
  private:
    unsigned int _doy;
    unsigned int _dow;
    unsigned int _hours;
    unsigned int _minutes;

    unsigned int _year;
    unsigned int _doe;          // hold current utc / 86400 to skip calc. on update()ing.
    unsigned int _doeStart;     // utc / 86400 of 1st Jan of this year.
    unsigned int _doeNextStart; // utc / 86400 of 1st Jan of the next year.

  public:
    decltype(_year)
    year() const {
      return _year;
    }

    decltype(_doy)
    doy() const {
      return _doy;
    }

    decltype(_dow)
    dow() const {
      return _dow;
    }

    decltype(_hours)
    hours() const {
      return _hours;
    }

    decltype(_minutes)
    minutes() const {
      return _minutes;
    }

    japanTime() {
      _doeStart = _doeNextStart = 0;
      _doe = (unsigned int) - 1;
      _year = 1970;
    }

    void
    update(unsigned long utc) {
      // XXX DST
      // XXX Y2038
      unsigned long jst = utc + SECS_TZ_OFFSET;
      // XXX Y2038
      decltype(_doe) doe = jst / (24 * 60 * 60UL);
      assert(doe + 366 > doe);

      unsigned int today = jst % (24 * 60 * 60UL);
      _hours = today / (60 * 60);
      _minutes = (today % (60 * 60)) / 60;

      if (_doe == doe) {
        return;
      }
      _doe = doe;

      // XXX Y2038
      _dow = (doe + DOW_EPOCH) % 7;

      decltype(_year) y;
      if (doe < _doeNextStart) {
        if (_doeStart <= doe) {
          _doy = doe - _doeStart;
          return;
        }
        _doeStart = 0;
        // XXX Y2038
        y = 1970;
      } else {
        y = _year;
      }
      for (;; ++y) {
        if (0 == y % 4 && (0 != y % 100 || 0 == y % 400)) {
          _doeNextStart = _doeStart + 366;
        } else {
          _doeNextStart = _doeStart + 365;
        }
        if (doe < _doeNextStart) {
          _year = y;
          _doy = doe - _doeStart;
          return;
        }
        _doeStart = _doeNextStart;
      }
    }
};

class currentUnixMilliTime {
  private:
    // m = adruino millis()
    // unixTime = offsetSecs + (m + offsetMillis) / 1000U  '.'  (m + offsetMillis) % 1000U
    unsigned long offsetSecs;
    unsigned int  offsetMillis;
    unsigned long adjustedUTC;

  public:
    void
    update(const milliTime_t &utc, unsigned long arduinoClock) {
      assert(1000 > utc.getMillis());
      adjustedUTC = utc.getSeconds();
      unsigned long arduinoSecs = arduinoClock / 1000U;
      unsigned int arduinoMillis = arduinoClock % 1000U;
      if (utc.getMillis() >= arduinoMillis) {
        offsetMillis = utc.getMillis() - arduinoMillis;
        offsetSecs = utc.getSeconds() - arduinoSecs;
      } else {
        offsetMillis = 1000U + utc.getMillis() - arduinoMillis;
        offsetSecs = utc.getSeconds() - arduinoSecs - 1U;
      }
    }

    currentUnixMilliTime() {
      offsetSecs = 0;
      offsetMillis = 0;
      adjustedUTC = 0;
    }

    void
    timex(milliTime_t &milliTime) {
      unsigned long amil = millis();
      unsigned long mil = offsetMillis + amil;
      unsigned long utc = offsetSecs + mil / 1000U;
      if (utc < adjustedUTC) {
        // wraparound
        do {
          milliTime.set(
            (unsigned long)((1 + (unsigned long long)std::numeric_limits<unsigned long>::max()) / 1000U) + utc,
            (unsigned long)((1 + (unsigned long long)std::numeric_limits<unsigned long>::max()) % 1000U) + (mil % 1000U));
        } while (milliTime.getSeconds() < adjustedUTC);
        update(milliTime, amil);

        // we did too much stuffs. try again.
        mil = offsetMillis + millis();
        utc = offsetSecs + mil / 1000U;
      }
      milliTime.set(utc, mil % 1000U);
    }

    unsigned long
    time() {
      unsigned long utc = offsetSecs + (offsetMillis + millis()) / 1000U;
      if (utc >= adjustedUTC) {
        return utc;
      }
      // wraparound
      milliTime_t m;
      timex(m);
      return m.getSeconds();
    }

    unsigned long
    toMillis(const milliTime_t utc, unsigned int additionalMillis = 0) const {
      unsigned long until = (utc.getSeconds() - offsetSecs) * 1000U + utc.getMillis() - offsetMillis + additionalMillis;
      unsigned long w;
      if ((w = until - millis()) > 2000 || w < 2) {
        Serial.printf("\npossible overload. invalid wait: %lums, changed to: 1s.\n", w);
        return 1000 + millis();
      }
      return until;
    }

    void
    bulse(milliTime_t startUTC, unsigned int bulseMilli, int ledPin = -1) {
      unsigned long s = toMillis(startUTC);
      unsigned long w = s - millis();
      if (w <= 1000) {
        delay(w);
        pwm.setLevelHigh();
        if (0 <= ledPin) {
          digitalWrite(ledPin, HIGH);
        }
        w = s + bulseMilli - millis();
        if (w <= 1000) {
          delay(w);
        }
      }
      pwm.setLevelLow();
      if (0 <= ledPin) {
        digitalWrite(ledPin, LOW);
      }
    }
};

//=========================== JJY ===========================
// http://jjy.nict.go.jp/jjy/trans/index.html
//   http://jjy.nict.go.jp/jjy/trans/timecode1.html
class jjy_timecode {
  private:
    std::bitset<10> _jjy_timecode_table[6];
    unsigned int _seconds;
    unsigned long utcApplicableMin;
    unsigned long utcApplicableMax;
    japanTime *jst;

  public:
    unsigned int
    get() {
      static_assert(0 == (unsigned int) false, "fix me");
      static_assert(1 == (unsigned int) true, "fix me");
      auto sec10 = _seconds % 10;
      return 0 == _seconds || 9 == sec10 ? 2 : static_cast<unsigned int>(_jjy_timecode_table[_seconds / 10][sec10]);
    }

    inline void
    setBit(unsigned int seconds60, boolean val) {
      //      assert(0 < seconds60);
      //      assert(seconds60 < 59);
      auto sel = seconds60 / 10;
      auto sec10 = seconds60 % 10;
      //      assert(9 > sec10);
      _jjy_timecode_table[sel][sec10] = val;
    }

    inline void
    clearParity(unsigned int seconds60) {
      auto sel = seconds60 / 10;
      _jjy_timecode_table[sel][9] = false;
    }

    inline boolean
    getParity(unsigned int seconds60) {
      auto sel = seconds60 / 10;
      return _jjy_timecode_table[sel][9];
    }

    inline void
    setDigit(unsigned int seconds60, size_t bits, unsigned int digit, boolean calcParity = false) {
      //      assert(0 < seconds60);
      //      assert(seconds60 < 59);
      auto sel = seconds60 / 10;
      auto sec10 = seconds60 % 10;
      //      assert(digit <= 9);
      //      assert(1 <= bits);
      //      assert(4 >= bits);
      //      assert(sec10 + bits <= 9);
      auto b = sec10 + bits - 1;
      auto d = digit;
      for (size_t c = 0; c < bits; ++c, --b, d >>= 1 ) {
        static_assert((boolean) 0 == false, "fix me");
        static_assert((boolean) 1 == true, "fix me");
        boolean x = static_cast<boolean>(d & 1);
        _jjy_timecode_table[sel][b] = x;
        if (calcParity && x) {
          _jjy_timecode_table[sel][9] = _jjy_timecode_table[sel][9] ^ x;
        }
      }
    }

    char
    timecodeDisp() {
      switch (get()) {
        case 0: return '0';
        case 1: return '1';
        case 2: return '_';
      }
    }

    unsigned int
    bulseMilli() {
      switch (get()) {
        case 0: return 800;
        case 1: return 500;
        case 2: return 200;
      }
    }

    boolean
    isApplicable(unsigned long utc) {
      static_assert((unsigned long)(ULONG_MAX + 10) - (unsigned long)ULONG_MAX == 10, "cannot handle wraparound(positive)");
      static_assert((unsigned long)ULONG_MAX - (unsigned long)(ULONG_MAX + 10) == -10, "cannot handle wraparound(negative)");
      if (utcApplicableMax < utcApplicableMin) {
        // wraparound
        return utc <= utcApplicableMax || utcApplicableMin <= utc;
      }
      return utcApplicableMin <= utc && utc <= utcApplicableMax;
    }

    void
    setSeconds(unsigned long utc) {
      if (!isApplicable(utc)) {
        update(utc);
      } else {
        _seconds = utc % 60;
      }
    }

    decltype(_seconds)
    getSeconds() {
      return _seconds;
    }

    jjy_timecode() {
      // XXX Y2038
      utcApplicableMin = 1561091580;
      utcApplicableMax = 1561091580;
      jst = new japanTime();
    }

    void
    update(unsigned long utc, boolean printValue = false) {
      _seconds = utc % 60;
      utcApplicableMin = utc - _seconds;
      utcApplicableMax = utcApplicableMin + 59;

      jst->update(utc);

      auto minutes = jst->minutes();
      if (printValue) {
        Serial.printf("min: %u", minutes);
      }
      clearParity(0);
      // 0: M
      setDigit(1, 3, minutes / 10, true);
      // 4: 0
      setDigit(5, 4, minutes % 10, true);
      // 9: P1

      auto hours = jst->hours();
      if (printValue) {
        Serial.printf(", hours: %u", hours);
      }
      clearParity(10);
      // 10: 0
      // 11: 0
      setDigit(12, 2, hours / 10, true);
      // 14: 0
      setDigit(15, 4, hours % 10, true);
      // 19: P2

      auto doy_1 = 1 + jst->doy();
      if (printValue) {
        Serial.printf(", doy_1: %u", doy_1);
      }
      // 20: 0
      // 21: 0
      setDigit(22, 2, doy_1 / 100);
      // 24: 0
      setDigit(25, 4, (doy_1 % 100) / 10);
      // 29: P3

      setDigit(30, 4, doy_1 % 10);
      // 34: 0
      // 35: 0
      setBit(36, getParity(10)); // PA1: hours
      setBit(37, getParity(0));  // PA2: minutes
      // 38: 0: SU1
      // 39: P4

      // 40: 0: SU2
      if (15 == (minutes % 30)) {
        setDigit(41, 8, 0);
        // 49: P5

        //setDigit(50, 3, 0); // ST1, ST2, ST3
        setDigit(50, 3, 6); // ST1, ST2, ST3

        setBit(53, 0); // ST4

        //setDigit(54, 2, 0); // ST5, ST6
        setDigit(54, 2, 1); // ST5, ST6
      } else {
        auto year = jst->year();
        if (printValue) {
          Serial.printf(", year: %u", year);
        }
        setDigit(41, 4, (year % 100) / 10);
        setDigit(45, 4, year % 10);
        // 49: P5

        auto dow = jst->dow();
        if (printValue) {
          Serial.printf(", dow: %u\n", dow);
        }
        setDigit(50, 3, dow);
        setDigit(53, 2, 0); // LS1, LS2
        setBit(55, false);
      }

      // 56-58: 0
      // 59: P0
    }
};
//=========================== JJY ===========================

class mymode {
  public:
    enum modeval {
      normal = 0, // every oclock: 5min 1st jjy freq. -> 5min 2nd jjy freq.
      long_1 = 1, // now: 10min 1st jjy freq. -> 10min 2nd jjy freq.
      long_2 = 2, // now: 10min 2nd jjy freq. -> 10min 1st jjy freq.
    };

  private:
    modeval mode;
    unsigned long cycleStart;

  public:
    mymode() {
      mode = normal;
      cycleStart = millis();
    }

    inline void
    next() {
      switch (mode) {
        case normal:
          mode = long_1;
          break;
        case long_1:
          mode = long_2;
          break;
        case long_2:
          mode = normal;
          break;
        default:
          assert(false);
      }
      cycleStart = millis();
      Serial.printf("\ncurrent mode: %d\n", mode);
    }

    inline void
    setJJYFreq() const { // and set LED
      long elap = millis() - cycleStart;
      switch (mode) {
        case normal:
          pwm.setJJYFreq(elap < FREQ1_DURATION_MINUTES_NORMAL * 60UL * 1000 ? 0 : 1);
          break;
        case long_1:
          pwm.setJJYFreq(elap < FREQ1_DURATION_MINUTES_LONG * 60UL * 1000 ? 0 : 1);
          break;
        case long_2:
          pwm.setJJYFreq(elap < FREQ1_DURATION_MINUTES_LONG * 60UL * 1000 ? 1 : 0);
          break;
      }
      switch (mode) {
        case normal:
        case long_1:
          digitalWrite(LEDPIN, LOW);
          break;
        case long_2:
          digitalWrite(LEDPIN, HIGH);
          break;
      }
    }

    inline void
    sleepy(unsigned long utc) const {
      if (0 > (WAKEUP_MINUTES)) {
        return;
      }
      long elap = millis() - cycleStart;
      boolean sleep;
      switch (mode) {
        case normal:
          sleep = (elap >= ((FREQ1_DURATION_MINUTES_NORMAL) + (FREQ2_DURATION_MINUTES_NORMAL)) * 60UL * 1000);
          break;
        case long_1:
        case long_2:
          sleep = (elap >= ((FREQ1_DURATION_MINUTES_LONG) + (FREQ2_DURATION_MINUTES_LONG)) * 60UL * 1000);
          break;
      }
      if (!sleep) {
        return;
      }

      unsigned int minutes = ((utc + SECS_TZ_OFFSET) % (60 * 60)) / 60;
      unsigned int awakeMinutes = (60 + (WAKEUP_MINUTES) - minutes) % 60;
      Serial.printf("\nc u in %uminutes...", awakeMinutes);
      static_assert(std::numeric_limits<uint64_t>::max() >= 120 * (60ULL * 1000 * 1000), "need larger type.");
      // https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/system/sleep_modes.html#_CPPv414esp_deep_sleep8uint64_t
      esp_deep_sleep(awakeMinutes * (60ULL * 1000 * 1000));
    }

    inline int
    ledPin() const {
      switch (mode) {
        case long_1:
          return LEDPIN;
      }
      return -1;
    }
};



static currentUnixMilliTime currentTime;
static jjy_timecode currentTimecode;

volatile int swPushed = 0;
static unsigned long lastPushed;
static mymode currentMode;
// http://esp32.info/docs/esp_idf/html/dc/d35/portmacro_8h.html
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
IRAM_ATTR void
swHandler() {
  portENTER_CRITICAL_ISR(&mux);
  ++swPushed;
  portEXIT_CRITICAL_ISR(&mux);
}

void
init_pins() {
  // say hello.
  Serial.println("hi.");
  Serial.printf("output pin: " STR2(OUTPIN) "(%d)\n", OUTPIN);
  Serial.printf("led pin: " STR2(LEDPIN) "(%d)\n", LEDPIN);
  pinMode(OUTPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  delay(100);
  for (int i = 0; i < 5; ++i) {
    digitalWrite(OUTPIN, HIGH);
    digitalWrite(LEDPIN, HIGH);
    delay(100);
    digitalWrite(OUTPIN, LOW);
    digitalWrite(LEDPIN, LOW);
    delay(200);
  }

  Serial.printf("sw pin: " STR2(SWPIN) "(%d)\n", SWPIN);
  digitalWrite(SWPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWPIN), swHandler, FALLING);
  lastPushed = millis();
}

void
init_time() {
  wifi.start(true);
  milliTime_t utc;
  unsigned long mil = wifi.ntpClient(utc, true);
  wifi.stop();
  assert(0 != mil);

  currentTime.update(utc, mil);

  currentTime.timex(utc);
  Serial.printf("    current unix time = %lu.%03lu\n", utc.getSeconds(), utc.getMillis());

  currentTimecode.update(2 + utc.getSeconds(), true);
  unsigned long w = currentTime.toMillis(utc.addSeconds(1).setMillis(0)) - millis();
  if (w <= 1000) {
    delay(w);
  }
}

} // namespace

void
setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  abct::init_pins();
  abct::pwm.reset();
  abct::init_time();
}

void
loop() {
  while (abct::swPushed > 0) {
    portENTER_CRITICAL(&abct::mux);
    --abct::swPushed;
    portEXIT_CRITICAL(&abct::mux);
    unsigned long mil = millis();
    if ((int) (mil - abct::lastPushed) < 200) {
      continue;
    }
    abct::lastPushed = mil;
    abct::currentMode.next();
  }
  abct::currentMode.setJJYFreq();

  abct::milliTime_t nx;
  abct::currentTime.timex(nx);
  unsigned long utc = nx.addSeconds(1).getSeconds();
  abct::currentMode.sleepy(utc);

  abct::currentTimecode.setSeconds(utc);
  abct::currentTime.bulse(nx.setMillis(0), abct::currentTimecode.bulseMilli(), abct::currentMode.ledPin());

  //Serial.printf("%c(%d)", abct::currentTimecode.timecodeDisp(), abct::currentTimecode.bulseMilli());
  Serial.print(abct::currentTimecode.timecodeDisp());
  if (59 == abct::currentTimecode.getSeconds()) {
    //abct::currentTimecode.update(1 + nx.getSeconds());
    Serial.printf(" : %lu %luHz\n", nx.getSeconds(), abct::pwm.currentFreq());
  } else if (9 == abct::currentTimecode.getSeconds() % 10) {
    Serial.print(' ');
  }
}

// end of file
/*
  Udp NTP Client
  set
  Get the time from a Network Time Protocol (NTP) time server
  Demonstrates use of UDP sendPacket and ReceivePacket
  For more on NTP time servers and the messages needed to communicate with them,
  see http://en.wikipedia.org/wiki/Network_Time_Protocol

  created 4 Sep 2010
  by Michael Margolis
  modified 9 Apr 2012
  by Tom Igoe
  updated for the ESP8266 12 Apr 2015
  by Ivan Grokhotkov
*/
