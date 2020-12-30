//  Copyright 2020 Albert Frisch
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see http://www.gnu.org/licenses/

#include <WiFi.h>
#include "time.h"

#define MAX_SRV_CLIENTS  2

const char* CMD_HIGH        =           "on";
const char* CMD_LOW         =          "off";
const char* CMD_SEP         =            " ";
const char* CMD_START       =            "#";
const char* CMD_GETSTATUS   =       "status";
const char* CMD_RESET       =        "reset";
const char* CMD_RESETWIFI   =    "resetwifi";
const char* CMD_SYNCTIME    =         "sync";

const char* WIFI_SSID       =    "YOUR_SSID";
const char* WIFI_PASSWORD   =    "YOUR_PASS";

const int   DCF_PIN         =              2; // LED0 and open-collector output is on GPIO2
const int   PERIOD_DCF_MS   =            100; // DCF period in milliseconds
const int   PERIOD_SYNCED_S =              2; // recheck if synced period
const int   PERIOD_NTP_S    =          10*60; // NTP synchronization period in seconds
const char* NTP_SERVER      = "pool.ntp.org"; // NTP server
const long  NTP_GMT_S       =          60*60; // GMT offset in seconds
const int   NTP_DAYLIGHT_S  =          60*60; // daylight offset in seconds

struct {
  boolean stringComplete = false;
  boolean commandAck     = false;
  boolean synced         = false;
  boolean setnow         = false;
} flags;
long olddcftime, oldsyncedtime, oldntptime;
int oldsec;
String inputString       =    "";
String commandString     =    "";
String valueString       =    "";
char formatString[32];

WiFiServer server(23);   // telnet server
WiFiClient serverClients[MAX_SRV_CLIENTS];

template <class T> void consolePrint(T output);
template <class T> void consolePrint(T output, int type);
template <class T> void consolePrintLn(T output);
template <class T> void consolePrintLn(T output, int type);

//--------------------
// START of DCF77generator
// based on https://blog.blinkenlight.net/experiments/dcf77/dcf77-generator/
//
//  www.blinkenlight.net
//
//  Copyright 2014 Udo Klein
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see http://www.gnu.org/licenses/

namespace BCD {
  typedef union {
    struct {
      uint8_t lo: 4;
      uint8_t hi: 4;
    } digit;
  
    struct {
      uint8_t b0: 1;
      uint8_t b1: 1;
      uint8_t b2: 1;
      uint8_t b3: 1;
      uint8_t b4: 1;
      uint8_t b5: 1;
      uint8_t b6: 1;
      uint8_t b7: 1;
    } bit;
  
    uint8_t val;
  } bcd_t;
  
  void increment(bcd_t &value);
  
  bcd_t int_to_bcd(const uint8_t value);
  uint8_t bcd_to_int(const bcd_t value);
}

namespace Debug {
  void bcddigit(uint8_t data);
  void bcddigits(uint8_t data);
}

typedef union {
  struct {
    uint8_t b0: 2;
    uint8_t b1: 2;
    uint8_t b2: 2;
    uint8_t b3: 2;
  } signal;
  uint8_t byte;
} tData;

namespace DCF77 {
  typedef enum {
    long_tick  = 3,
    short_tick = 2,
    undefined  = 1,
    sync_mark  = 0
  } tick_t;
  
  typedef struct {
    BCD::bcd_t year;     // 0..99
    BCD::bcd_t month;    // 1..12
    BCD::bcd_t day;      // 1..31
    BCD::bcd_t weekday;  // Mo = 1, So = 7
    BCD::bcd_t hour;     // 0..23
    BCD::bcd_t minute;   // 0..59
    uint8_t second;      // 0..60
    bool uses_summertime           : 1;  // false -> wintertime, true, summertime
    bool uses_backup_antenna       : 1;  // typically false
    bool timezone_change_scheduled : 1;
    bool leap_second_scheduled     : 1;
  
    bool undefined_minute_output                    : 1;
    bool undefined_uses_summertime_output           : 1;
    bool undefined_uses_backup_antenna_output       : 1;
    bool undefined_timezone_change_scheduled_output : 1;
  } time_data_t;
  
  typedef void (*output_handler_t)(const DCF77::time_data_t &decoded_time);
  
  typedef enum {
    useless  = 0,  // waiting for good enough signal
    dirty    = 1,  // time data available but unreliable
    free     = 2,  // clock was once synced but now may deviate more than 200 ms, must not re-lock if valid phase is detected
    unlocked = 3,  // lock was once synced, inaccuracy below 200 ms, may re-lock if a valid phase is detected
    locked   = 4,  // no valid time data but clock driven by accurate phase
    synced   = 5   // best possible quality, clock is 100% synced
  } clock_state_t;
}

namespace DCF77_Encoder {
  // What *** exactly *** is the semantics of the "Encoder"?
  // It only *** encodes *** whatever time is set
  // It does never attempt to verify the data
  
  void reset(DCF77::time_data_t &now);
  
  uint8_t weekday(const DCF77::time_data_t &now);         // sunday == 0
  BCD::bcd_t bcd_weekday(const DCF77::time_data_t &now);  // sunday == 7
  
  DCF77::tick_t get_current_signal(const DCF77::time_data_t &now);
  
  // This will advance the second. It will consider the control
  // bits while doing so. It will NOT try to properly set the
  // control bits. If this is desired "autoset" must be called in
  // advance.
  void advance_second(DCF77::time_data_t &now);
  
  // The same but for the minute
  void advance_minute(DCF77::time_data_t &now);
  
  // This will set the weekday by evaluating the date.
  void autoset_weekday(DCF77::time_data_t &now);
  
  // This will set the control bits, as a side effect it sets the weekday
  // It will generate the control bits exactly like DCF77 would.
  // Look at the leap second and summer / wintertime transistions
  // to understand the subtle implications.
  void autoset_control_bits(DCF77::time_data_t &now);
  
  void debug(const DCF77::time_data_t &clock);
  void debug(const DCF77::time_data_t &clock, const uint16_t cycles);
  
  // Bit      Bezeichnung     Wert    Pegel   Bedeutung
  // 0        M                       0       Minutenanfang (
  
  // 1..14    n/a                             reserviert
  
  // 15       R                               Reserveantenne aktiv (0 inaktiv, 1 aktiv)
  // 16       A1                              Ankündigung Zeitzonenwechsel (1 Stunde vor dem Wechsel für 1 Stunde, d.h ab Minute 1)
  // 17       Z1               2              Zeitzonenbit Sommerzeit (MEZ = 0, MESZ = 1); also Zeitzone = UTC + 2*Z1 + Z2
  // 18       Z2               1              Zeitzonenbit Winterzeit (MEZ = 1, MESZ = 0); also Zeitzone = UTC + 2*Z1 + Z2
  // 19       A2                              Ankündigung einer Schaltsekunde (1 Stunde vor der Schaltsekunde für 1 Stunde, d.h. ab Minute 1)
  
  // 20       S                       1       Startbit für Zeitinformation
  
  // 21                        1              Minuten  1er
  // 22                        2              Minuten  2er
  // 23                        4              Minuten  4er
  // 24                        8              Minuten  8er
  // 25                       10              Minuten 10er
  // 26                       20              Minuten 20er
  // 27                       40              Minuten 40er
  // 28       P1                              Prüfbit 1 (gerade Parität)
  
  // 29                        1              Stunden  1er
  // 30                        2              Stunden  2er
  // 31                        4              Stunden  4er
  // 32                        8              Stunden  8er
  // 33                       10              Stunden 10er
  // 34                       20              Stunden 20er
  // 35       P2                              Prüfbit 2 (gerade Parität)
  
  // 36                        1              Tag  1er
  // 37                        2              Tag  2er
  // 38                        4              Tag  4er
  // 39                        8              Tag  8er
  // 40                       10              Tag 10er
  // 41                       20              Tag 20er
  
  // 42                        1              Wochentag 1er (Mo = 1, Di = 2, Mi = 3,
  // 43                        2              Wochentag 2er (Do = 4, Fr = 5, Sa = 6,
  // 44                        4              Wochentag 4er (So = 7)
  
  // 45                        1              Monat  1er
  // 46                        2              Monat  2er
  // 47                        4              Monat  4er
  // 48                        8              Monat  8er
  // 49                       10              Monat 10er
  
  // 50                        1              Jahr  1er
  // 51                        2              Jahr  2er
  // 52                        4              Jahr  4er
  // 53                        8              Jahr  8er
  // 54                       10              Jahr 10er
  // 55                       20              Jahr 20er
  // 56                       40              Jahr 40er
  // 57                       80              Jahr 80er
  
  // 58       P3                              Prüftbit 3 (gerade Parität)
  
  // 59       sync                            Sync Marke, kein Impuls (übliches Minutenende)
  // 59                               0       Schaltsekunde (sehr selten, nur nach Ankündigung)
  // 60       sync                            Sync Marke, kein Impuls (nur nach Schaltsekunde)
  
  // Falls eine Schaltsekunde eingefügt wird, wird bei Bit 59 eine Sekundenmarke gesendet.
  // Der Syncimpuls erfolgt dann in Sekunde 60 statt 59. Üblicherweise wird eine 0 als Bit 59 gesendet
  
  // Üblicherweise springt die Uhr beim Wechsel Winterzeit nach Sommerzeit von 1:59:59 auf 3:00:00
  //                               beim Wechsel Sommerzeit nach Winterzeit von 2:59:59 auf 2:00:00
  
  // Die Zeitinformation wird immer 1 Minute im Vorraus übertragen. D.h. nach der Syncmarke hat
  // man die aktuelle Zeit
  
  // http://www.dcf77logs.de/SpecialFiles.aspx
  
  // Schaltsekunden werden in Deutschland von der Physikalisch-Technischen Bundesanstalt festgelegt,
  // die allerdings dazu nur die international vom International Earth Rotation and Reference Systems
  // Service (IERS) festgelegten Schaltsekunden übernimmt. Im Mittel sind Schaltsekunden etwa alle 18
  // Monate nötig und werden vorrangig am 31. Dezember oder 30. Juni, nachrangig am 31. März oder
  // 30. September nach 23:59:59 UTC (also vor 1:00 MEZ bzw. 2:00 MESZ) eingefügt. Seit der Einführung
  // des Systems 1972 wurden ausschließlich die Zeitpunkte im Dezember und Juni benutzt.
  }
  
  namespace Debug {
  void bcddigit(uint8_t data) {
    if (data <= 0x09) {
      consolePrint(data, DEC);
    } else {
      consolePrint('?');
    }
  }
  
  void bcddigits(uint8_t data) {
    bcddigit(data >>  4);
    bcddigit(data & 0xf);
  }
}

namespace BCD {
  void increment(bcd_t &value) {
    if (value.digit.lo < 9) {
      ++value.digit.lo;
    } else {
      value.digit.lo = 0;
  
      if (value.digit.hi < 9) {
        ++value.digit.hi;
      } else {
        value.digit.hi = 0;
      }
    }
  }
  
  bcd_t int_to_bcd(const uint8_t value) {
    const uint8_t hi = value / 10;
  
    bcd_t result;
    result.digit.hi = hi;
    result.digit.lo = value - 10 * hi;
  
    return result;
  }
  
  uint8_t bcd_to_int(const bcd_t value) {
    return value.digit.lo + 10 * value.digit.hi;
  }
}

namespace Arithmetic_Tools {
  template <uint8_t N> inline void bounded_increment(uint8_t &value) __attribute__((always_inline));
  template <uint8_t N>
  void bounded_increment(uint8_t &value) {
    if (value >= 255 - N) {
      value = 255;
    } else {
      value += N;
    }
  }
  
  template <uint8_t N> inline void bounded_decrement(uint8_t &value) __attribute__((always_inline));
  template <uint8_t N>
  void bounded_decrement(uint8_t &value) {
    if (value <= N) {
      value = 0;
    } else {
      value -= N;
    }
  }
  
  inline void bounded_add(uint8_t &value, const uint8_t amount) __attribute__((always_inline));
  void bounded_add(uint8_t &value, const uint8_t amount) {
    if (value >= 255 - amount) {
      value = 255;
    } else {
      value += amount;
    }
  }
  
  inline void bounded_sub(uint8_t &value, const uint8_t amount) __attribute__((always_inline));
  void bounded_sub(uint8_t &value, const uint8_t amount) {
    if (value <= amount) {
      value = 0;
    } else {
      value -= amount;
    }
  }
  
  inline uint8_t bit_count(const uint8_t value) __attribute__((always_inline));
  uint8_t bit_count(const uint8_t value) {
    const uint8_t tmp1 = (value & 0b01010101) + ((value >> 1) & 0b01010101);
    const uint8_t tmp2 = (tmp1  & 0b00110011) + ((tmp1 >> 2) & 0b00110011);
    return (tmp2 & 0x0f) + (tmp2 >> 4);
  }
  
  inline uint8_t parity(const uint8_t value) __attribute__((always_inline));
  uint8_t parity(const uint8_t value) {
    uint8_t tmp = value;
  
    tmp = (tmp & 0xf) ^ (tmp >> 4);
    tmp = (tmp & 0x3) ^ (tmp >> 2);
    tmp = (tmp & 0x1) ^ (tmp >> 1);
  
    return tmp;
  }
  
  void minimize(uint8_t &minimum, const uint8_t value) {
    if (value < minimum) {
      minimum = value;
    }
  }
  
  void maximize(uint8_t &maximum, const uint8_t value) {
    if (value > maximum) {
      maximum = value;
    }
  }
  
  uint8_t set_bit(const uint8_t data, const uint8_t number, const uint8_t value) {
    return value ? data | (1 << number) : data & ~(1 << number);
  }
}

namespace DCF77_Encoder {
  using namespace DCF77;
  
  inline uint8_t days_per_month(const DCF77::time_data_t &now) __attribute__((always_inline));
  uint8_t days_per_month(const DCF77::time_data_t &now) {
    switch (now.month.val) {
      case 0x02:
        // valid till 31.12.2399
        // notice year mod 4 == year & 0x03
        return 28 + ((now.year.val != 0) && ((bcd_to_int(now.year) & 0x03) == 0) ? 1 : 0);
      case 0x01: case 0x03: case 0x05: case 0x07: case 0x08: case 0x10: case 0x12: return 31;
      case 0x04: case 0x06: case 0x09: case 0x11:                  return 30;
      default: return 0;
    }
  }
  
  void reset(DCF77::time_data_t &now) {
    now.second    = 0;
    now.minute.val  = 0x00;
    now.hour.val  = 0x00;
    now.day.val   = 0x01;
    now.month.val   = 0x01;
    now.year.val  = 0x00;
    now.weekday.val = 0x01;
    now.uses_summertime           = false;
    now.uses_backup_antenna       = false;
    now.timezone_change_scheduled = false;
    now.leap_second_scheduled     = false;
  
    now.undefined_minute_output                    = false;
    now.undefined_uses_summertime_output           = false;
    now.undefined_uses_backup_antenna_output       = false;
    now.undefined_timezone_change_scheduled_output = false;
  }
  
  uint8_t weekday(const DCF77::time_data_t &now) {  // attention: sunday will be ==0 instead of 7
    using namespace BCD;
  
    if (now.day.val <= 0x31 && now.month.val <= 0x12 && now.year.val <= 0x99) {
      // This will compute the weekday for each year in 2001-2099.
      // If you really plan to use my code beyond 2099 take care of this
      // on your own. My assumption is that it is even unclear if DCF77
      // will still exist then.
  
      // http://de.wikipedia.org/wiki/Gau%C3%9Fsche_Wochentagsformel
      const uint8_t  d = bcd_to_int(now.day);
      const uint16_t m = now.month.val <= 0x02 ? now.month.val + 10 :
                         bcd_to_int(now.month) - 2;
      const uint8_t  y = bcd_to_int(now.year) - (now.month.val <= 0x02);
      // m must be of type uint16_t otherwise this will compute crap
      uint8_t day_mod_7 = d + (26 * m - 2) / 10 + y + y / 4;
      // We exploit 8 mod 7 = 1
      while (day_mod_7 >= 7) {
        day_mod_7 -= 7;
        day_mod_7 = (day_mod_7 >> 3) + (day_mod_7 & 7);
      }
  
      return day_mod_7;  // attention: sunday will be == 0 instead of 7
    } else {
      return 0xff;
    }
  }
  
  BCD::bcd_t bcd_weekday(const DCF77::time_data_t &now) {
    BCD::bcd_t today;
  
    today.val = weekday(now);
    if (today.val == 0) {
      today.val = 7;
    }
  
    return today;
  }
  
  void autoset_weekday(DCF77::time_data_t &now) {
    now.weekday = bcd_weekday(now);
  }
  
  void autoset_timezone(DCF77::time_data_t &now) {
    // timezone change may only happen at the last sunday of march / october
    // the last sunday is always somewhere in [25-31]
  
    // Wintertime --> Summertime happens at 01:00 UTC == 02:00 CET == 03:00 CEST,
    // Summertime --> Wintertime happens at 01:00 UTC == 02:00 CET == 03:00 CEST
  
    if (now.month.val < 0x03) {
      // January or February
      now.uses_summertime = false;
    } else if (now.month.val == 0x03) {
      // March
      if (now.day.val < 0x25) {
        // Last Sunday of March must be 0x25-0x31
        // Thus still to early for summertime
        now.uses_summertime = false;
      } else if (uint8_t wd = weekday(now)) {
        // wd != 0 --> not a Sunday
        if (now.day.val - wd < 0x25) {
          // early March --> wintertime
          now.uses_summertime = false;
        } else {
          // late march summertime
          now.uses_summertime = true;
        }
      } else {
        // last sunday of march
        // decision depends on the current hour
        now.uses_summertime = (now.hour.val > 2);
      }
    } else if (now.month.val < 0x10) {
      // April - September
      now.uses_summertime = true;
    } else if (now.month.val == 0x10) {
      // October
      if (now.day.val < 0x25) {
        // early October
        now.uses_summertime = true;
      } else if (uint8_t wd = weekday(now)) {
        // wd != 0 --> not a Sunday
        if (now.day.val - wd < 0x25) {
          // early October --> summertime
          now.uses_summertime = true;
        } else {
          // late October --> wintertime
          now.uses_summertime = false;
        }
      } else {  // last sunday of october
        if (now.hour.val == 2) {
          // can not derive the flag from time data
          // this is the only time the flag is derived
          // from the flag vector
  
        } else {
          // decision depends on the current hour
          now.uses_summertime = (now.hour.val < 2);
        }
      }
    } else {
      // November and December
      now.uses_summertime = false;
    }
  }
  
  void autoset_timezone_change_scheduled(DCF77::time_data_t &now) {
    // summer/wintertime change will always happen
    // at clearly defined hours
    // http://www.gesetze-im-internet.de/sozv/__2.html
  
    // in doubt have a look here: http://www.dcf77logs.de/
    if (now.day.val < 0x25 || weekday(now) != 0) {
      // timezone change may only happen at the last sunday of march / october
      // the last sunday is always somewhere in [25-31]
  
      // notice that undefined (==0xff) day/weekday data will not cause any action
      now.timezone_change_scheduled = false;
    } else {
      if (now.month.val == 0x03) {
        if (now.uses_summertime) {
          now.timezone_change_scheduled = (now.hour.val == 0x03 && now.minute.val == 0x00); // wintertime to summertime, preparing first minute of summertime
        } else {
          now.timezone_change_scheduled = (now.hour.val == 0x01 && now.minute.val != 0x00); // wintertime to summertime
        }
      } else if (now.month.val == 0x10) {
        if (now.uses_summertime) {
          now.timezone_change_scheduled = (now.hour.val == 0x02 && now.minute.val != 0x00); // summertime to wintertime
        } else {
          now.timezone_change_scheduled = (now.hour.val == 0x02 && now.minute.val == 0x00); // summertime to wintertime, preparing first minute of wintertime
        }
      } else if (now.month.val <= 0x12) {
        now.timezone_change_scheduled = false;
      }
    }
  }
  
  void verify_leap_second_scheduled(DCF77::time_data_t &now) {
    // If day or month are unknown we default to "no leap second" because this is alway a very good guess.
    // If we do not know for sure we are either acquiring a lock right now --> we will easily recover from a wrong guess
    // or we have very noisy data --> the leap second bit is probably noisy as well --> we should assume the most likely case
  
    now.leap_second_scheduled &= (now.day.val == 0x01);
  
    // leap seconds will always happen at 00:00 UTC == 01:00 CET == 02:00 CEST
    if (now.month.val == 0x01) {
      now.leap_second_scheduled &= ((now.hour.val == 0x00 && now.minute.val != 0x00) ||
                                    (now.hour.val == 0x01 && now.minute.val == 0x00));
    } else if (now.month.val == 0x07 || now.month.val == 0x04 || now.month.val == 0x10) {
      now.leap_second_scheduled &= ((now.hour.val == 0x01 && now.minute.val != 0x00) ||
                                    (now.hour.val == 0x02 && now.minute.val == 0x00));
    } else {
      now.leap_second_scheduled = false;
    }
  }
  
  void autoset_control_bits(DCF77::time_data_t &now) {
    autoset_weekday(now);
    autoset_timezone(now);
    autoset_timezone_change_scheduled(now);
    // we can not compute leap seconds, we can only verify if they might happen
    verify_leap_second_scheduled(now);
  }
  
  void advance_second(DCF77::time_data_t &now) {
    // in case some value is out of range it will not be advanced
    // this is on purpose
    if (now.second < 59) {
      ++now.second;
      if (now.second == 15) {
        autoset_control_bits(now);
      }
  
    } else if (now.leap_second_scheduled && now.second == 59 && now.minute.val == 0x00) {
      now.second = 60;
      now.leap_second_scheduled = false;
    } else if (now.second == 59 || now.second == 60) {
      now.second = 0;
      advance_minute(now);
    }
  }
  
  void advance_minute(DCF77::time_data_t &now) {
    if (now.minute.val < 0x59) {
      increment(now.minute);
    } else if (now.minute.val == 0x59) {
      now.minute.val = 0x00;
      // in doubt have a look here: http://www.dcf77logs.de/
      if (now.timezone_change_scheduled && !now.uses_summertime && now.hour.val == 0x01) {
        // Wintertime --> Summertime happens at 01:00 UTC == 02:00 CET == 03:00 CEST,
        // the clock must be advanced from 01:59 CET to 03:00 CEST
        increment(now.hour);
        increment(now.hour);
        now.uses_summertime = true;
      }  else if (now.timezone_change_scheduled && now.uses_summertime && now.hour.val == 0x02) {
        // Summertime --> Wintertime happens at 01:00 UTC == 02:00 CET == 03:00,
        // the clock must be advanced from 02:59 CEST to 02:00 CET
        now.uses_summertime = false;
      } else {
        if (now.hour.val < 0x23) {
          increment(now.hour);
        } else if (now.hour.val == 0x23) {
          now.hour.val = 0x00;
  
          if (now.weekday.val < 0x07) {
            increment(now.weekday);
          } else if (now.weekday.val == 0x07) {
            now.weekday.val = 0x01;
          }
  
          if (bcd_to_int(now.day) < days_per_month(now)) {
            increment(now.day);
          } else if (bcd_to_int(now.day) == days_per_month(now)) {
            now.day.val = 0x01;
  
            if (now.month.val < 0x12) {
              increment(now.month);
            } else if (now.month.val == 0x12) {
              now.month.val = 0x01;
  
              if (now.year.val < 0x99) {
                increment(now.year);
              } else if (now.year.val == 0x99) {
                now.year.val = 0x00;
              }
            }
          }
        }
      }
    }
  }
  
  DCF77::tick_t get_current_signal(const DCF77::time_data_t &now) {
    using namespace Arithmetic_Tools;
  
    if (now.second >= 1 && now.second <= 14) {
      // weather data or other stuff we can not compute
      return undefined;
    }
  
    bool result;
    switch (now.second) {
      case 0:  // start of minute
        return short_tick;
  
      case 15:
        if (now.undefined_uses_backup_antenna_output) { return undefined; }
        result = now.uses_backup_antenna; break;
  
      case 16:
        if (now.undefined_timezone_change_scheduled_output) { return undefined; }
        result = now.timezone_change_scheduled; break;
  
      case 17:
        if (now.undefined_uses_summertime_output) { return undefined; }
        result = now.uses_summertime; break;
  
      case 18:
        if (now.undefined_uses_summertime_output) { return undefined; }
        result = !now.uses_summertime; break;
  
      case 19:
        result = now.leap_second_scheduled; break;
  
      case 20:  // start of time information
        return long_tick;
  
      case 21:
        if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
        result = now.minute.digit.lo & 0x1; break;
      case 22:
        if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
        result = now.minute.digit.lo & 0x2; break;
      case 23:
        if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
        result = now.minute.digit.lo & 0x4; break;
      case 24:
        if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
        result = now.minute.digit.lo & 0x8; break;
  
      case 25:
        if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
        result = now.minute.digit.hi & 0x1; break;
      case 26:
        if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
        result = now.minute.digit.hi & 0x2; break;
      case 27:
        if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
        result = now.minute.digit.hi & 0x4; break;
  
      case 28:
        if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
        result = parity(now.minute.val); break;
  
  
      case 29:
        if (now.hour.val > 0x23) { return undefined; }
        result = now.hour.digit.lo & 0x1; break;
      case 30:
        if (now.hour.val > 0x23) { return undefined; }
        result = now.hour.digit.lo & 0x2; break;
      case 31:
        if (now.hour.val > 0x23) { return undefined; }
        result = now.hour.digit.lo & 0x4; break;
      case 32:
        if (now.hour.val > 0x23) { return undefined; }
        result = now.hour.digit.lo & 0x8; break;
  
      case 33:
        if (now.hour.val > 0x23) { return undefined; }
        result = now.hour.digit.hi & 0x1; break;
      case 34:
        if (now.hour.val > 0x23) { return undefined; }
        result = now.hour.digit.hi & 0x2; break;
  
      case 35:
        if (now.hour.val > 0x23) { return undefined; }
        result = parity(now.hour.val); break;
  
      case 36:
        if (now.day.val > 0x31) { return undefined; }
        result = now.day.digit.lo & 0x1; break;
      case 37:
        if (now.day.val > 0x31) { return undefined; }
        result = now.day.digit.lo & 0x2; break;
      case 38:
        if (now.day.val > 0x31) { return undefined; }
        result = now.day.digit.lo & 0x4; break;
      case 39:
        if (now.day.val > 0x31) { return undefined; }
        result = now.day.digit.lo & 0x8; break;
  
      case 40:
        if (now.day.val > 0x31) { return undefined; }
        result = now.day.digit.hi & 0x1; break;
      case 41:
        if (now.day.val > 0x31) { return undefined; }
        result = now.day.digit.hi & 0x2; break;
  
      case 42:
        if (now.weekday.val > 0x7) { return undefined; }
        result = now.weekday.val & 0x1; break;
      case 43:
        if (now.weekday.val > 0x7) { return undefined; }
        result = now.weekday.val & 0x2; break;
      case 44:
        if (now.weekday.val > 0x7) { return undefined; }
        result = now.weekday.val & 0x4; break;
  
      case 45:
        if (now.month.val > 0x12) { return undefined; }
        result = now.month.digit.lo & 0x1; break;
      case 46:
        if (now.month.val > 0x12) { return undefined; }
        result = now.month.digit.lo & 0x2; break;
      case 47:
        if (now.month.val > 0x12) { return undefined; }
        result = now.month.digit.lo & 0x4; break;
      case 48:
        if (now.month.val > 0x12) { return undefined; }
        result = now.month.digit.lo & 0x8; break;
  
      case 49:
        if (now.month.val > 0x12) { return undefined; }
        result = now.month.digit.hi & 0x1; break;
  
      case 50:
        if (now.year.val > 0x99) { return undefined; }
        result = now.year.digit.lo & 0x1; break;
      case 51:
        if (now.year.val > 0x99) { return undefined; }
        result = now.year.digit.lo & 0x2; break;
      case 52:
        if (now.year.val > 0x99) { return undefined; }
        result = now.year.digit.lo & 0x4; break;
      case 53:
        if (now.year.val > 0x99) { return undefined; }
        result = now.year.digit.lo & 0x8; break;
  
      case 54:
        if (now.year.val > 0x99) { return undefined; }
        result = now.year.digit.hi & 0x1; break;
      case 55:
        if (now.year.val > 0x99) { return undefined; }
        result = now.year.digit.hi & 0x2; break;
      case 56:
        if (now.year.val > 0x99) { return undefined; }
        result = now.year.digit.hi & 0x4; break;
      case 57:
        if (now.year.val > 0x99) { return undefined; }
        result = now.year.digit.hi & 0x8; break;
  
      case 58:
        if (now.weekday.val > 0x07 ||
            now.day.val     > 0x31 ||
            now.month.val   > 0x12 ||
            now.year.val    > 0x99) { return undefined; }
  
        result = parity(now.day.digit.lo)   ^
                 parity(now.day.digit.hi)   ^
                 parity(now.month.digit.lo) ^
                 parity(now.month.digit.hi) ^
                 parity(now.weekday.val)    ^
                 parity(now.year.digit.lo)  ^
                 parity(now.year.digit.hi); break;
  
      case 59:
        // special handling for leap seconds
        if (now.leap_second_scheduled && now.minute.val == 0) { result = 0; break; }
      // standard case: fall through to "sync_mark"
      case 60:
        return sync_mark;
  
      default:
        return undefined;
    }
  
    return result ? long_tick : short_tick;
  }

  void debug(const DCF77::time_data_t &clock) {
    using namespace Debug;
    using namespace std;
  
    consolePrint(F("  "));
    bcddigits(clock.year.val);
    consolePrint('.');
    bcddigits(clock.month.val);
    consolePrint('.');
    bcddigits(clock.day.val);
    consolePrint('(');
    bcddigit(clock.weekday.val);
    consolePrint(',');
    bcddigit(weekday(clock));
    consolePrint(')');
    bcddigits(clock.hour.val);
    consolePrint(':');
    bcddigits(clock.minute.val);
    consolePrint(':');
    if (clock.second < 10) {
      consolePrint('0');
    }
    consolePrint(clock.second, DEC);
    if (clock.uses_summertime) {
      consolePrint(F(" MESZ "));
    } else {
      consolePrint(F(" MEZ "));
    }
    if (clock.leap_second_scheduled) {
      consolePrint(F("leap second scheduled"));
    }
    if (clock.timezone_change_scheduled) {
      consolePrint(F("time zone change scheduled"));
    }
  }
}

static uint8_t times_100ms = 0;
uint8_t stop_modulation_after_times_100ms = 5;
DCF77::time_data_t now;

void modulate() {
  if (times_100ms == 0) {
    // start of second
    if (times_100ms != stop_modulation_after_times_100ms) {
      digitalWrite(DCF_PIN, LOW);
    }
  }

  if (times_100ms == stop_modulation_after_times_100ms) {
    digitalWrite(DCF_PIN, HIGH);
  }

  if (times_100ms < 9) {
    ++times_100ms;
  } else {
    // after 900ms
    times_100ms = 0;

    DCF77_Encoder::advance_second(now);
    DCF77::time_data_t l_now = now;
    DCF77_Encoder::advance_minute(l_now);
    const DCF77::tick_t output_tick = DCF77_Encoder::get_current_signal(l_now);
    stop_modulation_after_times_100ms = output_tick == DCF77::long_tick  ? 2:
                                        output_tick == DCF77::short_tick ? 1:
                                        output_tick == DCF77::undefined  ? 1:
                                                                           0;
  }
}

// END of DCF77generator
//--------------------

void setup() {
  pinMode(DCF_PIN, OUTPUT);
  Serial.begin(115200);
  initWifi();
  initTime();
  server.begin();
  server.setNoDelay(true);
}

void loop() {
  serialEvent();
  serialCommands();
  telnetEvent();
  telnetCommands();
  delay(1);
  if ((millis()-olddcftime)>=PERIOD_DCF_MS) {
    olddcftime = millis();
    if (flags.synced && flags.setnow) checkSetNow();
    modulate();
  }
  if ((millis()-oldsyncedtime)>=PERIOD_SYNCED_S*1000) {
    oldsyncedtime = millis();
    if (!flags.synced) syncNTP();
  }
  if ((millis()-oldntptime)>=PERIOD_NTP_S*1000) {
    oldntptime = millis();
    flags.synced = false;
    flags.setnow = true;
  }
}

void initWifi() {
  consolePrintLn("");
  consolePrint(F("Connecting to "));
  consolePrintLn(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    consolePrint(".");
  }
  consolePrintLn("");
  consolePrintLn(F("WiFi connected"));
  consolePrint(F("IP address: "));
  IPAddress ip = WiFi.localIP();
  sprintf(formatString, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  consolePrintLn(formatString);
}

void stopWifi() {
  consolePrintLn(F("Disconnecting WiFi ..."));
  WiFi.disconnect();
}

void initTime() {
  resetNow();
  syncNTP();
}

void resetNow() {
  now.year.val = 0x01;
  now.month.val = 0x01;
  now.day.val = 0x01;
  now.weekday.val = 1;
  now.hour.val = 0x00;
  now.minute.val = 0x00;
  now.second = 0;
  now.uses_summertime = 0;
  now.uses_backup_antenna = 0;
  now.timezone_change_scheduled = 0;
  now.leap_second_scheduled = 0;
}

void checkSetNow() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    int cursec = timeinfo.tm_sec;
    if (cursec != oldsec) {
      setNowToTime(&timeinfo);
      flags.setnow = false;
      times_100ms = 0;
    }
    oldsec = cursec;
  }
}

void setNowToTime(struct tm * t) {
  size_t written = strftime(formatString, 32, "%d.%m.%Y %H:%M:%S", t);
  now.day.digit.hi = (uint8_t) formatString[0];
  now.day.digit.lo = (uint8_t) formatString[1];
  now.month.digit.hi = (uint8_t) formatString[3];
  now.month.digit.lo = (uint8_t) formatString[4];
  now.year.digit.hi = (uint8_t) formatString[6];
  now.year.digit.lo = (uint8_t) formatString[7];
  now.hour.digit.hi = (uint8_t) formatString[11];
  now.hour.digit.lo = (uint8_t) formatString[12];
  now.minute.digit.hi = (uint8_t) formatString[14];
  now.minute.digit.lo = (uint8_t) formatString[15];
  now.second = t->tm_sec;
  now.uses_summertime = t->tm_isdst;
}

void syncNTP() {
  struct tm timeinfo;
  consolePrint(F("Synchronizing with "));
  consolePrintLn(NTP_SERVER);
  configTime(NTP_GMT_S, NTP_DAYLIGHT_S, NTP_SERVER);
  if (getLocalTime(&timeinfo)) {
    flags.synced = true;
    flags.setnow = true;
    consolePrint("Time synchronized to ");
    consolePrintTime(&timeinfo);
  } else {
    consolePrintLn("Failed to synchronize time!");
  }
}

void getStatus() {
  struct tm timeinfo;
  boolean getNTPTime;
  // get current times before printing to avoid differences due to printing
  getNTPTime = getLocalTime(&timeinfo);
  DCF77::time_data_t p_now = now;
  if (getNTPTime) {
    consolePrint(F("NTP time: "));
    consolePrintTime(&timeinfo);
    consolePrintLn(F("      dd.mm.YYYY HH:MM:SS"));
  } else {
    consolePrintLn("Failed to get NTP time!");
  }
  consolePrint(F("DCF time: "));
  consolePrintDCF(p_now);
  consolePrintLn(F("      DD.MM.YY hh:mm:ss w sbtl"));
  consolePrintLn(F("      w = weekday, s = summertime, b = backup antenna"));
  consolePrintLn(F("      t = timezone change scheduled, l = leap second scheduled"));
}

void consolePrintNTP() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    consolePrintTime(&timeinfo);
  } else {
    consolePrintLn("Failed to obtain time");
  }
}

template <class T> void consolePrint(T output) {
  uint8_t i;
  if (Serial) Serial.print(output);
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {
      serverClients[i].print(output);
      delay(1);
    }
  }
}

template <class T> void consolePrint(T output, int type) {
  uint8_t i;
  if (Serial) Serial.print(output, type);
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {
      serverClients[i].print(output, type);
      delay(1);
    }
  }
}

template <class T> void consolePrintLn(T output) {
  consolePrint(output);
  consolePrint("\n");
}

template <class T> void consolePrintLn(T output, int type) {
  consolePrint(output, type);
  consolePrint("\n");
}

void consolePrintTime(struct tm * t) {
  size_t written = strftime(formatString, 32, "%d.%m.%Y %H:%M:%S", t);
  consolePrintLn(formatString);
}

void consolePrintDCF(const DCF77::time_data_t &now) {
  const DCF77::time_data_t l_now = now;
  Debug::bcddigits(l_now.day.val);
  consolePrint('.');
  Debug::bcddigits(l_now.month.val);
  consolePrint('.');
  Debug::bcddigits(l_now.year.val);
  consolePrint(' ');
  Debug::bcddigits(l_now.hour.val);
  consolePrint(':');
  Debug::bcddigits(l_now.minute.val);
  consolePrint(':');
  consolePrint(l_now.second / 10);
  consolePrint(l_now.second % 10);
  consolePrint(' ');
  consolePrint(l_now.weekday.val);
  consolePrint(' ');
  consolePrint(l_now.uses_summertime);
  consolePrint(l_now.uses_backup_antenna);
  consolePrint(l_now.timezone_change_scheduled);
  consolePrintLn(l_now.leap_second_scheduled);
}

void procCommands() {
  flags.commandAck = false;
  if (inputString.startsWith(CMD_GETSTATUS)) {
    getStatus();
    flags.commandAck = true;
  }
  if (inputString.startsWith(CMD_RESET)) {
    flags.commandAck = true;
  }
  if (inputString.startsWith(CMD_RESETWIFI)) {
    stopWifi();
    initWifi();
    flags.commandAck = true;
  }
  if (inputString.startsWith(CMD_SYNCTIME)) {
    flags.synced = false;
    flags.commandAck = true;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      inputString.replace("\n", "");
      inputString.replace("\r", "");
      flags.stringComplete = true;
    }
  }
}

void serialCommands() {
  if (flags.stringComplete) {
    procCommands();
    Serial.println(inputString);
    if (flags.commandAck) Serial.println("ACK");
    else Serial.println("ERR");
    Serial.println();
    inputString = "";
    flags.stringComplete = false;
  }
}

void telnetEvent() {
  uint8_t i;
  //check if there are any new clients
  if (server.hasClient()) {
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      //find free/disconnected spot
      if (!serverClients[i] || !serverClients[i].connected()) {
        if (serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
        Serial.print("New client: ");
        Serial.println(i);
        inputString = "";
        flags.stringComplete = false;
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }
  //check clients for data
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {
      if (serverClients[i].available()) {
        //get data from the telnet client and push it to the UART
        while (serverClients[i].available()) {
          char inChar = serverClients[i].read();
          inputString += inChar;
          if (inChar == '\n') {
            inputString = inputString.substring(inputString.indexOf(CMD_START) + 1, inputString.length());
            inputString.replace("\n", "");
            inputString.replace("\r", "");
            flags.stringComplete = true;
          }
        }
      }
    }
  }
}

void telnetCommands() {
  uint8_t i;
  if (flags.stringComplete) {
    procCommands();
    consolePrintLn(inputString);
    if (flags.commandAck) consolePrintLn("ACK");
    else consolePrintLn("ERR");
    consolePrintLn("");
    inputString = "";
    flags.stringComplete = false;
  }
}
