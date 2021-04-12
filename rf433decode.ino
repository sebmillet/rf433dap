// rf433decode.ino

/*
  Copyright 2021 Sébastien Millet

  rf433dap is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  rf433dap is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses>.
*/

//
// Schematic:
//   RF433 RECEIVER data pin plugged on Arduino D2
//   See file schema.fzz (Fritzing format) or schema.png
//

// BEGIN SCHEMATIC CONSTANTS
#define PIN_RFINPUT  2
#define INT_RFINPUT  (digitalPinToInterrupt(PIN_RFINPUT))
// END SCHEMATIC CONSTANTS

#include <Arduino.h>
#include <avr/sleep.h>

#define SIMULATE


// * *********************************** **************************************
// * Manage printf-like output to Serial **************************************
// * *********************************** **************************************

#include <stdarg.h>

char serial_printf_buffer[80];
void serial_printf(const char* msg, ...)
     __attribute__((format(printf, 1, 2)));

    // NOTE
    //   Assume Serial has been initialized (Serial.begin(...))
void serial_printf(const char* msg, ...) {
    va_list args;

    va_start(args, msg);

    vsnprintf(serial_printf_buffer, sizeof(serial_printf_buffer), msg, args);
    va_end(args);
    Serial.print(serial_printf_buffer);
}

#define FATAL { serial_printf("FATAL: %s, %i\n", __FILE__, __LINE__); \
        delay(10); \
        while (1) \
            ; \
    }


// * ********************** ***************************************************
// * Read input from serial ***************************************************
// * ********************** ***************************************************

#ifdef SIMULATE

//
// SerialLine
//
// Manages USB input as lines.
//
// Interest = non blocking I/O. Serial.readString() works with timeout and a
// null timeout is not well documented (meaning: even if zeroing timeout leads
// to non-blocking I/O, I'm not sure it'll consistently and robustly *always*
// behave this way).
class SerialLine {
    private:
        char buf[23]; // 20-character strings (then CR+LF then NULL-terminating)
        size_t head;
        bool got_a_line;
        void reset();

    public:
        SerialLine();

        static const size_t buf_len;

        void do_events();
        bool is_line_available();
        bool get_line(char *s, size_t len);
        void get_line_blocking(char *s, size_t len);
        void split_s_into_func_args(char *s, char **func, char **args) const;
};
const size_t SerialLine::buf_len = sizeof(SerialLine::buf);

SerialLine::SerialLine():head(0),got_a_line(false) { };

void SerialLine::do_events() {
    if (got_a_line)
        return;
    if (!Serial.available())
        return;

    int b;
    do {
        b = Serial.read();
        if (b == -1)
            break;
        buf[head++] = (char)b;
    } while (head < buf_len - 1 && b != '\n' && Serial.available());

    if (head < buf_len - 1 && b != '\n')
        return;

    buf[head] = '\0';

        // Remove trailing cr and/or nl
        // FIXME?
        //   WON'T WORK WITH MAC-OS NEWLINES!
        //   (SEE ABOVE: NO STOP IF ONLY CR ENCOUNTERED)
    if (head >= 1 && buf[head - 1] == '\n')
        buf[--head] = '\0';
    if (head >= 1 && buf[head - 1] == '\r')
        buf[--head] = '\0';
    got_a_line = true;
}

bool SerialLine::is_line_available() {
    do_events();
    return got_a_line;
}

void SerialLine::reset() {
    head = 0;
    got_a_line = false;
}

// Get USB input as a simple line, copied in caller buffer.
// A 'line' is a set of non-null characters followed by 'new line', 'new line'
// being either as per Unix or Windows convention, see below.
// Returns true if a copy was done (there was a line available), false if not
// (in which case, s is not updated).
// The terminating newline character (or 2-character CR-LF sequence) is NOT part
// of the string given to the caller.
// If the line length is above the buffer size (SerialLine::buf_len), then it'll
// be cut into smaller pieces.
// Because of the way the received buffer is parsed, and when using CR-LF as
// end-of-line marker (default even under Linux), it can result in a empty
// string seen after a first string with a length close to the limit.
//
// About new lines:
// - Works fine with Unix new lines (\n), tested
// - Supposed to work fine with Windows new lines (\r\n), NOT TESTED
// - WON'T WORK WITH MAC-OS NEW LINES (\r)
bool SerialLine::get_line(char *s, size_t len) {
    do_events();
    if (!got_a_line)
        return false;
    snprintf(s, len, buf);
    reset();
    return true;
}

// Same as get_line, but with blocking I/O =
// Wait without time limit, until a line comes in.
void SerialLine::get_line_blocking(char *s, size_t len) {
    while (!get_line(s, len))
        ;
}

#endif // SIMULATE


// * ******************** *****************************************************
// * Bands, Rails, Tracks *****************************************************
// * ******************** *****************************************************

/*

About classes Band, Rail and Track

1. About none of these - the signal as we see it.

The Radio-Frequence signal is supposed to be OOK (On-Off Keying), and
auto-synchronized.

The signal is a succession of low signal and high signal, low when no RF signal
received, high when a RF signal is received.
The coding relies on durations being either 'short' or 'long', and sometimes
much longer (to initialize, and to separate signal pieces).

The durations can be one of:
    - short
    - long, typically, twice as long as short
    - separator, much longer than the long one (at least 3 or 4 times longer)
    - initialization, at least as long as the separator, often much longer. It
      serves to make receiver ready to receive coded signal to come.

A signal structure is as follows:
    1. Initialization (very long high signal)
    2. Succession of low and high signals being 'short' or 'long'
    3. Separator (high signal)
    4. Possibly, repetition of steps 2 and 3

The succession of 'short' and 'long' is then decoded into original data, either
based on tri-bit scheme, or, Manchester.

Note that there can be complexities:

- After the long initialization high signal, addition of 'intermediate' prefix
  to the signal (longer than 'long', but shorter than 'separator'). Seen on a
  NICE FLO/R telecommand (/R means Rolling Code), while not seen on NICE FLO
  (fix code).

- After the long initialization high signal, succession of {low=short,
  high=short} followed by a separator, *in a tri-bit coding system* (although
  this does not make sense, only short-long or long-short are allowed). This
  serves as a synchronization sequence.

- While most protocols use same lengths for low and high signals, on NICE FLO/R
  this rule is not met, that is: the 'short' and 'long' durations of the low
  signal are different from 'short' and 'long' durations of the high signal.

2. About Rail

The Rail manages the succession of durations for one, and only one, of signal
realms (low or high).

That is, if you note dow the signal as usually (by line, one low followed by one
high):
      LOW, HIGH
      150,  200
      145,  400
      290,  195
        ...

Then the values below LOW (150, 145, 290, ...) are one Rail, and the values
below HIGH (200, 400, 195, ...) are another Rail.

In the code, rails[0] is the one of the low signal, rails[1] is the one of the
high signal.

3. About Bands

A band aims to categorize a duration, short or long. Therefore, a Rail is made
of 2 bands, one for the short duration, one for the long duration.

4. About Tracks

Rails live their own live but at some point, they must work in conjunction
(start and stop together, and provide final decoded values). This is the purpose
of a Track, that is made of 2 Rails.

In the end, a Track provides a convenient interface to the caller.

5. Overall schema

track ->  rails[0] ->  band[0] = manage short duration on LOW signal
      |            `-> band[1] = manage long duration on LOW signal
      |
      `-> rails[1] ->  band[0] = manage short duration on HIGH signal
                   `-> band[1] = manage long duration on HIGH signal

*/


// * **** *********************************************************************
// * Band *********************************************************************
// * **** *********************************************************************

#define ST_WAIT_SIGNAL    0
#define ST_RECORDING      1
#define ST_DATA_AVAILABLE 2

struct Band {
    unsigned long inf;
    unsigned long mid;
    unsigned long sup;

    bool got_it;
    bool test_value(unsigned long d);

    void init(unsigned long d);
};

inline void Band::init(unsigned long d) {
    mid = d;
    inf = d - (d >> 2);
    sup = d + (d >> 2);
    got_it = false;
}

inline bool Band::test_value(unsigned long d) {
    if (!mid) {
        init(d);
        got_it = true;
    } else {
        got_it = (d >= inf && d <= sup);
    }
    return got_it;
}


// * **** *********************************************************************
// * Rail *********************************************************************
// * **** *********************************************************************

#define RAIL_OPEN     0
#define RAIL_FULL     1
#define RAIL_STP_RCVD 2
#define RAIL_CLOSED   3
#define RAIL_ERROR    4
class Rail {
    friend class Track;

    private:
        Band band[2];
        uint32_t recorded;
        byte status;
        byte index;

    public:
        Rail();
        bool eat(unsigned long d);
        void reset();
        void rail_debug() const;
        byte get_nth_bit(byte n) const;
};

Rail::Rail() {
    reset();
}

inline void Rail::reset() {
    status = RAIL_OPEN;
    index = 0;

    for (byte i = 0; i < 2; ++i)
        band[i].mid = 0;

    recorded = 0;
}

bool Rail::eat(unsigned long d) {
    if (status != RAIL_OPEN)
        return false;

    byte count_got_it = 0;
    if (band[0].test_value(d))
        ++count_got_it;
    if (band[1].test_value(d))
        ++count_got_it;

    byte band_count = (band[0].mid == band[1].mid ? 1 : 2);

    if (band_count == 1 && !count_got_it) {
        byte new_band;
        if (d < band[0].inf) {
            new_band = 0;
        } else if (d > band[0].sup) {
            new_band = 1;
        } else {
                // Should not happen.
                // If value is within band range, then why the hell didn't the
                // range grab it?
            FATAL
        }
        band[new_band].init(d);
        band[new_band].got_it = true;
        ++count_got_it;
        band_count = 2;

            // FIXME
            //   Test if intervals overlap?
            //   (that is, test if band[0].sup >= band[1].inf)
        ;

        if (new_band == 0) {
                // The first N signals received ('N' equals 'index') happened to
                // be LONG ones => to be recorded as as many ONEs.
            recorded = ((uint32_t)1 << index) - 1;
        }
    }

    if (!count_got_it || (band_count == 2 && count_got_it == 2)) {
        status = (d >= (band[0].mid << 1) && d >= (band[1].mid << 1))
                 ? RAIL_STP_RCVD : RAIL_ERROR;
        return false;
    }

    if (band_count == 2) {
        if (band[0].got_it == band[1].got_it) {
            FATAL;
        }
        recorded = (recorded << 1) | (band[0].got_it ? 0 : 1);
    }
    if (++index == 32) {
        status = RAIL_FULL;
    }

    return (status == RAIL_OPEN);
}

void Rail::rail_debug() const {
    serial_printf("      \"bits\":%i,\"v\":0x%08lx,\"railstatus\":",
                  index, recorded);
    if (status == RAIL_OPEN) {
        serial_printf("\"open\",");
    } else if (status == RAIL_FULL) {
        serial_printf("\"full\",");
    } else if (status == RAIL_STP_RCVD) {
        serial_printf("\"stop received\",");
    } else if (status == RAIL_CLOSED) {
        serial_printf("\"closed\",");
    } else if (status == RAIL_ERROR) {
        serial_printf("\"error\",");
    } else {
        serial_printf("\"unknown\",");
    }
    serial_printf("\"n\":%d,\n", band[0].mid == band[1].mid ? 1 : 2);
    for (byte i = 0; i < 2; ++i) {
        serial_printf("      \"band%d\":{", i);
        serial_printf("\"inf\":%lu,\"mid\":%lu,\"sup\":%lu",
                      band[i].inf, band[i].mid, band[i].sup);
        if (band[0].mid == band[1].mid) {
            serial_printf("},\n      \"band1\":\"*****\"\n");
            break;
        }
        serial_printf("}%s\n", i == 1 ? "" : ",");
    }
}

byte Rail::get_nth_bit(byte n) const {
    return !!(recorded & (uint32_t)1 << n);
}


// * ***** ********************************************************************
// * Track ********************************************************************
// * ***** ********************************************************************

#define TS_OPEN       0
#define TS_FULL       1
#define TS_STP_RCVD_L 2
#define TS_STP_RCVD_H 3
#define TS_ERROR      4
class Track {
    private:
        Rail rails[2];

    public:
        Track();

        void reset();
        void track_eat(byte n, unsigned long d);
        void track_debug() const;
        byte get_nb_bits() const;
        byte track_get_nth_bit(byte r, byte n) const;
        byte status() const;
};

Track::Track() {
    reset();
}

void Track::reset() {
    rails[0].reset();
    rails[1].reset();
}

void Track::track_eat(byte r, unsigned long d) {
    if (rails[r].status != RAIL_OPEN)
        return;

    if (!rails[r].eat(d) || (r == 1 && rails[0].status != RAIL_OPEN)) {
        if (r == 1) {
            int dec = -1;
            if (rails[0].index >= 1 && rails[1].index >= 1) {
                if (rails[0].index > rails[1].index) {
                    dec = 0;
                } else if (rails[1].index > rails[0].index) {
                    dec = 1;
                }
                if (dec >= 0) {
                    rails[dec].recorded >>= 1;
                    rails[dec].index--;
                }
            }
            if (rails[1].index != rails[0].index) {
                FATAL;
            }
            if (rails[0].status == RAIL_OPEN)
                rails[0].status = RAIL_CLOSED;
            if (rails[1].status == RAIL_OPEN)
                rails[1].status = RAIL_CLOSED;
        }
    }
}

void Track::track_debug() const {
    serial_printf("    \"trackstatus\":");
    if (status() == TS_OPEN) {
        serial_printf("\"open\"\n");
    } else if (status() == TS_FULL) {
        serial_printf("\"full\",");
    } else if (status() == TS_STP_RCVD_L) {
        serial_printf("\"stop received (low)\",");
    } else if (status() == TS_STP_RCVD_H) {
        serial_printf("\"stop received (high)\",");
    } else if (status() == TS_ERROR) {
        serial_printf("\"error\",");
    } else {
        serial_printf("\"unknown\",");
    }
    uint32_t xorval = rails[0].recorded ^ rails[1].recorded;
    if (status() != TS_OPEN) {
        serial_printf("\"xorval\":0x%08lx,\n", xorval);
        for (byte i = 0; i < 2; ++i) {
            serial_printf("    \"rail%d\":{\n", i);
            rails[i].rail_debug();
            serial_printf("    }%s\n", i == 1 ? "" : ",");
        }
    }
}

byte Track::status() const {
    if (rails[0].status == RAIL_OPEN || rails[1].status == RAIL_OPEN)
        return TS_OPEN;
    if (rails[0].status == RAIL_FULL && rails[1].status == RAIL_FULL)
        return TS_FULL;
    if (rails[0].status == RAIL_STP_RCVD)
        return TS_STP_RCVD_L;
    if (rails[1].status == RAIL_STP_RCVD)
        return TS_STP_RCVD_H;
    return TS_ERROR;
}

byte Track::get_nb_bits() const {
    return rails[0].index < rails[1].index ? rails[0].index : rails[1].index;
}

byte Track::track_get_nth_bit(byte r, byte n) const {
    return rails[r].get_nth_bit(n);
}


// * ************* ************************************************************
// * Interruptions ************************************************************
// * ************* ************************************************************

unsigned long start_time;
unsigned long end_time;
unsigned long signal_duration;

volatile unsigned long last_t = 0;
volatile short status = ST_WAIT_SIGNAL;
volatile unsigned int write_head_pos;
//unsigned int write_head_pos_when_consistency_got_lost;
volatile unsigned long timings_d0;
volatile uint32_t timings[80];

Track track;

#ifdef SIMULATE
SerialLine sl;
char buffer[SerialLine::buf_len];

unsigned int sim_timings[110];
unsigned int sim_timings_count = 0;

unsigned int sim_int_count = 0;
unsigned int counter;

#endif

void handle_interrupt() {
    const unsigned long t = micros();

    if (!last_t) {
        last_t = t;
        return;
    }

#ifndef SIMULATE
    const unsigned long d = t - last_t;
    last_t = t;
#else
    unsigned long d;
    if (sim_int_count >= sim_timings_count) {
        d = 100;
    } else {
        d = sim_timings[sim_int_count++];
    }
#endif

    if (status == ST_WAIT_SIGNAL) {
#ifdef SIMULATE
        if (d >= 4000 && !(sim_int_count % 2)) {
#else
        if (d >= 4000 && digitalRead(PIN_RFINPUT) == HIGH) {
#endif
            timings_d0 = d;
            write_head_pos = 0;
            start_time = micros();
            track.reset();
            status = ST_RECORDING;
        }

        return;
    }

    if (status == ST_RECORDING) {
        if (write_head_pos >= 20 || (d >= 60 && d < 10000)) {
            track.track_eat(write_head_pos % 2, d);
            timings[write_head_pos++] = d;
            if (write_head_pos >= sizeof(timings) / sizeof(*timings)) {
                end_time = micros();
                status = ST_DATA_AVAILABLE;
            }
        } else {
            status = ST_WAIT_SIGNAL;
        }

        return;
    }

    if (status == ST_DATA_AVAILABLE) {
        return;
    }

    while (1)
        ;
}


// * ********* ****************************************************************
// * Execution ****************************************************************
// * ********* ****************************************************************

void setup() {
    pinMode(PIN_RFINPUT, INPUT);
    Serial.begin(115200);

    sleep_enable();
    set_sleep_mode(SLEEP_MODE_IDLE);
}

void loop() {

#ifdef SIMULATE

    if (sim_int_count >= sim_timings_count) {
        sim_timings_count = 0;
        sim_int_count = 0;
        counter = 0;
        buffer[0] = '\0';
        for (   ;
                strcmp(buffer, ".");
                sl.get_line_blocking(buffer, sizeof(buffer))
            ) {

            if (!strlen(buffer))
                continue;

            char *p = buffer;
            while (*p != ',' && *p != '\0')
                ++p;
            if (*p != ',') {
                serial_printf("FATAL: each line must have a ',' character!\n");
                FATAL;
            }

            *p = '\0';
            unsigned int l = atoi(buffer);
            unsigned int h = atoi(p + 1);

            if (sim_timings_count >=
                    sizeof(sim_timings) / sizeof(*sim_timings) - 1) {
                serial_printf("FATAL: timings buffer full!\n");
                FATAL;
            }

            sim_timings[sim_timings_count++] = l;
            sim_timings[sim_timings_count++] = h;
        }
    }

    track.reset();
    write_head_pos = 0;
    for (byte i = 0; i < sizeof(timings) / sizeof(*timings); ++i) {
        timings[i] = 0;
    }
    unsigned int sim_int_count_svg = sim_int_count;
    while (track.status() == TS_OPEN
           && sim_int_count < sim_timings_count) {
        handle_interrupt();
    }

    if (!counter) {
        delay(100);
        serial_printf("----- BEGIN TEST -----\n");
        serial_printf("[\n");
    }

    ++counter;

    if (sim_int_count - sim_int_count_svg >= 6) {
        serial_printf("%s  {\n", counter >= 2 ? ",\n" : "");
        serial_printf("    \"N\":%d,\"start\":%u,\"end\":%u,\n",
                      sim_timings_count, sim_int_count_svg, sim_int_count - 1);
    } else {
        serial_printf("\n]\n----- END TEST -----\n\n");
    }

#else

    serial_printf("Waiting for signal\n");

    attachInterrupt(INT_RFINPUT, &handle_interrupt, CHANGE);
    while (status != ST_DATA_AVAILABLE) {
        sleep_mode();
    }
    detachInterrupt(INT_RFINPUT);

    signal_duration = end_time - start_time;

    serial_printf("duration: %lu us\n", signal_duration);
    serial_printf("  N  %6s,%6s\n", "LOW", "HIGH");
    serial_printf("-----BEGIN RF433 LOW HIGH SEQUENCE-----\n");
    serial_printf("     %6s,%6lu\n", "", timings_d0);

    byte nb_bits = track.get_nb_bits();
    for(int i = 0; i + 1 < (nb_bits << 1) + 2; i += 2) {
        byte s = (i >> 1);
        char clow, chigh;
        if (s >= nb_bits) {
            clow = '-';
            chigh = '-';
        } else {
            clow = track.track_get_nth_bit(0, nb_bits - s - 1) ? 'l' :'s';
            chigh = track.track_get_nth_bit(1, nb_bits - s - 1) ? 'l' : 's';
        }
        serial_printf("%03i  %6lu,%6lu  %c, %c\n", s,
                      (unsigned long)timings[i],
                      (unsigned long)timings[i + 1],
                      clow, chigh);
    }
#endif

#ifdef SIMULATE
    if (sim_int_count - sim_int_count_svg >= 6) {
        track.track_debug();
        serial_printf("  }");
    }
#else
    serial_printf("-----END RF433 LOW HIGH SEQUENCE-----\n");
    status = ST_WAIT_SIGNAL;
#endif
}

// vim: ts=4:sw=4:tw=80:et
