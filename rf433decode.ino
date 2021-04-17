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

//#define SIMULATE
//#define TRACE
#define REC_TIMINGS

#define MAX_DURATION 65535


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

3. About Bands

A band aims to categorize a duration, short or long. Therefore, a Rail is made
of 2 bands, one for the short duration, one for the long duration.

4. About Tracks

Rails live their own live but at some point, they must work in conjunction
(start and stop together, and provide final decoded values). This is the purpose
of a Track, that is made of 2 Rails.

In the end, a Track provides a convenient interface to the caller.

5. Overall schema

track ->  r_low  ->  b_short = manage short duration on LOW signal
      |          `-> b_long  = manage long duration on LOW signal
      |
      `-> r_high ->  b_short = manage short duration on HIGH signal
                 `-> b_long  = manage long duration on HIGH signal

*/


// * **** *********************************************************************
// * Band *********************************************************************
// * **** *********************************************************************

#define BAND_MIN_D    64
#define BAND_MAX_D 20000

struct Band {
    unsigned int inf;
    unsigned int mid;
    unsigned int sup;

    bool got_it;
    bool test_value(unsigned int d);

    void init(unsigned int d);
};

inline void Band::init(unsigned int d) {
    if (d >= BAND_MIN_D && d <= BAND_MAX_D) {
        mid = d;
        unsigned int d_divided_by_4 = d >> 2;
        inf = d - d_divided_by_4;
        sup = d + d_divided_by_4;
        got_it = true;
    } else
        got_it = false;
}

inline bool Band::test_value(unsigned int d) {
    if (!mid) {
        init(d);
#ifdef TRACE
        serial_printf("B> initialized band with %u\n", d);
#endif
    } else {
        got_it = (d >= inf && d <= sup);
#ifdef TRACE
        serial_printf("B> compared d (%u) with [%u, %u]\n", d, inf, sup);
#endif
    }
#ifdef TRACE
    serial_printf("B> result = %d\n", got_it);
#endif
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
        Band b_short;
        Band b_long;
        uint32_t recorded;
        byte status;
        byte index;

    public:
        Rail();
        bool rail_eat(unsigned int d);
        void reset();
        void rail_debug() const;
//        byte get_nth_bit(byte n) const;
        byte get_band_count() const;
};

Rail::Rail() {
    reset();
}

inline void Rail::reset() {
    status = RAIL_OPEN;
    index = 0;

    b_short.mid = 0;
    b_long.mid = 0;

    recorded = 0;
}

inline bool Rail::rail_eat(unsigned int d) {

#ifdef TRACE
        // FIXME
    serial_printf("R> index = %d, d = %u\n", index, d);
#endif

    if (status != RAIL_OPEN)
        return false;

    byte count_got_it = 0;
    if (b_short.test_value(d))
        ++count_got_it;
    if (b_long.test_value(d))
        ++count_got_it;

    byte band_count = get_band_count();

#ifdef TRACE
        // FIXME
    serial_printf("R> b_short.got_it = %d, b_long.got_it = %d, "
                  "band_count = %d\n", b_short.got_it, b_long.got_it,
                  band_count);
    for (int i = 0; i < 2; ++i) {
        serial_printf("R>  [%i]: inf = %u, mid = %u, sup = %u\n", i,
                      (i == 0 ? b_short.inf : b_long.inf),
                      (i == 0 ? b_short.mid : b_long.mid),
                      (i == 0 ? b_short.sup : b_long.sup));
    }
#endif

    if (band_count == 1 && !count_got_it) {
        Band *pband;
        unsigned int small;
        unsigned int big;
        if (d < b_short.inf) {
            pband = &b_short;
            small = d;
            big = b_short.mid;
        } else if (d > b_short.sup) {
            pband = &b_long;
            small = b_short.mid;
            big = d;
        } else {
                // Should not happen.
                // If value is within band range, then why the hell didn't the
                // range grab it?
            FATAL;
        }

#ifdef TRACE
        serial_printf("R> P0\n");
#endif

        if ((small << 2) >= big) {
            pband->init(d);
            if (pband->got_it) {

#ifdef TRACE
                serial_printf("R> P1\n");
#endif

                count_got_it = 1;
                band_count = 2;

                    // FIXME
                    //   Test if intervals overlap?
                    //   That is, test if b_short.sup >= b_long.inf?
                ;

                if (pband == &b_short) {
                        // The first N signals received ('N' equals 'index')
                        // happened to be LONG ones => to be recorded as as many
                        // ONEs.
                    recorded = ((uint32_t)1 << index) - 1;
                }
            }
        }
    }

    if (!band_count) {
        status = RAIL_ERROR;
        return false;
    }

    if (!count_got_it || (band_count == 2 && count_got_it == 2)) {
        status = (d >= (b_short.mid << 1) && d >= (b_long.mid << 1))
                 ? RAIL_STP_RCVD : RAIL_ERROR;
        return false;
    }

    if (band_count == 2) {
        if (b_short.got_it == b_long.got_it) {
            FATAL;
        }
        recorded = (recorded << 1) | (b_short.got_it ? 0 : 1);
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
    serial_printf("\"n\":%d,\n", b_short.mid == b_long.mid ? 1 : 2);
    for (byte i = 0; i < 2; ++i) {
        serial_printf("      \"%s\":{", (i == 0 ? "b_short" : "b_long"));
        serial_printf("\"inf\":%u,\"mid\":%u,\"sup\":%u",
                      (i == 0 ? b_short.inf : b_long.inf),
                      (i == 0 ? b_short.mid : b_long.mid),
                      (i == 0 ? b_short.sup : b_long.sup));
        if (b_short.mid == b_long.mid) {
            serial_printf("},\n      \"b_long\":\"*****\"\n");
            break;
        }
        serial_printf("}%s\n", i == 1 ? "" : ",");
    }
}

//byte Rail::get_nth_bit(byte n) const {
//    return !!(recorded & (uint32_t)1 << n);
//}

byte Rail::get_band_count() const {
    return b_short.mid == b_long.mid ? (b_short.mid ? 1 : 0) : 2;
}


// * ***** ********************************************************************
// * Track ********************************************************************
// * ***** ********************************************************************

#define TRACK_MIN_INITSEQ_DURATION 4000
#define TRACK_MIN_BITS             8

typedef enum {TRK_WAIT, TRK_RECV, TRK_DATA} trk_t;
class Track {
    private:
        volatile trk_t trk;
        Rail r_low;
        Rail r_high;

#ifdef REC_TIMINGS
        unsigned int timings[80];
        unsigned int exec_d[80];
        int timing_pos;
#endif

    public:
        Track();

        void treset();
        void track_eat(byte n, unsigned int d);
        void track_debug() const;
        byte get_nb_bits() const;

        trk_t get_trk() const { return trk; }
        bool rails_have_2_bands() const;

#ifdef REC_TIMINGS
        void record_exec_d(unsigned int d);
#endif
};

Track::Track() {
    treset();
}

inline void Track::treset() {
    trk = TRK_WAIT;
#ifdef REC_TIMINGS
    timing_pos = 0;
#endif
}

inline void Track::track_eat(byte r, unsigned int d) {

#ifdef TRACE
        // FIXME
    serial_printf("T> trk = %d, r = %d, d = %u\n", trk, r, d);
#endif

    if (trk == TRK_WAIT) {
        if (r == 1 && d >= TRACK_MIN_INITSEQ_DURATION) {
            r_low.reset();
            r_high.reset();
            trk = TRK_RECV;
#ifdef REC_TIMINGS
            timings[timing_pos++] = d;
#endif
        }
        return;
    } else if (trk != TRK_RECV) {
        return;
    }

    Rail *prail = (r == 0 ? &r_low : &r_high);
    if (prail->status != RAIL_OPEN)
        return;

#ifdef REC_TIMINGS
    timings[timing_pos++] = d;
#endif

    bool b = prail->rail_eat(d);
    if (!b || (r == 1 && r_low.status != RAIL_OPEN)) {

#ifdef TRACE
        serial_printf("T> b = %d\n", b);
#endif

        if (r_low.index < TRACK_MIN_BITS || r_high.index < TRACK_MIN_BITS) {

#ifdef TRACE
            serial_printf("T> P0\n");
#endif

            treset();
            // WARNING
            // Re-entrant call... not ideal, but, avoids to re-write what is to
            // be done if the condition of a new signal is met <=>
            //   (r == 1 && d >= TRACK_MIN_INITSEQ_DURATION)
            track_eat(r, d);
        } else if (r == 1) {
            Rail *prail = nullptr;
            if (r_low.index >= 1 && r_high.index >= 1) {
                if (r_low.index > r_high.index) {
                    prail = &r_low;
                } else if (r_high.index > r_low.index) {
                    prail = &r_high;
                }
                if (prail) {
                    prail->recorded >>= 1;
                    prail->index--;
                }
            }
            if (r_high.index != r_low.index) {
                FATAL;
            }

            if (r_low.status == RAIL_OPEN)
                r_low.status = RAIL_CLOSED;
            if (r_high.status == RAIL_OPEN)
                r_high.status = RAIL_CLOSED;

            trk = TRK_DATA;
        }
    }
}

#ifdef REC_TIMINGS
void Track::record_exec_d(unsigned int d) {
    if (trk == TRK_RECV && timing_pos >= 1)
        exec_d[timing_pos - 1] = d;
}
#endif

void Track::track_debug() const {
    serial_printf("    \"trk\":");
    if (trk == TRK_WAIT) {
        serial_printf("\"TRK_WAIT\"\n");
    } else if (trk == TRK_RECV) {
        serial_printf("\"TRK_RECV\",");
    } else if (trk == TRK_DATA) {
        serial_printf("\"TRK_DATA\",");
    } else {
        serial_printf("\"(UNKNOWN)\",");
    }
    uint32_t xorval = r_low.recorded ^ r_high.recorded;
    if (trk != TRK_WAIT) {
        serial_printf("\"xorval\":0x%08lx,\n", xorval);
        for (byte i = 0; i < 2; ++i) {
            serial_printf("    \"%s\":{\n", (i == 0 ? "r_low" : "r_high"));
            (i == 0 ? &r_low : &r_high)->rail_debug();
            serial_printf("    }%s\n", i == 1 ? "" : ",");
        }
    }

#ifdef REC_TIMINGS
    serial_printf("%4s, %4d\n", "", timings[0]);
    for (int i = 1; i < timing_pos - 1; i += 2) {
        serial_printf("%4d, %4d  |  %5d, %5d\n", timings[i], timings[i + 1],
                      exec_d[i], exec_d[i + 1]);
    }
#endif
}

byte Track::get_nb_bits() const {
    if (trk == TRK_WAIT)
        return 0;
    return r_low.index < r_high.index ? r_low.index : r_high.index;
}

    // Returns true if both rails have 2 bands, meaning, the signal received
    // showed both short and long signal durations.
    // If one rail has one band, that means we don't know if signal on this band
    // is 'short' or 'long' (= the duration was constant).
    // This can be important to know, because if the function returns false, the
    // caller should NOT consider the signal received contains information (this
    // most likely corresponds to a sync prefix before real information
    // encoding).
bool Track::rails_have_2_bands() const {
    return false;
}


// * ************* ************************************************************
// * Interruptions ************************************************************
// * ************* ************************************************************

Track track;

#ifdef SIMULATE
SerialLine sl;
char buffer[SerialLine::buf_len];

unsigned int sim_timings[130];
unsigned int sim_timings_count = 0;

unsigned int sim_int_count = 0;
unsigned int counter;

#endif

bool handle_interrupt_in_progress = false;
void handle_interrupt() {
    static unsigned long last_t = 0;

    if (handle_interrupt_in_progress)
        return;
    handle_interrupt_in_progress = true;

    sei();

    const unsigned long t = micros();

    if (!last_t) {
        last_t = t;
        handle_interrupt_in_progress = false;
        return;
    }

#ifndef SIMULATE
    unsigned long d = t - last_t;
    last_t = t;
    byte r = (digitalRead(PIN_RFINPUT) == HIGH ? 1 : 0);
    if (d > MAX_DURATION)
        d = MAX_DURATION;
#else
    unsigned long d;
    byte r = sim_int_count % 2;
    if (sim_int_count >= sim_timings_count) {
        d = 100;
    } else {
        d = sim_timings[sim_int_count++];
    }
#endif

    track.track_eat(r, d);

#ifdef REC_TIMINGS
    unsigned long t1 = micros();
    track.record_exec_d(t1 - t);
#endif

    handle_interrupt_in_progress = false;
    return;
}


// * ********* ****************************************************************
// * Execution ****************************************************************
// * ********* ****************************************************************

#ifdef SIMULATE
void read_simulated_timings_from_usb() {
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
#endif

void setup() {
    pinMode(PIN_RFINPUT, INPUT);
    Serial.begin(115200);

    sleep_enable();
    set_sleep_mode(SLEEP_MODE_IDLE);
}

void loop() {

#ifdef SIMULATE

    if (sim_int_count >= sim_timings_count)
        read_simulated_timings_from_usb();

    track.treset();
    unsigned int sim_int_count_svg = sim_int_count;
    while (track.get_trk() != TRK_DATA && sim_int_count < sim_timings_count) {
        handle_interrupt();
    }

    if (!counter) {
        delay(100);
        serial_printf("----- BEGIN TEST -----\n");
        serial_printf("[\n");
    }

    ++counter;

    if (track.get_trk() == TRK_DATA) {
        serial_printf("%s  {\n", counter >= 2 ? ",\n" : "");
        serial_printf("    \"N\":%d,\"start\":%u,\"end\":%u,\n",
                      sim_timings_count, sim_int_count_svg, sim_int_count - 1);
        track.track_debug();
    } else {
        serial_printf("\n]\n----- END TEST -----\n\n");
    }

    serial_printf("  }");

#else

    serial_printf("Waiting for signal\n");
    track.treset();

    attachInterrupt(INT_RFINPUT, &handle_interrupt, CHANGE);
    while (track.get_trk() != TRK_DATA) {
        sleep_mode();
    }
    detachInterrupt(INT_RFINPUT);

    track.track_debug();

#endif

}

// vim: ts=4:sw=4:tw=80:et
