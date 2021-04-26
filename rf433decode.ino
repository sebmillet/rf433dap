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

#include "debug.h"

#include <Arduino.h>
#include <avr/sleep.h>

#if TESTPLAN == 1

    // *WARNING*
    //   DON'T UPDATE THE BELOW UNLESS YOU KNOW WHAT YOU ARE DOING!!!
    //
    // If you want to tune such defines, go to the [OK_TO_UPDATE] paragraph
    // below.
    // The test plan requires very specific defines to be set, hence this logic.
#define SIMULATE
#define TRACKDEBUG

#else // TESTPLAN

#ifdef TESTPLAN
#error "TESTPLAN macro has an illegal value."
#endif

// [OK_TO_UPDATE]
// It is OK to update the below, because if this code is compiled, then we are
// not in the test plan.

//#define SIMULATE
//#define TRACE
//#define REC_TIMINGS
//#define TRACKDEBUG
#define TRACKSECTIONDEBUG

#endif // TESTPLAN

#if defined(SIMULATE) || defined(TRACE) || defined(REC_TIMINGS) \
    || defined(TRACKDEBUG) || defined(TRACKSECTIONDEBUG)
#define DEBUG
#endif

#define MAX_DURATION 65535


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
        char buf[19]; // 16-character strings (then CR+LF then NULL-terminating)
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
    // IMPORTANT
    //   See comment below about BAND_MAX_D => value must be so that
    //   BAND_MAX_D * 2 can be stored in a uint16_t.
    //   That means BAND_MAX_D must be lower than 32768.
#define BAND_MAX_D 30000

struct Band {
    uint16_t inf;
    uint16_t mid;
    uint16_t sup;

    bool got_it;

    bool test_value_init_if_needed(uint16_t d);
    bool test_value(uint16_t d);

    void reset();
    bool init(uint16_t d);
    bool init_sep(uint16_t d);
};

inline void Band::reset() {
    inf = 0;
    sup = 0;
    mid = 0;
}

inline bool Band::init(uint16_t d) {

#ifdef TRACE
    dbgf("B> init: %u", d);
#endif

    if (d >= BAND_MIN_D && d <= BAND_MAX_D) {
        mid = d;
        uint16_t d_divided_by_4 = d >> 2;
        inf = d - d_divided_by_4;
        sup = d + d_divided_by_4;
        got_it = true;
    } else {
        got_it = false;
    }

    return got_it;
}

inline bool Band::init_sep(uint16_t d) {

#ifdef TRACE
    dbgf("BSEP> init: %u", d);
#endif

    uint32_t tmp = d;
    tmp <<= 1;
    if (tmp > 65534)
        tmp = 65534;
    sup = tmp;
    inf = d >> 1;
    inf += (inf >> 2);
    mid = d;

    got_it = true;
    return got_it;
}

inline bool Band::test_value_init_if_needed(uint16_t d) {
    if (!mid) {
        init(d);
    } else {
        got_it = (d >= inf && d <= sup);
#ifdef TRACE
        dbgf("B> cmp %u to [%u, %u]", d, inf, sup);
#endif
    }
#ifdef TRACE
    dbgf("B> res: %d", got_it);
#endif
    return got_it;
}

inline bool Band::test_value(uint16_t d) {
    if (!mid) {
        got_it = false;
#ifdef TRACE
        dbgf("BSEP> cmp %u to uninit'd", d);
#endif
    } else {
        got_it = (d >= inf && d <= sup);
#ifdef TRACE
        dbgf("BSEP> cmp %u to [%u, %u]", d, inf, sup);
#endif
    }
#ifdef TRACE
    dbgf("BSEP> res: %d", got_it);
#endif
    return got_it;
}


// * **** *********************************************************************
// * Rail *********************************************************************
// * **** *********************************************************************

typedef uint32_t recorded_t;
#define FMTRECORDEDT "%08lx"
//typedef uint8_t recorded_t;
//#define FMTRECORDEDT "%02x"

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
        Band b_sep;
        byte last_bit_recorded;
        recorded_t rec;
        byte status;
        byte index;

    public:
        Rail();
        bool rail_eat(uint16_t d);
        void reset();
        void reset_soft();
#ifdef TRACKDEBUG
        void rail_debug() const;
#endif
        byte get_band_count() const;
};

Rail::Rail() {
    reset();
}

inline void Rail::reset() {
    reset_soft();

    b_short.reset();
    b_long.reset();
    b_sep.reset();
}

inline void Rail::reset_soft() {
    status = RAIL_OPEN;
    index = 0;
    rec = 0;
}

inline bool Rail::rail_eat(uint16_t d) {

#ifdef TRACE
    dbgf("R> index = %d, d = %u", index, d);
#endif

    if (status != RAIL_OPEN)
        return false;

    byte count_got_it = 0;
    if (b_short.test_value_init_if_needed(d))
        ++count_got_it;
    if (b_long.test_value_init_if_needed(d))
        ++count_got_it;

    byte band_count = get_band_count();

#ifdef TRACE
    dbgf("R> b_short.got_it = %d, b_long.got_it = %d, "
                  "band_count = %d", b_short.got_it, b_long.got_it,
                  band_count);
    for (int i = 0; i < 2; ++i) {
        dbgf("R>  [%i]: inf = %u, mid = %u, sup = %u", i,
                      (i == 0 ? b_short.inf : b_long.inf),
                      (i == 0 ? b_short.mid : b_long.mid),
                      (i == 0 ? b_short.sup : b_long.sup));
    }
#endif

    if (band_count == 1 && !count_got_it) {
        Band *pband;
            // IMPORTANT
            //   We are using below 'unsigned long' although they are
            //   initialized using uint16_t values.
            //   We need 'unsigned long' because later in the code, we check
            //   whether 'big' is not more than 4 times 'short' (if it is, then
            //   the coding shape is too distorted and we give up).
            //   We do this check by calculating 'small << 2', therefore it
            //   could be that this operation ends up above 16-bit max unsigned
            //   integer value.
        unsigned long small;
        unsigned long big;
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
            assert(false);
        }

#ifdef TRACE
        dbg("R> P0");
#endif

        if ((small << 2) >= big) {
            if (pband->init(d)) {

#ifdef TRACE
                dbg("R> P1");
#endif

                    // As we now know who's who (b_short is b_short and b_long
                    // is b_long, yes), we can adjust boundaries accordingly.

                b_short.inf = (b_short.mid >> 1) - (b_short.mid >> 3);

                b_long.inf = b_short.sup + (b_short.sup >> 3);
                b_long.sup = b_long.mid + (b_long.mid >> 1) + (b_long.mid >> 3);

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
                    rec = ((recorded_t)1 << index) - 1;
                }
            }
        }
    }

    if (!band_count) {
        status = RAIL_ERROR;
        return false;
    }

    if (!count_got_it || (band_count == 2 && count_got_it == 2)) {
        if (!b_sep.mid) {
                // BAND_MAX_D is 30000, and multiplying .mid by 2 will produce a
                // maximum value of 60000, that's OK for an unsigned 16-bit int.
            if (d >= (b_short.mid << 1) && d >= (b_long.mid << 1)) {
                    // We can end up with an overlap between b_sep and b_long.
                    // Not an issue.
                b_sep.init_sep(d);
            }
        }
        status = (b_sep.test_value(d) ? RAIL_STP_RCVD : RAIL_ERROR);
            // FIXME?
            //   A big separator (bigger than b_sep.sup) could be seen as a
            //   TERMINATION separator (or INITIALIZATION SEQUENCE of the next
            //   signal arriving, that's just about the same).
            //   Seen on portail signal.
        return false;
    }

    if (band_count == 2) {
        if (b_short.got_it == b_long.got_it) {
            assert(false);
        }
        last_bit_recorded = (b_short.got_it ? 0 : 1);
        rec = (rec << 1) | last_bit_recorded;
    } else {
        last_bit_recorded = 0;
    }
    if (++index == (sizeof(rec) << 3)) {
        status = RAIL_FULL;
    }

    return (status == RAIL_OPEN);
}

#ifdef TRACKDEBUG
const char* status_names[] = {
    "open",
    "full",
    "stop received",
    "closed",
    "error"
};
void Rail::rail_debug() const {
    dbgf("      \"bits\":%i,\"v\":0x" FMTRECORDEDT
         ",\"railstatus\":\"%s\",\"n\":%d,", index, rec, status_names[status],
         (b_short.mid == b_long.mid ? 1 : 2));
    for (byte i = 0; i < 3; ++i) {
        dbgf("      \"%s\":{\"inf\":%u,\"mid\":%u,\"sup\":%u}%s",
                 (i == 0 ? "b_short" : (i == 1 ? "b_long" : "b_sep")),
                 (i == 0 ? b_short.inf : (i == 1 ? b_long.inf : b_sep.inf)),
                 (i == 0 ? b_short.mid : (i == 1 ? b_long.mid : b_sep.mid)),
                 (i == 0 ? b_short.sup : (i == 1 ? b_long.sup : b_sep.sup)),
                 (i == 2 ? "" : ",")
             );
    }
}
#endif

byte Rail::get_band_count() const {
    return b_short.mid == b_long.mid ? (b_short.mid ? 1 : 0) : 2;
}


// * ***** ********************************************************************
// * Section ******************************************************************
// * ***** ********************************************************************

typedef enum {
    STS_CONTINUED,
    STS_SHORT_SEP,
    STS_LONG_SEP,
    STS_SEP_SEP,
    STS_ERROR
} section_term_status_t;

struct Section {
    recorded_t rec_low;
    byte nb_bits_low;
    recorded_t rec_high;
    byte nb_bits_high;
    section_term_status_t sts;
};


// * ***** ********************************************************************
// * Track ********************************************************************
// * ***** ********************************************************************

#ifdef SIMULATE
SerialLine sl;
char buffer[SerialLine::buf_len];

uint16_t sim_timings[260];

uint16_t sim_timings_count = 0;

unsigned int sim_int_count = 0;
unsigned int sim_int_count_svg;
unsigned int counter;
#endif

#define TRACK_MIN_INITSEQ_DURATION 4000
#define TRACK_MIN_BITS             8

typedef enum {TRK_WAIT, TRK_RECV, TRK_DATA} trk_t;
class Track {
    private:
        volatile trk_t trk;
        Rail r_low;
        Rail r_high;

        byte prev_r;

        uint16_t low_short;
        uint16_t low_long;
        uint16_t high_short;
        uint16_t high_long;
        uint16_t sep;

        byte nb_sections;
        Section sections[12];

    public:
        Track();

        void treset_hard();
        void track_eat(byte r, uint16_t d);
#ifdef TRACKDEBUG
        void track_debug() const;
#endif
#ifdef TRACKSECTIONDEBUG
        void track_debug_sections() const;
#endif

        trk_t get_trk() const { return trk; }
        bool rails_have_2_bands() const;
};

Track::Track() {
    treset_hard();
}

inline void Track::treset_hard() {
    trk = TRK_WAIT;
    nb_sections = 0;
}

inline void Track::track_eat(byte r, uint16_t d) {

#ifdef TRACE
    dbgf("T> trk = %d, r = %d, d = %u", trk, r, d);
#endif

    if (trk == TRK_WAIT) {
        if (r == 1 && d >= TRACK_MIN_INITSEQ_DURATION) {
            r_low.reset();
            r_high.reset();
            prev_r = r;
            trk = TRK_RECV;
        }
        return;
    } else if (trk != TRK_RECV) {
        return;
    }

        // We missed an interrupt apparently (two calls with same r), so we
        // should discard the ongoing signal.
    if (r == prev_r)
        d = 0;
    prev_r = r;

    Rail *prail = (r == 0 ? &r_low : &r_high);
    if (prail->status != RAIL_OPEN)
        return;

    bool b = prail->rail_eat(d);
    if (r == 1 && (!b || r_low.status != RAIL_OPEN)) {

#ifdef TRACE
        dbgf("T> b = %d", b);
#endif

//        if (r_low.index >= 1 || r_high.index >= 1) {
//            byte li = r_low.index;
//            byte hi = r_high.index;
//            Rail *prail = nullptr;
//            if (r_low.index > r_high.index) {
//                prail = &r_low;
//            } else if (r_high.index > r_low.index) {
//                prail = &r_high;
//            }
//            if (prail) {
//                prail->rec >>= 1;
//                prail->index--;
//            }
//            if (r_high.index != r_low.index) {
//                dbgf("(prev) li = %i, hi = %i", li, hi);
//                dbgf("r_low.index = %i, r_high.index = %i",
//                     r_low.index, r_high.index);
//            }
//            assert(r_high.index == r_low.index);
//        }

        if (r_low.status == RAIL_OPEN)
            r_low.status = RAIL_CLOSED;
        if (r_high.status == RAIL_OPEN)
            r_high.status = RAIL_CLOSED;

        section_term_status_t sts;
        if (r_low.status == RAIL_FULL && r_high.status == RAIL_FULL) {
            sts = STS_CONTINUED;
        } else if (r_high.status == RAIL_STP_RCVD) {
            if (r_low.status == RAIL_CLOSED || r_low.status == RAIL_FULL) {
                sts = (r_low.last_bit_recorded ? STS_LONG_SEP : STS_SHORT_SEP);
            } else if (r_low.status == RAIL_STP_RCVD) {
                sts = STS_SEP_SEP;
            } else {
                sts = STS_ERROR;
            }
        } else {
            sts = STS_ERROR;
        }


/*
Tests implemented below reproduce the following decision table

Notations:
  "pr=cont":  the previous track terminated as STS_CONTINUED
  "pr!=cont": the previous track didn't terminate as STS_CONTINUED
  "nbsec": nb_sections
  "cur":   how did current track end? ->
    "sep":   it ended with a separator
    "err":   it ended with an error
    "full":  it didn't end but record is full
   CUR?:  shall we record the current track?
   NEXT?: what to do next? (reset track, start new section)

  +---------------+-------- +----------+-------++-------+--------------+
  |nb_bits        | nbsec   | prev     | cur   ||  CUR? | NEXT?        |
  +---------------+-------- +----------+-------++-------+--------------+
  |bits<min_bits  | !nbsec  | n/a      | sep   ||  DISC | RESET        |
  |               |         |          | err   ||  DISC | RESET        |
  |               |         |          | full  ||  n/a  | n/a          |
  |               | nbsec>0 | pr=cont  | sep   ||  REC  | NEWSEC       |
  |               |         |          | err   ||  DISC | DATA         |
  |               |         |          | full  ||  n/a  | n/a          |
  |               | nbsec>0 | pr!=cont | sep   ||  DISC | DATA         |
  |               |         |          | err   ||  DISC | DATA         |
  |               |         |          | full  ||  n/a  | n/a          |
  |bits>=min_bits | !nbsec  | n/a      | sep   ||  REC  | NEWSEC       |
  |               |         |          | err   ||  DISC | RESET        |
  |               |         |          | ful   ||  REC  | NEWSEC(CONT) |
  |               | nbsec>0 | pr=cont  | sep   ||  REC  | NEWSEC       |
  |               |         |          | err   ||  DISC | DATA         |
  |               |         |          | ful   ||  REC  | NEWSEC(CONT) |
  |               | nbsec>0 | pr!=cont | sep   ||  REC  | NEWSEC       |
  |               |         |          | err   ||  DISC | DATA         |
  |               |         |          | ful   ||  REC  | NEWSEC(CONT) |
  +---------------+-------- +----------+-------++-------+--------------+
*/

        bool record_current_section;

#ifdef TRACKDEBUG
        bool do_track_debug = false;
        (void)do_track_debug;
#endif

        if (r_low.index < TRACK_MIN_BITS || r_high.index < TRACK_MIN_BITS) {
            record_current_section =
                (sts != STS_ERROR
                 && nb_sections
                 && sections[nb_sections - 1].sts == STS_CONTINUED);

#ifdef TRACKDEBUG
            do_track_debug = record_current_section;
#endif

        } else {
            record_current_section = (sts != STS_ERROR);

#ifdef TRACKDEBUG
            do_track_debug = true;
#endif

        }

#if defined(SIMULATE) && defined(TRACKDEBUG)
#ifdef TRACE
        dbgf("T> reccursec=%i, sts=%i", record_current_section, sts);
#endif
        if (do_track_debug) {
            dbgf("%s  {", counter >= 2 ? ",\n" : "");
            dbgf("    \"N\":%d,\"start\":%u,\"end\":%u,",
                sim_timings_count, sim_int_count_svg, sim_int_count - 1);
            track_debug();
            dbg("  }");
        }
#endif

        if (record_current_section) {
            Section *psec = &sections[nb_sections++];
            psec->sts = sts;
            psec->rec_low = r_low.rec;
            psec->nb_bits_low = r_low.index;
            psec->rec_high = r_high.rec;
            psec->nb_bits_high = r_high.index;
            trk = ((nb_sections == sizeof(sections) / sizeof(*sections)) ?
                   TRK_DATA : TRK_RECV);

            if (trk == TRK_RECV) {
                r_low.reset_soft();
                r_high.reset_soft();
            }
        } else {
            if (nb_sections) {
                trk = TRK_DATA;
            } else {
                treset_hard();
                    // WARNING
                    //   Re-entrant call... not ideal.
                track_eat(r, d);
            }
        }

    }
}

#ifdef TRACKDEBUG
const char* trk_names[] = {
    "TRK_WAIT",
    "TRK_RECV",
    "TRK_DATA"
};
void Track::track_debug() const {
    recorded_t xorval = r_low.rec ^ r_high.rec;
    dbgf("    \"trk\":%s,\"xorval\":0x" FMTRECORDEDT ",",
         trk_names[trk], xorval);
    if (trk != TRK_WAIT) {
        for (byte i = 0; i < 2; ++i) {
            dbgf("    \"%s\":{", (i == 0 ? "r_low" : "r_high"));
            (i == 0 ? &r_low : &r_high)->rail_debug();
            dbgf("    }%s", i == 1 ? "" : ",");
        }
    }

}
#endif

#ifdef TRACKSECTIONDEBUG
void Track::track_debug_sections() const {
#ifdef TRACE
    dbgf("S> nbsec=%i", nb_sections);
#endif
    for (byte i = 0; i < nb_sections; ++i) {
        const Section *psec = &sections[i];
        dbgf("[%02d] %s", i,
                (psec->sts == STS_CONTINUED ? "CONT" :
                    (psec->sts == STS_SHORT_SEP ? "SSEP" :
                        (psec->sts == STS_LONG_SEP ? "LSEP" :
                            (psec->sts == STS_SEP_SEP ? "2SEP" :
                                (psec->sts == STS_ERROR ? "ERROR" :
                                    "(UNK)"))))));
        dbgf("     low:  n = %d, v = " FMTRECORDEDT "",
                      psec->nb_bits_low, psec->rec_low);
        dbgf("     high: n = %d, v = " FMTRECORDEDT "",
                      psec->nb_bits_high, psec->rec_high);
    }
}
#endif

    // Returns true if both rails have 2 bands, meaning, the signal received
    // showed both short and long signal durations.
    // If one rail has one band, that means we don't know if signal on this band
    // is 'short' or 'long' (= the duration was constant).
    // This can be important to know, because if the function returns false, the
    // caller should NOT consider the signal received contains information (this
    // most likely corresponds to a sync prefix before real information
    // encoding).
bool Track::rails_have_2_bands() const {
    return r_low.get_band_count() == 2 && r_high.get_band_count() == 2;
}


// * ************* ************************************************************
// * Interruptions ************************************************************
// * ************* ************************************************************

#ifdef REC_TIMINGS
uint16_t ih_dbg_timings[80];
uint16_t ih_exec[80];
unsigned int ih_dbg_timing_pos = 0;
#endif

typedef struct {
    byte r;
    uint16_t d;
} IH_timing_t;

    // IMPORTANT
    //   IH_MASK must be equal to the size of IH_timings - 1.
    //   The size of IH_timings must be a power of 2.
    //   Thus, IH_MASK allows to quickly calculate modulo, while walking through
    //   IH_timings.
#define IH_SIZE 32
#define IH_MASK (IH_SIZE - 1)
IH_timing_t IH_timings[IH_SIZE];
volatile unsigned char IH_write_head = 0;
volatile unsigned char IH_read_head = 0;
unsigned char IH_max_pending_timings = 0;

    // This one fills the array IH_timings

// CRITICAL SECTION - NO INTERRUPTS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void handle_interrupt() {
    static unsigned long last_t = 0;
    const unsigned long t = micros();

#ifdef SIMULATE
    unsigned long d;
    byte r = sim_int_count % 2;
    if (sim_int_count >= sim_timings_count) {
        d = 100;
    } else {
        d = sim_timings[sim_int_count++];
    }
    (void)last_t;
    (void)t;
#else
    unsigned long d = t - last_t;
    last_t = t;
    byte r = (digitalRead(PIN_RFINPUT) == HIGH ? 1 : 0);
#endif

    if (d > MAX_DURATION)
        d = MAX_DURATION;

    unsigned char next_IH_write_head = (IH_write_head + 1) & IH_MASK;
    if (next_IH_write_head != IH_read_head) {
        IH_write_head = next_IH_write_head;
        IH_timings[IH_write_head].r = r;
        IH_timings[IH_write_head].d = d;
    }
}
// END OF CRITICAL SECTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // This one consumes what got filled in the array IH_timings by
    // handle_interrupt().
    // Returns true if there was data in IH_timings, false otherwise.

bool process_interrupt_timing(Track *ptrack) {

        // FIXME
        //   I don't think the '+ IH_SIZE' term is necessary, as we are working
        //   in unsigned arithmetic... But I prefer to leave it for now.
    unsigned char IH_pending_timings =
        ((IH_write_head + IH_SIZE) - IH_read_head) & IH_MASK;
    if (IH_pending_timings > IH_max_pending_timings)
        IH_max_pending_timings = IH_pending_timings;

    bool ret;

// CRITICAL SECTION - NO INTERRUPTS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    cli();

    if (IH_read_head != IH_write_head) {
        IH_timing_t timing = IH_timings[IH_read_head];
        IH_read_head = (IH_read_head + 1) & IH_MASK;

        sei();
// END OF CRITICAL SECTION (CASE 1) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//        check_veryrecent(ptrack, timing.r, timing.d);

        ptrack->track_eat(timing.r, timing.d);
        ret = true;

    } else {

        sei();
// END OF CRITICAL SECTION (CASE 2) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        ret = false;
    }

    return ret;
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
            dbg("FATAL: each line must have a ',' character!");
            assert(false);
        }

        *p = '\0';
        unsigned int l = atoi(buffer);
        unsigned int h = atoi(p + 1);

        if (sim_timings_count >=
                sizeof(sim_timings) / sizeof(*sim_timings) - 1) {
            dbg("FATAL: timings buffer full!");
            assert(false);
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

Track track;

void loop() {

//    initialize_veryrecent();

#ifdef SIMULATE

    if (sim_int_count >= sim_timings_count)
        read_simulated_timings_from_usb();

    if (!counter) {
        delay(100);
        dbg("----- BEGIN TEST -----");
#ifdef TRACKDEBUG
        dbg("[");
#endif
    }

    ++counter;

    track.treset_hard();
    sim_int_count_svg = sim_int_count;
    while (track.get_trk() != TRK_DATA && sim_int_count < sim_timings_count) {
        for (int i = 0; i < 6; ++i)
            handle_interrupt();
        while (track.get_trk() != TRK_DATA && process_interrupt_timing(&track)) {
        }
    }
#ifdef TRACKSECTIONDEBUG
    dbgf("IH_max_pending_timings = %d", IH_max_pending_timings);
#endif

#ifdef TRACKDEBUG
    if (sim_int_count >= sim_timings_count) {
        dbg("]");
    }
#endif

#ifdef TRACKSECTIONDEBUG
    track.track_debug_sections();
#endif

    if (sim_int_count >= sim_timings_count) {
        dbg("----- END TEST -----");
    }

#else // SIMULATE

    dbg("Waiting for signal");
    track.treset_hard();

    attachInterrupt(INT_RFINPUT, &handle_interrupt, CHANGE);
    while (track.get_trk() != TRK_DATA) {
        while (track.get_trk() != TRK_DATA && process_interrupt_timing(&track)) {
        }

#ifdef REC_TIMINGS
        if (track.get_trk() == TRK_WAIT)
            ih_dbg_timing_pos = 0;
#endif

        sleep_mode();
    }
    detachInterrupt(INT_RFINPUT);

#ifdef TRACKSECTIONDEBUG
    dbgf("IH_max_pending_timings = %d", IH_max_pending_timings);
    track.track_debug_sections();
#endif

#endif // SIMULATE

#ifdef REC_TIMINGS
    for (unsigned int i = 0; i < ih_dbg_timing_pos - 1; i += 2) {
        dbgf("%4u, %4u  |  %5u, %5u", ih_dbg_timings[i], ih_dbg_timings[i + 1],
             ih_exec[i], ih_exec[i + 1]);
    }
#endif

}

// vim: ts=4:sw=4:tw=80:et
