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

#define DBG_SIMULATE
#define DBG_TRACK

#elif TESTPLAN == 2 // TESTPLAN

#define DBG_SIMULATE
#define DBG_RAWCODE

#else // TESTPLAN

#ifdef TESTPLAN
#error "TESTPLAN macro has an illegal value."
#endif


// ****************************************************************************
// ****************************************************************************

// It is OK to update the below, because if this code is compiled, then we are
// not in the test plan.

#define DBG_SIMULATE
//#define DBG_TRACE
//#define DBG_TIMINGS
//#define DBG_TRACK
//#define DBG_RAWCODE
#define DBG_DECODER
#define DBG_SMALL_RECORDED_T

#endif // TESTPLAN ************************************************************
// ****************************************************************************


#if defined(DBG_SIMULATE) || defined(DBG_TRACE) || defined(DBG_TIMINGS) \
    || defined(DBG_TRACK) || defined(DBG_RAWCODE) \
    || defined(DBG_DECODER)
#define DEBUG
#endif

#define MAX_DURATION     65535
#define MAX_SEP_DURATION 65534

#define MAX_SECTIONS 12


// * ********************** ***************************************************
// * Read input from serial ***************************************************
// * ********************** ***************************************************

#ifdef DBG_SIMULATE

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

#endif // DBG_SIMULATE


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

    void breset();
    bool init(uint16_t d);
    bool init_sep(uint16_t d);
};

inline void Band::breset() {
    inf = 0;
    sup = 0;
    mid = 0;
}

inline bool Band::init(uint16_t d) {

#ifdef DBG_TRACE
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

#ifdef DBG_TRACE
    dbgf("BSEP> init: %u", d);
#endif

    uint32_t tmp = d;
    tmp <<= 1;
    if (tmp > MAX_SEP_DURATION)
        tmp = MAX_SEP_DURATION;
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
#ifdef DBG_TRACE
        dbgf("B> cmp %u to [%u, %u]", d, inf, sup);
#endif
    }
#ifdef DBG_TRACE
    dbgf("B> res: %d", got_it);
#endif
    return got_it;
}

inline bool Band::test_value(uint16_t d) {
    if (!mid) {
        got_it = false;
#ifdef DBG_TRACE
        dbgf("BSEP> cmp %u to uninit'd", d);
#endif
    } else {
        got_it = (d >= inf && d <= sup);
#ifdef DBG_TRACE
        dbgf("BSEP> cmp %u to [%u, %u]", d, inf, sup);
#endif
    }
#ifdef DBG_TRACE
    dbgf("BSEP> res: %d", got_it);
#endif
    return got_it;
}


// * **** *********************************************************************
// * Rail *********************************************************************
// * **** *********************************************************************

#ifdef DBG_SMALL_RECORDED_T

typedef uint8_t recorded_t;
#define FMTRECORDEDT "%02x"

#else // DBG_SMALL_RECORDED_T

typedef uint32_t recorded_t;
#define FMTRECORDEDT "%08lx"

#endif // DBG_SMALL_RECORDED_T

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
        void rreset();
        void rreset_soft();
#ifdef DBG_TRACK
        void rail_debug() const;
#endif
        byte get_band_count() const;
};

Rail::Rail() {
    rreset();
}

inline void Rail::rreset() {
    rreset_soft();

    b_short.breset();
    b_long.breset();
    b_sep.breset();
}

inline void Rail::rreset_soft() {
    status = RAIL_OPEN;
    index = 0;
    rec = 0;
}

inline bool Rail::rail_eat(uint16_t d) {

#ifdef DBG_TRACE
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

#ifdef DBG_TRACE
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

#ifdef DBG_TRACE
        dbg("R> P0");
#endif

        if ((small << 2) >= big) {
            if (pband->init(d)) {

#ifdef DBG_TRACE
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
#ifdef DBG_TRACE
                dbg("R> init b_sep");
#endif
                    // We can end up with an overlap between b_sep and b_long.
                    // Not an issue.
                b_sep.init_sep(d);
            } else {
#ifdef DBG_TRACE
                dbg("R> no init of b_sep (d too small)");
#endif
            }
        }
        status = (b_sep.test_value(d) ? RAIL_STP_RCVD : RAIL_ERROR);

#ifdef DBG_TRACE
        dbgf("R> rail terminated, status = %d", status);
#endif

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

#ifdef DBG_TRACK
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
//    uint16_t initseq;
    recorded_t low_rec;
    unsigned char low_bits   :6;
    unsigned char low_bands  :2;
    recorded_t high_rec;
    unsigned char high_bits  :6;
    unsigned char high_bands :2;
    uint16_t sep;
    section_term_status_t sts;
};

struct RawCode {
    uint16_t initseq;
    byte nb_sections;
    Section sections[MAX_SECTIONS];

    void debug_rawcode() const;
};

#ifdef DBG_RAWCODE
const char *sts_names[] = {
    "CONT",
    "SSEP",
    "LSEP",
    "2SEP",
    "ERR"
};
void RawCode::debug_rawcode() const {
    dbgf("> nb_sections = %d, initseq = %u",
            nb_sections, initseq);
    for (byte i = 0; i < nb_sections; ++i) {
        const Section *psec = &sections[i];
        dbgf("  %02d  %s", i, sts_names[psec->sts]);
        dbgf("      sep = %u", psec->sep);
        dbgf("      low:  [%d] n = %2d, v = 0x" FMTRECORDEDT "",
                      psec->low_bands, psec->low_bits, psec->low_rec);
        dbgf("      high: [%d] n = %2d, v = 0x" FMTRECORDEDT "",
                      psec->high_bands, psec->high_bits, psec->high_rec);
    }
}
#endif


// * ***** ********************************************************************
// * Track ********************************************************************
// * ***** ********************************************************************

#ifdef DBG_SIMULATE
SerialLine sl;
char buffer[SerialLine::buf_len];

#ifdef DBG_TIMINGS
uint16_t sim_timings[150];
#else
uint16_t sim_timings[260];
//uint16_t sim_timings[140];
#endif

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

        RawCode rawcode;

    public:
        Track();

        void treset();
        void track_eat(byte r, uint16_t d);
#ifdef DBG_TRACK
        void track_debug() const;
#endif

        trk_t get_trk() const { return trk; }
        const RawCode *get_rawcode() const { return &rawcode; }
};

Track::Track() {
    treset();
}

inline void Track::treset() {
    trk = TRK_WAIT;
    rawcode.nb_sections = 0;
    rawcode.initseq = 0;
}

inline void Track::track_eat(byte r, uint16_t d) {

#ifdef DBG_TRACE
    dbgf("T> trk = %d, r = %d, d = %u", trk, r, d);
#endif

    if (trk == TRK_WAIT) {
        if (r == 1 && d >= TRACK_MIN_INITSEQ_DURATION) {
            r_low.rreset();
            r_high.rreset();
            prev_r = r;
            rawcode.initseq = d;
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

#ifdef DBG_TRACE
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

#ifdef DBG_TRACK
        bool do_track_debug = false;
        (void)do_track_debug;
#endif

        if (r_low.index < TRACK_MIN_BITS || r_high.index < TRACK_MIN_BITS) {
            record_current_section =
                (sts != STS_ERROR
                 && rawcode.nb_sections
                 && rawcode.sections[rawcode.nb_sections - 1].sts
                    == STS_CONTINUED);

#ifdef DBG_TRACK
            do_track_debug = record_current_section;
#endif

        } else {
            record_current_section = (sts != STS_ERROR);

#ifdef DBG_TRACK
            do_track_debug = true;
#endif

        }

#ifdef DBG_TRACE
        dbgf("T> reccursec=%i, sts=%i", record_current_section, sts);
#endif
#if defined(DBG_SIMULATE) && defined(DBG_TRACK)
        if (do_track_debug) {
            dbgf("%s  {", counter >= 2 ? ",\n" : "");
            dbgf("    \"N\":%d,\"start\":%u,\"end\":%u,",
                sim_timings_count, sim_int_count_svg, sim_int_count - 1);
            track_debug();
            dbg("  }");
        }
#endif

        if (record_current_section) {
#ifdef DBG_TRACE
            dbg("T> recording current section");
#endif
            Section *psec = &rawcode.sections[rawcode.nb_sections++];
            psec->sts = sts;
            psec->sep = (sts == STS_SHORT_SEP
                        || sts == STS_LONG_SEP
                        || sts == STS_SEP_SEP ? d : 0);
            psec->low_rec = r_low.rec;
            psec->low_bits = r_low.index;
            psec->low_bands = r_low.get_band_count();
            psec->high_rec = r_high.rec;
            psec->high_bits = r_high.index;
            psec->high_bands = r_high.get_band_count();
            trk = ((rawcode.nb_sections == MAX_SECTIONS) ? TRK_DATA : TRK_RECV);

            if (trk == TRK_RECV) {
#ifdef DBG_TRACE
                dbg("T> keep receiving (soft reset)");
#endif
                r_low.rreset_soft();
                r_high.rreset_soft();
            } else {
#ifdef DBG_TRACE
                dbg("T> stop receiving (data)");
#endif
            }
        } else {
            if (rawcode.nb_sections) {
                trk = TRK_DATA;
            } else {
                treset();
                    // WARNING
                    //   Re-entrant call... not ideal.
                track_eat(r, d);
            }
        }

    }
}

#ifdef DBG_TRACK
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


// * ******* ******************************************************************
// * Decoder ******************************************************************
// * ******* ******************************************************************

    // IMPORTANT
    //   VALUES ARE NOT ARBITRARY.
    //   CONVENTION_0 must be 0 and CONVENTION_1 must be 1.
    //   This is due to the decoding that uses a bit value ultimately coming
    //   from CONVENTION_0 or CONVENTION_1.
#define CONVENTION_0 0
#define CONVENTION_1 1

typedef enum {
    SD_SHORT,
    SD_LONG,
    SD_OTHER
} sgn_duration_t;

#define DEC_ID_RAW_INCONSISTENT 0
#define DEC_ID_RAW_SYNC         1
#define DEC_ID_UNKNOWN_CODING   2
#define DEC_ID_START            3 // Start of enumeration of real decoders
#define DEC_ID_TRIBIT           3
#define DEC_ID_TRIBIT_INV       4
#define DEC_ID_MANCHESTER       5
#define DEC_ID_END              5 // End of enumeration of real decoders

#ifdef DBG_DECODER
const char *dec_id_names[] = {
    "INCONSISTENT",
    "SYNC",
    "UNKNOWN",
    "TRI",
    "TRIINV",
    "MAN"
};
#endif

#define DECODER_NB_BYTES 16
//#define DECODER_NB_BYTES 6

class Decoder {
    protected:
        uint8_t data[DECODER_NB_BYTES];
        byte convention;

        byte nb_bits;
        byte nb_errors;
        bool last_is_error;

        Decoder *next;

        void add_decoded_bit(byte valbit);

    public:
        Decoder(byte arg_convention);
        virtual ~Decoder();
        virtual byte get_id() const = 0;

        static Decoder *build_decoder(byte id);

        virtual void add_sgn_lo_hi(sgn_duration_t low, sgn_duration_t high) = 0;
        virtual byte get_nb_errors() const;
        virtual byte get_nb_bits() const;

        virtual void attach_next(Decoder *pdec);

#ifdef DBG_DECODER
        void output_data() const;
#endif
};

Decoder::Decoder(byte arg_convention):
        convention(arg_convention),
        nb_bits(0),
        nb_errors(0),
        last_is_error(false),
        next(nullptr)
{
    for (byte i = 0; i < DECODER_NB_BYTES; ++i)
        data[i] = 0;
}

Decoder::~Decoder() {
    if (next)
        delete next;
}

void Decoder::attach_next(Decoder *pdec) {
    assert(!next);
    next = pdec;
}

void Decoder::add_decoded_bit(byte valbit) {
    if (++nb_bits < (DECODER_NB_BYTES << 3)) {
        for (int i = DECODER_NB_BYTES - 1; i >= 0; --i) {

            byte b;
            if (i > 0) {
                b = !!(data[i - 1] & 0x80);
            } else {
                    // Defensive programming
                    // Normally valbit is 0 or 1, but I normalize it, just in
                    // case...
                b = !!valbit;
            }

            data[i] = (data[i] << 1) | b;

        }
    }
}

byte Decoder::get_nb_errors() const { return nb_errors; }

byte Decoder::get_nb_bits() const { return nb_bits; }

#ifdef DBG_DECODER
void Decoder::output_data() const {
    if (nb_bits) {
        byte nb_bytes = (nb_bits + 7) >> 3;
        char *buf = new char[nb_bytes * 3];
        char tmp[3];
        int j = 0;
        for (int i = nb_bytes - 1; i >= 0 ; --i) {
            snprintf(tmp, sizeof(tmp), "%02x", data[i]);
            buf[j] = tmp[0];
            buf[j + 1] = tmp[1];
            buf[j + 2] = (i > 0 ? ' ' : '\0');
            j += 3;
        }
        assert(j <= nb_bytes * 3);
        dbgf("Received %d bits: %s", get_nb_bits(), buf);
        delete buf;
    } else {
        dbgf("No data received, type = %s", dec_id_names[get_id()]);
    }
}
#endif


// * ********** ***************************************************************
// * DecoderRaw ***************************************************************
// * ********** ***************************************************************

    // This Decoder is 'fake', it won't decode anything, it'll rather record raw
    // data when signal is no good.
    // But it is convenient to give it a Decoder interface.
class DecoderRaw: public Decoder {
    private:
        byte dec_id_type;

    public:
        DecoderRaw(byte arg_dec_id_type):
                Decoder(CONVENTION_0),
                dec_id_type(arg_dec_id_type) {
        }
        ~DecoderRaw() { }

        virtual byte get_id() const override { return dec_id_type; }

        virtual void add_sgn_lo_hi(sgn_duration_t lo, sgn_duration_t hi)
            override { }
};


// * ************* ************************************************************
// * DecoderTriBit ************************************************************
// * ************* ************************************************************

class DecoderTriBit: public Decoder {
    private:
        sgn_duration_t unused_final_low;

    public:
        DecoderTriBit(byte arg_convention = CONVENTION_0)
                :Decoder(arg_convention) {
        }
        ~DecoderTriBit() { }

        virtual byte get_id() const override { return DEC_ID_TRIBIT; }
        virtual void add_sgn_lo_hi(sgn_duration_t low, sgn_duration_t high)
            override;
};

void DecoderTriBit::add_sgn_lo_hi(sgn_duration_t lo, sgn_duration_t hi) {
    last_is_error = false;

        // The below case corresponds to the reception of (short, sep) or (high,
        // sep), meaning, the low signal is meaningful whereas the high signal
        // has no encoding meaning.
    if (hi == SD_OTHER) {
        unused_final_low = lo;
        return;
    }

    byte valbit;
    if (lo == SD_SHORT && hi == SD_LONG)
        valbit = convention;
    else if (lo == SD_LONG && hi == SD_SHORT)
        valbit = !convention;
    else {
        ++nb_errors;
        last_is_error = true;
    }

    if (!last_is_error)
        add_decoded_bit(valbit);
}


// * **************** *********************************************************
// * DecoderTriBitInv *********************************************************
// * **************** *********************************************************

class DecoderTriBitInv: public Decoder {
    private:
        bool first_call_to_add_sgn_lo_hi;
        sgn_duration_t unused_initial_low;
        sgn_duration_t last_hi;

    public:
        DecoderTriBitInv(byte arg_convention = CONVENTION_0)
                :Decoder(arg_convention),
                first_call_to_add_sgn_lo_hi(true) {
        }
        ~DecoderTriBitInv() { }

        virtual byte get_id() const override { return DEC_ID_TRIBIT_INV; }
        virtual void add_sgn_lo_hi(sgn_duration_t low, sgn_duration_t high)
            override;
};

void DecoderTriBitInv::add_sgn_lo_hi(sgn_duration_t lo, sgn_duration_t hi) {

    last_is_error = false;

    if (first_call_to_add_sgn_lo_hi) {
        first_call_to_add_sgn_lo_hi = false;
        unused_initial_low = lo;
        last_hi = hi;
        return;
    }

    byte valbit;
    if (lo == SD_SHORT && last_hi == SD_LONG)
        valbit = !convention;
    else if (lo == SD_LONG && last_hi == SD_SHORT)
        valbit = convention;
    else {
        ++nb_errors;
        last_is_error = true;
    }

    if (!last_is_error)
        add_decoded_bit(valbit);

    last_hi = hi;
}


// * ***************** ********************************************************
// * DecoderManchester ********************************************************
// * ***************** ********************************************************

class DecoderManchester: public Decoder {
    private:
        byte buf[3];
        byte buf_pos;

        void add_buf(byte r);
        void consume_buf();

    public:
        DecoderManchester(byte arg_convention = CONVENTION_0);
        ~DecoderManchester() { }

        virtual byte get_id() const override { return DEC_ID_MANCHESTER; }
        virtual void add_sgn_lo_hi(sgn_duration_t low, sgn_duration_t high)
            override;
};

DecoderManchester::DecoderManchester(byte arg_convention)
        :Decoder(arg_convention),buf_pos(0) {
    for (byte i = 0; i < sizeof(buf) / sizeof(*buf); ++i) {
        buf[i] = 0;
    }
}

void DecoderManchester::add_buf(byte r) {
    assert(buf_pos < sizeof(buf)  /sizeof(*buf));
    buf[buf_pos++] = r;
}

void DecoderManchester::consume_buf() {
    last_is_error = false;
    if (buf_pos >= 2) {
        if (buf[0] == 0 && buf[1] == 1) {
            add_decoded_bit(convention);
        } else if (buf[0] == 1 && buf[1] == 0) {
            add_decoded_bit(!convention);
        } else {
                // FIXME: créer register_error pour gérer ça de manière
                // cohérente entre les différents descendants de Decoder.
            ++nb_errors;
            last_is_error = true;
        }
            // Not always necessary, but harmless if done while not necessary
        buf[0] = buf[2];
        buf_pos -= 2;
    }
}

void DecoderManchester::add_sgn_lo_hi(sgn_duration_t lo, sgn_duration_t hi) {
    if (lo == SD_OTHER) {
        ++nb_errors;
        last_is_error = true;
        return;
    }

    for (byte i = 0; i < 2; ++i) {
        sgn_duration_t sgn = (i == 0 ? lo : hi);
        add_buf(i);
        if (sgn == SD_LONG)
            add_buf(i);
        consume_buf();
    }
}

Decoder* Decoder::build_decoder(byte id) {
    switch (id) {
        case DEC_ID_TRIBIT:     return new DecoderTriBit();
        case DEC_ID_TRIBIT_INV: return new DecoderTriBitInv();
        case DEC_ID_MANCHESTER: return new DecoderManchester();
        default:
            assert(false);
    }
    return nullptr; // Never executed
}


// * ************* ************************************************************
// * Interruptions ************************************************************
// * ************* ************************************************************

#ifdef DBG_TIMINGS
uint16_t ih_dbg_timings[60];
uint16_t ih_dbg_exec[60];
unsigned int ih_dbg_pos = 0;
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
#define IH_SIZE 16 // FIXME (used to be 32)
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

#ifdef DBG_SIMULATE
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
        // No ideal solution here: we reached the buffer size, so either we
        // write nothing, or, we loose the oldest entry that was the next one to
        // read.
        // Solution here: we loose oldest entry in buffer and do the write.
    if (next_IH_write_head == IH_read_head) {
        IH_read_head = (IH_read_head + 1) & IH_MASK;
    }
    IH_write_head = next_IH_write_head;
    IH_timings[IH_write_head].r = r;
    IH_timings[IH_write_head].d = d;
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

#ifdef DBG_TIMINGS
        unsigned long t0 = micros();
#endif
        ptrack->track_eat(timing.r, timing.d);
#ifdef DBG_TIMINGS
        unsigned long d = micros() - t0;
        if (d > MAX_DURATION)
            d = MAX_DURATION;
        ih_dbg_exec[ih_dbg_pos] = d;
        if (ptrack->get_trk() == TRK_WAIT)
            ih_dbg_pos = 0;
        else {
            if (ih_dbg_pos < sizeof(ih_dbg_timings) / sizeof(*ih_dbg_timings))
                ih_dbg_timings[ih_dbg_pos++] = timing.d;
        }
#endif

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

#ifdef DBG_SIMULATE
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

Decoder* decode_rawcode(const RawCode *raw) {
    Decoder *pdec_head = nullptr;
    Decoder *pdec_tail = nullptr;
    Decoder *pdec = nullptr;

    for (byte i = 0; i < raw->nb_sections; ++i) {

        const Section *psec = &raw->sections[i];
        bool forward_pdec;

        if (abs(psec->low_bits - psec->high_bits) >= 2) {
                // Defensive programming (should never happen).
                // It should never happen because we continually check that 'r'
                // submitted values (as argument to track_eat()) switch between
                // 0 and 1, so that low and high rails are populated equally.
            assert(false);
        } else if (psec->low_bands == 1 && psec->high_bands == 1) {
            assert(!pdec);
            pdec = new DecoderRaw(DEC_ID_RAW_SYNC);
            forward_pdec = true;

        } else if (psec->low_bands == 1 || psec->high_bands == 1) {
            assert(!pdec);
            pdec = new DecoderRaw(DEC_ID_RAW_INCONSISTENT);
            forward_pdec = true;

        } else {
            byte enum_decoders = DEC_ID_START;
            bool is_continuation_of_prev_section = pdec;
            do {
                if (!pdec)
                    pdec = Decoder::build_decoder(enum_decoders);

                byte pos_low = psec->low_bits;
                byte pos_high = psec->high_bits;

                while (pos_low >= 1 || pos_high >= 1) {
                    sgn_duration_t sd_low = SD_OTHER;
                    sgn_duration_t sd_high = SD_OTHER;
                    if (pos_low >= 1) {
                        --pos_low;
                        sd_low = ((((recorded_t)1 << pos_low) & psec->low_rec) ?
                                    SD_LONG : SD_SHORT);
                    }
                    if (pos_high >= 1) {
                        --pos_high;
                        sd_high =
                            ((((recorded_t)1 << pos_high) & psec->high_rec) ?
                            SD_LONG : SD_SHORT);
                    }
                    pdec->add_sgn_lo_hi(sd_low, sd_high);
                }
                if (!is_continuation_of_prev_section && pdec->get_nb_errors()) {
                    delete pdec;
                    pdec = nullptr;
                }
            } while (!pdec && ++enum_decoders <= DEC_ID_END);

            if (!pdec)
                pdec = new DecoderRaw(DEC_ID_UNKNOWN_CODING);

            forward_pdec = (psec->sts != STS_CONTINUED);
        }

#ifdef DBG_DECODER
//        dbgf("code = %s", code_names[code]);
#endif
        if (forward_pdec) {
            if (!pdec_head) {
                assert(!pdec_tail);
                pdec_head = pdec;
                pdec_tail = pdec;
            } else {
                assert(pdec_tail);
                pdec_tail->attach_next(pdec);
                pdec_tail = pdec;
            }
            pdec = nullptr;
        }
    }
    return pdec_head;
}

void loop() {

//    initialize_veryrecent();

#ifdef DBG_SIMULATE

    if (sim_int_count >= sim_timings_count)
        read_simulated_timings_from_usb();

    if (!counter) {
        delay(100);
        dbg("----- BEGIN TEST -----");
#ifdef DBG_TRACK
        dbg("[");
#endif
    }

    ++counter;

    track.treset();
    sim_int_count_svg = sim_int_count;
    while (track.get_trk() != TRK_DATA && sim_int_count < sim_timings_count) {
        for (int i = 0; i < 6; ++i)
            handle_interrupt();
        while (track.get_trk() != TRK_DATA && process_interrupt_timing(&track)) {
        }
    }
#ifdef DBG_RAWCODE
    dbgf("IH_max_pending_timings = %d", IH_max_pending_timings);
#endif

#ifdef DBG_TRACK
    if (sim_int_count >= sim_timings_count) {
        dbg("]");
    }
#endif

    const RawCode *praw = track.get_rawcode();

#ifdef DBG_RAWCODE
    praw->debug_rawcode();
#endif

    Decoder *pdec = decode_rawcode(praw);
    if (pdec) {
#ifdef DBG_DECODER
        pdec->output_data();
#endif
        delete pdec;
    }

    if (sim_int_count >= sim_timings_count) {
        dbg("----- END TEST -----");
    }

#else // !defined(DBG_SIMULATE)

    dbg("Waiting for signal");
    track.treset();

    attachInterrupt(INT_RFINPUT, &handle_interrupt, CHANGE);
    while (track.get_trk() != TRK_DATA) {
        while (track.get_trk() != TRK_DATA && process_interrupt_timing(&track)) {
        }

        sleep_mode();
    }
    detachInterrupt(INT_RFINPUT);

#ifdef DBG_RAWCODE
    dbgf("IH_max_pending_timings = %d", IH_max_pending_timings);
#endif
    const RawCode *praw = track.get_rawcode();

#ifdef DBG_RAWCODE
    praw->debug_rawcode();
#endif

    decode_rawcode(praw);

#endif // DBG_SIMULATE

#ifdef DBG_TIMINGS
    for (unsigned int i = 0; i + 1 < ih_dbg_pos; i += 2) {
        dbgf("%4u, %4u  |  %5u, %5u", ih_dbg_timings[i], ih_dbg_timings[i + 1],
             ih_dbg_exec[i], ih_dbg_exec[i + 1]);
    }
#endif

}

// vim: ts=4:sw=4:tw=80:et
