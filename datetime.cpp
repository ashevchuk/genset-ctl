#include "datetime.h"

// Days in Jan..Nov (Dec omitted deliberately — see date2days()); ported
// directly from RTClib.
static const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30};

static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000U) y -= 2000U;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i) days += daysInMonth[i - 1];
    if (m > 2 && y % 4 == 0) ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static uint32_t time2ulong(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24UL + h) * 60 + m) * 60 + s;
}

static uint8_t conv2d(const char *p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9') v = *p - '0';
    return 10 * v + *++p - '0';
}

static uint8_t monthFromName(char c0, char c1, char c2) {
    switch (c0) {
        case 'J': return (c1 == 'a') ? 1 : ((c2 == 'n') ? 6 : 7);
        case 'F': return 2;
        case 'A': return c2 == 'r' ? 4 : 8;
        case 'M': return c2 == 'r' ? 3 : 5;
        case 'S': return 9;
        case 'O': return 10;
        case 'N': return 11;
        case 'D': return 12;
        default:  return 1;
    }
}

DateTime::DateTime(uint32_t t) {
    t -= SECONDS_FROM_1970_TO_2000;

    ss_ = t % 60; t /= 60;
    mm_ = t % 60; t /= 60;
    hh_ = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;
    for (yOff_ = 0;; ++yOff_) {
        leap = yOff_ % 4 == 0;
        if (days < 365U + leap) break;
        days -= 365 + leap;
    }
    for (m_ = 1; m_ < 12; ++m_) {
        uint8_t daysPerMonth = daysInMonth[m_ - 1];
        if (leap && m_ == 2) ++daysPerMonth;
        if (days < daysPerMonth) break;
        days -= daysPerMonth;
    }
    d_ = days + 1;
}

DateTime::DateTime(uint16_t year, uint8_t month, uint8_t day,
                    uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000U) year -= 2000U;
    yOff_ = (uint8_t)year;
    m_ = month;
    d_ = day;
    hh_ = hour;
    mm_ = min;
    ss_ = sec;
}

DateTime::DateTime(const char *date, const char *time) {
    yOff_ = conv2d(date + 9);
    m_ = monthFromName(date[0], date[1], date[2]);
    d_ = conv2d(date + 4);
    hh_ = conv2d(time);
    mm_ = conv2d(time + 3);
    ss_ = conv2d(time + 6);
}

uint8_t DateTime::dayOfTheWeek(void) const {
    uint16_t day = date2days(yOff_, m_, d_);
    return (day + 6) % 7;  // Jan 1, 2000 was a Saturday
}

uint32_t DateTime::unixtime(void) const {
    uint16_t days = date2days(yOff_, m_, d_);
    uint32_t t = time2ulong(days, hh_, mm_, ss_);
    t += SECONDS_FROM_1970_TO_2000;
    return t;
}
