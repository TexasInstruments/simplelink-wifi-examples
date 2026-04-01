#include "date_time_service.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "osi_kernel.h"
#include "task.h"




uint8_t g_dateTimeInitDone = 0;


struct tm;


int32_t datetime_init()
{
    g_dateTimeInitDone = 1;
    return 0;
}

int32_t datetime_deinit()
{
    g_dateTimeInitDone = 0;
    return 0;
}

/* return second since epoch time :00:00 Jan 1 1970*/
uint32_t datetime_secondsGet(void)
{
    struct timespec ts = {0};


    if(!g_dateTimeInitDone)
    {
        Report("\n\rdatetime was not initialize");
        return 0;
    }
    ts.tv_sec = osi_GetDateTimeS();

    return(ts.tv_sec);
}

/* set second from epoch time :00:00 Jan 1 1970*/
void datetime_SecondsSet(uint32_t newtime)
{
    osi_SetDateTimeS(newtime);
}


void datetime_printCurTime()
{

    time_t t = time(&t);
    struct tm *currentTime = gmtime(&t);

    // Format 1: YYYY-MM-DD HH:MM:SS
    Report("\n\rCurrent UTC time: ");
    Report("%04d-%02d-%02d",currentTime->tm_year + 1900, currentTime->tm_mon + 1, currentTime->tm_mday);
    Report("T%02d:%02d:%02d\n",currentTime->tm_hour, currentTime->tm_min, currentTime->tm_sec);
}

static bool datetime_isLeapYear(int y)
{
    return (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
}
/* convert date and time to epoch time */
uint32_t datetime_to_epoch(uint32_t year, uint32_t month, uint32_t day,
        uint32_t hour, uint32_t minute, uint32_t second) {
    static const int daysBeforeMonth[] = { 0,31,59,90,120,151,181,212,243,273,304,334 };

     // Days from 1970 to current year
     int y = year - 1;
     uint32_t days = (y - 1969) * 365
     + (y - 1969) / 4
     - (y - 1969) / 100
     + (y - 1600) / 400;

     // Days in current year
     days += daysBeforeMonth[month - 1];
     if (month > 2 && datetime_isLeapYear(year)) {
         days += 1; // Leap day
     }

     days += day - 1; // Days in current month

     return days * 86400UL + hour * 3600UL + minute * 60UL + second;
}

// Convert epoch ? date/time (UTC)
void epoch_to_datetime(uint32_t epoch,
        uint32_t *year, uint32_t *month, uint32_t *day,
        uint32_t *hour, uint32_t *minute, uint32_t *second) {
    uint32_t seconds_in_day = epoch % 86400UL;
    uint32_t days = epoch / 86400UL;

    // Time of day
    *hour = seconds_in_day / 3600;
    *minute = (seconds_in_day % 3600) / 60;
    *second = seconds_in_day % 60;

    // Algorithm from Howard Hinnant's civil_from_days
    // Shift so that 1970-01-01 is day 0
    int32_t z = days + 719468;
    int32_t era = (z >= 0 ? z : z - 146096) / 146097;
    int32_t doe = z - era * 146097; // [0, 146096]
    int32_t yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365; // [0, 399]
    int32_t y = yoe + era * 400;
    int32_t doy = doe - (365*yoe + yoe/4 - yoe/100); // [0, 365]
    int32_t mp = (5*doy + 2) / 153; // [0, 11]
    int32_t d = doy - (153*mp+2)/5 + 1; // [1, 31]
    int32_t m = mp + (mp < 10 ? 3 : -9); // [1, 12]
    y += (m <= 2);

    *year = y;
    *month = m;
    *day = d;
}


 int32_t testDateTime(void) {
    // Convert to epoch
    uint32_t epoch = datetime_to_epoch(2025, 8, 10, 14, 30, 15);
    Report("Epoch: %lu\n", (unsigned long)epoch);

    // Convert back
    uint32_t y, m, d, hh, mm, ss;
    epoch_to_datetime(epoch, &y, &m, &d, &hh, &mm, &ss);
    Report("Converted back: %04d-%02d-%02d %02d:%02d:%02d\n",
    y, m, d, hh, mm, ss);

return 0;
}


