


#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "user_lib.h"

unsigned int cal_average(const unsigned int *pdata, unsigned int size)
{
    unsigned int _u32Sum = 0;
    unsigned int _u32Cnt = 0;
    if ((pdata == NULL) || (size == 0)) { return 0; }
    for (_u32Cnt = 0; _u32Cnt < size; _u32Cnt++) {
        _u32Sum += pdata[_u32Cnt];
    }
    return (_u32Sum/size);
}

unsigned short petkit_crc16(const unsigned char* p_data, unsigned int size, const unsigned short* p_crc)
{
	unsigned int i;
    unsigned short crc = (p_crc == NULL) ? 0xffff : *p_crc;

    for (i = 0; i < size; i++) {
        crc = (unsigned char) (crc >> 8) | (crc << 8);
        crc ^= p_data[i];
        crc ^= (unsigned char) (crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }

    return crc;
}

int ASCIItoNum(char s08Char)
{
    int s32Data = 0;
    if ((s08Char < '0')
     || (s08Char > '9'))
    {
        printf("Input char Out '0' ~ '9', is %c \n", s08Char);
        return s32Data;
    }
    s32Data = s08Char - '0';
    return s32Data;
}

int StringtoNum(char *strChar, unsigned short u16Len)
{
    int s32Data = 0;
    unsigned int u32CharLen = 0;
    unsigned int u32Cnt = 0;
    if ((strChar == NULL)
     || (u16Len == 0))
    {
        printf("Input params Error! \n");
        return s32Data;
    }
    u32CharLen = strlen(strChar);
    if (u32CharLen > u16Len) { u32CharLen = u16Len; }
    for (u32Cnt = 0; u32Cnt < u32CharLen; u32Cnt++)
    {
        s32Data += (ASCIItoNum(strChar[u32Cnt]) * pow_n(10, (u32CharLen - u32Cnt - 1)));
    }
    return s32Data;
}

char NumtoASCII(int s32Num)
{
    char s08Data = 0;
    int s32Data = s32Num % 10;
    s08Data = s32Data + '0';
    return s08Data;
}
int NumtoString(const unsigned int u32SrcNum, unsigned int u32Len, char *strDst)
{
    int s32StrLen = u32Len;
    unsigned short u16Cnt = 0;
    unsigned int u32Data = u32SrcNum;
    unsigned int u32Remainder = 0;

    if ((strDst == NULL) 
     || (u32Len == 0))
    { 
        printf("strDst is NULL! \n");
        return -1; 
    }

    for (u16Cnt = 0; u16Cnt < u32Len; u16Cnt++)
    {
        u32Remainder = u32Data % 10;
        u32Data = u32Data / 10;
        strDst[u32Len - u16Cnt - 1] =  NumtoASCII(u32Remainder);
    }

    return s32StrLen;
}

double pow_n(double a, unsigned int n)
{
	unsigned int u32Cnt = 0;
	double dbRtData = 1.0;
	for (u32Cnt = 0; u32Cnt < n; u32Cnt++)
	{
		dbRtData = dbRtData * a;
	}
	return dbRtData;    
}

void disable_timer_tick(timer_tick_t *p_timer)
{
    if (p_timer) {
        p_timer->enable = 0;
    }
}
void set_timer_tick(timer_tick_t *p_timer, int os_time, int delay_time, int random_range)
{
    int _random_cal = 0;
    if (random_range) {
        _random_cal = random() % random_range;
    }
    if (p_timer) {
        p_timer->enable = 1;
        p_timer->base_tick = os_time;
        p_timer->tick_left = os_time + _random_cal;
        p_timer->next_tick = os_time + _random_cal + delay_time;
    }
}
int Is_timer_tick_up(timer_tick_t *p_timer, int os_time)
{
    int _ret = 0;
    if (p_timer->enable) {
        if (p_timer->next_tick < os_time) {
            _ret = 1;
        }
    }
    return _ret;
}

