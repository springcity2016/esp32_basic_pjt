
#ifndef __USER_LIB_H_
#define __USER_LIB_H_



unsigned int cal_average(const unsigned int *pdata, unsigned int size);

unsigned short petkit_crc16(const unsigned char* p_data, unsigned int size, const unsigned short* p_crc);

int ASCIItoNum(char s08Char);
int StringtoNum(char *strChar, unsigned short u16Len);
char NumtoASCII(int s32Num);
int NumtoString(const unsigned int u32SrcNum, unsigned int u32Len, char *strDst);

double pow_n(double a, unsigned int n);         // 平方


typedef struct _timer_tcik_t
{
    unsigned char enable;
    int base_tick;
    int tick_left;
    int next_tick;
}timer_tick_t;
void disable_timer_tick(timer_tick_t *p_timer);
void set_timer_tick(timer_tick_t *p_timer, int os_time, int delay_time, int random_range);
int Is_timer_tick_up(timer_tick_t *p_timer, int os_time);

#endif


