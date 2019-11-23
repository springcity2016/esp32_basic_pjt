
#ifndef __PK_SERIAL_H_
#define __PK_SERIAL_H_


#ifdef __cplusplus
extern "C" {
#endif


#pragma pack(1)
typedef struct CommFunc_t
{
    unsigned char u08SendCmd;           // ESP -> MCU 的 CMD
    unsigned char u08Node;              // ESP -> MCU 的 Node
    unsigned char u08Seq;               // ESP -> MCU 的 Seq
    unsigned short u16Length;            // Datas Length   - 是 *pu08Data的数据长度 
    unsigned char *pu08Data;            // Active PackDatas
}CommFunc_t;
#pragma pack()



int pk_serial_init(void);


int pk_serial_taskloop(void *pvParams);


int pkcomm_timer_start(int force);
int pkcomm_timer_stop(void);



int pk_esp_mcu_comm(unsigned char cmd, unsigned char wr_read, unsigned char node, const unsigned char *pcdata, unsigned short size, unsigned char bfInt);


#ifdef __cplusplus
}
#endif



#endif
