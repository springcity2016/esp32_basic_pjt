
#ifndef __PKJIG_SERIAL_H_
#define __PK_SERIAL_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "pkjig_comm_table.h"

// #pragma pack(1)
// typedef struct CommFunc_t
// {
//     unsigned char u08SendCmd;           // ESP -> MCU 的 CMD
//     unsigned char u08Node;              // ESP -> MCU 的 Node
//     unsigned char u08Seq;               // ESP -> MCU 的 Seq
//     unsigned short u16Length;            // Datas Length   - 是 *pu08Data的数据长度 
//     unsigned char *pu08Data;            // Active PackDatas
// }CommFunc_t;
// #pragma pack()



int pkjig_serial_init(void);


int pkjig_serial_taskloop(void *pvParams);

int pkjig_esp_mcu_comm(unsigned char cmd, unsigned char node, const unsigned char *pcdata, unsigned short size, unsigned char bfInt);

/*****对外通信接口******************************************************************/
int pkjig_set_out_data(unsigned short addr, unsigned short data);
int pkjig_get_in_data(unsigned short addr, unsigned short data);

// int pkjig_set_out_data(jig_gpio_e port, );
/*********************************************************************************/


#ifdef __cplusplus
}
#endif



#endif
