
#ifndef __PKJIG_COMM_TABLE_H_
#define __PKJIG_COMM_TABLE_H_


#define JIGCOMM_HEADER_0_DATA        (0x5A)
#define JIGCOMM_HEADER_1_DATA        (0xA5)

#define JIGCOMM_ACK_SUCCESS          (0xFF)

#define JIGCOMM_HEADER_0_INDEX       (0)
#define JIGCOMM_HEADER_1_INDEX       (1)
#define JIGCOMM_LEN_INDEX            (2)
#define JIGCOMM_CMD_INDEX            (3)
#define JIGCOMM_SEQ_INDEX            (4)
#define JIGCOMM_ACK_INDEX            (5)
#define JIGCOMM_NODE_INDEX           (6)
#define JIGCOMM_PAYLOAD_INDEX        (7)

#define JIGCOMM_HEADERS_SIZE         (7)
#define JIGCOMM_CRC_SIZE             (2)

#define JIG_IN_0_ADDR           (0x0001)
#define JIG_IN_1_ADDR           (0x0002)
#define JIG_IN_2_ADDR           (0x0003)
#define JIG_IN_3_ADDR           (0x0004)
#define JIG_IN_4_ADDR           (0x0005)
#define JIG_IN_5_ADDR           (0x0006)
#define JIG_IN_6_ADDR           (0x0007)
#define JIG_IN_7_ADDR           (0x0008)
#define JIG_IN_ALL_ADDR         (0x000F)

#define JIG_OUT_0_ADDR          (0x0010)
#define JIG_OUT_1_ADDR          (0x0020)
#define JIG_OUT_2_ADDR          (0x0030)
#define JIG_OUT_3_ADDR          (0x0040)
#define JIG_OUT_4_ADDR          (0x0050)
#define JIG_OUT_5_ADDR          (0x0060)
#define JIG_OUT_6_ADDR          (0x0070)
#define JIG_OUT_7_ADDR          (0x0080)
#define JIG_OUT_ALL_ADDR        (0x00F0)

typedef enum
{
    JIG_IN_0 = 0,
    JIG_IN_1,
    JIG_IN_2,
    JIG_IN_3,
    JIG_IN_4,
    JIG_IN_5,
    JIG_IN_6,
    JIG_IN_7,
    JIG_IN_ALL,

    JIG_OUT_0 = 0,
    JIG_OUT_1,
    JIG_OUT_2,
    JIG_OUT_3,
    JIG_OUT_4,
    JIG_OUT_5,
    JIG_OUT_6,
    JIG_OUT_7,
    JIG_OUT_ALL,

}jig_gpio_e;

enum
{
    E_JIG_COMM_SET_IO_CMD = 0,
    E_JIG_COMM_GET_IO_CMD,


    E_JIG_COMM_MAX_CMD,
};


enum
{
    E_JIG_COMM_UART_TYPE = 0,
    E_JIG_COMM_IIC_TYPE,
    E_JIG_COMM_SPI_TYPE,

    E_JIG_COMM_MAX_TYPE
};

#pragma pack(1)
typedef struct _jig_io_payload_t
{
    unsigned short addr;
    unsigned short data;
}jig_io_payload_t;
#pragma pack()

typedef struct _jig_params_t
{
    unsigned char comm_type;        // 通信选择  // E_JIG_COMM_UART_TYPE / E_JIG_COMM_IIC_TYPE / E_JIG_COMM_SPI_TYPE
    
    
}jig_params_t;





#endif
