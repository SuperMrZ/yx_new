#include "usart.h"
#include "bsp_can.h"
#include "damiao.h"
#include "main.h"
#include "bsp_uart.h"
#include "motorCmd.h"
#include "ins_task.h"
#include "crc.h"
#include "usbd_cdc_if.h"

// typedef struct {
//     uint8_t header;
//     uint8_t version;
//     union {
//         struct {
//             uint8_t detect_color : 1;
//             uint8_t reset_tracker : 1;
//             uint8_t mode : 3;
//             uint8_t reserved : 3;
//         } bits;
//         uint8_t value;
//     } flags;
//     uint8_t valid_flags;
//     float pitch;
//     float yaw;
//     float bullet_speed;
//     int8_t pitch_bias;
//     int8_t yaw_bias;
//     int8_t pretime_bias;
//     uint16_t checksum;
// } ReceivePacket __attribute__((packed));


void up_receive(uint8_t* Buf, uint32_t *Len);

 void StartTask02(void *argument);
 void Sendmessage(void *argument);






