//
// Created by user on 08/03/22.
//

#ifndef OPENAIRINTERFACE_FAPI_NVIPC_H
#define OPENAIRINTERFACE_FAPI_NVIPC_H

#include "../nvidia/include/nvIPC/nv_ipc.h"
#include "../nvidia/include/nvIPC/nv_ipc_utils.h"
#include "../nvidia/include/nvlog/nvlog.h"
#include <nfapi_vnf_interface.h>
#include <debug.h>

typedef struct {
  uint8_t num_msg;
  uint8_t opaque_handle;
  uint16_t message_id;
  uint32_t message_length;
} fapi_phy_api_msg;

int test_nv_ipc_send_msg_P5(void* packedBuf, uint32_t packedMsgLength, nfapi_p4_p5_message_header_t* header);
int test_nv_ipc_send_msg_P7(void* packedBuf, uint32_t packedMsgLength, nfapi_p7_message_header_t* header);
void set_config(nfapi_vnf_config_t *conf);
int nvIPC_Init();

#endif //OPENAIRINTERFACE_FAPI_NVIPC_H
