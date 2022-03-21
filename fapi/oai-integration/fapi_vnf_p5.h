//
// Created by user on 11/04/22.
//

#ifndef OPENAIRINTERFACE_FAPI_VNF_P5_H
#define OPENAIRINTERFACE_FAPI_VNF_P5_H
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "nfapi_nr_interface_scf.h"
#include "nfapi_vnf_interface.h"
#include "nfapi_vnf.h"
#include "nfapi.h"
#include "nfapi/oai_integration/vendor_ext.h"
#include "fapi_nvIPC.h"
//#include "nfapi/oai_integration/nfapi.c"

#include "PHY/defs_eNB.h"
#include "PHY/LTE_TRANSPORT/transport_proto.h"
#include "openair2/LAYER2/NR_MAC_gNB/nr_mac_gNB.h"
#include <targets/RT/USER/lte-softmodem.h>

#include "common/ran_context.h"
#include "openair2/PHY_INTERFACE/queue_t.h"
#include "gnb_ind_vars.h"
#include "nfapi/open-nFAPI/vnf/inc/vnf.h"

void aerial_configure_nr_fapi_vnf();
int aerial_nr_send_config_request(nfapi_vnf_config_t *config, int p5_idx);
uint8_t aerial_unpack_nr_param_response(uint8_t **ppReadPackedMsg, uint8_t *end, void *msg, nfapi_p4_p5_codec_config_t *config);
uint8_t aerial_unpack_nr_config_response(uint8_t **ppReadPackedMsg, uint8_t *end, void *msg, nfapi_p4_p5_codec_config_t *config);
int aerial_pnf_nr_connection_indication_cb(nfapi_vnf_config_t *config, int p5_idx);
int aerial_nfapi_nr_vnf_p7_start(nfapi_vnf_p7_config_t* config);

int oai_fapi_ul_tti_req(nfapi_nr_ul_tti_request_t *ul_tti_req);
int oai_fapi_ul_dci_req(nfapi_nr_ul_dci_request_t *ul_dci_req);
int oai_fapi_tx_data_req(nfapi_nr_tx_data_request_t *tx_data_req);
int oai_fapi_dl_tti_req(nfapi_nr_dl_tti_request_t *dl_config_req);
#endif //OPENAIRINTERFACE_FAPI_VNF_P5_H