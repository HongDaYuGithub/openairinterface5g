/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/* \file NR_IF_Module.c
 * \brief functions for NR UE FAPI-like interface
 * \author R. Knopp, K.H. HSU
 * \date 2018
 * \version 0.1
 * \company Eurecom / NTUST
 * \email: knopp@eurecom.fr, kai-hsiang.hsu@eurecom.fr
 * \note
 * \warning
 */

#include "PHY/defs_nr_UE.h"
#include "NR_IF_Module.h"
#include "NR_MAC_UE/mac_proto.h"
#include "assertions.h"
#include "NR_MAC_UE/mac_extern.h"
#include "SCHED_NR_UE/fapi_nr_ue_l1.h"
#include "executables/softmodem-common.h"
#include "openair2/RRC/NR_UE/rrc_proto.h"

#include <stdio.h>

#define MAX_IF_MODULES 100

const char *dl_indication_type[] = {"MIB", "SIB", "DLSCH", "DCI", "RAR"};

queue_t dl_itti_config_req_tx_data_req_queue;
queue_t ul_dci_config_req_queue;

UL_IND_t *UL_INFO = NULL;


static nr_ue_if_module_t *nr_ue_if_module_inst[MAX_IF_MODULES];
static int ue_tx_sock_descriptor = -1;
static int ue_rx_sock_descriptor = -1;
int current_sfn_slot;
sem_t sfn_slot_semaphore;

void nrue_init_standalone_socket(const char *addr, int tx_port, int rx_port)
{
  {
    struct sockaddr_in server_address;
    int addr_len = sizeof(server_address);
    memset(&server_address, 0, addr_len);
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(tx_port);

    int sd = socket(server_address.sin_family, SOCK_DGRAM, 0);
    if (sd < 0)
    {
      LOG_E(MAC, "Socket creation error standalone PNF\n");
      return;
    }

    if (inet_pton(server_address.sin_family, addr, &server_address.sin_addr) <= 0)
    {
      LOG_E(MAC, "Invalid standalone PNF Address\n");
      close(sd);
      return;
    }

    // Using connect to use send() instead of sendto()
    if (connect(sd, (struct sockaddr *)&server_address, addr_len) < 0)
    {
      LOG_E(MAC, "Connection to standalone PNF failed: %s\n", strerror(errno));
      close(sd);
      return;
    }
    assert(ue_tx_sock_descriptor == -1);
    ue_tx_sock_descriptor = sd;
    LOG_D(NR_RRC, "Sucessfully set up tx_socket in %s.\n", __FUNCTION__);
  }

  {
    struct sockaddr_in server_address;
    int addr_len = sizeof(server_address);
    memset(&server_address, 0, addr_len);
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(rx_port);

    int sd = socket(server_address.sin_family, SOCK_DGRAM, 0);
    if (sd < 0)
    {
      LOG_E(MAC, "Socket creation error standalone PNF\n");
      return;
    }

    if (inet_pton(server_address.sin_family, addr, &server_address.sin_addr) <= 0)
    {
      LOG_E(MAC, "Invalid standalone PNF Address\n");
      close(sd);
      return;
    }

    if (bind(sd, (struct sockaddr *)&server_address, addr_len) < 0)
    {
      LOG_E(MAC, "Connection to standalone PNF failed: %s\n", strerror(errno));
      close(sd);
      return;
    }
    assert(ue_rx_sock_descriptor == -1);
    ue_rx_sock_descriptor = sd;
    LOG_D(NR_RRC, "Sucessfully set up rx_socket in %s.\n", __FUNCTION__);
  }
  LOG_I(NR_RRC, "NRUE standalone socket info: tx_port %d  rx_port %d on %s.\n",
        tx_port, rx_port, addr);
}

void send_nsa_standalone_msg(nfapi_nr_rach_indication_t *rach_ind)
{
    char buffer[NFAPI_MAX_PACKED_MESSAGE_SIZE];
    int encoded_size = nfapi_nr_p7_message_pack(rach_ind, buffer, sizeof(buffer), NULL);
    LOG_I(NR_MAC, "NR_RACH_IND sent to Proxy, Size: %d Frame %d Subframe %d\n", encoded_size,
          rach_ind->sfn, rach_ind->slot);
    if (send(ue_tx_sock_descriptor, buffer, encoded_size, 0) < 0)
    {
        LOG_E(NR_MAC, "Send Proxy NR_UE failed\n");
        return;
    }
}
static void save_nr_measurement_info(nfapi_nr_dl_tti_request_t *dl_tti_request)
{
    int num_pdus = dl_tti_request->dl_tti_request_body.nPDUs;
    char buffer[MAX_MESSAGE_SIZE];
    if (num_pdus <= 0)
    {
        LOG_E(NR_PHY, "%s: dl_tti_request number of PDUS <= 0\n", __FUNCTION__);
        abort();
    }
    LOG_D(NR_PHY, "%s: dl_tti_request number of PDUS: %d\n", __FUNCTION__, num_pdus);
    for (int i = 0; i < num_pdus; i++)
    {
        nfapi_nr_dl_tti_request_pdu_t *pdu_list = &dl_tti_request->dl_tti_request_body.dl_tti_pdu_list[i];
        if (pdu_list->PDUType == NFAPI_NR_DL_TTI_SSB_PDU_TYPE)
        {
            LOG_D(NR_PHY, "Cell_id: %d, the ssb_block_idx %d, sc_offset: %d and payload %d\n",
                pdu_list->ssb_pdu.ssb_pdu_rel15.PhysCellId,
                pdu_list->ssb_pdu.ssb_pdu_rel15.SsbBlockIndex,
                pdu_list->ssb_pdu.ssb_pdu_rel15.SsbSubcarrierOffset,
                pdu_list->ssb_pdu.ssb_pdu_rel15.bchPayload);
            pdu_list->ssb_pdu.ssb_pdu_rel15.ssbRsrp = 60;
            LOG_D(NR_RRC, "Setting pdulist[%d].ssbRsrp to %d\n", i, pdu_list->ssb_pdu.ssb_pdu_rel15.ssbRsrp);
        }
    }

    size_t pack_len = nfapi_nr_p7_message_pack((void *)dl_tti_request,
                                    buffer,
                                    sizeof(buffer),
                                    NULL);
    if (pack_len < 0)
    {
        LOG_E(NR_PHY, "%s: Error packing nr p7 message.\n", __FUNCTION__);
    }
    nsa_sendmsg_to_lte_ue(buffer, pack_len, NR_UE_RRC_MEASUREMENT);
    LOG_A(NR_RRC, "Populated NR_UE_RRC_MEASUREMENT information and sent to LTE UE\n");
}

void *nrue_standalone_pnf_task(void *context)
{
  struct sockaddr_in server_address;
  socklen_t addr_len = sizeof(server_address);
  char buffer[NFAPI_MAX_PACKED_MESSAGE_SIZE];
  int sd = ue_rx_sock_descriptor;
  assert(sd > 0);
  LOG_I(NR_RRC, "Sucessfully started %s.\n", __FUNCTION__);

  while (true)
  {
    ssize_t len = recvfrom(sd, buffer, sizeof(buffer), MSG_TRUNC, (struct sockaddr *)&server_address, &addr_len);
    if (len == -1)
    {
      LOG_E(NR_PHY, "reading from standalone pnf sctp socket failed \n");
      continue;
    }
    if (len > sizeof(buffer))
    {
      LOG_E(NR_PHY, "%s(%d). Message truncated. %zd\n", __FUNCTION__, __LINE__, len);
      continue;
    }
    if (len == sizeof(uint16_t))
    {
      uint16_t sfn_slot = 0;
      memcpy((void *)&sfn_slot, buffer, sizeof(sfn_slot));
      current_sfn_slot = sfn_slot;

      if (sem_post(&sfn_slot_semaphore) != 0)
      {
        LOG_E(NR_PHY, "sem_post() error\n");
        abort();
      }
      int sfn = NFAPI_SFNSLOT2SFN(sfn_slot);
      int slot = NFAPI_SFNSLOT2SLOT(sfn_slot);
      LOG_I(NR_PHY, "Received from proxy sfn %d slot %d\n", sfn, slot);
    }
    else
    {
      nfapi_p7_message_header_t header;
      nfapi_nr_dl_tti_request_t dl_tti_request;
      if (nfapi_p7_message_header_unpack((void *)buffer, len, &header, sizeof(header), NULL) < 0)
      {
        LOG_E(NR_PHY, "Header unpack failed for nrue_standalone pnf\n");
        continue;
      }
      else
      {
        switch (header.message_id)
        {
          case NFAPI_NR_PHY_MSG_TYPE_DL_TTI_REQUEST:
            LOG_D(NR_PHY, "Received an NFAPI_NR_PHY_MSG_TYPE_DL_TTI_REQUEST message. \n");
            if (nfapi_nr_p7_message_unpack((void *)buffer, len, &dl_tti_request,
                                           sizeof(nfapi_nr_dl_tti_request_t), NULL) < 0)
            {
                LOG_E(NR_PHY, "Message dl_tti_request failed to unpack\n");
                break;
            }
            save_nr_measurement_info(&dl_tti_request);
            break;
          case NFAPI_NR_PHY_MSG_TYPE_TX_DATA_REQUEST:
            LOG_I(NR_PHY, "Received an NFAPI_NR_PHY_MSG_TYPE_TX_DATA_REQUEST message. \n");
            break;
          case NFAPI_NR_PHY_MSG_TYPE_UL_DCI_REQUEST:
            LOG_I(NR_PHY, "Received an NFAPI_NR_PHY_MSG_TYPE_UL_DCI_REQUEST message. \n");
            break;
          case NFAPI_NR_PHY_MSG_TYPE_UL_TTI_REQUEST:
            LOG_I(NR_PHY, "Received an NFAPI_NR_PHY_MSG_TYPE_UL_TTI_REQUEST message. \n");
            if (nfapi_nr_p7_message_unpack((void *)buffer, len, &dl_tti_request,
                                           sizeof(dl_tti_request), NULL) < 0)
            {
                LOG_E(NR_PHY, "Message dl_tti_request failed to unpack\n");
                break;
            }
            break;
          default:
            LOG_E(NR_PHY, "Case Statement has no corresponding nfapi message, this is the header ID %d\n", header.message_id);
            break;
        }
      }
    }
  } //while(true)
}

//  L2 Abstraction Layer
int handle_bcch_bch(module_id_t module_id, int cc_id, unsigned int gNB_index, uint8_t *pduP, unsigned int additional_bits, uint32_t ssb_index, uint32_t ssb_length, uint16_t cell_id){

  return nr_ue_decode_mib(module_id,
			  cc_id,
			  gNB_index,
			  additional_bits,
			  ssb_length,  //  Lssb = 64 is not support    
			  ssb_index,
			  pduP, 
			  cell_id);

}

//  L2 Abstraction Layer
int handle_bcch_dlsch(module_id_t module_id, int cc_id, unsigned int gNB_index, uint32_t sibs_mask, uint8_t *pduP, uint32_t pdu_len){
  return nr_ue_decode_sib1(module_id, cc_id, gNB_index, sibs_mask, pduP, pdu_len);
}

//  L2 Abstraction Layer
int handle_dci(module_id_t module_id, int cc_id, unsigned int gNB_index, frame_t frame, int slot, fapi_nr_dci_indication_pdu_t *dci){

  return nr_ue_process_dci_indication_pdu(module_id, cc_id, gNB_index, frame, slot, dci);

}

// L2 Abstraction Layer
// Note: sdu should always be processed because data and timing advance updates are transmitted by the UE
int8_t handle_dlsch (nr_downlink_indication_t *dl_info, NR_UL_TIME_ALIGNMENT_t *ul_time_alignment, int pdu_id){

  nr_ue_send_sdu(dl_info, ul_time_alignment, pdu_id);

  return 0;

}

int nr_ue_ul_indication(nr_uplink_indication_t *ul_info){

  NR_UE_L2_STATE_t ret;
  module_id_t module_id = ul_info->module_id;
  NR_UE_MAC_INST_t *mac = get_mac_inst(module_id);

  ret = nr_ue_scheduler(NULL, ul_info);

  if (is_nr_UL_slot(mac->scc, ul_info->slot_tx))
    nr_ue_prach_scheduler(module_id, ul_info->frame_tx, ul_info->slot_tx, ul_info->thread_id);

  switch(ret){
  case UE_CONNECTION_OK:
    break;
  case UE_CONNECTION_LOST:
    break;
  case UE_PHY_RESYNCH:
    break;
  case UE_PHY_HO_PRACH:
    break;
  default:
    break;
  }

  return 0;
}

int nr_ue_dl_indication(nr_downlink_indication_t *dl_info, NR_UL_TIME_ALIGNMENT_t *ul_time_alignment){

  int32_t i;
  uint32_t ret_mask = 0x0;
  module_id_t module_id = dl_info->module_id;
  NR_UE_MAC_INST_t *mac = get_mac_inst(module_id);
  fapi_nr_dl_config_request_t *dl_config = &mac->dl_config_request;
  
  if (!dl_info->dci_ind && !dl_info->rx_ind) {
    // UL indication to schedule DCI reception
    nr_ue_scheduler(dl_info, NULL);
  } else {
    // UL indication after reception of DCI or DL PDU
    if(dl_info->dci_ind != NULL){
      LOG_D(MAC,"[L2][IF MODULE][DL INDICATION][DCI_IND]\n");
      for(i=0; i<dl_info->dci_ind->number_of_dcis; ++i){
        LOG_D(MAC,">>>NR_IF_Module i=%d, dl_info->dci_ind->number_of_dcis=%d\n",i,dl_info->dci_ind->number_of_dcis);
        nr_scheduled_response_t scheduled_response;
        int8_t ret = handle_dci(dl_info->module_id,
                                dl_info->cc_id,
                                dl_info->gNB_index,
                                dl_info->frame,
                                dl_info->slot,
                                dl_info->dci_ind->dci_list+i);

        ret_mask |= (ret << FAPI_NR_DCI_IND);
        if (ret >= 0) {
          AssertFatal( nr_ue_if_module_inst[module_id] != NULL, "IF module is NULL!\n" );
          AssertFatal( nr_ue_if_module_inst[module_id]->scheduled_response != NULL, "scheduled_response is NULL!\n" );
          fill_scheduled_response(&scheduled_response, dl_config, NULL, NULL, dl_info->module_id, dl_info->cc_id, dl_info->frame, dl_info->slot, dl_info->thread_id);
          nr_ue_if_module_inst[module_id]->scheduled_response(&scheduled_response);
        }
      }
    }

    if(dl_info->rx_ind != NULL){

      for(i=0; i<dl_info->rx_ind->number_pdus; ++i){

        LOG_D(MAC, "In %s sending DL indication to MAC. 1 PDU type %s of %d total number of PDUs \n",
          __FUNCTION__,
          dl_indication_type[dl_info->rx_ind->rx_indication_body[i].pdu_type - 1],
          dl_info->rx_ind->number_pdus);

        switch(dl_info->rx_ind->rx_indication_body[i].pdu_type){

        case FAPI_NR_RX_PDU_TYPE_MIB:

          ret_mask |= (handle_bcch_bch(dl_info->module_id, dl_info->cc_id, dl_info->gNB_index,
                      (dl_info->rx_ind->rx_indication_body+i)->mib_pdu.pdu,
                      (dl_info->rx_ind->rx_indication_body+i)->mib_pdu.additional_bits,
                      (dl_info->rx_ind->rx_indication_body+i)->mib_pdu.ssb_index,
                      (dl_info->rx_ind->rx_indication_body+i)->mib_pdu.ssb_length,
                      (dl_info->rx_ind->rx_indication_body+i)->mib_pdu.cell_id)) << FAPI_NR_RX_PDU_TYPE_MIB;

          break;

        case FAPI_NR_RX_PDU_TYPE_SIB:

          ret_mask |= (handle_bcch_dlsch(dl_info->module_id,
                       dl_info->cc_id, dl_info->gNB_index,
                      (dl_info->rx_ind->rx_indication_body+i)->sib_pdu.sibs_mask,
                      (dl_info->rx_ind->rx_indication_body+i)->sib_pdu.pdu,
                      (dl_info->rx_ind->rx_indication_body+i)->sib_pdu.pdu_length)) << FAPI_NR_RX_PDU_TYPE_SIB;

          break;

        case FAPI_NR_RX_PDU_TYPE_DLSCH:

          ret_mask |= (handle_dlsch(dl_info, ul_time_alignment, i)) << FAPI_NR_RX_PDU_TYPE_DLSCH;

          break;

        case FAPI_NR_RX_PDU_TYPE_RAR:

          ret_mask |= (handle_dlsch(dl_info, ul_time_alignment, i)) << FAPI_NR_RX_PDU_TYPE_RAR;

          break;

        default:
          break;
        }
      }
    }

    //clean up nr_downlink_indication_t *dl_info
    dl_info->rx_ind = NULL;
    dl_info->dci_ind = NULL;

  }
  return 0;
}

nr_ue_if_module_t *nr_ue_if_module_init(uint32_t module_id){

  if (nr_ue_if_module_inst[module_id] == NULL) {
    nr_ue_if_module_inst[module_id] = (nr_ue_if_module_t *)malloc(sizeof(nr_ue_if_module_t));
    memset((void*)nr_ue_if_module_inst[module_id],0,sizeof(nr_ue_if_module_t));

    nr_ue_if_module_inst[module_id]->cc_mask=0;
    nr_ue_if_module_inst[module_id]->current_frame = 0;
    nr_ue_if_module_inst[module_id]->current_slot = 0;
    nr_ue_if_module_inst[module_id]->phy_config_request = nr_ue_phy_config_request;
    if (get_softmodem_params()->nsa) //Melissa, this is also a hack. Get a better flag.
      nr_ue_if_module_inst[module_id]->scheduled_response = nr_ue_scheduled_response_stub;
    else
      nr_ue_if_module_inst[module_id]->scheduled_response = nr_ue_scheduled_response;
    nr_ue_if_module_inst[module_id]->dl_indication = nr_ue_dl_indication;
    nr_ue_if_module_inst[module_id]->ul_indication = nr_ue_ul_indication;
  }

  return nr_ue_if_module_inst[module_id];
}

int nr_ue_if_module_kill(uint32_t module_id) {

  if (nr_ue_if_module_inst[module_id] != NULL){
    free(nr_ue_if_module_inst[module_id]);
  }
  return 0;
}

int nr_ue_dcireq(nr_dcireq_t *dcireq) {

  fapi_nr_dl_config_request_t *dl_config = &dcireq->dl_config_req;
  NR_UE_MAC_INST_t *UE_mac = get_mac_inst(0);
  dl_config->sfn = UE_mac->dl_config_request.sfn;
  dl_config->slot = UE_mac->dl_config_request.slot;

  LOG_D(PHY, "Entering UE DCI configuration frame %d slot %d \n", dcireq->frame, dcireq->slot);

  ue_dci_configuration(UE_mac, dl_config, dcireq->frame, dcireq->slot);

  return 0;
}
