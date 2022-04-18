# NTN Adaptation for the 3GPP stack #

# 5g-ntn
 Solutions for LTE and 5G-NR to support Non-Terrestrial Networks (NTN). 
<table style="border-collapse: collapse; border: none;">
  <tr style="border-collapse: collapse; border: none;">
    <td style="border-collapse: collapse; border: none;">
      <a href="https://www.securityandtrust.lu//">
         <img src="https://i.imgur.com/HVsVMK2.png?raw=true" alt="" border=3 height=220 width=300>
         </img>
      </a>
    </td>
    <td style="border-collapse: collapse; border: none; vertical-align: center;">
      <b><font size = "5">Solutions for LTE and 5G-NR to support Non-Terrestrial Networks (NTN)</font></b>
    </td>
  </tr>
</table>

#### Citation
1- [Random Access Procedure Over Non-Terrestrial Networks: From Theory to Practice
](https://ieeexplore.ieee.org/document/9502071)

#### List of Actions

1. Added source code for eNodeB NTN 
2. Technical Document [Random Access Procedure Over Non-Terrestrial Networks: From Theory to Practice
](https://ieeexplore.ieee.org/document/9502071)
3. Source code documentation **Pending ...**
4. Hands-On guide **Pending ...**

|OpenAirInterface5G development and test suite for 4G/5G Non-Terrestrial Network support. Current work is dedicated to support NTN delay using 4G/5G waveform over LEO and GEO orbits. Current progress supports a fixed delay up to 20 msec with IP connectivity and up to 800 msec RTT with only data transmission|
| --- |

> Complete set of features will be provided.



# Guidelines: #

* [Hardware and Software Requirements (**START HERE**)](req)
* [EPC](epc)
* [FlexRAN](flexran)
* [eNodeB](enb)
* [UE](ue)
* [NTN Channel Emulator](chem)


# OpenAirInterface License #

OpenAirInterface is under OpenAirInterface Software Alliance license.

 *  [OAI License Model](http://www.openairinterface.org/?page_id=101)
 *  [OAI License v1.1 on our website](http://www.openairinterface.org/?page_id=698)

It is distributed under **OAI Public License V1.1**.

The license information is distributed under [LICENSE](LICENSE) file in the same directory.

Please see [NOTICE](NOTICE.md) file for third party software that is included in the sources.

# Where to Start #

 *  [The implemented features](./doc/FEATURE_SET.md)
 *  [How to build](./doc/BUILD.md)
 *  [How to run the modems](./doc/RUNMODEM.md)

# RAN repository structure #

The OpenAirInterface (OAI) software is composed of the following parts: 

<pre>
openairinterface5g
├── ci-scripts        : Meta-scripts used by the OSA CI process. Contains also configuration files used day-to-day by CI.
├── cmake_targets     : Build utilities to compile (simulation, emulation and real-time platforms), and generated build files.
├── common            : Some common OAI utilities, other tools can be found at openair2/UTILS.
├── doc               : Contains an up-to-date feature set list and starting tutorials.
├── executables       : Top-level executable source files.
├── LICENSE           : License file.
├── maketags          : Script to generate emacs tags.
├── nfapi             : Contains the NFAPI code. A local Readme file provides more details.
├── openair1          : 3GPP LTE Rel-10/12 PHY layer / 3GPP NR Rel-15 layer. A local Readme file provides more details.
│   ├── PHY
│   ├── SCHED
│   ├── SCHED_NBIOT
│   ├── SCHED_NR
│   ├── SCHED_NR_UE
│   ├── SCHED_UE
│   └── SIMULATION    : PHY RF simulation.
├── openair2          : 3GPP LTE Rel-10 RLC/MAC/PDCP/RRC/X2AP + LTE Rel-14 M2AP implementation. Also 3GPP NR Rel-15 RLC/MAC/PDCP/RRC/X2AP.
│   ├── COMMON
│   ├── DOCS
│   ├── ENB_APP
│   ├── F1AP
│   ├── GNB_APP
│   ├── LAYER2/RLC/   : with the following subdirectories: UM_v9.3.0, TM_v9.3.0, and AM_v9.3.0.
│   ├── LAYER2/PDCP/PDCP_v10.1.0
│   ├── M2AP
│   ├── MCE_APP
│   ├── NETWORK_DRIVER
│   ├── NR_PHY_INTERFACE
│   ├── NR_UE_PHY_INTERFACE
│   ├── PHY_INTERFACE
│   ├── RRC
│   ├── UTIL
│   └── X2AP
├── openair3          : 3GPP LTE Rel10 for S1AP, NAS GTPV1-U for both ENB and UE.
│   ├── COMMON
│   ├── DOCS
│   ├── GTPV1-U
│   ├── M3AP
│   ├── MME_APP
│   ├── NAS
│   ├── S1AP
│   ├── SCTP
│   ├── SECU
│   ├── TEST
│   ├── UDP
│   └── UTILS
└── targets           : Top-level wrappers for unitary simulation for PHY channels, system-level emulation (eNB-UE with and without S1), and realtime eNB and UE and RRH GW.
</pre>
