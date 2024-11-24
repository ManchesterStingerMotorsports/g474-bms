/**
********************************************************************************
*
* @file:    adi_bms_2950data.h
*
* @brief:   This file contains 2950 data, enums & functions.
*
* @details:
*
*******************************************************************************
Copyright(c) 2020 Analog Devices, Inc. All Rights Reserved. This software is
proprietary & confidential to Analog Devices, Inc. and its licensors. By using
this software you agree to the terms of the associated Analog Devices License
Agreement.
*******************************************************************************
*/
/*! \addtogroup BMS_Driver
*  @{
*/

/*! \addtogroup Data_Management
*  @{
*/

#pragma once

/*============= I N C L U D E S =============*/
/*============== D E F I N E S ===============*/
/*============= E X T E R N A L S ============*/
/*============= E N U M E R A T O R S ============*/

#include "common.h"

namespace AD29_NS 
{
  constexpr int VR_SIZE = 12;              /*!< Bms ic number of Voltage Resister  */
  constexpr int VRX_SIZE = 6;              /*!< Bms ic number of Voltage Resister X */
  constexpr int RVR_SIZE = 6;              /*!< Bms ic number of Redundant Voltage Resister  */
  constexpr int COMM = 3;                  /*!< communication comm reg byte size   */
  constexpr int RSID = 6;                  /*!< Bms ic number of SID byte          */
  constexpr int TX_DATA = 6;               /*!< Bms tx data byte                   */
  constexpr int RX_DATA = 8;               /*!< Bms rx data byte                   */
  constexpr int RDALLA_SIZE =    34;       /*!< RDALLA data byte size              */
  constexpr int RDALLB_SIZE =    34;       /*!< RDALLB data byte size              */
  constexpr int RDCALL_SIZE =    34;       /*!< RDCALL data byte size              */
  constexpr int RDASALL_SIZE =   16;       /*!< RDASALL data byte size             */
  constexpr int ALLVR_SIZE =     24;       /*!< ALL Voltage Reg. byte size         */
  constexpr int ALLREDVR_SIZE =  12;       /*!< ALL Redundant Voltage Reg. byte size*/

  /*!< ADBMS2950 Configuration Register Group A structure */
  typedef struct
  {
    uint8_t       refon   :1;
    uint8_t       vs1     :2;
    uint8_t       vs10    :1;
    uint8_t       flag_d  :8;
    uint8_t       soak    :3;
    uint8_t       vs6     :1;
    uint8_t       vs7     :1;
    uint8_t       gpio    :4;
    uint8_t       gpo     :6;
    uint8_t       snap_st :1;
    uint8_t       comm_bk :1;
    uint8_t       vs5     :1;
    uint8_t       vs4     :1;
    uint8_t       vs3     :1;
  }cfa_;

  /*!< ADBMS2950 Configuration Register Group B structure */
  typedef struct
  {
    uint16_t 	oc1th   :7;
    uint16_t 	oc2th   :7;
    uint8_t 	vs2     :2;
    uint8_t 	dg1th   :3;
    uint8_t 	dg2th   :3;
    uint8_t 	vs9     :1;
    uint8_t 	vs8     :1;
  }cfb_;

  /*!< ADBMS2950 ClrFlag Register Data structure*/
  typedef struct
  {
    uint8_t  cl_oc1a      :1;
    uint8_t  cl_oc1m      :1;
    uint8_t  cl_oc2a      :1;
    uint8_t  cl_oc2m      :1;

    uint8_t  cl_opt2_med  :1;
    uint8_t  cl_opt2_ed   :1;
    uint8_t  cl_opt1_med  :1;
    uint8_t  cl_opt1_ed   :1;
    uint8_t  cl_vduv      :1;
    uint8_t  cl_vdov      :1;
    uint8_t  cl_vauv      :1;
    uint8_t  cl_vaov      :1;

    uint8_t  cl_oscchk    :1;
    uint8_t  cl_tmode     :1;
    uint8_t  cl_thsd      :1;
    uint8_t  cl_sleep     :1;
    uint8_t  cl_spiflt    :1;
    uint8_t  cl_vdel      :1;
    uint8_t  cl_vde       :1;
  } clrflag_;

  /*!< ADBMS2950 Current Register Data structure */
  typedef struct
  {
    uint32_t i1;
    uint32_t i2;
  } crg_;

  /*!< ADBMS2950 Battery Voltage Register Data structure */
  typedef struct
  {
    uint16_t vbat1;
    uint16_t vbat2;
  } vbat_;

  /*!< ADBMS2950 Current and Battery Voltage Register Data structure */
  typedef struct
  {
    uint32_t i1;
    uint16_t vbat1;
  } i_vbat_;

  /*!< ADBMS2950 Overcurrent ADC Register Data structure */
  typedef struct
  {
    uint8_t oc1r;
    uint8_t oc2r;
  } oc_;

  /*!< ADBMS2950 Average Current Register Data structure */
  typedef struct
  {
    uint32_t i1avg;
    uint32_t i2avg;
  } iavg_;

  /*!< ADBMS2950 Average Batter Voltage Register Data structure */
  typedef struct
  {
    uint32_t vb1avg;
    uint32_t vb2avg;
  } vbavg_;

  /*!< ADBMS2950 Average Batter Current and Voltage Register Data structure */
  typedef struct
  {
    uint32_t i1avg;
    uint32_t vb1avg;
  } i_vbavg_;

  /*!< ADBMS2950 Voltage Register Data structure */
  typedef struct
  {
    uint16_t v_codes[VR_SIZE];
  } vr_;

  /*!< ADBMS2950 Voltage Register X Data structure */
  typedef struct
  {
    uint16_t vx_codes[VRX_SIZE];
  } vrx_;

  /*!< ADBMS2950 Redundant Voltage Register Data structure */
  typedef struct
  {
    uint16_t redv_codes[RVR_SIZE];
  } rvr_;

  /*!< ADBMS2950 Status A Register Data structure */
  typedef struct
  {
    uint16_t vref1p25;
    uint16_t itmp;
    uint16_t vreg2;
  } stata_;

  /*!< ADBMS2950 Status B Register Data structure */
  typedef struct
  {
    uint8_t oc1min;
    uint8_t oc1max;
    uint8_t oc2min;
    uint8_t oc2max;
  }statb_;

  /*!< ADBMS2950 Status C Register Data structure */
  typedef struct
  {
    uint8_t oc1a           :1;
    uint8_t oc1a_inv       :1;
    uint8_t oc2a           :1;
    uint8_t oc2a_inv       :1;

    uint8_t cts           :2;
    uint16_t ct           :11;

    uint8_t va_ov         :1;
    uint8_t va_uv         :1;
    uint8_t vd_ov         :1;
    uint8_t vd_uv         :1;
    uint8_t otp1_ed       :1;
    uint8_t otp1_med      :1;
    uint8_t otp2_ed       :1;
    uint8_t otp2_med      :1;

    uint8_t vde           :1;
    uint8_t vdel          :1;
    uint8_t insync        :1;
    uint8_t spiflt        :1;
    uint8_t sleep         :1;
    uint8_t thsd          :1;
    uint8_t tmodchk       :1;
    uint8_t oscchk        :1;
  } statc_;

  /*!< ADBMS2950 Status D Register Data structure */
  typedef struct
  {
    uint8_t oc_cntr;
  } statd_;

  /*!< ADBMS2950 Status E Register Data structure */
  typedef struct
  {
    uint8_t gpio  :4;
    uint8_t gpo   :6;
    uint8_t rev   :4;
  } state_;

  /*!< ADBMS2950 COMM register Data structure*/
  typedef struct
  {
    uint8_t fcomm[COMM];
    uint8_t icomm[COMM];
    uint8_t data[COMM];
  } com_;

  /*!< ADBMS2950 SID Register Structure */
  typedef struct
  {
    uint8_t sid[RSID];
  } sid_;

  /*!< Transmit byte and recived byte data structure */
  typedef struct
  {
    uint8_t tx_data[TX_DATA];
    uint8_t rx_data[RX_DATA];
  } ic_register_;

  /*!< Command counter and pec error data Structure */
  typedef struct
  {
    uint8_t cmd_cntr;
    uint8_t cfgr_pec;
    uint8_t cr_pec;
    uint8_t vbat_pec;
    uint8_t ivbat_pec;
    uint8_t oc_pec;
    uint8_t avgcr_pec;
    uint8_t avgvbat_pec;
    uint8_t avgivbat_pec;
    uint8_t vr_pec;
    uint8_t vrx_pec;
    uint8_t rvr_pec;
    uint8_t comm_pec;
    uint8_t stat_pec;
    uint8_t sid_pec;
  } cmdcnt_pec_;

  /*!< Diagnostic test result data structure */
  typedef struct
  {
    uint8_t osc_mismatch;
    uint8_t supply_error;
    uint8_t supply_ovuv;
    uint8_t thsd;
    uint8_t fuse_ed;
    uint8_t fuse_med;
    uint8_t tmodchk;
  } diag_test_;

  /*!< ADBMS2950 IC main structure */
  typedef struct
  {
    cfa_ tx_cfga;
    cfa_ rx_cfga;
    cfb_ tx_cfgb;
    cfb_ rx_cfgb;
    clrflag_ clflag;
    crg_ i;
    iavg_ iavg;
    vbat_ vbat;
    vbavg_ vbavg;
    i_vbat_ ivbat;
    i_vbavg_ i_vbavg;
    vr_  vr;
    vrx_ vrx;
    rvr_ rvr;
    oc_ oc;
    stata_ stata;
    statb_ statb;
    statc_ statc;
    statd_ statd;
    state_ state;
    com_ tx_comm;
    com_ rx_comm;
    sid_ sid;
    ic_register_ configa;
    ic_register_ configb;
    ic_register_ clrflag;
    ic_register_ reg;
    ic_register_ sta;
    ic_register_ stb;
    ic_register_ stc;
    ic_register_ std;
    ic_register_ ste;
    ic_register_ com;
    ic_register_ rsid;
    cmdcnt_pec_ cccrc;
  } cell_asic;

  /*!< *************************************** CMD Enums *************************************************/
  /*!
  *  \enum Single & Round-Robin Measurement.
  * VCH: Single & Round-Robin Measurement Cmd bytes.
  */
  typedef enum
  {
    SM_V1 = 0,
    SM_V2,
    SM_V3,
    SM_V4,
    SM_V5,
    SM_V6,
    SM_V7_V9,
    SM_V8_V10,
    SM_VREF2,
    RR_VCH0_VCH8,
    RR_VCH0_VCH7,
    RR_VCH0_VCH5,
    RR_VCH0_VCH3,
    RR_VCH4_VCH8,
    RR_VCH4_VCH7,
    RR_VCH6_VCH8
  }VCH;

  /*!
  *  \enum Selection for Aux Inputs.
  * ACH: Selection for Aux Inputs Cmd bytes.
  */
  typedef enum
  {
    ALL = 0,
    ITMP_TEMP,
    VREF1P25,
    VREG,
    CMD_IGNORED
  }ACH;

  /*!
  *  \enum CONT
  * CONT: Continuous or single measurement.
  */
  typedef enum { SINGLE = 0X0, CONTINUOUS = 0X1} CONT;

  /*!
  *  \enum OPEN WIRE
  * OW: OPEN WIRE.
  */
  typedef enum { OW_OFF = 0X0, OW_SRC_P_SNK_N, OW_SRC_N_SNK_P, OW_OFF_} OW;

  /*!
  *  \enum ERR
  * ERR: Inject error is spi read out.
  */
  /* Inject error is spi read out */
  typedef enum  { WITHOUT_ERR = 0x0, WITH_ERR = 0x1 } ERR;

  /*!
  *  \enum RD
  * RD: Read Device.
  */
  typedef enum { RD_OFF = 0X0, RD_ON = 0X1} RD;


  /**************************************** Mem bits *************************************************/
  /*!< Configuration Register A */
  /*!
  *  \enum REFON
  * REFON: Refernece remains power up/down.
  */
  typedef enum  { PWR_DOWN = 0x0, PWR_UP = 0x1 } REFON;

  /*!
  *  \enum SOAK
  * SOAK: Enables soak on V- ADCs
  */
  typedef enum
  {
    SOAK_DISABLE = 0,
    SOAK_100us,
    SOAK_500us,
    SOAK_1ms,
    SOAK_2ms,
    SOAK_10ms,
    SOAK_20ms,
    SOAK_150ms
  }SOAK;

  /*!
  *  \enum FLAG_D
  * FLAG_D: Fault flags.
  */
  typedef enum
  {
    FLAG_D0 = 0,          /* Force oscillator counter fast */
    FLAG_D1,              /* Force oscillator counter slow */
    FLAG_D2,              /* Force Supply Error detection  */
    FLAG_D3,              /* FLAG_D[3]: 1--> Select Supply OV and delta detection, 0 --> Selects UV */
    FLAG_D4,              /* Set THSD */
    FLAG_D5,              /* Force Fuse ED */
    FLAG_D6,              /* Force Fuse MED */
    FLAG_D7,              /* Force TMODCHK  */
  } FLAG_D;

  typedef enum  { FLAG_CLR = 0x0, FLAG_SET = 0x1 } CFGA_FLAG;

  /*!
  *  \enum CL FLAG
  * CL FLAG: Fault clear bit set or clear enum
  */
  typedef enum  { CL_FLAG_CLR = 0x0, CL_FLAG_SET = 0x1 } CL_FLAG;

  /*!
  *  \enum COMM_BK
  * COMM_BK: Communication Break.
  */
  typedef enum  { COMM_BK_OFF = 0x0, COMM_BK_ON = 0x1 } COMM_BK;

  /*!
  *  \enum SNAPSHOT
  * SNAPSHOT: Snapshot.
  */
  typedef enum  { SNAP_OFF = 0x0, SNAP_ON = 0x1 } SNAPSHOT;

  /*!
  *  \enum GPIO
  * GPIO: GPIO Pins.
  */
  typedef enum
  {
    GPIO1 = 0,
    GPIO2,
    GPIO3,
    GPIO4,
  } GPIO;

  /*!
  *  \enum GPIO
  * GPIO: GPIO Pin Control.
  */
  typedef enum  { GPIO_CLR = 0x0, GPIO_SET = 0x1 } CFGA_GPIO;

  /*!
  *  \enum GPO
  * GPO: GPO Pins.
  */
  typedef enum
  {
    GPO1 = 0,
    GPO2,
    GPO3,
    GPO4,
    GPO5,
    GPO6
  } GPO;

  /*!
  *  \enum GPO
  * GPIO: GPO Pin Control.
  */
  typedef enum  { GPO_CLR = 0x0, GPO_SET = 0x1 } CFGA_GPO;

  /*!
  *  \enum Reference Voltage for VSx (x=3 to 10) measurement
  * VSx: Reference Voltage for VSx measurement.
  */
  typedef enum  { VSMV_SGND = 0x0, VSMV_VREF2P5 = 0x1 } VSB;

  /*!
  *  \enum Reference Voltage for VSx[1:0] Measurement
  * VS[1:0]: Reference Voltage for VSx[1:0] Measurement.
  */
  typedef enum
  {
    VSM_SGND = 0,
    VSM_VREF2P5,
    VSM_V3,
    VSM_V4
  } VS;

  /*!< Configuration Register B */

  /*!< General Enums */
  typedef enum { ALL_GRP = 0x0, A, B, C, D, E, F, X, Y, Z, NONE} GRP;
  typedef enum { Vr = 0x0, Vrx, Rvr, Wrcfg, Config, Cr, Vbat, Ivbat, Oc, AvgCr, AvgVbat, AvgIvbat, Status, Wrcomm, Comm, SID, Time, Clrflag, Rdalla, Rdallb, Rdcall, Rdasall} TYPE;
  typedef enum { OSC_MISMATCH = 0x0, SUPPLY_ERROR, THSD, FUSE_ED, FUSE_MED, TMODCHK} DIAGNOSTIC_TYPE;
  typedef enum { DISABLED = 0X0, ENABLED = 0X1} LOOP_MEASURMENT;
  typedef enum { FAIL = 0x0, PASS } RESULT ;

  void adBms2950_Adi1(RD rd, CONT cont, OW ow);
  void adBms2950_Adi2(CONT cont, OW ow);
  void adBms2950_Adv(OW ow, VCH vch);
  void adBms2950_Adaux(ACH ach);
  void adBms2950_Stcomm(void);
  uint8_t adBms2950ConfigA_Flag(FLAG_D flag_d, CFGA_FLAG flag);
  uint8_t adBms2950ConfigA_Gpio(GPIO gpio, CFGA_GPIO set_clr);
  uint8_t adBms2950ConfigA_Gpo(GPO gpo, CFGA_GPO set_clr);
  void adBms2950ParseConfiga(uint8_t tIC, cell_asic *ic, uint8_t *data);
  void adBms2950ParseConfigb(uint8_t tIC, cell_asic *ic, uint8_t *data);
  void adBms2950ParseConfig(uint8_t tIC, cell_asic *ic, GRP grp, uint8_t *data);
  void adBms2950ParseCurrentRegData(uint8_t tIC, cell_asic *ic, uint8_t *i_data);
  void adBms2950ParseVbatRegData(uint8_t tIC, cell_asic *ic, uint8_t *vbat_data);
  void adBms2950ParseIVbatRegData(uint8_t tIC, cell_asic *ic, uint8_t *ivbat_data);
  void adBms2950ParseOcRegData(uint8_t tIC, cell_asic *ic, uint8_t *oc_data);
  void adBms2950ParseAvgCurrentRegData(uint8_t tIC, cell_asic *ic, uint8_t *iavg_data);
  void adBms2950ParseAvgVbatRegData(uint8_t tIC, cell_asic *ic, uint8_t *avgvbat_data);
  void adBms2950ParseAvgIVbatRegData(uint8_t tIC, cell_asic *ic, uint8_t *avgivbat_data);
  void adBms2950VrParseData(uint8_t tIC, cell_asic *ic, GRP grp, uint8_t *vr_data);
  void adBms2950VrxParseData(uint8_t tIC, cell_asic *ic, GRP grp, uint8_t *vrx_data);
  void adBms2950RedVrParseData(uint8_t tIC, cell_asic *ic, GRP grp, uint8_t *redvr_data);
  void adBms2950ParseStatusA(uint8_t tIC, cell_asic *ic, uint8_t *data);
  void adBms2950ParseStatusB(uint8_t tIC, cell_asic *ic, uint8_t *data);
  void adBms2950ParseStatusC(uint8_t tIC, cell_asic *ic, uint8_t *data);
  void adBms2950ParseStatusD(uint8_t tIC, cell_asic *ic, uint8_t *data);
  void adBms2950ParseStatusE(uint8_t tIC, cell_asic *ic, uint8_t *data);
  void adBms2950ParseStatus(uint8_t tIC, cell_asic *ic, GRP grp, uint8_t *data);
  void adBms2950ParseComm(uint8_t tIC, cell_asic *ic, uint8_t *data);
  void adBms2950ParseSID(uint8_t tIC, cell_asic *ic, uint8_t *data);
  void adBms2950CreateConfiga(uint8_t tIC, cell_asic *ic);
  void adBms2950CreateConfigb(uint8_t tIC, cell_asic *ic);
  void adBms2950CreateClrflagData(uint8_t tIC, cell_asic *ic);
  void adBms2950CreateComm(uint8_t tIC, cell_asic *ic);

}

/** @}*/
/** @}*/
