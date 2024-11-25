/*
 * bms_datatypes.h
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#pragma once

#include "stdint.h"

/* ADBMS2950 CFGA Data structure*/
typedef struct
{
  uint8_t       ocen      :1;
  uint8_t       vs5       :1;
  uint8_t       vs4       :1;
  uint8_t       vs3       :1;
  uint8_t       vs2       :2;
  uint8_t       vs1       :2;

  uint8_t       injtm     :1;
  uint8_t       injecc    :1;
  uint8_t       injts     :1;
  uint8_t       injmon    :2;
  uint8_t       injosc    :2;

  uint8_t       soak      :3;
  uint8_t       vs10      :1;
  uint8_t       vs9       :1;
  uint8_t       vs8       :1;
  uint8_t       vs7       :1;
  uint8_t       vs6       :1;

  uint8_t       gpo6c     :2;
  uint8_t       gpo5c     :1;
  uint8_t       gpo4c     :1;
  uint8_t       gpo3c     :1;
  uint8_t       gpo2c     :1;
  uint8_t       gpo1c     :1;

  uint8_t       spi3w     :1;
  uint8_t       gpio1fe   :1;
  uint8_t       gpo6od    :1;
  uint8_t       gpo5od    :1;
  uint8_t       gpo4od    :1;
  uint8_t       gpo3od    :1;
  uint8_t       gpo2od    :1;
  uint8_t       gpo1od    :1;

  uint8_t       vb2mux    :1;
  uint8_t       vb1mux    :1;
  uint8_t       snapst    :1;
  uint8_t       refup     :1;
  uint8_t       commbk    :1;
  uint8_t       acci      :3;
} cfa_;

/*!< ADBMS2950 Configuration Register Group B structure */
typedef struct
{
  uint8_t   oc1ten    :1;
  uint8_t   oc1th     :7;

  uint8_t   oc2ten    :1;
  uint8_t   oc2th     :7;

  uint8_t   oc3ten    :1;
  uint8_t   oc3th     :7;

  uint8_t   octsel    :2;
  uint8_t   reften    :1;
  uint8_t   ocdp      :1;
  uint8_t   ocdgt     :2;

  uint8_t   ocbx      :1;
  uint8_t   ocax      :1;
  uint8_t   ocmode    :2;
  uint8_t   oc3gc     :1;
  uint8_t   oc2gc     :1;
  uint8_t   oc1gc     :1;
  uint8_t   ocod      :1;

  uint8_t       gpio4c    :1;
  uint8_t       gpio3c    :1;
  uint8_t       gpio2c    :1;
  uint8_t       gpio1c    :1;
  uint8_t       gpio2eoc  :1;
  uint8_t       diagsel   :3;
} cfb_;

/* ADBMS2950 Flag Register Data structure*/
typedef struct
{
  uint8_t  i1d          :1;
  uint8_t  v1d          :1;
  uint8_t  vdruv        :1;
  uint8_t  ocmm         :1;
  uint8_t  oc3l         :1;
  uint8_t  ocagd_clrm   :1;
  uint8_t  ocal         :1;
  uint8_t  oc1l         :1;

  uint8_t  i2d          :1;
  uint8_t  v2d          :1;
  uint8_t  vdduv        :1;
  uint8_t  noclk        :1;
  uint8_t  refflt       :1;
  uint8_t  ocbgd        :1;
  uint8_t  ocbl         :1;
  uint8_t  oc2l         :1;

  uint8_t  i2cnt        :3;
  uint16_t  i1cnt       :11;
  uint8_t  i1pha        :2;

  uint8_t  vregov       :1;
  uint8_t  vreguv       :1;
  uint8_t  vdigov       :1;
  uint8_t  vdiguv       :1;
  uint8_t  sed1         :1;
  uint8_t  med1         :1;
  uint8_t  sed2         :1;
  uint8_t  med2         :1;

  uint8_t  vdel         :1;
  uint8_t  vde          :1;
  uint8_t  spiflt       :1;
  uint8_t  reset        :1;
  uint8_t  thsd         :1;
  uint8_t  tmode        :1;
  uint8_t  oscflt       :1;
} flag_;


