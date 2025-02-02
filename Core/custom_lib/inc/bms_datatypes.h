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
  uint8_t       vs1       :2;
  uint8_t       vs2       :2;
  uint8_t       vs3       :1;
  uint8_t       vs4       :1;
  uint8_t       vs5       :1;
  uint8_t       ocen      :1;

  uint8_t       injosc    :2;
  uint8_t       injmon    :2;
  uint8_t       injts     :1;
  uint8_t       unused1   :1;
  uint8_t       injecc    :1;
  uint8_t       injtm     :1;

  uint8_t       vs6       :1;
  uint8_t       vs7       :1;
  uint8_t       vs8       :1;
  uint8_t       vs9       :1;
  uint8_t       vs10      :1;
  uint8_t       soak      :3;

  uint8_t       gpo1c     :1;
  uint8_t       gpo2c     :1;
  uint8_t       gpo3c     :1;
  uint8_t       gpo4c     :1;
  uint8_t       gpo5c     :1;
  uint8_t       gpo6c     :2;
  uint8_t       unused2   :1;

  uint8_t       gpo1od    :1;
  uint8_t       gpo2od    :1;
  uint8_t       gpo3od    :1;
  uint8_t       gpo4od    :1;
  uint8_t       gpo5od    :1;
  uint8_t       gpo6od    :1;
  uint8_t       gpio1fe   :1;
  uint8_t       spi3w     :1;

  uint8_t       acci      :3;
  uint8_t       commbk    :1;
  uint8_t       refup     :1;
  uint8_t       snapst    :1;
  uint8_t       vb1mux    :1;
  uint8_t       vb2mux    :1;
} ad29_cfa_t;

/*!< ADBMS2950 Configuration Register Group B structure */
typedef struct
{
  uint8_t   oc1th     :7;
  uint8_t   unused1   :1;

  uint8_t   oc2th     :7;
  uint8_t   unused2   :1;

  uint8_t   oc3th     :7;
  uint8_t   unused3   :1;

  uint8_t   ocdgt     :2;
  uint8_t   unused4   :1;
  uint8_t   ocdp      :1;
  uint8_t   unused5   :4;

  uint8_t   ocod      :1;
  uint8_t   oc1gc     :1;
  uint8_t   oc2gc     :1;
  uint8_t   oc3gc     :1;
  uint8_t   ocmode    :2;
  uint8_t   ocax      :1;
  uint8_t   ocbx      :1;

  uint8_t       diagsel   :3;
  uint8_t       gpio2eoc  :1;
  uint8_t       gpio1c    :1;
  uint8_t       gpio2c    :1;
  uint8_t       gpio3c    :1;
  uint8_t       gpio4c    :1;
} ad29_cfb_t;


/* For ADBMS6830 config register structure */
typedef struct
{
  uint8_t       cth     :3;
  uint8_t       unused1 :4;
  uint8_t       refon   :1;

  uint8_t       flag_d  :8;

  uint8_t       unused2 :3;
  uint8_t       owa     :3;
  uint8_t       owrng   :1;
  uint8_t       soakon  :1;

  uint8_t       gpo1to8 :8;

  uint8_t       gpo9to10:2;
  uint8_t       unused3 :6;

  uint8_t       fc      :3;
  uint8_t       comm_bk :1;
  uint8_t       mute_st :1;
  uint8_t       snap_st :1;
  uint8_t       unused4 :2;
} ad68_cfa_t;


/* For ADBMS6830 config register structure */
typedef struct
{
  uint8_t     vuv0to7 :8;

  uint8_t     vuv8to11:4;
  uint8_t     vov0to3 :4;

  uint8_t     vov4to11:8;

  uint8_t     dcto    :6;
  uint8_t     dtrng   :1;
  uint8_t     dtmen   :1;

  uint16_t    dcc     :16;
} ad68_cfb_t;


typedef struct
{
    uint8_t     pwm1    :4;
    uint8_t     pwm2    :4;

    uint8_t     pwm3    :4;
    uint8_t     pwm4    :4;

    uint8_t     pwm5    :4;
    uint8_t     pwm6    :4;

    uint8_t     pwm7    :4;
    uint8_t     pwm8    :4;

    uint8_t     pwm9    :4;
    uint8_t     pwm10   :4;

    uint8_t     pwm11   :4;
    uint8_t     pwm12   :4;
} ad68_pwma_t;


typedef struct
{
    uint8_t     pwm13   :4;
    uint8_t     pwm14   :4;

    uint8_t     pwm15   :4;
    uint8_t     pwm16   :4;

    uint32_t    rsv     :32;
} ad68_pwmb_t;





