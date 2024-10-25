/**
  ******************************************************************************
  * @file           : multistate.h
  * @author         : David Shirvanyants
  * @date           : Sep 11, 2024
  * @brief          : Header for basic Multistate{Input,Output,Value} clusters.
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef _MULTISTATE_H_
#define _MULTISTATE_H_

#include "zcl/zcl.h"

/*--------------------------------------------------------------------------
 *  DESCRIPTION
 *      Interface definition for the Custom Multistate value cluster.
 *      Based on the custom Long String cluster definition from the examples.
 *--------------------------------------------------------------------------
 */

/*
 * @enum Basic Multistate cluster types
 */
enum {
    ZCL_CLUSTER_MULTISTATE_INPUT = 0x0012, /*!< Multistate input cluster - read a sensor */
    ZCL_CLUSTER_MULTISTATE_OUTPUT = 0x0013, /*!< Multistate input output - control a device */
    ZCL_CLUSTER_MULTISTATE_VALUE = 0x0014 /*!< Multistate value cluster - setting or parameter */
};

/*
 * @enum Basic Multistate cluster types (named enum for typed function parameter)
 */
enum multistate_mode {
    MULTISTATE_INPUT = 0x0012, /*!< Multistate input cluster - read a sensor */
    MULTISTATE_OUTPUT = 0x0013, /*!< Multistate input output - control a device */
    MULTISTATE_VALUE = 0x0014 /*!< Multistate value cluster - setting or parameter */
};


/* Custom response  */
#define ZCL_MSV_RSP     0x01

/*
 * @enum Multistate cluster attribute identifiers
 * */
enum {
	ZCL_MULTISTATE_STATE_TEXT_ATTR = 0x000E,
	ZCL_MULTISTATE_DESCRIPTION_ATTR = 0x001C,
	ZCL_MULTISTATE_NUMBER_OF_STATES_ATTR = 0x004A,
	ZCL_MULTISTATE_OUT_OF_SERVICE_ATTR = 0x0051,
	ZCL_MULTISTATE_PRESENT_VALUE_ATTR = 0x0055,
	ZCL_MULTISTATE_STATUS_FLAGS_ATTR = 0x006F,

};

/*
 * @struct Attribute access callback
 * */
struct zcl_msv_attr_callbacks_t {
    enum ZclStatusCodeT (*msv_presentValue)(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info);
};

/*
 * @struct Multistate cluster data
 * */
struct zcl_msv_server_cluster_t {
    struct ZbZclClusterT cluster;
    struct zcl_msv_attr_callbacks_t callbacks;
    void *arg;
    uint8_t state_text_len;
    uint8_t state_text_buffer[0];
};


struct ZbZclClusterT * ZbZcl_msv_ServerAlloc(struct ZigBeeT *zb, uint8_t endpoint, enum multistate_mode mode, unsigned numitems,
		const char **itemtext, struct zcl_msv_attr_callbacks_t *callbacks, void *arg);


#endif
