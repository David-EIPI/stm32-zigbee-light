/**
  ******************************************************************************
  * @file           : binsensor.h
  * @author         : David Shirvanyants
  * @date           : Oct 1, 2024
  * @brief          : Header for basic Binary{Input,Output,Value} clusters.
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef BINSENSOR_H_
#define BINSENSOR_H_

#include "zcl/zcl.h"

/*--------------------------------------------------------------------------
 *  DESCRIPTION
 *      Interface definition for the Basic Binary Sensor cluster.
 *--------------------------------------------------------------------------
 */


/*
 * @enum Basic Binary cluster types
 */
enum {
    ZCL_CLUSTER_BINARY_INPUT = 0x000f, /* Binary input cluster - read a sensor */
    ZCL_CLUSTER_BINARY_OUTPUT = 0x0010, /* Binary input output - control a device */
    ZCL_CLUSTER_BINARY_VALUE = 0x0011 /* Binary value cluster - setting or parameter */
};

/*
 * @enum Basic Binary cluster types (named enum for typed function parameter)
 */
enum binary_sensor_mode {
    BINARY_SENSOR_INPUT = 0x000f, /*!< Binary input cluster - read a sensor */
    BINARY_SENSOR_OUTPUT = 0x0010, /*!< Binary input output - control a device */
    BINARY_SEBSOR_VALUE = 0x0011 /*!< Binary value cluster - setting or parameter */
};

/*
 * @enum Binary cluster attribute identifiers
 * */
enum {
	ZCL_BINARY_ACTIVE_TEXT_ATTR = 0x0004,
	ZCL_BINARY_DESCRIPTION_ATTR = 0x001C,
	ZCL_BINARY_INACTIVE_TEXT_ATTR = 0x002E,
	ZCL_BINARY_OUT_OF_SERVICE_ATTR = 0x0051,
	ZCL_BINARY_POLARITY_ATTR = 0x0054,
	ZCL_BINARY_PRESENT_VALUE_ATTR = 0x0055,
	ZCL_BINARY_RELIABILITY_ATTR = 0x0067,
	ZCL_BINARY_STATUS_FLAGS_ATTR = 0x006F,
	ZCL_BINARY_APPLICATION_TYPE_ATTR = 0x0100,

};

/*
 * @struct Attribute access callbacks
 * */
struct zcl_binary_attr_callbacks_t {
    enum ZclStatusCodeT (*read)(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info, void *arg);
    enum ZclStatusCodeT (*write)(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info, void *arg);
};

/*
 * @struct Binary cluster data
 * */
struct zcl_binary_server_cluster_t {
    struct ZbZclClusterT cluster;
    struct zcl_binary_attr_callbacks_t callbacks;
    void *arg;
    uint8_t *valuePtr;
};


struct ZbZclClusterT * ZbZcl_Binary_ServerAlloc(struct ZigBeeT *zb, uint8_t endpoint, enum binary_sensor_mode mode,
		uint8_t *valuePtr, struct zcl_binary_attr_callbacks_t *callbacks, void *arg);


#endif /* BINSENSOR_H_ */
