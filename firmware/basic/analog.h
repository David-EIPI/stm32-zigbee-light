/**
  ******************************************************************************
  * @file           : analog.h
  * @author         : David Shirvanyants
  * @date           : Sep 23, 2024
  * @brief          : Header for basic Analog{Input,Output,Value} clusters.
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef ANALOG_H_
#define ANALOG_H_

#include "zcl/zcl.h"

/*--------------------------------------------------------------------------
 *  DESCRIPTION
 *      Interface definition for the Custom Analog value cluster.
 *      Based on the custom Multistate Value cluster.
 *--------------------------------------------------------------------------
 */

/*
 * @enum Engineering unit constants (compatible with Home Assistant Zigbee extension)
 */
enum {
	ZCL_ENGINEERING_UNIT_AMPERES = 3,
	ZCL_ENGINEERING_UNIT_VOLTS = 5,
	ZCL_ENGINEERING_UNIT_HERTZ = 27,
	ZCL_ENGINEERING_UNIT_LUMENS = 36,
	ZCL_ENGINEERING_UNIT_LUXES = 37,
	ZCL_ENGINEERING_UNIT_PASCALS = 53,
	ZCL_ENGINEERING_UNIT_DEG_C = 62,
	ZCL_ENGINEERING_UNIT_DEG_F = 64,
	ZCL_ENGINEERING_UNIT_HOURS = 71,
	ZCL_ENGINEERING_UNIT_MINUTES = 72,
	ZCL_ENGINEERING_UNIT_SECONDS = 73,
	ZCL_ENGINEERING_UNIT_NONE = 95,
	ZCL_ENGINEERING_UNIT_PERCENT = 98,
	ZCL_ENGINEERING_UNIT_MILLISECONDS = 159,
	ZCL_ENGINEERING_UNIT_MICROSECONDS = 194,
};

/*
 * @enum Device icon constants (compatible with Home Assistant Zigbee extension)
 */
enum {
	ZCL_ANALOG_ICON_TEMP_C,
	ZCL_ANALOG_ICON_WATER_PERCENT,
	ZCL_ANALOG_ICON_GAUGE,
	ZCL_ANALOG_ICON_SPEEDOMETER,
	ZCL_ANALOG_ICON_PERCENT,
	ZCL_ANALOG_ICON_AIR_FILTER,
	ZCL_ANALOG_ICON_FAN,
	ZCL_ANALOG_ICON_FLASH,
	ZCL_ANALOG_ICON_CURRENT_AC,
	ZCL_ANALOG_ICON_COUNTER = 12,
	ZCL_ANALOG_ICON_THERMOMETER_LINES,
	ZCL_ANALOG_ICON_TIMER,
	ZCL_ANALOG_ICON_PALETTE,
	ZCL_ANALOG_ICON_BRIGHTNESS_PERCENT,
};

/*
 * @enum Basic Analog cluster types
 */
enum {
    ZCL_CLUSTER_ANALOG_INPUT = 0x000c, /*!< Analog input cluster - read a sensor */
    ZCL_CLUSTER_ANALOG_OUTPUT = 0x000d, /*!< Analog input output - control a device */
    ZCL_CLUSTER_ANALOG_VALUE = 0x000e /*!< Analog value cluster - a setting or a parameter */
};

/*
 * @enum Basic Analog cluster types (named enum for typed function parameter)
 */
enum analog_mode {
    ANALOG_INPUT = 0x000c, /*!< Analog input cluster - read a sensor */
    ANALOG_OUTPUT = 0x000d, /*!< Analog input output - control a device */
    ANALOG_VALUE = 0x000e /*!< Analog value cluster - setting or parameter */
};

/*
 * @enum Analog cluster attribute identifiers
 * */
enum {
	ZCL_ANALOG_DESCRIPTION_ATTR = 0x001C,
	ZCL_ANALOG_MAX_PRESENT_ATTR = 0x0041,
	ZCL_ANALOG_MIN_PRESENT_ATTR = 0x0045,
	ZCL_ANALOG_OUT_OF_SERVICE_ATTR = 0x0051,
	ZCL_ANALOG_PRESENT_VALUE_ATTR = 0x0055,
	ZCL_ANALOG_RELINQUISH_DEFAULT_ATTR = 0x006A,
	ZCL_ANALOG_RESOLUTION_ATTR = 0x006A,
	ZCL_ANALOG_STATUS_FLAGS_ATTR = 0x006F,
	ZCL_ANALOG_ENGINEERING_UNITS_ATTR = 0x0075,
	ZCL_ANALOG_APPLICATION_TYPE_ATTR = 0x0100,
};

/*
 * @struct Attribute access callbacks
 * */
struct zcl_analog_attr_callbacks_t {
    enum ZclStatusCodeT (*read)(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info, void *arg);
    enum ZclStatusCodeT (*write)(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info, void *arg);
};

/*
 * @struct Analog cluster data
 * */
struct zcl_analog_server_cluster_t {
    struct ZbZclClusterT cluster;
    struct zcl_analog_attr_callbacks_t callbacks;
    void *arg;
    float *valuePtr;
};


struct ZbZclClusterT * ZbZcl_Analog_ServerAlloc(struct ZigBeeT *zb, uint8_t endpoint, enum analog_mode mode,
		float *valuePtr, struct zcl_analog_attr_callbacks_t *callbacks, void *arg);

float ZbZclAttrFloatRead(struct ZbZclClusterT *cluster, uint16_t attributeId,
    enum ZclDataTypeT *typePtr, enum ZclStatusCodeT *statusPtr);

enum ZclStatusCodeT ZbZclAttrFloatWrite(struct ZbZclClusterT *cluster, uint16_t attributeId, float value);

#endif /* ANALOG_H_ */
