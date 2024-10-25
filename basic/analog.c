/**
  ******************************************************************************
  * @file           : analog.c
  * @author         : David Shirvanyants
  * @date           : Sep 23, 2024
  * @brief          : Implements basic Analog{Input,Output,Value} clusters.
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "../basic/analog.h"

#include "app_common.h"
#include "app_zigbee.h"
#include "dbg_trace.h"
#include "stm_logging.h"

/**
 * @brief  Provide a simple callback to only read and update the value in memory.
 *
 * @param  clusterPtr : Cluster data previously allocated using @ref ZbZcl_Analog_ServerAlloc
 * @param  info: Attribute description
 * @retval ZCL Status code
 */
enum ZclStatusCodeT simple_AnalogValue_callback(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info)
{
    struct zcl_analog_server_cluster_t *cluster = (struct zcl_analog_server_cluster_t *)clusterPtr;
	uint16_t attributeId = info->info->attributeId;
	uint8_t *inputData = info->zcl_data;
	unsigned int len = info->zcl_len;
	void *attrData = info->attr_data;

	enum ZclStatusCodeT result = ZCL_STATUS_UNSUPP_ATTRIBUTE; //ZB_STATUS_SUCCESS;
	if (ZCL_ANALOG_PRESENT_VALUE_ATTR == attributeId) {

		if (info->type == ZCL_ATTR_CB_TYPE_WRITE) {

			if (NULL == cluster->valuePtr || len > sizeof(*(cluster->valuePtr)))
				result = ZCL_STATUS_FAILURE;
			else {
				memcpy(cluster->valuePtr, inputData, len);
				result = ZCL_STATUS_SUCCESS;
			}

			if (result == ZCL_STATUS_SUCCESS && NULL != cluster->callbacks.write)
				result = cluster->callbacks.write(clusterPtr, info, cluster->arg);

			if (ZCL_STATUS_SUCCESS == result && attrData/* && len <= sizeof(float)*/) {
				memcpy(attrData, inputData, len);
			}

		} else if (info->type == ZCL_ATTR_CB_TYPE_READ) {

			if (NULL == cluster->valuePtr || len < sizeof(*(cluster->valuePtr)))
				result = ZCL_STATUS_FAILURE;
			else {
				memcpy(inputData, cluster->valuePtr, sizeof(*(cluster->valuePtr)));
				result = ZCL_STATUS_SUCCESS;
			}

			if (result == ZCL_STATUS_SUCCESS && NULL != cluster->callbacks.read)
				result = cluster->callbacks.read(clusterPtr, info, cluster->arg);
		} else {
			return ZCL_STATUS_UNSUPP_ATTRIBUTE;//ZCL_STATUS_FAILURE;
		}
	}

	return result;
}

enum {
	ANALOG_PRESENT_VALUE_IDX,
};


/*
 * @var    Attributes added to the newly allocated cluster.
 */
static struct ZbZclAttrT analogAttrList[] = {
	[ANALOG_PRESENT_VALUE_IDX] = {
		ZCL_ANALOG_PRESENT_VALUE_ATTR, ZCL_DATATYPE_FLOATING_SINGLE,
		ZCL_ATTR_FLAG_WRITABLE | ZCL_ATTR_FLAG_PERSISTABLE | ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_CB_WRITE,
		sizeof(float), simple_AnalogValue_callback, {0, 0}, {0, 0},
	},
	{
		ZCL_ANALOG_DESCRIPTION_ATTR, ZCL_DATATYPE_STRING_CHARACTER, 0, 16, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_ANALOG_MAX_PRESENT_ATTR, ZCL_DATATYPE_FLOATING_SINGLE, 0, sizeof(float), NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_ANALOG_MIN_PRESENT_ATTR, ZCL_DATATYPE_FLOATING_SINGLE, 0, sizeof(float), NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_ANALOG_OUT_OF_SERVICE_ATTR, ZCL_DATATYPE_BOOLEAN, 0, 0, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_ANALOG_RESOLUTION_ATTR, ZCL_DATATYPE_FLOATING_SINGLE, 0, sizeof(float), NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_ANALOG_STATUS_FLAGS_ATTR, ZCL_DATATYPE_BITMAP_8BIT, 0, 0, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_ANALOG_ENGINEERING_UNITS_ATTR, ZCL_DATATYPE_ENUMERATION_16BIT, 0, 0, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_ANALOG_APPLICATION_TYPE_ATTR, ZCL_DATATYPE_UNSIGNED_32BIT, 0, 0, NULL, {0, 0}, {0, 0},
	},
};


/**
 * @brief  Allocate a new Analog Input/Analog Output/Analog Value cluster.
 *
 * @param  zb: Zigbee data pointer obtained from ZbInit
 * @param  endpoint: Endpoint id
 * @param  mode: Cluster mode constant
 * @param  valuePtr: Present value attribute storage
 * @param  callbacks: Present value attribute access callbacks
 * @param  arg: User-defined callback argument
 * @retval ZCL Status code
 */
struct ZbZclClusterT * ZbZcl_Analog_ServerAlloc(struct ZigBeeT *zb, uint8_t endpoint,enum analog_mode mode,
		float *valuePtr, struct zcl_analog_attr_callbacks_t *callbacks, void *arg)
{
    struct zcl_analog_server_cluster_t *clusterPtr;

    clusterPtr = ZbZclClusterAlloc(zb, sizeof(struct zcl_analog_server_cluster_t),\
            mode, endpoint, ZCL_DIRECTION_TO_SERVER);

    if (clusterPtr == NULL) {
        return NULL;
    }

    clusterPtr->cluster.txOptions |= ZB_APSDE_DATAREQ_TXOPTIONS_SECURITY;

    memset(&clusterPtr->callbacks, 0, sizeof(clusterPtr->callbacks));
    if (callbacks != NULL) {
        memcpy(&clusterPtr->callbacks, callbacks, sizeof(clusterPtr->callbacks));
    }

    clusterPtr->arg = arg;
    clusterPtr->valuePtr = valuePtr;

    enum ZclStatusCodeT result;

    unsigned analog_attr_count = ZCL_ATTR_LIST_LEN(analogAttrList);
    result = ZbZclAttrAppendList(&clusterPtr->cluster, analogAttrList, analog_attr_count);
    if (ZCL_STATUS_SUCCESS != result) {
        ZbZclClusterFree(&clusterPtr->cluster);
        return NULL;
    }

	(void)ZbZclAttrWrite(&clusterPtr->cluster, NULL, ZCL_ANALOG_OUT_OF_SERVICE_ATTR, zcl_attr_str_short_zero, 1, ZCL_ATTR_WRITE_FLAG_NORMAL);
	(void)ZbZclAttrIntegerWrite(&clusterPtr->cluster, ZCL_GLOBAL_ATTR_CLUSTER_REV, 1);

    ZbZclClusterAttach(&clusterPtr->cluster);
    return &clusterPtr->cluster;
}

/**
 * @brief  Helper function to read a floating point attribute.
 *
 * @param  cluster: ZCL cluster descriptor
 * @param  attributeId: Attribute id
 * @param  typePtr: If not Null will receive the attribute data type
 * @param  statusPtr: If not Null will receive the ZCL status code
 * @retval Attribute value on success or 0.0f if failed to read
 */
float ZbZclAttrFloatRead(struct ZbZclClusterT *cluster, uint16_t attributeId,
    enum ZclDataTypeT *typePtr, enum ZclStatusCodeT *statusPtr)
{
	static float value = 0.0f;

	enum ZclStatusCodeT result = ZbZclAttrRead(cluster, attributeId, typePtr,
	    &value, sizeof(value), false);

	if (statusPtr != NULL)
		*statusPtr = result;

	return value;
}

/**
 * @brief  Helper function to write a floating point attribute.
 *
 * @param  cluster: ZCL cluster descriptor
 * @param  attributeId: Attribute id
 * @param  value: New value
 * @retval ZCL Status code
 */
enum ZclStatusCodeT ZbZclAttrFloatWrite(struct ZbZclClusterT *cluster, uint16_t attributeId, float value)
{
	static float tmp = 0.0f;
	tmp = value;

	enum ZclStatusCodeT result =
		ZbZclAttrWrite(cluster, NULL, attributeId, (uint8_t*)&tmp, sizeof(value), ZCL_ATTR_WRITE_FLAG_FORCE);

	return result;
}
