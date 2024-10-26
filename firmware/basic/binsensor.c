/**
  ******************************************************************************
  * @file           : bitsensor.c
  * @author         : David Shirvanyants
  * @date           : Oct 1, 2024
  * @brief          : Implements basic Binary{Input,Output,Value} clusters.
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "../basic/binsensor.h"

#include "app_common.h"
#include "app_zigbee.h"
#include "dbg_trace.h"
#include "stm_logging.h"


/*
 * Binary value cluster: server commands and attributes.
 * */

/**
 * @brief  Provide a simple callback to only read and update the value in memory.
 *
 * @param  clusterPtr : Cluster data previously allocated using @ref ZbZcl_BinaryValue_ServerAlloc
 * @param  info: Attribute description
 * @retval ZCL Status code
 */
enum ZclStatusCodeT simple_BinaryValue_callback(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info)
{
    struct zcl_binary_server_cluster_t *cluster = (struct zcl_binary_server_cluster_t *)clusterPtr;
	uint16_t attributeId = info->info->attributeId;
	uint8_t *inputData = info->zcl_data;
	unsigned int len = info->zcl_len;
	void *attrData = info->attr_data;

	enum ZclStatusCodeT result = ZCL_STATUS_UNSUPP_ATTRIBUTE; //ZB_STATUS_SUCCESS;
	if (ZCL_BINARY_PRESENT_VALUE_ATTR == attributeId) {
		if (len > sizeof(*(cluster->valuePtr)))
				len = sizeof(*(cluster->valuePtr));

		if (info->type == ZCL_ATTR_CB_TYPE_WRITE) {

			if (NULL == cluster->valuePtr || (len == 0))
				result = ZCL_STATUS_FAILURE;
			else {
				memcpy(cluster->valuePtr, inputData, len);
				result = ZCL_STATUS_SUCCESS;
			}

			if (result == ZCL_STATUS_SUCCESS && NULL != cluster->callbacks.write)
				result = cluster->callbacks.write(clusterPtr, info, cluster->arg);

			if (ZCL_STATUS_SUCCESS == result && attrData && len <= sizeof(uint8_t))
				memcpy(attrData, inputData, len);

		} else if (info->type == ZCL_ATTR_CB_TYPE_READ) {

			if (NULL == cluster->valuePtr || (len == 0))
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
	BINARY_PRESENT_VALUE_IDX,
};

/*
 * @var    Attributes added to the newly allocated cluster.
 */
static struct ZbZclAttrT binaryAttrList[] = {
	[BINARY_PRESENT_VALUE_IDX] = {
		ZCL_BINARY_PRESENT_VALUE_ATTR, ZCL_DATATYPE_BOOLEAN,
		ZCL_ATTR_FLAG_WRITABLE | ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_CB_READ | ZCL_ATTR_FLAG_CB_WRITE,
		sizeof(uint8_t), simple_BinaryValue_callback, {0, 0}, {0, 0},
	},
	{
		ZCL_BINARY_ACTIVE_TEXT_ATTR, ZCL_DATATYPE_STRING_CHARACTER, 0, 16, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_BINARY_DESCRIPTION_ATTR, ZCL_DATATYPE_STRING_CHARACTER, 0, 16, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_BINARY_INACTIVE_TEXT_ATTR, ZCL_DATATYPE_STRING_CHARACTER, 0, 16, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_BINARY_POLARITY_ATTR, ZCL_DATATYPE_ENUMERATION_8BIT, 0, 1, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_BINARY_OUT_OF_SERVICE_ATTR, ZCL_DATATYPE_BOOLEAN, 0, 0, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_BINARY_STATUS_FLAGS_ATTR, ZCL_DATATYPE_BITMAP_8BIT, 0, 0, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_BINARY_APPLICATION_TYPE_ATTR, ZCL_DATATYPE_UNSIGNED_32BIT, 0, 0, NULL, {0, 0}, {0, 0},
	},
};

/**
 * @brief  Allocate a new Binary Input/Binary Output/Binary Value cluster.
 *
 * @param  zb: Zigbee data pointer obtained from ZbInit
 * @param  endpoint: Endpoint id
 * @param  mode: Cluster mode constant
 * @param  valuePtr: Present value attribute storage
 * @param  callbacks: Present value attribute access callbacks
 * @param  arg: User-defined callback argument
 * @retval ZCL Status code
 */
struct ZbZclClusterT * ZbZcl_Binary_ServerAlloc(struct ZigBeeT *zb, uint8_t endpoint, enum binary_sensor_mode mode,
		uint8_t *valuePtr, struct zcl_binary_attr_callbacks_t *callbacks, void *arg)
{
    struct zcl_binary_server_cluster_t *clusterPtr;

    clusterPtr = ZbZclClusterAlloc(zb, sizeof(struct zcl_binary_server_cluster_t),\
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

    /* Sensor readings need not be persistable */
    if (mode == BINARY_SENSOR_INPUT)
    	binaryAttrList[BINARY_PRESENT_VALUE_IDX].flags &=  ~ZCL_ATTR_FLAG_PERSISTABLE;
    else
    	binaryAttrList[BINARY_PRESENT_VALUE_IDX].flags |=  ZCL_ATTR_FLAG_PERSISTABLE;

    unsigned binary_attr_count = ZCL_ATTR_LIST_LEN(binaryAttrList);
    result = ZbZclAttrAppendList(&clusterPtr->cluster, binaryAttrList, binary_attr_count);
    if (ZCL_STATUS_SUCCESS != result) {
        ZbZclClusterFree(&clusterPtr->cluster);
        return NULL;
    }

	(void)ZbZclAttrWrite(&clusterPtr->cluster, NULL, ZCL_BINARY_OUT_OF_SERVICE_ATTR, zcl_attr_str_short_zero, 1, ZCL_ATTR_WRITE_FLAG_NORMAL);
	(void)ZbZclAttrIntegerWrite(&clusterPtr->cluster, ZCL_GLOBAL_ATTR_CLUSTER_REV, 1);

	if (valuePtr)
		(void)ZbZclAttrWrite(&clusterPtr->cluster, NULL, ZCL_BINARY_PRESENT_VALUE_ATTR, valuePtr, 1, ZCL_ATTR_WRITE_FLAG_NORMAL);

    ZbZclClusterAttach(&clusterPtr->cluster);
    return &clusterPtr->cluster;
}

