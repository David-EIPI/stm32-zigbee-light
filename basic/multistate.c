#include "../basic/multistate.h"

#include "app_common.h"
#include "app_zigbee.h"
#include "dbg_trace.h"
#include "stm_logging.h"

/*
 * Multistate value cluster: server commands and attributes.
 * Implementation is based on the custom long-string cluster from the examples.
 * */

/**
 * @brief  Provide a simple callback to read and update state text attribute.
 *
 * @param  clusterPtr : Cluster data previously allocated using @ref ZbZcl_Analog_ServerAlloc
 * @param  info: Attribute description
 * @retval ZCL Status code
 */
enum ZclStatusCodeT msv_StateText_CB(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info)
{
    struct zcl_msv_server_cluster_t *cluster = (struct zcl_msv_server_cluster_t *)clusterPtr;
	uint16_t attributeId = info->info->attributeId;
	uint8_t *inputData = info->zcl_data;
	unsigned int len = info->zcl_len;
	void *attrData = info->attr_data;

	if (ZCL_MULTISTATE_STATE_TEXT_ATTR == attributeId) {


		if (info->type == ZCL_ATTR_CB_TYPE_WRITE) {

			if (len > cluster->state_text_len)
				return ZCL_STATUS_FAILURE;

			memcpy(cluster->state_text_buffer, inputData, len);

			if (attrData)
				memcpy(attrData, cluster->state_text_buffer, len);

		} else if (info->type == ZCL_ATTR_CB_TYPE_READ) {

			if (len < cluster->state_text_len)
				return ZCL_STATUS_FAILURE;

			memcpy(inputData, cluster->state_text_buffer, cluster->state_text_len);

		} else {
			return ZCL_STATUS_UNSUPP_ATTRIBUTE;//ZCL_STATUS_FAILURE;
		}
	}

	return ZB_STATUS_SUCCESS;
}

enum {
	MSV_STATE_TEXT_IDX,
	MSV_PRESENT_VALUE_IDX,
};

/*
 * @var    Attributes added to the newly allocated cluster.
 */
static struct ZbZclAttrT msvAttrList[] = {
	[MSV_STATE_TEXT_IDX] = {
		ZCL_MULTISTATE_STATE_TEXT_ATTR, ZCL_DATATYPE_ARRAY, ZCL_ATTR_FLAG_CB_READ | ZCL_ATTR_FLAG_CB_WRITE,
			0, msv_StateText_CB, {0, 0}, {0, 0},
	},
	[MSV_PRESENT_VALUE_IDX] = {
		ZCL_MULTISTATE_PRESENT_VALUE_ATTR, ZCL_DATATYPE_UNSIGNED_16BIT, ZCL_ATTR_FLAG_WRITABLE | ZCL_ATTR_FLAG_PERSISTABLE | ZCL_ATTR_FLAG_REPORTABLE,
		sizeof(uint16_t), NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_MULTISTATE_DESCRIPTION_ATTR, ZCL_DATATYPE_STRING_CHARACTER, 0, 16, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_MULTISTATE_NUMBER_OF_STATES_ATTR, ZCL_DATATYPE_UNSIGNED_16BIT, ZCL_ATTR_FLAG_WRITABLE, sizeof(uint16_t), NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_MULTISTATE_OUT_OF_SERVICE_ATTR, ZCL_DATATYPE_BOOLEAN, 0, 0, NULL, {0, 0}, {0, 0},
	},
	{
		ZCL_MULTISTATE_STATUS_FLAGS_ATTR, ZCL_DATATYPE_BITMAP_8BIT, 0, 0, NULL, {0, 0}, {0, 0},
	},
};

/**
 * @brief  Allocate a new Multistate Input/Analog Output/Analog Value cluster.
 *
 * @param  zb: Zigbee data pointer obtained from ZbInit
 * @param  endpoint: Endpoint id
 * @param  mode: Cluster mode constant
 * @param  valuePtr: Present value attribute storage
 * @param  numitems: Number of state items
 * @param  itemtext: Names of state items
 * @param  callbacks: Present value attribute access callbacks
 * @param  arg: User-defined callback argument
 * @retval ZCL Status code
 */
struct ZbZclClusterT * ZbZcl_msv_ServerAlloc(struct ZigBeeT *zb, uint8_t endpoint, enum multistate_mode mode, unsigned numitems,
		const char **itemtext, struct zcl_msv_attr_callbacks_t *callbacks, void *arg)
{
    struct zcl_msv_server_cluster_t *clusterPtr;


	unsigned totalsize = 1 + 2; /* space for data type and element count */
    if (itemtext != NULL){
    	unsigned i;
    	for (i = 0; i < numitems; i++) {
    		int sz = 1 + strlen(itemtext[i]); /* adding space string length */
    		if (sz > 16) sz = 16;
    		totalsize += sz;
    	}
    	msvAttrList[MSV_STATE_TEXT_IDX].customValSz = totalsize;
    }

    clusterPtr = ZbZclClusterAlloc(zb, sizeof(struct zcl_msv_server_cluster_t) + totalsize,\
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

    if (callbacks != NULL && callbacks->msv_presentValue != NULL) {
    	msvAttrList[MSV_PRESENT_VALUE_IDX].callback = callbacks->msv_presentValue;
    	msvAttrList[MSV_PRESENT_VALUE_IDX].flags |= ZCL_ATTR_FLAG_CB_NOTIFY;
    }

    enum ZclStatusCodeT result;

    unsigned msv_attr_count = ZCL_ATTR_LIST_LEN(msvAttrList);
    result = ZbZclAttrAppendList(&clusterPtr->cluster, msvAttrList, msv_attr_count);
    if (ZCL_STATUS_SUCCESS != result) {
        ZbZclClusterFree(&clusterPtr->cluster);
        return NULL;
    }

	clusterPtr->state_text_len = totalsize;

	(void)ZbZclAttrWrite(&clusterPtr->cluster, NULL, ZCL_MULTISTATE_OUT_OF_SERVICE_ATTR, zcl_attr_str_short_zero, 1, ZCL_ATTR_WRITE_FLAG_NORMAL);
	result = ZbZclAttrIntegerWrite(&clusterPtr->cluster, ZCL_MULTISTATE_NUMBER_OF_STATES_ATTR, numitems);
    if (ZCL_STATUS_SUCCESS == result) {
    	uint8_t * buffer = malloc(totalsize);
    	if (buffer != NULL) {
    		unsigned i;
    		uint8_t *ptr = buffer;
    		*(uint8_t *)ptr = ZCL_DATATYPE_STRING_CHARACTER;
    		ptr += sizeof(uint8_t);
    		*(uint16_t *)ptr = numitems;
    		ptr += sizeof(uint16_t);
    		for (i = 0; i < numitems; i++) {
    			int sz = strlen(itemtext[i]);
    			if (sz > 15) sz = 15;
    			ptr[0] = sz;
    			memcpy(ptr + 1, itemtext[i], sz);
    			ptr += sz + 1;
    		}
    		memcpy(&clusterPtr->state_text_buffer, buffer, totalsize);
   			result = ZbZclAttrWrite(&clusterPtr->cluster, NULL, ZCL_MULTISTATE_STATE_TEXT_ATTR, buffer, totalsize, ZCL_ATTR_WRITE_FLAG_FORCE);
   			if (ZCL_STATUS_SUCCESS == result) {
   				(void)ZbZclAttrIntegerWrite(&clusterPtr->cluster, ZCL_GLOBAL_ATTR_CLUSTER_REV, 1);
   				enum ZclDataTypeT atype = 0;
   	   			result = ZbZclAttrRead(&clusterPtr->cluster, ZCL_MULTISTATE_STATE_TEXT_ATTR, &atype, buffer, totalsize, 0);
   	   			if (ZCL_STATUS_SUCCESS == result) {
   	   				APP_DBG("State text type is %d:", atype);
   	   			}
    		}
    		free(buffer);
    	}

    }

    ZbZclClusterAttach(&clusterPtr->cluster);
    return &clusterPtr->cluster;
}
