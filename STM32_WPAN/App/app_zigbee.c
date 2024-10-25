
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/app_zigbee.c
  * @author  MCD Application Team
  * @author  David Shirvanyants
  * @brief   Zigbee Application. Implements outdoor light controller.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "app_entry.h"
#include "dbg_trace.h"
#include "app_zigbee.h"
#include "zigbee_interface.h"
#include "shci.h"
#include "stm_logging.h"
#include "app_conf.h"
#include "stm32wbxx_core_interface_def.h"
#include "zigbee_types.h"
#include "stm32_seq.h"

/* Private includes -----------------------------------------------------------*/
#include <assert.h>
#include "zcl/zcl.h"
#include "zcl/general/zcl.temp.meas.h"
#include "zcl/general/zcl.illum.meas.h"

/* USER CODE BEGIN Includes */
#include "zcl.basic.h"
#include "main.h"
#include "hw_flash.h"
#include "ee.h"

#include "main.h"

#include "../../basic/analog.h"
#include "../../basic/binsensor.h"
#include "../../basic/multistate.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define APP_ZIGBEE_STARTUP_FAIL_DELAY               500U
#define CHANNEL                                     25
#define ZED_SLEEP_TIME_30S                           1 /* 30s sleep time unit */

#define SW1_ENDPOINT                                1
#define SW2_ENDPOINT                                2
#define SW3_ENDPOINT                                3
#define SW4_ENDPOINT                                4
#define SW5_ENDPOINT                                5
#define SW6_ENDPOINT                                6

/* Temperature_meas (endpoint 1) specific defines ------------------------------------------------*/
#define TEMP_MIN_1                      -2000
#define TEMP_MAX_1                      5000
#define TEMP_TOLERANCE_1                      200
/* USER CODE BEGIN Temperature_meas (endpoint 1) defines */
/* USER CODE END Temperature_meas (endpoint 1) defines */

/* Illuminance_meas (endpoint 1) specific defines ------------------------------------------------*/
#define ILLUM_MIN_1                      1
#define ILLUM_MAX_1                      65534
/* USER CODE BEGIN Illuminance_meas (endpoint 1) defines */
/* USER CODE END Illuminance_meas (endpoint 1) defines */

/* USER CODE BEGIN PD */
#define CFG_NVM                 1U /* use FLASH */
#define RESET_ZB_NWK            0U

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Neutralize the auto-generated macro.
 * When active, this flag caps RX in idle state making Zigbee device slow to respond. */

#undef MCP_ASSOC_CAP_RXONIDLE
#define MCP_ASSOC_CAP_RXONIDLE 0

/*
 * Replacing the auto-generated macro with the hack below expands the channel list to all 2.4GHz channels.
 *  */
#undef CHANNEL
#define CHANNEL  1 ? (((1 << (26-11+1))-1) << 11) : 0
/* USER CODE END PM */

/* External definition -------------------------------------------------------*/
enum ZbStatusCodeT ZbStartupWait(struct ZigBeeT *zb, struct ZbStartupT *config);

/* USER CODE BEGIN ED */
/* USER CODE END ED */

/* Private function prototypes -----------------------------------------------*/
static void APP_ZIGBEE_StackLayersInit(void);
static void APP_ZIGBEE_ConfigEndpoints(void);
static void APP_ZIGBEE_NwkForm(void);

static void APP_ZIGBEE_TraceError(const char *pMess, uint32_t ErrCode);
static void APP_ZIGBEE_CheckWirelessFirmwareInfo(void);

static void Wait_Getting_Ack_From_M0(void);
static void Receive_Ack_From_M0(void);
static void Receive_Notification_From_M0(void);

static void APP_ZIGBEE_ProcessNotifyM0ToM4(void);
static void APP_ZIGBEE_ProcessRequestM0ToM4(void);

/* USER CODE BEGIN PFP */

enum ZclStatusCodeT multistate_presentValue_CB(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info);
enum ZclStatusCodeT analog_presentValue_CB(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info, void *arg);

static void APP_ZIGBEE_ConfigBasic(void);
void set_default_attr_values(void);

static bool APP_ZIGBEE_persist_load(void);
static bool APP_ZIGBEE_persist_save(void);
static void APP_ZIGBEE_persist_delete(void);
static void APP_ZIGBEE_persist_notify_cb(struct ZigBeeT *zb, void *cbarg);
static enum ZbStatusCodeT APP_ZIGBEE_ZbStartupPersist(struct ZigBeeT *zb);
static void APP_ZIGBEE_PersistCompleted_callback(enum ZbStatusCodeT status,void *arg);


#ifdef CFG_NVM
static void APP_ZIGBEE_NVM_Init(void);
static bool APP_ZIGBEE_NVM_Read(void);
static bool APP_ZIGBEE_NVM_Write(void);
static void APP_ZIGBEE_NVM_Erase(void);
#endif /* CFG_NVM */
/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
static TL_CmdPacket_t   *p_ZIGBEE_otcmdbuffer;
static TL_EvtPacket_t   *p_ZIGBEE_notif_M0_to_M4;
static TL_EvtPacket_t   *p_ZIGBEE_request_M0_to_M4;
static __IO uint32_t    CptReceiveNotifyFromM0 = 0;
static __IO uint32_t    CptReceiveRequestFromM0 = 0;

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_ZIGBEE_Config_t ZigbeeConfigBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ZigbeeOtCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ZigbeeNotifRspEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ZigbeeNotifRequestBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];

struct zigbee_app_info
{
  bool has_init;
  struct ZigBeeT *zb;
  enum ZbStartType startupControl;
  enum ZbStatusCodeT join_status;
  uint32_t join_delay;
  bool init_after_join;

  struct ZbZclClusterT *temperature_meas_server_1;
  struct ZbZclClusterT *illuminance_meas_server_1;
};
static struct zigbee_app_info zigbee_app_info;

/* USER CODE BEGIN PV */

/* NVM variables */

static unsigned int persistNumWrites = 0;

/* cache in uninit RAM to store/retrieve persistent data */
union cache
{
  uint8_t  U8_data[ST_PERSIST_MAX_ALLOC_SZ];     // in bytes
  uint32_t U32_data[ST_PERSIST_MAX_ALLOC_SZ/4U]; // in U32 words
};
// __attribute__ ((section(".noinit"))) union cache cache_persistent_data;

// __attribute__ ((section(".noinit"))) union cache cache_diag_reference;

union cache cache_persistent_data;
union cache cache_diag_reference;


struct ZbZclClusterT * analog_clusters[SW6_ENDPOINT];
struct ZbZclClusterT * multistate_cluster;
struct ZbZclClusterT * binary_cluster_light_on;
struct ZbZclClusterT * binary_cluster_motion_detect;
struct ZbZclClusterT * binary_cluster_dusk_detect;

const char *opmode_state_text[] = {
		"Auto",
		"Full",
		"Dim",
		"Off"
};

struct analog_cluster_attr {
	float value;
	float minPresent, maxPresent;
	float step;
	char desc[16];
	enum analog_mode mode;
	uint16_t engUnit;
	uint16_t icon;
	int32_t *extSetting;
	void (*ext_setting_cb)(int32_t *value);
} analog_cluster_attrs[] = {
		{
				.mode = ANALOG_OUTPUT,
				.desc = "\x09" "Dim level",
				.value = 50.0,
				.minPresent = 1.0,
				.maxPresent = 100.0,
				.step = 1.0,
				.engUnit = ZCL_ENGINEERING_UNIT_PERCENT,
				.icon = ZCL_ANALOG_ICON_BRIGHTNESS_PERCENT,
				.extSetting = &dimLevel,
		},
		{
				.mode = ANALOG_OUTPUT,
				.desc = "\x0a" "Dusk level",
				.value = 80.0,
				.minPresent = 1.0,
				.maxPresent = 200.0,
				.step = 1.0,
				.engUnit = ZCL_ENGINEERING_UNIT_PERCENT,
				.icon = ZCL_ANALOG_ICON_BRIGHTNESS_PERCENT,
				.extSetting = &ambientThreshold,
		},
		{
				.mode = ANALOG_OUTPUT,
				.desc = "\x0d" "Motion detect",
				.value = 80.0,
				.minPresent = 0.0,
				.maxPresent = 100.0,
				.step = 1.0,
				.engUnit = ZCL_ENGINEERING_UNIT_PERCENT,
				.icon = ZCL_ANALOG_ICON_PERCENT,
				.extSetting = &motionSensitivity,
				.ext_setting_cb = update_motion_sensitivity,
		},
		{
				.mode = ANALOG_OUTPUT,
				.desc = "\x0c" "After motion",
				.value = 300.0,
				.minPresent = 0.0,
				.maxPresent = 2*3600.0,
				.step = 1.0,
				.engUnit = ZCL_ENGINEERING_UNIT_SECONDS,
				.icon = ZCL_ANALOG_ICON_TIMER,
				.extSetting = &motionLight,
		},
		{
				.mode = ANALOG_OUTPUT,
				.desc = "\x0c" "After dusk",
				.value = 0.0,
				.minPresent = 1.0,
				.maxPresent = 12*60.0,
				.step = 1.0,
				.engUnit = ZCL_ENGINEERING_UNIT_MINUTES,
				.icon = ZCL_ANALOG_ICON_TIMER,
				.extSetting = &nightLight,
		},
		{
				.mode = ANALOG_OUTPUT,
				.desc = "\x0d" "Phase correct",
				.value = 0.0,
				.minPresent = -10.0,
				.maxPresent = 10,
				.step = 0.001,
				.engUnit = ZCL_ENGINEERING_UNIT_MILLISECONDS,
				.icon = ZCL_ANALOG_ICON_TIMER,
				.extSetting = &phase_correction,
		},
};

/* Stored attributes' values */
static uint8_t lightState = 0;
static uint8_t motionState = 0;
static uint8_t duskState = 0;

/* Data block for NVM storage */
struct persistence_attrs {
	uint32_t struct_len;
	float analogValues[lengthof(analog_cluster_attrs)];
	uint32_t multistateValue;
};



/* USER CODE END PV */
/* Functions Definition ------------------------------------------------------*/

/**
 * @brief  Zigbee application initialization
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_Init(void)
{
  SHCI_CmdStatus_t ZigbeeInitStatus;

  APP_DBG("APP_ZIGBEE_Init");

  /* Check the compatibility with the Coprocessor Wireless Firmware loaded */
  APP_ZIGBEE_CheckWirelessFirmwareInfo();

  /* Register cmdbuffer */
  APP_ZIGBEE_RegisterCmdBuffer(&ZigbeeOtCmdBuffer);

  /* Init config buffer and call TL_ZIGBEE_Init */
  APP_ZIGBEE_TL_INIT();

  /* Register task */
  /* Create the different tasks */
  UTIL_SEQ_RegTask(1U << (uint32_t)CFG_TASK_NOTIFY_FROM_M0_TO_M4, UTIL_SEQ_RFU, APP_ZIGBEE_ProcessNotifyM0ToM4);
  UTIL_SEQ_RegTask(1U << (uint32_t)CFG_TASK_REQUEST_FROM_M0_TO_M4, UTIL_SEQ_RFU, APP_ZIGBEE_ProcessRequestM0ToM4);

  /* Task associated with network creation process */
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, UTIL_SEQ_RFU, APP_ZIGBEE_NwkForm);

  /* USER CODE BEGIN APP_ZIGBEE_INIT */
  /* NVM Init */
#if CFG_NVM
  APP_ZIGBEE_NVM_Init();
#endif

  /* USER CODE END APP_ZIGBEE_INIT */

  /* Start the Zigbee on the CPU2 side */
  ZigbeeInitStatus = SHCI_C2_ZIGBEE_Init();
  /* Prevent unused argument(s) compilation warning */
  UNUSED(ZigbeeInitStatus);

  /* Initialize Zigbee stack layers */
  APP_ZIGBEE_StackLayersInit();

}

/**
 * @brief  Initialize Zigbee stack layers
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_StackLayersInit(void)
{
  APP_DBG("APP_ZIGBEE_StackLayersInit");

  zigbee_app_info.zb = ZbInit(0U, NULL, NULL);
  assert(zigbee_app_info.zb != NULL);

  /* Create the endpoint and cluster(s) */
  APP_ZIGBEE_ConfigEndpoints();

  /* USER CODE BEGIN APP_ZIGBEE_StackLayersInit */
  /* Assign default attribute values */
//  set_default_attr_values();

  /* STEP 1 - TRY to START FROM PERSISTENCE */
  enum ZbStatusCodeT status;

  /* First we disable the persistent notification */
  ZbPersistNotifyRegister(zigbee_app_info.zb,NULL,NULL);

  /* Call a startup from persistence */
#if CFG_NVM
#if (RESET_ZB_NWK)
  APP_ZIGBEE_persist_delete();
#else
  status = APP_ZIGBEE_ZbStartupPersist(zigbee_app_info.zb);
  if(status == ZB_STATUS_SUCCESS)
  {
     /* no fresh startup need anymore */
     APP_DBG("ZbStartupPersist: SUCCESS, restarted from persistence");

     zigbee_app_info.join_delay = HAL_GetTick(); /* now */
     zigbee_app_info.startupControl = ZbStartTypeJoin;
     zigbee_app_info.join_status = ZB_STATUS_SUCCESS;
     return;
  }
  else
  {
       /* Start-up form persistence failed perform a fresh ZbStartup */
       APP_DBG("ZbStartupPersist: FAILED to restart from persistence with status: 0x%02x",status);
  }
#endif
#endif
  /* USER CODE END APP_ZIGBEE_StackLayersInit */

  /* Configure the joining parameters */
  zigbee_app_info.join_status = (enum ZbStatusCodeT) 0x01; /* init to error status */
  zigbee_app_info.join_delay = HAL_GetTick(); /* now */
  zigbee_app_info.startupControl = ZbStartTypeJoin;

  /* Initialization Complete */
  zigbee_app_info.has_init = true;

  /* run the task */
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
}

/**
 * @brief  Configure Zigbee application endpoints
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ConfigEndpoints(void)
{
  struct ZbApsmeAddEndpointReqT req;
  struct ZbApsmeAddEndpointConfT conf;

  memset(&req, 0, sizeof(req));

  /* Endpoint: SW1_ENDPOINT */
  req.profileId = ZCL_PROFILE_HOME_AUTOMATION;
  req.deviceId = ZCL_DEVICE_LEVEL_OUTPUT;
  req.endpoint = SW1_ENDPOINT;
  ZbZclAddEndpoint(zigbee_app_info.zb, &req, &conf);
  assert(conf.status == ZB_STATUS_SUCCESS);

  /* Temperature meas server */
  zigbee_app_info.temperature_meas_server_1 = ZbZclTempMeasServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, TEMP_MIN_1, TEMP_MAX_1, TEMP_TOLERANCE_1);
  assert(zigbee_app_info.temperature_meas_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.temperature_meas_server_1);
  /* Illuminance meas server */
  zigbee_app_info.illuminance_meas_server_1 = ZbZclIllumMeasServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, ILLUM_MIN_1, ILLUM_MAX_1);
  assert(zigbee_app_info.illuminance_meas_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.illuminance_meas_server_1);
  /* Endpoint: SW2_ENDPOINT */
  req.profileId = ZCL_PROFILE_HOME_AUTOMATION;
  req.deviceId = ZCL_DEVICE_LEVEL_OUTPUT;
  req.endpoint = SW2_ENDPOINT;
  ZbZclAddEndpoint(zigbee_app_info.zb, &req, &conf);
  assert(conf.status == ZB_STATUS_SUCCESS);

  /* Endpoint: SW3_ENDPOINT */
  req.profileId = ZCL_PROFILE_HOME_AUTOMATION;
  req.deviceId = ZCL_DEVICE_LEVEL_OUTPUT;
  req.endpoint = SW3_ENDPOINT;
  ZbZclAddEndpoint(zigbee_app_info.zb, &req, &conf);
  assert(conf.status == ZB_STATUS_SUCCESS);

  /* Endpoint: SW4_ENDPOINT */
  req.profileId = ZCL_PROFILE_HOME_AUTOMATION;
  req.deviceId = ZCL_DEVICE_LEVEL_OUTPUT;
  req.endpoint = SW4_ENDPOINT;
  ZbZclAddEndpoint(zigbee_app_info.zb, &req, &conf);
  assert(conf.status == ZB_STATUS_SUCCESS);

  /* Endpoint: SW5_ENDPOINT */
  req.profileId = ZCL_PROFILE_HOME_AUTOMATION;
  req.deviceId = ZCL_DEVICE_LEVEL_OUTPUT;
  req.endpoint = SW5_ENDPOINT;
  ZbZclAddEndpoint(zigbee_app_info.zb, &req, &conf);
  assert(conf.status == ZB_STATUS_SUCCESS);

  /* Endpoint: SW6_ENDPOINT */
  req.profileId = ZCL_PROFILE_HOME_AUTOMATION;
  req.deviceId = ZCL_DEVICE_ONOFF_SWITCH;
  req.endpoint = SW6_ENDPOINT;
  ZbZclAddEndpoint(zigbee_app_info.zb, &req, &conf);
  assert(conf.status == ZB_STATUS_SUCCESS);

  /* USER CODE BEGIN CONFIG_ENDPOINT */
  struct zcl_msv_attr_callbacks_t cb = {
		  .msv_presentValue = multistate_presentValue_CB,
  };
  multistate_cluster = ZbZcl_msv_ServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, MULTISTATE_VALUE,
		  lengthof(opmode_state_text), opmode_state_text, &cb, NULL);
  assert(multistate_cluster != NULL);

  char *msv_desc = "\x09" "Operation";
  (void)ZbZclAttrStringWriteShort(multistate_cluster, ZCL_MULTISTATE_DESCRIPTION_ATTR, (uint8_t*)msv_desc);

  ZbZclClusterEndpointRegister(multistate_cluster);

  unsigned ep_idx;
  for (ep_idx = SW1_ENDPOINT; ep_idx <= SW6_ENDPOINT; ep_idx += 1) {
	  unsigned cl_idx = ep_idx - 1;
	  struct zcl_analog_attr_callbacks_t analog_cb = {
			  .read = NULL,
			  .write = analog_presentValue_CB,
	  };

	  analog_clusters[cl_idx] = ZbZcl_Analog_ServerAlloc(
			  zigbee_app_info.zb,
			  ep_idx,
			  analog_cluster_attrs[cl_idx].mode,
			  &analog_cluster_attrs[cl_idx].value,
			  &analog_cb,
			  &analog_cluster_attrs[cl_idx]);

	  assert(analog_clusters[cl_idx] != NULL);

	  (void)ZbZclAttrFloatWrite(analog_clusters[cl_idx], ZCL_ANALOG_PRESENT_VALUE_ATTR, analog_cluster_attrs[cl_idx].value);
	  (void)ZbZclAttrFloatWrite(analog_clusters[cl_idx], ZCL_ANALOG_MAX_PRESENT_ATTR, analog_cluster_attrs[cl_idx].maxPresent);
	  (void)ZbZclAttrFloatWrite(analog_clusters[cl_idx], ZCL_ANALOG_MIN_PRESENT_ATTR, analog_cluster_attrs[cl_idx].minPresent);
	  (void)ZbZclAttrFloatWrite(analog_clusters[cl_idx], ZCL_ANALOG_RESOLUTION_ATTR, analog_cluster_attrs[cl_idx].step);
	  (void)ZbZclAttrIntegerWrite(analog_clusters[cl_idx], ZCL_ANALOG_ENGINEERING_UNITS_ATTR, analog_cluster_attrs[cl_idx].engUnit);
	  (void)ZbZclAttrIntegerWrite(analog_clusters[cl_idx], ZCL_ANALOG_APPLICATION_TYPE_ATTR, analog_cluster_attrs[cl_idx].icon << 16);
	  (void)ZbZclAttrStringWriteShort(analog_clusters[cl_idx], ZCL_ANALOG_DESCRIPTION_ATTR, (uint8_t*)analog_cluster_attrs[cl_idx].desc);
	  ZbZclClusterEndpointRegister(analog_clusters[cl_idx]);
  }

  binary_cluster_light_on = ZbZcl_Binary_ServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, BINARY_SENSOR_INPUT,
		  &lightState, NULL, NULL);
  assert(binary_cluster_light_on != NULL);

  uint8_t light_on_sensor_desc[] = "\x0b" "Light state";
  (void)ZbZclAttrStringWriteShort(binary_cluster_light_on, ZCL_BINARY_DESCRIPTION_ATTR, light_on_sensor_desc);

  ZbZclClusterEndpointRegister(binary_cluster_light_on);

  binary_cluster_motion_detect = ZbZcl_Binary_ServerAlloc(zigbee_app_info.zb, SW3_ENDPOINT, BINARY_SENSOR_INPUT,
		  &motionState, NULL, NULL);
  assert(binary_cluster_motion_detect != NULL);

  uint8_t motion_detect_sensor_desc[] = "\x0d" "Motion detect";
  (void)ZbZclAttrStringWriteShort(binary_cluster_motion_detect, ZCL_BINARY_DESCRIPTION_ATTR, motion_detect_sensor_desc);

  ZbZclClusterEndpointRegister(binary_cluster_motion_detect);

  binary_cluster_dusk_detect = ZbZcl_Binary_ServerAlloc(zigbee_app_info.zb, SW2_ENDPOINT, BINARY_SENSOR_INPUT,
		  &duskState, NULL, NULL);
  assert(binary_cluster_dusk_detect != NULL);

  uint8_t dusk_detect_sensor_desc[] = "\x0b" "Dusk detect";
  (void)ZbZclAttrStringWriteShort(binary_cluster_dusk_detect, ZCL_BINARY_DESCRIPTION_ATTR, dusk_detect_sensor_desc);

  ZbZclClusterEndpointRegister(binary_cluster_dusk_detect);

#if 0
/* This section is disabled because the current triac control is not compatible with the trailing edge drive mode */
  binary_triac_mode_cluster = ZbZcl_Binary_ServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, BINARY_SENSOR_OUTPUT,
		  &trailing_edge, NULL, NULL);
  assert(binary_triac_mode_cluster != NULL);
  ZbZclClusterEndpointRegister(binary_triac_mode_cluster);

  uint8_t triac_mode_desc[] = "\x0d" "Trailing edge";
  (void)ZbZclAttrStringWriteShort(binary_triac_mode_cluster, ZCL_BINARY_DESCRIPTION_ATTR, triac_mode_desc);
#endif

  APP_ZIGBEE_ConfigBasic();
  /* USER CODE END CONFIG_ENDPOINT */
}

/**
 * @brief  Handle Zigbee network forming and joining
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_NwkForm(void)
{
  if ((zigbee_app_info.join_status != ZB_STATUS_SUCCESS) && (HAL_GetTick() >= zigbee_app_info.join_delay))
  {
    struct ZbStartupT config;
    enum ZbStatusCodeT status;

    /* Configure Zigbee Logging */
    ZbSetLogging(zigbee_app_info.zb, ZB_LOG_MASK_LEVEL_5, NULL);

    /* Attempt to join a zigbee network */
    ZbStartupConfigGetProDefaults(&config);

    /* Set the centralized network */
    APP_DBG("Network config : APP_STARTUP_CENTRALIZED_END_DEVICE");
    config.startupControl = zigbee_app_info.startupControl;

    /* Using the default HA preconfigured Link Key */
    memcpy(config.security.preconfiguredLinkKey, sec_key_ha, ZB_SEC_KEYSIZE);

    config.channelList.count = 1;
    config.channelList.list[0].page = 0;
    config.channelList.list[0].channelMask = 1 << CHANNEL; /*Channel in use */

    /* Add End device configuration */
    config.capability &= ~(MCP_ASSOC_CAP_RXONIDLE | MCP_ASSOC_CAP_DEV_TYPE | MCP_ASSOC_CAP_ALT_COORD);
    config.endDeviceTimeout=ZED_SLEEP_TIME_30S;

    /* Using ZbStartupWait (blocking) */
    status = ZbStartupWait(zigbee_app_info.zb, &config);

    APP_DBG("ZbStartup Callback (status = 0x%02x)", status);
    zigbee_app_info.join_status = status;

    if (status == ZB_STATUS_SUCCESS)
    {
      zigbee_app_info.join_delay = 0U;
      zigbee_app_info.init_after_join = true;
      APP_DBG("Startup done !\n");
      /* USER CODE BEGIN 0 */

      /* Register Persistent data change notification */
      ZbPersistNotifyRegister(zigbee_app_info.zb,APP_ZIGBEE_persist_notify_cb,NULL);
      /* Call the callback once here to save persistence data */
      APP_ZIGBEE_persist_notify_cb(zigbee_app_info.zb,NULL);


      /* USER CODE END 0 */
    }
    else
    {
      APP_DBG("Startup failed, attempting again after a short delay (%d ms)", APP_ZIGBEE_STARTUP_FAIL_DELAY);
      zigbee_app_info.join_delay = HAL_GetTick() + APP_ZIGBEE_STARTUP_FAIL_DELAY;
      /* USER CODE BEGIN 1 */

      /* USER CODE END 1 */
    }
  }

  /* If Network forming/joining was not successful reschedule the current task to retry the process */
  if (zigbee_app_info.join_status != ZB_STATUS_SUCCESS)
  {
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
  }
  /* USER CODE BEGIN NW_FORM */
  /* USER CODE END NW_FORM */
}

/*************************************************************
 * ZbStartupWait Blocking Call
 *************************************************************/
struct ZbStartupWaitInfo
{
  bool active;
  enum ZbStatusCodeT status;
};

static void ZbStartupWaitCb(enum ZbStatusCodeT status, void *cb_arg)
{
  struct ZbStartupWaitInfo *info = cb_arg;

  info->status = status;
  info->active = false;
  UTIL_SEQ_SetEvt(EVENT_ZIGBEE_STARTUP_ENDED);
}

enum ZbStatusCodeT ZbStartupWait(struct ZigBeeT *zb, struct ZbStartupT *config)
{
  struct ZbStartupWaitInfo *info;
  enum ZbStatusCodeT status;

  info = malloc(sizeof(struct ZbStartupWaitInfo));
  if (info == NULL)
  {
    return ZB_STATUS_ALLOC_FAIL;
  }
  memset(info, 0, sizeof(struct ZbStartupWaitInfo));

  info->active = true;
  status = ZbStartup(zb, config, ZbStartupWaitCb, info);
  if (status != ZB_STATUS_SUCCESS)
  {
    free(info);
    return status;
  }

  UTIL_SEQ_WaitEvt(EVENT_ZIGBEE_STARTUP_ENDED);
  status = info->status;
  free(info);
  return status;
}

/**
 * @brief  Trace the error or the warning reported.
 * @param  ErrId :
 * @param  ErrCode
 * @retval None
 */
void APP_ZIGBEE_Error(uint32_t ErrId, uint32_t ErrCode)
{
  switch (ErrId)
  {
    default:
      APP_ZIGBEE_TraceError("ERROR Unknown ", 0);
      break;
  }
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief  Warn the user that an error has occurred.
 *
 * @param  pMess  : Message associated to the error.
 * @param  ErrCode: Error code associated to the module (Zigbee or other module if any)
 * @retval None
 */
static void APP_ZIGBEE_TraceError(const char *pMess, uint32_t ErrCode)
{
  APP_DBG("**** Fatal error = %s (Err = %d)", pMess, ErrCode);
  /* USER CODE BEGIN TRACE_ERROR */
  /* USER CODE END TRACE_ERROR */

}

/**
 * @brief Check if the Coprocessor Wireless Firmware loaded supports Zigbee
 *        and display associated information
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_CheckWirelessFirmwareInfo(void)
{
  WirelessFwInfo_t wireless_info_instance;
  WirelessFwInfo_t *p_wireless_info = &wireless_info_instance;

  if (SHCI_GetWirelessFwInfo(p_wireless_info) != SHCI_Success)
  {
    APP_ZIGBEE_Error((uint32_t)ERR_ZIGBEE_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
  }
  else
  {
    APP_DBG("**********************************************************");
    APP_DBG("WIRELESS COPROCESSOR FW:");
    /* Print version */
    APP_DBG("VERSION ID = %d.%d.%d", p_wireless_info->VersionMajor, p_wireless_info->VersionMinor, p_wireless_info->VersionSub);

    switch (p_wireless_info->StackType)
    {
      case INFO_STACK_TYPE_ZIGBEE_FFD:
        APP_DBG("FW Type : FFD Zigbee stack");
        break;

      case INFO_STACK_TYPE_ZIGBEE_RFD:
        APP_DBG("FW Type : RFD Zigbee stack");
        break;

      default:
        /* No Zigbee device supported ! */
        APP_ZIGBEE_Error((uint32_t)ERR_ZIGBEE_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
        break;
    }

    /* print the application name */
    char *__PathProject__ = (strstr(__FILE__, "Zigbee") ? strstr(__FILE__, "Zigbee") + 7 : __FILE__);
    char *pdel = NULL;
    if((strchr(__FILE__, '/')) == NULL)
    {
      pdel = strchr(__PathProject__, '\\');
    }
    else
    {
      pdel = strchr(__PathProject__, '/');
    }

    int index = (int)(pdel - __PathProject__);
    APP_DBG("Application flashed: %*.*s", index, index, __PathProject__);

    /* print channel */
    APP_DBG("Channel used: %d", CHANNEL);
    /* print Link Key */
    APP_DBG("Link Key: %.16s", sec_key_ha);
    /* print Link Key value hex */
    char Z09_LL_string[ZB_SEC_KEYSIZE*3+1];
    Z09_LL_string[0] = 0;
    for (int str_index = 0; str_index < ZB_SEC_KEYSIZE; str_index++)
    {
      sprintf(&Z09_LL_string[str_index*3], "%02x ", sec_key_ha[str_index]);
    }

    APP_DBG("Link Key value: %s", Z09_LL_string);
    /* print clusters allocated */
    APP_DBG("Clusters allocated are:");
    APP_DBG("temperature_meas Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("illuminance_meas Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("**********************************************************");
  }
}

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

void APP_ZIGBEE_RegisterCmdBuffer(TL_CmdPacket_t *p_buffer)
{
  p_ZIGBEE_otcmdbuffer = p_buffer;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_OTCmdPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)p_ZIGBEE_otcmdbuffer->cmdserial.cmd.payload;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_OTCmdRspPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)((TL_EvtPacket_t *)p_ZIGBEE_otcmdbuffer)->evtserial.evt.payload;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_NotificationPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)(p_ZIGBEE_notif_M0_to_M4)->evtserial.evt.payload;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_M0RequestPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)(p_ZIGBEE_request_M0_to_M4)->evtserial.evt.payload;
}

/**
 * @brief  This function is used to transfer the commands from the M4 to the M0.
 *
 * @param   None
 * @return  None
 */
void ZIGBEE_CmdTransfer(void)
{
  Zigbee_Cmd_Request_t *cmd_req = (Zigbee_Cmd_Request_t *)p_ZIGBEE_otcmdbuffer->cmdserial.cmd.payload;

  /* Zigbee OT command cmdcode range 0x280 .. 0x3DF = 352 */
  p_ZIGBEE_otcmdbuffer->cmdserial.cmd.cmdcode = 0x280U;
  /* Size = otCmdBuffer->Size (Number of OT cmd arguments : 1 arg = 32bits so multiply by 4 to get size in bytes)
   * + ID (4 bytes) + Size (4 bytes) */
  p_ZIGBEE_otcmdbuffer->cmdserial.cmd.plen = 8U + (cmd_req->Size * 4U);

  TL_ZIGBEE_SendM4RequestToM0();

  /* Wait completion of cmd */
  Wait_Getting_Ack_From_M0();
}

/**
 * @brief  This function is called when the M0+ acknowledge the fact that it has received a Cmd
 *
 *
 * @param   Otbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_ZIGBEE_CmdEvtReceived(TL_EvtPacket_t *Otbuffer)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Otbuffer);

  Receive_Ack_From_M0();
}

/**
 * @brief  This function is called when notification from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_ZIGBEE_NotReceived(TL_EvtPacket_t *Notbuffer)
{
  p_ZIGBEE_notif_M0_to_M4 = Notbuffer;

  Receive_Notification_From_M0();
}

/**
 * @brief  This function is called before sending any ot command to the M0
 *         core. The purpose of this function is to be able to check if
 *         there are no notifications coming from the M0 core which are
 *         pending before sending a new ot command.
 * @param  None
 * @retval None
 */
void Pre_ZigbeeCmdProcessing(void)
{
  UTIL_SEQ_WaitEvt(EVENT_SYNCHRO_BYPASS_IDLE);
}

/**
 * @brief  This function waits for getting an acknowledgment from the M0.
 *
 * @param  None
 * @retval None
 */
static void Wait_Getting_Ack_From_M0(void)
{
  UTIL_SEQ_WaitEvt(EVENT_ACK_FROM_M0_EVT);
}

/**
 * @brief  Receive an acknowledgment from the M0+ core.
 *         Each command send by the M4 to the M0 are acknowledged.
 *         This function is called under interrupt.
 * @param  None
 * @retval None
 */
static void Receive_Ack_From_M0(void)
{
  UTIL_SEQ_SetEvt(EVENT_ACK_FROM_M0_EVT);
}

/**
 * @brief  Receive a notification from the M0+ through the IPCC.
 *         This function is called under interrupt.
 * @param  None
 * @retval None
 */
static void Receive_Notification_From_M0(void)
{
  CptReceiveNotifyFromM0++;
  UTIL_SEQ_SetTask(1U << (uint32_t)CFG_TASK_NOTIFY_FROM_M0_TO_M4, CFG_SCH_PRIO_0);
}

/**
 * @brief  This function is called when a request from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_ZIGBEE_M0RequestReceived(TL_EvtPacket_t *Reqbuffer)
{
  p_ZIGBEE_request_M0_to_M4 = Reqbuffer;

  CptReceiveRequestFromM0++;
  UTIL_SEQ_SetTask(1U << (uint32_t)CFG_TASK_REQUEST_FROM_M0_TO_M4, CFG_SCH_PRIO_0);
}

/**
 * @brief Perform initialization of TL for Zigbee.
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_TL_INIT(void)
{
  ZigbeeConfigBuffer.p_ZigbeeOtCmdRspBuffer = (uint8_t *)&ZigbeeOtCmdBuffer;
  ZigbeeConfigBuffer.p_ZigbeeNotAckBuffer = (uint8_t *)ZigbeeNotifRspEvtBuffer;
  ZigbeeConfigBuffer.p_ZigbeeNotifRequestBuffer = (uint8_t *)ZigbeeNotifRequestBuffer;
  TL_ZIGBEE_Init(&ZigbeeConfigBuffer);
}

/**
 * @brief Process the messages coming from the M0.
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ProcessNotifyM0ToM4(void)
{
  if (CptReceiveNotifyFromM0 != 0)
  {
    /* Reset counter */
    CptReceiveNotifyFromM0 = 0;
    Zigbee_CallBackProcessing();
  }
}

/**
 * @brief Process the requests coming from the M0.
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ProcessRequestM0ToM4(void)
{
  if (CptReceiveRequestFromM0 != 0)
  {
    CptReceiveRequestFromM0 = 0;
    Zigbee_M0RequestProcessing();
  }
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */

/**
 * @brief  Setup basic cluster attributes.
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ConfigBasic(void)
{
 	uint8_t mfr_str[] = "\x02" "DS";
 	ZbZclBasicWriteDirect(zigbee_app_info.zb, SW1_ENDPOINT,
 			ZCL_BASIC_ATTR_MFR_NAME, mfr_str, sizeof(mfr_str) - 1);

 	uint8_t model_str[] = "\x06" "Model1";
 	ZbZclBasicWriteDirect(zigbee_app_info.zb, SW1_ENDPOINT,
 			ZCL_BASIC_ATTR_MODEL_NAME, model_str, sizeof(model_str) - 1);

 	uint8_t location_str[] = "\x0d" "Outdoor light";
 	ZbZclBasicWriteDirect(zigbee_app_info.zb, SW1_ENDPOINT,
 			ZCL_BASIC_ATTR_LOCATION, model_str, sizeof(location_str) - 1);

 	uint8_t power_source = 0x01; // single phase
 	ZbZclBasicWriteDirect(zigbee_app_info.zb, SW1_ENDPOINT,
 			ZCL_BASIC_ATTR_POWER_SOURCE, &power_source, 1);

 	uint8_t physical_env = 0x0c; // deck
 	ZbZclBasicWriteDirect(zigbee_app_info.zb, SW1_ENDPOINT,
 			ZCL_BASIC_ATTR_ENVIRONMENT, &physical_env, 1);
}

/**
 * @brief  Set default attribute values if not starting from persistence
 * @param  None
 * @retval None
 */
void set_default_attr_values(void)
{
	unsigned ep_idx;
	for (ep_idx = SW1_ENDPOINT; ep_idx <= SW6_ENDPOINT; ep_idx += 1) {
		unsigned cl_idx = ep_idx - 1;
		(void)ZbZclAttrFloatWrite(analog_clusters[cl_idx], ZCL_ANALOG_PRESENT_VALUE_ATTR, analog_cluster_attrs[cl_idx].value);
	}

	(void)ZbZclAttrIntegerWrite(multistate_cluster, ZCL_MULTISTATE_PRESENT_VALUE_ATTR, operationMode);
}


/**
 * @brief  Set global parameter in response to the changed attribute value
 * @param  New attribute value
 * @retval None
 */
static void multistate_set_mode(uint16_t opmode)
{
	if (opmode >= OP_AUTO && opmode <= OP_OFF) {
		operationMode = opmode;
	}

}

/**
 * @brief  Callback function for the MultistateValue cluster
 * @param  Pointer to the cluster data
 * @param  Pointer to the attribute data
 * @retval Zigbee cluster library Status code
 */
enum ZclStatusCodeT multistate_presentValue_CB(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info)
{
	uint16_t attributeId = info->info->attributeId;
	uint8_t endpoint = clusterPtr->endpoint;

	if (attributeId == ZCL_MULTISTATE_PRESENT_VALUE_ATTR && endpoint == SW1_ENDPOINT ) {
			enum ZclStatusCodeT result = ZCL_STATUS_SUCCESS;
			enum ZclDataTypeT type;
			uint16_t opmode = ZbZclAttrIntegerRead(clusterPtr, attributeId, &type, &result);
			if (ZCL_STATUS_SUCCESS == result)
				multistate_set_mode(opmode);
	}
	return ZCL_STATUS_SUCCESS;
}

/**
 * @brief  Set global parameter in response to the changed attribute value. Converts float value to integer value.
 * @param  Pointer to the attribute description data
 * @retval None
 */
void update_analog_attr_setting(struct analog_cluster_attr *attr)
{
	if (attr->extSetting) {
		float step_cnt = attr->value / attr->step;
		step_cnt =	 attr->value < 0 ? step_cnt - 0.5 : step_cnt + 0.5;
		*attr->extSetting = (int32_t)step_cnt;
		if (attr->ext_setting_cb)
			attr->ext_setting_cb(attr->extSetting);
	}
}

/**
 * @brief  Set global parameter in response to the changed attribute value
 * @param  Pointer to the cluster data
 * @param  Pointer to the attribute data
 * @param  Callback argument (stores attribute description data)
 * @retval Zigbee cluster library Status code
 */
enum ZclStatusCodeT analog_presentValue_CB(struct ZbZclClusterT *clusterPtr, struct ZbZclAttrCbInfoT *info, void *arg)
{
	struct analog_cluster_attr *attr = (struct analog_cluster_attr *)arg;
	update_analog_attr_setting(attr);
	return ZCL_STATUS_SUCCESS;
}


/**
 * @brief  Update value attribute of the illuminance measurement cluster
 * @param  Illuminance measurement
 * @retval None
 */
void APP_ZIGBEE_update_illum(uint16_t illum)
{
	int ha_illum;
/*
 * We need to send 10000 * log10(I), and we have 10*log2(I) + 50, so convert here
 */
	ha_illum = ((int)illum - 50) * 301;
	if (ha_illum <= 0) ha_illum = 1;
	(void)ZbZclAttrIntegerWrite(zigbee_app_info.illuminance_meas_server_1, ZCL_ILLUM_MEAS_ATTR_MEAS_VAL, ha_illum);
}

/**
 * @brief  Update value attribute of the binary input cluster
 * @param  Light state
 * @retval None
 */
void APP_ZIGBEE_update_lightstate(uint16_t state)
{
	if ((0 == state) != (0 == lightState))
		(void)ZbZclAttrIntegerWrite(binary_cluster_light_on, ZCL_BINARY_PRESENT_VALUE_ATTR, state);
}

/**
 * @brief  Update value attribute of the binary input cluster
 * @param  Motion detect
 * @retval None
 */
void APP_ZIGBEE_update_motion_detect(uint16_t state)
{
	if ((0 == state) != (0 == motionState))
		(void)ZbZclAttrIntegerWrite(binary_cluster_motion_detect, ZCL_BINARY_PRESENT_VALUE_ATTR, state);
}

/**
 * @brief  Update value attribute of the binary input cluster
 * @param  Dusk detect
 * @retval None
 */
void APP_ZIGBEE_update_dusk_detect(uint16_t state)
{
	if ((0 == state) != (0 == duskState))
		(void)ZbZclAttrIntegerWrite(binary_cluster_dusk_detect, ZCL_BINARY_PRESENT_VALUE_ATTR, state);
}

/**
 * @brief  Update value attribute of the temperature sensor cluster
 * @param  Temperature C x 100
 * @retval None
 */
void APP_ZIGBEE_update_temperature(int16_t temp)
{
	(void)ZbZclAttrIntegerWrite(zigbee_app_info.temperature_meas_server_1, ZCL_TEMP_MEAS_ATTR_MEAS_VAL, temp);
}


/*************************************************************
 *
 * NVM FUNCTIONS
 *
 *************************************************************/
static void APP_ZIGBEE_pack_persist_values(struct persistence_attrs *persistence_attrs)
{
    persistence_attrs->struct_len = sizeof(struct persistence_attrs);

    unsigned n;
    for (n = 0; n < lengthof(analog_cluster_attrs); n++)
    	persistence_attrs->analogValues[n] = analog_cluster_attrs[n].value;

    persistence_attrs->multistateValue = operationMode;
}

static void APP_ZIGBEE_unpack_persist_values(struct persistence_attrs *persistence_attrs)
{
	if (persistence_attrs->struct_len == sizeof(struct persistence_attrs)) {
		unsigned n;
		for (n = 0; n < lengthof(analog_cluster_attrs); n++)
			analog_cluster_attrs[n].value = persistence_attrs->analogValues[n];

		operationMode = persistence_attrs->multistateValue;
	}
}

/**
 * @brief  notify to save persistent data callback
 * @param  zb: Zigbee device object pointer, cbarg: callback arg pointer
 * @retval None
 */
static void APP_ZIGBEE_persist_notify_cb(struct ZigBeeT *zb, void *cbarg)
{
  APP_DBG("Notification to save persistent data requested from stack");
  /* Save the persistent data */
  APP_ZIGBEE_persist_save();
}

/**
 * @brief  Start Zigbee Network from persistent data
 * @param  zb: Zigbee device object pointer
 * @retval Zigbee stack Status code
 */
static enum ZbStatusCodeT APP_ZIGBEE_ZbStartupPersist(struct ZigBeeT* zb)
{
   bool read_status;
   enum ZbStatusCodeT status = ZB_STATUS_SUCCESS;

   /* Restore persistence */
   read_status = APP_ZIGBEE_persist_load();

   if (read_status)
   {
       /* Make sure the EPID is cleared, before we are allowed to restore persistence */
       uint64_t epid = 0U;
       ZbNwkSet(zb, ZB_NWK_NIB_ID_ExtendedPanId, &epid, sizeof(uint64_t));

       /* Start-up from persistence */
       APP_DBG("APP_ZIGBEE_ZbStartupPersist: restoring stack persistence");
       status = ZbStartupPersist(zb, &cache_persistent_data.U8_data[4], cache_persistent_data.U32_data[0],NULL,APP_ZIGBEE_PersistCompleted_callback,NULL);
   }
   else
   {
       /* Failed to restart from persistence */
       APP_DBG("APP_ZIGBEE_ZbStartupPersist: no persistence data to restore");
       status = ZB_STATUS_ALLOC_FAIL;
   }

   /* Only for debug purpose, depending of persistent data, following traces
      could display bytes that are irrelevants to on off cluster */
   if(status == ZB_STATUS_SUCCESS)
   {
     /* read the last bytes of data where the ZCL on off persistent data shall be*/
      uint32_t len = cache_persistent_data.U32_data[0] + 4 ;
      APP_DBG("ClusterID %02x %02x",cache_persistent_data.U8_data[len-9],cache_persistent_data.U8_data[len-10]);
      APP_DBG("Endpoint %02x %02x",cache_persistent_data.U8_data[len-7],cache_persistent_data.U8_data[len-8]);
      APP_DBG("Direction %02x",cache_persistent_data.U8_data[len-6]);
      APP_DBG("AttrID %02x %02x",cache_persistent_data.U8_data[len-4],cache_persistent_data.U8_data[len-5]);
      APP_DBG("Len %02x %02x",cache_persistent_data.U8_data[len-2],cache_persistent_data.U8_data[len-3]);
      APP_DBG("Value %02x",cache_persistent_data.U8_data[len-1]);
   }

   return status;
}/* APP_ZIGBEE_ZbStartupPersist */

/**
 * @brief  timer callback to wait end of restore cluster persistence form M0
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_PersistCompleted_callback(enum ZbStatusCodeT status,void *arg)
{
   if(status == ZB_WPAN_STATUS_SUCCESS)
   {
	   APP_DBG("Persist complete callback entered with SUCCESS");
	   set_default_attr_values();
   }
   else
   {
	   APP_DBG("Error in persist complete callback %x",status);
   }
   /* STEP3 - Activate back the persistent notification */
     /* Register Persistent data change notification */
     ZbPersistNotifyRegister(zigbee_app_info.zb,APP_ZIGBEE_persist_notify_cb,NULL);

     /* Call the callback once here to save persistence data */
     APP_ZIGBEE_persist_notify_cb(zigbee_app_info.zb,NULL);
}/* APP_ZIGBEE_PersistCompleted_callback */


/**
 * @brief  Load persistent data
 * @param  None
 * @retval true if success, false if fail
 */
static bool APP_ZIGBEE_persist_load(void)
{
#ifdef CFG_NVM
    APP_DBG("Retrieving persistent data from FLASH");
    bool status = APP_ZIGBEE_NVM_Read();

    if (status) {
    	unsigned len = cache_persistent_data.U32_data[0];
    	if (len > sizeof(struct persistence_attrs))
    	{
    		len -= sizeof(struct persistence_attrs);
    		cache_persistent_data.U32_data[0] = len;
    		struct persistence_attrs *persistence_attrs = (struct persistence_attrs *)&cache_persistent_data.U8_data[ST_PERSIST_FLASH_DATA_OFFSET + len];
    		APP_ZIGBEE_unpack_persist_values(persistence_attrs);
    	}
    }
    return status;
#else
    /* Check length range */
    if ((cache_persistent_data.U32_data[0] == 0) ||
        (cache_persistent_data.U32_data[0] > ST_PERSIST_MAX_ALLOC_SZ))
    {
        APP_DBG("No data or too large length : %d",cache_persistent_data.U32_data[0]);
        return false;
    }
    return true;
#endif /* CFG_NVM */
} /* APP_ZIGBEE_persist_load */

/**
 * @brief  Save persistent data
 * @param  None
 * @retval true if success , false if fail
 */
static bool APP_ZIGBEE_persist_save(void)
{
    uint32_t len;

    /* Clear the RAM cache before saving */
    memset(cache_persistent_data.U8_data, 0x00, ST_PERSIST_MAX_ALLOC_SZ);

    /* Call the stack API to get current persistent data */
    len = ZbPersistGet(zigbee_app_info.zb, 0, 0);
    /* Check Length range */
    if (len == 0U)
    {
        /* If the persistence length was zero then no data available. */
        APP_DBG("APP_ZIGBEE_persist_save: no persistence data to save !");
        return false;
    }
    if (len > ST_PERSIST_MAX_ALLOC_SZ - sizeof(struct persistence_attrs))
    {
        /* if persistence length too big to store */
        APP_DBG("APP_ZIGBEE_persist_save: persist size too large for storage (%d)", len);
        return false;
    }

    /* Store in cache the persistent data */
    len = ZbPersistGet(zigbee_app_info.zb, &cache_persistent_data.U8_data[ST_PERSIST_FLASH_DATA_OFFSET], len);

    /* Append attribute values that need to survive restarts. ZCL persistence code is unreliable, so implement the separate storage. */
    struct persistence_attrs *persistence_attrs = (struct persistence_attrs *)&cache_persistent_data.U8_data[ST_PERSIST_FLASH_DATA_OFFSET + len];
    APP_ZIGBEE_pack_persist_values(persistence_attrs);

    /* Store in cache the persistent data length */
    cache_persistent_data.U32_data[0] = len + sizeof(struct persistence_attrs);

    persistNumWrites++;
    APP_DBG("APP_ZIGBEE_persist_save: Persistence written in cache RAM (num writes = %d) len=%d",
             persistNumWrites, cache_persistent_data.U32_data[0]);

#ifdef CFG_NVM
    if(!APP_ZIGBEE_NVM_Write())
    {
    	return false;
    }

    APP_DBG("APP_ZIGBEE_persist_save: Persistent data FLASHED");
#endif /* CFG_NVM */

    return true;
} /* APP_ZIGBEE_persist_save */

/**
 * @brief  Delete persistent data
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_persist_delete(void)
{
  /* Clear RAM cache */
   memset(cache_persistent_data.U8_data, 0x00, ST_PERSIST_MAX_ALLOC_SZ);
   APP_DBG("Persistent Data RAM cache cleared");
#ifdef CFG_NVM
   APP_DBG("FLASH ERASED");
   APP_ZIGBEE_NVM_Erase();
#endif /* CFG_NVM */
} /* APP_ZIGBEE_persist_delete */


#ifdef CFG_NVM
/**
 * @brief  Init the NVM
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_NVM_Init(void)
{
  int eeprom_init_status;

  APP_DBG("Flash starting address = %x",HW_FLASH_ADDRESS  + CFG_NVM_BASE_ADDRESS);
  eeprom_init_status = EE_Init( 0 , HW_FLASH_ADDRESS + CFG_NVM_BASE_ADDRESS );

  if(eeprom_init_status != EE_OK)
  {
    /* format NVM since init failed */
    eeprom_init_status= EE_Init( 1, HW_FLASH_ADDRESS + CFG_NVM_BASE_ADDRESS );
  }
  APP_DBG("EE_init status = %d",eeprom_init_status);

} /* APP_ZIGBEE_NVM_Init */

/**
*@brief  Read the persistent data from NVM
* @param  None
* @retval true if success , false if failed
*/
static bool APP_ZIGBEE_NVM_Read(void)
{
    uint16_t num_words = 0;
    bool status = true;
    int ee_status = 0;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR | FLASH_FLAG_OPTVERR);

    /* Read the data length from cache */
    ee_status = EE_Read(0, ZIGBEE_DB_START_ADDR, &cache_persistent_data.U32_data[0]);
    if (ee_status != EE_OK)
    {
        APP_DBG("Read -> persistent data length not found ERASE to be done - Read Stopped");
        status = false;
    }
      /* Check length is not too big nor zero */
    else if((cache_persistent_data.U32_data[0] == 0) ||
            (cache_persistent_data.U32_data[0]> ST_PERSIST_MAX_ALLOC_SZ))
    {
            APP_DBG("No data or too large length : %d", cache_persistent_data.U32_data[0]);
            status = false;
    }
        /* Length is within range */
    else
    {
           /* Adjust the length to be U32 aligned */
            num_words = (uint16_t) (cache_persistent_data.U32_data[0]/4) ;
            if (cache_persistent_data.U32_data[0] % 4 != 0)
            {
                num_words++;
            }

            /* copy the read data from Flash to cache including length */
            for (uint16_t local_length = 1; local_length <= num_words; local_length++)
            {
            	if (local_length >= ST_PERSIST_MAX_ALLOC_SZ/4)
            	{
                    APP_DBG("Local length exceeds the size of the cache persistent data!");
                    status = false;
                    break;
            	}

                /* read data from first data in U32 unit */
                ee_status = EE_Read(0, local_length + ZIGBEE_DB_START_ADDR, &cache_persistent_data.U32_data[local_length] );
                if (ee_status != EE_OK)
                {
                    APP_DBG("Read not found leaving");
                    status = false;
                    break;
                }
            }
    }

    HAL_FLASH_Lock();
    if(status)
    {
        APP_DBG("READ PERSISTENT DATA LEN = %d",cache_persistent_data.U32_data[0]);
    }
    return status;
} /* APP_ZIGBEE_NVM_Read */

/**
 * @brief  Write the persistent data in NVM
 * @param  None
 * @retval None
 */
static bool APP_ZIGBEE_NVM_Write(void)
{
    int ee_status = 0;

    uint16_t num_words;
    uint16_t local_current_size;


    num_words = 1U; /* 1 words for the length */
    num_words+= (uint16_t) (cache_persistent_data.U32_data[0]/4);


    /* Adjust the length to be U32 aligned */
    if (cache_persistent_data.U32_data[0] % 4 != 0)
    {
        num_words++;
    }

    //save data in flash
    for (local_current_size = 0; local_current_size < num_words; local_current_size++)
    {
        ee_status = EE_Write(0, (uint16_t)local_current_size + ZIGBEE_DB_START_ADDR, cache_persistent_data.U32_data[local_current_size]);
        if (ee_status != EE_OK)
        {
           if(ee_status == EE_CLEAN_NEEDED) /* Shall not be there if CFG_EE_AUTO_CLEAN = 1*/
           {
              APP_DBG("CLEAN NEEDED, CLEANING");
              EE_Clean(0,0);
           }
           else
           {
              /* Failed to write , an Erase shall be done */
              APP_DBG("APP_ZIGBEE_NVM_Write failed @ %d status %d", local_current_size,ee_status);
              break;
           }
        }
    }


    if(ee_status != EE_OK)
    {
       APP_DBG("WRITE STOPPED, need a FLASH ERASE");
       return false;
    }

    APP_DBG("WRITTEN PERSISTENT DATA LEN = %d",cache_persistent_data.U32_data[0]);
    return true;

} /* APP_ZIGBEE_NVM_Write */

/**
 * @brief  Erase the NVM
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_NVM_Erase(void)
{
   EE_Init(1, HW_FLASH_ADDRESS + CFG_NVM_BASE_ADDRESS); /* Erase Flash */
} /* APP_ZIGBEE_NVM_Erase */

#endif /* CFG_NVM */


static void zb_leave_cb(struct ZbNlmeLeaveConfT *conf, void *arg)
{
	struct zigbee_app_info *app = (struct zigbee_app_info *)arg;
	ZbReset(app->zb);

	/* Configure the joining parameters */
	zigbee_app_info.join_delay = HAL_GetTick(); /* now */
	zigbee_app_info.startupControl = ZbStartTypeJoin;

	/* attempt to reconnect */
	UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);

}

void ZIGBEE_Leave(void)
{
	enum ZbStatusCodeT status =
			ZbLeaveReq(zigbee_app_info.zb, zb_leave_cb, &zigbee_app_info);

	zigbee_app_info.join_status = (enum ZbStatusCodeT) 0x01; /* init to error status */

	if (ZB_STATUS_SUCCESS == status) {
		APP_ZIGBEE_persist_delete();
	}
}

enum ZbStatusCodeT ZIGBEE_JoinStatus(void)
{
		return zigbee_app_info.join_status;
}

/* USER CODE END FD_LOCAL_FUNCTIONS */
