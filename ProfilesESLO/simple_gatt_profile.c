/******************************************************************************

 @file  simple_gatt_profile.c

 @brief This file contains the Simple GATT profile sample GATT service profile
 for use with the BLE sample application.

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2010-2021, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
 its contributors may be used to endorse or promote products derived
 from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "simple_gatt_profile.h"

#ifdef SYSCFG
#include "ti_ble_config.h"

#ifdef USE_GATT_BUILDER
#include "ble_gatt_service.h"
#endif

#endif

extern ICall_EntityID selfEntity;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#ifndef USE_GATT_BUILDER

#define SERVAPP_NUM_ATTR_SUPPORTED        29

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0XAB00
CONST uint8 simpleProfileServUUID[ATT_BT_UUID_SIZE] = { LO_UINT16(
		SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

// Characteristic 1
CONST uint8 simpleProfilechar1UUID[ATT_BT_UUID_SIZE] = { LO_UINT16(
		SIMPLEPROFILE_CHAR1_UUID), HI_UINT16(SIMPLEPROFILE_CHAR1_UUID) };

// Characteristic 2
CONST uint8 simpleProfilechar2UUID[ATT_BT_UUID_SIZE] = { LO_UINT16(
		SIMPLEPROFILE_CHAR2_UUID), HI_UINT16(SIMPLEPROFILE_CHAR2_UUID) };

// Characteristic 3
CONST uint8 simpleProfilechar3UUID[ATT_BT_UUID_SIZE] = { LO_UINT16(
		SIMPLEPROFILE_CHAR3_UUID), HI_UINT16(SIMPLEPROFILE_CHAR3_UUID) };

// Characteristic 4
CONST uint8 simpleProfilechar4UUID[ATT_BT_UUID_SIZE] = { LO_UINT16(
		SIMPLEPROFILE_CHAR4_UUID), HI_UINT16(SIMPLEPROFILE_CHAR4_UUID) };

// Characteristic 5
CONST uint8 simpleProfilechar5UUID[ATT_BT_UUID_SIZE] = { LO_UINT16(
		SIMPLEPROFILE_CHAR5_UUID), HI_UINT16(SIMPLEPROFILE_CHAR5_UUID) };

// Characteristic 6
CONST uint8 simpleProfilechar6UUID[ATT_BT_UUID_SIZE] = { LO_UINT16(
		SIMPLEPROFILE_CHAR6_UUID), HI_UINT16(SIMPLEPROFILE_CHAR6_UUID) };

// Characteristic 7
CONST uint8 simpleProfilechar7UUID[ATT_BT_UUID_SIZE] = { LO_UINT16(
		SIMPLEPROFILE_CHAR7_UUID), HI_UINT16(SIMPLEPROFILE_CHAR7_UUID) };

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static simpleProfileCBs_t *simpleProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t simpleProfileService = { ATT_BT_UUID_SIZE,
		simpleProfileServUUID };

// LED_0 RW (N?)
// still use an array, avoids confusion with pointers later on
static uint8 simpleProfileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE
		| GATT_PROP_NOTIFY;
static uint8 simpleProfileChar1[SIMPLEPROFILE_CHAR1_LEN] = { 0 };
static gattCharCfg_t *simpleProfileChar1Config; // notify only

// vBatt/vitals RN
static uint8 simpleProfileChar2Props = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 simpleProfileChar2[SIMPLEPROFILE_CHAR2_LEN] = { 0 };
static gattCharCfg_t *simpleProfileChar2Config; // notify only

// Settings RWN
static uint8 simpleProfileChar3Props = GATT_PROP_READ | GATT_PROP_WRITE
		| GATT_PROP_NOTIFY;
static uint8 simpleProfileChar3[SIMPLEPROFILE_CHAR3_LEN] = { 0 };
static gattCharCfg_t *simpleProfileChar3Config; // notify only

// EEG RN
static uint8 simpleProfileChar4Props = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 simpleProfileChar4[SIMPLEPROFILE_CHAR4_LEN] = { 0 };
static gattCharCfg_t *simpleProfileChar4Config; // notify only

// AXY RN
static uint8 simpleProfileChar5Props = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 simpleProfileChar5[SIMPLEPROFILE_CHAR5_LEN] = { 0 };
static gattCharCfg_t *simpleProfileChar5Config; // notify only

// ADDR RW
static uint8 simpleProfileChar6Props = GATT_PROP_READ | GATT_PROP_WRITE
		| GATT_PROP_NOTIFY;
static uint8 simpleProfileChar6[SIMPLEPROFILE_CHAR6_LEN] = { 0 };
static gattCharCfg_t *simpleProfileChar6Config; // notify only

// SWA RN
static uint8 simpleProfileChar7Props = GATT_PROP_READ | GATT_PROP_INDICATE;
static uint8 simpleProfileChar7[SIMPLEPROFILE_CHAR7_LEN] = { 0 };
static gattCharCfg_t *simpleProfileChar7Config;

static uint8 simpleProfileChar1UserDesp[17] = "ESLO LEDs       ";
static uint8 simpleProfileChar2UserDesp[17] = "ESLO Vitals     ";
static uint8 simpleProfileChar3UserDesp[17] = "ESLO Settings   ";
static uint8 simpleProfileChar4UserDesp[17] = "ESLO EEG        ";
static uint8 simpleProfileChar5UserDesp[17] = "ESLO AXY        ";
static uint8 simpleProfileChar6UserDesp[17] = "ESLO ADDR       ";
static uint8 simpleProfileChar7UserDesp[17] = "ESLO SWA        ";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simpleProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = {
// Simple Profile Service
		{ { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
		GATT_PERMIT_READ, /* permissions */
		0, /* handle */
		(uint8*) &simpleProfileService /* pValue */
		},

		// Characteristic 1 Declaration
		{ { ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 0, &simpleProfileChar1Props },
		// Characteristic Value 1
		{ { ATT_BT_UUID_SIZE, simpleProfilechar1UUID }, GATT_PERMIT_READ
				| GATT_PERMIT_WRITE, 0, simpleProfileChar1 },
		// Characteristic 1 configuration
		{ { ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0,
				(uint8*) &simpleProfileChar1Config },
		// Characteristic 1 User Description
		{ { ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 0, simpleProfileChar1UserDesp },

		// Characteristic 2 Declaration
		{ { ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 0, &simpleProfileChar2Props },
		// Characteristic Value 2
		{ { ATT_BT_UUID_SIZE, simpleProfilechar2UUID }, GATT_PERMIT_READ, 0,
				simpleProfileChar2 },
		// Characteristic 2 configuration
		{ { ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0,
				(uint8*) &simpleProfileChar2Config },
		// Characteristic 2 User Description
		{ { ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 0, simpleProfileChar2UserDesp },

		// Characteristic 3 Declaration
		{ { ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 0, &simpleProfileChar3Props },
		// Characteristic Value 3
		{ { ATT_BT_UUID_SIZE, simpleProfilechar3UUID }, GATT_PERMIT_READ
				| GATT_PERMIT_WRITE, 0, simpleProfileChar3 },
		// Characteristic 3 configuration
		{ { ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0,
				(uint8*) &simpleProfileChar3Config },
		// Characteristic 3 User Description
		{ { ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 0, simpleProfileChar3UserDesp },

		// Characteristic 4 Declaration
		{ { ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 0, &simpleProfileChar4Props },
		// Characteristic Value 4
		{ { ATT_BT_UUID_SIZE, simpleProfilechar4UUID }, GATT_PERMIT_READ, 0,
				simpleProfileChar4 },
		// Characteristic 4 configuration
		{ { ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0,
				(uint8*) &simpleProfileChar4Config },
		// Characteristic 4 User Description
		{ { ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 0, simpleProfileChar4UserDesp },

		// Characteristic 5 Declaration
		{ { ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 0, &simpleProfileChar5Props },
		// Characteristic Value 5
		{ { ATT_BT_UUID_SIZE, simpleProfilechar5UUID }, GATT_PERMIT_READ, 0,
				simpleProfileChar5 },
		// Characteristic 5 configuration
		{ { ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0,
				(uint8*) &simpleProfileChar5Config },
		// Characteristic 5 User Description
		{ { ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 0, simpleProfileChar5UserDesp },

		// Characteristic 6 Declaration
		{ { ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 0, &simpleProfileChar6Props },
		// Characteristic Value 6
		{ { ATT_BT_UUID_SIZE, simpleProfilechar6UUID }, GATT_PERMIT_READ
				| GATT_PERMIT_WRITE, 0, simpleProfileChar6 },
		// Characteristic 6 configuration
		{ { ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0,
				(uint8*) &simpleProfileChar6Config },
		// Characteristic 6 User Description
		{ { ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 0, simpleProfileChar6UserDesp },

		// Characteristic 7 Declaration
		{ { ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 0, &simpleProfileChar7Props },
		// Characteristic Value 7
		{ { ATT_BT_UUID_SIZE, simpleProfilechar7UUID }, GATT_PERMIT_READ, 0,
				simpleProfileChar7 },
		// Characteristic 7 configuration
		{ { ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0,
				(uint8*) &simpleProfileChar7Config },
		// Characteristic 7 User Description
		{ { ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 0, simpleProfileChar7UserDesp },

};
#endif // USE_GATT_BUILDER
/*********************************************************************
 * LOCAL FUNCTIONS
 */
bStatus_t simpleProfile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
		uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen,
		uint8_t method);
bStatus_t simpleProfile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
		uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method);
#ifndef USE_GATT_BUILDER
/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t simpleProfileCBs = { simpleProfile_ReadAttrCB, // Read callback function pointer
		simpleProfile_WriteAttrCB, // Write callback function pointer
		NULL                       // Authorization callback function pointer
		};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SimpleProfile_AddService(uint32 services) {
	uint8 status;

	// Allocate Client Characteristic 1 Configuration table
	simpleProfileChar1Config = (gattCharCfg_t*) ICall_malloc(
			sizeof(gattCharCfg_t) *
			MAX_NUM_BLE_CONNS);
	if (simpleProfileChar1Config == NULL) {
		return ( bleMemAllocError);
	}
	GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID,
			simpleProfileChar1Config);

	// Allocate Client Characteristic 2 Configuration table
	simpleProfileChar2Config = (gattCharCfg_t*) ICall_malloc(
			sizeof(gattCharCfg_t) *
			MAX_NUM_BLE_CONNS);
	if (simpleProfileChar2Config == NULL) {
		return ( bleMemAllocError);
	}
	GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID,
			simpleProfileChar2Config);

	// Allocate Client Characteristic 3 Configuration table
	simpleProfileChar3Config = (gattCharCfg_t*) ICall_malloc(
			sizeof(gattCharCfg_t) *
			MAX_NUM_BLE_CONNS);
	if (simpleProfileChar3Config == NULL) {
		return ( bleMemAllocError);
	}
	GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID,
			simpleProfileChar3Config);

	// Allocate Client Characteristic 4 Configuration table
	simpleProfileChar4Config = (gattCharCfg_t*) ICall_malloc(
			sizeof(gattCharCfg_t) *
			MAX_NUM_BLE_CONNS);
	if (simpleProfileChar4Config == NULL) {
		return ( bleMemAllocError);
	}
	GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID,
			simpleProfileChar4Config);

	// Allocate Client Characteristic 5 Configuration table
	simpleProfileChar5Config = (gattCharCfg_t*) ICall_malloc(
			sizeof(gattCharCfg_t) *
			MAX_NUM_BLE_CONNS);
	if (simpleProfileChar5Config == NULL) {
		return ( bleMemAllocError);
	}
	GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID,
			simpleProfileChar5Config);

	// Allocate Client Characteristic 6 Configuration table
	simpleProfileChar6Config = (gattCharCfg_t*) ICall_malloc(
			sizeof(gattCharCfg_t) *
			MAX_NUM_BLE_CONNS);
	if (simpleProfileChar6Config == NULL) {
		return ( bleMemAllocError);
	}
	GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID,
			simpleProfileChar6Config);

	// Allocate Client Characteristic 7 Configuration table
	simpleProfileChar7Config = (gattCharCfg_t*) ICall_malloc(
			sizeof(gattCharCfg_t) *
			MAX_NUM_BLE_CONNS);
	if (simpleProfileChar7Config == NULL) {
		return ( bleMemAllocError);
	}
	GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID,
			simpleProfileChar7Config);

	if (services & SIMPLEPROFILE_SERVICE) {
		// Register GATT attribute list and CBs with GATT Server App
		status = GATTServApp_RegisterService(simpleProfileAttrTbl,
				GATT_NUM_ATTRS( simpleProfileAttrTbl ),
				GATT_MAX_ENCRYPT_KEY_SIZE, &simpleProfileCBs);
	} else {
		status = SUCCESS;
	}

	return (status);
}

/*********************************************************************
 * @fn      SimpleProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t SimpleProfile_RegisterAppCBs(simpleProfileCBs_t *appCallbacks) {
	if (appCallbacks) {
		simpleProfile_AppCBs = appCallbacks;

		return ( SUCCESS);
	} else {
		return ( bleAlreadyInRequestedMode);
	}
}

/*********************************************************************
 * @fn      SimpleProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_SetParameter(uint8 param, uint8 len, void *value) {
	bStatus_t ret = SUCCESS;
	switch (param) {
	case SIMPLEPROFILE_CHAR1:
		if (len == SIMPLEPROFILE_CHAR1_LEN) {
			VOID memcpy(simpleProfileChar1, value, SIMPLEPROFILE_CHAR1_LEN);
			GATTServApp_ProcessCharCfg(simpleProfileChar1Config,
					simpleProfileChar1, FALSE, simpleProfileAttrTbl,
					GATT_NUM_ATTRS(simpleProfileAttrTbl),
					INVALID_TASK_ID, simpleProfile_ReadAttrCB);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case SIMPLEPROFILE_CHAR2:
		if (len == SIMPLEPROFILE_CHAR2_LEN) {
			VOID memcpy(simpleProfileChar2, value, SIMPLEPROFILE_CHAR2_LEN);
			// See if Notification has been enabled
			GATTServApp_ProcessCharCfg(simpleProfileChar2Config,
					simpleProfileChar2, FALSE, simpleProfileAttrTbl,
					GATT_NUM_ATTRS(simpleProfileAttrTbl),
					INVALID_TASK_ID, simpleProfile_ReadAttrCB);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case SIMPLEPROFILE_CHAR3:
		if (len == SIMPLEPROFILE_CHAR3_LEN) {
			VOID memcpy(simpleProfileChar3, value, SIMPLEPROFILE_CHAR3_LEN);
			// See if Notification has been enabled
			GATTServApp_ProcessCharCfg(simpleProfileChar3Config,
					simpleProfileChar3, FALSE, simpleProfileAttrTbl,
					GATT_NUM_ATTRS(simpleProfileAttrTbl),
					INVALID_TASK_ID, simpleProfile_ReadAttrCB);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case SIMPLEPROFILE_CHAR4:
		if (len == SIMPLEPROFILE_CHAR4_LEN) {
			VOID memcpy(simpleProfileChar4, value, SIMPLEPROFILE_CHAR4_LEN);
			// See if Notification has been enabled
			GATTServApp_ProcessCharCfg(simpleProfileChar4Config,
					simpleProfileChar4, FALSE, simpleProfileAttrTbl,
					GATT_NUM_ATTRS(simpleProfileAttrTbl),
					INVALID_TASK_ID, simpleProfile_ReadAttrCB);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case SIMPLEPROFILE_CHAR5:
		if (len == SIMPLEPROFILE_CHAR5_LEN) {
			VOID memcpy(simpleProfileChar5, value, SIMPLEPROFILE_CHAR5_LEN);
			// See if Notification has been enabled
			GATTServApp_ProcessCharCfg(simpleProfileChar5Config,
					simpleProfileChar5, FALSE, simpleProfileAttrTbl,
					GATT_NUM_ATTRS(simpleProfileAttrTbl),
					INVALID_TASK_ID, simpleProfile_ReadAttrCB);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case SIMPLEPROFILE_CHAR6:
		if (len == SIMPLEPROFILE_CHAR6_LEN) {
			VOID memcpy(simpleProfileChar6, value, SIMPLEPROFILE_CHAR6_LEN);
			// See if Notification has been enabled
			GATTServApp_ProcessCharCfg(simpleProfileChar6Config,
					simpleProfileChar6, FALSE, simpleProfileAttrTbl,
					GATT_NUM_ATTRS(simpleProfileAttrTbl),
					INVALID_TASK_ID, simpleProfile_ReadAttrCB);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case SIMPLEPROFILE_CHAR7:
		if (len == SIMPLEPROFILE_CHAR7_LEN) {
			VOID memcpy(simpleProfileChar7, value, SIMPLEPROFILE_CHAR7_LEN);
			// See if Notification has been enabled
			GATTServApp_ProcessCharCfg(simpleProfileChar7Config,
					simpleProfileChar7, FALSE, simpleProfileAttrTbl,
					GATT_NUM_ATTRS(simpleProfileAttrTbl),
					selfEntity, simpleProfile_ReadAttrCB);
		} else {
			ret = bleInvalidRange;
		}
		break;

	default:
		ret = INVALIDPARAMETER;
		break;
	}

	return (ret);
}

/*********************************************************************
 * @fn      SimpleProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_GetParameter(uint8 param, void *value) {
	bStatus_t ret = SUCCESS;
	switch (param) {
	case SIMPLEPROFILE_CHAR1:
		VOID memcpy(value, simpleProfileChar1, SIMPLEPROFILE_CHAR1_LEN);
		break;

	case SIMPLEPROFILE_CHAR2:
		VOID memcpy(value, simpleProfileChar2, SIMPLEPROFILE_CHAR2_LEN);
		break;

	case SIMPLEPROFILE_CHAR3:
		VOID memcpy(value, simpleProfileChar3, SIMPLEPROFILE_CHAR3_LEN);
		break;

	case SIMPLEPROFILE_CHAR4:
		VOID memcpy(value, simpleProfileChar4, SIMPLEPROFILE_CHAR4_LEN);
		break;

	case SIMPLEPROFILE_CHAR5:
		VOID memcpy(value, simpleProfileChar5, SIMPLEPROFILE_CHAR5_LEN);
		break;

	case SIMPLEPROFILE_CHAR6:
		VOID memcpy(value, simpleProfileChar6, SIMPLEPROFILE_CHAR6_LEN);
		break;

	case SIMPLEPROFILE_CHAR7:
		VOID memcpy(value, simpleProfileChar7, SIMPLEPROFILE_CHAR7_LEN);
		break;

	default:
		ret = INVALIDPARAMETER;
		break;
	}

	return (ret);
}
#endif // USE_GATT_BUILDER
/*********************************************************************
 * @fn          simpleProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
bStatus_t simpleProfile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
		uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen,
		uint8_t method) {
	bStatus_t status = SUCCESS;

	// Make sure it's not a blob operation (no attributes in the profile are long)
	if (offset > 0) {
		return ( ATT_ERR_ATTR_NOT_LONG);
	}

	if (pAttr->type.len == ATT_BT_UUID_SIZE) {
		// 16-bit UUID
		uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
		switch (uuid) {
		// No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
		// gattserverapp handles those reads

		// include all read or notofication chars
		case SIMPLEPROFILE_CHAR1_UUID:
			*pLen = SIMPLEPROFILE_CHAR1_LEN;
			VOID memcpy(pValue, pAttr->pValue, SIMPLEPROFILE_CHAR1_LEN);
			break;
		case SIMPLEPROFILE_CHAR2_UUID:
			*pLen = SIMPLEPROFILE_CHAR2_LEN;
			VOID memcpy(pValue, pAttr->pValue, SIMPLEPROFILE_CHAR2_LEN);
			break;
		case SIMPLEPROFILE_CHAR3_UUID:
			*pLen = SIMPLEPROFILE_CHAR3_LEN;
			VOID memcpy(pValue, pAttr->pValue, SIMPLEPROFILE_CHAR3_LEN);
			break;
		case SIMPLEPROFILE_CHAR4_UUID:
			*pLen = SIMPLEPROFILE_CHAR4_LEN;
			VOID memcpy(pValue, pAttr->pValue, SIMPLEPROFILE_CHAR4_LEN);
			break;
		case SIMPLEPROFILE_CHAR5_UUID:
			*pLen = SIMPLEPROFILE_CHAR5_LEN;
			VOID memcpy(pValue, pAttr->pValue, SIMPLEPROFILE_CHAR5_LEN);
			break;
		case SIMPLEPROFILE_CHAR6_UUID:
			*pLen = SIMPLEPROFILE_CHAR6_LEN;
			VOID memcpy(pValue, pAttr->pValue, SIMPLEPROFILE_CHAR6_LEN);
			break;
		case SIMPLEPROFILE_CHAR7_UUID:
			*pLen = SIMPLEPROFILE_CHAR7_LEN;
			VOID memcpy(pValue, pAttr->pValue, SIMPLEPROFILE_CHAR7_LEN);
			break;

		default:
			// Should never get here! (characteristics 3 and 4 do not have read permissions)
			*pLen = 0;
			status = ATT_ERR_ATTR_NOT_FOUND;
			break;
		}
	} else {
		// 128-bit UUID
		*pLen = 0;
		status = ATT_ERR_INVALID_HANDLE;
	}

	return (status);
}

/*********************************************************************
 * @fn      simpleProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
bStatus_t simpleProfile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
		uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method) {
	bStatus_t status = SUCCESS;
	uint8 notifyApp = 0xFF;

	if (pAttr->type.len == ATT_BT_UUID_SIZE) {
		// 16-bit UUID
		uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
		switch (uuid) {
		case SIMPLEPROFILE_CHAR1_UUID:
		case SIMPLEPROFILE_CHAR3_UUID:
		case SIMPLEPROFILE_CHAR6_UUID:

			// Validate the value
			// kind of sloppy, should validate length for each, but requires refactoring
			if (offset == 0) {
				if (len > SIMPLEPROFILE_CHAR1_LEN
						&& len > SIMPLEPROFILE_CHAR3_LEN
						&& len > SIMPLEPROFILE_CHAR6_LEN) {
					status = ATT_ERR_INVALID_VALUE_SIZE;
				}
			} else {
				status = ATT_ERR_ATTR_NOT_LONG;
			}

			// Write the value
			if (status == SUCCESS) {
				uint8 *pCurValue = (uint8*) pAttr->pValue;
//				*pCurValue = pValue[0];
				memcpy(pCurValue, pValue, len);

				if (pAttr->pValue == simpleProfileChar1) {
					notifyApp = SIMPLEPROFILE_CHAR1;
				}
				if (pAttr->pValue == simpleProfileChar3) {
					notifyApp = SIMPLEPROFILE_CHAR3;
				}
				if (pAttr->pValue == simpleProfileChar6) {
					notifyApp = SIMPLEPROFILE_CHAR6;
				}
			}

			break;

		case GATT_CLIENT_CHAR_CFG_UUID:
			status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue,
					len, offset, GATT_CLIENT_CFG_NOTIFY | GATT_CLIENT_CFG_INDICATE);
			break;

		default:
			// Should never get here!
			status = ATT_ERR_ATTR_NOT_FOUND;
			break;
		}
	} else {
		// 128-bit UUID
		status = ATT_ERR_INVALID_HANDLE;
	}

	// If a characteristic value changed then callback function to notify application of change
	if ((notifyApp != 0xFF) && simpleProfile_AppCBs
			&& simpleProfile_AppCBs->pfnSimpleProfileChange) {
		simpleProfile_AppCBs->pfnSimpleProfileChange(notifyApp);
	}

	return (status);
}

/*********************************************************************
 *********************************************************************/
