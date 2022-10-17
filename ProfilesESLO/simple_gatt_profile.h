/******************************************************************************

 @file  simple_gatt_profile.h

 @brief This file contains the Simple GATT profile definitions and prototypes
        prototypes.

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

#ifndef SIMPLEGATTPROFILE_H
#define SIMPLEGATTPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
#ifndef USE_GATT_BUILDER
// Profile Parameters
#define SIMPLEPROFILE_CHAR1                   0  // LEDs
#define SIMPLEPROFILE_CHAR2                   1  // Vitals
#define SIMPLEPROFILE_CHAR3                   2  // Settings
#define SIMPLEPROFILE_CHAR4                   3  // EEG
#define SIMPLEPROFILE_CHAR5                   4  // AXY
#define SIMPLEPROFILE_CHAR6                   5  // Addr
#define SIMPLEPROFILE_CHAR7                   6  // SWA

// Simple Profile Service UUID
#define SIMPLEPROFILE_SERV_UUID             0xE000

#define SIMPLEPROFILE_CHAR1_UUID            0xE001
#define SIMPLEPROFILE_CHAR2_UUID            0xE002
#define SIMPLEPROFILE_CHAR3_UUID            0xE003
#define SIMPLEPROFILE_CHAR4_UUID            0xE004
#define SIMPLEPROFILE_CHAR5_UUID            0xE005
#define SIMPLEPROFILE_CHAR6_UUID            0xE006
#define SIMPLEPROFILE_CHAR7_UUID            0xE007

// Simple Keys Profile Services bit fields
#define SIMPLEPROFILE_SERVICE               0x00000001

// Length of Characteristic in bytes
#define SIMPLEPROFILE_CHAR1_LEN			  1		// LED_0-3
#define SIMPLEPROFILE_CHAR2_LEN			  17	// vBatt, low battery status, therm, esloAddr, axyLog (uint8)
#define SIMPLEPROFILE_CHAR3_LEN			  17	// settings
#define SIMPLEPROFILE_CHAR4_LEN			  248 	// 62 int32, EEG (eslo packets)
#define SIMPLEPROFILE_CHAR5_LEN           16	// !! was 128 AXY (eslo packets)
#define SIMPLEPROFILE_CHAR6_LEN			  4		// Addr
#define SIMPLEPROFILE_CHAR7_LEN			  16	// SWA

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*simpleProfileChange_t)( uint8 paramID );

typedef struct
{
  simpleProfileChange_t        pfnSimpleProfileChange;  // Called when characteristic value changes
} simpleProfileCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * SimpleProfile_AddService- Initializes the Simple GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t SimpleProfile_AddService( uint32 services );

/*
 * SimpleProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t SimpleProfile_RegisterAppCBs( simpleProfileCBs_t *appCallbacks );

/*
 * SimpleProfile_SetParameter - Set a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value );

/*
 * SimpleProfile_GetParameter - Get a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t SimpleProfile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/
#endif // USE_GATT_BUILDER

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEGATTPROFILE_H */
