/******************************************************************************

 @file  simple_peripheral.c

 Matt Gaidica, ESLO_RB2 device
 https://dev.ti.com/tirex/content/simplelink_cc13x0_sdk_3_20_00_23/docs/proprietary-rf/proprietary-rf-users-guide/proprietary-rf-guide/debugging-index.html#deciphering-cpu-exceptions
 https://training.ti.com/sites/default/files/docs/TIRTOS_CCSDebugging.pdf
 ******************************************************************************/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <math.h> // atan2(x,y), M_PI

// CMSIS Math
#include "arm_math.h"
#include "arm_const_structs.h"

#include <ti/drivers/ADC.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/NVS.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/UART.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#if (!(defined __TI_COMPILER_VERSION__) && !(defined __GNUC__))
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util_eslo.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <simple_gatt_profile.h>

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <ti_drivers_config.h>
#include "simple_peripheral.h"
#include "ti_ble_config.h"
#include <ESLO.h>

#ifdef PTM_MODE
#include "npi_task.h"               // To allow RX event registration
#include "npi_ble.h"                // To enable transmission of messages to UART
#include "icall_hci_tl.h"   // To allow ICall HCI Transport Layer
#endif // PTM_MODE

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// How often to perform periodic event (in ms)
#define ES_VITALS_EVT_PERIOD                 3000	// ms
#define ES_PERIODIC_EVT_PERIOD				 1000	// ms
#define ES_LOG_PERIODIC					     60 // x ES_PERIODIC_EVT_PERIOD
#define ES_AXY_PERIOD				 		 1000	// ms
#define ES_ADV_SLEEP_TIMEOUT_MIN			 30		// s
#define ES_ADV_SLEEP_TIMEOUT_MAX			 60		// s
#define ES_ADV_AWAKE_PERIOD					 3  	// s
#define ES_BUFFER_SIZE						 10		// @1Hz = s
#define ES_IND_LOOP_TIMEOUT					 100 // ms

// Task configuration
#define SP_TASK_PRIORITY                     1

#ifndef SP_TASK_STACK_SIZE
#define SP_TASK_STACK_SIZE                   1024
#endif

// Application events
#define SP_CHAR_CHANGE_EVT                   0
#define SP_ADV_EVT                           1
#define SP_PAIR_STATE_EVT                    2
#define SP_PASSCODE_EVT                      3
#define SP_READ_RPA_EVT                      4
#define SP_SEND_PARAM_UPDATE_EVT             5
#define SP_CONN_EVT                          6
#define ES_PERIODIC_EVT                      7
#define ES_VITALS_EVT						 8
#define ES_EEG_NOTIF						 9
#define ES_XL_NOTIF							 10
#define ES_AXY_EVT						     11
#define ES_ADV_SLEEP					     12
#define ES_REC_PERIOD 						 13
#define ES_REC_DURATION						 14
#define ES_SHIP_SWA							 15
#define ES_DATA_TIMEOUT						 16
#define ES_FORCE_DISCONNECT					 17
#define ES_SPEAKER_DELAY				     18

// Internal Events for RTOS application
#define SP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define SP_ALL_EVENTS                        (SP_ICALL_EVT             | \
                                              SP_QUEUE_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SP_ADDR_STR_SIZE     15

// For storing the active connections
#define SP_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define SP_MAX_RSSI_STORE_DEPTH    5
#define SP_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30           
#define RSSI_1M_THRSHLD           -40           
#define RSSI_S2_THRSHLD           -50           
#define RSSI_S8_THRSHLD           -60           
#define SP_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define SIMPLEPERIPHERAL_ASSERT(expr) if (!(expr)) simple_peripheral_spin();

/*********************************************************************
 * TYPEDEFS
 */

// Auto connect
enum {
	AUTOCONNECT_DISABLE = 0,              // Disable
};

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct {
	uint8_t event;                // event type
	void *pData;               // pointer to message
} spEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct {
	uint8_t state;
	uint16_t connHandle;
	uint8_t status;
} spPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct {
	uint8_t deviceAddr[B_ADDR_LEN];
	uint16_t connHandle;
	uint8_t uiInputs;
	uint8_t uiOutputs;
	uint32_t numComparison;
} spPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct {
	uint32_t event;
	void *pBuf;
} spGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct {
	uint8_t event;                //
	uint8_t data[];
} spClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct {
	List_Elem elem;
	uint16_t connHandle;
} spConnHandleEntry_t;

// Connected device information
typedef struct {
	uint16_t connHandle;                        // Connection Handle
	spClockEventData_t *pParamUpdateEventData;
	Clock_Struct *pUpdateClock;                      // pointer to clock struct
	int8_t rssiArr[SP_MAX_RSSI_STORE_DEPTH];
	uint8_t rssiCntr;
	int8_t rssiAvg;
	bool phyCngRq;           // Set to true if PHY change request is in progress
	uint8_t currPhy;
	uint8_t rqPhy;
	uint8_t phyRqFailCnt;                      // PHY change request count
	bool isAutoPHYEnable;                   // Flag to indicate auto phy change
} spConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task configuration
Task_Struct spTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(spTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t spTaskStack[SP_TASK_STACK_SIZE];

//#define APP_EVT_EVENT_MAX 0x9
//char *appEventStrings[] = { "APP_STATE_CHANGE_EVT     ",
//		"APP_CHAR_CHANGE_EVT      ", "APP_KEY_CHANGE_EVT       ",
//		"APP_ADV_EVT              ", "APP_PAIR_STATE_EVT       ",
//		"APP_PASSCODE_EVT         ", "APP_READ_RPA_EVT         ",
//		"APP_PERIODIC_EVT         ", "APP_SEND_PARAM_UPDATE_EVT",
//		"APP_CONN_EVT             ", };

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;
spClockEventData_t argRpaRead = { .event = SP_READ_RPA_EVT };

// Per-handle connection info
static spConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Auto connect Disabled/Enabled {0 - Disabled, 1- Group A , 2-Group B, ...}
uint8_t autoConnect = AUTOCONNECT_DISABLE;

// Advertising handles
static uint8 advHandleLegacy;
static uint8 advHandleLongRange;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = { 0 };

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init(void);
static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg);
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf,
		uintptr_t arg);
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData);
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg);
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId);
static void SimplePeripheral_notifyVitals(void);
static void SimplePeripheral_updateRPA(void);
static void SimplePeripheral_clockHandler(UArg arg);
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr,
		uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs,
		uint32_t numComparison);
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
		uint8_t status);
static void SimplePeripheral_processPairState(spPairStateData_t *pPairState);
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData);
static void SimplePeripheral_charValueChangeCB(uint8_t paramId);
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData);
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void SimplePeripheral_initPHYRSSIArray(void);
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t SimplePeripheral_addConn(uint16_t connHandle);
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle);
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle);
static void SimplePeripheral_processParamUpdate(uint16_t connHandle);
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
		uint8_t txPhy, uint8_t rxPhy, uint16_t phyOpts);
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle);
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t SimplePeripheral_BondMgrCBs = { SimplePeripheral_passcodeCb, // Passcode callback
		SimplePeripheral_pairStateCb       // Pairing/Bonding state Callback
		};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t SimplePeripheral_simpleProfileCBs = {
		SimplePeripheral_charValueChangeCB // Simple GATT Characteristic value change callback
		};

/*********************************************************************
 * ESLO FUNCTIONS
 */

static uint8_t updateEEGFromSettings(bool actOnInterrupt);
static void eegInterrupt(bool enableInterrupt);

void updateXlFromSettings(); // always on
//static void xlInterrupt(bool enableInterrupt);

static uint8_t USE_EEG(uint8_t *esloSettings);
//static uint8_t USE_AXY(uint8_t *esloSettings);

static void mapEsloSettings(uint8_t *esloSettingsNew);
static void ESLO_performPeriodicTask();

static void esloSetVersion();
static void readBatt();
static void readTherm();
static void esloRecoverSession();
static void esloUpdateNVS();
static void esloResetVersion();
//static void WatchdogCallbackFxn();
static void advSleep();
static void resetSWA();

static Clock_Struct clkESLOPeriodic;
spClockEventData_t argESLOPeriodic = { .event = ES_PERIODIC_EVT };

static Clock_Struct clkESLOAxy;
spClockEventData_t argESLOAxy = { .event = ES_AXY_EVT };

static Clock_Struct clkESLOAdvSleep;
spClockEventData_t argESLOAdvSleep = { .event = ES_ADV_SLEEP };
uint8_t isAdvLong = 0;

static Clock_Struct clkNotifyVitals;
spClockEventData_t argESLOVitals = { .event = ES_VITALS_EVT };

static Clock_Struct clkESLORecDuration;
spClockEventData_t argESLORecDuration = { .event = ES_REC_DURATION };

static Clock_Struct clkESLORecPeriod;
spClockEventData_t argESLORecPeriod = { .event = ES_REC_PERIOD };

static Clock_Struct clkESLODataTimeout;
spClockEventData_t argESLODataTimeout = { .event = ES_DATA_TIMEOUT };

static Clock_Struct clkESLOSpeakerDelay;
spClockEventData_t argESLOSpeakerDelay = { .event = ES_SPEAKER_DELAY };

uint8_t esloSettings[SIMPLEPROFILE_CHAR3_LEN] = { 0 };
uint8_t esloSettingsSleep[SIMPLEPROFILE_CHAR3_LEN] = { 0 };

bool isPaired = false;

uint32_t lowVoltage; // acts as boolean, use int32 to unwrap easily in app
uint32_t vbatt_uV;
ADC_Handle adc_vBatt;
ADC_Params adcParams_vBatt;

int32_t temp_uC;
ADC_Handle adc_therm;
ADC_Params adcParams_therm;

int_fast16_t adcRes;
uint16_t adcValue;

uint32_t absoluteTime = 0;
uint32_t esloVersion = 0;
uint32_t axyCount = 0;
uint32_t eegCount = 0;

int32_t eeg1Buffer[PACKET_SZ_EEG];
int32_t eeg2Buffer[PACKET_SZ_EEG];
int32_t eeg3Buffer[PACKET_SZ_EEG];
int32_t eeg4Buffer[PACKET_SZ_EEG];
int32_t swaBuffer[SWA_LEN * 2] = { 0 };
uint32_t iSWA = 0;
uint8_t iEEG = 0;
uint8_t iEEGDiv = 0;

uint8_t vitalsBuffer[SIMPLEPROFILE_CHAR2_LEN];

/* AXY Vars */
int32_t xlXBuffer[PACKET_SZ_XL];
int32_t xlYBuffer[PACKET_SZ_XL];
int32_t xlZBuffer[PACKET_SZ_XL];
uint8_t iXL = 0;
uint8_t moveCount = 0;
float_t MoveXBuffer[ES_BUFFER_SIZE] = { 0 };
float_t MoveYBuffer[ES_BUFFER_SIZE] = { 0 };
float_t MoveZBuffer[ES_BUFFER_SIZE] = { 0 };
uint8_t isMoving = 0;

/* NAND Vars */
uint8_t ret;
uint16_t devId;
uint8_t esloBuffer[PAGE_DATA_SIZE]; // used for writing
uint8_t readBuf[PAGE_SIZE]; // 2176, always allocate full page size
uint32_t packet;
uAddrType esloAddr, esloExportBlock;
uint8_t iLog = 0;

/* ADS129X Vars */
int32_t status;
int32_t ch1, ch2, ch3, ch4;

Watchdog_Params watchdogParams;
Watchdog_Handle watchdogHandle;

UART_Handle uart = NULL;
uint8_t xl_online = ESLO_FAIL;
uint8_t eeg_online = ESLO_FAIL;
uint8_t mem_online = ESLO_FAIL;

stmdev_ctx_t dev_ctx_xl;

static uint32_t nvsBuffer[3]; // esloSignature, esloVersion, esloAddr
NVS_Handle nvsHandle;

static float32_t complexFFT[FFT_LEN], realFFT[FFT_HALF_LEN],
		imagFFT[FFT_HALF_LEN], angleFFT[FFT_HALF_LEN], powerFFT[FFT_HALF_LEN];
float32_t swaFFT[FFT_LEN] = { 0 };
float32_t Fs, stepSize, Fc;

////Band#       Frequencies (Hz)     Att/Ripple (dB)
//    1         0.000,    0.500         60.000
//    2         0.800,   12.000          1.000
//    3        15.000,   62.500         60.000
////
// Arithmetic = 'Floating Point (Double Precision)';
// Architecture = 'IIR';
// Structure = 'Direct Form II Transposed';
// Response = 'Bandpass';
// Method = 'Butterworth';
// Biquad = 'Yes';
// Stable = 'Yes';
// Fs = 125.0000; //Hz
// Filter Order = 4;

float32_t filtInput[SWA_LEN], filtOutput[SWA_LEN];
float32_t *InputValuesf32_ptr = &filtInput[0];  // declare Input pointer
float32_t *OutputValuesf32_ptr = &filtOutput[0]; // declare Output pointer

#define NUM_SECTIONS_IIR 2

float32_t iirCoeffsf32[NUM_SECTIONS_IIR * 5] = { // b0, b1, b2, a1, a2
		0.06076204501892, 0.12152409003785, 0.06076204501892, 1.22223307244950,
				-0.46699938172924, 0.96615019075240, -1.93230038150480,
				0.96615019075240, 1.94530596833882, -0.94697940426943 };

#define BLOCKSIZE 32
#define NUMBLOCKS  (SWA_LEN/BLOCKSIZE)
float32_t iirStatesf32[NUM_SECTIONS_IIR * 2];
arm_biquad_cascade_df2T_instance_f32 Sf;

uint32_t fftSize = FFT_LEN;
uint32_t ifftFlag = 0;
arm_rfft_fast_instance_f32 S;
uint32_t maxIndex = 0;
float32_t maxValue;
uint32_t timeElapsed;
//arm_status armStatus;

uint8_t SWAsent = 0;
uint16_t iSWAfill = 0;
uint16_t iIndication = 0; // used for indications
uint8_t hasMovedSinceReset = 0; // for shelf mode
uint8_t triedDisconnecting = 0;
uint8_t isShippingSwa = 0;

static uint8_t central_isSpeaker = 0;
static uint8_t ESLO_SPEAKER_ADDR[6] = { 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0 };
uint32_t SWATrial = 1;

#include <ti/sysbios/family/arm/m3/Hwi.h>
volatile uintptr_t *excPC = 0;
volatile uintptr_t *excCaller = 0;
void execHandlerHook(Hwi_ExcContext *ctx) {
	excPC = ctx->pc;     // Program counter where exception occurred
	excCaller = ctx->lr; // Link Register when exception occurred

	while (2)
		;
}

static float32_t ESLO_ADSgain_uV(int32_t rawValue) {
	float32_t rawFloat = (float32_t) rawValue;
	float32_t refV = (VREF / ADS_GAIN) / 8388607;
	float32_t empiricMult = 1.6667;
	float32_t uV = rawFloat * (refV) * 1000000 * empiricMult; // 5/3 is empiric, see MATLAB
	return uV;
}

static void resetSWA() {
	iIndication = 0;
	iSWA = 0;
	SWAsent = 0; // see ATT_HANDLE_VALUE_CFM
	isShippingSwa = 0;
	iSWAfill = 0;
	memset(swaBuffer, 0, sizeof(int32_t) * SWA_LEN * 2);
}

// iIndication == 1 from the stim SimpleProfile_SetParameter(), so if this enters with
// iIndication == 0, stim was not ack'd, therefore disconnect
static void shipSWA() {
	eslo_dt swa_trial, eslo_eeg;

	// should never happen
	if (iIndication == 0) {
		if (triedDisconnecting == 0) {
			triedDisconnecting = 1; // avoid overflowing event handler from EEG area
			SimplePeripheral_enqueueMsg(ES_FORCE_DISCONNECT, NULL);
		}
	} else {
		if (iIndication == 1) {
			SWATrial++;
			swa_trial.type = Type_SWATrial;
			swa_trial.data = SWATrial;
			ESLO_Write(&esloAddr, esloBuffer, esloVersion, swa_trial);
		}
		// keep this loop active (skip reset) until no indications within period
		Util_rescheduleClock(&clkESLODataTimeout, ES_IND_LOOP_TIMEOUT, 0);
		if (iIndication < (SWA_LEN * 2) / 4 + 1) { // avoid overflow, +1 for stim
			uint32_t charData[4]; // SIMPLEPROFILE_CHAR7_LEN is 16, so send 4 uint32's each loop
			switch (esloSettings[Set_SWA]) {
			case 1:
				eslo_eeg.type = Type_EEG1;
				break;
			case 2:
				eslo_eeg.type = Type_EEG2;
				break;
			case 3:
				eslo_eeg.type = Type_EEG3;
				break;
			case 4:
				eslo_eeg.type = Type_EEG4;
				break;
			}

			for (uint8_t iPacket = 0; iPacket < 4; iPacket++) {
				eslo_eeg.data = swaBuffer[(iIndication - 1) * 4 + iPacket];
				ESLO_Packet(eslo_eeg, &packet);
				charData[iPacket] = packet;
			}
			SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR7,
			SIMPLEPROFILE_CHAR7_LEN, charData);
		}
	}
}

static void advSleep() {
	if (isAdvLong) { // wake-up, enable advertise for short period
		Util_restartClock(&clkESLOAdvSleep, ES_ADV_AWAKE_PERIOD * 1000);
		// check axy here and see if Z position is near 0
		// also make sure Axy is on!
		if (isMoving > 0) {
			hasMovedSinceReset = 1; // requires reset push
		}
		if (hasMovedSinceReset > 0 || xl_online == false) {
			GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX,
					0);
		}
		isAdvLong = 0;
	} else {
		// sleep rnd amt to avoid advertisement syncing
		uint8_t rndSleep = (rand()
				% (ES_ADV_SLEEP_TIMEOUT_MAX - ES_ADV_SLEEP_TIMEOUT_MIN + 1))
				+ ES_ADV_SLEEP_TIMEOUT_MIN;
		Util_restartClock(&clkESLOAdvSleep, (uint32_t) rndSleep * 1000); // back to millis
		GapAdv_disable(advHandleLongRange);
		GapAdv_disable(advHandleLegacy);
		isAdvLong = 1;
	}
}

// !! reset ESLO settings
static void esloResetVersion() {
	isMoving = 0;
	hasMovedSinceReset = 0;
	moveCount = 0;
	SWATrial = 1;
	esloAddr = 0; // comes first, so NAND first entry is version
	nvsHandle = NVS_open(ESLO_NVS_0, NULL);
	if (nvsHandle != NULL) {
		esloSetVersion();
		ESLO_encodeNVS(nvsBuffer, &ESLOSignature, &esloVersion, &esloAddr);
		NVS_write(nvsHandle, 0, (void*) nvsBuffer, sizeof(nvsBuffer),
		NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
		NVS_close(nvsHandle);
	}
}

// called on periodic (1/min) !! also save ESLO settings
static void esloUpdateNVS() {
	nvsHandle = NVS_open(ESLO_NVS_0, NULL);
	if (nvsHandle != NULL) {
		ESLO_encodeNVS(nvsBuffer, &ESLOSignature, &esloVersion, &esloAddr);
		NVS_write(nvsHandle, 0, (void*) nvsBuffer, sizeof(nvsBuffer),
		NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
		NVS_close(nvsHandle);
	}
}

// set esloAddr and esloVersion, called only at startup
static void esloRecoverSession() {
	uint32_t tempSignature;
	uint32_t tempVersion;
	uint32_t tempAddress;
	bool doVersion = false;

	nvsHandle = NVS_open(ESLO_NVS_0, NULL);
	if (nvsHandle != NULL) {
		NVS_read(nvsHandle, 0, (void*) nvsBuffer, sizeof(nvsBuffer)); // 0 offset
		// compare eslo sig
		ESLO_decodeNVS(nvsBuffer, &tempSignature, &tempVersion, &tempAddress);
		if (tempSignature == ESLOSignature) {
			esloVersion = tempVersion;
			esloAddr = tempAddress;
		} else {
			doVersion = true;
		}
		NVS_close(nvsHandle);
	} else {
		doVersion = true;
	}

	if (doVersion) {
		esloResetVersion();
	}
}

static void esloSetVersion() {
	eslo_dt eslo;
//	esloVersion = GitCommit; // this is semi-static
	ESLO_GenerateVersion(&esloVersion, CONFIG_TRNG_1);
	eslo.type = Type_Version;
	eslo.data = esloVersion;
	ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);
}

static void readTherm() {
	adcRes = ADC_convert(adc_therm, &adcValue);
	if (adcRes == ADC_STATUS_SUCCESS) {
		// read, multiply by 2 for voltage divider
		temp_uC = ESLO_convertTherm(
				ADC_convertToMicroVolts(adc_therm, adcValue));
	}
}

// vbatt_uV will never exceed 24-bits
static void readBatt() {
	adcRes = ADC_convert(adc_vBatt, &adcValue);
	if (adcRes == ADC_STATUS_SUCCESS) {
		vbatt_uV = ESLO_convertBatt(
				ADC_convertToMicroVolts(adc_vBatt, adcValue));
		if (lowVoltage == 0 || vbatt_uV < lowVoltage) {
			lowVoltage = vbatt_uV;
		}
	}
}

// sleep should only be called internally
// ...mapEsloSettings() is called when central pushes
static void esloSleep() {
	// right now zeros and sleep mode are same
	uint8_t esloSettingsNew[SIMPLEPROFILE_CHAR3_LEN] = { 0 };
	// carry over these settings
	esloSettingsNew[Set_AdvLong] = esloSettings[Set_AdvLong];
	// overwrite esloSettings and force sleep mode to take effect
	mapEsloSettings(esloSettingsNew);

	// tell memory recording has stopped
	eslo_dt eslo;
	eslo.type = Type_EEGState;
	eslo.data = 0;
	ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);
}

static void mapEsloSettings(uint8_t *esloSettingsNew) {
	eslo_dt eslo;
	// resetVersion only comes from iOS, never maintains value (one and done)
	if (esloSettingsNew[Set_ResetVersion] > 0) {
		esloResetVersion();
	}

	if (esloSettings[Set_RecPeriod] != *(esloSettingsNew + Set_RecPeriod)) {
		esloSettings[Set_RecPeriod] = *(esloSettingsNew + Set_RecPeriod);
	}

	if (esloSettings[Set_RecDuration] != *(esloSettingsNew + Set_RecDuration)) {
		esloSettings[Set_RecDuration] = *(esloSettingsNew + Set_RecDuration);
	}

// this needs some logic: we will never write abstime=0 here
// but cond can occur when the settings are mapped from wakeup
	if (esloSettingsNew[Set_Time1] | esloSettingsNew[Set_Time2]
			| esloSettingsNew[Set_Time3] | esloSettingsNew[Set_Time4] > 0) {
		memcpy(&absoluteTime, esloSettingsNew + Set_Time1, 4);
		eslo.type = Type_AbsoluteTime;
		eslo.data = absoluteTime;
		ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);
	}

	// can't happen when connected, see: GAP_LINK_TERMINATED_EVENT
	if (esloSettings[Set_AdvLong] != *(esloSettingsNew + Set_AdvLong)) {
		esloSettings[Set_AdvLong] = *(esloSettingsNew + Set_AdvLong);
	}

	if (esloSettings[Set_Record] != *(esloSettingsNew + Set_Record)) {
		esloSettings[Set_Record] = *(esloSettingsNew + Set_Record);
	}
//	if (esloSettings[Set_TxPower] != *(esloSettingsNew + Set_TxPower)) {
//		esloSettings[Set_TxPower] = *(esloSettingsNew + Set_TxPower);
//		switch (*(esloSettingsNew + Set_TxPower)) {
//		case 0:
//			HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_20_DBM);
//			break;
//		case 1:
//			HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_10_DBM);
//			break;
//		case 2:
//			HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
//			break;
//		case 3:
//			HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
//			break;
//		default:
//			break;
//		}
//	}
	if (esloSettings[Set_SWAThresh] != *(esloSettingsNew + Set_SWAThresh)) {
		esloSettings[Set_SWAThresh] = *(esloSettingsNew + Set_SWAThresh);
	}
	if (esloSettings[Set_SWARatio] != *(esloSettingsNew + Set_SWARatio)) {
		esloSettings[Set_SWARatio] = *(esloSettingsNew + Set_SWARatio);
	}
	if (esloSettings[Set_SWA] != *(esloSettingsNew + Set_SWA)) {
		esloSettings[Set_SWA] = *(esloSettingsNew + Set_SWA);
		resetSWA();
	}
	if (esloSettings[Set_AxyMode] != *(esloSettingsNew + Set_AxyMode)) {
		// set it first, Xl function uses them
		esloSettings[Set_AxyMode] = *(esloSettingsNew + Set_AxyMode);
		updateXlFromSettings();
	}
	if (esloSettings[Set_EEG1] != *(esloSettingsNew + Set_EEG1)
			| esloSettings[Set_EEG2] != *(esloSettingsNew + Set_EEG2)
			| esloSettings[Set_EEG3] != *(esloSettingsNew + Set_EEG3)
			| esloSettings[Set_EEG4] != *(esloSettingsNew + Set_EEG4)) {
		// set them first, EEG function uses them
		esloSettings[Set_EEG1] = *(esloSettingsNew + Set_EEG1);
		esloSettings[Set_EEG2] = *(esloSettingsNew + Set_EEG2);
		esloSettings[Set_EEG3] = *(esloSettingsNew + Set_EEG3);
		esloSettings[Set_EEG4] = *(esloSettingsNew + Set_EEG4);
		updateEEGFromSettings(true);
	}

// set and notify iOS, since export data now overrides some settings
	SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN,
			esloSettings);
}

static uint8_t USE_EEG(uint8_t *esloSettings) {
	return (esloSettings[Set_EEG1] | esloSettings[Set_EEG2]
			| esloSettings[Set_EEG3] | esloSettings[Set_EEG4]) & 1;
}

//static uint8_t USE_AXY(uint8_t *esloSettings) {
//	uint8_t retAxy = 0;
//	if (esloSettings[Set_AxyMode] > 0) {
//		retAxy = 1;
//	}
//	return retAxy;
//}

// gates in place, see eegDataReady
static void eegDataHandler(void) {
	eslo_dt eslo_eeg1, eslo_eeg2, eslo_eeg3, eslo_eeg4;
	eegCount++; // turned off when EEG is turned off

	if (USE_EEG(esloSettings)
			== ESLO_MODULE_ON&& eegCount > EEG_STARTUP_SAMPLES) { // double check

		ADS_updateData(&status, &ch1, &ch2, &ch3, &ch4);
		if (status == 0) {
			return; // wonder if this is ever encountered?
		}

		if (esloSettings[Set_SWA] > 0 && isPaired == 1
				&& central_isSpeaker == 1) {

			if (esloSettings[Set_SWA] > 0
					&& (isMoving & (uint8_t) AXY_HAS_MOVED_EEG) > 0
					&& triedDisconnecting == 0
					&& esloSettings[Set_SWAThresh] > 0
					&& esloSettings[Set_SWARatio] > 0) {
				triedDisconnecting = 1; // avoid overflowing event handler from EEG area
				SimplePeripheral_enqueueMsg(ES_FORCE_DISCONNECT, NULL);
				return;
			}

			if (SWAsent == 0 && iIndication == 0) {
				// shift data and pop sample onto front of array
				for (int iShift = 1; iShift < SWA_LEN; iShift++) {
					swaBuffer[iShift - 1] = swaBuffer[iShift];
				}
				switch (esloSettings[Set_SWA]) {
				case 1:
					swaBuffer[SWA_LEN - 1] = ch1;
					break;
				case 2:
					swaBuffer[SWA_LEN - 1] = ch2;
					break;
				case 3:
					swaBuffer[SWA_LEN - 1] = ch3;
					break;
				case 4:
					swaBuffer[SWA_LEN - 1] = ch4;
					break;
				}
				iSWA++;
				if (iSWA > SWA_LEN && iSWA % 20 == 0) { // shouldn't this happen more often? ~50ms?
					timeElapsed = Clock_getTicks();
					uint32_t k, iStep;
					uint32_t swaDiv = 2;

					// convert to float for CMSIS functions
					float32_t filtSum = 0;
					for (k = 0; k < SWA_LEN; k++) {
						//			filtInput[k] = (float32_t) swaBuffer[k];
						filtInput[k] = ESLO_ADSgain_uV(swaBuffer[k]);
						filtSum += filtInput[k];
					}
					// remove DC component
					float32_t mean_uV = filtSum / SWA_LEN;
					for (k = 0; k < SWA_LEN; k++) {
						filtInput[k] = filtInput[k] - mean_uV;
					}

					// Initialise Biquads
					arm_biquad_cascade_df2T_init_f32(&Sf, NUM_SECTIONS_IIR,
							&(iirCoeffsf32[0]), &(iirStatesf32[0]));

					// Perform IIR filtering operation
					for (k = 0; k < NUMBLOCKS; k++)
						arm_biquad_cascade_df2T_f32(&Sf,
								InputValuesf32_ptr + (k * BLOCKSIZE),
								OutputValuesf32_ptr + (k * BLOCKSIZE),
								BLOCKSIZE); // perform filtering

					// find max uV of filtered signal
					float32_t max_uV = 0;
					for (k = 0; k < SWA_LEN; k++) {
						if (fabs(filtOutput[k]) > max_uV) {
							max_uV = fabs(filtOutput[k]);
						}
					}

					// is the filtered max amplitude above user thresh
					if (max_uV < (float32_t) esloSettings[Set_SWAThresh]) {
						return;
					}

					// only init once
					arm_rfft_fast_init_f32(&S, fftSize);

					// subsample to reduce Fs and make FFT more accurate for SWA freqs
					while (1) {
						memset(swaFFT, 0, sizeof(float32_t) * FFT_LEN); // swaFFT is manipulated in place, always reset to zero
						if (swaDiv == 2) {
							for (k = 0; k < SWA_LEN / swaDiv; k++) {
								swaFFT[k] = filtOutput[k * swaDiv]; // subsample
							}
						} else {
							for (k = 0; k < SWA_LEN / 2; k++) {
								swaFFT[k] = filtOutput[k + (SWA_LEN / 2)]; // memcpy from tail
							}
						}

						// input is real, output is interleaved real and complex
						arm_rfft_fast_f32(&S, swaFFT, complexFFT, ifftFlag);

						// compute power
						arm_cmplx_mag_squared_f32(complexFFT, powerFFT,
						FFT_HALF_LEN);
						arm_max_f32(powerFFT, FFT_HALF_LEN, &maxValue,
								&maxIndex);

						Fs = EEG_FS / EEG_SAMPLING_DIV / swaDiv; // effective Fs
						stepSize = (Fs / 2) / FFT_HALF_LEN;
						Fc = stepSize * maxIndex;

						// only reject Fc if thresh and ratio are set
						if (esloSettings[Set_SWAThresh] > 0
								&& esloSettings[Set_SWARatio] > 0) {
							if (Fc < SWA_F_MIN || Fc >= SWA_F_MAX) {
								return;
							}
						}
						if (Fc < 2 || swaDiv == 1) {
							break;
						} // retry swaDiv=1
						swaDiv--;
					}

					float32_t SWA_mean = 0;
					float32_t THETA_mean = 0;
					uint16_t SWA_count = 0;
					uint16_t THETA_count = 0;
					for (iStep = 0; iStep < FFT_HALF_LEN; iStep++) {
						if (stepSize * iStep >= SWA_F_MIN
								&& stepSize * iStep < SWA_F_MAX) {
							SWA_mean = SWA_mean + powerFFT[iStep];
							SWA_count++;
						}
						if (stepSize * iStep >= THETA_F_MIN
								&& stepSize * iStep < THETA_F_MAX) {
							THETA_mean = THETA_mean + powerFFT[iStep];
							THETA_count++;
						}
					}
					SWA_mean = SWA_mean / (float32_t) SWA_count;
					THETA_mean = THETA_mean / (float32_t) THETA_count;
					float32_t swaRatio = SWA_mean / THETA_mean;

					// is the SWA/THETA ratio above user thresh?
					if (swaRatio < (float32_t) esloSettings[Set_SWARatio]) {
						return;
					}

					// redo FFT on raw data to get true phase
					memset(swaFFT, 0, sizeof(float32_t) * FFT_LEN);
					if (swaDiv == 2) {
						for (k = 0; k < SWA_LEN / swaDiv; k++) {
							swaFFT[k] = (float32_t) swaBuffer[k * swaDiv]; // subsample
						}
					} else {
						for (k = 0; k < SWA_LEN / 2; k++) {
							swaFFT[k] =
									(float32_t) swaBuffer[k + (SWA_LEN / 2)]; // memcpy from tail
						}
					}
					arm_rfft_fast_f32(&S, swaFFT, complexFFT, ifftFlag);

					// de-interleave real and complex values, used in atan() for phase
					for (k = 0; k <= (FFT_LEN / 2) - 1; k++) {
						realFFT[k] = complexFFT[k * 2];
						imagFFT[k] = complexFFT[(k * 2) + 1];
					}
					// find angle of FFT
					for (k = 0; k <= FFT_LEN / 2; k++) {
						angleFFT[k] = atan2f(imagFFT[k], realFFT[k]);
					}

					float32_t degSec = 360 * Fc;
					float32_t windowLength = SWA_LEN / (Fs * EEG_SAMPLING_DIV);
					timeElapsed = (Clock_getTicks() - timeElapsed)
							* Clock_tickPeriod;
					float32_t computeDegrees = degSec * (float32_t) timeElapsed
							/ 1000000;
					float32_t endAngle = degSec * windowLength
							+ (angleFFT[maxIndex] * 180 / M_PI)
							+ computeDegrees;

					int32_t phaseAngle = (int32_t) (1000 * endAngle)
							% (360 * 1000);
					int32_t dominantFreq = (int32_t) (Fc * 1000);
					// SIMPLEPROFILE_CHAR7_LEN = 16 bytes, first int32 is SWA stim flag
					int32_t swaCharData[4] = { absoluteTime, dominantFreq,
							phaseAngle, SWATrial };

					SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR7,
					SIMPLEPROFILE_CHAR7_LEN, swaCharData);

					SWAsent = 1;
					iSWAfill = SWA_LEN;
				}
			} else if (SWAsent == 1 & iSWAfill < 2 * SWA_LEN) { // SWA sent
				switch (esloSettings[Set_SWA]) {
				case 1:
					swaBuffer[iSWAfill] = ch1;
					break;
				case 2:
					swaBuffer[iSWAfill] = ch2;
					break;
				case 3:
					swaBuffer[iSWAfill] = ch3;
					break;
				case 4:
					swaBuffer[iSWAfill] = ch4;
					break;
				}
				iSWAfill++;
				if (iSWAfill == 2 * SWA_LEN) {
					if (iIndication == 0) { // stim never ack'd
//						resetSWA(); // allow to try again without disconnecting
//						Util_restartClock(&clkESLODataTimeout, DATA_TIMEOUT_PERIOD * 2);
						triedDisconnecting = 1; // avoid overflowing event handler from EEG area
						SimplePeripheral_enqueueMsg(ES_FORCE_DISCONNECT, NULL);
					} else {
						isShippingSwa = 1;
						SimplePeripheral_enqueueMsg(ES_SHIP_SWA, NULL); // initial ship
					}
				}
			}
			return;
		}

		if (esloSettings[Set_SWA] == 0) {
//			GPIO_write(LED_1, 1);
			if (esloSettings[Set_EEG1]) {
				eslo_eeg1.type = Type_EEG1;
				eslo_eeg1.data = ch1;
				ESLO_Packet(eslo_eeg1, &packet);
				if (!isPaired) {
					ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_eeg1);
				} else {
					eeg1Buffer[iEEG] = packet;
				}
			}

			if (esloSettings[Set_EEG2]) {
				eslo_eeg2.type = Type_EEG2;
				eslo_eeg2.data = ch2;
				ESLO_Packet(eslo_eeg2, &packet);
				if (!isPaired) {
					ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_eeg2);
				} else {
					eeg2Buffer[iEEG] = packet;
				}
			}

			if (esloSettings[Set_EEG3]) {
				eslo_eeg3.type = Type_EEG3;
				eslo_eeg3.data = ch3;
				ESLO_Packet(eslo_eeg3, &packet);
				if (!isPaired) {
					ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_eeg3);
				} else {
					eeg3Buffer[iEEG] = packet;
				}
			}

			if (esloSettings[Set_EEG4]) {
				eslo_eeg4.type = Type_EEG4;
				eslo_eeg4.data = ch4;
				ESLO_Packet(eslo_eeg4, &packet);
				if (!isPaired) {
					ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_eeg4);
				} else {
					eeg4Buffer[iEEG] = packet;
				}
			}
//			GPIO_write(LED_1, 0);

			iEEG++;
			if (iEEG == PACKET_SZ_EEG) {
				if (isPaired) {
					if (esloSettings[Set_EEG1]) {
						SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
						SIMPLEPROFILE_CHAR4_LEN, eeg1Buffer);
					}
					if (esloSettings[Set_EEG2]) {
						SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
						SIMPLEPROFILE_CHAR4_LEN, eeg2Buffer);
					}
					if (esloSettings[Set_EEG3]) {
						SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
						SIMPLEPROFILE_CHAR4_LEN, eeg3Buffer);
					}
					if (esloSettings[Set_EEG4]) {
						SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
						SIMPLEPROFILE_CHAR4_LEN, eeg4Buffer);
					}
				}
				iEEG = 0;
			}
		}
	}
}

// !! handle ret values?
static void xlDataHandler(void) {
	eslo_dt eslo_xlx;
	eslo_dt eslo_xly;
	eslo_dt eslo_xlz;
	lsm6dsox_status_t xl_status;
	int16_t xl_data[3];

	if (xl_online) { // double check
		// XL
		lsm6dsox_status_get(&dev_ctx_xl, NULL, &xl_status);
		if (xl_status.drdy_xl) {
			lsm6dsox_acceleration_raw_get(&dev_ctx_xl, xl_data);

			eslo_xlx.type = Type_AxyXlx;
			eslo_xlx.data = (uint32_t) xl_data[0];
			ESLO_Packet(eslo_xlx, &packet);
			xlXBuffer[iXL] = packet;
			if (!isPaired
					& (esloSettings[Set_Record] | esloSettings[Set_SWA] > 0)) {
				ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_xlx);
			}

			eslo_xly.type = Type_AxyXly;
			eslo_xly.data = (uint32_t) xl_data[1];
			ESLO_Packet(eslo_xly, &packet);
			xlYBuffer[iXL] = packet;
			if (!isPaired
					& (esloSettings[Set_Record] | esloSettings[Set_SWA] > 0)) {
				ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_xly);
			}

			eslo_xlz.type = Type_AxyXlz;
			eslo_xlz.data = (uint32_t) xl_data[2];
			ESLO_Packet(eslo_xlz, &packet);
			xlZBuffer[iXL] = packet;
			if (!isPaired
					& (esloSettings[Set_Record] | esloSettings[Set_SWA] > 0)) {
				ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo_xlz);
			}

			// could make ring buffer, but then it performs check every 1s, not ideal
			if (moveCount == ES_BUFFER_SIZE) {
				isMoving = (isMoving << 1) & AXY_MOVE_MASK; // movement in last ES_BUFFER_SIZE samples
				float_t SD_X = ESLO_calculateSD(MoveXBuffer);
				float_t SD_Y = ESLO_calculateSD(MoveYBuffer);
				float_t SD_Z = ESLO_calculateSD(MoveZBuffer);
				if ((SD_X + SD_Y + SD_Z) > AXY_MOVE_THRESH) { // dynamic motion
					isMoving = isMoving | 1; // set LSB
				}
				moveCount = 0;
			} else {
				MoveXBuffer[moveCount] = lsm6dsox_from_fs2_to_mg(xl_data[0]);
				MoveYBuffer[moveCount] = lsm6dsox_from_fs2_to_mg(xl_data[1]);
				MoveZBuffer[moveCount] = lsm6dsox_from_fs2_to_mg(xl_data[2]);
				moveCount++;
			}

			iXL++;
		}

		if (iXL == PACKET_SZ_XL) {
			if (isPaired && central_isSpeaker == 0) {
				SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
				SIMPLEPROFILE_CHAR5_LEN, xlXBuffer);
				SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
				SIMPLEPROFILE_CHAR5_LEN, xlYBuffer);
				SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
				SIMPLEPROFILE_CHAR5_LEN, xlZBuffer);
			}
			iXL = 0;
		}
		axyCount++;
	}
}

void eegDataReady(uint_least8_t index) {
//	GPIO_write(LED_1, triedDisconnecting);
	if (isShippingSwa == 0 && triedDisconnecting == 0) {
		if (iEEGDiv < (EEG_SAMPLING_DIV - 1)) {
			iEEGDiv++;
		} else {
			SimplePeripheral_enqueueMsg(ES_EEG_NOTIF, NULL);
			iEEGDiv = 0;
		}
	}
}

void axyXlReady(uint_least8_t index) {
	if (xl_online) {
		SimplePeripheral_enqueueMsg(ES_XL_NOTIF, NULL);
	}
}

static void eegInterrupt(bool enableInterrupt) {
	if (enableInterrupt) {
		GPIO_enableInt(_EEG_DRDY);
	} else {
		GPIO_disableInt(_EEG_DRDY);
	}
}

static uint8_t updateEEGFromSettings(bool actOnInterrupt) {
	bool enableInterrupt;
	uint8_t shdnState = GPIO_read(_SHDN);

	if (USE_EEG(esloSettings) == ESLO_MODULE_ON) {
		if (shdnState == ESLO_LOW) {
			GPIO_write(_SHDN, ESLO_HIGH);
			GPIO_write(_EEG_PWDN, ESLO_HIGH);
			GPIO_setConfig(_EEG_CS,
			GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW | GPIO_CFG_OUT_LOW); // !!consider rm now that 1.8v is supplied
			eeg_online = ESLO_FAIL;
			// !! what to do if eeg never
			while (eeg_online == ESLO_FAIL) {
				Task_sleep(10000 / Clock_tickPeriod); // 1 ms (1000 us)
				eeg_online = ADS_init();
			}
			enableInterrupt = true;
			if (actOnInterrupt & eeg_online == ESLO_PASS) {
				eegInterrupt(enableInterrupt);
			}
		}
		// assumes this function is not called unless channel config has changed
		if (eeg_online == ESLO_PASS) {
			ADS_enableChannels(esloSettings[Set_EEG1], esloSettings[Set_EEG2],
					esloSettings[Set_EEG3], esloSettings[Set_EEG4]);
		}
	}
	if (USE_EEG(esloSettings) == ESLO_MODULE_OFF & shdnState == ESLO_HIGH) {
		enableInterrupt = false;
		eegInterrupt(enableInterrupt); // always turn off before shutting down
		GPIO_setConfig(_EEG_CS, GPIO_CFG_IN_NOPULL); // !!consider rm now that 1.8v is supplied
		ADS_close();
		GPIO_write(_SHDN, ESLO_LOW);
		GPIO_write(_EEG_PWDN, ESLO_LOW);
		eegCount = 0;
	}
	return enableInterrupt;
}

//static void xlInterrupt(bool enableInterrupt) {
//	if (enableInterrupt) {
////		GPIO_enableInt(AXY_DRDY);
//	} else {
////		GPIO_disableInt(AXY_DRDY);
//	}
//}

void updateXlFromSettings() {
	if (xl_online) {
		// reschedule handles stop/start, but only returns clock to previous state
		if (Util_isActive(&clkESLOAxy)) {
			Util_stopClock(&clkESLOAxy);
		}
		switch (esloSettings[Set_AxyMode]) {
		case 0:
//			lsm6dsox_xl_power_mode_set(&dev_ctx_xl,
//					LSM6DSOX_ULTRA_LOW_POWER_MD);
//			lsm6dsox_xl_data_rate_set(&dev_ctx_xl, LSM6DSOX_XL_ODR_1Hz6);
			lsm6dsox_xl_power_mode_set(&dev_ctx_xl,
					LSM6DSOX_LOW_NORMAL_POWER_MD);
			lsm6dsox_xl_data_rate_set(&dev_ctx_xl, LSM6DSOX_XL_ODR_12Hz5);
			Util_rescheduleClock(&clkESLOAxy, 0, ES_AXY_PERIOD);
			break;
		case 1:
			lsm6dsox_xl_power_mode_set(&dev_ctx_xl,
					LSM6DSOX_LOW_NORMAL_POWER_MD);
			lsm6dsox_xl_data_rate_set(&dev_ctx_xl, LSM6DSOX_XL_ODR_12Hz5);
			Util_rescheduleClock(&clkESLOAxy, 0, ES_AXY_PERIOD / 10);
			break;
		default:
			break;
		}
		Util_startClock(&clkESLOAxy);
	} else {
		// deprecated, axy always on
//		Util_stopClock(&clkESLOAxy);
//		enableInterrupt = false;
//		xlInterrupt(enableInterrupt); // always turn off before powering down
//		lsm6dsox_gy_power_mode_set(&dev_ctx_xl, LSM6DSOX_GY_NORMAL);
//		lsm6dsox_gy_data_rate_set(&dev_ctx_xl, LSM6DSOX_GY_ODR_OFF);
//		lsm6dsox_xl_power_mode_set(&dev_ctx_xl, LSM6DSOX_ULTRA_LOW_POWER_MD);
//		lsm6dsox_xl_data_rate_set(&dev_ctx_xl, LSM6DSOX_XL_ODR_OFF);
	}
}

static void ESLO_dumpMemUART() {
	uint32_t i;
	uint8_t k;
	uint32_t exportAddr = 0; // block
	UART_Params uartParams;
	UART_Params_init(&uartParams);
	uartParams.baudRate = 115200;
	uart = UART_open(CONFIG_UART_0, &uartParams); // UART_close(uart);

	for (k = 0; k < 10; k++) {
		UART_write(uart, &k, sizeof(uint8_t));
	}

	while (exportAddr < esloAddr) {
		FlashPageRead(exportAddr, readBuf);
		for (i = 0; i < PAGE_DATA_SIZE; i++) {
			UART_write(uart, &readBuf[i], sizeof(uint8_t));
			if (i == 0) {
				GPIO_toggle(LED_0);
				GPIO_toggle(LED_1);
			}
		}
		exportAddr += 0x1000;
	}
	UART_close(uart);
}

static void ESLO_error() {
	while (1) {
		GPIO_toggle(LED_0);
		GPIO_toggle(LED_1);
		Task_sleep(10000); // blink
	}
}

static void ESLO_startup() {
	GPIO_init();
	SPI_init();
	ADC_init();
	NVS_init();
	UART_init();

	GPIO_write(LED_0, 1);
	ESLO_SPI = ESLO_SPI_init(CONFIG_SPI);
	ESLO_SPI_EEG = ESLO_SPI_EEG_init(CONFIG_SPI_EEG);

	dev_ctx_xl.write_reg = write_reg;
	dev_ctx_xl.read_reg = read_reg;
	dev_ctx_xl.handle = (void*) ESLO_SPI;
	xl_online = AXY_Init(dev_ctx_xl); // do this here, but init eeg is as-needed

// init Settings
	SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN,
			esloSettings);

	/* NVS */
	esloRecoverSession();

	/* NAND */
	mem_online = NAND_Init();
	if (mem_online == ESLO_MODULE_OFF) {
		ESLO_error();
	}

// break here if debug mode
	if (GPIO_read(DEBUG) == ESLO_LOW) {
		ESLO_dumpMemUART();
	}

	/* ADS129X - Defaults in SysConfig */
	bool enableEEGInterrupt = updateEEGFromSettings(false); // do not turn on yet

	ADC_Params_init(&adcParams_vBatt);
	adc_vBatt = ADC_open(R_VBATT, &adcParams_vBatt);
	if (adc_vBatt == NULL) {
		ESLO_error();
	}
	ADC_Params_init(&adcParams_therm);
	adc_therm = ADC_open(THERM, &adcParams_therm);
	if (adc_therm == NULL) {
// !! non-critical, but maybe a way to not record therm?
	}

	Watchdog_init();
	Watchdog_Params_init(&watchdogParams);
	watchdogParams.resetMode = Watchdog_RESET_ON;
//	params.callbackFxn = (Watchdog_Callback) WatchdogCallbackFxn;
	watchdogParams.callbackFxn = NULL;
	watchdogHandle = Watchdog_open(CONFIG_WATCHDOG_0, &watchdogParams);
	if (watchdogHandle == NULL) {
		ESLO_error();
	}
	resetSWA();

	updateXlFromSettings();
	eegInterrupt(enableEEGInterrupt); // turn on now

	GPIO_write(LED_0, 0);
	GPIO_write(LED_1, 0);
}

// assumes graceful watchdog
//void WatchdogCallbackFxn(Watchdog_Handle handle) {
//	esloSleep();
//}

/*********************************************************************
 * @fn      simple_peripheral_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void simple_peripheral_spin(void) {
	volatile uint8_t x = 0;

	while (1) {
		x++;
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 */
void SimplePeripheral_createTask(void) {
	Task_Params taskParams;

// Configure task
	Task_Params_init(&taskParams);
	taskParams.stack = spTaskStack;
	taskParams.stackSize = SP_TASK_STACK_SIZE;
	taskParams.priority = SP_TASK_PRIORITY;

	Task_construct(&spTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void SimplePeripheral_init(void) {
	// swap address to include BLE MAC address (unique for each device)
	uint64_t bleAddress = *((uint64_t*) (FCFG1_BASE + FCFG1_O_MAC_BLE_0))
			& 0xFFFFFFFFFFFF;
	char newAddress[GAP_DEVICE_NAME_LEN] = ""; // +1 for null
	sprintf(newAddress, "ES_%llX", bleAddress); // use <4 chars as prepend
	memcpy(attDeviceName, newAddress, GAP_DEVICE_NAME_LEN);

// ******************************************************************
// N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
// ******************************************************************
// Register the current thread as an ICall dispatcher application
// so that the application can send and receive messages.
	ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
// Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
	HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
	RCOSC_enableCalibration();
#endif // USE_RCOSC

// Create an RTOS queue for message from profile to be sent to app.
	appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

	Util_constructClock(&clkNotifyVitals, SimplePeripheral_clockHandler, 0,
	ES_VITALS_EVT_PERIOD, false, (UArg) &argESLOVitals);

	Util_constructClock(&clkESLOPeriodic, SimplePeripheral_clockHandler, 0,
	ES_PERIODIC_EVT_PERIOD, true, (UArg) &argESLOPeriodic);

	Util_constructClock(&clkESLOAxy, SimplePeripheral_clockHandler, 0,
	ES_AXY_PERIOD, false, (UArg) &argESLOAxy);

// don't turn on because advertising is enabled on startup below, turn on at conn. terminate
	Util_constructClock(&clkESLOAdvSleep, SimplePeripheral_clockHandler,
	ES_ADV_SLEEP_TIMEOUT_MIN * 1000, 0, false, (UArg) &argESLOAdvSleep);

//	 turn on later with updated values from ESLOSettings
	Util_constructClock(&clkESLORecPeriod, SimplePeripheral_clockHandler, 0, 0,
	false, (UArg) &argESLORecPeriod);

	Util_constructClock(&clkESLORecDuration, SimplePeripheral_clockHandler, 0,
			0,
			false, (UArg) &argESLORecDuration);

	Util_constructClock(&clkESLODataTimeout, SimplePeripheral_clockHandler,
	DATA_TIMEOUT_PERIOD * 2, 0,
	false, (UArg) &argESLODataTimeout); // make longer than central so it can reset before

	Util_constructClock(&clkESLOSpeakerDelay, SimplePeripheral_clockHandler,
	SPEAKER_CONN_DELAY, 0,
	false, (UArg) &argESLOSpeakerDelay);

// Set the Device Name characteristic in the GAP GATT Service
// For more information, see the section in the User's Guide:
// http://software-dl.ti.com/lprf/ble5stack-latest/
	GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

// Configure GAP
	{
		uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

// Pass all parameter update requests to the app for it to decide
		GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
	}

// Setup the GAP Bond Manager. For more information see the GAP Bond Manager
// section in the User's Guide
	setBondManagerParameters();

// Initialize GATT attributes
	GGS_AddService(GATT_ALL_SERVICES);// GAP GATT Service
	GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
	DevInfo_AddService();                      // Device Information Service
	SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

// Setup the SimpleProfile Characteristic Values
// For more information, see the GATT and GATTServApp sections in the User's Guide:
// http://software-dl.ti.com/lprf/ble5stack-latest/
	{
		uint8_t charValue1[SIMPLEPROFILE_CHAR1_LEN] = { 0 };
		uint8_t charValue2[SIMPLEPROFILE_CHAR2_LEN] = { 0 };
//		uint8_t charValue3[SIMPLEPROFILE_CHAR3_LEN] = { 0 }; // set by ESLO init
		uint8_t charValue4[SIMPLEPROFILE_CHAR4_LEN] = { 0 };
		uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 0 };
		uint8_t charValue6[SIMPLEPROFILE_CHAR6_LEN] = { 0 };
		uint8_t charValue7[SIMPLEPROFILE_CHAR7_LEN] = { 0 };

		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1,
		SIMPLEPROFILE_CHAR1_LEN, charValue1);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2,
		SIMPLEPROFILE_CHAR2_LEN, charValue2);
//		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN,
//				charValue3);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
		SIMPLEPROFILE_CHAR4_LEN, charValue4);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5,
		SIMPLEPROFILE_CHAR5_LEN, charValue5);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6,
		SIMPLEPROFILE_CHAR6_LEN, charValue6);
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR7,
		SIMPLEPROFILE_CHAR7_LEN, charValue7);
	}

// Register callback with SimpleGATTprofile
	SimpleProfile_RegisterAppCBs(&SimplePeripheral_simpleProfileCBs);

// Start Bond Manager and register callback
	VOID GAPBondMgr_Register(&SimplePeripheral_BondMgrCBs);

// Register with GAP for HCI/Host messages. This is needed to receive HCI
// events. For more information, see the HCI section in the User's Guide:
// http://software-dl.ti.com/lprf/ble5stack-latest/
	GAP_RegisterForMsgs(selfEntity);

// Register for GATT local events and ATT Responses pending for transmission
	GATT_RegisterForMsgs(selfEntity);

// Set default values for Data Length Extension
// Extended Data Length Feature is already enabled by default
	{
// Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
// Some brand smartphone is essentially needing 251/2120, so we set them here.
#define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
#define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

// This API is documented in hci.h
// See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
// http://software-dl.ti.com/lprf/ble5stack-latest/
		HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE,
				APP_SUGGESTED_TX_TIME);
	}

// Initialize GATT Client
	GATT_InitClient("");

// Initialize Connection List
	SimplePeripheral_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

//Initialize GAP layer for Peripheral role and register to receive GAP events
	GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode,
			&pRandomAddress);

// Initialize array to store connection handle and RSSI values
	SimplePeripheral_initPHYRSSIArray();

	ESLO_startup();
}

/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 */
static void SimplePeripheral_taskFxn(UArg a0, UArg a1) {
// Initialize application
	SimplePeripheral_init();

// Application main loop
	for (;;) {
		uint32_t events;

// Waits for an event to be posted associated with the calling thread.
// Note that an event associated with a thread is posted when a
// message is queued to the message receive queue of the thread
		events = Event_pend(syncEvent, Event_Id_NONE, SP_ALL_EVENTS,
		ICALL_TIMEOUT_FOREVER);

		if (events) {
			ICall_EntityID dest;
			ICall_ServiceEnum src;
			ICall_HciExtEvt *pMsg = NULL;

			// Fetch any available messages that might have been sent from the stack
			if (ICall_fetchServiceMsg(&src, &dest,
					(void**) &pMsg) == ICALL_ERRNO_SUCCESS) {
				uint8 safeToDealloc = TRUE;

				if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) {
					ICall_Stack_Event *pEvt = (ICall_Stack_Event*) pMsg;

					// Check for BLE stack events first
					if (pEvt->signature != 0xffff) {
						// Process inter-task message
						safeToDealloc = SimplePeripheral_processStackMsg(
								(ICall_Hdr*) pMsg);
					}
				}

				if (pMsg && safeToDealloc) {
					ICall_freeMsg(pMsg);
				}
			}

			// If RTOS queue is not empty, process app message.
			if (events & SP_QUEUE_EVT) {
				while (!Queue_empty(appMsgQueueHandle)) {
					spEvt_t *pMsg = (spEvt_t*) Util_dequeueMsg(
							appMsgQueueHandle);
					if (pMsg) {
						// Process message.
						SimplePeripheral_processAppMsg(pMsg);

						// Free the space from the message.
						ICall_free(pMsg);
					}
				}
			}
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg) {
// Always dealloc pMsg unless set otherwise
	uint8_t safeToDealloc = TRUE;

	switch (pMsg->event) {
	case GAP_MSG_EVENT:
		SimplePeripheral_processGapMessage((gapEventHdr_t*) pMsg);
		break;

	case GATT_MSG_EVENT:
// Process GATT message
		safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t*) pMsg);
		break;

	case HCI_GAP_EVENT_EVENT: {
// Process HCI message
		switch (pMsg->status) {
		case HCI_COMMAND_COMPLETE_EVENT_CODE:
			// Process HCI Command Complete Events here
		{
			SimplePeripheral_processCmdCompleteEvt(
					(hciEvt_CmdComplete_t*) pMsg);
			break;
		}

		case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
			AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
			break;

			// HCI Commands Events
		case HCI_COMMAND_STATUS_EVENT_CODE: {
			hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t*) pMsg;
			switch (pMyMsg->cmdOpcode) {
			case HCI_LE_SET_PHY: {
				if (pMyMsg->cmdStatus
						== HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE) {
//                Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
//                        "PHY Change failure, peer does not support this");
				} else {
//                Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
//                               "PHY Update Status Event: 0x%x",
//                               pMyMsg->cmdStatus);
				}

				SimplePeripheral_updatePHYStat(HCI_LE_SET_PHY, (uint8_t*) pMsg);
				break;
			}

			default:
				break;
			}
			break;
		}

			// LE Events
		case HCI_LE_EVENT_CODE: {
			hciEvt_BLEPhyUpdateComplete_t *pPUC =
					(hciEvt_BLEPhyUpdateComplete_t*) pMsg;

			// A Phy Update Has Completed or Failed
			if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT) {
				if (pPUC->status != SUCCESS) {
//              Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
//                             "PHY Change failure");
				} else {
					// Only symmetrical PHY is supported.
					// rxPhy should be equal to txPhy.
				}

				SimplePeripheral_updatePHYStat(
				HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t*) pMsg);
			}
			break;
		}

		default:
			break;
		}

		break;
	}

	default:
// do nothing
		break;
	}

	return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg) {
	if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT) {
// ATT request-response or indication-confirmation flow control is
// violated. All subsequent ATT requests or indications will be dropped.
// The app is informed in case it wants to drop the connection.

// Display the opcode of the message that caused the violation.
//    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
	} else if (pMsg->method == ATT_MTU_UPDATED_EVENT) {
// MTU size updated
//    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
	} else if (pMsg->method == ATT_HANDLE_VALUE_CFM) {
		// only do logic and increment iIndication if SWA detection has kicked off
		if (SWAsent == 1) {
			iIndication++;
			if (iIndication > 1) { // ship next after stim indication
				SimplePeripheral_enqueueMsg(ES_SHIP_SWA, NULL);
			}
		}
	}

// Free message payload. Needed only for ATT Protocol messages
	GATT_bm_free(&pMsg->msg, pMsg->method);

// It's safe to free the incoming message
	return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg) {
	bool dealloc = TRUE;
	eslo_dt eslo;
	eslo.type = Type_EEGState;

	switch (pMsg->event) {
	case SP_CHAR_CHANGE_EVT:
		SimplePeripheral_processCharValueChangeEvt(*(uint8_t*) (pMsg->pData));
		break;
	case SP_ADV_EVT:
		SimplePeripheral_processAdvEvent((spGapAdvEventData_t*) (pMsg->pData));
		break;
	case SP_PAIR_STATE_EVT:
		SimplePeripheral_processPairState((spPairStateData_t*) (pMsg->pData));
		break;
	case SP_PASSCODE_EVT:
		SimplePeripheral_processPasscode((spPasscodeData_t*) (pMsg->pData));
		break;
	case SP_READ_RPA_EVT:
		SimplePeripheral_updateRPA();
		break;
	case SP_SEND_PARAM_UPDATE_EVT: {
// Extract connection handle from data
		uint16_t connHandle =
				*(uint16_t*) (((spClockEventData_t*) pMsg->pData)->data);
//		paramsSynced = 1;
		SimplePeripheral_processParamUpdate(connHandle);
// This data is not dynamically allocated
		dealloc = FALSE;
		break;
	}
	case SP_CONN_EVT:
		SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t*) (pMsg->pData));
		break;
	case ES_PERIODIC_EVT:
		ESLO_performPeriodicTask();
		break;
	case ES_VITALS_EVT:
		SimplePeripheral_notifyVitals();
		break;
	case ES_AXY_EVT:
		xlDataHandler(); // already in queue, go get data
		break;
	case ES_EEG_NOTIF:
		eegDataHandler();
		break;
	case ES_XL_NOTIF:
		xlDataHandler();
		break;
	case ES_ADV_SLEEP:
		advSleep();
		break;
	case ES_REC_PERIOD: {
		uint32_t recDurationInMillis = 1000 * 60
				* (uint32_t) esloSettings[Set_RecDuration];
		if (recDurationInMillis > 0) {
			// reinstate EEG settings
			esloSettings[Set_EEG1] = esloSettingsSleep[Set_EEG1];
			esloSettings[Set_EEG2] = esloSettingsSleep[Set_EEG2];
			esloSettings[Set_EEG3] = esloSettingsSleep[Set_EEG3];
			esloSettings[Set_EEG4] = esloSettingsSleep[Set_EEG4];
			updateEEGFromSettings(true);
			eslo.data = 1;
			ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);
			// if values are equal or rec dur is somehow greater than period, skip turning off
			if (esloSettings[Set_RecDuration] < esloSettings[Set_RecPeriod]) {
				Util_restartClock(&clkESLORecDuration, recDurationInMillis);
			}
		}
		break;
	}

	case ES_REC_DURATION: {
		esloSettings[Set_EEG1] = 0;
		esloSettings[Set_EEG2] = 0;
		esloSettings[Set_EEG3] = 0;
		esloSettings[Set_EEG4] = 0;
		updateEEGFromSettings(false);
		eslo.data = 0;
		ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);
		break;
	}

	case ES_SHIP_SWA: {
		shipSWA();
		break;
	}

	case ES_FORCE_DISCONNECT: {
		GAP_TerminateLinkReq(LINKDB_CONNHANDLE_ALL,
				HCI_DISCONNECT_REMOTE_USER_TERM);
		break;
	}

	default:
// Do nothing.
		break;
	}

// Free message data if it exists and we are to dealloc
	if ((dealloc == TRUE) && (pMsg->pData != NULL)) {
		ICall_free(pMsg->pData);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg) {
	switch (pMsg->opcode) {
	case GAP_DEVICE_INIT_DONE_EVENT: {
		bStatus_t status = FAILURE;

		gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t*) pMsg;

		if (pPkt->hdr.status == SUCCESS) {
			// Store the system ID
			uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

			// use 6 bytes of device address for 8 bytes of system ID value
			systemId[0] = pPkt->devAddr[0];
			systemId[1] = pPkt->devAddr[1];
			systemId[2] = pPkt->devAddr[2];

			// set middle bytes to zero
			systemId[4] = 0;
			systemId[3] = 0;

			// shift three bytes up
			systemId[7] = pPkt->devAddr[5];
			systemId[6] = pPkt->devAddr[4];
			systemId[5] = pPkt->devAddr[3];

			// Set Device Info Service Parameter
			DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
					systemId);

			// Setup and start Advertising
			// For more information, see the GAP section in the User's Guide:
			// http://software-dl.ti.com/lprf/ble5stack-latest/

			// Create Advertisement set #1 and assign handle
			status = GapAdv_create(&SimplePeripheral_advCallback, &advParams1,
					&advHandleLegacy);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Load advertising data for set #1 that is statically allocated by the app
			status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
					sizeof(advData1), advData1);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Load scan response data for set #1 that is statically allocated by the app
			status = GapAdv_loadByHandle(advHandleLegacy,
					GAP_ADV_DATA_TYPE_SCAN_RSP, sizeof(scanResData1),
					scanResData1);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Set event mask for set #1
			status = GapAdv_setEventMask(advHandleLegacy,
					GAP_ADV_EVT_MASK_START_AFTER_ENABLE
							| GAP_ADV_EVT_MASK_END_AFTER_DISABLE
							| GAP_ADV_EVT_MASK_SET_TERMINATED);

			// Enable legacy advertising for set #1
			status = GapAdv_enable(advHandleLegacy,
					GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Create Advertisement set #2 and assign handle
			status = GapAdv_create(&SimplePeripheral_advCallback, &advParams2,
					&advHandleLongRange);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Load advertising data for set #2 that is statically allocated by the app
			status = GapAdv_loadByHandle(advHandleLongRange,
					GAP_ADV_DATA_TYPE_ADV, sizeof(advData2), advData2);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Set event mask for set #2
			status = GapAdv_setEventMask(advHandleLongRange,
					GAP_ADV_EVT_MASK_START_AFTER_ENABLE
							| GAP_ADV_EVT_MASK_END_AFTER_DISABLE
							| GAP_ADV_EVT_MASK_SET_TERMINATED);

			// Enable long range advertising for set #2
			status = GapAdv_enable(advHandleLongRange,
					GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

			// Display device address
//        Display_printf(dispHandle, SP_ROW_IDA, 0, "%s Addr: %s",
//                       (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
//                       Util_convertBdAddr2Str(pPkt->devAddr));

			if (addrMode > ADDRMODE_RANDOM) {
				SimplePeripheral_updateRPA();

				// Create one-shot clock for RPA check event.
				Util_constructClock(&clkRpaRead, SimplePeripheral_clockHandler,
				READ_RPA_PERIOD, 0, true, (UArg) &argRpaRead);
			}
		}

		break;
	}

	case GAP_LINK_ESTABLISHED_EVENT: {
		gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t*) pMsg;

// Display the amount of current connections
		uint8_t numActive = linkDB_NumActive("");
//      Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Num Conns: %d",
//                     (uint16_t)numActive);

		if (pPkt->hdr.status == SUCCESS) {
			// see pPkt->devAddr
			if (memcmp(pPkt->devAddr, ESLO_SPEAKER_ADDR, 6) == 0) {
				central_isSpeaker = 1;
			} else {
				central_isSpeaker = 0;
			}
			Util_stopClock(&clkESLOAdvSleep);
			Util_stopClock(&clkESLORecPeriod);
			Util_stopClock(&clkESLORecDuration);

			// always tell memory rec has stopped
			eslo_dt eslo;
			eslo.type = Type_EEGState;
			eslo.data = 0;
			ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);

			// Add connection to list and start RSSI
			SimplePeripheral_addConn(pPkt->connectionHandle);

			// Display the address of this connection
//        Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connected to %s",
//                       Util_convertBdAddr2Str(pPkt->devAddr));
			isPaired = true;

			// recover old settings
			mapEsloSettings(esloSettingsSleep);

			// Start Periodic Vitals
			if (central_isSpeaker == 0) {
				Util_startClock(&clkNotifyVitals);
			} else {
				if (esloSettings[Set_SWA] == 0) {
					Util_startClock(&clkESLOSpeakerDelay);
				} else {
					Util_rescheduleClock(&clkESLODataTimeout,
					DATA_TIMEOUT_PERIOD * 2, 0);
					Util_startClock(&clkESLODataTimeout); // protect from persistent base connection
				}
			}

		}
		if ((numActive < MAX_NUM_BLE_CONNS)
				&& (autoConnect == AUTOCONNECT_DISABLE)) {
			// Start advertising since there is room for more connections
			// Not used for ESLO (max=1)
			GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX,
					0);
		} else {
			// Stop all advertising since there is no room for more connections
			GapAdv_disable(advHandleLongRange);
			GapAdv_disable(advHandleLegacy);
		}
		break;
	}

	case GAP_LINK_TERMINATED_EVENT: {
		gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t*) pMsg;
		uint8_t numActive = linkDB_NumActive("");

// Remove the connection from the list and disable RSSI if needed
		SimplePeripheral_removeConn(pPkt->connectionHandle);

// If no active connections
		if (numActive == 0) {
			// Stop periodic clock
			Util_stopClock(&clkNotifyVitals);
			Util_stopClock(&clkESLODataTimeout);
			isPaired = false;

			// always save, if device is not sleeping they will have no effect when reloaded
			memcpy(esloSettingsSleep, esloSettings,
			SIMPLEPROFILE_CHAR3_LEN);
			if (esloSettings[Set_Record] == ESLO_MODULE_OFF) {
				esloSleep();
			} else {
				uint32_t recPeriodMillis = 1000 * 60
						* (uint32_t) esloSettings[Set_RecPeriod];
				// if equal, the timer is pointless
				if (recPeriodMillis > 0) {
					// schedule recording period/cycle
					Util_rescheduleClock(&clkESLORecPeriod, 100,
							recPeriodMillis); // give timeout to disconnect
					Util_startClock(&clkESLORecPeriod);
				}
				if (esloSettings[Set_RecPeriod] == 0
						|| esloSettings[Set_RecDuration] == 0) {
					SimplePeripheral_enqueueMsg(ES_REC_DURATION, NULL); // turn off
				}
			}

			isAdvLong = 0; // reset every disconnect
			if (esloSettings[Set_AdvLong] > 0) { // long adv
				Util_startClock(&clkESLOAdvSleep);
			} else { // keep advertising on at full rate
				GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX,
						0);
				GapAdv_enable(advHandleLongRange,
						GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
			}
			central_isSpeaker = 0;
			triedDisconnecting = 0;
			resetSWA();
		}
		break;
	}

	case GAP_UPDATE_LINK_PARAM_REQ_EVENT: {
		gapUpdateLinkParamReqReply_t rsp;

		gapUpdateLinkParamReqEvent_t *pReq =
				(gapUpdateLinkParamReqEvent_t*) pMsg;

		rsp.connectionHandle = pReq->req.connectionHandle;
		rsp.signalIdentifier = pReq->req.signalIdentifier;

// Only accept connection intervals with slave latency of 0
// This is just an example of how the application can send a response
		if (pReq->req.connLatency == 0) {
			rsp.intervalMin = pReq->req.intervalMin;
			rsp.intervalMax = pReq->req.intervalMax;
			rsp.connLatency = pReq->req.connLatency;
			rsp.connTimeout = pReq->req.connTimeout;
			rsp.accepted = TRUE;
		} else {
			rsp.accepted = FALSE;
		}

// Send Reply
		VOID GAP_UpdateLinkParamReqReply(&rsp);

		break;
	}

	case GAP_LINK_PARAM_UPDATE_EVENT: {
		gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t*) pMsg;

// Get the address from the connection handle
		linkDBInfo_t linkInfo;
		linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

		if (pPkt->status == SUCCESS) {
			// Display the address of the connection update
//        Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Link Param Updated: %s",
//                       Util_convertBdAddr2Str(linkInfo.addr));
		} else {
			// Display the address of the connection update failure
//        Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
//                       "Link Param Update Failed 0x%x: %s", pPkt->opcode,
//                       Util_convertBdAddr2Str(linkInfo.addr));
		}

// Check if there are any queued parameter updates
		spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t*) List_get(
				&paramUpdateList);
		if (connHandleEntry != NULL) {
			// Attempt to send queued update now
			SimplePeripheral_processParamUpdate(connHandleEntry->connHandle);

			// Free list element
			ICall_free(connHandleEntry);
		}

		break;
	}

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
    case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
    {
      linkDBInfo_t linkInfo;
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      // Display the address of the connection update failure
//      Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
//                     "Peer Device's Update Request Rejected 0x%x: %s", pPkt->opcode,
//                     Util_convertBdAddr2Str(linkInfo.addr));

      break;
    }
#endif

	default:
//      Display_clearLines(dispHandle, SP_ROW_STATUS_1, SP_ROW_STATUS_2);
		break;
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void SimplePeripheral_charValueChangeCB(uint8_t paramId) {
	uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

	if (pValue) {
		*pValue = paramId;

		if (SimplePeripheral_enqueueMsg(SP_CHAR_CHANGE_EVT, pValue) != SUCCESS) {
			ICall_free(pValue);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId) {
	uint8_t len;
	bStatus_t retProfile;

	switch (paramId) {
	case SIMPLEPROFILE_CHAR1:
		len = SIMPLEPROFILE_CHAR1_LEN;
		break;
	case SIMPLEPROFILE_CHAR3:
		len = SIMPLEPROFILE_CHAR3_LEN;
		break;
	case SIMPLEPROFILE_CHAR6:
		len = SIMPLEPROFILE_CHAR6_LEN;
		break;
	default:
		break;
	}

	uint8_t *pValue = ICall_malloc(len); // dynamic allocation

	switch (paramId) {
// only characteristics with GATT_PROP_WRITE, all others are written elsewhere
	case SIMPLEPROFILE_CHAR1:
		retProfile = SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, pValue);
		GPIO_write(LED_0, pValue[0]);
		break;
	case SIMPLEPROFILE_CHAR3:
		retProfile = SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, pValue);
		mapEsloSettings(pValue);
		break;
	case SIMPLEPROFILE_CHAR6:
		retProfile = SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR6, pValue);
		memcpy(&esloExportBlock, pValue, sizeof(uint32_t));
		break;
	default:
// should not reach here!
		break;
	}
	if (retProfile) {
		ICall_free(pValue);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_notifyVitals
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (ES_VITALS_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_notifyVitals(void) {
	readBatt();
	readTherm();

	ESLO_compileVitals(&vbatt_uV, &lowVoltage, &temp_uC, &esloAddr, &isMoving,
			vitalsBuffer);
	bStatus_t retProfile = SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2,
	SIMPLEPROFILE_CHAR2_LEN, vitalsBuffer);
}

static void ESLO_performPeriodicTask() {
	eslo_dt eslo;
	ReturnType retEslo;

	Watchdog_clear(watchdogHandle);
	absoluteTime += (ES_PERIODIC_EVT_PERIOD / 1000);
	iLog++;

	readBatt();
	if (vbatt_uV < V_DROPOUT) {
		esloSleep(); // good night
		return;
//		Util_stopClock(&clkESLOPeriodic); // not sure about this, watchdog needs to be handled
	}
	if (iLog >= ES_LOG_PERIODIC) {
		eslo.type = Type_BatteryVoltage;
		eslo.data = vbatt_uV / 1000; // use mV
		ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);

		readTherm();
		eslo.type = Type_Therm;
		eslo.data = temp_uC / 1000; // use mC
		retEslo = ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);

		eslo.type = Type_AbsoluteTime;
		eslo.data = absoluteTime;
		ESLO_Write(&esloAddr, esloBuffer, esloVersion, eslo);

		if (retEslo == Flash_MemoryOverflow) {
			esloSleep(); // good night
		}

		esloUpdateNVS(); // save esloAddress to recover session
		iLog = 0;
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_updateRPA(void) {
	uint8_t *pRpaNew;

// Read the current RPA.
	pRpaNew = GAP_GetDevAddress(FALSE);

	if (memcmp(pRpaNew, rpa, B_ADDR_LEN)) {
		memcpy(rpa, pRpaNew, B_ADDR_LEN);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimplePeripheral_clockHandler(UArg arg) {
	spClockEventData_t *pData = (spClockEventData_t*) arg;

	if (pData->event == ES_PERIODIC_EVT) {
		SimplePeripheral_enqueueMsg(ES_PERIODIC_EVT, NULL);
	} else if (pData->event == ES_VITALS_EVT) {
		SimplePeripheral_enqueueMsg(ES_VITALS_EVT, NULL);
	} else if (pData->event == ES_AXY_EVT) {
		SimplePeripheral_enqueueMsg(ES_AXY_EVT, NULL);
	} else if (pData->event == SP_READ_RPA_EVT) {
		Util_startClock(&clkRpaRead);
		SimplePeripheral_enqueueMsg(SP_READ_RPA_EVT, NULL);
	} else if (pData->event == ES_ADV_SLEEP) {
		SimplePeripheral_enqueueMsg(ES_ADV_SLEEP, NULL);
	} else if (pData->event == ES_REC_PERIOD) {
		SimplePeripheral_enqueueMsg(ES_REC_PERIOD, NULL);
	} else if (pData->event == ES_REC_DURATION) {
		SimplePeripheral_enqueueMsg(ES_REC_DURATION, NULL);
	} else if (pData->event == SP_SEND_PARAM_UPDATE_EVT) {
		SimplePeripheral_enqueueMsg(SP_SEND_PARAM_UPDATE_EVT, pData);
	} else if (pData->event == ES_DATA_TIMEOUT) {
		resetSWA();
	} else if (pData->event == ES_SPEAKER_DELAY) {
		// disconnect after delay if SWA is off and is speaker
		if (triedDisconnecting == 0) {
			triedDisconnecting = 1; // avoid overflowing event handler from EEG area
			SimplePeripheral_enqueueMsg(ES_FORCE_DISCONNECT, NULL);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf,
		uintptr_t arg) {
	spGapAdvEventData_t *pData = ICall_malloc(sizeof(spGapAdvEventData_t));

	if (pData) {
		pData->event = event;
		pData->pBuf = pBuf;

		if (SimplePeripheral_enqueueMsg(SP_ADV_EVT, pData) != SUCCESS) {
			ICall_free(pData);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData) {
	switch (pEventData->event) {
	case GAP_EVT_ADV_START_AFTER_ENABLE:
//      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d Enabled",
//                     *(uint8_t *)(pEventData->pBuf));
		break;

	case GAP_EVT_ADV_END_AFTER_DISABLE:
//      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d Disabled",
//                     *(uint8_t *)(pEventData->pBuf));
		break;

	case GAP_EVT_ADV_START:
		break;

	case GAP_EVT_ADV_END:
		break;

	case GAP_EVT_ADV_SET_TERMINATED: {
#ifndef Display_DISABLE_ALL
//      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);
//      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d disabled after conn %d",
//                     advSetTerm->handle, advSetTerm->connHandle );
#endif
	}
		break;

	case GAP_EVT_SCAN_REQ_RECEIVED:
		break;

	case GAP_EVT_INSUFFICIENT_MEMORY:
		break;

	default:
		break;
	}

// All events have associated memory to free except the insufficient memory
// event
	if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY) {
		ICall_free(pEventData->pBuf);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
		uint8_t status) {
	spPairStateData_t *pData = ICall_malloc(sizeof(spPairStateData_t));

// Allocate space for the event data.
	if (pData) {
		pData->state = state;
		pData->connHandle = connHandle;
		pData->status = status;

// Queue the event.
		if (SimplePeripheral_enqueueMsg(SP_PAIR_STATE_EVT, pData) != SUCCESS) {
			ICall_free(pData);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr,
		uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs,
		uint32_t numComparison) {
	spPasscodeData_t *pData = ICall_malloc(sizeof(spPasscodeData_t));

// Allocate space for the passcode event.
	if (pData) {
		pData->connHandle = connHandle;
		memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
		pData->uiInputs = uiInputs;
		pData->uiOutputs = uiOutputs;
		pData->numComparison = numComparison;

// Enqueue the event.
		if (SimplePeripheral_enqueueMsg(SP_PASSCODE_EVT, pData) != SUCCESS) {
			ICall_free(pData);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimplePeripheral_processPairState(spPairStateData_t *pPairData) {
	uint8_t state = pPairData->state;
	uint8_t status = pPairData->status;

	switch (state) {
	case GAPBOND_PAIRING_STATE_STARTED:
//      Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing started");
		break;

	case GAPBOND_PAIRING_STATE_COMPLETE:
		if (status == SUCCESS) {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing success");
		} else {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing fail: %d", status);
		}
		break;

	case GAPBOND_PAIRING_STATE_ENCRYPTED:
		if (status == SUCCESS) {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Encryption success");
		} else {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Encryption failed: %d", status);
		}
		break;

	case GAPBOND_PAIRING_STATE_BOND_SAVED:
		if (status == SUCCESS) {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Bond save success");
		} else {
//        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Bond save failed: %d", status);
		}
		break;

	default:
		break;
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData) {
// Display passcode to user
	if (pPasscodeData->uiOutputs != 0) {
//    Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Passcode: %d",
//                   B_APP_DEFAULT_PASSCODE);
	}

// Send passcode response
	GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle, SUCCESS,
			B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport) {
// Get index from handle
	uint8_t connIndex = SimplePeripheral_getConnIndex(pReport->handle);

	if (connIndex >= MAX_NUM_BLE_CONNS) {
//    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
		return;
	}

// If auto phy change is enabled
	if (connList[connIndex].isAutoPHYEnable == TRUE) {
// Read the RSSI
		HCI_ReadRssiCmd(pReport->handle);
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData) {
	uint8_t success;
	spEvt_t *pMsg = ICall_malloc(sizeof(spEvt_t));

// Create dynamic pointer to message.
	if (pMsg) {
		pMsg->event = event;
		pMsg->pData = pData;

// Enqueue the message.
		success = Util_enqueueMsg(appMsgQueueHandle, syncEvent,
				(uint8_t*) pMsg);
		return (success) ? SUCCESS : FAILURE;
	}

	return (bleMemAllocError);
}

/*********************************************************************
 * @fn      SimplePeripheral_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_addConn(uint16_t connHandle) {
	uint8_t i;
	uint8_t status = bleNoResources;

// Try to find an available entry
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++) {
		if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID) {
			// Found available entry to put a new connection info in
			connList[i].connHandle = connHandle;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
			// Allocate data to send through clock handler
			connList[i].pParamUpdateEventData = ICall_malloc(
					sizeof(spClockEventData_t) + sizeof(uint16_t));
			if (connList[i].pParamUpdateEventData) {
				connList[i].pParamUpdateEventData->event =
				SP_SEND_PARAM_UPDATE_EVT;
				*((uint16_t*) connList[i].pParamUpdateEventData->data) =
						connHandle;

				// Create a clock object and start
				connList[i].pUpdateClock = (Clock_Struct*) ICall_malloc(
						sizeof(Clock_Struct));

				if (connList[i].pUpdateClock) {
					Util_constructClock(connList[i].pUpdateClock,
							SimplePeripheral_clockHandler,
							SEND_PARAM_UPDATE_DELAY, 0, true,
							(UArg) (connList[i].pParamUpdateEventData));
				} else {
					ICall_free(connList[i].pParamUpdateEventData);
				}
			} else {
				status = bleMemAllocError;
			}
#endif

			// Set default PHY to 1M
			connList[i].currPhy = HCI_PHY_1_MBPS;

			break;
		}
	}

	return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle) {
	uint8_t i;

	for (i = 0; i < MAX_NUM_BLE_CONNS; i++) {
		if (connList[i].connHandle == connHandle) {
			return i;
		}
	}

	return (MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle) {
	uint8_t i;
// Set to invalid connection index initially
	uint8_t connIndex = MAX_NUM_BLE_CONNS;

	if (connHandle != LINKDB_CONNHANDLE_ALL) {
// Get connection index from handle
		connIndex = SimplePeripheral_getConnIndex(connHandle);
		if (connIndex >= MAX_NUM_BLE_CONNS) {
			return (bleInvalidRange);
		}
	}

// Clear specific handle or all handles
	for (i = 0; i < MAX_NUM_BLE_CONNS; i++) {
		if ((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL)) {
			connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
			connList[i].currPhy = 0;
			connList[i].phyCngRq = 0;
			connList[i].phyRqFailCnt = 0;
			connList[i].rqPhy = 0;
			memset(connList[i].rssiArr, 0, SP_MAX_RSSI_STORE_DEPTH);
			connList[i].rssiAvg = 0;
			connList[i].rssiCntr = 0;
			connList[i].isAutoPHYEnable = FALSE;
		}
	}

	return (SUCCESS);
}

/*********************************************************************
 * @fn      SimplePeripheral_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void SimplePeripheral_clearPendingParamUpdate(uint16_t connHandle) {
	List_Elem *curr;

	for (curr = List_head(&paramUpdateList); curr != NULL;
			curr = List_next(curr)) {
		if (((spConnHandleEntry_t*) curr)->connHandle == connHandle) {
			List_remove(&paramUpdateList, curr);
		}
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle) {
	uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);

	if (connIndex != MAX_NUM_BLE_CONNS) {
		Clock_Struct *pUpdateClock = connList[connIndex].pUpdateClock;

		if (pUpdateClock != NULL) {
			// Stop and destruct the RTOS clock if it's still alive
			if (Util_isActive(pUpdateClock)) {
				Util_stopClock(pUpdateClock);
			}

			// Destruct the clock object
			Clock_destruct(pUpdateClock);
			// Free clock struct
			ICall_free(pUpdateClock);
			// Free ParamUpdateEventData
			ICall_free(connList[connIndex].pParamUpdateEventData);
		}
// Clear pending update requests from paramUpdateList
		SimplePeripheral_clearPendingParamUpdate(connHandle);
// Clear Connection List Entry
		SimplePeripheral_clearConnListEntry(connHandle);
	}

	return connIndex;
}

/*********************************************************************
 * @fn      SimplePeripheral_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void SimplePeripheral_processParamUpdate(uint16_t connHandle) {
	gapUpdateLinkParamReq_t req;
	uint8_t connIndex;

	req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
	req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
	req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
	req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
	req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif

	connIndex = SimplePeripheral_getConnIndex(connHandle);
	if (connIndex >= MAX_NUM_BLE_CONNS) {
//    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
		return;
	}

// Deconstruct the clock object
	Clock_destruct(connList[connIndex].pUpdateClock);
// Free clock struct, only in case it is not NULL
	if (connList[connIndex].pUpdateClock != NULL) {
		ICall_free(connList[connIndex].pUpdateClock);
		connList[connIndex].pUpdateClock = NULL;
	}
// Free ParamUpdateEventData, only in case it is not NULL
	if (connList[connIndex].pParamUpdateEventData != NULL)
		ICall_free(connList[connIndex].pParamUpdateEventData);

// Send parameter update
	bStatus_t status = GAP_UpdateLinkParamReq(&req);

// If there is an ongoing update, queue this for when the udpate completes
	if (status == bleAlreadyInRequestedMode) {
		spConnHandleEntry_t *connHandleEntry = ICall_malloc(
				sizeof(spConnHandleEntry_t));
		if (connHandleEntry) {
			connHandleEntry->connHandle = connHandle;

			List_put(&paramUpdateList, (List_Elem*) connHandleEntry);
		}
	}
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg) {
	uint8_t status = pMsg->pReturnParam[0];

//Find which command this command complete is for
	switch (pMsg->cmdOpcode) {
	case HCI_READ_RSSI: {
		int8 rssi = (int8) pMsg->pReturnParam[3];

// Display RSSI value, if RSSI is higher than threshold, change to faster PHY
		if (status == SUCCESS) {
			uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1],
					pMsg->pReturnParam[2]);

			uint8_t index = SimplePeripheral_getConnIndex(handle);
			if (index >= MAX_NUM_BLE_CONNS) {
//          Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
				return;
			}

			if (rssi != LL_RSSI_NOT_AVAILABLE) {
				connList[index].rssiArr[connList[index].rssiCntr++] = rssi;
				connList[index].rssiCntr %= SP_MAX_RSSI_STORE_DEPTH;

				int16_t sum_rssi = 0;
				for (uint8_t cnt = 0; cnt < SP_MAX_RSSI_STORE_DEPTH; cnt++) {
					sum_rssi += connList[index].rssiArr[cnt];
				}
				connList[index].rssiAvg = (uint32_t)(
						sum_rssi / SP_MAX_RSSI_STORE_DEPTH);

				uint8_t phyRq = SP_PHY_NONE;
				uint8_t phyRqS = SP_PHY_NONE;
				uint8_t phyOpt = LL_PHY_OPT_NONE;

				if (connList[index].phyCngRq == FALSE) {
					if ((connList[index].rssiAvg >= RSSI_2M_THRSHLD)
							&& (connList[index].currPhy != HCI_PHY_2_MBPS)
							&& (connList[index].currPhy != SP_PHY_NONE)) {
						// try to go to higher data rate
						phyRqS = phyRq = HCI_PHY_2_MBPS;
					} else if ((connList[index].rssiAvg < RSSI_2M_THRSHLD)
							&& (connList[index].rssiAvg >= RSSI_1M_THRSHLD)
							&& (connList[index].currPhy != HCI_PHY_1_MBPS)
							&& (connList[index].currPhy != SP_PHY_NONE)) {
						// try to go to legacy regular data rate
						phyRqS = phyRq = HCI_PHY_1_MBPS;
					} else if ((connList[index].rssiAvg >= RSSI_S2_THRSHLD)
							&& (connList[index].rssiAvg < RSSI_1M_THRSHLD)
							&& (connList[index].currPhy != SP_PHY_NONE)) {
						// try to go to lower data rate S=2(500kb/s)
						phyRqS = HCI_PHY_CODED;
						phyOpt = LL_PHY_OPT_S2;
						phyRq = BLE5_CODED_S2_PHY;
					} else if (connList[index].rssiAvg < RSSI_S2_THRSHLD) {
						// try to go to lowest data rate S=8(125kb/s)
						phyRqS = HCI_PHY_CODED;
						phyOpt = LL_PHY_OPT_S8;
						phyRq = BLE5_CODED_S8_PHY;
					}
					if ((phyRq != SP_PHY_NONE) &&
					// First check if the request for this phy change is already not honored then don't request for change
							(((connList[index].rqPhy == phyRq)
									&& (connList[index].phyRqFailCnt < 2))
									|| (connList[index].rqPhy != phyRq))) {
						//Initiate PHY change based on RSSI
						SimplePeripheral_setPhy(connList[index].connHandle, 0,
								phyRqS, phyRqS, phyOpt);
						connList[index].phyCngRq = TRUE;

						// If it a request for different phy than failed request, reset the count
						if (connList[index].rqPhy != phyRq) {
							// then reset the request phy counter and requested phy
							connList[index].phyRqFailCnt = 0;
						}

						if (phyOpt == LL_PHY_OPT_NONE) {
							connList[index].rqPhy = phyRq;
						} else if (phyOpt == LL_PHY_OPT_S2) {
							connList[index].rqPhy = BLE5_CODED_S2_PHY;
						} else {
							connList[index].rqPhy = BLE5_CODED_S8_PHY;
						}

					} // end of if ((phyRq != SP_PHY_NONE) && ...
				} // end of if (connList[index].phyCngRq == FALSE)
			} // end of if (rssi != LL_RSSI_NOT_AVAILABLE)

//        Display_printf(dispHandle, SP_ROW_RSSI, 0,
//                       "RSSI:%d dBm, AVG RSSI:%d dBm",
//                       (uint32_t)(rssi),
//                       connList[index].rssiAvg);

		} // end of if (status == SUCCESS)
		break;
	}

	case HCI_LE_READ_PHY: {
		if (status == SUCCESS) {
//        Display_printf(dispHandle, SP_ROW_RSSI + 2, 0, "RXPh: %d, TXPh: %d",
//                       pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
		}
		break;
	}

	default:
		break;
	} // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
 * @fn      SimplePeripheral_initPHYRSSIArray
 *
 * @brief   Initializes the array of structure/s to store data related
 *          RSSI based auto PHy change
 *
 * @param   connHandle - the connection handle
 *
 * @param   addr - pointer to device address
 *
 * @return  index of connection handle
 */
static void SimplePeripheral_initPHYRSSIArray(void) {
//Initialize array to store connection handle and RSSI values
	memset(connList, 0, sizeof(connList));
	for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++) {
		connList[index].connHandle = SP_INVALID_HANDLE;
	}
}

/*********************************************************************
 * @fn      SimplePeripheral_setPhy
 *
 * @brief   Call the HCI set phy API and and add the handle to a
 *          list to match it to an incoming command status event
 */
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
		uint8_t txPhy, uint8_t rxPhy, uint16_t phyOpts) {
// Allocate list entry to store handle for command status
	spConnHandleEntry_t *connHandleEntry = ICall_malloc(
			sizeof(spConnHandleEntry_t));

	if (connHandleEntry) {
		connHandleEntry->connHandle = connHandle;

// Add entry to the phy command status list
		List_put(&setPhyCommStatList, (List_Elem*) connHandleEntry);

// Send PHY Update
		HCI_LE_SetPhyCmd(connHandle, allPhys, txPhy, rxPhy, phyOpts);
	}

	return SUCCESS;
}

/*********************************************************************
 * @fn      SimplePeripheral_updatePHYStat
 *
 * @brief   Update the auto phy update state machine
 *
 * @param   connHandle - the connection handle
 *
 * @return  None
 */
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg) {
	uint8_t connIndex;

	switch (eventCode) {
	case HCI_LE_SET_PHY: {
// Get connection handle from list
		spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t*) List_get(
				&setPhyCommStatList);

		if (connHandleEntry) {
			// Get index from connection handle
			connIndex = SimplePeripheral_getConnIndex(
					connHandleEntry->connHandle);

			ICall_free(connHandleEntry);

			// Is this connection still valid?
			if (connIndex < MAX_NUM_BLE_CONNS) {
				hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t*) pMsg;

				if (pMyMsg->cmdStatus
						== HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE) {
					// Update the phychange request status for active RSSI tracking connection
					connList[connIndex].phyCngRq = FALSE;
					connList[connIndex].phyRqFailCnt++;
				}
			}
		}
		break;
	}

// LE Event - a Phy update has completed or failed
	case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT: {
		hciEvt_BLEPhyUpdateComplete_t *pPUC =
				(hciEvt_BLEPhyUpdateComplete_t*) pMsg;

		if (pPUC) {
			// Get index from connection handle
			connIndex = SimplePeripheral_getConnIndex(pPUC->connHandle);

			// Is this connection still valid?
			if (connIndex < MAX_NUM_BLE_CONNS) {
				// Update the phychange request status for active RSSI tracking connection
				connList[connIndex].phyCngRq = FALSE;

				if (pPUC->status == SUCCESS) {
					connList[connIndex].currPhy = pPUC->rxPhy;
				}
				if (pPUC->rxPhy != connList[connIndex].rqPhy) {
					connList[connIndex].phyRqFailCnt++;
				} else {
					// Reset the request phy counter and requested phy
					connList[connIndex].phyRqFailCnt = 0;
					connList[connIndex].rqPhy = 0;
				}
			}
		}

		break;
	}

	default:
		break;
	} // end of switch (eventCode)
}
