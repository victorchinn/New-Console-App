//
// CPDL-100A/PDL-100A - Programmable Delay Line with HW1 VER 2 HARDWARE WITH NEW MOTOR
// (c) 2016-2018 Colby Instruments, Bellevue, WA.
//
// 11.01.16 ... base minimal code ... start as base to integrate code from ver 1.82 HW1 software
// 12.20.16 ... use 192.168.100.10 as BASE DEFAULT ADDRESS
//
// 01.18.17 ... add support for bar graph display
// 01.21.17	... changes made in SHIFT_REG_INPUT_HW1_VER2.LIB
// 02.13.17 ... add support to display BarGraphDisplay in cmdPROMPT
// 02.22.17	... support specifying to fs resolution in setting delay
// 05.19.17	... added back in support for MODE command
// 05.19.17	... support for step resolution to 0.050 ps per step (50 fs)
// 06.27.17	... support for single channel 312.50 max delay (add 312.5 ps version)
// 06.28.17 ... added #define DEVICE_PDL1000A to distinguish PDL-1000A from default PDL-100A
// 07.26.17 ... initial support to handle overshoot=ON feature by adding 5 ps to each delay setting
// 07.27.17 ... add nv_overshoot_PS default to 5 ps and command to change overshoot amount
// 07.27.17 ... add cmdOVS and cmdOVSQ to set and query overshoot amount in NVRAM
// 08.01.17 ... add cmdDEL_ONE and cmdDEL_TWO and cmdDEL_ONEQ and cmdDEL_TWOQ and cmdDEL_ONE_TWO_Q
// 08.01.17	... these modifications were added in REFACTOR_HW1.LIB
// 08.01.17 *modify cmdSTEP_INC and cmdSTEP_DEC to handle PDL-2000A model
// 08.01.17 *modify EXECUTE_COMMAND to handle PDL-2000A model
// 08.01.17 *modify cmdPROMPT to handle PDL-2000A model
// 08.01.17 * == still in process and not done yet
//
// 01.22.18	PDL-1000A replace PDL-100A
// 01.22.18	Configuration of PDL-1000A starts at 312.50 ps, 625.0 ps, 1.25 ns, 2.50 ns, etc..
// 01.22.18 overshoot wait time support added // _MotorMovementWaitTime and _MotorMovementDelta_PS
// 01.23.18 in COMMANDS.LIB
// 01.24.18 use OVER ON and OVER OFF to turn on off OVERSHOOT, OVS X and OVS? to set overshoot amount
// 01.31.18 add CTSTORE in REFACTOR_HW1.LIB, update DEVICE_NVPARAMETERS
// 01.31.18	add to GLOBAL_SETTINGS
// 02.01.18 0 thru 6250 inclusive for CAL_TABLE for PDL-1000A
// 02.01.18	added g_REMOTE_LOCAL_MODE to indicate in REMOTE (TRUE) or LOCAL (FALSE) mode to avoid terminal prompts
// 02.01.18	use REM ON or REM OFF to set remote/local mode and rem? to query remote/local mode
// 02.02.18	use _CalTableEntryOffsetAmount into each MOTOR_SetDelay is using GLOBAL_SETTINGS.USE_CAL_TABLE
// 02.03.18 CALIBRATION TABLE WORKING !! (subtract offset amount) for correct calibration adjustment in cmdSET_DELAY
// 02.06.18	CTSTOREM and ParseValue for multiple entries into CTSTORE cal table
// 02.06.18 INPUT_BUFFER_SIZE_MAX = 128 (replaced 100 for defined constant == 128)
// 02.06.18	added CTSTOREMQ to display multiple (up to 20) entries in cal table
// 02.07.18 include initializing GLOBAL_SETTINGS.USE_CAL_TABLE in instrument setup at start
// 02.07.18 fixes for when using CTSTORE TABLE in COMMAND.LIB and setting delay
// 03.19.18 g_COMMAND_LINE_COPY is replaced by GLOBAL_SETTINGS.COMMAND_LINE_COPY and code restored
// 03.19.18 in cmdPARSE to handle CTSTORE INFO text data field correctly ...
// 03.27.18 rename PDL-1000A to PDL-100A and use HW1 VER 2 to distinguish different models
// 03.27.18	VERSION 032718 -- code complete
// 03.28.18	replaced PDL100A to DEVICE_PDL100A to support changes in REFACTOR.LIB to handle PDL-200A
// 03.28.18 added stateMT100A_DISPLAY;	// 03.28.18 MT-100A DISPLAY STATE: 1 = CH1+PS,2=CH1+NS,3=CH2+PS,4=CH2+NS,5=CH_BOTH+PS,6=CH_BOTH+NS

// 03.28.18	PRIMARY AND SECONDARY TROMBONE CODE FOR PDL-200A-625PS AND FOR CPDL-200A-1.25NS
// 03.28.18	PRIMARY AND SECONDARY TROMBONE CODE FOR PDL-200A-625PS AND FOR CPDL-200A-1.25NS
// 03.28.18	PRIMARY AND SECONDARY TROMBONE CODE FOR PDL-200A-625PS AND FOR CPDL-200A-1.25NS

// 03.28.18 Use VERSION 032718 as code base start
// 03.28.18	DUAL INDEPENDENT CHANNELS IN DUAL ADDRESS CONFIGURATION (2x ETHERNET ADDRESSES) NO LONGER SUPPORTED
// 03.28.18 PRIMARY TROMBONE AND SECONDARY TROMBONE ONLY (FOR PDL-200A-625PS and CPDL-200A-1.25NS)
// 03.28.18 PDL-200A-625PS is dual channel,(DEVICE_PDL200A) 625 ps in each channel, one ethernet address
// 03.28.18 with a (PRIMARY_TROMBONE), serial connection to (SECONDARY_TROMBONE)
// 03.28.18 CPDL-200A-1.25NS is dual channel,(DEVICE_CPDL200A) 1.25 ns in each channel, one ethernet address
// 03.28.18 with a (PRIMARY_TROMBONE), serial connection to (SECONDARY_TROMBONE)

// 04.10.18	in REFACTOR.LIB: cmdPROMPT - insert _ValueToUseForDisplay for bar graph/determine device type
// 04.10.18 in COMMANDS.LIB: added DEL1,DEL2,DEL1?,DEL2? in EXECUTE_COMMAND_HW1_VER
// 04.10.18 in REFACTOR.LIB: cmdSTEP_INC and cmdSTEP_DEC: replace DISPLAY_SETTINGS with INSTRUMENT_SETTINGS
// 04.10.18 in Init_SERIAL_PORTS: send out initial message on MT-100A
// 04.10.18 in Init_INSTRUMENT_Variables: Handle NS_PS CYCLE MODE on startup
// 04.10.18	in COMMANDS.LIB: EXECUTE_COMMAND_HW1_VER2: added DEL1,DEL2,DEL1?,DEL2?
// 04.10.18 in COMMANDS.LIB: LoadNVParameters_HW1VER2: added g_NVParameters.nv_nsps_cycle_mode = CYCLE_UNIT
// 04.10.18 in COMMANDS.LIB: LoadNVParameters_HW1VER2: default address is 192.168.100.8 for PDL-100A and CPDL-100A
// 04.10.18 for SECONDARY_TROMBONE using SERIAL PORT B, ignore SERIAL PORT E MT-100A MICROTERMINAL
// 04.10.18 for SECONDARY_TROMBONE connect to Secondary Trombone ONLY through ETHERNET PORT (192.168.100.9 default)
// 04.11.18 in REFACTOR.LIB: split Service_SERIAL_PORT_CHAR into Service_SERIAL_PORT_B_CHAR and Service_SERIAL_PORT_E_CHAR
// 04.11.18 in REFACTOR.LIB: cmdRST to handle PRIMARY_TROMBONE to not get hw switch addresses because of conflict with SERIAL PORT B
// 04.13.19 in COMMANDS.LIB: added checkGPIB_HW_ADDR_MSG to handle 01111010 (8 char) msgs from PRI TO SEC TROMBONE over shared serial port
// 04.17.18 in REFACTOR.LIB: handleMenu_Mode: enable overshoot on/off from MT-100A menu for PRIMARY_TROMBONE
// 04.17.18 in COMMANDS.LIB: cmdSET_DELAY: handle overshoot for CHANNEL ONE of PRIMARY_TROMBONE for PDL-200A
// 04.18.18 in REFACTOR.LIB: cmdDEL_ONE and cmdDEL_TWO: use cmdSET_DELAY instead of cmdDEL_PDL100
// 04.18.18 in COMMANDS: cmdSET_DELAY: display delay settings correctly after setting delay
// 04.18.18 use INSTRUMENT_SETTINGS.CURRENT_DELAY_ONE_PS and INSTRUMENT_SETTINGS.CURRENT_DELAY_TWO_PS replaces INSTRUMENT_SETTINGS.CurrentDelayOne/Two
// 04.18.18 base for FINAL PDL-100A CODE
// 04.19.18 add for DEVICE_CPDL100A - disable any MOTOR related code #ifndef DEVICE_CPDL100A
// 04.19.18 in COMMANDS.LIB: cmdSET_DELAY: handle PDL-100A and CPDL-100A in separate #ifdef code structure
// 04.19.18 in REFACTOR.LIB: deviceSET_DELAY: add support for #ifdef DEVICE_CPDL100A
// 04.19.18 in REFACTOR.LIB: cmdPROMPT: change formatting within sprintf for display in DEVICE_CPDL100A or DEVICE_PDL100A
// 04.20.18 in REFACTOR.LIB: cmdCAL: added void _ExecuteTromboneCAL (void) internal function and handle all device types
// 04.20.18 in COMMANDS.LIB: EXECUTE_COMMAND_HW1_VER2: handle option 4= for DEVICE_PDL100A and DEVICE_CPDL100A
// 04.25.18 PDL-100A-OEM as SECONDARY_TROMBONE for PDL-200A-625PS DUAL CHANNEL
// 04.30.18 change #define SIZE_CAL_TABLE 6251 to 1251 to handle only 0.50 ps step resolution table
// 04.30.18 FOR SECONDARY TROMBONE USE: #define SECONDARY_TROMBONE,#define DEVICE_PDL100A, and #define DEVICE_PDL100A_CPDL100A
// 04.30.18 in COMMANDS.LIB: EXECUTE_COMMAND_HW1_VER2: add 4=CTS in Menu for CTSTORE ON OR OFF
// 04.30.18 in REFACTOR.LIB: handleMENU_MODE: handle CTSTORE ON or OFF in DIAG KEY menu option 4
// 04.30.18 in COMMANDS.LIB: cmdSET_DELAY: truncate delay setting depending on SIZE_CAL_TABLE
// 04.30.18 in COMMANDS.LIB: EXECUTE_COMMAND_HW1_VER2: change 3=RESET to 3=RES in MT-100A MENU
// 05.01.18 cmdSET_DELAY: change "Requested delay beyond range ... " to "LIMIT" in error message
// 05.08.18 in COMMANDS.LIB: LoadNVParameters_HW1VER2: initialize the CTSTORE? INFO area and the CTSTORE CAL table
// 05.09.18 in REFACTOR.LIB: cmdCTSTORE: reset/clear CTSTORE table and INFO field in NVParameters in CTSTORE RESET command
// 05.09.18 #define INPUT_BUFFER_SIZE_MAX 256 changed from 128 for CTSTOREM command
//
// 04.30.18 BASE CODE FOR SYSTEM BOARD HW1 VERSION 2 REVISION 4 FOR PDL-100A-625PS-OEM
// 04.30.18 BASE CODE FOR SYSTEM BOARD HW1 VERSION 2 REVISION 4 FOR PDL-100A-625PS-OEM
// 04.30.18 BASE CODE FOR SYSTEM BOARD HW1 VERSION 2 REVISION 4 FOR PDL-100A-625PS-OEM
// 04.30.18
// 04.30.18	FINAL PRODUCTION CODE VERSION 2.00
// 04.30.18	FINAL PRODUCTION CODE VERSION 2.00
// 04.30.18	FINAL PRODUCTION CODE VERSION 2.00
//
// 05.22.18 in REFACTOR_HW1.LIB: cmdNET: net ad SAVE into NVRAM bug fix
// 05.23.18 in REFACTOR_HW1.LIB: cmdREL,cmdRELQ,deviceSET_DELAY: handle #ifdef TEN_X when any section is above 65535 ps in length
// 05.24.18 in COMMANDS.LIB: EXECUTE_COMMAND_HW1_VER2: Removed cmdSET_DEVOPSremoved cmdSET_DEVOPS
// 05.24.18 in REFACTOR_HW1.LIB: cmdSET_DEVOPS, cmdTACS, cmdTACSQ: removed
// 05.25.18 in REFACTOR_HW1.LIB: cmdPROMPT: _NumberOfBars_i bug fix (must be <= 20) and determine if on INTEGER BOUNDARY for LED dislplay
//
// 05.30.18 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 05.30.18 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 05.30.18 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 05.31.18 in RELAY_NEW_PCB.LIB: relaySetupRelaysX: remainder is practically ZERO so make it zero (rather than negative because of rounding error)
// 05.31.18 in RELAY_NEW_PCB.LIB: in relaySendData: removed unused code
// 06.01.18 in REFACTOR_HW1.LIB: cmdDEL_PDL100: restore rounding down code to 0.50 ps per step since HW1VER2 supports only 0.50 ps per step
// 06.01.18 in REFACTOR_HW1.LIB: "Requested delay is beyond range of device." replaced with "LIMIT"
// 06.05.18 in COMMANDS.LIB: cmdSET_DELAY: handle MAX delay above 65.535 ns and precision correctly with rounding down
// 06.05.18 in COMMANDS.LIB: EXECUTE_COMMAND_HW1_VER2: incorrect display settings reported in DEL?
// 09.11.18 in COMMANDS.LIB: cmdSET_DELAY: handle scientific notation when setting delay value - BUG FIX - NOW VERSION 2.01
//
// 09.12.18 BASE CODE FOR SYSTEM BOARD HW1 VERSION 2 REVISION 4 FOR PDL-100A-1.25NS
// 09.12.18 BASE CODE FOR SYSTEM BOARD HW1 VERSION 2 REVISION 4 FOR PDL-100A-1.25NS
// 09.12.18 BASE CODE FOR SYSTEM BOARD HW1 VERSION 2 REVISION 4 FOR PDL-100A-1.25NS
// 09.12.18
// 09.12.18	FINAL PRODUCTION CODE VERSION 2.01
// 09.12.18	FINAL PRODUCTION CODE VERSION 2.01
// 09.12.18	FINAL PRODUCTION CODE VERSION 2.01

// 01.03.19 START WITH VERSION 2.01 PDL-100A-OEM HW1VER2 AND FOLD IN PDL-200A-PRIMARY HW1VER1 CODE
// 01.03.19 ------------------------------------------------------------------------------------------------------------------------------------------
// 01.03.19 ------------------------------------------------------------------------------------------------------------------------------------------
// 01.03.19 ------------------------------------------------------------------------------------------------------------------------------------------
// 01.03.19 in COMMANDS.LIB: cmdSET_DELAY: handle PDL-200A DUAL CHANNEL SINGLE UNIT FORWARD MIGRATION TO HW1VER2 HARDWARE
//
// 01.04.19 added #ifndef SECONDARY_TROMBONE to fix duplicate deviceSN declaration
// 01.04.19	uses new updated COMMANDS.LIB (with support for PDL-200A) 01.03.19
// 01.04.19 for PRIMARY TROMBONE in PDL-200A-625PS define DEVICE_PDL200A and define DEVICE_PDL100A_CPDL100A and define PRIMARY_TROMBONE
// 01.04.19	for SECONDARY TROMBONE in PDL-200A-625PS, PDL-100A-OEM is secondary trombone and define DEVICE_PDL100A and define DEVICE_PDL100A_CPDL100A and define SECONDARY_TROMBONE
//
// 04.10.19 VERSION 1906 FINAL PRODUCTION: PDL-200A_HW1_VER2_01_BASE_CODE-FINAL-PRODUCTION-PDL-200A-SINGLE-UNIT-041019-PRIMARY-SN1906XXXX
// 04.10.19 THIS VERSION HAS NOT BEEN COMPILED TO WORK FOR HW1VER2 HARDWARE ... TBD ... CHANGE DOWNLOAD.LIB FILE AND ADD REFACTOR_HW1.LIB
//
// 06.11.19 VERSION 1909 FINAL PRODUCTION: PDL-200A_HW1_VER2_01_BASE_CODE-FINAL-PRODUCTION-PDL-200A-SINGLE-UNIT-MMDDYY-PRIMARY-SN1909XXXX.C
// 06.11.19 VERSION 1909 FINAL PRODUCTION: PDL-200A_HW1_VER2_01_BASE_CODE-FINAL-PRODUCTION-PDL-200A-SINGLE-UNIT-MMDDYY-PRIMARY-SN1909XXXX.C
// 07.30.19 in COMMANDS.LIB: cmdSET_DELAY: BUG FIX: OPC bit was not set for GPIB after completing operation
// 07.30.19 VERSION 1909 FINAL PRODUCTION: PDL-100A_HW1_VER2_01_BASE_CODE-FINAL-PRODUCTION-PDL-100A-OEM-MMDDYY-PDL100A-SN1909XXXX
//
// 07.30.19	FINAL PRODUCTION CODE VERSION 2.82 PDL-200A-625PS VER 2.82
// 07.30.19	FINAL PRODUCTION CODE VERSION 2.82 PDL-200A-625PS VER 2.82
// 07.30.19	FINAL PRODUCTION CODE VERSION 2.82 PDL-200A-625PS VER 2.82
//
// 02.17.20 BUILD 2003XXXX

// 01.26.21	VER 2.83 USING UPDATED METHODS.LIB AND REFACTOR.LIB AND #DHCP_SEND_HOSTNAME

// 01.25.21	HOSTNAME FEATURE VERSION 2.20
// 01.25.21 USE NEW UPDATED ENET HW1V2 LIB FILE
// 01.25.21	add #ifdef DHCP_SEND_HOSTNAME
// 01.25.21 added in NVParameters char  nv_hostname[16];     // hostname
// 01.25.21 add gRet
// 01.26.21 replace HOST with four digits of serial number
// 01.26.21 uses updated

// 01.26.21	FINAL PRODUCTION CODE VERSION 2.83
// 01.26.21	FINAL PRODUCTION CODE VERSION 2.83
// 01.26.21	FINAL PRODUCTION CODE VERSION 2.83
// RD012821
// 01.28.21	using RD012821 VERSIONS OF LIB: METHODS, COMMANDS, AND REFACTOR
// RD01 WEB_SERVER IS NOW ENABLED BY SPECIFYING #define ENABLE_WEB_SERVER
// 02.07.21	use #define WEB_SERVER when adding code to support the enabled web server
// RD02 removed motor movement wait in COMMANDS.LIB with #define NOMOTORWAIT
// RD02 changes were all in COMMANDS.LIB and not required in MAIN code
// RD02 added #define NET_STATUS into ENET LIB to use for network status LED
// RD03	added INSTRUMENT_SETTINGS.CURRENT BAR GRAPH AND CURRENT LED STATUS - global variables
// RD03 added
// RD03 added #define TOP_TO_BOTTOM_NEW_LED for cmdPrompt in REFACTOR_HW1.LIB to FLIP BYTES TOP TO BOTTOM FOR LEDS
//
// RD04	PDL-200A CODE
// RD030421 added #define X_CHASSIS
// RD030421 GPIB IS DISABLED
// RD030421 CHECK FOR SWITCH IS ADDED
// RD030421 04.06.21 USE THIS AS BASE FOR WEB XT-200 FIRST VERSION
// RD030421 04.06.21 ADDED 	// RD030421  #ifndef NO_GPIB_XT_200_HW1V2 FOR USE IN REFACTOR.LIB

// 04.09.21 WEB WORKING !!

/* Set default scope */
#class auto
#memmap xmem // default to use eXtended memory (rather than ROOT) space

//#nodebug // enable to set NODEBUG code

// 03.28.18 FOR PDL-200A-1.25NS PRIMARY_TROMBONE AND FOR DUAL CHANNEL SINGLE UNIT PRIMARY TROMBONE PDL-100A-625PS DUAL CHANNEL SINGLE UNIT
// 01.03.19
#define PRIMARY_TROMBONE
// 03.28.18 FOR PDL-200A-1.25NS SECONDARY_TROMBONE AND FOR DUAL CHANNEL SINGLE UNIT PRIMARY TROMBONE PDL-100A-625PS DUAL CHANNEL SINGLE UNIT
//#define SECONDARY_TROMBONE

#define BUSY_WAIT
#define PORTA_AUX_IO	 // This must be defined before including Rabbit Libraries when using AUX IO
#define ETHERNET_ENABLED // ENABLE this for TCPIP code for RCM3710 else DISABLE for RCM3610
//RD030421
//#define GPIB_ENABLED	 // ENABLE this for GPIB to initialize TNT5002 (standard)
#define NVSTORAGE		 // ENABLE this to use NVSTORAGE (standard)

// #define PDL100A
//    -or-
// #define CPDL
//---------------------------------------------------------------------------------------------------------------------//

// 03.28.18 DEFINE FOR THE EXACT MODEL TYPE (uncomment SPECIFIC model type)
// 03.28.18 for CPDL VERSIONS (not tested yet...need to work with REFACTOR.LIB)
//
// MUST DEFINE ONE SPECIFIC DEVICE TYPE
//#define DEVICE_PDL100A
//01.03.19
#define DEVICE_PDL200A
//#define DEVICE_CPDL100A
//#define DEVICE_CPDL200A
//#define DEVICE_PDL100A_312PT5
//
// MUST DEFINE ONE MODEL TYPE
//#define DEVICE_PDL100A_CPDL100A
//01.03.19
#define DEVICE_PDL200A_CPDL200A

// RD03 TOP_TO_BOTTOM_NEW_LED ADDED USE THIS FOR NEW LED
#define TOP_TO_BOTTOM_NEW_LED

// RD02 ENABLE NET STATUS FOR GLOBAL VARIABLE FOR LED
#define NET_STATUS

// RD030421 ENABLE BIT 21 FOR NEW X CHASSIS FOR NETWORK LED STATUS
#define X_CHASSIS

// RD030421 DEFINE NO GPIB IN REFACTOR.LIB
#define NO_GPIB_XT_200_HW1V2

//---------------------------------------------------------------------------------------------------------------------//

/******* Internal function declarations ************************************/
//
// Functions defined in MAINPDL100A and used in other libraries
// need to be defined here first before the libraries are # used
//

//
// GPIB_SETTINGS
//

typedef struct gpibvars
{
	unsigned char ESR_MASK;
	unsigned char SET_OPC_BIT; // set to turn on OPC bit after every command completion
	int TACS_WAIT;			   //added 09.14.07 for # of wait cycles after sending data on TACS (prevent GPIB timeout)
	unsigned char ESR_register_bits;
	int DEVICE_CLEAR;
	int INTERFACE_CLEAR; //added 09.19.07
	int DEVICE_TRIGGER;	 //added 09.19.07
};
struct gpibvars GPIB_SETTINGS; // create the GPIB structure here so other libs can access them

// TURN ON OR OFF WEB_SERVER CAPABILITY
#define ENABLE_WEB_SERVER

//Include libs
// RD030421
#use PDLCONFIG_HW1_VER2_DOWNLOAD_B2106_RD030421.LIB // this library commands which other libraries to use
#use REFACTOR_HW1_B2106_RD030421.LIB	 // holds all the common code from HW1 VER1 code
//#use "sflash.lib"                      // add support for serial flash chip

// ERROR CODE DEFINITIONS

#define NO_ERROR 0
#define INVALID_COMMAND 1
#define INVALID_ARG 2
#define NO_CALIBRATION 3
#define DELAY_LIMIT 4
#define DELAY_NOT_SET 5
#define STACK_FULL 6 // added 04.25.07
#define STACK_EMPTY 7
#define INVALID_RESPONSE 8
#define BUFFER_OVERFLOW 99

#define PS 1
#define NS 2
#define ON 1
#define OFF 0
#define DEVICE_SERIAL 1
#define DEVICE_PARALLEL 2
#define DEVICE_HYBRID 3
#define DEVICE_CHANNEL_ONE 4
#define DEVICE_CHANNEL_TWO 5

#define TMO_DEFAULT 0 // 5 = 1 ms \
					  // TIMEOUT TIME TO SIT IN TNT5002 RECEIVE CALL
// 08.01.17 THESE ARE USED FOR PDL-200A
#define CHANNEL_ONE 1
#define CHANNEL_TWO 2
#define CHANNEL_BOTH 3

#define CHAN_ONE_PS 1
#define CHAN_ONE_NS 2
#define CHAN_TWO_PS 3
#define CHAN_TWO_NS 4
#define CHAN_BOTH_PS 5
#define CHAN_BOTH_NS 6

#define CYCLE_SEQ 1
#define CYCLE_UNIT 2
#define CYCLE_CHANNEL 3
// 08.01.17 THESE ARE USED FOR PDL-200A

#define MAX_STACK_SIZE 100
#define RATE_MIN 100 // lowest motor rate
#define RATE_MAX 550 // highest motor rate
#define XDD_MIN 500
#define XDD_MAX 2000

#define COMMAND_MAX 500 // 06.18.08
#define BUFFER_MAX 500
#define DISP_LINE_MAX 512
#define cmdARG1_MAX 20
#define cmdARG2_MAX 128
#define cmdARG3_MAX 20

#define INPUT_BUFFER_SIZE_MAX 256 // 05.09.18 was 128, increase to 256 for CTSTOREM command
#define DEFAULT_TACS_WAIT 2000	  // optimal wait time after TACS and LACS to let programmed I/O finish

// 11.1.2016 NEW MOTOR DEFINITIONS
// serial buffer size for SERIAL PORT C RS-485 MOTOR
#define CINBUFSIZE 31
#define COUTBUFSIZE 31

// MAX STEPS
#define MAX_NUMBER_MOTOR_STEPS 520312

// 3000 ms to move one MIN delay to MAX delay wait time // 02.03.18 change to 4000
#define MIN_MAX_MOTOR_WAIT_TIME 4000

//#define MAX_DELAY_SETTING 312.500
#ifdef DEVICE_PDL100A_312PT5
#define MAX_DELAY_SETTING 312.500
#endif

// MUST EDIT FOR EACH MAXIMUM DELAY SIZE STEP# 0
#ifdef DEVICE_PDL100A
#define MAX_DELAY_SETTING 625
#endif

// 03.28.18 define DEVICE_PDL200A
#ifdef DEVICE_PDL200A
#define MAX_DELAY_SETTING 625.00
#endif

#define SC_CODE_MOTOR_ENABLED 0x0001
#define SC_CODE_DRIVE_FAULT 0x0004
#define SC_CODE_IN_POSITION 0x0008
#define SC_CODE_ALARM_PRESENT 0x0200

#define AL_CODE_POS_LIMIT 0x0001
#define AL_CODE_CCW_LIMIT 0x0002
#define AL_CODE_CW_LIMIT 0x0004
#define AL_CODE_OVER_TEMP 0x0008
#define AL_CODE_COMM_ERROR 0x0400
#define AL_CODE_NO_MOVE 0x1000

#define DI_STOP_DISTANCE_AFTER_SENSOR 1
#define HALF_SEC 500
#define ONE_SEC 1000
#define MAX_CALIBRATION_POINTS 315
#define CAL_PTS_ZERO_TO_TWO 21

#define WAIT 1
#define NOWAIT 0

#define NVPARAMS_USERBLOCK_LOCATION 0
#define MAGIC_XSUM_BYTE 0x44 // This can be any random number besides 0x00 and 0xFF.

// 01.26.21 TO ADD DHCP_SEND_HOSTNAME FEATURE
#define DHCP_SEND_HOSTNAME

// 04.30.18 change from 6251 to 1251 (0 to 625.0 ps with 0.50 ps step resolution)
//#define SIZE_CAL_TABLE 6251
#define SIZE_CAL_TABLE 1251

#define SIZE_CAL_INFO_FIELD 240

#define REMOTE 1
#define LOCAL 0

// 01.26.21 TO ADD DHCP_SEND_HOSTNAME FEATURE
#define DHCP_SEND_HOSTNAME
typedef struct
{

	unsigned int nv_dev_ops;			   //number of operations device has performed  (not used! else too much wear on Flash memory user block, only 10K writes lifetime)
	long nv_ip_addr;					   // static IP address
	long nv_netmask;					   // netmask
	long nv_gateway;					   // gateway
	int nv_port;						   // port ID to use
	char nv_useDHCP;					   // TRUE == use DHCP, FALSE == use static IP addr
	char nv_terminal_mode;				   // TRUE == use RS-232 port in TERMINAL MODE, FALSE == use RS-232 port in MT-100A LCD TERMINAL
	int nv_GPIB_addr;					   // GPIB address for instrument (if used in OEM version)
										   // NOTE: The following XORsum byte should remain in this structure.
	char nv_XSUM;						   // This is used to test the validity of the structure in the User Block.
	int nv_overshoot;					   // indicate whether to overshoot delay settings (for increased repeatability)
	int nv_overshoot_PS;				   // amount of overshoot to apply in ps
	int nv_autodrop;					   // 03.16.15 save the autodrop connections over a power cycle
	int nv_nsps_cycle_mode;				   // 03.28.18 added for PDL-200A/CPDL-200A stores the ns/ps button cycle mode (default = CYCLE_UNIT)
	int nv_cal_table[SIZE_CAL_TABLE];	   // float for PDL-1000A # of PS with 3 decimal pt precision adjustment
	char nv_cal_info[SIZE_CAL_INFO_FIELD]; // calibration table information max of 128 characters
	char nv_useCTSTORE;					   // use CAL TABLE (CTSTORE)

#ifdef ENABLE_WEB_SERVER
	char nv_description[200]; // 02.04.21 instrument description
	char nv_password[16];	  // 02.04.21 added for web_server_support and changed length (from 40) to 16
#endif
	char nv_hostname[16];  // hostname // 01.26.21 added
	int nv_cal_date_month; // calibration month, day, year
	int nv_cal_date_day;
	int nv_cal_date_year;

} DEVICE_NVPARAMETERS;

DEVICE_NVPARAMETERS g_NVParameters;

// ----------------------------------------------------------------
// GLOBAL Variables
// ----------------------------------------------------------------
enum MOTOR_COMMAND
{
	AL,
	IP_COMMAND,
	SC,
	DI,
	IS,
	AR,
	SP,
	EP,
	FL,
	FP,
	RE,
	ME,
	VE,
	AC,
	DE,
	ER,
	MO,
	MD,
	ML,
	MR,
	RS
};

// new methods and commands necessary for HW1 VER 2 hardware and new motor
// in LIBX library folder

// 01.28.21	USE A SPECIAL VERSION OF METHODS LIBRARY
#use METHODS_B2106.LIB

// 01.28.21	USE A SPECIAL VERSION OF METHODS LIBRARY
#use COMMANDS_B2106.LIB

// MOTOR COMMANDS
const static char Motor_FL[] = "0FL";
const static char Motor_SC[] = "0SC";
const static char Motor_DI[] = "0DI";
const static char Motor_FS[] = "0FS";
const static char Motor_IS[] = "0IS";
const static char Motor_DL[] = "0DL";
const static char Motor_EP[] = "0EP";
const static char Motor_SP[] = "0SP";
const static char Motor_IP[] = "0IP"; // read current location
const static char Motor_AR[] = "0AR"; // alarm reset
const static char Motor_AL[] = "0AL"; // alarm code
const static char Motor_FP[] = "0FP"; // feed position
const static char Motor_RE[] = "0RE"; // reset
const static char Motor_ME[] = "0ME"; // motor enable
const static char Motor_VE[] = "0VE"; // read velocity
const static char Motor_AC[] = "0AC"; // read acceleration
const static char Motor_DE[] = "0DE"; // read deceleration
const static char Motor_ER[] = "0ER"; // read encoder resolution
const static char Motor_RS[] = "0RS"; // request status

// 10.26.16
// const static char Motor_MO[] = "0FS1L";   // feed motor until signal goes LOW -- ONLY FOR PROTOTYPE HARDWARE //
const static char Motor_MO[] = "0FS1H"; // feed motor until signal goes HIGH - HW1 VER2 //

const static char Motor_MD[] = "0DI"; // set DI value

const static char se1[] = "SERIAL E OUT\r";
const static char sePrompt[] = "#";
const static char sONEPSOUT[] = "0FL1665\r";
const static char sONEPSIN[] = "0FL-1665\r";
const static char strCRLF[] = "\r\n";

//
// GLOBALS VARIABLES
//

short int stateSERIAL_CHAR_PORTC_IN;
short int stateCOMMAND_LINE_PORTC_ENTER;
short int stateCOMMAND_LINE_PORTC_ENTER_INIT_MOTOR;
short int stateSERIAL_CHAR_PORTC_IN_INIT_MOTOR;

char CharFromSerialPortC;
static char strCOMMAND_LINE_PORTC[100];
static char strCharFromSerialPortC[2];

char cmdCOMMAND[INPUT_BUFFER_SIZE_MAX]; // contains entire command line input  // was 100 02.01.18
char cmdARG1[cmdARG1_MAX];				// argument#1 - after parsing
char cmdARG2[cmdARG2_MAX];				// argument#2
char cmdARG3[cmdARG3_MAX];				// argument#3
char cmdBUFFER[INPUT_BUFFER_SIZE_MAX];	// pending command line buffer             // was 100 02.01.18
short cmdINDEX;							// index into pending cmdCOMMAND[] array

static const char COMMAND_PROMPT[] = "\n\rCommand:";

//
// DISPLAY_SETTINGS
//
typedef struct Settings
{
	// for PDL-100A
	float CURRENT_DELAY_F;	   // current delay value as float for display in Command line
	float CURRENT_DELAY_E;	   // current delay (as a float var) for display
	float CURRENT_STEP_SIZE_E; // current step size (as a float var) for display

	// for PDL-200A use these instead
	float CURRENT_DELAY_ONE_F; // current delay value as float for display in Command line
	float CURRENT_DELAY_ONE_E; // current delay (as a float var) for display

	float CURRENT_DELAY_TWO_F; // current delay value as float for display in Command line
	float CURRENT_DELAY_TWO_E; // current delay (as a float var) for display

	unsigned char CURRENT_GPIB_ADDR; //current GPIB address
};
struct Settings DISPLAY_SETTINGS;

//
// INSTRUMENT_SETTINGS
//
typedef struct InstSettings
{
	short CURRENT_UNITS;				  // either PS or NS.
	float CURRENT_STEP_SIZE;			  // current Step Size
	float CURRENT_DELAY;				  // current delay value in picoseconds
	float CURRENT_DELAY_ONE_PS;			  // current delay value in picoseconds for Channel One
	float CURRENT_DELAY_TWO_PS;			  // current delay value in picoseconds for Channel Two
	float CURRENT_DELAY_ONE_F;			  // 03.28.18 added
	float CURRENT_DELAY_TWO_F;			  // 03.28.18 added
	float CURRENT_DELAY_ONE_E;			  // 03.28.18 added
	float CURRENT_DELAY_TWO_E;			  // 03.28.18 added
	unsigned long CURRENT_BAR_GRAPH;	  // ADDED 02.05.2021 FOR BARGRAPH RD02
	unsigned char CURRENT_LAN_LED_STATUS; // ADDED 02.05.2021 FOR BARGRAPH RD02
};
struct InstSettings INSTRUMENT_SETTINGS;

//
// INSTRUMENT
// Hardware State machine -- boolean variables
// these stateXXXXX variables are used as triggers (in waitfor() commands) to trigger action
//
typedef struct statevars
{
	unsigned char statePARSE;			   // TRUE = Parse cmdCOMMAND[] array into cmdARG1, cmdARG2, cmdARG3
	unsigned char stateERROR;			   // TRUE = Error condition.  See stateERROR_CODE for error code.
	unsigned char stateSER_PORT_B_CHAR;	   // TRUE = valid char available on serial port B
	unsigned char stateSER_PORT_E_CHAR;	   // TRUE = valid char available on serial port E (MT-100A)
	unsigned char stateSER_MOTOR_CHAR;	   // TRUE = valid char available on serial port MOTOR
	unsigned char stateERROR_CODE;		   // contains error_code from previous command.  It is cleared to 0 after each
										   // command is successfully performed.
	unsigned char stateCMD_FROM_GPIB;	   // TRUE indicates cmdCOMMAND[] came via GPIB
	unsigned char stateCMD_FROM_TERM;	   // TRUE indicates cmdCOMMAND[] came via SERIAL TERMINAL
	unsigned char stateCMD_FROM_LAN;	   // TRUE indicates cmdCOMMAND[] came via LAN
	unsigned char stateDEVICE_MODE;		   // == DEVICE_PARALLEL or == DEVICE_SERIAL to interpret delay commands
	unsigned char stateDEVICE_MODE_MT100A; // TRUE indicates running MT-100A over serial port, FALSE indicates running HyperTerm on serial port
	unsigned char stateDEVICE_DISPLAY_NS;  // TRUE to display in NS, FALSE to display in PS on COMMAND PROMPT. default depends on model type
	int stateMENU_MODE;					   // 0 == NORMAL, else any other number indicates MENU MODE #
	unsigned char stateMT100A_DISPLAY;	   // 03.28.18 MT-100A DISPLAY STATE: 1 = CH1+PS,2=CH1+NS,3=CH2+PS,4=CH2+NS,5=CH_BOTH+PS,6=CH_BOTH+NS
	unsigned char stateCYCLE_MODE;		   // 03.28.18 set the cycle mode of hitting the NS/PS button MT-100A
	short stateGPIB_DATA_IN;			   // TRUE == GPIB data available to process
	short stateGPIB_DATA_OUT;			   // TRUE == GPIB data available to send out
	short stateTCPIP_DATA_IN;			   // TRUE == TCPIP data available to process
	short stateTCPIP_DATA_OUT;			   // TRUE == TCPIP data available to process
	unsigned int stateMENU_MODE_LAST_RELAY;
	unsigned int stateGPIB_LINES;				//added for testing 04.18.07
	int deviceOPERATIONS;						// # of device operations since power ON
	unsigned char stateMOTOR_ResponseProcessed; // TRUE indicates a response to a sent command has been processed
	long stateMOTOR_Response_VALUE;
	unsigned char stateMOTOR_ResponseACK;
	enum MOTOR_COMMAND stateMOTOR_ResponseACK_TYPE;
	enum MOTOR_COMMAND stateMOTOR_Response_TYPE;
	char deviceOPTION[5];			  // to store device option type, e.g. OEM or 625PS, etc...
	unsigned char stateDEL_CHANNEL;	  // 1 == CHANNEL 1 and 2 = CHANNEL 2 // 08.01.2017 FOR PDL-2000A
	unsigned int charSER_PORT_B_ONLY; // actual character from Serial Port B
	unsigned int charSER_PORT_E_ONLY; // actual character from Serial Port E
};
struct statevars INSTRUMENT;

//
// HW_RELAYS
//
typedef struct hwrelays
{
	int SETTINGS;						// 16 BIT INTEGER VALUE CORRESPONDING ONE BIT TO EACH RELAY
	unsigned int RELAY_ON_OFF[17];		// Relay values 1 thru 16, either TRUE=1 or FALSE=0
	unsigned int RELAY_DELAY_VALUE[17]; // Relay values 1 thru 16, unsigned int = 65535 MAX
	int NUM_OF_SECTIONS;				// # of installed RELAY_SECTIONS (used in CPDL delay calculations)
	unsigned char SWITCH_SETTINGS;		// 8 bit value containing last read of hw switches
};
struct hwrelays HW_RELAYS;

//
// GLOBAL_SETTINGS
//
typedef struct globalvars
{
	unsigned char LAST_RELAY_SECTION_ODD;		 //indicate whether last relay section is on even (FALSE) or odd (TRUE) boundary
	unsigned char AUTO_SET_PLUS, AUTO_SET_MINUS; // auto set to new delay value after specifying step size on +/- buttons on Microterminal
	unsigned char CALIBRATION_TEST_RESULT;		 // results of last calibration check (*CAL), *CAL? to get results
	float SET_OPTO_LOCN;						 //location of motor for special set opto detector voltage routine in DIAG menu
	unsigned char ENABLE_STEP_DIAG;				 // enables single step (full or half) to set voltage in DIAG menu
	long ELAPSED_DELAY_TIME;					 // elapsed time
	long START_DELAY_TIME;						 // start time before motor moves to new delay
	long END_DELAY_TIME;						 // end time after motor moves to new delay
	unsigned char userOVERSHOOT;				 // turn ON or OFF overshoot  added 10.04.06
	int QUEUED_ERROR_NUMBER;					 //added 04.18.07
	int tcpipSession;							 // The session that sent the command  // added 03.05.08 for multi-sessions
	short COMMAND_CONTINUE;						 // TRUE = semicolon in command line more command to process
	unsigned long SUM_RELAYS_LESS_1;
	char USE_CAL_TABLE; // 09.01.08 // 01.31.18 flag to indicate use NVRAM CAL TABLE OR NOT
	char COMMAND_LINE_COPY[256];
	char REMOTE_LOCAL_MODE; // indicate in REMOTE MODE (TRUE) or LOCAL MODE (FALSE)
};
struct globalvars GLOBAL_SETTINGS;

//
// MOTOR
//
typedef struct motorvars
{
	long CurrentStepPosition;
	float CurrentDelaySettingPS;
	float CurrentStepSizePS;
	char RESPONSE_Text[25]; // contains entire response string from MOTOR
	long RESPONSE_Value;
};
struct motorvars MOTOR;

//
// PARAMETERS
//
static const char swVERSION[] = "V2.83"; //VERSION 2.0 and above is for HW1 VER2 Hardware

typedef struct paramvars
{
	float deviceMAX_DELAY_PS;
	float deviceMAX_DELAY_NS; // for CPDL display startup message
	float deviceSTEP_SIZE_NS; // step size in NS (for display on startup) VER 1.7
};
struct paramvars PARAMETERS;

//
// BUFFERS
//
typedef struct buffervars
{
	char DISPLAY_LINE[INPUT_BUFFER_SIZE_MAX];		// used as output buffer to display on terminal
	char COMMAND_LINE[INPUT_BUFFER_SIZE_MAX];		// used to build Command Line display
	char gpibBUFFER_IN[500];						// input buffer for received chars from GPIB
	char gpibBUFFER_OUT[500];						// output buffer for chars ready to send over GPIB
	char tcpipBUFFER_IN[500];						// input buffer for chars received over TCPIP
	char tcpipBUFFER_OUT[500];						// output buffer for chars received over TCPIP
	char INPUT_COMMAND_LINE[INPUT_BUFFER_SIZE_MAX]; // buffer to hold rolling parsed command line 04.18.07
};
struct buffervars BUFFERS;

//---------------------------------------------------------------------------------------------------------------------//
//
// local variables for MAIN:
//
unsigned int _serial36;
char buf[150];
short i, j, k;
int charSER_PORT;
int charSER_MOTOR;
unsigned char keyESC;
unsigned int wait;
unsigned int wait_loops;

// 01.26.21 FOR DHCP_SEND_HOSTNAME FEATURE
static const char g_StaMem_HostName[16] = "COLBY_XT200_RD\0"; // EDIT HERE // DHCP_SEND_HOSTNAME feature
char g_DynMem_HostName[16];
char *g_RetCode;

//
// GLOBAL ITEMS SPECIFIC TO EACH DEVICE BUILT
//
//---------------------------------------------------------------------------------------------------------------------//
// ENTER SERIAL NUMBER, MANUFACTURE DATE, AND MAXIMUM DELAY
//
// STEP#1 THESE MUST BE SET PER EACH DEVICE MANUFACTURED!!

static const char deviceNAME[] = "Programmable Delay Line"; //NAME

// SELECT THIS OPTION FOR PDL-100A (dual trombone unit, each with one 312.50 single trombone tube)
#ifdef DEVICE_PDL100A_312PT5
static const char deviceMODEL[] = "PDL-100A";							   // MODEL NAME PDL-100A or CPDL
static const char versionString[] = "PDL-100A-312PS,V2.02,HW1V2,1804xxxx"; //Used for ethernet downloader  -- added 01.17.07 for WEB_SERVER
#endif

#ifdef DEVICE_PDL100A
static const char deviceMODEL[] = "PDL-100A";							   // MODEL NAME PDL-100A or CPDL
static const char versionString[] = "PDL-100A-625PS,V2.02,HW1V2,1812XXXX"; // Used for ethernet downloader  -- added 01.17.07 for WEB_SERVER
static const char deviceOPTION[] = "OEM";								   // FOR PDL_100A OPTION # INSTALLED, e.g "000","OEM",or "010","020",...
static const char deviceMANUFACTURER[] = "Colby Instruments";			   //MANUFACTURER NAME
#endif

#ifdef DEVICE_PDL200A
static const char deviceMODEL[] = "PDL-200A";							   // MODEL NAME PDL-200A
static const char versionString[] = "PDL-200A-625PS,V2.82,HW1V2,COLBY_XT200_RD"; // EDIT HERE Used for ethernet downloader  -- added 01.17.07 for WEB_SERVER
static const char deviceOPTION[] = "200";								   // FOR PDL_100A OPTION # INSTALLED, e.g "000","OEM",or "010","020",...
static const char deviceMANUFACTURER[] = "Colby Instruments";			   // MANUFACTURER NAME
#endif

#ifdef ENABLE_WEB_SERVER
static const char deviceIVIDRIVER[] = "ColbyPDM200A"; // 02.06.2021 RD01
static const char deviceHOMEPAGE[] = "http://www.colbyinstruments.com";
#endif

// 04.20.18 define if want to emulate ORIGINAL CPDL-100A device (not normally used)
//#define ORG_CPDL

#ifdef DEVICE_CPDL100A
static const char deviceIDN_DISPLAY[] = "50.00NS-10.0PS-13";					 // FOR BASE MODEL 625.0PS
static const char deviceOPTION[] = "10.23";										 // FOR PDL_100A OPTION # INSTALLED, e.g "000","OEM",or "010","020",...
static const char versionString[] = "CPDL-100A-50NS-10PS-13,V2.02,HW1,1804xxxx"; //Used for ethernet downloader  -- added 01.17.07 for WEB_SERVER
#define MAX_DELAY_SETTING 50000.0

#ifdef ORG_CPDL
static const char deviceMODEL[] = "CPDL/.01/8.0";				  // MODEL NAME CPDL for compatability for LBL 05.30.07
static const char deviceMANUFACTURER[] = "COLBY INSTRUMENTS INC"; //MANUFACTURER NAME for LBL 05.30.07
#else
static const char deviceMODEL[] = "CPDL-100A";				  // MODEL NAME PDL-100A or CPDL
static const char deviceMANUFACTURER[] = "Colby Instruments"; //MANUFACTURER NAME
#endif
#endif

#ifdef PRIMARY_TROMBONE
static const char deviceSN[] = "COLBY_XT200_RD-PRI"; //EDIT HERE  ENTER YYMM and SERIAL NUMBER HERE e.g. "YYMMxxxx" for 1.25NS UNIT
#endif
#ifdef SECONDARY_TROMBONE
static const char deviceSN[] = "XXXXXXXX-SEC"; //ENTER YYMM and SERIAL NUMBER HERE e.g. "YYMMxxxx"
#endif

#ifdef DEVICE_PDL100A_CPDL100A
#ifndef SECONDARY_TROMBONE
static const char deviceSN[] = "1812XXXX"; // ENTER YYMM and SERIAL NUMBER HERE e.g. "YYMMxxxx"
#endif
#endif

#ifdef DEVICE_PDL100A_312PT5
static const char deviceIDN_DISPLAY[] = "312.50PS"; //FOR BASE MODEL PDL-100A 312.50 PS DUAL CHANNEL
#endif

#ifdef DEVICE_PDL100A
static const char deviceIDN_DISPLAY[] = "625.0PS"; // FOR BASE MODEL 625.0PS
#endif

#ifdef DEVICE_PDL200A
static const char deviceIDN_DISPLAY[] = "625.0PS"; //FOR BASE MODEL 625.0PS
#endif

#ifdef NO_GPIB_XT_200_HW1V2
unsigned long int DATA_COUNT;
#endif

//---------------------------------------------------------------------------------------------------------------------//

// FUNCTION DEFINITIONS USED IN MAIN.C

void Init_SERIALPORTS(void)
{

	// NOTE: On some RCM3710s, the freq_divider variable is incorrectly set because
	// of the long rise time in the +5 power supply voltage. freq_divider is used
	// to set the serial port baud rate.  If set incorrectly, then garbage characters will
	// be displayed over serial port.
	// REMEDY: remove capacitor C11 (for faster rise time) in +5v AND
	// force freq_divider = 36 before opening serial ports. This has not been
	// a problem with RCM3720.

	serEclose(); //added 05.14.06
	_serial36 = FALSE;

	if (freq_divider != 36)
	{ // 36 is constant used for RCM3710 at x.xx MHz
		freq_divider = 36;
		_serial36 = TRUE;
	}

	//
	// SERIAL PORT E IS FOR THE MT-100A MICROTERMINAL
	//
	serEopen(9600); // Serial Port E is the EIA level RS-232 port
	serEwrFlush();	// located on Power Supply Board and on the
	serErdFlush();	// PDL-100A trombone BACK END MT-100A port

	//
	// SERIAL PORT B IS FOR THE AUX SECONDARY SERIAL PORT
	//
	serBopen(9600); // Serial Port B is the EIA level RS-232 port
	serBflowcontrolOff();
	serBwrFlush(); // located on PDL-100A controller board
	serBrdFlush(); // can be made to accept TTL level RS-232
				   // by removing MAX232 chip

	//
	// SERIAL PORT C IS FOR THE RS-485 PORT TO CONTROL THE NEW MOTOR
	//
	serCopen(9600);
	serCflowcontrolOff();
	serCwrFlush();
	serCrdFlush();

	prtTERM("#"); // clear the LCD display
	if (_serial36 == TRUE)
	{
		prtTERM("*");
	} // BEEP THE TERMINAL AT STARTUP

	//04.18.07  for error message testing
	GLOBAL_SETTINGS.QUEUED_ERROR_NUMBER = 0;

// 04.10.18 send out initial message on MT-100A
#ifdef DEVICE_PDL100A
	prtTERM(" PDL100A-HW1-V2 ");
	prtTERM("\tCOLBYINSTRUMENTS\r\n");
#endif
#ifdef DEVICE_CPDL100A
	prtTERM("CPDL100A-HW1-V2 ");
	prtTERM("\tCOLBYINSTRUMENTS\r\n");
#endif
#ifdef DEVICE_PDL200A
	prtTERM(" PDL200A-HW1-V2 ");
	prtTERM("\tCOLBYINSTRUMENTS\r\n");
#endif
#ifdef DEVICE_CPDL200A
	prtTERM("CPDL200A-HW1-V2 ");
	prtTERM("\tCOLBYINSTRUMENTS\r\n");
#endif
} // void Init_SERIALPORTS(void) //

void Init_RELAYS(void)
{

	//Initialize Hardware Relays
	HW_RELAYS.SETTINGS = 0x0000; // hardware Relay Settings
	relayInit();
	relaySetRelay(HW_RELAYS.SETTINGS); // turn them all OFF

	msDelay(250);
	HW_RELAYS.SETTINGS = 0xFFFF;	   // turn them all ON
	relaySetRelay(HW_RELAYS.SETTINGS); // turn them all ON

	msDelay(250);
	HW_RELAYS.SETTINGS = 0x0000;	   // turn them all OFF
	relaySetRelay(HW_RELAYS.SETTINGS); // turn them all OFF

	//***********************************************************************************************************************
	// MUST DEFINE FOR EACH DEVICE MANUFACTURED
	//
	// STEP#2 //
	//
	// DEFINE EACH RELAY SECTION HERE WHETHER FOR PDL-100A OR CPDL-100A MODELS
	//
	//---------------------------------------------------------------------------------------------------------------------//

	//
	// # of picoseconds delay for EACH relay section must be defined
	//

	HW_RELAYS.RELAY_DELAY_VALUE[0] = 625; // # of picoseconds delay for each relay section //
	HW_RELAYS.RELAY_DELAY_VALUE[1] = 0;	  // STEP SIZES ... SPECIFY ACCORDING TO CONFIGURATION
	HW_RELAYS.RELAY_DELAY_VALUE[2] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[3] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[4] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[5] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[6] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[7] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[8] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[9] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[10] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[11] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[12] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[13] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[14] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[15] = 0;
	HW_RELAYS.RELAY_DELAY_VALUE[16] = 0;
	HW_RELAYS.NUM_OF_SECTIONS = 0;

	//---------------------------------------------------------------------------------------------------------------------//

	GLOBAL_SETTINGS.SUM_RELAYS_LESS_1 = 0; // MUST COMPUTE FOR CPDL and PDL-100A
	for (i = 1; i < HW_RELAYS.NUM_OF_SECTIONS; i++)
	{
		GLOBAL_SETTINGS.SUM_RELAYS_LESS_1 = GLOBAL_SETTINGS.SUM_RELAYS_LESS_1 + HW_RELAYS.RELAY_DELAY_VALUE[i];
	}

	INSTRUMENT.stateDEVICE_DISPLAY_NS = TRUE; // INITIAL DISPLAY IN NS (DEFAULT)

	//
	// GLOBAL_SETTINGS.LAST_SECTION_ODD = TRUE indicates that the last relay section delay
	// is LESS than the next to last relay delay because total delay
	// range does not fall on an even boundary.
	//

	GLOBAL_SETTINGS.LAST_RELAY_SECTION_ODD = FALSE;

	if (HW_RELAYS.NUM_OF_SECTIONS != 0)
	{ // don't need to count for TROMBONE ONLY PDL-100A
		if (HW_RELAYS.RELAY_DELAY_VALUE[HW_RELAYS.NUM_OF_SECTIONS] < HW_RELAYS.RELAY_DELAY_VALUE[HW_RELAYS.NUM_OF_SECTIONS - 1])
		{
			GLOBAL_SETTINGS.LAST_RELAY_SECTION_ODD = TRUE;
		}
	}

	for (i = 0; i <= 16; i++)
	{
		HW_RELAYS.RELAY_ON_OFF[i] = FALSE;
	} // clear out all hwRELAY bit settings
} // void Init_RELAYS(void) //

void Init_GPIB(void)
{

	//
	// GPIB consists of the TNT5002 IC and needs to be initialized before using
	// INITIALIZATION SECTION ...
	//

	HW_RELAYS.SWITCH_SETTINGS = 0;

	//
	// 11.02.16 need to setup the shift register before using read address
	// prior to getting the gpib address, need to initialize the hardware ONCE
	//
	shiftRegInputInitHW1VER2();
	HW_RELAYS.SWITCH_SETTINGS = pdlGetSwitchAddressHW1VER2();

	GLOBAL_SETTINGS.SET_OPTO_LOCN = 0.0;	  // to set voltage in opto detect DIAG menu option
	GLOBAL_SETTINGS.ENABLE_STEP_DIAG = FALSE; // flag to enable/disable single step movement in DIAG menu

#ifdef GPIB_ENABLED
	// GPIB ASIC
	Initialize_Interface();			// Initialize TNT5002 chip
	Set_Timeout(TMO_DEFAULT, TRUE); /* 1 ms is OPTIMAL */
	Set_Address_Mode(0);			/* Set to single primary mode     */
	DISPLAY_SETTINGS.CURRENT_GPIB_ADDR = HW_RELAYS.SWITCH_SETTINGS;
	Change_Primary_Address(DISPLAY_SETTINGS.CURRENT_GPIB_ADDR); /* Set primary address            */
#endif

	GPIB_SETTINGS.DEVICE_CLEAR = FALSE; // 12.05.06 added to signal device clear received
	GPIB_SETTINGS.INTERFACE_CLEAR = FALSE;
	GPIB_SETTINGS.DEVICE_TRIGGER = FALSE;

} // void Init_GPIB (void) //

void Init_Variables(void)
{

	//
	// STARTUP PROCESS
	//

	INSTRUMENT.statePARSE = FALSE;
	INSTRUMENT.stateERROR = FALSE;
	INSTRUMENT.stateERROR_CODE = 0;
	INSTRUMENT.stateSER_PORT_B_CHAR = FALSE;
	INSTRUMENT.stateSER_PORT_E_CHAR = FALSE;
	INSTRUMENT.stateSER_MOTOR_CHAR = FALSE;

	INSTRUMENT.stateGPIB_DATA_IN = FALSE;
	INSTRUMENT.stateGPIB_DATA_OUT = FALSE;

	INSTRUMENT.stateTCPIP_DATA_IN = FALSE;
	INSTRUMENT.stateTCPIP_DATA_OUT = FALSE;

	INSTRUMENT.stateCMD_FROM_GPIB = FALSE;
	INSTRUMENT.stateCMD_FROM_TERM = FALSE;
	INSTRUMENT.stateCMD_FROM_LAN = FALSE;
	INSTRUMENT.stateDEVICE_MODE = DEVICE_SERIAL; //default is SERIAL TROMBONE MODE
	INSTRUMENT.stateDEVICE_MODE_MT100A = TRUE;	 //default is MT-100A terminal mode
	INSTRUMENT.stateMENU_MODE = 0;				 // 0 = normal (not is menu mode)
	INSTRUMENT.stateMENU_MODE_LAST_RELAY = 0;	 // number of last relay specified in MenuMode

	INSTRUMENT.stateMOTOR_Response_VALUE = 0;
	INSTRUMENT.stateMOTOR_ResponseProcessed = FALSE;

	INSTRUMENT_SETTINGS.CURRENT_UNITS = PS; // set default current units (change for diff models?)
	INSTRUMENT_SETTINGS.CURRENT_STEP_SIZE = 0.0;
	INSTRUMENT_SETTINGS.CURRENT_DELAY = 0.0;

	INSTRUMENT_SETTINGS.CURRENT_DELAY_ONE_PS = 0.0; // current delay value in picoseconds for Channel One
	INSTRUMENT_SETTINGS.CURRENT_DELAY_TWO_PS = 0.0; // current delay value in picoseconds for Channel Two
	INSTRUMENT_SETTINGS.CURRENT_DELAY_ONE_F = 0.0;	// 03.28.18 added
	INSTRUMENT_SETTINGS.CURRENT_DELAY_TWO_F = 0.0;	// 03.28.18 added
	INSTRUMENT_SETTINGS.CURRENT_DELAY_ONE_E = 0.0;	// 03.28.18 added
	INSTRUMENT_SETTINGS.CURRENT_DELAY_TWO_E = 0.0;	// 03.28.18 added

	INSTRUMENT_SETTINGS.CURRENT_BAR_GRAPH = 0x00000000; // RD03 BARGRAPH // 02.05.21
	INSTRUMENT_SETTINGS.CURRENT_LAN_LED_STATUS = FALSE; // RD03 BARGRAPH // 02.05.21

	DISPLAY_SETTINGS.CURRENT_STEP_SIZE_E = 0.0; //for display purposes
	DISPLAY_SETTINGS.CURRENT_DELAY_E = 0.0;		//SETTINGS.CurrentDelay formatted for display (EXPONENT)
	DISPLAY_SETTINGS.CURRENT_DELAY_F = 0.0;		//SETTINGS.CurrentDelay formatted for display (FLOAT)
	DISPLAY_SETTINGS.CURRENT_GPIB_ADDR = 0;
	DISPLAY_SETTINGS.CURRENT_DELAY_ONE_F = 0.0; // current delay value as float for display in Command line
	DISPLAY_SETTINGS.CURRENT_DELAY_ONE_E = 0.0; // current delay (as a float var) for display
	DISPLAY_SETTINGS.CURRENT_DELAY_TWO_F = 0.0; // current delay value as float for display in Command line
	DISPLAY_SETTINGS.CURRENT_DELAY_TWO_E = 0.0; // current delay (as a float var) for display

	INSTRUMENT.deviceOPERATIONS = 0; // record # of operations since power on

	GLOBAL_SETTINGS.AUTO_SET_PLUS = FALSE;
	GLOBAL_SETTINGS.AUTO_SET_MINUS = FALSE;
	GPIB_SETTINGS.ESR_MASK = 0;		  // ESR MASK (set by using *ESE mask command
	GPIB_SETTINGS.SET_OPC_BIT = TRUE; // 04.02.07 make the bit TRUE on startup!! VER 1.50+
	GPIB_SETTINGS.ESR_register_bits = 0;

	strcpy(INSTRUMENT.deviceOPTION, deviceOPTION);

	GLOBAL_SETTINGS.REMOTE_LOCAL_MODE = LOCAL;

	INSTRUMENT.charSER_PORT_B_ONLY = 0;
	INSTRUMENT.charSER_PORT_E_ONLY = 0;

} // void Init_Variables(void) //

void Init_ETHERNET(void)
{

	// INITIALIZE the Ethernet chip if using TCPIP
	// IF ADDRESS SWITCHES ARE ALL ZEROS THEN USE DEFAULT PARAMETERS
	// ELSE GET NVRAM PARAMETERS AND USE SETTINGS STORED IN NV RAM

	HW_RELAYS.SWITCH_SETTINGS = 0;

	if ((strcmp(deviceOPTION, "OEM") == 0))
	{ //get from NVRAM settings
		LoadNVParameters_HW1VER2();
		HW_RELAYS.SWITCH_SETTINGS = g_NVParameters.nv_GPIB_addr;
		if ((HW_RELAYS.SWITCH_SETTINGS < 0) || (HW_RELAYS.SWITCH_SETTINGS > 31))
		{
			HW_RELAYS.SWITCH_SETTINGS = 0;
		}
	}
	else
	{
		HW_RELAYS.SWITCH_SETTINGS = pdlGetSwitchAddressHW1VER2();
	}

	if (HW_RELAYS.SWITCH_SETTINGS == 0)
	{
		// if hardware switch settings == 0; then RESET TO DEFAULT CONDITIONS
		// GPIB default address = 5, and IP addresses are defaulted
		// RESET TO DEFAULT CONDITION
		DISPLAY_SETTINGS.CURRENT_GPIB_ADDR = 5; //default GPIB address
		// get the parameters from NV storage
		LoadNVParameters_HW1VER2();
// overwrite them with new DEFAULT values
#ifdef PRIMARY_TROMBONE
		g_NVParameters.nv_ip_addr = aton("192.168.100.8");
#endif
#ifdef SECONDARY_TROMBONE
		g_NVParameters.nv_ip_addr = aton("192.168.100.9");
#endif
		g_NVParameters.nv_netmask = aton("255.255.0.0");
		g_NVParameters.nv_gateway = aton("192.168.100.1");
		g_NVParameters.nv_port = 1234;
		g_ENET_PORT = g_NVParameters.nv_port;
		g_NVParameters.nv_useDHCP = FALSE;
		g_NVParameters.nv_GPIB_addr = 5;
		g_NVParameters.nv_overshoot = TRUE;
		g_NVParameters.nv_overshoot_PS = 5;					  // 07.27.17 5 ps for default overshoot amount
		g_NVParameters.nv_autodrop = TRUE;					  // 03.16.15
		g_NVParameters.nv_terminal_mode = FALSE;			  //added 05.24.05
		g_NVParameters.nv_useCTSTORE = FALSE;				  // 02.07.18
		strcpy(g_NVParameters.nv_hostname, "COLBY_XXXXXXXX"); // 01.25.21
#ifdef WEB_SERVER
		strcpy(g_NVParameters.nv_password, "password");
		strcpy(g_NVParameters.nv_description, "PDL-100A Programmable Delay Line Instrument");
#endif
		// save all parameters to NV storage
		SaveNVParameters_HW1VER2();
		ENET_Init(FALSE, g_NVParameters.nv_ip_addr, g_NVParameters.nv_netmask, g_NVParameters.nv_gateway, g_NVParameters.nv_autodrop);
	} // end-if
	else
	{
		DISPLAY_SETTINGS.CURRENT_GPIB_ADDR = HW_RELAYS.SWITCH_SETTINGS;
		LoadNVParameters_HW1VER2(); // normal operation; load values from NVRAM

#ifndef DHCP_SEND_HOSTNAME
		if (g_NVParameters.nv_useDHCP == TRUE)
		{
			//DHCP = ON
			ENET_Init(TRUE, g_NVParameters.nv_ip_addr, g_NVParameters.nv_netmask, g_NVParameters.nv_gateway, g_NVParameters.nv_autodrop);
		}
		else
		{
			//DHCP = OFF
			ENET_Init(FALSE, g_NVParameters.nv_ip_addr, g_NVParameters.nv_netmask, g_NVParameters.nv_gateway, g_NVParameters.nv_autodrop);
		}
#endif

#ifdef DHCP_SEND_HOSTNAME

		// ON A NEW FLASH MODULE, THE COLBY_XXXXXXXX IS THE DEFAULT nv_hostname set in NVRAM.
		// REPLACE WITH THE CONSTANT STRING "COLBY_" + SN# (e.g. "21031234") THAT IS EDITED/CREATED FOR EACH INSTRUEMTN C SOURCE FILE
		//
		if (strcmp(g_NVParameters.nv_hostname, "COLBY_XXXXXXXX") == 0)
		{
			// MAKE THE DEFAULT TO BE g_StaMem_HostName (static memory COLBY_12345678) as defined at production
			strcpy(g_NVParameters.nv_hostname, g_StaMem_HostName);
			SaveNVParameters_HW1VER2();
			strcpy(g_DynMem_HostName, g_StaMem_HostName); // use Static Mem HostName as the Dynamic Mem HostName
		}
		else
		{
			// g_DynMem_HostName is in dynamic working memory
			strcpy(g_DynMem_HostName, g_NVParameters.nv_hostname); // get the hostname from NV memory
		}

		g_RetCode = sethostname(g_DynMem_HostName); // 01.24.21

		if (g_NVParameters.nv_useDHCP == TRUE)
		{
			//DHCP = ON
			ENET_Init(TRUE, g_NVParameters.nv_ip_addr, g_NVParameters.nv_netmask, g_NVParameters.nv_gateway, g_NVParameters.nv_autodrop);
		}
		else
		{
			//DHCP = OFF
			ENET_Init(FALSE, g_NVParameters.nv_ip_addr, g_NVParameters.nv_netmask, g_NVParameters.nv_gateway, g_NVParameters.nv_autodrop);
		}

		g_RetCode = sethostname(g_DynMem_HostName); // 01.24.21
#endif

	} //end else-if

} // void Init_ETHERNET (void) //

void Init_INSTRUMENT_Variables(void)
{

	// check to see whether TERMINAL_MODE was set TRUE in NV storage or not
	if (g_NVParameters.nv_terminal_mode == TRUE)
	{
		INSTRUMENT.stateDEVICE_MODE_MT100A = FALSE;
	}
	else
	{
		INSTRUMENT.stateDEVICE_MODE_MT100A = TRUE;
	}

	if (g_NVParameters.nv_overshoot == TRUE)
	{
		GLOBAL_SETTINGS.userOVERSHOOT = TRUE; //04.18.07
	}
	else
	{
		// 02.28.07 AGILENT FIX FOR USE WITH PHASE NOISE ANALYZER
		GLOBAL_SETTINGS.userOVERSHOOT = FALSE; //04.18.07
	}

	if (g_NVParameters.nv_useCTSTORE == TRUE)
	{
		GLOBAL_SETTINGS.USE_CAL_TABLE = TRUE; // 02.07.18
	}
	else
	{
		GLOBAL_SETTINGS.USE_CAL_TABLE = FALSE; // 02.07.18
	}

	// 04.10.18 Handle NS_PS CYCLE MODE on startup
	INSTRUMENT.stateCYCLE_MODE = g_NVParameters.nv_nsps_cycle_mode;

	if ((g_NVParameters.nv_nsps_cycle_mode <= 0) || (g_NVParameters.nv_nsps_cycle_mode > CYCLE_CHANNEL))
	{
		INSTRUMENT.stateCYCLE_MODE = CYCLE_UNIT; //default
		g_NVParameters.nv_nsps_cycle_mode = CYCLE_UNIT;
		SaveNVParameters_HW1VER2();
	}

	//***********************************************************************************************************************
	// MUST DEFINE FOR EACH DEVICE MANUFACTURED
	//
	// THIS VALUE MUST BE SET FOR EACH DEVICE TYPE.  THIS IS THE MAX DELAY IN PS
	//---------------------------------------------------------------------------------------------------------------------//
	// STEP#3

#ifdef DEVICE_PDL100A_312PT5
	PARAMETERS.deviceMAX_DELAY_PS = (float)312.5;  // MAX DELAY IN NUMBER OF PICOSECONDS, E.G. "XXXXX.XX"
												   // 625.00 for PDL-100A-000 TROMBONE ONLY
												   // 10000.00 for PDL-100A-010 for 10 NS Version
	PARAMETERS.deviceMAX_DELAY_NS = (float)0.3125; // for STARTUP DISPLAY PURPOSES, MAX DELAY IN NUMBER OF NANOSECONDS
												   // 0.312 for PDL-100A-312PS HALF TROMBONE
												   // 0.625 for PDL-100A-625PS TROMBONE ONLY
												   // 10.00 for PDL-100A-10NS
												   // 20.00 for PDL-100A-20NS
#endif

#ifdef DEVICE_PDL100A
	PARAMETERS.deviceMAX_DELAY_PS = (float)625.0; // MAX DELAY IN NUMBER OF PICOSECONDS, E.G. "XXXXX.XX"
												  // 625.00 for PDL-100A-000 TROMBONE ONLY
												  // 10000.00 for PDL-100A-010 for 10 NS Version
	PARAMETERS.deviceMAX_DELAY_NS = (float)0.625; //  for STARTUP DISPLAY PURPOSES, MAX DELAY IN NUMBER OF NANOSECONDS
												  // 0.312 for PDL-100A-312PS HALF TROMBONE
												  // 0.625 for PDL-100A-625PS TROMBONE ONLY
												  // 10.00 for PDL-100A-10NS
												  // 20.00 for PDL-100A-20NS
#endif

// 03.28.18 for PDL-200A
#ifdef DEVICE_PDL200A
	PARAMETERS.deviceMAX_DELAY_PS = (float)625.0; // MAX DELAY IN NUMBER OF PICOSECONDS, E.G. "XXXXX.XX"
	PARAMETERS.deviceMAX_DELAY_NS = (float)0.625; // for STARTUP DISPLAY PURPOSES, MAX DELAY IN NUMBER OF NANOSECONDS
#endif

	// MUST DEFINE FOR CPDL-100A UNITS	-- CPDL-100A -- SPECIFY HERE -- for EACH instrument
#ifdef DEVICE_CPDL100A
	PARAMETERS.deviceMAX_DELAY_PS = (float)50000.0; // MAX DELAY IN NUMBER OF PICOSECONDS, E.G. "XXXXX.XX"
	PARAMETERS.deviceMAX_DELAY_NS = (float)50.0;	// for STARTUP DISPLAY PURPOSES, MAX DELAY IN NUMBER OF NANOSECONDS
#endif

	PARAMETERS.deviceSTEP_SIZE_NS = (float)0.001; // for startup display of step size and range in CPDL model
												  // NOT USED in PDL-100A
												  //---------------------------------------------------------------------------------------------------------------------//

	// determine the INSTRUMENT display setting for NS OR PS
	INSTRUMENT.stateDEVICE_DISPLAY_NS = TRUE; // DEFAULT FOR CPDL-100A MODELS IS NS DISPLAY TRUE

	if ((PARAMETERS.deviceMAX_DELAY_NS == 0.625) || (PARAMETERS.deviceMAX_DELAY_NS == 0.312) || (PARAMETERS.deviceMAX_DELAY_NS == 0.3125)) // EXCEPT FOR TROMBONE ONLY
		INSTRUMENT.stateDEVICE_DISPLAY_NS = FALSE;																						   // DEFAULT IS PS DISPLAY TRUE

	// clear out the cmdBUFFER which holds the command to process
	memset(cmdBUFFER, 0x00, sizeof(cmdBUFFER)); // clear out buffer
	cmdINDEX = 0;								// cmdINDEX is the index into the cmdBUFFER

	GLOBAL_SETTINGS.COMMAND_CONTINUE = FALSE; // indicate if there are addtional commands to be processed
	GPIB_SETTINGS.TACS_WAIT = DEFAULT_TACS_WAIT;

	// PAUSE AT STARTUP TO ALLOW MT-100A MICROTERMINAL TO POWER UP BEFORE SENDING
	// STARTUP TEXT MESSAGE
	// instead of busy wait loop ... use

	msDelay(250);

	prtTERM("\n$");		  // clear the display
	prtTERM(deviceMODEL); // display the INSTRUMENT Model
	// determine the # of digits to display on TERMINAL STARTUP
	if ((PARAMETERS.deviceMAX_DELAY_NS == 0.625) || (PARAMETERS.deviceMAX_DELAY_NS == 0.312) || (PARAMETERS.deviceMAX_DELAY_NS == 0.3125))
	{
		sprintf(BUFFERS.DISPLAY_LINE, "%6.3fNS", PARAMETERS.deviceMAX_DELAY_NS); // digits of prec to display 3
	}
	else
	{
		sprintf(BUFFERS.DISPLAY_LINE, "%6.2fNS", PARAMETERS.deviceMAX_DELAY_NS); // digits of prec to display 2
	}
	prtTERM(BUFFERS.DISPLAY_LINE);
	prtTERM("\tCOLBYINSTRUMENTS");

	/////////////////////

	prtTERM("\r"); // overwrite the Initializing ... prompt

	GLOBAL_SETTINGS.ELAPSED_DELAY_TIME = 0; // added for DTIME? 10.04.06
	GLOBAL_SETTINGS.START_DELAY_TIME = 0;
	GLOBAL_SETTINGS.END_DELAY_TIME = 0;
} // void Init_INSTRUMENT_Variables (void) //

void Init_MainFromHW1VER1(void)
{

	unsigned long _BarGraphDisplay;
	_BarGraphDisplay = 0x00000000;

	Init_Variables();

	// get the hardware switch address then bring up the GPIB I/O
	Init_GPIB();

	shiftRegBarGraphInitHW1VER2();

	// 02.05.21
	INSTRUMENT_SETTINGS.CURRENT_BAR_GRAPH = 0x00000000; // ADDED RD03 02.05.2021 FOR BARGRAPH
	INSTRUMENT_SETTINGS.CURRENT_LAN_LED_STATUS = FALSE; // ADDED RD03 02.05.2021
														//

	INSTRUMENT_SETTINGS.CURRENT_BAR_GRAPH = 0x00000000;
	shiftRegBarGraphOutputHW1VER2(INSTRUMENT_SETTINGS.CURRENT_BAR_GRAPH);

	INSTRUMENT_SETTINGS.CURRENT_BAR_GRAPH = 0x000FFFFF;
	shiftRegBarGraphOutputHW1VER2(INSTRUMENT_SETTINGS.CURRENT_BAR_GRAPH);

	INSTRUMENT_SETTINGS.CURRENT_BAR_GRAPH = 0x00000000;
	shiftRegBarGraphOutputHW1VER2(INSTRUMENT_SETTINGS.CURRENT_BAR_GRAPH);

	// initialize serial ports
	Init_SERIALPORTS();

	// Start up the hardware relays if attached
	Init_RELAYS();

	msDelay(2000); // need to wait at least 2 seconds to wait for new motor to startup

// 04.19.18 add for DEVICE_CPDL100A - disable any MOTOR related code
#ifndef DEVICE_CPDL100A
	// Initialize Motor through RS 485 serial port
	Init_MOTOR_CS();
	MOTOR.CurrentDelaySettingPS = 0; // 02.05.18 -- ensure starting point is at ZERO
#endif

	// Start up the Ethernet Connection
	Init_ETHERNET();

	// Initialize all Instrument Variables and Instrument Settings
	Init_INSTRUMENT_Variables();

    #ifdef NO_GPIB_XT_200_HW1V2
    DATA_COUNT = 0;
    #endif

} // void Init_MainFromHW1VER1(void)

void MainWhileLoopHW1VER1(void)
{

	//
	// Main While Loop Code from Version 1.82 PDL-100A HW1 VER 1 production code:
	//

	while (1)
	{ // MAIN() program while loop

		////////////////////////////////////////////////////////////////////////////////

#ifdef ETHERNET_ENABLED
		ENET_Handler(); // service the Ethernet
#endif

#ifdef ENABLE_WEB_SERVER
		http_handler(); // HTTP server // RD01 added 02.06.21 WEB_SERVER
#endif

		// Serial Port B ALTERNATE
		// if ((ch = serBgetc()) != -1)     // any chars available from MicroTerminal?
		// stateSER_PORT_CHAR = TRUE;       // check Serial Port B

#ifdef SECONDARY_TROMBONE
		// if this is the secondary trombone, then must listen via SERIAL PORT B
		// Serial Port B ALTERNATE
		if ((charSER_PORT = serBgetc()) != -1)
		{ // any chars available from PRIMARY TROMBONE?
			if (charSER_PORT != 0)
			{
				INSTRUMENT.stateSER_PORT_B_CHAR = TRUE; // check Serial Port B
				INSTRUMENT.charSER_PORT_B_ONLY = charSER_PORT;
			}
		}
#endif

// 04.19.18 add for DEVICE_CPDL100A - disable any MOTOR related code
#ifndef DEVICE_CPDL100A
		// Serial Port C MOTOR
		if ((charSER_MOTOR = serCgetc()) != -1)	   // any chars from MOTOR ?
			INSTRUMENT.stateSER_MOTOR_CHAR = TRUE; // check Serial Port C
#endif

		// Serial Port E MT-100A
		if ((charSER_PORT = serEgetc()) != -1)
		{											// any chars available from MicroTerminal?
			INSTRUMENT.stateSER_PORT_E_CHAR = TRUE; // check Serial Port E
			INSTRUMENT.charSER_PORT_E_ONLY = charSER_PORT;
		}

        // RD030421 REMOVE ANY GPIB CODE 04.06.21
        // Service_GPIB_HW_IO(); // handle any GPIB related HW I/O


        ///////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////

        // RD030421 REMOVE GPIB CODE 04.10.21
        #ifndef NO_GPIB_XT_200_HW1V2
		costate
		{
			waitfor(INSTRUMENT.stateGPIB_DATA_IN);
			INSTRUMENT.stateGPIB_DATA_IN = FALSE;					   // reset the flag
			memset(cmdCOMMAND, 0x00, sizeof(cmdCOMMAND));			   // clear out buffer
			strcpy(BUFFERS.INPUT_COMMAND_LINE, BUFFERS.gpibBUFFER_IN); //copy into global buffer
			extractONE_COMMAND();									   // extract 1 command into cmdCOMMAND[]
			if (strlen(cmdCOMMAND) != 0)							   // see if it is command that needs parsing
				INSTRUMENT.statePARSE = TRUE;						   // set this flag to TRUE to trigger action!
		}															   // end costate stateGPIB_DATA_IN;
        #endif

		costate
		{
			waitfor(INSTRUMENT.stateTCPIP_DATA_IN);
			INSTRUMENT.stateTCPIP_DATA_IN = FALSE;		  // reset the flag
			memset(cmdCOMMAND, 0x00, sizeof(cmdCOMMAND)); // clear out buffer
			if (strlen(BUFFERS.tcpipBUFFER_IN) > INPUT_BUFFER_SIZE_MAX)
			{
				// potential OVERFLOW PROBLEM ... too many characters received over ETHERNET
				// therefore cut it off by making it an end of string character at the end of it
				BUFFERS.tcpipBUFFER_IN[INPUT_BUFFER_SIZE_MAX - 1] = 0x00; // NULL CHAR
			}
			strcpy(BUFFERS.INPUT_COMMAND_LINE, BUFFERS.tcpipBUFFER_IN); // copy into global buffer
			GLOBAL_SETTINGS.tcpipSession = g_ENETactiveSession;			// record the current session
			extractONE_COMMAND();
			if (strlen(cmdCOMMAND) != 0)		 // see if command that needs parsing
				INSTRUMENT.statePARSE = TRUE;	 // set this flag to TRUE to trigger action!
			INSTRUMENT.stateCMD_FROM_LAN = TRUE; // indicate the source of the command was from LAN
		}										 // end costate stateTCPIP_DATA_IN

		costate
		{
			// statePARSE triggered when data in is cmdCOMMAND[] to be parsed
			waitfor(INSTRUMENT.statePARSE); // statePARSE == TRUE to trigger
			INSTRUMENT.statePARSE = FALSE;	// reset the flag
			// use cmdParse to parse cmdCOMMAND[] into 3 components cmdARG1,2,3
			if (strlen(cmdCOMMAND) != 0)
			{
				cmdParse(cmdCOMMAND);
			}

			//SPECIAL CASE -- ESCAPE KEY CLEAR HIT?
			if (strcmp(cmdARG1, "\x1B") == 0)
			{
				INSTRUMENT.stateMENU_MODE = 0;
				INSTRUMENT.stateERROR_CODE = NO_ERROR;
				sprintf(cmdARG1, "");
			}
			//SCAN to see if ESC char is in cmdARG1
			keyESC = FALSE;
			for (k = 0; k <= strlen(cmdARG1); k++)
			{
				if (cmdARG1[k] == '\x1B')
				{
					keyESC = TRUE;
					break;
				}
			} // end for
			if (keyESC == TRUE)
			{
				INSTRUMENT.stateMENU_MODE = 0;
				INSTRUMENT.stateERROR_CODE = NO_ERROR;
				sprintf(cmdARG1, "");
			}

			//see if entry is during reading of network address setting
			if (INSTRUMENT.stateMENU_MODE != 0)
			{
				//a network address was entered into cmdARG1, so call handleMENU_MODE directly
				handleMENU_MODE();
			}
			else
			{
				// Command is parsed into cmdARG1,cmdARG2,cmdARG3.  Decode and process.
				EXECUTE_COMMAND_HW1_VER2();
			} //end else-if

			//
			// POST-PROCESSING ... Command has been completed
			// cmdCOMMAND[] was processed so issue a prompt only if previous command was from TERMINAL
			// and the Microterminal is NOT in MENU MODE.  Check also to see if there
			// are multiple commands so those need to be processed too.
			//
			// if the COMMAND[] came from the MT-100A, then only ONE command is possible to have
			// been processed.  Issue a command prompt if COMMAND[] came from MT-100A.
			//

			if ((INSTRUMENT.stateCMD_FROM_TERM) && (INSTRUMENT.stateMENU_MODE == 0))
			{
				INSTRUMENT.stateCMD_FROM_TERM = FALSE;
				INSTRUMENT.stateCMD_FROM_GPIB = FALSE;
				memset(cmdARG1, 0x00, sizeof(cmdARG1)); // clear out this buffer for next time
				memset(cmdARG2, 0x00, sizeof(cmdARG2));
				memset(cmdARG3, 0x00, sizeof(cmdARG3));
			} // end if

			//
			// Don't reset the stateCMD_FROM_GPIB until LAST command is finished
			//

			if ((INSTRUMENT.stateCMD_FROM_GPIB) && (!GLOBAL_SETTINGS.COMMAND_CONTINUE))
			{
				//11.09.06 BUG FIX - handle GPIB multiple commands in queue */
				//11.09.06 ONLY SET THIS GPIB STATE TO FALSE IF THERE ARE NO OTHER GPIB COMMANDS
				//11.09.06 OTHERWISE CODE WILL MISS PROCESSING THIS COMMMAND AND INSTRUMENT WILL TIMEOUT
				if (!INSTRUMENT.stateGPIB_DATA_IN)
				{
					INSTRUMENT.stateCMD_FROM_GPIB = FALSE;
				}
				//11.09.06 BUG FIX - END
				// remove this original line because always assumed GPIB command is finished processing
				// before the next one is received and this is incorrect.
				//          stateCMD_FROM_GPIB = FALSE;
				INSTRUMENT.stateCMD_FROM_TERM = FALSE;
				sprintf(BUFFERS.DISPLAY_LINE, "\r\n");
				prtTERM(BUFFERS.DISPLAY_LINE);
			} // end if

			// added 02.21.07 don't reset the stateCMD_FROM_LAN until LAST command is finished
			if ((INSTRUMENT.stateCMD_FROM_LAN) && (!GLOBAL_SETTINGS.COMMAND_CONTINUE))
			{
				INSTRUMENT.stateCMD_FROM_LAN = FALSE;
				INSTRUMENT.stateCMD_FROM_TERM = FALSE;
				sprintf(BUFFERS.DISPLAY_LINE, "\r\n");
				prtTERM(BUFFERS.DISPLAY_LINE);
			}

			//
			// display a prompt on the MT-100A as long as not in MENU_MODE
			//

			if (INSTRUMENT.stateMENU_MODE == 0)
			{
				cmdPrompt();
			}

			//
			// if there are more commands to process, extract the next one and
			// continue by indicating there is another command to Parse

			if (GLOBAL_SETTINGS.COMMAND_CONTINUE)
			{
				extractONE_COMMAND();		  // get next command into cmdCOMMAND
				INSTRUMENT.statePARSE = TRUE; // indicate there are more commands
											  // to follow because of ;
			}								  // end-if
		}									  // end of costate

		costate
		{
			waitfor(INSTRUMENT.stateTCPIP_DATA_OUT);
			INSTRUMENT.stateTCPIP_DATA_OUT = FALSE;															   // reset the flag
			ENET_Send(GLOBAL_SETTINGS.tcpipSession, BUFFERS.tcpipBUFFER_OUT, strlen(BUFFERS.tcpipBUFFER_OUT)); // send
		}																									   // end costate stateTCPIP_DATA_IN

		costate
		{
			waitfor(INSTRUMENT.stateERROR);
			INSTRUMENT.stateERROR = FALSE; // toggle stateERROR

#ifndef NO_GPIB_XT_200_HW1V2
			// set ESR register bits depending on the ERROR_CODE
			UpdateGPIB_ESR_Register();
#endif

			if (INSTRUMENT.stateERROR_CODE == BUFFER_OVERFLOW)
				cmdPrompt(); // send a new COMMAND prompt out
		}					 // end costate stateERROR;

#ifdef SECONDARY_TROMBONE
		costate
		{
			waitfor(INSTRUMENT.stateSER_PORT_B_CHAR); // char in serial port B from PRIMARY TROMBONE
			INSTRUMENT.stateSER_PORT_B_CHAR = FALSE;  // toggle stateSER_PORT_CHAR;
			Service_SERIAL_PORT_B_CHAR();
		} // end costate stateSER_PORT_CHAR
#endif

		costate
		{
			waitfor(INSTRUMENT.stateSER_PORT_E_CHAR); // char in serial port E from MT-100A
			INSTRUMENT.stateSER_PORT_E_CHAR = FALSE;  // toggle stateSER_PORT_CHAR;
			Service_SERIAL_PORT_E_CHAR();
		} // end costate stateSER_PORT_CHAR

		// 04.19.18 add for DEVICE_CPDL100A - disable any MOTOR related code
#ifndef DEVICE_CPDL100A
		costate
		{
			waitfor(INSTRUMENT.stateSER_MOTOR_CHAR); // char in serial port from MOTOR
			INSTRUMENT.stateSER_MOTOR_CHAR = FALSE;	 // toggle stateSER_MOTOR_CHAR;
			//
			// for each char from the MOTOR, add into MOTOR_Response until a \r
			// is received to signal to process the entire response line
			if (charSER_MOTOR != 0x0D)
			{
				// build the response line string by adding each character
				// strcat(MOTOR_Response,charSER_MOTOR);
				strCharFromSerialPortC[0] = charSER_MOTOR;
				strCharFromSerialPortC[1] = 0; // null
				strcat(MOTOR.RESPONSE_Text, strCharFromSerialPortC);
			}
			else
			{
				// 0x0D terminates a line of response from the Motor
				// decode the response line into corresponding values
				ProcessMOTOR_Response(MOTOR.RESPONSE_Text);
				memset(MOTOR.RESPONSE_Text, 0x00, sizeof(MOTOR.RESPONSE_Text)); // clear out buffer
			}
		} // end costate stateSER_MOTOR_CHAR
#endif

	} // end MAIN() while (1)

} // void MainWhileLoopHW1VER1(void) //

//////////////////////////////////////////////////////////////////////////////
////////////////////////////    MAIN    //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void main(void)
{

	//
	// This is the main entry point for this program.
	//
	// SHOULD PRE INITIALIZE ALL GLOBAL VARIABLES //
	//

	i = 0;
	j = 0;
	k = 0;
	charSER_PORT = 0;
	keyESC = 0;
	wait = 0;
	wait_loops = 0;

	for (i = 0; i < 150; i++)
	{
		buf[i] = 0;
	} // used as temp buffer for receive data over GPIB

	for (i = 0; i < INPUT_BUFFER_SIZE_MAX; i++)
	{
		BUFFERS.DISPLAY_LINE[i] = 0;
	}
	for (i = 0; i < INPUT_BUFFER_SIZE_MAX; i++)
	{
		BUFFERS.COMMAND_LINE[i] = 0;
	}
	for (i = 0; i < 500; i++)
	{
		BUFFERS.gpibBUFFER_IN[i] = 0;
	}
	for (i = 0; i < 500; i++)
	{
		BUFFERS.gpibBUFFER_OUT[i] = 0;
	}
	for (i = 0; i < 500; i++)
	{
		BUFFERS.tcpipBUFFER_IN[i] = 0;
	}
	for (i = 0; i < 500; i++)
	{
		BUFFERS.tcpipBUFFER_OUT[i] = 0;
	}
	for (i = 0; i < INPUT_BUFFER_SIZE_MAX; i++)
	{
		BUFFERS.INPUT_COMMAND_LINE[i] = 0;
	}

	for (i = 0; i < sizeof(MOTOR.RESPONSE_Text); i++)
	{
		MOTOR.RESPONSE_Text[i] = 0;
	}

	MOTOR.CurrentStepPosition = 0;
	// END PRE-INITIALIZATION GLOBAL VARIABLES

	IOPreInit();
	Init_MainFromHW1VER1(); // from original production code

	cmdRST(1);	 // MUST DO THIS TO HAVE ETHERNET PORTS COME UP CORRECTLY
	cmdPrompt(); // send out FIRST COMMAND: prompt

#ifdef ENABLE_WEB_SERVER
	Web_Init(); // RD01 - initialize web varialbles // RD01 added 02.06.21 WEB_SERVER
#endif

	///////////////////////////////////////////////////////////////////////////////
	while (1)
	{ // MAIN() program while loop
		MainWhileLoopHW1VER1();
	} // end MAIN() while (1)
	  ////////////////////////////////////////////////////////////////////////////////

} //end main