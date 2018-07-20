#ifndef __TOUCHSCREEN_FW__
#define __TOUCHSCREEN_FW__

#if defined(CONFIG_BOARD_GIFT)
#define SYN_SUCCESS_FW_NAME	"ZTE_Z936L_Success_DS3.6.0_S7020_PR1484324_36333033.img"
#elif defined(CONFIG_BOARD_LEO)
#define SYN_SUCCESS_FW_NAME		"ZTE_P816T55_Success_DS3.6.0_S7020_PR1484324_36333038.img"
#define SYN_ECW_FW_NAME		"ZTE_P816T55_ECW_DS3.6.0_S7020_PR1484324_36393034.img"
#elif defined(CONFIG_BOARD_LOL)
#define SYN_SUCCESS_FW_NAME		"ZTE_P816T55_Success_DS3.6.0_S7020_PR1484324_36333038.img"
#define SYN_ECW_FW_NAME		"ZTE_P816T55_ECW_DS3.6.0_S7020_PR1484324_36393034.img"
#elif defined(CONFIG_BOARD_WARP6)
#define SYN_SUCCESS_FW_NAME		"ZTE_N9518-SUCCESS-PR1484324-s7020_36333035.img"
#define SYN_ECW_FW_NAME		"ZTE_N9518-ECW-PR1484324-s7020_36393032.img"
#elif defined(CONFIG_BOARD_TOM) || defined(CONFIG_BOARD_NANCY) || defined(CONFIG_BOARD_LILY)
#define SYN_SUCCESS_FW_NAME		"ZTE_P895A16_Success_DS4_3.6.4_PR2030544_36333032.img"
#else

#define SYN_TPK_FW_NAME		""
#define SYN_TURLY_FW_NAME	""
#define SYN_SUCCESS_FW_NAME	""
#define SYN_OFILM_FW_NAME	""
#define SYN_LEAD_FW_NAME	""
#define SYN_WINTEK_FW_NAME	""
#define SYN_LAIBAO_FW_NAME	""
#define SYN_CMI_FW_NAME		""
#define SYN_ECW_FW_NAME		""
#define SYN_GOWORLD_FW_NAME	""
#define SYN_BAOMING_FW_NAME	""
#define SYN_JUNDA_FW_NAME	""
#define SYN_JIAGUAN_FW_NAME	""
#define SYN_MUDONG_FW_NAME	""
#define SYN_EACHOPTO_FW_NAME	""
#define SYN_AVC_FW_NAME ""
#define SYN_SAMSUNG_FW_NAME  ""
#define SYN_JDI_FW_NAME  ""
#define SYN_DIJING_FW_NAME  ""
#define SYN_LCE_FW_NAME	""
#define SYN_BOE_FW_NAME	""
#define SYN_TIANMA_FW_NAME	""

#define FTC_TPK_FW_NAME		""
#define FTC_TURLY_FW_NAME	""
#define FTC_SUCCESS_FW_NAME	""
#define FTC_OFILM_FW_NAME	""
#define FTC_LEAD_FW_NAME	""
#define FTC_WINTEK_FW_NAME	""
#define FTC_LAIBAO_FW_NAME	""
#define FTC_CMI_FW_NAME		""
#define FTC_ECW_FW_NAME		""
#define FTC_GOWORLD_FW_NAME	""
#define FTC_BAOMING_FW_NAME	""
#define FTC_JUNDA_FW_NAME	""
#define FTC_JIAGUAN_FW_NAME	""
#define FTC_MUDONG_FW_NAME	""
#define FTC_EACHOPTO_FW_NAME	""
#define FTC_AVC_FW_NAME	""
#define FTC_LIANCHUANG_FW_NAME	""

#define GTP_SENSOR_ID_1_FW_NAME		""
#define GTP_SENSOR_ID_2_FW_NAME		""
#define GTP_SENSOR_ID_3_FW_NAME		""
#define GTP_SENSOR_ID_4_FW_NAME		""
#define GTP_SENSOR_ID_5_FW_NAME		""
#define GTP_SENSOR_ID_6_FW_NAME		""

#define CY_TPK_FW_NAME		""
#define CY_TURLY_FW_NAME	""
#define CY_SUCCESS_FW_NAME	""
#define CY_OFILM_FW_NAME	""
#define CY_LEAD_FW_NAME	""
#define CY_WINTEK_FW_NAME	""
#define CY_LAIBAO_FW_NAME	""
#define CY_CMI_FW_NAME		""
#define CY_ECW_FW_NAME		""
#define CY_GOWORLD_FW_NAME	""
#define CY_BAOMING_FW_NAME	""
#define CY_JUNDA_FW_NAME	""
#define CY_JIAGUAN_FW_NAME	""
#define CY_MUDONG_FW_NAME	""
#define CY_EACHOPTO_FW_NAME	""

#endif

#ifndef SYN_TPK_FW_NAME
#define SYN_TPK_FW_NAME		""
#endif
#ifndef SYN_TURLY_FW_NAME
#define SYN_TURLY_FW_NAME	""
#endif
#ifndef SYN_SUCCESS_FW_NAME
#define SYN_SUCCESS_FW_NAME	""
#endif
#ifndef SYN_OFILM_FW_NAME
#define SYN_OFILM_FW_NAME	""
#endif
#ifndef SYN_LEAD_FW_NAME
#define SYN_LEAD_FW_NAME	""
#endif
#ifndef SYN_WINTEK_FW_NAME
#define SYN_WINTEK_FW_NAME	""
#endif
#ifndef SYN_LAIBAO_FW_NAME
#define SYN_LAIBAO_FW_NAME	""
#endif
#ifndef SYN_CMI_FW_NAME
#define SYN_CMI_FW_NAME		""
#endif
#ifndef SYN_ECW_FW_NAME
#define SYN_ECW_FW_NAME		""
#endif
#ifndef SYN_GOWORLD_FW_NAME
#define SYN_GOWORLD_FW_NAME	""
#endif
#ifndef SYN_BAOMING_FW_NAME
#define SYN_BAOMING_FW_NAME	""
#endif
#ifndef SYN_EACHOPTO_FW_NAME
#define SYN_EACHOPTO_FW_NAME	""
#endif
#ifndef SYN_MUTTO_FW_NAME
#define SYN_MUTTO_FW_NAME	""
#endif
#ifndef SYN_JUNDA_FW_NAME
#define SYN_JUNDA_FW_NAME	""
#endif
#ifndef SYN_JIAGUAN_FW_NAME
#define SYN_JIAGUAN_FW_NAME	""
#endif
#ifndef SYN_MUDONG_FW_NAME
#define SYN_MUDONG_FW_NAME	""
#endif
#ifndef SYN_EACHOPTO_FW_NAME
#define SYN_EACHOPTO_FW_NAME	""
#endif
#ifndef SYN_AVC_FW_NAME
#define SYN_AVC_FW_NAME	""
#endif
#ifndef SYN_SAMSUNG_FW_NAME
#define SYN_SAMSUNG_FW_NAME  ""
#endif
#ifndef SYN_JDI_FW_NAME
#define SYN_JDI_FW_NAME  ""
#endif
#ifndef SYN_BOE_FW_NAME
#define SYN_BOE_FW_NAME	""
#endif
#ifndef SYN_TIANMA_FW_NAME
#define SYN_TIANMA_FW_NAME	""
#endif
#ifndef SYN_DIJING_FW_NAME
#define SYN_DIJING_FW_NAME  ""
#endif
#ifndef SYN_LCE_FW_NAME
#define SYN_LCE_FW_NAME	""
#endif


#ifndef FTC_TPK_FW_NAME
#define FTC_TPK_FW_NAME		""
#endif
#ifndef FTC_TURLY_FW_NAME
#define FTC_TURLY_FW_NAME	""
#endif
#ifndef FTC_SUCCESS_FW_NAME
#define FTC_SUCCESS_FW_NAME	""
#endif
#ifndef FTC_OFILM_FW_NAME
#define FTC_OFILM_FW_NAME	""
#endif
#ifndef FTC_LEAD_FW_NAME
#define FTC_LEAD_FW_NAME	""
#endif
#ifndef FTC_WINTEK_FW_NAME
#define FTC_WINTEK_FW_NAME	""
#endif
#ifndef FTC_LAIBAO_FW_NAME
#define FTC_LAIBAO_FW_NAME	""
#endif
#ifndef FTC_CMI_FW_NAME
#define FTC_CMI_FW_NAME		""
#endif
#ifndef FTC_ECW_FW_NAME
#define FTC_ECW_FW_NAME		""
#endif
#ifndef FTC_GOWORLD_FW_NAME
#define FTC_GOWORLD_FW_NAME	""
#endif
#ifndef FTC_BAOMING_FW_NAME
#define FTC_BAOMING_FW_NAME	""
#endif
#ifndef FTC_JUNDA_FW_NAME
#define FTC_JUNDA_FW_NAME	""
#endif
#ifndef FTC_JIAGUAN_FW_NAME
#define FTC_JIAGUAN_FW_NAME	""
#endif
#ifndef FTC_MUDONG_FW_NAME
#define FTC_MUDONG_FW_NAME	""
#endif
#ifndef FTC_EACHOPTO_FW_NAME
#define FTC_EACHOPTO_FW_NAME	""
#endif

#ifndef FTC_AVC_FW_NAME
#define FTC_AVC_FW_NAME	""
#endif

#ifndef FTC_LIANCHUANG_FW_NAME
#define FTC_LIANCHUANG_FW_NAME	""
#endif


#ifndef GTP_SENSOR_ID_1_FW_NAME
#define GTP_SENSOR_ID_1_FW_NAME		""
#endif
#ifndef GTP_SENSOR_ID_2_FW_NAME
#define GTP_SENSOR_ID_2_FW_NAME		""
#endif
#ifndef GTP_SENSOR_ID_3_FW_NAME
#define GTP_SENSOR_ID_3_FW_NAME		""
#endif
#ifndef GTP_SENSOR_ID_4_FW_NAME
#define GTP_SENSOR_ID_4_FW_NAME		""
#endif
#ifndef GTP_SENSOR_ID_5_FW_NAME
#define GTP_SENSOR_ID_5_FW_NAME		""
#endif
#ifndef GTP_SENSOR_ID_6_FW_NAME
#define GTP_SENSOR_ID_6_FW_NAME		""
#endif

#ifndef CY_TPK_FW_NAME
#define CY_TPK_FW_NAME		""
#endif
#ifndef CY_TURLY_FW_NAME
#define CY_TURLY_FW_NAME	""
#endif
#ifndef CY_SUCCESS_FW_NAME
#define CY_SUCCESS_FW_NAME	""
#endif
#ifndef CY_OFILM_FW_NAME
#define CY_OFILM_FW_NAME	""
#endif
#ifndef CY_LEAD_FW_NAME
#define CY_LEAD_FW_NAME	""
#endif
#ifndef CY_WINTEK_FW_NAME
#define CY_WINTEK_FW_NAME	""
#endif
#ifndef CY_LAIBAO_FW_NAME
#define CY_LAIBAO_FW_NAME	""
#endif
#ifndef CY_CMI_FW_NAME
#define CY_CMI_FW_NAME		""
#endif
#ifndef CY_ECW_FW_NAME
#define CY_ECW_FW_NAME		""
#endif
#ifndef CY_GOWORLD_FW_NAME
#define CY_GOWORLD_FW_NAME	""
#endif
#ifndef CY_BAOMING_FW_NAME
#define CY_BAOMING_FW_NAME	""
#endif
#ifndef CY_JUNDA_FW_NAME
#define CY_JUNDA_FW_NAME	""
#endif
#ifndef CY_JIAGUAN_FW_NAME
#define CY_JIAGUAN_FW_NAME	""
#endif
#ifndef CY_MUDONG_FW_NAME
#define CY_MUDONG_FW_NAME	""
#endif
#ifndef CY_EACHOPTO_FW_NAME
#define CY_EACHOPTO_FW_NAME	""
#endif

/*
固件名按照这个顺序排列
*/
enum TOUCH_MOUDLE {
	TPK = 0,
	TRULY,
	SUCCESS,
	OFILM,
	LEAD,
	WINTEK,
	LAIBAO,
	CMI,
	ECW,
	GOWORLD,
	BAOMING,
	EACHOPTO,
	MUTTO,
	JUNDA,
	JIAGUAN,
	MUDONG,
	AVC,
	BOE,
	TIANMA,
	SAMSUNG,
	JDI,
	DIJING,
	LCE,
	UNKNOWN = 0xff
};
#define SYN_MOUDLE_NUM_MAX 30
#define FTC_MOUDLE_NUM_MAX 17
#define GTP_MOUDLE_NUM_MAX 6
#define CY_MOUDLE_NUM_MAX 15

#endif

