/* =========================================================================
** Copyright (C) 2007 - 2021 Spin & Turn, Lda. All rights reserved.
** =========================================================================
**
** Project:     RLNP Reader
**
** =========================================================================
**
** History:
**
** Date        Author          Comment
** -------------------------------------------------------------------------
** 2021-01-25  Paulo Arede	   Implementation 4 nRF IC
** =========================================================================
*/
#ifndef ST_MODULES_BQ24155_CHARGER_BQ24155_H_
#define ST_MODULES_BQ24155_CHARGER_BQ24155_H_

/* status register */
#define BQ2415X_BIT_TMR_RST		7
#define BQ2415X_BIT_OTG			7
#define BQ2415X_BIT_EN_STAT		6
#define BQ2415X_MASK_STAT		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_STAT		4
#define BQ2415X_BIT_BOOST		3
#define BQ2415X_MASK_FAULT		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_FAULT		0

enum bq24155_status_events {
	EV_READY = 0,
	EV_CHARGE_IN_PROGRESS,
	EV_CHARGE_DONE,
	EV_FAULT,
	EV_HIGH_Z,
	EV_NO_BATT,		//Until this, event is same as BQ Status
	EV_POWER_PLUGED,
	EV_POWER_UNPLUGED,
	EV_OUTPUT_OVP = 0x43,			//100-Output OVP
	EV_NONE=128,
};

enum bq24155_generic_status {
	BQ_READY=0,
	BQ_CHARGING,
	BQ_CHARGE_DONE,
	BQ_FAULT,
	BQ_HIGH_Z,
	BQ_NO_BATT,
};

//BQ Charge Status Register
const char *bq2415x_status[] = {
	"Ready",
	"Charge in progress",
	"Charge done",
	"Fault",
	"High impedance mode",
	"No battery",
	"Fault - Output OVP",
};
//BQ Charge Mode Register
const char *bq2415x_charge_mode[] = {
	"Normal",
	"VBUS OVP",
	"Sleep mode",
	"Poor input source or VBUS<UVLO",
	"Output OVP",
	"Thermal shutdown",
	"Timer fault",
	"No battery",
};

//Engine status charger
enum internal_status_charger {
	IDLE = 0,
	IDLE_HIGH_Z,
	RESET_ENABLE_BQ,
	FAST_CHARGE_SET_PARAMS,
	WAIT_STATUS,
	FAST_CHARGING,
	FAST_CHARGE_FAULT,
	DISABLE_FAST_FAULT,
	RESET_ENABLE_4WAIT,
	WAIT_4_T2,
	FAST_CHARGE_DONE,
	WAIT_T20MIN_OR_EVENT,
	WAIT_EVENT,
	NO_BATTERY,
};
//Engine status charger STRING
const char *internal_status_charger_str[] = {
	"IDLE",
	"IDLE_HIGH_Z",
	"RESET_ENABLE_BQ",
	"FAST_CHARGE_SET_PARAMS",
	"WAIT_STATUS",
	"FAST_CHARGING",
	"FAST_CHARGE_FAULT",
	"DISABLE_FAST_FAULT",
	"RESET_ENABLE_4WAIT",
	"WAIT_4_T2",
	"FAST_CHARGE_DONE",
	"WAIT_T20MIN_OR_EVENT",
	"WAIT_EVENT",
	"NO_BATTERY"
};


#endif /* ST_MODULES_BQ24155_CHARGER_BQ24155_H_ */
