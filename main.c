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
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


uint8_t get_bq24155_flag( void);
void set_bq24155_flag(uint8_t new_flg);
void start_bq24155(void);
int init_bq24155( void);
void bq24155_engine(void);

#define PIN_TEST 	BSP_LED_1
/**
 * @brief Function for main application entry.
 */
int main(void)
{
	uint8_t bqflg;
	ret_code_t err_code;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\tTWI TI's BQ24155 Li-ion battery charger.");
    NRF_LOG_FLUSH();

    init_bq24155();

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_TEST, &out_config);
    APP_ERROR_CHECK(err_code);

    start_bq24155();

    while (1)
    {
    	if( (bqflg = get_bq24155_flag()))
    	{
    		if( bqflg == 2)
    		{
    			set_bq24155_flag(0);
    			nrfx_gpiote_out_set(PIN_TEST); //Turn OFF LED TEST
    			__WFI();
    		}
    		else
    		{
    			set_bq24155_flag(0);
    			bq24155_engine();
    			//nrfx_gpiote_out_toggle(PIN_TEST);
    		}

    	}
    	nrfx_gpiote_out_clear(PIN_TEST);	//Turn ON LED TEST
        //__WFI();
    }
}

