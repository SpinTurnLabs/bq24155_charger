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
#include "bq24155.h"
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

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define BQ24155_ADDR          (0x6BU)


/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* TIMER instance */
const nrf_drv_timer_t TIMER_BQ24155 = NRF_DRV_TIMER_INSTANCE(0);
static uint8_t flg_timer;

/* Buffer to read from battery charge. */
static uint8_t m_sample;
volatile uint8_t flg_pwr, flg_unpluged;

void set_charge_parameters( uint8_t *bq_reg);
void print_changed_status( uint8_t *bq_reg);
void reset_bit_of_tmr_rst(void);
void bq24155_engine(void);
int init_bq24155( void);

/**
 * @brief Function for setting register on BQ24155 Li-ion battery charger.
 */
uint8_t bq24155_write_register(uint8_t bq_reg, uint8_t val)
{
    ret_code_t err_code;

    /* Writing to BQ24155 register. */
    uint8_t reg[2];
    reg[0] = bq_reg;
    reg[1] = val;
    err_code = nrf_drv_twi_tx(&m_twi, BQ24155_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    return 0;
}

/**
 * @brief Function for reading register on BQ24155 Li-ion battery charger.
 */
uint16_t bq24155_read_registers(uint8_t *bq_regs)
{
    ret_code_t err_code;

	for( uint8_t reg_id=0; reg_id<6; ++reg_id)
	{
		/* Reading BQ24155 register. */
		uint8_t reg[1];
		reg[0] = reg_id;
		err_code = nrf_drv_twi_tx(&m_twi, BQ24155_ADDR, reg, sizeof(reg), true);
		APP_ERROR_CHECK(err_code);
//		while (m_xfer_done == false);
	    nrf_delay_us(500);

	    /* Read 1 byte from the specified address - previous write identifies the given register. */
	    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, BQ24155_ADDR, &m_sample, sizeof(m_sample));
	    APP_ERROR_CHECK(err_code);
	    while (m_xfer_done == false);
	    bq_regs[reg_id] = m_sample;
	    nrf_delay_ms(250);
	}
//NOTE PA2021: Don't understand the reason for registers are shift.
	for( uint8_t n=0; n<6; ++n)
		bq_regs[n] = bq_regs[n+1];
    return 0;
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX) {}
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_bq24155_config = {
       .scl                = PIG_READER_SCL_PIN,
       .sda                = PIG_READER_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_bq24155_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


#define PIN_OUT 	BSP_LED_0
#define PIN_IN 		PIG_READER_13_PIN

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	bool pin_state = nrfx_gpiote_in_is_set(PIG_READER_13_PIN);

	flg_pwr = (uint8_t)pin_state;
	//Make a charger reset mode
	if( !pin_state )
		flg_unpluged = 1;
	else
		nrf_drv_timer_enable(&TIMER_BQ24155);
}

/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;		//NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}


/**
 * @brief Handler for timer events.
 */
void timer_bq24155_event_handler(nrf_timer_event_t event_type, void* p_context)
{

    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
        	flg_timer = 1;
            break;

        default:
            //Do nothing.
            break;
    }
}


uint8_t get_bq24155_flag( void)
{
	return flg_timer;
}


void set_bq24155_flag(uint8_t new_flg)
{
	flg_timer = new_flg;
}


/**
 * @brief Function for timing bq24155 controller application entry.
 */
void bq24155_timer_init(void)
{
    uint32_t time_ms = 500; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_BQ24155, &timer_cfg, timer_bq24155_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_BQ24155, time_ms);

    nrf_drv_timer_extended_compare(
         &TIMER_BQ24155, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}


void start_bq24155(void)
{
	nrf_drv_timer_enable(&TIMER_BQ24155);
}

uint8_t bq_registers[8];
int16_t T2, counter_tmr_rst;
char dbg_str[256];
bool fast_charging;
int16_t counter_down_r;
volatile int8_t last_bq_status, bq_status;

void bq24155_status_machine( uint8_t bq_event)
{
static uint8_t status, last_status;
uint8_t rgv;

	if( bq_event == EV_POWER_UNPLUGED) // && status != IDLE && status != IDLE_HIGH_Z && status != NO_BATTERY)
	{
		status = IDLE;
		return;
	}
	if( bq_event == EV_NO_BATT)
	{
		status = NO_BATTERY;
	}
	switch( status)
	{
		case IDLE:
			if( bq_event == EV_POWER_PLUGED)
				status = RESET_ENABLE_BQ;
			if( bq_event == EV_HIGH_Z)
				status = IDLE_HIGH_Z;
			if( bq_event == EV_READY)
			{
				uint8_t rgc =  bq_registers[1] | 0x02; //High impedance mode
				bq24155_write_register( 1, rgc);
			}
			if( bq_event == EV_CHARGE_IN_PROGRESS)
				status = RESET_ENABLE_BQ;
			if( (bq_event & 0x0F) == EV_FAULT)
				status = RESET_ENABLE_BQ;
			break;
		case IDLE_HIGH_Z:
			if( bq_event == EV_POWER_PLUGED)
			{
				T2 = 0;
				counter_tmr_rst = 0;
				fast_charging = false;
				rgv =  bq_registers[4] | 0x80; //Reset
				bq24155_write_register( 4, rgv);
				nrf_delay_ms(500);
				rgv =   (bq_registers[0] & 0x40) == 0 ? (bq_registers[0] | 0x40) :  (bq_registers[0] & 0xBF) ; //EN stat
				bq24155_write_register( 0, rgv);
				status = RESET_ENABLE_BQ;
				break;
			}
			if( bq_event == EV_READY)
			{
				uint8_t rgc =  bq_registers[1] | 0x02; //High impedance mode
				bq24155_write_register( 1, rgc);
				status = IDLE;
				break;
			}
			if( bq_event == EV_HIGH_Z)
			{
				sprintf( dbg_str, "IDLE_HIGH_Z state go to sleep\r\n");
				//usart_write_buffer_wait(&usart_instance, (uint8_t *) dbg_str, strlen(dbg_str));
				NRF_LOG_INFO("%s", dbg_str);
//TODO: Understand similar nrf function.
				//system_sleep();
				//__WFE();
				//Prepare to go to sleep
				nrf_drv_timer_disable(&TIMER_BQ24155);
				flg_timer = 2;
			}
			break;
		case RESET_ENABLE_BQ:
			if( bq_event == EV_FAULT )
			{
				rgv =  bq_registers[1] | 0x02; //High impedance mode
				bq24155_write_register( 1, rgv);
				status = WAIT_EVENT;
			}
			else if ( bq_event == EV_OUTPUT_OVP )
			{
				T2 = 0;
				counter_tmr_rst = 0;
				fast_charging = true;
				rgv =  bq_registers[4] | 0x80; //Reset
				bq24155_write_register( 4, rgv);
				nrf_delay_ms(500);
				rgv =   (bq_registers[0] & 0x40) == 0 ? (bq_registers[0] | 0x40) :  (bq_registers[0] & 0xBF) ; //EN stat
				bq24155_write_register( 0, rgv);
				nrf_delay_ms(500);
//TODO: Set just battery regulation voltage
				rgv =  bq_registers[2] & 0x03;
				//gv |=  0x90;						//Battery Regulation Voltage: 4.22V
				rgv |=  0x8C;						//Battery Regulation Voltage: 4.20V
				bq24155_write_register( 2, rgv );
				//set_charge_parameters( bq_registers);
				status = FAST_CHARGE_SET_PARAMS;
			}
			else
			{
				T2 = 0;
				counter_tmr_rst = 0;
				fast_charging = true;
				set_charge_parameters( bq_registers);
				status = FAST_CHARGE_SET_PARAMS;
			}
			break;
		case FAST_CHARGE_SET_PARAMS:
/*			counter_tmr_rst = 0;
			fast_charging = true;
			set_charge_parameters( bq_registers);
*/
			status = WAIT_STATUS;
			break;
		case WAIT_STATUS:
			if( bq_event == EV_CHARGE_IN_PROGRESS)
				status = FAST_CHARGING;
			else if ( bq_event == EV_CHARGE_DONE)
				status = FAST_CHARGE_DONE;
			else
			{
				fast_charging = false;
				status = FAST_CHARGE_FAULT;
			}
/*			if( (bq_registers[0] & 0x30) != 0x0 )				//Check if it is charging; STATUS = XX11 XXXX-Fault; XX10 XXXX-Charge Done
				status = FAST_CHARGE_FAULT;
			if( (bq_registers[0] & 0x30) == 0x10 )				//Check if it is charging; STATUS = XX01 XXXX-Charge in progress
				status = FAST_CHARGING;
*/
			break;
		case FAST_CHARGING:
			if( bq_event == EV_CHARGE_DONE)
				status = FAST_CHARGE_DONE;
			else if( bq_event == EV_CHARGE_IN_PROGRESS)
				status = FAST_CHARGING;
			else
			{
				fast_charging = false;
				status = FAST_CHARGE_FAULT;
			}
/*			if( (bq_registers[0] & 0x30) != 0x20 )				//Check if it is charging; XX10 XXXX-Charge Done
				status = FAST_CHARGE_DONE;
*/
			break;
		case FAST_CHARGE_FAULT:
			fast_charging = false;
/*			if( (bq_registers[0] & 0x30) != 0x30 )				//Check if it is charging; STATUS = XX11 XXXX-Fault
				status = RESET_ENABLE_4WAIT;
*/
			if( (bq_event & 0x0F) == EV_FAULT || bq_event == EV_READY)				//Check if it is charging; STATUS = XX11 XXXX-Fault
				status = RESET_ENABLE_4WAIT;
			else
				status = RESET_ENABLE_BQ;
			break;
		case RESET_ENABLE_4WAIT:
			rgv =  bq_registers[4] | 0x80; //Reset
			bq24155_write_register( 4, rgv);
			nrf_delay_ms(500);
			rgv =   (bq_registers[0] & 0x40) == 0 ? (bq_registers[0] | 0x40) :  (bq_registers[0] & 0xBF) ; //EN stat
			bq24155_write_register( 0, rgv);
			T2 = 30;			//About 30 seconds
			status = WAIT_4_T2;
			break;
		case WAIT_4_T2:
			sprintf( dbg_str, ".");
			if( T2-- < 0 )
			{
				sprintf( dbg_str, "\r\n");
				status = RESET_ENABLE_BQ;
			}
			//usart_write_buffer_wait(&usart_instance, (uint8_t *) dbg_str, strlen(dbg_str));
			NRF_LOG_INFO("%s", dbg_str);
			break;
		case FAST_CHARGE_DONE:
			fast_charging = false;
			rgv =  bq_registers[4] | 0x80; //Reset
			bq24155_write_register( 4, rgv);
			nrf_delay_ms(500);
			rgv =   (bq_registers[0] & 0x40) == 0 ? (bq_registers[0] | 0x40) :  (bq_registers[0] & 0xBF) ; //EN stat
			bq24155_write_register( 0, rgv);
			T2 = 1200;		//About 20 minutes
			status = WAIT_T20MIN_OR_EVENT;
			break;
		case WAIT_T20MIN_OR_EVENT:
			if( T2-- < 0 || bq_event == EV_CHARGE_IN_PROGRESS)				//Check if it is charging; STATUS = XX01 XXXX-Charge in progress
			{
				rgv =  bq_registers[1] | 0x02; //High impedance mode
				bq24155_write_register( 1, rgv);
				flg_pwr = 0;
				status = IDLE_HIGH_Z;
			}
			if( (bq_event & 0x0F) == EV_FAULT)
				status = RESET_ENABLE_BQ;
			break;
		case WAIT_EVENT:
			if( bq_event == EV_HIGH_Z)
				status = IDLE_HIGH_Z;
			else
				status = IDLE;
			break;
		case NO_BATTERY:
			if( bq_event == EV_NO_BATT)
				break;
			if( bq_event == EV_HIGH_Z)
				status = IDLE_HIGH_Z;
			else if( bq_event < EV_POWER_UNPLUGED)
				status = IDLE;
			break;
		default:
			break;
	}
	if( last_status != status)
	{
		last_status = status;
		sprintf( dbg_str, "STATUS: %s", internal_status_charger_str[status]);
		//usart_write_buffer_wait(&usart_instance, (uint8_t *) dbg_str, strlen(dbg_str));
		NRF_LOG_INFO("%s", dbg_str);
		NRF_LOG_FLUSH();
	}
}


int init_bq24155( void)
{
    twi_init();

    gpio_init();
    nrfx_gpiote_out_set( PIN_OUT);

	fast_charging = false;
	flg_pwr = 0;
	flg_unpluged = 1;

    bq24155_read_registers( bq_registers);

	counter_down_r = 20;
	last_bq_status = -1;
	bq_status = 0;

	bq24155_timer_init();
	return 0;
}


void bq24155_engine(void)
{

	if( flg_pwr)
	{
		memset( dbg_str, 0, sizeof(dbg_str));
		sprintf( dbg_str, "Detected power plug");

		NRF_LOG_INFO("%s", dbg_str);
		NRF_LOG_FLUSH();
		bq24155_status_machine(EV_POWER_PLUGED);
		flg_pwr = 0;
		counter_down_r = 0;
	}
	//Independent loop for reading registers
	if( counter_down_r++ > 0)
	{
		counter_down_r = 0;
		for( int8_t t=0; t<8; ++t)	bq_registers[t] = 0;
		bq24155_read_registers( bq_registers);
		print_changed_status( bq_registers);
		if( (bq_registers[0] & 0x30) == 0x10 )				//Check if it is charging
			nrfx_gpiote_out_clear( PIN_OUT);					//STATUS = 01-Charge in progress
		else
			nrfx_gpiote_out_set( PIN_OUT);					//Other status

		if( bq_registers[3] == 0 || (bq_registers[0] & 0x07) == 0x07)
		{
			bq_status = BQ_NO_BATT;
		}
		else
		{
			if( (bq_registers[1] & 0x02))	//HIGH -Z
			{
				bq_status = BQ_HIGH_Z;
			}
			else
			{
				if( (bq_registers[0] & 0x37) == 0x34)
					bq_status = EV_OUTPUT_OVP;
				else
					bq_status = (bq_registers[0] & 0x30) >> 4;

			}
		}
		if( last_bq_status != bq_status)
		{
			if( bq_status == EV_OUTPUT_OVP)
				sprintf( dbg_str, "BQ Status: %s", bq2415x_status[6]);
			else
				sprintf( dbg_str, "BQ Status: %s", bq2415x_status[bq_status]);

			NRF_LOG_INFO("%s", dbg_str);
			NRF_LOG_FLUSH();
			last_bq_status = bq_status;
		}
		bq24155_status_machine( bq_status );
		return;
	}

	//General condition in case of unplugged.
	if( !nrfx_gpiote_in_is_set(PIG_READER_13_PIN)) // && bq_status == BQ_HIGH_Z)
	{
		if( (bq_registers[1] & 0xFD) == 0)
		{
			uint8_t rgc =  bq_registers[1] | 0x02; //High impedance mode
			bq24155_write_register( 1, rgc);
		}
		if( flg_unpluged )
		{
			sprintf( dbg_str, "Power unplugged");

			NRF_LOG_INFO("%s", dbg_str);
			NRF_LOG_FLUSH();
			nrfx_gpiote_out_set( PIN_OUT);		//Other status
			bq24155_status_machine( EV_POWER_UNPLUGED);
			flg_unpluged = 0;
		}
	}
	if( fast_charging && counter_tmr_rst++ > 10)
	{
		if( nrfx_gpiote_in_is_set(PIG_READER_13_PIN))
		{
			reset_bit_of_tmr_rst();
			bq24155_status_machine(EV_CHARGE_IN_PROGRESS);
		}
		counter_tmr_rst = 0;
	}
/*
	bq24155_read_registers( bq_registers);
	sprintf( dbg_str, "Register value; ");
	for( uint8_t t=0; t<5; ++t)
		sprintf( dbg_str, "%s %02X", dbg_str, bq_registers[t] );
	NRF_LOG_INFO("%s", dbg_str);
	NRF_LOG_FLUSH();
*/

}


void set_charge_parameters( uint8_t *bq_reg)
{
	uint8_t tmpreg;

	if( fast_charging)
	{
		tmpreg =  0x20;		//Charge Current (Ichg) -> 200 mA
		tmpreg |=  0x01;	//Enable charge Current Termination (Iterm) -> 50 mA
		bq24155_write_register( 4, tmpreg);

		nrf_delay_ms(500);
		tmpreg =  bq_reg[1] & 0x0F;
		tmpreg |=  0x30;						//Low battery Voltage Threshold - 3.7V
		tmpreg |=  0x40;						//Input Current Limit: 500mA
		tmpreg |=  0x08;						//Enable charge current termination
		bq24155_write_register( 1, tmpreg );

		nrf_delay_ms(500);
		tmpreg =  bq_reg[2] & 0x03;
		tmpreg |=  0x90;						//Battery Regulation Voltage: 4.22V
		bq24155_write_register( 2, tmpreg );
	}
}


void print_changed_status( uint8_t *bq_reg)
{
	static uint8_t last_bq_reg[]= { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	int8_t flgptr, tp;
	char dbg_str_t[64];

	flgptr = 0;
	for( int8_t n=0; n<5; ++n)
	{
		if( last_bq_reg[n] != bq_reg[n])
			flgptr = 1;
		last_bq_reg[n] = bq_reg[n];
	}
	if( flgptr)
	{
			tp = bq_reg[0] & 0x07;
			sprintf( dbg_str, "BQ24155 Charge mode: %s\t", bq2415x_charge_mode[tp]);

			tp = (bq_reg[0] >> 4) & 0x03;

			sprintf( dbg_str_t, "Status: %s (", bq2415x_status[tp]);
			strcat( dbg_str,  dbg_str_t);

			for( int8_t t=0; t<5; ++t)
			{
				sprintf( dbg_str_t, "%02X  ", bq_reg[t]);
				strcat( dbg_str,  dbg_str_t);

			}
			if( bq_reg[1] & 0x02)
			{
				sprintf( dbg_str_t, ")  - High impedance mode");
				strcat( dbg_str,  dbg_str_t);
			}
			if( bq_reg[3] == 0)
			{
				sprintf( dbg_str_t, ")  - No battery");
				strcat( dbg_str,  dbg_str_t);
			}

			sprintf( dbg_str_t, ")");
			strcat( dbg_str,  dbg_str_t);

			NRF_LOG_INFO("%s", dbg_str);
			NRF_LOG_FLUSH();
	}
}

void reset_bit_of_tmr_rst(void)
{
	static int8_t ty;
//	char char_roll[] = {"-\\|/"};
	if( fast_charging)
	{
		bq24155_write_register( 0, 0x80);	//reset bit of TMR_RST
		//sprintf( dbg_str, "%c\r", char_roll[ty++]);
		//For testing...
		sprintf( dbg_str, ".");
		//usart_write_buffer_wait(&usart_instance, (uint8_t *) dbg_str, strlen(dbg_str));
		if( ty == 4 ) ty = 0;
	}
}

/** @} */
