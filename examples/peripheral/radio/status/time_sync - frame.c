/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "time_sync.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "boards.h"

#include "app_error.h"
#include "app_util_platform.h"
#include "nrf.h"
#include "nrf_atomic.h"
#include "nrf_balloc.h"
#include "nrf_error.h"
#include "nrf_soc.h"
//#include "nrf_sdh_soc.h"
#include "nrf_sdm.h"

#define NRF_LOG_MODULE_NAME time_sync
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#if   defined ( __CC_ARM )
#define TX_CHAIN_DELAY_PRESCALER_0 (699 - 235)
#elif defined ( __ICCARM__ )
#define TX_CHAIN_DELAY_PRESCALER_0 (703 - 235)
#elif defined ( __GNUC__ )
#define TX_CHAIN_DELAY_PRESCALER_0 (704 - 235)
#endif

#define SYNC_TIMER_PRESCALER 0
#define SYNC_RTC_PRESCALER 0

#if SYNC_TIMER_PRESCALER == 0
#define TX_CHAIN_DELAY TX_CHAIN_DELAY_PRESCALER_0
#else
#error Invalid prescaler value
#endif

//changed
#define NRF_RADIO_DISTANCE_MAX_US         (128000000UL - 1UL)

#define TS_LEN_US                            (5000UL)   /**< The timeslot duration, the occupied time in radio of one packet is 540us. > */
#define TX_LEN_EXTENSION_US                  (1000UL)
#define TS_SAFETY_MARGIN_US                  (500UL)   /**< The timeslot activity should be finished with this much to spare. */
#define TS_EXTEND_MARGIN_US                  (700UL)   /**< The timeslot activity should request an extension this long before end of timeslot. */


#define MAIN_DEBUG                           0x12345678UL

static void ppi_sync_timer_adjust_configure(void);
static void ppi_sync_timer_adjust_enable(void);
static void ppi_sync_timer_adjust_disable(void);
static void ppi_radio_rx_disable(void);
static void ppi_radio_rx_configure(void);
static void ppi_radio_tx_configure(void);

typedef PACKED_STRUCT
{
    uint8_t   device_name;  //device num
    uint32_t  count;        //count in current send slot
    uint32_t  time;         //send time in current slot
} sync_pkt_t;

NRF_BALLOC_DEF(m_sync_pkt_pool, sizeof(sync_pkt_t), 10);

static ts_params_t       m_params;

static volatile bool m_send_sync_pkt = false;
static volatile bool peer_radio_received = true;

#define NAME                  (4)

static volatile uint32_t m_master_counter = 0;
static volatile uint32_t m_rcv_count      = 0;
static volatile uint8_t m_device_num      = NAME;
static volatile uint8_t m_period          = 0;
static volatile uint16_t m_ask_period      = 0;
static volatile sync_pkt_t * mp_curr_adj_pkt;

#define SEND_TIMESLOT         (NAME*10)  
uint8_t send_timeslot         = SEND_TIMESLOT;
#define END_TIMESLOT          (200)
#define ASK_TIMESLOT          (800)
static uint32_t COUNT         = 250*400;
#define STORAGE               (4)

static volatile int32_t  timer_offset[STORAGE] = {0};
static volatile uint8_t  cur_point = 0;
static volatile uint32_t send_time_estimate = 0;  //compensate the skew
static volatile uint32_t cur_time_receive = 0;    //real time of receiving in current timeslot
static volatile uint32_t last_count = 0;          //time count in last receive
static volatile uint32_t time = 0;								//record current timestamp


static volatile enum
{
    RADIO_STATE_IDLE, /* Default state */
    RADIO_STATE_RX,   /* Waiting for packets */
    RADIO_STATE_TX    /* Trying to transmit packet */
} m_radio_state = RADIO_STATE_IDLE;

static bool sync_timer_record(sync_pkt_t * p_pkt);
static void timeslot_begin_handler(void);
static void timeslot_end_handler(void);

void RADIO_IRQHandler(void)
{
    if (NRF_RADIO->EVENTS_END != 0)
    {
        NRF_RADIO->EVENTS_END = 0;
        (void)NRF_RADIO->EVENTS_END;
        m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
        time=m_params.high_freq_timer[0]->CC[5];
        NRF_LOG_INFO("radio=%d\n",time);
        if (m_radio_state == RADIO_STATE_RX && 
           (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_Msk) == (RADIO_CRCSTATUS_CRCSTATUS_CRCOk << RADIO_CRCSTATUS_CRCSTATUS_Pos))
        {
            sync_pkt_t * p_pkt;
            bool         adjustment_procedure_started;

            p_pkt = (sync_pkt_t *) NRF_RADIO->PACKETPTR;

            adjustment_procedure_started = sync_timer_record(p_pkt);

            if (adjustment_procedure_started)
            {
                p_pkt = nrf_balloc_alloc(&m_sync_pkt_pool);
                APP_ERROR_CHECK_BOOL(p_pkt != 0);

                NRF_RADIO->PACKETPTR = (uint32_t) p_pkt;
            }
            ++m_rcv_count;
        }

        NRF_RADIO->TASKS_START = 1;
    }
}


void radio_parameter_init(void)
{
    // TX power
    NRF_RADIO->TXPOWER  = RADIO_TXPOWER_TXPOWER_0dBm   << RADIO_TXPOWER_TXPOWER_Pos;

    // RF bitrate
    NRF_RADIO->MODE     = RADIO_MODE_MODE_Nrf_2Mbit       << RADIO_MODE_MODE_Pos;

    // Fast startup mode
    NRF_RADIO->MODECNF0 = RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;

    // CRC configuration
    NRF_RADIO->CRCCNF  = RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos;
    NRF_RADIO->CRCINIT = 0xFFFFFFUL;      // Initial value
    NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1

    // Packet format
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_LFLEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos);
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled     << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big           << RADIO_PCNF1_ENDIAN_Pos)  |
                       (4                                << RADIO_PCNF1_BALEN_Pos)   |
                       (sizeof(sync_pkt_t)               << RADIO_PCNF1_STATLEN_Pos) |
                       (sizeof(sync_pkt_t)               << RADIO_PCNF1_MAXLEN_Pos);
    // Radio address config
    NRF_RADIO->PREFIX0 = m_params.rf_addr[0];
    NRF_RADIO->BASE0   = (m_params.rf_addr[1] << 24 | m_params.rf_addr[2] << 16 | m_params.rf_addr[3] << 8 | m_params.rf_addr[4]);

    NRF_RADIO->TXADDRESS   = 0; //use channal 0 to transmit.
    NRF_RADIO->RXADDRESSES = (1 << 0);  //use channel 1 to receive.

    NRF_RADIO->FREQUENCY = m_params.rf_chn; //Frequency = 2400 + FREQUENCY (MHz). ->125

    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_READY_START_Msk;

    m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
    time=m_params.high_freq_timer[0]->CC[5];
    NRF_LOG_INFO("t=%d\n",m_period,time);
}

static void update_radio_parameters(sync_pkt_t * p_pkt)
{  
//    radio_parameter_init();
    NRF_RADIO->PACKETPTR = (uint32_t) p_pkt;
    NRF_RADIO->EVENTS_END = 0;

}

void timeslot_end_handler(void)
{
//    NRF_RADIO->TASKS_DISABLE = 1;
//    NRF_RADIO->INTENCLR      = 0xFFFFFFFF;
//    ppi_radio_rx_disable();

    nrf_balloc_free(&m_sync_pkt_pool, (void*) NRF_RADIO->PACKETPTR);

    m_radio_state           = RADIO_STATE_IDLE;
    m_rcv_count = 0;    //only counting packets received in one timeslot.
    if (m_period==END_TIMESLOT)
    {
        m_period=0;
        send_timeslot=SEND_TIMESLOT;
        NRF_LOG_INFO("end\n");
    }
    if (m_ask_period==ASK_TIMESLOT){
        m_ask_period=0;
    }
    //NRF_LOG_INFO("end=%d\n",m_period);
}

void change_to_RX_state(void)
{
//    if(NRF_RADIO->EVENTS_DISABLED==0)
//    {
////        NRF_RADIO->EVENTS_DISABLED = 0;
////        NRF_RADIO->TASKS_DISABLE = 1;
//        while(NRF_RADIO->EVENTS_DISABLED == 0)
//        { 
//            __NOP(); 
//        }
//    }
//    m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
//    time=m_params.high_freq_timer[0]->CC[5];
//    NRF_LOG_INFO("TX to RX =%d\n",time);
    nrf_balloc_free(&m_sync_pkt_pool, (void*) NRF_RADIO->PACKETPTR);

    sync_pkt_t * p_pkt = nrf_balloc_alloc(&m_sync_pkt_pool);
    APP_ERROR_CHECK_BOOL(p_pkt != 0);
    NRF_RADIO->PACKETPTR = (uint32_t) p_pkt;
	
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_RXEN = 1;

    m_radio_state = RADIO_STATE_RX;
}

void timeslot_begin_handler(void)
{
    sync_pkt_t * p_pkt;
    ++m_period;
    ++m_ask_period;
    //NRF_LOG_INFO("b=%d\n",m_period);

    if (m_period==send_timeslot && m_send_sync_pkt)
    {
        if (m_radio_state == RADIO_STATE_RX)
        {
            NRF_RADIO->EVENTS_DISABLED = 0;
            NRF_RADIO->TASKS_DISABLE = 1;
            while (NRF_RADIO->EVENTS_DISABLED == 0)
            {
                __NOP();
            }
        }

        p_pkt = nrf_balloc_alloc(&m_sync_pkt_pool);
        APP_ERROR_CHECK_BOOL(p_pkt != 0);
        NRF_RADIO->PACKETPTR = (uint32_t) p_pkt;
				
				NRF_RADIO->EVENTS_END = 0;
				NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_TXEN = 1;

        while (NRF_RADIO->EVENTS_READY == 0)
        {
            __NOP();
        }

        uint32_t last_send=send_time_estimate;
        send_time_estimate = m_params.high_freq_timer[0]->CC[1]+448;
        uint32_t count=m_params.high_freq_timer[1]->CC[1];

        p_pkt->device_name  = m_device_num;
        p_pkt->time         = send_time_estimate;
        p_pkt->count        = count;

        m_radio_state = RADIO_STATE_TX; //Trying to transmit packet!!!!!
//        if(NRF_RADIO->EVENTS_END==0)
//        {
//            while (NRF_RADIO->EVENTS_END == 0)
//            {
//                __NOP();  //wait RADIO to transmit packet
//            }
//        }

        m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
        time=m_params.high_freq_timer[0]->CC[5];
        NRF_LOG_INFO("p=%d,send=%d\n",m_period,time);
        change_to_RX_state();
    }
    else
    {
        if (m_radio_state    != RADIO_STATE_RX ||
            NRF_RADIO->STATE != (RADIO_STATE_STATE_Rx << RADIO_STATE_STATE_Pos))
        {
            p_pkt = nrf_balloc_alloc(&m_sync_pkt_pool);
            APP_ERROR_CHECK_BOOL(p_pkt != 0);

			NRF_RADIO->PACKETPTR = (uint32_t) p_pkt;
					
			NRF_RADIO->EVENTS_END = 0;
            NRF_RADIO->TASKS_RXEN = 1;
					
            m_radio_state = RADIO_STATE_RX;
        }
    }

}

static void timestamp_counter_start(void)
{
    m_params.high_freq_timer[1]->TASKS_STOP  = 1;
    m_params.high_freq_timer[1]->TASKS_CLEAR = 1;
    m_params.high_freq_timer[1]->PRESCALER   = 0;
    m_params.high_freq_timer[1]->MODE        = TIMER_MODE_MODE_Counter << TIMER_MODE_MODE_Pos;
    m_params.high_freq_timer[1]->BITMODE     = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    m_params.high_freq_timer[1]->CC[0]       = END_TIMESLOT;  //how to indicate LED??
    m_params.high_freq_timer[1]->CC[2]       = COUNT;         //count period
    m_params.high_freq_timer[1]->SHORTS      = TIMER_SHORTS_COMPARE2_CLEAR_Msk;
    m_params.high_freq_timer[1]->TASKS_START = 1;
}

static void sync_timer_start(void)
{
    m_params.high_freq_timer[0]->TASKS_STOP  = 1;
    m_params.high_freq_timer[0]->TASKS_CLEAR = 1;
    m_params.high_freq_timer[0]->PRESCALER   = SYNC_TIMER_PRESCALER;
    m_params.high_freq_timer[0]->BITMODE     = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    m_params.high_freq_timer[0]->MODE        = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
    m_params.high_freq_timer[0]->EVENTS_COMPARE[0]   = 0;
    m_params.high_freq_timer[0]->EVENTS_COMPARE[2]   = 0;
//    m_params.high_freq_timer[0]->INTENSET    = (TIMER_INTENSET_COMPARE2_Set << TIMER_INTENSET_COMPARE2_Pos);
    m_params.high_freq_timer[0]->CC[0]       = TIME_SYNC_TIMER_MAX_VAL;
    m_params.high_freq_timer[0]->CC[1]       = 0xFFFFFFFF;
    m_params.high_freq_timer[0]->CC[2]       = (TIME_SYNC_TIMER_MAX_VAL - TIME_SYNC_TIMER_SAFETY_MARGIN);
    m_params.high_freq_timer[0]->CC[3]       = 0xFFFFFFFF;
    if (m_params.high_freq_timer[0] == NRF_TIMER3 || m_params.high_freq_timer[0] == NRF_TIMER4)
    {
        // TIMERS 0,1, and 2 only have 4 compare registers
        m_params.high_freq_timer[0]->CC[4]   = TIME_SYNC_TIMER_MAX_VAL; // Only used for debugging purposes such as pin toggling
        m_params.high_freq_timer[0]->CC[5]   = 0xFFFFFFFF;
    }
    m_params.high_freq_timer[0]->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    m_params.high_freq_timer[0]->TASKS_START = 1;

    //changed
    NVIC_ClearPendingIRQ(TIMER3_IRQn);    //not safe!
    NVIC_SetPriority(TIMER3_IRQn, 3);
    NVIC_EnableIRQ(TIMER3_IRQn);
}

void SWI3_EGU3_IRQHandler(void)   //changed
{
    if (NRF_EGU3->EVENTS_TRIGGERED[0] != 0)
    {
        NRF_EGU3->EVENTS_TRIGGERED[0] = 0;

        m_params.high_freq_timer[0]->EVENTS_COMPARE[0] = 0;
        (void)m_params.high_freq_timer[0]->EVENTS_COMPARE[0];
        
        //NRF_RADIO->POWER = (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos);
        //NRF_RADIO->TASKS_START = 1;
        timeslot_begin_handler();
    }
    else if(NRF_EGU3->EVENTS_TRIGGERED[1] != 0)
    {
        NRF_EGU3->EVENTS_TRIGGERED[1] = 0;
        
        m_params.high_freq_timer[0]->EVENTS_COMPARE[2] = 0;
        (void)m_params.high_freq_timer[0]->EVENTS_COMPARE[2];

        timeslot_end_handler();
//        NVIC_DisableIRQ(RADIO_IRQn);
    }
}

static inline bool sync_timer_record(sync_pkt_t * p_pkt)
{
		uint32_t peer_time = p_pkt->time;
		uint32_t peer_count = p_pkt->count;

		cur_time_receive = m_params.high_freq_timer[0]->CC[1];
		last_count = m_params.high_freq_timer[1]->CC[1];
		NRF_LOG_INFO("record,cur=%d,count=%d\n", cur_time_receive, last_count);
		NRF_LOG_INFO("peer=%d,time=%d,count=%d\n", p_pkt->device_name, peer_time, peer_count);

    mp_curr_adj_pkt = p_pkt;
    nrf_balloc_free(&m_sync_pkt_pool, (void*)mp_curr_adj_pkt);

    return true;
}

static void ppi_radio_rx_configure(void)
{
    uint32_t chn2;

    chn2 = m_params.ppi_chns[2];

    NRF_PPI->CH[chn2].EEP   = (uint32_t) &NRF_RADIO->EVENTS_ADDRESS;
    NRF_PPI->CH[chn2].TEP   = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[1];
    NRF_PPI->FORK[chn2].TEP = (uint32_t) &m_params.high_freq_timer[1]->TASKS_CAPTURE[1];
    NRF_PPI->CHENSET       = (1 << chn2);
}

static void ppi_radio_tx_configure(void)
{
    uint32_t chn4;

    chn4 = m_params.ppi_chns[4];

    NRF_PPI->CH[chn4].EEP   = (uint32_t) &NRF_RADIO->EVENTS_READY;
    NRF_PPI->CH[chn4].TEP   = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[1];
    NRF_PPI->FORK[chn4].TEP = (uint32_t) &m_params.high_freq_timer[1]->TASKS_CAPTURE[1];
    NRF_PPI->CHENSET       = (1 << chn4);
}

static void ppi_timeslot_begin_configure(void)
{
    uint32_t chn3 = m_params.ppi_chns[3];

    NRF_PPI->CH[chn3].EEP   = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[0];
    NRF_PPI->CH[chn3].TEP   = (uint32_t) &m_params.high_freq_timer[1]->TASKS_COUNT;    //Increment Timer (Counter mode only)
    NRF_PPI->FORK[chn3].TEP = (uint32_t) &m_params.egu->TASKS_TRIGGER[0];
    NRF_PPI->CHENSET       = (1 << chn3);
}

static void ppi_timeslot_end_configure(void)
{
    uint32_t chn1 = m_params.ppi_chns[1];

    NRF_PPI->CH[chn1].EEP   = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[2];
    NRF_PPI->CH[chn1].TEP   = (uint32_t) &m_params.egu->TASKS_TRIGGER[1];
    NRF_PPI->FORK[chn1].TEP = 0;
    NRF_PPI->CHENSET       = (1 << chn1);
}

static void ppi_counter_timer_capture_disable(uint32_t chn)
{
    NRF_PPI->CHENCLR = (1 << chn);

    NRF_PPI->CH[chn].EEP    = 0;
    NRF_PPI->CH[chn].TEP    = 0;
    NRF_PPI->FORK[chn].TEP  = 0;
}

static void ppi_radio_rx_disable(void)
{
    uint32_t chn2;

    chn2 = m_params.ppi_chns[2];

    NRF_PPI->CHENCLR = (1 << chn2);
}

static void ppi_sync_timer_adjust_enable(void)
{
    NRF_PPI->TASKS_CHG[m_params.ppi_chg].EN = 1;
}

static void ppi_sync_timer_adjust_disable(void)
{
    NRF_PPI->TASKS_CHG[m_params.ppi_chg].DIS = 1;
}

uint32_t ts_init(const ts_params_t * p_params)
{
    memcpy(&m_params, p_params, sizeof(ts_params_t));

    if (m_params.high_freq_timer[0] == 0 ||
        m_params.rtc                == 0 ||
        m_params.egu                == 0)
    {
        // TODO: Check all params
        return NRF_ERROR_INVALID_PARAM;
    }

    if (m_params.egu != NRF_EGU3)
    {
        // TODO: Remove hardcoded use of SWI3_EGU3_IRQHandler()
        return NRF_ERROR_INVALID_PARAM;
    }

    APP_ERROR_CHECK(nrf_balloc_init(&m_sync_pkt_pool));

    return NRF_SUCCESS;
}

uint32_t ts_enable(void)
{
    ppi_timeslot_begin_configure();
    ppi_timeslot_end_configure();
		ppi_radio_tx_configure();
		ppi_radio_rx_configure();
		
		timestamp_counter_start();
    sync_timer_start();
    radio_parameter_init();

    NVIC_ClearPendingIRQ(m_params.egu_irq_type);
    NVIC_SetPriority(m_params.egu_irq_type, 7);
    NVIC_EnableIRQ(m_params.egu_irq_type);

    m_params.egu->INTENCLR = 0xFFFFFFFF;
    m_params.egu->INTENSET = EGU_INTENSET_TRIGGERED0_Msk | EGU_INTENSET_TRIGGERED1_Msk;

    m_send_sync_pkt            = false;
    m_radio_state              = RADIO_STATE_IDLE;

    NRF_LOG_INFO("sync init finished.\n");

    return NRF_SUCCESS;
}

uint32_t ts_tx_start(uint32_t freq)
{
    m_send_sync_pkt = true;

    return NRF_SUCCESS;
}

uint32_t ts_tx_stop()
{
    m_send_sync_pkt = false;

    return NRF_SUCCESS;
}
