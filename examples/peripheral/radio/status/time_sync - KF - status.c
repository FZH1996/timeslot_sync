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
//static void ts_on_sys_evt(uint32_t sys_evt, void * p_context);
//
//NRF_SDH_SOC_OBSERVER(timesync_soc_obs,     \
//                     TS_SOC_OBSERVER_PRIO, \
//                     ts_on_sys_evt, 0);

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
    //uint32_t  sequence;
    //int32_t  rtc_val;
    uint8_t   device_name;  //device num
    uint32_t  last_receive; //receive in lastest receive slot
    uint32_t  last_count;   //count in lastest receive slot
    uint32_t  count;        //count in current send slot
    uint32_t  time;         //send time in current slot
    float     skew;         //skew in current slot
    int32_t   offset;       //offset in current slot
    uint32_t  pri_time;     //receive in last two receive slot
    uint32_t  pri_count;    //count in last two receive slot
    uint32_t  last_send;    //send time in last period
    uint32_t  ask_time;     //need other nodes to return their global times when receive
} sync_pkt_t;

NRF_BALLOC_DEF(m_sync_pkt_pool, sizeof(sync_pkt_t), 10);

static volatile bool     m_timeslot_session_open;
static volatile uint32_t m_blocked_cancelled_count;
static uint32_t          m_timeslot_distance = 0;
static ts_params_t       m_params;

static volatile bool m_send_sync_pkt = false;
//static volatile bool m_timer_update_in_progress = false;
static volatile bool pkt_send_in_last = false;
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
//#define COMPENSATE_TIMESLOT   (200)
#define END_TIMESLOT          (200)
#define ASK_TIMESLOT          (800)
static uint32_t COUNT         = 250*400;
#define STORAGE               (4)

static volatile int32_t  timer_offset[STORAGE] = {0};
static volatile uint8_t  cur_point = 0;
static volatile int32_t  offset = 0;
static volatile int32_t  delay = 0;
static volatile float    skew = 1.0;                 //estimated skew of clock
static volatile float    relative_skew = 1.0;        //global consensus skew
static volatile uint32_t send_time_estimate = 0;  //compensate the skew
static volatile uint32_t pri_time_update = 0;     //compensated time in last timeslot
static volatile uint32_t pri_time_receive = 0;    //compensated time in last last timeslot
static volatile uint32_t cur_time_receive = 0;    //real time of receiving in current timeslot
static volatile uint32_t last_count = 0;          //time count in last receive
static volatile uint32_t pri_count = 0;           //time count in last last receive
static volatile uint32_t last_period[4] = {0};    //updated time in last period
static volatile uint32_t ask_time = 0;
static volatile bool     m_ask_period_print = true;

static volatile float x_prior[3] = {0};
static volatile float x_est[3] = {0};
static volatile uint8_t kk[8] = {0};
static volatile float p_prior[3][3] = {0};
static volatile float A[3][3] = {0};
static volatile uint32_t H[2][3] = {0};
static volatile float W[3] = {0};
static volatile float S[2] = {0};
static volatile float K[3][2] = {0};
static volatile float P[3][3] = {0};
static volatile float R[2] = {0};

static volatile enum
{
    RADIO_STATE_IDLE, /* Default state */
    RADIO_STATE_RX,   /* Waiting for packets */
    RADIO_STATE_TX    /* Trying to transmit packet */
} m_radio_state = RADIO_STATE_IDLE;

//static void sync_timer_consensus_compensate();
static bool sync_timer_record(sync_pkt_t * p_pkt);
static void timeslot_begin_handler(void);
static void timeslot_end_handler(void);

/**< This will be used when requesting the first timeslot or any time a timeslot is blocked or cancelled. */
//static nrf_radio_request_t m_timeslot_req_earliest = {
//        NRF_RADIO_REQ_TYPE_EARLIEST,
//        .params.earliest = {
//            NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
//            NRF_RADIO_PRIORITY_HIGH,
//            TS_LEN_US,                            //length_us
//            NRF_RADIO_EARLIEST_TIMEOUT_MAX_US     //timeout_us
//        }};
//
///**< This will be used at the end of each timeslot to request the next timeslot. */
//static nrf_radio_request_t m_timeslot_req_normal = {
//        NRF_RADIO_REQ_TYPE_NORMAL,
//        .params.normal = {
//            NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
//            NRF_RADIO_PRIORITY_HIGH,
//            TS_LEN_US,                        //distance_us
//            TS_LEN_US                           //length_us
//        }};
//
///**< This will be used at the end of each timeslot to request the next timeslot. */
//static nrf_radio_signal_callback_return_param_t m_rsc_return_sched_next_normal = {
//        NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
//        .params.request = {
//                (nrf_radio_request_t*) &m_timeslot_req_normal
//        }};
//
///**< This will be used at the end of each timeslot to request the next timeslot. */
//static nrf_radio_signal_callback_return_param_t m_rsc_return_sched_next_earliest = {
//        NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
//        .params.request = {
//                (nrf_radio_request_t*) &m_timeslot_req_earliest
//        }};
//
///**< This will be used at the end of each timeslot to request an extension of the timeslot. */
//static nrf_radio_signal_callback_return_param_t m_rsc_extend = {
//        NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND,
//        .params.extend = {TX_LEN_EXTENSION_US}
//        };
//
///**< This will be used at the end of each timeslot to request the next timeslot. */
//static nrf_radio_signal_callback_return_param_t m_rsc_return_no_action = {
//        NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE,
//        .params.request = {NULL}
//        };
static volatile uint32_t time=0;

void  kalman_parameter_init()
{
    //float per=(1000*1000/(TS_LEN_US*END_TIMESLOT));   //5=1000/(40*5)
    A[0][0]=1;A[1][1]=1;A[1][2]=1;A[2][2]=1;A[1][0]=TIME_SYNC_TIMER_MAX_VAL;
    H[0][0]=1;H[1][1]=1;H[1][2]=1;
    W[0]=1/10000.0/10000.0/16.0;W[1]=160.0*160.0;W[2]=16.0*16.0;
    R[0]=1/10000.0/10000.0/16.0;R[1]=32.0*32.0;
}

void RADIO_IRQHandler(void)
{
    if (NRF_RADIO->EVENTS_END != 0)
    {
        NRF_RADIO->EVENTS_END = 0;
        (void)NRF_RADIO->EVENTS_END;
        //NRF_LOG_INFO("m_radio_state=0x%x\n",m_radio_state);
        m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
        time=m_params.high_freq_timer[0]->CC[5];
        NRF_LOG_INFO("radio=%d,p=%d\n",time,m_period);
        //if (m_radio_state == RADIO_STATE_RX) NRF_LOG_INFO("CRC=%d\n",NRF_RADIO->CRCSTATUS);
        if (m_radio_state == RADIO_STATE_RX && 
           (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_Msk) == (RADIO_CRCSTATUS_CRCSTATUS_CRCOk << RADIO_CRCSTATUS_CRCSTATUS_Pos))
        {
            sync_pkt_t * p_pkt;
            bool         adjustment_procedure_started;

            p_pkt = (sync_pkt_t *) NRF_RADIO->PACKETPTR;
            //NRF_LOG_INFO("name=0x%x\n",p_pkt->device_name);

            //if(p_pkt->device_name <= 3)    //&& m_period != COMPENSATE_TIMESLOT
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

/**@brief   Function for handling timeslot events.
 */
//static nrf_radio_signal_callback_return_param_t * radio_callback (uint8_t signal_type)  //sd_radio_session_open
//{
//    // NOTE: This callback runs at lower-stack priority (the highest priority possible).
//    switch (signal_type) {
//    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
//        // TIMER0 is pre-configured for 1Mhz.
//        NRF_TIMER0->TASKS_STOP          = 1;
//        NRF_TIMER0->TASKS_CLEAR         = 1;
//        NRF_TIMER0->MODE                = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
//        NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
//        NRF_TIMER0->EVENTS_COMPARE[1]   = 0;
//
////        if (m_send_sync_pkt)
////        {
//            NRF_TIMER0->INTENSET  = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos);
////        }
////        else
////        {
////            NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos) |
////                                   (TIMER_INTENSET_COMPARE1_Set << TIMER_INTENSET_COMPARE1_Pos);
////        }
//        
//        //if (!m_send_sync_pkt)
//        NRF_TIMER0->CC[0]               = (TS_LEN_US - TS_SAFETY_MARGIN_US);
//        NRF_TIMER0->CC[1]               = (TS_LEN_US - TS_SAFETY_MARGIN_US)/2;
//        //else
//        //NRF_TIMER0->CC[0]               = (m_timeslot_distance - TS_SAFETY_MARGIN_US);
////        NRF_TIMER0->CC[1]               = (TS_LEN_US - TS_EXTEND_MARGIN_US);
//        NRF_TIMER0->BITMODE             = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
//        NRF_TIMER0->TASKS_START         = 1;
//
////        m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
////        time=m_params.high_freq_timer[0]->CC[5];
////        NRF_LOG_INFO("%dstart=%d\n",m_period,time);
//
////        NRF_TIMER0->TASKS_CAPTURE[5]=1;
////        time=NRF_TIMER0->CC[5];
////        NRF_LOG_INFO("%dstart=%d\n",m_period,time);
//
////        if(m_radio_state == RADIO_STATE_TX){
////            NRF_LOG_INFO("TX TASKS_START\n");
////        }
////        else if(m_radio_state == RADIO_STATE_RX){
////            NRF_LOG_INFO("RX TASKS_START\n");
////        }
////        NRF_LOG_INFO("RADIO start,%d\n",m_params.high_freq_timer[0]->CC[1]);
//
//        NRF_RADIO->POWER                = (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos);
//
//        NVIC_EnableIRQ(TIMER0_IRQn);
//
////        NRF_LOG_INFO("m_period_start=%d\n",m_period);
//
////        timeslot_begin_handler();
//
//        break;
//
//    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
//        if (NRF_TIMER0->EVENTS_COMPARE[0] &&
//           (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENCLR_COMPARE0_Pos)))
//        {
//            NRF_TIMER0->TASKS_STOP  = 1;
//            NRF_TIMER0->EVENTS_COMPARE[0] = 0;
//            (void)NRF_TIMER0->EVENTS_COMPARE[0];
//            NRF_TIMER0->EVENTS_COMPARE[1] = 0;
//            (void)NRF_TIMER0->EVENTS_COMPARE[1];
//
//            // This is the "timeslot is about to end" timeout
//            //NRF_LOG_INFO("RADIO end,%d\n",m_params.high_freq_timer[0]->CC[1]);
////            timeslot_end_handler();
//            
////            m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
////            time=m_params.high_freq_timer[0]->CC[5];
////            NRF_LOG_INFO("%dend=%d\n",m_period,time);
//
//            // Schedule next timeslot
//            if (m_send_sync_pkt)
//            {
//////                rand_prng_seed(&m_adv_prng);
//////                uint32_t rand_offset=rand_prng_get(&m_adv_prng);
//////                uint32_t m_distance = (m_timeslot_distance + rand_offset) % NRF_RADIO_DISTANCE_MAX_US;
////                uint32_t m_distance = m_total_timeslot_length + m_timeslot_distance;
////                m_timeslot_req_normal.params.normal.distance_us = m_timeslot_distance; //Distance from the start of the previous radio timeslot
////                m_timeslot_req_normal.params.normal.length_us = m_distance - TS_SAFETY_MARGIN_US;
//////                NRF_LOG_INFO("rand_offset=%d,m_distance=%d\n",rand_offset,m_timeslot_distance);
//////                NRF_LOG_INFO("distance_us=%d,length_us=%d\n",m_timeslot_req_normal.params.normal.distance_us,m_timeslot_req_normal.params.normal.length_us);
//                //NRF_LOG_INFO("m_rsc_return_sched_next_normal\n");
//                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_normal;
//            }
//            else
//            {
//                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_normal;
//            }
//        }
//        break;
//               
//
////        if (NRF_TIMER0->EVENTS_COMPARE[1] &&
////           (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENCLR_COMPARE1_Pos)))
////        {
////            NRF_TIMER0->EVENTS_COMPARE[1] = 0;
////            (void)NRF_TIMER0->EVENTS_COMPARE[1];
////
////            // This is the "try to extend timeslot" timeout
////
////            if (m_total_timeslot_length < (128000000UL - 5000UL - TX_LEN_EXTENSION_US) && !m_send_sync_pkt)
////            {
////                // Request timeslot extension if total length does not exceed 128 seconds 扩展时隙长度
////                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_extend;
////            }
////            else if (!m_send_sync_pkt)
////              {
////                  // Don't do anything. Timeslot will end and new one requested upon the next timer0 compare.
////              }
////        }
//
//    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
////        m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
////        time=m_params.high_freq_timer[0]->CC[5];
////        NRF_LOG_INFO("p=%d,rcv=%d,n=%d\n",m_period,time,((sync_pkt_t *)NRF_RADIO->PACKETPTR)->device_name);
//        RADIO_IRQHandler();
//        //NRF_LOG_INFO("NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO\n");
//        break;
//
//    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
//        // Don't do anything. Our timer will expire before timeslot ends
//        return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_no_action;
//
////    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
////        // Extension succeeded: update timer
////        NRF_TIMER0->TASKS_STOP          = 1;
////        NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
////        NRF_TIMER0->EVENTS_COMPARE[1]   = 0;
////        NRF_TIMER0->CC[0]               += (TX_LEN_EXTENSION_US - 25);
////        NRF_TIMER0->CC[1]               += (TX_LEN_EXTENSION_US - 25);
////        NRF_TIMER0->TASKS_START         = 1;
////
////        // Keep track of total length
////        m_total_timeslot_length += TX_LEN_EXTENSION_US;
////        //NRF_LOG_INFO("EXTEND_SUCCEEDED,\n");
////        break;
//
//    default:
//        app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
//        NRF_LOG_INFO("app_error_handler\n");
//        break;
//    };
//
//    // Fall-through return: return with no action request
//    return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_no_action;
//}

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
    //channel -> 37:2402MHz, 38:2426MHz, 39:2480MHz
//    NRF_RADIO->TXPOWER   = RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos;

    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
    NVIC_EnableIRQ(RADIO_IRQn);
    //__enable_irq();
    NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_READY_START_Msk;

    m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
    time=m_params.high_freq_timer[0]->CC[5];
    NRF_LOG_INFO("t=%d\n",m_period,time);
}

static void update_radio_parameters(sync_pkt_t * p_pkt)
{  
//    radio_parameter_init();
    NRF_RADIO->PACKETPTR = (uint32_t) p_pkt;
    //Packet address to be used for the next transmission or reception. 
    //When transmitting, the packet pointed to by this address will be transmitted
    //and when receiving, the received packet will be written to this address.
    NRF_RADIO->EVENTS_END = 0;

}

/**@brief IRQHandler used for execution context management.
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to stop and disable UESB
  */
void timeslot_end_handler(void)
{
    //changed
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

//    NRF_RADIO->TASKS_START = 1;
//    m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
//    time=m_params.high_freq_timer[0]->CC[5];
//    NRF_LOG_INFO("p=%d,chg=%d\n",m_period,time);
}

void timeslot_begin_handler(void)
{
    sync_pkt_t * p_pkt;
    ++m_period;
    ++m_ask_period;
    //NRF_LOG_INFO("b=%d\n",m_period);

    if ((m_period==send_timeslot || m_ask_period==ASK_TIMESLOT) && m_send_sync_pkt)   // m_period%20==send_timeslot||m_period==ASK_TIMESLOT
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
            // PPI is used to trigger sync timer capture when radio is ready
            // Radio will automatically start transmitting once ready, so the captured timer value must be copied into radio packet buffer ASAP
            __NOP();
        }

        uint32_t last_send=send_time_estimate;
        send_time_estimate = m_params.high_freq_timer[0]->CC[1]+448;
        uint32_t count=m_params.high_freq_timer[1]->CC[1];
        if(m_period!=send_timeslot && m_ask_period==ASK_TIMESLOT){
            p_pkt->device_name  = 5;
            p_pkt->ask_time = TIME_SYNC_TIMER_MAX_VAL;
        }
        else{ 
            p_pkt->device_name  = m_device_num;
            p_pkt->ask_time = 0;
        }

        p_pkt->time         = send_time_estimate;
        p_pkt->count        = count;
        p_pkt->last_receive = pri_time_update;
        p_pkt->last_count   = last_count;
        p_pkt->pri_time     = pri_time_receive;
        p_pkt->pri_count    = pri_count;
        p_pkt->skew         = skew;
        p_pkt->offset       = offset;
        p_pkt->last_send    = last_send;

        m_radio_state = RADIO_STATE_TX; //Trying to transmit packet!!!!!
//        if(NRF_RADIO->EVENTS_END==0)
//        {
//            while (NRF_RADIO->EVENTS_END == 0)
//            {
//                __NOP();  //wait RADIO to transmit packet
//            }
//        }


        if(m_period==send_timeslot) pkt_send_in_last=true;
        else{
            uint32_t count_itv=(count-last_count+COUNT)%COUNT;
            float res=(2-skew)*(send_time_estimate+count_itv*TIME_SYNC_TIMER_MAX_VAL);
            ask_time=(int)(res+offset+TIME_SYNC_TIMER_MAX_VAL)%TIME_SYNC_TIMER_MAX_VAL;
            //NRF_LOG_INFO("ask=%d\n",ask_time);
        }
        
        //uint32_t count_interval=(count-last_count+COUNT)%COUNT;
        //NRF_LOG_INFO("s:p=%d,c=%d\n",m_period,send_time_estimate);
        //NRF_LOG_INFO("count=%d,last=%d,cnt=%d\n",count,last_count,count_interval);
        
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

            //确保定时主机在实际传输数据包时以一致的时间增量捕获自由运行的定时器值
						NRF_RADIO->PACKETPTR = (uint32_t) p_pkt;
					
						NRF_RADIO->EVENTS_END = 0;
            NRF_RADIO->TASKS_RXEN = 1;
					
            m_radio_state = RADIO_STATE_RX;
        }
    }

}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
//void ts_on_sys_evt(uint32_t sys_evt, void * p_context)
//{
//    switch(sys_evt)
//    {
//        case NRF_EVT_FLASH_OPERATION_SUCCESS:
//        case NRF_EVT_FLASH_OPERATION_ERROR:
//            break;
//        case NRF_EVT_RADIO_BLOCKED:
//        case NRF_EVT_RADIO_CANCELED:
//        {
//            // Blocked events are rescheduled with normal priority. They could also
//            // be rescheduled with high priority if necessary.
//            uint32_t err_code = sd_radio_request((nrf_radio_request_t*) &m_timeslot_req_earliest);
//            APP_ERROR_CHECK(err_code);
//            NRF_LOG_INFO("NRF_EVT_RADIO_WRONG\n");
//            m_blocked_cancelled_count++;
//
//            break;
//        }
//        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
//            NRF_LOG_ERROR("NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN\r\n");
//            app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
//            break;
//        case NRF_EVT_RADIO_SESSION_CLOSED:
//            {
//                m_timeslot_session_open = false;
//
//                NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_CLOSED\r\n");
//            }
//
//            break;
//        case NRF_EVT_RADIO_SESSION_IDLE:
//        {
//            NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_IDLE\r\n");
//
//            uint32_t err_code = sd_radio_session_close();
//            APP_ERROR_CHECK(err_code);
//            break;
//        }
//        default:
//            // No implementation needed.
//            NRF_LOG_INFO("Event: 0x%08x\r\n", sys_evt);
//            break;
//    }
//}

static void timestamp_counter_start(void)
{
    // m_params.high_freq_timer[1] (NRF_TIMER) is used in counter mode to count the number of sync timer overflows/resets (m_params.high_freq_timer[0])
    // When timestamp API is used, the number of overflows/resets + current value of sync timer must be added up to give accurate timestamp information
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
    // m_params.high_freq_timer[0] (NRF_TIMER) is the always-running sync timer
    // The timing master never adjusts this timer
    // The timing slave(s) adjusts this timer whenever a sync packet is received and the logic determines that there is
    
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
    m_params.high_freq_timer[0]->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;// | TIMER_SHORTS_COMPARE2_CLEAR_Msk;
    //clear timer when compare0 or compare2 are triggered.
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

m_params.high_freq_timer[0]->TASKS_CAPTURE[5]=1;
time=m_params.high_freq_timer[0]->CC[5];
//NRF_LOG_INFO("%dEND=%d\n",m_period,time);
        timeslot_end_handler();
//        NVIC_DisableIRQ(RADIO_IRQn);
    }
}

static void cal_kalman_iteration(float z1,uint32_t z2,uint8_t peer_itv)
{
        float temp[3][3]={0};
        uint8_t i=0,j=0,k=0;
        float r=0;
        //float us=1000000.0;

        //p_prior=A*P*AT+W;
        memset(p_prior,0,sizeof(p_prior)); 
        for(k=0;k<3;k++){
          for(i=0;i<3;i++){
            r=A[i][k];
            for(j=0;j<3;j++){
              temp[i][j]+=r*P[k][j];
            }
          }
        }
        for(k=0;k<3;k++){
          for(i=0;i<3;i++){
            r=temp[i][k];
            for(j=0;j<3;j++){
              p_prior[i][j]+=r*A[j][k];
            }
          }
        }
        p_prior[0][0]+=W[0]*peer_itv/10000.0;
        p_prior[1][1]+=W[1]*peer_itv;
        p_prior[2][2]+=W[2];
//        float p1,p2;
//        p1=W[0]+P[0][0];
//        p2=P[0][0]*A[1][0]/us;
//        p_prior[0][0]=p1;
//        p_prior[0][1]=p2;
//        p_prior[1][1]=P[1][1]+2*P[1][2]+P[2][2]+p2*A[1][0]+W[1];
//        p_prior[1][2]=P[1][2]+P[2][2];
//        p_prior[2][1]=p_prior[1][2];
//        p_prior[2][2]=P[2][2]+W[2];
//NRF_LOG_INFO("A21=%d,p22="NRF_LOG_FLOAT_MARKER"\n",(int)(A[1][0]),NRF_LOG_FLOAT(p_prior[1][1]));
//NRF_LOG_INFO("p11="NRF_LOG_FLOAT_MARKER",p12="NRF_LOG_FLOAT_MARKER"\n",NRF_LOG_FLOAT(p_prior[0][0]),NRF_LOG_FLOAT(p_prior[0][1]));
        //S=H*p_prior*HT;
        float s1,s2;
        s1=p_prior[1][1]+p_prior[1][2];
        s2=p_prior[1][2]+p_prior[2][2];
        S[0]=0;
        S[1]=s1+s2;
        float SR[2][2]={0};
        SR[0][0]=1/R[0]/peer_itv/peer_itv;SR[1][1]=1/(S[1]+R[1]*peer_itv);
        //K=p_prior*HT*SR;
        memset(K,0,sizeof(K));
        memset(temp,0,sizeof(temp));
        for(k=0;k<3;k++){
          for(i=0;i<3;i++){
            r=p_prior[i][k];
            for(j=0;j<2;j++){
              temp[i][j]+=r*H[j][k];
            }
          }
        }
        for(k=0;k<2;k++){
          for(i=0;i<3;i++){
            r=temp[i][k];
            for(j=0;j<2;j++){
              K[i][j]+=r*SR[k][j];
            }
          }
        }
        //K[0][1]=K[0][1]*us;
//        K[0][0]=p1*SR[0][0];
//        K[0][1]=p2*SR[1][1]*us;
//        K[1][1]=s1*SR[1][1];
//        K[2][1]=s2*SR[1][1];
        //x_est=x_prior+K*(z-H*x_prior);
        int dis=(int)(z2-x_prior[1]);
        correct_overload(&dis,TIME_SYNC_TIMER_MAX_VAL);
        dis=(int)(dis-x_prior[2]);
//        memset(x_est,0,sizeof(x_est));
//        memset(temp,0,sizeof(temp));
//z1=1.0;
        temp[0][0]=z1-x_prior[0];
        temp[0][1]=dis;
        x_est[0]=x_prior[0];
        x_est[1]=x_prior[1];
        x_est[2]=x_prior[2];
        for(k=0;k<2;k++){
          for(i=0;i<3;i++){
            x_est[i]+=K[i][k]*temp[0][k];
          }
        }
//        x_est[0]=x_prior[0]+(r*(z1-x_prior[0])+K[0][1]*dis*us)/us;
//        x_est[1]=(int)(x_prior[1]+K[1][1]*dis);
//        x_est[2]=(int)(x_prior[2]+K[2][1]*dis);
//NRF_LOG_INFO("p12=" NRF_LOG_FLOAT_MARKER ",k11=" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(p_prior[0][1]),NRF_LOG_FLOAT(K[0][0]));
////NRF_LOG_INFO("k21=" NRF_LOG_FLOAT_MARKER ",k22=" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(K[1][0]),NRF_LOG_FLOAT(K[1][1]));
        //P=(I-K*H)*p_prior;
        memset(P,0,sizeof(P));
        memset(temp,0,sizeof(temp));
        for(k=0;k<2;k++){
          for(i=0;i<3;i++){
            r=K[i][k];
            for(j=0;j<3;j++){
              temp[i][j]-=r*H[k][j];
            }
          }
        }
        temp[0][0]+=1;
        temp[1][1]+=1;
        temp[2][2]+=1;
//NRF_LOG_INFO("1=" NRF_LOG_FLOAT_MARKER ",2=" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(temp[1][1]),NRF_LOG_FLOAT(temp[1][2]));
        for(k=0;k<3;k++){
          for(i=0;i<3;i++){
            r=temp[i][k];
            for(j=0;j<3;j++){
              P[i][j]+=r*p_prior[k][j];
            }
          }
        }
////        P[0][1]=(1-K[0][0]/us)*p_prior[0][1]-K[0][1]*(p_prior[1][1]+p_prior[1][2]);
////        //P[0][2]=(1-K[0][0])*p_prior[0][2]-K[0][1]*(p_prior[2][1]+p_prior[2][2]);
////        P[1][0]=P[0][1];P[2][0]=P[0][2]=0;
////        //P[0][0]=P[0][0]*us;
//        P[0][0]=p1-K[0][1]*p2*us;
//        P[1][1]=p_prior[1][1]-s1*K[1][1]+p_prior[0][1]*temp[1][0];
//        P[1][2]=p_prior[1][2]-s2*K[1][1]+p_prior[0][2]*temp[1][0];
//        P[2][1]=P[1][2];
//        P[2][2]=p_prior[2][2]-s2*K[2][1]+p_prior[0][2]*temp[2][0];
//NRF_LOG_INFO("p1=" NRF_LOG_FLOAT_MARKER ",p32=" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(p1),NRF_LOG_FLOAT(P[2][1]));

        //NRF_LOG_INFO("dis=%d\n",dis);
}

void sync_timer_consensus_compensate(uint8_t peer_num)
{
    uint32_t cc = 0;
    uint8_t index = cur_point;
    int32_t adjust = timer_offset[index];

//    if (index>1)        offset = timer_offset[index--]*0.7+timer_offset[index--]*0.2+timer_offset[index]*0.1;
//    else if (index==1)  offset = timer_offset[1]*0.7+timer_offset[0]*0.2+timer_offset[STORAGE-1]*0.1;
//    else                offset = timer_offset[0]*0.7+timer_offset[STORAGE-1]*0.2+timer_offset[STORAGE-2]*0.1;
//    offset = (offset + TIME_SYNC_TIMER_MAX_VAL) % TIME_SYNC_TIMER_MAX_VAL;

    // Write offset to timer compare register
    cc = (-adjust + TIME_SYNC_TIMER_MAX_VAL) % TIME_SYNC_TIMER_MAX_VAL;
//    m_params.high_freq_timer[0]->CC[2] = cc;

    //m_params.high_freq_timer[0]->CC[4] = TIME_SYNC_TIMER_MAX_VAL - (logic_r) % TIME_SYNC_TIMER_MAX_VAL;
//    NRF_TIMER0->TASKS_CAPTURE[1]=1;
//    if(NRF_TIMER0->CC[1]+adjust < NRF_TIMER0->CC[0]){
////        m_timeslot_req_normal.params.normal.distance_us=m_timeslot_req_normal.params.normal.distance_us-adjust;
////        m_timeslot_req_normal.params.normal.length_us=m_timeslot_req_normal.params.normal.length_us-adjust;
//        NRF_TIMER0->CC[0] = NRF_TIMER0->CC[0] - adjust;
//    }
	
    //NRF_LOG_INFO("offset=%d,pri=%d\n",cc,pri_time_update);

//        m_params.high_freq_timer[0]->TASKS_STOP = 1;
//        m_params.high_freq_timer[0]->CC[0] = TIME_SYNC_TIMER_MAX_VAL - skew_offset;
//        //m_params.high_freq_timer[0]->CC[4] = m_params.high_freq_timer[0]->CC[0] / 2;
//        m_params.high_freq_timer[0]->TASKS_START = 1;
		pri_time_receive = pri_time_update; 
    pri_time_update = cur_time_receive + timer_offset[index];
    pri_time_update = (pri_time_update+TIME_SYNC_TIMER_MAX_VAL)%TIME_SYNC_TIMER_MAX_VAL;
    last_period[peer_num-1] = pri_time_update;
    pkt_send_in_last = false;
}

//int abs(int a,int b)
//{
//    if(a>b) return a-b;
//    else return b-a;
//}

void correct_overload(int *p,const uint32_t num)
{
    if (*p<-0.9*num) *p=*p+num;
    else if (*p>0.9*num) *p=*p-num;
}

//接收器在收到同步信标数据包时如何更新其本地自由运行定时器
static inline bool sync_timer_record(sync_pkt_t * p_pkt)
{
    uint32_t peer_time;
    float peer_skew;
    int32_t peer_offset;
    uint8_t peer_num;
    uint32_t peer_count;
    uint32_t peer_last;
    uint32_t peer_last_count;
    uint32_t peer_send;
    uint32_t global_time_estimate;

    peer_time   = p_pkt->time;
    peer_skew   = p_pkt->skew;
    peer_offset = p_pkt->offset;
    peer_num    = p_pkt->device_name;
    peer_count  = p_pkt->count;
    peer_send   = p_pkt->last_send;

    if (pkt_send_in_last)
    {
        peer_last = p_pkt->pri_time;
        peer_last = p_pkt->last_send;   //for test
        peer_last_count = p_pkt->pri_count;
    }
    else
    {
        peer_last = p_pkt->last_receive;
        peer_last_count = p_pkt->last_count;
    }
    if(peer_skew>1.5 || peer_skew<0.5) peer_skew=1.0;
    if(x_est[0]>1.5 || x_est[0]<0.5) x_est[0]=1.0;
    //NRF_LOG_INFO("pp=%d,pl=%d\n",p_pkt->pri_time,p_pkt->last_receive);

    cur_time_receive = m_params.high_freq_timer[0]->CC[1];
    pri_count = last_count;
    last_count = m_params.high_freq_timer[1]->CC[1];
    //NRF_LOG_INFO("record,CC1=%d,pri=%d\n",cur_time_receive,pri_time_update);
//    NRF_LOG_INFO("p=%d,lc=%d\n",m_period,last_count);
    
    
    if(kk[peer_num]<4){ 
        kk[peer_num]=kk[peer_num]+1;
    }
    if (pri_time_update==0){
        x_est[0]=1;
        x_est[1]=peer_last;
        x_est[2]=0;
//        return true;
    }
    else
    { 
//peer_skew=1;
//skew=1;
//x_est[0]=1;
        uint32_t  local_itv = (last_count-pri_count+COUNT) % COUNT;
        uint32_t  peer_itv = (peer_count-peer_last_count+COUNT) % COUNT;
        uint32_t  peer_interval = peer_time-peer_last;
peer_itv=END_TIMESLOT;  //for test
        uint32_t  logic_s=(2-peer_skew)*(peer_time+peer_itv*TIME_SYNC_TIMER_MAX_VAL)+peer_offset; //+delay
        uint32_t  logic_r=(2-skew)*(cur_time_receive+local_itv*TIME_SYNC_TIMER_MAX_VAL)+offset;
        //global_time_estimate=((logic_r % TIME_SYNC_TIMER_MAX_VAL)+(logic_s % TIME_SYNC_TIMER_MAX_VAL))/2;
        global_time_estimate=((logic_r+logic_s))/2;
//NRF_LOG_INFO("pitv=%d,pt=%d,pl=%d\n",peer_itv,peer_time,peer_last);

        float interval=1.0*200*80000;
        int local_cur=cur_time_receive-last_period[peer_num-1];
        ////correct_overload(&local_cur,TIME_SYNC_TIMER_MAX_VAL);
        int local_period=local_cur-delay;
        correct_overload(&local_period,TIME_SYNC_TIMER_MAX_VAL);
        int peer_period=peer_time-peer_send;
        correct_overload(&peer_period,TIME_SYNC_TIMER_MAX_VAL);
        double a_ij=(interval+local_period)/(interval+peer_period);  //request interval+... unless a_ij becomes 1343.176513 when dis is negetive.
//NRF_LOG_INFO("d=%d,lp=%d,f=%d,lc=%d\n",delay,last_period[peer_num-1],local_period,local_cur);
//NRF_LOG_INFO("a_ij=" NRF_LOG_FLOAT_MARKER ",peer_skew=" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(a_ij),NRF_LOG_FLOAT(peer_skew));
        
        float z1;
        uint32_t z2;
        z1=a_ij*peer_skew;
        z2=cur_time_receive;
        //A[1][0]=peer_interval;
        A[1][0]=(peer_interval+peer_itv*TIME_SYNC_TIMER_MAX_VAL);    //need to plus the during period
        A[1][0]=A[1][0]*(2-peer_skew)-delay*(2-skew);

        //x_prior=A*x_est;
//        uint8_t i=0,j=0,k=0;
//        float r=0;
//        for(k=0;k<3;k++){
//          for(i=0;i<3;i++){
//            x_prior[i]=x_prior[i]+A[i][k]*x_est[k];
//          }
//        }
        //x_prior=A*x_est;
        int est=(int)(A[1][0]*x_est[0]+x_est[1]+x_est[2]); //x_est[1]+delay affacts x_est[2]
        x_prior[0]=x_est[0];
        x_prior[1]=(est) % TIME_SYNC_TIMER_MAX_VAL;//need an inter value , why ??
        x_prior[2]=x_est[2];
        if (kk[peer_num]==2){
            x_prior[0]=1;
            x_prior[1]=cur_time_receive;
            x_prior[2]=0;
        }
//NRF_LOG_INFO("xe2=%d,xp2=%d,xp3=%d,xp1="NRF_LOG_FLOAT_MARKER"\n",x_est[1],x_prior[1],x_prior[2],NRF_LOG_FLOAT(x_prior[0]));
////NRF_LOG_INFO("pi=%d,est=%d,A=" NRF_LOG_FLOAT_MARKER "\n", peer_interval,est,NRF_LOG_FLOAT(A[1][0]));
//NRF_LOG_INFO("xe2=%d,xe3=%d,xe1="NRF_LOG_FLOAT_MARKER"\n",x_est[1],x_est[2],NRF_LOG_FLOAT(x_est[0]));
//NRF_LOG_INFO("skew=%x,est=%d,delay=%d\n",skew,(est),delay);        

        cal_kalman_iteration(z1,z2,peer_itv);

        //x_est[0]=1;x_est[1]=cur_time_receive;x_est[2]=0;
        x_est[1]=(((int)x_est[1])%TIME_SYNC_TIMER_MAX_VAL+TIME_SYNC_TIMER_MAX_VAL)%TIME_SYNC_TIMER_MAX_VAL;
        x_est[2]=((int)x_est[2]) % TIME_SYNC_TIMER_MAX_VAL;
        skew=x_est[0];
        delay=x_est[2];
        offset=(global_time_estimate-(2-skew)*(cur_time_receive+local_itv*TIME_SYNC_TIMER_MAX_VAL));
        offset=offset+delay/2;
        offset=((int)offset)%TIME_SYNC_TIMER_MAX_VAL;
        //offset=offset+x_est[1] - cur_time_receive;
        int err=(int)(((2-skew)*(x_est[1]+local_itv*TIME_SYNC_TIMER_MAX_VAL)+offset)-logic_s);//%TIME_SYNC_TIMER_MAX_VAL;

        ++cur_point;
        if (cur_point==STORAGE)  cur_point=0;
        //if(x_est[1]>0)
        timer_offset[cur_point] = (x_est[1] - cur_time_receive);

        //print info every inquire period once a time
//        if(m_ask_period<=END_TIMESLOT){
          //int peer_logic=(int)(peer_time/peer_skew+peer_offset);
          //int peer_ask=(int)(p_pkt->ask_time);
          NRF_LOG_INFO("\r\n");
          NRF_LOG_INFO("p=%d\n",m_period);
          //NRF_LOG_INFO("rcv:n=%x,gg=%d,m=%d\n",peer_num,(int)(p_pkt->ask_time),ask_time); //represent sync error.
          NRF_LOG_INFO("pt=%d,pi=%d,po=%d,pv=%d\n",peer_time,peer_interval,peer_offset,peer_itv);
//          NRF_LOG_INFO("lc=%d,ps="NRF_LOG_FLOAT_MARKER"\n",last_count,NRF_LOG_FLOAT(peer_skew));
          NRF_LOG_INFO("xe2=%d,d=%d,skew="NRF_LOG_FLOAT_MARKER",lv=%d\n",x_est[1],(int)x_est[2],NRF_LOG_FLOAT(x_est[0]),local_itv);
          NRF_LOG_INFO("err=%d,ofs=%d,cc=%d,z1="NRF_LOG_FLOAT_MARKER"\n",err,offset,timer_offset[cur_point],NRF_LOG_FLOAT(z1));
NRF_LOG_INFO("cur=%d,xp2=%d,xp3=%d,xp1="NRF_LOG_FLOAT_MARKER"\n",cur_time_receive,x_prior[1],(int)x_prior[2],NRF_LOG_FLOAT(x_prior[0]));
//NRF_LOG_INFO("lp=%d,f=%d,lc=%d,pr=%d\n",last_period[peer_num-1],local_period,local_cur,pri_time_receive);
//NRF_LOG_INFO("a_ij=" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(a_ij));
//NRF_LOG_INFO("l=%d,pp=%d,s=%d\n",p_pkt->last_receive,p_pkt->pri_time,p_pkt->last_send);
//        }
    }

    // m_params.high_freq_timer[0]->CC[2] represent the deviation of sender timer
    if (timer_offset[cur_point] == 0 ||
        timer_offset[cur_point] == TIME_SYNC_TIMER_MAX_VAL)
    {
        // Already in sync
        NRF_LOG_INFO("p=%d,Already in sync\n",m_period);
        //return false;
    }

    //NRF_LOG_INFO("peer_timer=%d,local_timer=%d,global_time_estimate=%d\n",peer_time,local_time,global_time_estimate);
    //NRF_LOG_INFO("timer_offset=%d,timer[0]->CC[2]=%d\n",timer_offset[cur_point],m_params.high_freq_timer[0]->CC[2]);

//#ifdef DEBUG
//    if (m_params.high_freq_timer[0]->CC[2] > TIME_SYNC_TIMER_MAX_VAL)
//    {
//        NRF_LOG_ERROR("%d - %d - %d", peer_timer, local_timer, timer_offset);
//        APP_ERROR_CHECK_BOOL(false);
//    }
//#endif

    //ppi_sync_timer_adjust_enable();
    sync_timer_consensus_compensate(peer_num);

    //m_timer_update_in_progress = true;
    mp_curr_adj_pkt            = p_pkt; //save current packet for recording timestampts
    nrf_balloc_free(&m_sync_pkt_pool, (void*)mp_curr_adj_pkt);
    //ppi_sync_timer_adjust_enable();

    return true;
}

/* Set up timestamp capture */
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
	  //uint32_t chn5;

    chn4 = m_params.ppi_chns[4];
//    chn5 = m_params.ppi_chns[5];

    NRF_PPI->CH[chn4].EEP   = (uint32_t) &NRF_RADIO->EVENTS_READY;
    NRF_PPI->CH[chn4].TEP   = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[1];
    NRF_PPI->FORK[chn4].TEP = (uint32_t) &m_params.high_freq_timer[1]->TASKS_CAPTURE[1];
    NRF_PPI->CHENSET       = (1 << chn4);

//    NRF_PPI->CH[chn5].EEP = (uint32_t) &NRF_TIMER0->EVENTS_COMPARE[1];
//    //NRF_PPI->CH[chn5].TEP = (uint32_t) &NRF_RADIO->TASKS_START;
//    NRF_PPI->CH[chn5].TEP = (uint32_t) &NRF_RADIO->TASKS_TXEN;
//    NRF_PPI->CHENSET          = (1 << chn5);
}

static void ppi_timeslot_begin_configure(void)
{
    uint32_t chn3 = m_params.ppi_chns[3];
    //uint32_t log[]={'C','O','U','N','T','\n'};
    //uint32_t chn5 = m_params.ppi_chns[5];

    NRF_PPI->CH[chn3].EEP   = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[0];
    NRF_PPI->CH[chn3].TEP   = (uint32_t) &m_params.high_freq_timer[1]->TASKS_COUNT;    //Increment Timer (Counter mode only)
    NRF_PPI->FORK[chn3].TEP = (uint32_t) &m_params.egu->TASKS_TRIGGER[0];
    NRF_PPI->CHENSET       = (1 << chn3);

//    NRF_PPI->CH[chn5].EEP   = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[4];
//    NRF_PPI->CH[chn5].TEP   = (uint32_t) &NRF_GPIOTE->TASKS_OUT[3];
//    NRF_PPI->CHENSET       = (1 << chn5);
}

static void ppi_timeslot_end_configure(void)
{
    uint32_t chn1 = m_params.ppi_chns[1];
    //uint32_t log[]={'C','O','U','N','T','\n'};
    //uint32_t chn5 = m_params.ppi_chns[5];

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

    // TODO: Implement use of RTC as a low-power (and lower accuracy)
    // alternative to 16 MHz TIMER
    // if (SYNC_RTC_PRESCALER != m_params.rtc->PRESCALER)
    // {
    //     // TODO: Handle this
    //     return NRF_ERROR_INVALID_STATE;
    // }

    APP_ERROR_CHECK(nrf_balloc_init(&m_sync_pkt_pool));

    return NRF_SUCCESS;
}

uint32_t ts_enable(void)
{
//    uint32_t err_code;

//    if (m_timeslot_session_open)
//    {
//        return NRF_ERROR_INVALID_STATE;
//    }
//
//    err_code = sd_clock_hfclk_request();
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//
//    err_code |= sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//
//    err_code = sd_radio_session_open(radio_callback); //start session
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//
//    err_code = sd_radio_request(&m_timeslot_req_earliest);  //obtain the timeslot
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }

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

    m_blocked_cancelled_count  = 0;
    m_send_sync_pkt            = false;
    m_radio_state              = RADIO_STATE_IDLE;

    m_timeslot_session_open    = true;

    kalman_parameter_init();
    NRF_LOG_INFO("sync init finished.\n");

    return NRF_SUCCESS;
}

uint32_t ts_disable(void)
{
    // TODO:
    //       - Close SoftDevice radio session (sd_radio_session_close())
    //       - Stop radio activity
    //       - Stop timers
    //       - Disable used PPI channels
    //       - Disable used interrupts
    //       - Release HFCLK (sd_clock_hfclk_release()),
    //       - Go back to low-power (mode sd_power_mode_set(NRF_POWER_MODE_LOWPWR))
    // Care must be taken to ensure clean stop. Order of tasks above should be reconsidered.
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t ts_tx_start(uint32_t sync_freq_hz)
{
    uint32_t distance;

    distance = (1000000 / sync_freq_hz);

    if (distance >= NRF_RADIO_DISTANCE_MAX_US)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_timeslot_distance = distance;

    ////while(NRF_TIMER0->EVENTS_COMPARE[0]==0){}

    m_send_sync_pkt = true;
    //m_timer_update_in_progress = false;
    pkt_send_in_last = false;

    return NRF_SUCCESS;
}

uint32_t ts_tx_stop()
{
    m_send_sync_pkt = false;

    return NRF_SUCCESS;
}
