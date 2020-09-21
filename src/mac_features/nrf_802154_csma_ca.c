/* Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *      list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 *   This file implements CSMA-CA procedure for the 802.15.4 driver.
 *
 */

#define NRF_802154_MODULE_ID NRF_802154_MODULE_ID_CSMACA

#include "nrf_802154_csma_ca.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "nrf_802154_config.h"
#include "nrf_802154_const.h"
#include "nrf_802154_debug.h"
#include "nrf_802154_notification.h"
#include "nrf_802154_pib.h"
#include "nrf_802154_procedures_duration.h"
#include "nrf_802154_request.h"
#include "nrf_802154_stats.h"
#include "nrf_802154_utils.h"
#include "mac_features/nrf_802154_frame_parser.h"
#include "platform/random/nrf_802154_random.h"
#include "rsch/nrf_802154_rsch.h"
#include "timer_scheduler/nrf_802154_timer_sched.h"

#if NRF_802154_CSMA_CA_ENABLED

static uint8_t         m_nb;      ///< The number of times the CSMA-CA algorithm was required to back off while attempting the current transmission.
static uint8_t         m_be;      ///< Backoff exponent, which is related to how many backoff periods a device shall wait before attempting to assess a channel.
static const uint8_t * mp_data;   ///< Pointer to a buffer containing PHR and PSDU of the frame being transmitted.
static volatile bool   m_denied;  ///< Flag that indicates that access to preconditions was denied during the procedure.

/**
 * @brief State of the CSMA-CA procedure.
 */
typedef enum
{
    STATE_IDLE,        ///< The CSMA-CA procedure is not running.
    STATE_CFG_BACKOFF, ///< The module is actively configuring the next backoff (scheduling next transmission operation).
    STATE_BACKOFF,     ///< The module is waiting during the backoff.
    STATE_PREPARE,     ///< The module is actively requesting the Core to start CCA followed by TX.
    STATE_TRANSMIT,    ///< The CCA followed by TX is configured and managed by the Core. Waiting for the notification.
} procedure_state_t;

static volatile procedure_state_t m_procedure_state = STATE_IDLE; ///< Current state of the CSMA-CA procedure.

/**
 * @brief Perform appropriate actions for busy channel conditions.
 *
 * According to CSMA-CA description in 802.15.4 specification, when channel is busy NB and BE shall
 * be incremented and the device shall wait random delay before next CCA procedure. If NB reaches
 * macMaxCsmaBackoffs procedure fails.
 *
 * @retval true   Procedure failed and TX failure should be notified to the next higher layer.
 * @retval false  Procedure is still ongoing and TX failure should be handled internally.
 */
static bool channel_busy(void);

/**
 * @brief Atomically change state of the module.
 *
 * The state is changed only if the current state is in the @p expected_state_mask. The result of
 * the operation is notified in the return code.
 *
 * @param[in]  new_state            The state to set.
 * @param[in]  expected_state_mask  Bitmask of states expected before the state change.
 *
 * @retval true   The state was changed.
 * @retval false  The state was not changed, because the state before the state change was not in
 *                the @p expected_state_mask.
 */
static bool state_set(procedure_state_t new_state, uint32_t expected_state_mask)
{
#if !defined(UNIT_TEST)
    do
    {
        uint8_t current_state = __LDREXB((uint8_t *)&m_procedure_state);

        if (!((1UL << (uint32_t)current_state) & expected_state_mask))
        {
            __CLREX();
            return false;
        }
    }
    while (__STREXB(new_state, (uint8_t *)&m_procedure_state));
#else // !defined(UNIT_TEST)
    m_procedure_state = new_state;
#endif // !defined(UNIT_TEST)

    return true;
}

/**
 * @brief Check if CSMA-CA is ongoing.
 *
 * @retval true   CSMA-CA is running.
 * @retval false  CSMA-CA is not running currently.
 */
static bool procedure_is_running(void)
{
    return m_procedure_state != STATE_IDLE;
}

/**
 * @brief Stops CSMA-CA procedure.
 */
static void procedure_stop(void)
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_HIGH);

    bool result = state_set(STATE_IDLE, UINT32_MAX);
    assert(result);
    (void)result;
    m_denied = false;
    __DMB();
    
    nrf_802154_rsch_delayed_timeslot_cancel(RSCH_DLY_CSMACA);

    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_HIGH);
}

static void priority_leverage(void)
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_HIGH);

    bool first_transmit_attempt     = (0 == m_nb);
    bool coex_requires_boosted_prio = (nrf_802154_pib_coex_tx_request_mode_get() ==
                                       NRF_802154_COEX_TX_REQUEST_MODE_CCA_START);

    // Leverage priority only after the first backoff in the specified Coex TX request mode
    if (first_transmit_attempt && coex_requires_boosted_prio)
    {
        // It should always be possible to update this timeslot's priority here
        if (!nrf_802154_rsch_delayed_timeslot_priority_update(RSCH_DLY_CSMACA, RSCH_PRIO_TX))
        {
            assert(false);
        }
    }

    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_HIGH);
}

/**
 * @brief Callback called when the TX request is finished.
 *
 * This callback is called from the Core critical section and has following applications:
 * 1. Notify MAC layer that the procedure was aborted, if it was aborted during active states of
 *    the module.
 * 2. Notify MAC layer that the channel is busy if the TX request has failed and there are no
 *    retries left.
 * 3. Change module state to @ref STATE_TRANSMIT if the TX was requested successfully.
 *
 * @param[in]  result  Result of the TX request.
 */
static void transmit_request_finished(bool result)
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_HIGH);

    if (procedure_is_running())
    {
        if (m_denied)
        {
            nrf_802154_notify_transmit_failed(mp_data, NRF_802154_TX_ERROR_ABORTED);
            procedure_stop();
        }
        else if (!result && (m_nb >= (nrf_802154_pib_csmaca_max_backoffs_get() - 1)))
        {
            nrf_802154_notify_transmit_failed(mp_data, NRF_802154_TX_ERROR_BUSY_CHANNEL);
            procedure_stop();
        }
        else if (result)
        {
            bool state_result = state_set(STATE_TRANSMIT, (1UL << STATE_PREPARE));
            assert(state_result);
            (void)state_result;
        }
        else
        {
            // Intentionally empty
        }
    }

    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_HIGH);
}

/**
 * @brief Perform CCA procedure followed by frame transmission.
 *
 * If transmission is requested, CSMA-CA module waits for notification from the FSM module.
 * If transmission request fails, CSMA-CA module performs procedure for busy channel condition
 * @ref channel_busy().
 *
 * @param[in] dly_ts_id  Unused variable passed from the Radio Scheduler module.
 */
static void frame_transmit(rsch_dly_ts_id_t dly_ts_id)
{
    (void)dly_ts_id;

    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    if (state_set(STATE_PREPARE, (1UL << STATE_BACKOFF) | (1UL << STATE_CFG_BACKOFF)))
    {
        priority_leverage();

        if (!nrf_802154_request_transmit(NRF_802154_TERM_NONE,
                                         REQ_ORIG_CSMA_CA,
                                         mp_data,
                                         true,
                                         NRF_802154_CSMA_CA_WAIT_FOR_TIMESLOT ? false : true,
                                         transmit_request_finished))
        {
            (void)channel_busy();
        }
    }
    else
    {
        assert(!procedure_is_running());
    }

    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}

/**
 * @brief Delay CCA procedure for random (2^BE - 1) unit backoff periods.
 */
static void random_backoff_start(void)
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_HIGH);

    bool result = state_set(STATE_CFG_BACKOFF,
                            (1UL << STATE_IDLE) |
                            (1UL << STATE_PREPARE) |
                            (1UL << STATE_TRANSMIT));
    assert(result);
    (void)result;

    uint8_t backoff_periods = nrf_802154_random_get() % (1 << m_be);

    // If maximum number of CSMA-CA backoffs is equal to 0, this function is called only once
    // and no more backoffs will follow. Forcing the first and only backoff to 0 has the same
    // effect as no backoff at all.
    if (0 == nrf_802154_pib_csmaca_max_backoffs_get())
    {
        backoff_periods = 0;
    }

    rsch_dly_ts_param_t backoff_ts_param =
    {
        .t0               = nrf_802154_timer_sched_time_get(),
        .dt               = backoff_periods * UNIT_BACKOFF_PERIOD,
        .id               = RSCH_DLY_CSMACA,
        .type             = RSCH_DLY_TS_TYPE_RELAXED,
        .started_callback = frame_transmit,
    };

    switch (nrf_802154_pib_coex_tx_request_mode_get())
    {
        case NRF_802154_COEX_TX_REQUEST_MODE_FRAME_READY:
            // To request Coex precondition immediately, priority must be leveraged
            backoff_ts_param.prio = RSCH_PRIO_TX;
            break;

        case NRF_802154_COEX_TX_REQUEST_MODE_CCA_START:
            // Coex should be requested for all backoff periods but the first one
            backoff_ts_param.prio = (m_nb == 0) ? RSCH_PRIO_IDLE_LISTENING : RSCH_PRIO_TX;
            break;

        case NRF_802154_COEX_TX_REQUEST_MODE_CCA_DONE:
            // Coex should not be requested during backoff periods
            backoff_ts_param.prio = RSCH_PRIO_IDLE_LISTENING;
            break;

        default:
            assert(false);
            break;
    }

    // Delayed timeslot with these parameters should always be scheduled
    if (!nrf_802154_rsch_delayed_timeslot_request(&backoff_ts_param))
    {
        assert(false);
    }

    // It is possible that before this function, the requested timeslot is started what would
    // changes the module state. If it happens, we shall not update the state to
    // @ref STATE_BACKOFF. Because of that result of this function is ignored.
    (void)state_set(STATE_BACKOFF, 1UL << STATE_CFG_BACKOFF);
    __DMB();

    // Checking if m_denied flag was set during @ref STATE_CFG_BACKOFF must be performed after the
    // state was changed further.
    // There is possible a rare scenario in which the transmission is started before this condition
    // is checked. In such scenario, the m_denied flag is checked and cleared in
    // @ref transmit_request_finished().
    if (m_denied)
    {
        // If the procedure was aborted while configuring backoff, the procedure was not stopped,
        // to prevent canceling delayed timeslot while adding it.
        // Stop the procedure from the same context it was configured to prevent data races.
        procedure_stop();
        nrf_802154_notify_transmit_failed(mp_data, NRF_802154_TX_ERROR_ABORTED);
    }

    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_HIGH);
}

static bool channel_busy(void)
{
    bool result = true;

    if (procedure_is_running())
    {
        nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

        m_nb++;

        if (m_be < nrf_802154_pib_csmaca_max_be_get())
        {
            m_be++;
        }

        if (m_nb < nrf_802154_pib_csmaca_max_backoffs_get())
        {
            random_backoff_start();
            result = false;
        }
        else
        {
            procedure_stop();
        }

        nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
    }

    return result;
}

void nrf_802154_csma_ca_start(const uint8_t * p_data)
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    uint32_t now = nrf_802154_timer_sched_time_get();

#if (NRF_802154_FRAME_TIMESTAMP_ENABLED)
    nrf_802154_stat_timestamp_write(last_csmaca_start_timestamp, now);
#endif

    assert(!procedure_is_running());
    assert(!m_denied);

    mp_data = p_data;
    m_nb    = 0;
    m_be    = nrf_802154_pib_csmaca_min_be_get();

    random_backoff_start();

    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
}

bool nrf_802154_csma_ca_abort(nrf_802154_term_t term_lvl, req_originator_t req_orig)
{
    bool should_stop_procedure;
    bool should_notify_abort;

    switch (req_orig)
    {
        case REQ_ORIG_CORE:
        case REQ_ORIG_HIGHER_LAYER:
        case REQ_ORIG_RSCH:
        case REQ_ORIG_TX_TIMEOUT:
            switch (m_procedure_state)
            {
            case STATE_BACKOFF:
                // CSMA-CA backoff is associated with receive state in the driver, so the driver
                // does not know that a transmission procedure is being performed in the background
                // by this module. Notification needs to be triggered explicitly
                should_stop_procedure = true;
                should_notify_abort   = true;
                break;

            case STATE_CFG_BACKOFF:
            case STATE_PREPARE:
                // Do not stop procedure here. The procedure is going to be stopped together with
                // notification from m_denied flag handler (pretransmission hook or
                // transmit_request_finished or backoff initialization).
                m_denied              = true;
                should_stop_procedure = false;
                should_notify_abort   = false;
                break;

            default:
                should_stop_procedure = true;
                should_notify_abort   = false;
                break;
            }

            break;

        default:
            return true;
    }

    bool result = true;

    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    if (term_lvl >= NRF_802154_TERM_802154)
    {
        // Stop CSMA-CA if termination level is high enough.
        if (should_stop_procedure)
        {
            procedure_stop();
        }

        if (should_notify_abort)
        {
            nrf_802154_notify_transmit_failed(mp_data, NRF_802154_TX_ERROR_ABORTED);
        }
    }
    else
    {
        // Return success in case procedure is already stopped.
        result = !procedure_is_running();
    }

    nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);

    return result;
}

bool nrf_802154_csma_ca_pretransmission(const uint8_t                     * p_frame,
                                        bool                                cca,
                                        nrf_802154_transmit_failed_notify_t notify_function)
{
    (void)cca;
    (void)notify_function;

    bool result = true;

    if (m_denied)
    {
        assert(m_procedure_state == STATE_PREPARE);

        nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

        procedure_stop();
        nrf_802154_notify_transmit_failed(mp_data, NRF_802154_TX_ERROR_ABORTED);

        result = false;

        nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
    }

    return result;
}

bool nrf_802154_csma_ca_tx_failed_hook(const uint8_t * p_frame, nrf_802154_tx_error_t error)
{
    (void)error;

    bool result = true;

    if (p_frame == mp_data)
    {
        nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

        result = channel_busy();

        nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
    }

    return result;
}

bool nrf_802154_csma_ca_tx_started_hook(const uint8_t * p_frame)
{
    if (p_frame == mp_data)
    {
        nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

        procedure_stop();

        nrf_802154_log_function_exit(NRF_802154_LOG_VERBOSITY_LOW);
    }

    return true;
}

void nrf_802154_csma_ca_prio_changed_hook(uint32_t old_prio, uint32_t new_prio)
{
    // Only abort the procedure in this hook if change of priorities indicates denied Coex
    // and if the procedure was not stopped already by any of the other hooks.
    if (((rsch_prio_t)old_prio == RSCH_PRIO_TX) &&
       ((rsch_prio_t)new_prio == RSCH_PRIO_RX))
    {
        switch (m_procedure_state)
        {
            case STATE_IDLE:
                break;

            case STATE_CFG_BACKOFF:
            case STATE_BACKOFF:
            case STATE_PREPARE:
                nrf_802154_csma_ca_abort(NRF_802154_TERM_802154, REQ_ORIG_RSCH);
                break;

            case STATE_TRANSMIT:
                // Previous hook should have terminated the operation
                // Fallthrough

            default:
                assert(false);
                break;
        }
    }
}

#endif // NRF_802154_CSMA_CA_ENABLED
