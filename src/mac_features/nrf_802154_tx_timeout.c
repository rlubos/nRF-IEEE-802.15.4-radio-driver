/* Copyright (c) 2018, Nordic Semiconductor ASA
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
 *   This file implements transmission timeout procedure for the 802.15.4 driver.
 *
 */

#define NRF_802154_MODULE_ID NRF_802154_MODULE_ID_TX_TIMEOUT

#include "nrf_802154_tx_timeout.h"

#include "../nrf_802154_debug.h"
#include "nrf_802154_pib.h"
#include "nrf_802154_request.h"
#include "nrf_802154_utils.h"
#include "timer_scheduler/nrf_802154_timer_sched.h"

#define RETRY_DELAY       500                    ///< Procedure is delayed by this time if it cannot be performed at the moment [us].

static nrf_802154_timer_t m_timeout_timer;       ///< Timer used for handling transmission timeout.
static const uint8_t    * mp_frame;              ///< Pointer to a buffer containing PHR and PSDU of the frame being transmitted.
static volatile bool      m_procedure_is_active; ///< Flag that indicates if the current frame has already been serviced.

/**
 * @brief Notifies about transmission timeout.
 */
static void notify_tx_error(bool result)
{
    if (result)
    {
        nrf_802154_notify_transmit_failed(mp_frame, NRF_802154_TX_ERROR_TIMEOUT);
    }
}

static void timeout_retry(void)
{
    m_timeout_timer.dt += RETRY_DELAY;

    nrf_802154_timer_sched_add(&m_timeout_timer, true);
}

/**
 * @brief Handles transmission timeout.
 */
static void timeout_handle(void * p_context)
{
    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);

    (void)p_context;

    if (m_procedure_is_active)
    {
        if (nrf_802154_request_receive(NRF_802154_TERM_802154,
                                       REQ_ORIG_TX_TIMEOUT,
                                       notify_tx_error,
                                       false))
        {
            m_procedure_is_active = false;
        }
        else
        {
            timeout_retry();
        }
    }

    nrf_802154_log_function_enter(NRF_802154_LOG_VERBOSITY_LOW);
}

/**
 * @brief Stops timer for transmission timeout.
 */
static void timeout_abort(void)
{
    m_procedure_is_active = false;
    __DMB();

    nrf_802154_timer_sched_remove(&m_timeout_timer, NULL);
}

/**
 * @brief Schedules timer for transmission timeout.
 */
static void timeout_schedule(void)
{
    uint32_t timeout_us = nrf_802154_pib_tx_timeout_get();

    if (timeout_us)
    {
        timeout_abort();

        m_timeout_timer.t0        = nrf_802154_timer_sched_time_get();
        m_timeout_timer.dt        = timeout_us;
        m_timeout_timer.callback  = timeout_handle;
        m_timeout_timer.p_context = NULL;

        m_procedure_is_active = true;

        nrf_802154_timer_sched_add(&m_timeout_timer, false);
    }
}

void nrf_802154_tx_timeout_transmission_ready(const uint8_t * p_frame, bool ready)
{
    if (ready)
    {
        // Stop timeout timer when requested transmission is starting.
        if (p_frame == mp_frame)
        {
            timeout_abort();
        }
    }
    else
    {
        // Start timeout if transmission is requested, but it is not ready at the moment.
        // Transmission is pending in the core module.
        mp_frame = p_frame;
        timeout_schedule();
    }
}

bool nrf_802154_tx_timeout_abort(nrf_802154_term_t term_lvl, req_originator_t req_orig)
{
    if (term_lvl >= NRF_802154_TERM_802154)
    {
        timeout_abort();
    }

    return true;
}
