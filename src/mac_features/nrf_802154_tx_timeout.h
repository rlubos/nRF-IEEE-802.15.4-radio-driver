/* Copyright (c) 2020, Nordic Semiconductor ASA
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

#ifndef NRF_802154_TX_TIMEOUT_H__
#define NRF_802154_TX_TIMEOUT_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf_802154_const.h"
#include "nrf_802154_types.h"

/**
 * @brief Schedules or aborts transmission timeout timer.
 * 
 * If the transmission is ready to be performed, it certainly is not going to time out. The timeout
 * timer can be stopped safely.
 * If the transmission is not ready to be performed and it is pending, the timeout timer shall be
 * started
 * 
 * @param[in]  p_frame  Pointer to the buffer that contains the PHR and PSDU of the transmitted frame.
 * @param[in]  ready    Flag that indicates if the transmission is going to execute.
 */
void nrf_802154_tx_timeout_transmission_ready(const uint8_t * p_frame, bool ready);

/**
 * @brief Aborts transmission timeout timer.
 *
 * @param[in]  term_lvl  Termination level set by the request to abort the ongoing operation.
 * @param[in]  req_orig  Module that originates this request.
 *
 * @retval  true   Transmission timeout timer has been stopped.
 */
bool nrf_802154_tx_timeout_abort(nrf_802154_term_t term_lvl, req_originator_t req_orig);

#endif // NRF_802154_TX_TIMEOUT_H__
