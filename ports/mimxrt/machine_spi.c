/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020-2021 Damien P. George
 * Copyright (c) 2021 Robert Hammelrath
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <string.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "extmod/modmachine.h"
#include CLOCK_CONFIG_H

#include "fsl_cache.h"
#include "fsl_dmamux.h"
#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_edma.h"
#include "fsl_qtmr.h"
#include "dma_manager.h"
#include "hal/pwm_backport.h"  // QTMR_SetupPwm_u16 -- a project-local backport, not stock SDK

// --- Periodic DMA transfer extension (SPI.dma_periodic_start() and friends) ---
//
// Added to drive a fixed-rate register update (e.g. a DDS phase/frequency tuning word)
// with hardware timing, independent of CPU polling -- ordinary machine.SPI.write() here
// calls LPSPI_MasterTransferBlocking() (see machine_spi_transfer() below), a fully
// synchronous, CPU-blocking call with no DMA involvement at all; measured sustained
// throughput for that path was ~2000Hz when interleaved with real-time audio I2S, far
// short of a 192kHz per-sample DDS update target. See this project's own memory/design
// notes (not reproduced here) for the fuller rationale; this comment covers only the
// verified hardware facts this implementation depends on.
//
// Verified against the actual vendored SDK/device headers for this chip (i.MX RT1062),
// not assumed:
//   - PIT and GPT have NO entry in the dma_request_source_t enum
//     (sdk/devices/MIMXRT1062/MIMXRT1062.h) -- neither can trigger DMAMUX directly.
//   - Quad Timer (QTMR/TMR) modules DO have dedicated DMAMUX request-source entries for
//     their comparator-preload/reload ("Cmpld") events, e.g.
//     kDmaRequestMuxQTIMER2Cmpld1Timer0Cmpld2Timer1 for TMR2 channel 0.
//   - QTMR_EnableDma()/kQTMR_ComparatorPreload1DmaEnable (sdk/drivers/qtmr_1/fsl_qtmr.h)
//     arms that DMA request on a QTMR channel's period-reload event.
//   - QTMR_SetTimerPeriod()'s own doc comment: "writes to COMP1 or COMP2 depending on
//     count direction" -- an up-counting channel (this project's free-running,
//     kQTMR_PriSrcRiseEdge-off-an-internal-clock configuration, the same recipe
//     machine_pwm.c already uses for TMR-backed PWM objects) writes/compares against
//     COMP1, hence Preload1 (not Preload2) is the correct DMA-enable bit here.
//   - Pin D15 (SoC pad GPIO_AD_B1_03, this project's I/O_UPDATE pin per prior bring-up)
//     has a native alternate-function mux to QTIMER3_TIMER3
//     (IOMUXC_GPIO_AD_B1_03_QTIMER3_TIMER3, sdk/devices/MIMXRT1062/drivers/fsl_iomuxc.h)
//     -- so TMR3 channel 3, run as an ordinary hardware PWM exactly like
//     machine_pwm.c's QTMR_SetupPwm_u16() path, can generate the I/O_UPDATE latch pulse
//     with zero CPU/DMA involvement, no GPIO-toggle-via-DMA hack needed.
//   - LPSPI's own automatic FIFO-empty DMA request (its DER register) is deliberately
//     NOT used or enabled anywhere in this file -- it would be paced by the SPI baud
//     rate, not by the QTMR-sourced 192kHz trigger, and the two must not both drive the
//     same DMAMUX channel. Writes to TDR here happen unconditionally at the QTMR's pace;
//     the caller is responsible for choosing a baudrate high enough that one frame's
//     clock-out time stays comfortably under one rate_hz period (e.g. an 8-byte AD9910
//     Profile 0 write is 64 bits; at 192kHz that leaves ~5.2us, so a >=20Mbps baudrate
//     is needed for real margin -- NOT YET BENCH-VALIDATED, see the module's own
//     dma_periodic_start() docstring below).
//
// NOT yet verified -- real open items, deliberately left as runtime-computed or
// explicit caller-supplied values rather than guessed constants:
//   - The exact QTMR2 channel-0 tick count for a given rate_hz: computed at
//     spi_dma_periodic_start() time from CLOCK_GetFreq(kCLOCK_IpgClk), mirroring
//     machine_pwm.c's configure_qtmr()/calc_prescaler() exactly, not hardcoded.
//   - I/O_UPDATE pulse width/phase relative to the SPI frame's completion -- exposed as
//     explicit io_update_duty_u16/io_update_phase_ticks arguments rather than a default,
//     because getting this wrong either misses the datasheet's I/O_UPDATE setup/hold
//     window or clips into the next frame; needs ADALM2000 bench measurement.
//   - Whether TMR2 channel 0 and TMR3 channel 3 are free at runtime on top of whatever
//     else the running firmware claims (this file does not check for conflicts with
//     machine.PWM objects a user script may have already created on those same
//     channels -- a real gap, not yet handled).
//   - Multi-byte frame bit-ordering on the wire, end to end. dma_periodic_start() widens
//     LPSPI's TCR.FRAMESZ so hardware holds chip-select low across a whole
//     instruction-byte+payload frame instead of toggling it every byte (see that
//     function's own comment for why this is a read-modify-write with a real
//     precondition, not a from-scratch TCR reconstruction) -- but nobody has yet
//     compared a byte captured off this DMA path against a known-good blocking-mode
//     write of the same register on a logic analyzer. Do not trust this produces a
//     correct AD9910/9952/9954 register write until that comparison has been done.

#define DEFAULT_SPI_ID          (0)
#define DEFAULT_SPI_BAUDRATE    (1000000)
#define DEFAULT_SPI_POLARITY    (0)
#define DEFAULT_SPI_PHASE       (0)
#define DEFAULT_SPI_BITS        (8)
#define DEFAULT_SPI_FIRSTBIT    (kLPSPI_MsbFirst)
#define DEFAULT_SPI_DRIVE       (6)

#define CLOCK_DIVIDER           (1)

#if defined(MIMXRT117x_SERIES)
#define LPSPI_DMAMUX            DMAMUX0
#else
#define LPSPI_DMAMUX            DMAMUX
#endif

#define MICROPY_HW_SPI_NUM MP_ARRAY_SIZE(spi_index_table)

#define SCK (iomux_table[index])
#define CS0 (iomux_table[index + 1])
#define SDO (iomux_table[index + 2])
#define SDI (iomux_table[index + 3])
#define CS1 (iomux_table[index + 4])

// TMR2 channel 0 is dedicated as the periodic-DMA trigger for ALL SPI(id) objects that
// use dma_periodic_start() -- this project only ever needs one such stream at a time
// (the DDS phase path), so no per-instance allocation is implemented. TMR3 channel 3 is
// dedicated as the I/O_UPDATE pulse generator, chosen because it's the only TMR channel
// with a native mux to pin D15 (see the module-level comment above).
#define SPI_DMA_TMR_BASE            TMR2
#define SPI_DMA_TMR_CHANNEL         kQTMR_Channel_0
#define SPI_DMA_TMR_DMAMUX_SRC      kDmaRequestMuxQTIMER2Cmpld1Timer0Cmpld2Timer1
#define SPI_DMA_IO_UPDATE_TMR_BASE  TMR3
#define SPI_DMA_IO_UPDATE_CHANNEL   kQTMR_Channel_3

// Ring buffer size: two equal halves, refilled one half at a time while the DMA engine
// drains the other -- same double-buffering shape as machine_i2s.c's SIZEOF_DMA_BUFFER_
// IN_BYTES/feed_dma() pattern, sized in frames rather than assumed a fixed byte count
// since frame_bytes is caller-specified (5 bytes for an AD9952/9954-style FTW0 write, 8
// for an AD9910 Profile 0 write, etc.).
#define SPI_DMA_FRAMES_PER_HALF (64)

typedef struct _machine_spi_obj_t {
    mp_obj_base_t base;
    uint8_t spi_id;
    uint8_t mode;
    uint8_t spi_hw_id;
    bool transfer_busy;
    LPSPI_Type *spi_inst;
    lpspi_master_config_t *master_config;

    // --- periodic DMA extension state; all zero/NULL unless dma_periodic_start() has
    // been called on this object. See the module-level comment block above.
    bool dma_periodic_active;
    size_t dma_frame_bytes;
    uint8_t *dma_buffer;                 // 2 * SPI_DMA_FRAMES_PER_HALF * dma_frame_bytes,
                                          // allocated at dma_periodic_start() time
    uint8_t *dma_buffer_dcache_aligned;
    int dma_channel;
    edma_handle_t dma_edmaHandle;
    edma_tcd_t *dma_edmaTcd;
    mp_obj_t dma_refill_callback;         // called via mp_sched_schedule(cb, half_index)
} machine_spi_obj_t;

typedef struct _iomux_table_t {
    uint32_t muxRegister;
    uint32_t muxMode;
    uint32_t inputRegister;
    uint32_t inputDaisy;
    uint32_t configRegister;
} iomux_table_t;

static const uint8_t spi_index_table[] = MICROPY_HW_SPI_INDEX;
static LPSPI_Type *spi_base_ptr_table[] = LPSPI_BASE_PTRS;
static const iomux_table_t iomux_table[] = {
    IOMUX_TABLE_SPI
};

bool lpspi_set_iomux(int8_t spi, uint8_t drive, int8_t cs) {
    int index = (spi - 1) * 5;

    if (SCK.muxRegister != 0) {
        IOMUXC_SetPinMux(SCK.muxRegister, SCK.muxMode, SCK.inputRegister, SCK.inputDaisy, SCK.configRegister, 0U);
        IOMUXC_SetPinConfig(SCK.muxRegister, SCK.muxMode, SCK.inputRegister, SCK.inputDaisy, SCK.configRegister,
            pin_generate_config(PIN_PULL_UP_100K, PIN_MODE_OUT, drive, SCK.configRegister));

        if (cs == 0 && CS0.muxRegister != 0) {
            IOMUXC_SetPinMux(CS0.muxRegister, CS0.muxMode, CS0.inputRegister, CS0.inputDaisy, CS0.configRegister, 0U);
            IOMUXC_SetPinConfig(CS0.muxRegister, CS0.muxMode, CS0.inputRegister, CS0.inputDaisy, CS0.configRegister,
                pin_generate_config(PIN_PULL_UP_100K, PIN_MODE_OUT, drive, CS0.configRegister));
        } else if (cs == 1 && CS1.muxRegister != 0) {
            IOMUXC_SetPinMux(CS1.muxRegister, CS1.muxMode, CS1.inputRegister, CS1.inputDaisy, CS1.configRegister, 0U);
            IOMUXC_SetPinConfig(CS1.muxRegister, CS1.muxMode, CS1.inputRegister, CS1.inputDaisy, CS1.configRegister,
                pin_generate_config(PIN_PULL_UP_100K, PIN_MODE_OUT, drive, CS1.configRegister));
        } else if (cs != -1) {
            mp_raise_ValueError(MP_ERROR_TEXT("The chosen CS is not available"));
        }

        IOMUXC_SetPinMux(SDO.muxRegister, SDO.muxMode, SDO.inputRegister, SDO.inputDaisy, SDO.configRegister, 0U);
        IOMUXC_SetPinConfig(SDO.muxRegister, SDO.muxMode, SDO.inputRegister, SDO.inputDaisy, SDO.configRegister,
            pin_generate_config(PIN_PULL_UP_100K, PIN_MODE_OUT, drive, SDO.configRegister));

        IOMUXC_SetPinMux(SDI.muxRegister, SDI.muxMode, SDI.inputRegister, SDI.inputDaisy, SDI.configRegister, 0U);
        IOMUXC_SetPinConfig(SDI.muxRegister, SDI.muxMode, SDI.inputRegister, SDI.inputDaisy, SDI.configRegister,
            pin_generate_config(PIN_PULL_UP_100K, PIN_MODE_IN, drive, SDI.configRegister));

        return true;
    } else {
        return false;
    }
}

static void machine_spi_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    static const char *firstbit_str[] = {"MSB", "LSB"};
    machine_spi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "SPI(%u, baudrate=%u, polarity=%u, phase=%u, bits=%u, firstbit=%s, gap_ns=%d)",
        self->spi_id, self->master_config->baudRate, self->master_config->cpol,
        self->master_config->cpha, self->master_config->bitsPerFrame,
        firstbit_str[self->master_config->direction], self->master_config->betweenTransferDelayInNanoSec);
}

mp_obj_t machine_spi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_id, ARG_baudrate, ARG_polarity, ARG_phase, ARG_bits, ARG_firstbit, ARG_gap_ns, ARG_drive, ARG_cs };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id,       MP_ARG_INT, {.u_int = DEFAULT_SPI_ID} },
        { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = DEFAULT_SPI_BAUDRATE} },
        { MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_SPI_POLARITY} },
        { MP_QSTR_phase,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_SPI_PHASE} },
        { MP_QSTR_bits,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_SPI_BITS} },
        { MP_QSTR_firstbit, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_SPI_FIRSTBIT} },
        { MP_QSTR_gap_ns,   MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_drive,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_SPI_DRIVE} },
        { MP_QSTR_cs,       MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    // Parse the arguments.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Get the SPI bus id.
    int spi_id = args[ARG_id].u_int;
    if (spi_id < 0 || spi_id >= MP_ARRAY_SIZE(spi_index_table) || spi_index_table[spi_id] == 0) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("SPI(%d) doesn't exist"), spi_id);
    }

    // Get peripheral object.
    uint8_t spi_hw_id = spi_index_table[spi_id];  // the hw spi number 1..n
    machine_spi_obj_t *self = mp_obj_malloc(machine_spi_obj_t, &machine_spi_type);
    self->spi_id = spi_id;
    self->spi_inst = spi_base_ptr_table[spi_hw_id];
    self->spi_hw_id = spi_hw_id;
    self->dma_periodic_active = false;
    self->dma_buffer = NULL;
    self->dma_refill_callback = MP_OBJ_NULL;

    uint8_t drive = args[ARG_drive].u_int;
    if (drive < 1 || drive > 7) {
        drive = DEFAULT_SPI_DRIVE;
    }

    LPSPI_Reset(self->spi_inst);
    LPSPI_Enable(self->spi_inst, false);  // Disable first before new settings are applies

    self->master_config = m_new_obj(lpspi_master_config_t);
    LPSPI_MasterGetDefaultConfig(self->master_config);
    // Initialise the SPI peripheral.
    self->master_config->baudRate = args[ARG_baudrate].u_int;
    self->master_config->betweenTransferDelayInNanoSec = 1000000000 / self->master_config->baudRate * 2;
    self->master_config->cpol = args[ARG_polarity].u_int;
    self->master_config->cpha = args[ARG_phase].u_int;
    self->master_config->bitsPerFrame = args[ARG_bits].u_int;
    self->master_config->direction = args[ARG_firstbit].u_int;
    if (args[ARG_gap_ns].u_int != -1) {
        self->master_config->betweenTransferDelayInNanoSec = args[ARG_gap_ns].u_int;
    }
    self->master_config->lastSckToPcsDelayInNanoSec = self->master_config->betweenTransferDelayInNanoSec;
    self->master_config->pcsToSckDelayInNanoSec = self->master_config->betweenTransferDelayInNanoSec;
    int8_t cs = args[ARG_cs].u_int;
    // In the SPI master_config for automatic CS the value cs=0 is set already,
    // so only cs=1 has to be addressed here. The case cs == -1 for manual CS is handled
    // in the function spi_set_iomux() and the value in the master_config can stay at 0.
    if (cs == 1) {
        self->master_config->whichPcs = cs;
    }
    LPSPI_MasterInit(self->spi_inst, self->master_config, BOARD_BOOTCLOCKRUN_LPSPI_CLK_ROOT);
    lpspi_set_iomux(spi_index_table[spi_id], drive, cs);

    return MP_OBJ_FROM_PTR(self);
}

static void machine_spi_init(mp_obj_base_t *self_in, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_baudrate, ARG_polarity, ARG_phase, ARG_bits, ARG_firstbit, ARG_gap_ns };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_baudrate, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_phase,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_bits,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_firstbit, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_gap_ns,   MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    // Parse the arguments.
    machine_spi_obj_t *self = (machine_spi_obj_t *)self_in;
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Reconfigure the baudrate if requested.
    if (args[ARG_baudrate].u_int != -1) {
        self->master_config->baudRate = args[ARG_baudrate].u_int;
        self->master_config->betweenTransferDelayInNanoSec = 1000000000 / self->master_config->baudRate * 2;
    }
    // Reconfigure the format if requested.
    if (args[ARG_polarity].u_int != -1) {
        self->master_config->cpol = args[ARG_polarity].u_int;
    }
    if (args[ARG_phase].u_int != -1) {
        self->master_config->cpha = args[ARG_phase].u_int;
    }
    if (args[ARG_bits].u_int != -1) {
        self->master_config->bitsPerFrame = args[ARG_bits].u_int;
    }
    if (args[ARG_firstbit].u_int != -1) {
        self->master_config->direction = args[ARG_firstbit].u_int;
    }
    if (args[ARG_gap_ns].u_int != -1) {
        self->master_config->betweenTransferDelayInNanoSec = args[ARG_gap_ns].u_int;
    }
    self->master_config->lastSckToPcsDelayInNanoSec = self->master_config->betweenTransferDelayInNanoSec;
    self->master_config->pcsToSckDelayInNanoSec = self->master_config->betweenTransferDelayInNanoSec;
    LPSPI_Enable(self->spi_inst, false);  // Disable first before new settings are applies
    LPSPI_MasterInit(self->spi_inst, self->master_config, BOARD_BOOTCLOCKRUN_LPSPI_CLK_ROOT);
}

static void machine_spi_transfer(mp_obj_base_t *self_in, size_t len, const uint8_t *src, uint8_t *dest) {
    machine_spi_obj_t *self = (machine_spi_obj_t *)self_in;

    if (len > 0) {
        // Wait a short while for the previous transfer to finish, but not forever
        for (volatile int j = 0; (j < 5000) && ((self->spi_inst->SR & kLPSPI_ModuleBusyFlag) != 0); j++) {}

        lpspi_transfer_t masterXfer;
        masterXfer.txData = (uint8_t *)src;
        masterXfer.rxData = (uint8_t *)dest;
        masterXfer.dataSize = len;
        masterXfer.configFlags = (self->master_config->whichPcs << LPSPI_MASTER_PCS_SHIFT) | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;

        if (LPSPI_MasterTransferBlocking(self->spi_inst, &masterXfer) != kStatus_Success) {
            mp_raise_OSError(EIO);
        }
    }
}

// Forward declaration -- defined further down with the rest of the periodic-DMA
// implementation, but must be wired into machine_spi_p (right below) so a plain
// spi.deinit() (or garbage collection reclaiming an abandoned SPI object -- a real
// scenario given this project's habit of re-running bench scripts repeatedly in the
// same mpremote/REPL session) actually releases the DMA channel and stops both QTMR
// channels, instead of leaking them silently. Before this, machine_spi_p had no .deinit
// at all (generic machine_spi_deinit() in extmod/machine_spi.c already no-ops when it's
// NULL) -- a pre-existing gap this patch also closes as a side effect.
static void machine_spi_deinit_hw(mp_obj_base_t *self_in);

static const mp_machine_spi_p_t machine_spi_p = {
    .init = machine_spi_init,
    .deinit = machine_spi_deinit_hw,
    .transfer = machine_spi_transfer,
};

// --- Periodic DMA transfer implementation ---
// See the module-level comment block near the top of this file for the verified
// hardware facts this depends on, and what's still open.

// TCD storage for the periodic-DMA channel. Only one SPI(id) can run dma_periodic_start()
// at a time in this implementation (see SPI_DMA_TMR_BASE's own comment), so a single
// static TCD suffices -- mirrors machine_i2s.c's AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t
// edmaTcd[I2S_NUM_OBJ_SLOTS], 32) pattern, sized 1 instead of I2S_NUM_OBJ_SLOTS.
AT_NONCACHEABLE_SECTION_ALIGN(static edma_tcd_t spi_dma_tcd, 32);

static void edma_spi_periodic_callback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds) {
    machine_spi_obj_t *self = (machine_spi_obj_t *)userData;
    if (self->dma_refill_callback != MP_OBJ_NULL && self->dma_refill_callback != mp_const_none) {
        // transferDone == true means the eDMA major loop just wrapped, i.e. the bottom
        // half was the one just drained (mirrors edma_i2s_callback's own TOP_HALF/
        // BOTTOM_HALF convention in machine_i2s.c) -- pass which half needs refilling,
        // not which half just played, so the Python-side callback can call
        // dma_periodic_write() directly on the right half without re-deriving this.
        mp_sched_schedule(self->dma_refill_callback,
            MP_OBJ_NEW_SMALL_INT(transferDone ? 0 : 1));
    }
}

// Same clock-query + prescale-search recipe as machine_pwm.c's configure_qtmr()/
// calc_prescaler() (CLOCK_GetFreq(kCLOCK_IpgClk), not a hardcoded frequency) -- see the
// module comment's note on why this must be computed, not assumed.
static int spi_dma_calc_qtmr_prescale(uint32_t clock_hz, uint32_t rate_hz, uint16_t *ticks_out) {
    for (int prescale = 0; prescale < 8; prescale++) {
        uint32_t divided = clock_hz >> prescale;
        uint32_t ticks = divided / rate_hz;
        if (ticks <= 0xFFFF && ticks > 0) {
            *ticks_out = (uint16_t)ticks;
            return prescale;
        }
    }
    return -1; // rate_hz too low to represent even at maximum prescale
}

// spi.dma_periodic_start(frame_bytes, rate_hz, refill_callback, io_update_duty_u16=0,
//                         io_update_phase_ticks=0)
//
// Starts a QTMR2/eDMA-driven periodic transfer of `frame_bytes`-sized frames from an
// internal double-buffer into this SPI object's LPSPI TDR register at `rate_hz`, and a
// TMR3-channel-3 hardware PWM pulse on pin D15 (I/O_UPDATE) at the same rate.
// `refill_callback(half)` is scheduled (via mp_sched_schedule, i.e. run on the main
// thread, not from IRQ context) each time a half-buffer has just been drained and needs
// fresh frames written into it via dma_periodic_write(half, buf).
//
// `io_update_duty_u16`/`io_update_phase_ticks` are NOT yet bench-validated -- see the
// module comment's open-items list. Passing 0 for both currently means "no I/O_UPDATE
// pulse is generated," i.e. this call alone is NOT sufficient for a working DDS update
// path yet; it must be paired with a bench session to find real values for these before
// the AD9910 will actually latch anything written here. Left as explicit, required
// arguments rather than a guessed default specifically so that gap is visible at every
// call site, not hidden behind a silently-wrong default.
static mp_obj_t machine_spi_dma_periodic_start(size_t n_args, const mp_obj_t *args) {
    machine_spi_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t frame_bytes = mp_obj_get_int(args[1]);
    mp_int_t rate_hz = mp_obj_get_int(args[2]);
    mp_obj_t refill_callback = args[3];
    mp_int_t io_update_duty_u16 = (n_args > 4) ? mp_obj_get_int(args[4]) : 0;
    mp_int_t io_update_phase_ticks = (n_args > 5) ? mp_obj_get_int(args[5]) : 0;
    (void)io_update_phase_ticks;  // accepted but not yet wired to anything -- see the
                                  // module comment's open-items list on I/O_UPDATE phase

    if (self->dma_periodic_active) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("dma_periodic already active"));
    }
    if (frame_bytes <= 0 || frame_bytes > 16) {
        mp_raise_ValueError(MP_ERROR_TEXT("frame_bytes out of range"));
    }

    self->dma_frame_bytes = (size_t)frame_bytes;
    size_t half_bytes = SPI_DMA_FRAMES_PER_HALF * self->dma_frame_bytes;
    size_t total_bytes = 2 * half_bytes;

    // +0x1f for D-cache-line alignment slack, same as machine_i2s.c's dma_buffer field;
    // AT_NONCACHEABLE_SECTION_ALIGN is not used here because size is only known at this
    // point (frame_bytes is caller-specified), unlike the fixed-size TCD above -- this
    // buffer instead needs an explicit cache-maintenance call before each DMA-visible
    // write. TODO(bench): this implementation does NOT yet call DCACHE_CleanByRange()
    // anywhere -- machine_i2s.c's own dma_buffer apparently avoids needing this via
    // linker placement, not a runtime call (see this file's module comment); this heap-
    // allocated buffer has NOT been confirmed to get the same treatment and may need an
    // explicit clean call added in dma_periodic_write() below before it can be trusted.
    self->dma_buffer = m_new(uint8_t, total_bytes + 0x1f);
    self->dma_buffer_dcache_aligned = (uint8_t *)(((uint32_t)self->dma_buffer + 0x1f) & ~0x1f);
    memset(self->dma_buffer_dcache_aligned, 0, total_bytes);

    self->dma_refill_callback = refill_callback;

    // --- QTMR2 channel 0: free-running internal-clock periodic trigger, no pin output.
    // Identical setup recipe to machine_pwm.c's configure_qtmr(), minus QTMR_SetupPwm().
    uint32_t ipg_clk_hz = CLOCK_GetFreq(kCLOCK_IpgClk);
    uint16_t trigger_ticks;
    int prescale = spi_dma_calc_qtmr_prescale(ipg_clk_hz, (uint32_t)rate_hz, &trigger_ticks);
    if (prescale < 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("rate_hz too low to represent"));
    }
    qtmr_config_t trigger_config;
    QTMR_GetDefaultConfig(&trigger_config);
    trigger_config.primarySource = (qtmr_primary_count_source_t)(prescale + kQTMR_ClockDivide_1);
    QTMR_Init(SPI_DMA_TMR_BASE, SPI_DMA_TMR_CHANNEL, &trigger_config);
    QTMR_SetTimerPeriod(SPI_DMA_TMR_BASE, SPI_DMA_TMR_CHANNEL, trigger_ticks);
    QTMR_EnableDma(SPI_DMA_TMR_BASE, SPI_DMA_TMR_CHANNEL, kQTMR_ComparatorPreload1DmaEnable);

    // --- TMR3 channel 3 / pin D15: I/O_UPDATE pulse, ordinary hardware PWM.
    // TODO(bench): io_update_duty_u16/io_update_phase_ticks are passed through
    // uninterpreted below beyond what QTMR_SetupPwm_u16 itself does with a duty value --
    // the phase relationship to the SPI-triggering TMR2 channel is NOT yet established
    // by this code (both channels are simply started independently below; nothing here
    // guarantees a specific relative phase). This is the single biggest unproven piece
    // of this patch -- do not trust I/O_UPDATE timing without an ADALM2000 capture.
    if (io_update_duty_u16 > 0) {
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_QTIMER3_TIMER3, 0U);
        qtmr_config_t io_update_config;
        QTMR_GetDefaultConfig(&io_update_config);
        io_update_config.primarySource = (qtmr_primary_count_source_t)(prescale + kQTMR_ClockDivide_1);
        QTMR_Init(SPI_DMA_IO_UPDATE_TMR_BASE, SPI_DMA_IO_UPDATE_CHANNEL, &io_update_config);
        QTMR_SetupPwm_u16(SPI_DMA_IO_UPDATE_TMR_BASE, SPI_DMA_IO_UPDATE_CHANNEL,
            (uint32_t)rate_hz, (uint16_t)io_update_duty_u16, false,
            ipg_clk_hz / (1u << prescale), false);
        QTMR_StartTimer(SPI_DMA_IO_UPDATE_TMR_BASE, SPI_DMA_IO_UPDATE_CHANNEL, kQTMR_PriSrcRiseEdge);
    }

    // --- LPSPI frame-size fixup: a single AD9910/9952/9954 register write is one
    // instruction byte + N payload bytes inside ONE continuous chip-select-low window
    // (the datasheet's own serial-port timing diagram), not N+1 separate one-byte
    // transactions. Left at whatever FRAMESZ a normal machine.SPI.write() call last
    // configured (LPSPI defaults effectively to one CS pulse per 8 bits), the DMA path
    // below would toggle PCS after every single byte instead of once per whole frame --
    // almost certainly misinterpreted by the DDS as N+1 separate short writes rather
    // than one real register write. Fixed by widening just the FRAMESZ field so hardware
    // holds PCS low across the whole frame_bytes*8-bit frame, auto-toggling it only
    // between frames.
    //
    // Deliberately a read-modify-write of TCR's existing value, not a from-scratch
    // reconstruction: TCR also carries CPOL/CPHA/PRESCALE/PCS-select/byte-swap, and
    // LPSPI_MasterInit() does not appear to fully populate TCR itself (those fields seem
    // to only get set for real when a transfer actually runs) -- so this call has a real
    // PRECONDITION: at least one ordinary spi.write()/write_readinto() must already have
    // succeeded on this object before dma_periodic_start() is called, so TCR already
    // holds this project's correct working CPOL/CPHA/byte-order configuration from that
    // call, and this code only widens FRAMESZ on top of it rather than guessing those
    // other fields from scratch (guessing PRESCALE's encoding or BYSW independently was
    // judged a worse risk than requiring this precondition). NOT bench-verified that this
    // read-modify-write approach actually produces correct multi-byte bit ordering on the
    // wire -- see the module comment's open-items list; capture with the ADALM2000 and
    // compare against a known-good blocking-mode write of the same register before
    // trusting this.
    uint32_t frame_bits = (uint32_t)self->dma_frame_bytes * 8;
    self->spi_inst->TCR = (self->spi_inst->TCR & ~LPSPI_TCR_FRAMESZ_MASK) | LPSPI_TCR_FRAMESZ(frame_bits - 1);

    // --- eDMA: QTMR2-triggered, ring buffer -> LPSPI TDR, self-linked TCD for
    // continuous circular operation (same construct machine_i2s.c uses).
    self->dma_channel = allocate_dma_channel();
    if (self->dma_channel < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("no DMA channel available"));
    }
    DMAMUX_Init(DMAMUX);
    DMAMUX_SetSource(DMAMUX, self->dma_channel, SPI_DMA_TMR_DMAMUX_SRC);
    DMAMUX_EnableChannel(DMAMUX, self->dma_channel);

    // Idempotent (guarded internally) -- must still be called here rather than assumed
    // already done by some other peripheral's constructor (e.g. machine_i2s.c's own
    // i2s_init() also calls this), since this SPI object may be the first DMA user.
    dma_init();
    EDMA_CreateHandle(&self->dma_edmaHandle, DMA0, self->dma_channel);
    EDMA_SetCallback(&self->dma_edmaHandle, edma_spi_periodic_callback, self);
    EDMA_ResetChannel(DMA0, self->dma_channel);
    self->dma_edmaTcd = &spi_dma_tcd;

    // Per-beat transfer width MUST be a real eDMA SSIZE/DSIZE encoding (1/2/4/8/16/32
    // bytes) -- dma_frame_bytes (e.g. 9 for an AD9910 write) is NOT a valid width and an
    // earlier version of this patch wrongly passed it as one. Fixed: beats are always
    // 1 byte wide; dma_frame_bytes instead sets the MINOR LOOP byte count (how many
    // 1-byte beats fire back-to-back per single QTMR-sourced DMA request) -- this is the
    // standard eDMA minor-loop/major-loop split, not a workaround.
    edma_transfer_config_t transferConfig;
    EDMA_PrepareTransfer(&transferConfig,
        self->dma_buffer_dcache_aligned, 1,
        (void *)&self->spi_inst->TDR, 1,
        (uint32_t)self->dma_frame_bytes,     // minor loop: one whole frame per QTMR tick
        (uint32_t)total_bytes,               // major loop: whole ring buffer
        kEDMA_MemoryToPeripheral);
    memset(self->dma_edmaTcd, 0, sizeof(edma_tcd_t));
    EDMA_TcdSetTransferConfig(self->dma_edmaTcd, &transferConfig, self->dma_edmaTcd);
    EDMA_TcdEnableInterrupts(self->dma_edmaTcd, kEDMA_MajorInterruptEnable | kEDMA_HalfInterruptEnable);
    EDMA_InstallTCD(DMA0, self->dma_channel, self->dma_edmaTcd);
    EDMA_StartTransfer(&self->dma_edmaHandle);

    QTMR_StartTimer(SPI_DMA_TMR_BASE, SPI_DMA_TMR_CHANNEL, kQTMR_PriSrcRiseEdge);

    self->dma_periodic_active = true;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_spi_dma_periodic_start_obj, 4, 6, machine_spi_dma_periodic_start);

// spi.dma_periodic_write(half, buf) -- copy buf (must be exactly SPI_DMA_FRAMES_PER_HALF
// * frame_bytes long) into the specified half (0=bottom, 1=top) of the internal ring
// buffer. Intended to be called from the refill_callback passed to
// dma_periodic_start(), using the `half` value that callback was invoked with.
static mp_obj_t machine_spi_dma_periodic_write(mp_obj_t self_in, mp_obj_t half_in, mp_obj_t buf_in) {
    machine_spi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t half = mp_obj_get_int(half_in);
    if (!self->dma_periodic_active) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("dma_periodic not active"));
    }
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_READ);
    size_t half_bytes = SPI_DMA_FRAMES_PER_HALF * self->dma_frame_bytes;
    if (buf.len != half_bytes) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer wrong length for one half"));
    }
    uint8_t *dest = self->dma_buffer_dcache_aligned + (half ? half_bytes : 0);
    memcpy(dest, buf.buf, half_bytes);
    // TODO(bench): no DCACHE_CleanByRange() call here -- see the "not yet call..." TODO
    // in dma_periodic_start() above. If DMA reads stale data, this is the first place to
    // add one.
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(machine_spi_dma_periodic_write_obj, machine_spi_dma_periodic_write);

static mp_obj_t machine_spi_dma_periodic_stop(mp_obj_t self_in) {
    machine_spi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (!self->dma_periodic_active) {
        return mp_const_none;
    }
    EDMA_AbortTransfer(&self->dma_edmaHandle);
    free_dma_channel(self->dma_channel);
    QTMR_StopTimer(SPI_DMA_TMR_BASE, SPI_DMA_TMR_CHANNEL);
    QTMR_StopTimer(SPI_DMA_IO_UPDATE_TMR_BASE, SPI_DMA_IO_UPDATE_CHANNEL);
    self->dma_periodic_active = false;
    self->dma_refill_callback = MP_OBJ_NULL;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(machine_spi_dma_periodic_stop_obj, machine_spi_dma_periodic_stop);

static void machine_spi_deinit_hw(mp_obj_base_t *self_in) {
    machine_spi_dma_periodic_stop(MP_OBJ_FROM_PTR(self_in));
}

// --- Port-specific locals dict: the generic mp_machine_spi_locals_dict (extmod/
// machine_spi.c) has no way to carry these mimxrt-only additions, so this file defines
// its own dict combining the generic entries with the three new methods above, and
// points machine_spi_type at THIS dict instead. The extern declarations below assume
// external linkage for the generic *_obj symbols -- consistent with how
// MP_DEFINE_CONST_FUN_OBJ_* is used elsewhere in this codebase, but NOT bench/build-
// verified for this exact case; a build failure here would mean extmod/machine_spi.c
// needs those symbols marked non-static (a one-line, upstream-friendly fix) rather than
// anything wrong with this file's own logic.
// mp_machine_spi_{read,readinto,write,write_readinto}_obj are already declared (with the
// correct types -- read/readinto as mp_obj_fun_builtin_var_t, write/write_readinto as
// mp_obj_fun_builtin_fixed_t) by extmod/modmachine.h, already transitively included via
// py/runtime.h above -- redeclaring them here with guessed types was a real build error
// (conflicting types for mp_machine_spi_write_obj), not just redundant. Only
// machine_spi_init_obj/machine_spi_deinit_obj need a fresh extern here: modmachine.h
// doesn't declare them (they were static in extmod/machine_spi.c until this patch).
extern const mp_obj_fun_builtin_var_t machine_spi_init_obj;    // MP_DEFINE_CONST_FUN_OBJ_KW
extern const mp_obj_fun_builtin_fixed_t machine_spi_deinit_obj; // MP_DEFINE_CONST_FUN_OBJ_1

static const mp_rom_map_elem_t machine_spi_mimxrt_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&machine_spi_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&machine_spi_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_machine_spi_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_machine_spi_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_machine_spi_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_readinto), MP_ROM_PTR(&mp_machine_spi_write_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_MSB), MP_ROM_INT(MICROPY_PY_MACHINE_SPI_MSB) },
    { MP_ROM_QSTR(MP_QSTR_LSB), MP_ROM_INT(MICROPY_PY_MACHINE_SPI_LSB) },
    { MP_ROM_QSTR(MP_QSTR_dma_periodic_start), MP_ROM_PTR(&machine_spi_dma_periodic_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_dma_periodic_write), MP_ROM_PTR(&machine_spi_dma_periodic_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_dma_periodic_stop), MP_ROM_PTR(&machine_spi_dma_periodic_stop_obj) },
};
MP_DEFINE_CONST_DICT(mp_machine_spi_mimxrt_locals_dict, machine_spi_mimxrt_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_spi_type,
    MP_QSTR_SPI,
    MP_TYPE_FLAG_NONE,
    make_new, machine_spi_make_new,
    print, machine_spi_print,
    protocol, &machine_spi_p,
    locals_dict, &mp_machine_spi_mimxrt_locals_dict
    );
