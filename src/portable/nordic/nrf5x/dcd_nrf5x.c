/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if TUSB_OPT_DEVICE_ENABLED && CFG_TUSB_MCU == OPT_MCU_NRF5X

#include "nrf.h"
#include "nrf_clock.h"
#include "nrf_power.h"
#include "nrfx_usbd_errata.h"
#include "device/dcd.h"

#include "device/usbd.h"

#ifndef CFG_TUD_NRF5X_ISO
#define CFG_TUD_NRF5X_ISO 1
#endif

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/
enum
{
  // Max allowed by USB specs
  MAX_PACKET_SIZE   = 64,

  // Mask of all END event (IN & OUT) for all endpoints. ENDEPIN0-7, ENDEPOUT0-7, ENDISOIN, ENDISOOUT
  EDPT_END_ALL_MASK = (0xff << USBD_INTEN_ENDEPIN0_Pos) | (0xff << USBD_INTEN_ENDEPOUT0_Pos) |
                      ((CFG_TUD_NRF5X_ISO) ? (USBD_INTENCLR_ENDISOIN_Msk | USBD_INTEN_ENDISOOUT_Msk) : 0)
};

enum
{
  EP_ISO_NUM   = 8, // Endpoint number is fixed (8) for ISOOUT and ISOIN
  EP_CBI_COUNT = 8  // Control Bulk Interrupt endpoints count
};

enum {
  DMA_EP0_STATUS = 0,
  DMA_EP0_RCVOUT = 1,
  DMA_IN_BASE    = USBD_INTEN_ENDEPIN0_Pos,
  DMA_OUT_BASE   = USBD_INTEN_ENDEPOUT0_Pos,
  OUT_HALTED     = 2,
};
#define DMA_EP0_MASK (TU_BIT(DMA_IN_BASE) | TU_BIT(DMA_OUT_BASE) | 3u)
#define DMA_ALL_MASK (EDPT_END_ALL_MASK | 3u)
// Transfer Descriptor
typedef struct
{
  uint8_t* buffer;
  uint16_t total_len;
  volatile uint16_t actual_len;
  uint16_t mps; // max packet size

  // nRF will auto accept OUT packet after DMA is done
  // indicate packet is already ACK
  volatile uint8_t data_received; // 0: empty, 1: received, 2: halted

  // Set to true when data was transferred from RAM to ISO IN output buffer.
  // New data can be put in ISO IN output buffer after SOF.
  bool iso_in_transfer_ready;

} xfer_td_t;

// Data for managing dcd
static struct
{
  // All 8 endpoints including control IN & OUT (offset 1)
  // +1 for ISO endpoints
  xfer_td_t xfer[EP_CBI_COUNT + CFG_TUD_NRF5X_ISO][2];

  // nRF can only carry one DMA at a time, this is used to guard the access to EasyDMA
  volatile uint32_t dma_active; // one-hot END event of the DMA that owns the channel
  volatile uint8_t setup_dir; // captured with SETUP; its registers are unreadable during an OUT DMA
  uint32_t dma_pending;
}_dcd;

// Keep generations across bus_reset() clearing the transfer descriptors.
static struct {
  volatile uint8_t received;
  uint8_t          served;
  uint8_t          status_ep;
} _ep0;


static bool ep0_superseded(void) {
  return _ep0.received != _ep0.served || NRF_USBD->EVENTS_EP0SETUP;
}

void tusb_hal_nrf_power_event(uint32_t event);

// Nordic anomaly 199, matching nrfx's usbd_dma_pending_set/clear(). Only real
// EasyDMA transfers use the latch; EP0STATUS and EP0RCVOUT do not move data.
#define NRF_USBD_ERRATA_199_REG (*((volatile uint32_t *)0x40027C1CUL))

// The IRQ consumes events; a bounded DMA wait only observes them so the normal
// completion handler retains ownership of their transfer accounting.
static uint32_t usbd_events(uint32_t mask, bool consume) {
  volatile uint32_t *events = &NRF_USBD->EVENTS_USBRESET;
  uint32_t           status = 0;
  for (uint8_t i = 0; mask; i++, mask >>= 1) {
    if ((mask & 1u) && events[i]) {
      status |= TU_BIT(i);
      if (consume) {
        events[i] = 0;
        __ISB();
        __DSB();
      }
    }
  }
  return status;
}

static bool dma_wait_end(uint32_t captured) {
  const uint32_t active = _dcd.dma_active;
  if (!active || (captured & active)) {
    return true;
  }
  for (uint8_t scans = 0; scans < 21; scans++) {
    // Scan the full window so the bounded wait does not shrink for low EPs.
    if (!_dcd.dma_active || (usbd_events(EDPT_END_ALL_MASK | USBD_INTEN_ENDISOOUT_Msk, false) & active)) {
      return true;
    }
  }
  return false;
}
/*------------------------------------------------------------------*/
/* Control / Bulk / Interrupt (CBI) Transfer
 *------------------------------------------------------------------*/

// NVIC_GetEnableIRQ is only available in CMSIS v5
#ifndef NVIC_GetEnableIRQ
static inline uint32_t NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
#endif

// check if we are in ISR
TU_ATTR_ALWAYS_INLINE static inline bool is_in_isr(void)
{
  return __get_IPSR() != 0;
}

static __attribute__((noinline)) void dma_errata_199(uint32_t value) {
  if (nrfx_usbd_errata_199()) {
    NRF_USBD_ERRATA_199_REG = value;
  }
}

static void edpt_dma_end(void);

// helper to start DMA
// Caller holds the short global task-commit lock; no polling occurs here.
static void start_dma(volatile uint32_t *reg_startep) {
  *reg_startep = 1;
  __ISB();
  __DSB();
}

// DMA is complete
static void edpt_dma_end(void) {
  dma_errata_199(0);
  _dcd.dma_active = 0;
}

// helper getting td
static inline xfer_td_t *get_td(uint8_t epnum, uint8_t dir) {
  return &_dcd.xfer[epnum][dir];
}

// Serialize the current descriptor only when it owns the free DMA channel.
static void dma_start_transfer(uint8_t epnum, uint8_t dir) {
  xfer_td_t         *xfer = get_td(epnum, dir);
  volatile uint32_t *regs;
  volatile uint32_t *task;
  uint32_t           limit;
  if (dir == TUSB_DIR_OUT) {
#if CFG_TUD_NRF5X_ISO
    if (epnum == EP_ISO_NUM) {
      limit = NRF_USBD->SIZE.ISOOUT;
      if (limit & USBD_SIZE_ISOOUT_ZERO_Msk) {
        _dcd.dma_active = 0;
        return;
      }
      regs = &NRF_USBD->ISOOUT.PTR;
      task = &NRF_USBD->TASKS_STARTISOOUT;
    } else
#endif
    {
      limit = NRF_USBD->SIZE.EPOUT[epnum];
      regs  = &NRF_USBD->EPOUT[epnum].PTR;
      task  = &NRF_USBD->TASKS_STARTEPOUT[epnum];
    }
  } else {
    limit = xfer->mps;
    regs  = (CFG_TUD_NRF5X_ISO && epnum == EP_ISO_NUM) ? &NRF_USBD->ISOIN.PTR : &NRF_USBD->EPIN[epnum].PTR;
    task = (CFG_TUD_NRF5X_ISO && epnum == EP_ISO_NUM) ? &NRF_USBD->TASKS_STARTISOIN : &NRF_USBD->TASKS_STARTEPIN[epnum];
  }
  regs[0] = (uint32_t)xfer->buffer;
  regs[1] = tu_min16((uint16_t)limit, xfer->total_len - xfer->actual_len);
  dma_errata_199(0x82);
  start_dma(task);
}

// Pending operations refer to the current descriptor, never a queued buffer or
// register snapshot. Cancelled work therefore cannot revive after clear-halt.
static void dma_dispatch(void) {
  for (;;) {
    // Power removal can preempt USBD. Snapshot and commit one operation with
    // all interrupts masked, then restore them before another operation.
    // There are no DMA waits or host callbacks inside this short window.
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    uint32_t pending = _dcd.dma_pending;
    if ((pending & DMA_EP0_MASK) && ep0_superseded()) {
      pending &= ~DMA_EP0_MASK;
      _dcd.dma_pending = pending;
    }
    if (!NRF_USBD->ENABLE || _dcd.dma_active || !pending) {
      __set_PRIMASK(saved);
      return;
    }
    const uint8_t  operation = __builtin_ctz(pending);
    const uint32_t bit       = TU_BIT(operation);
    _dcd.dma_pending         = pending & ~bit;
    if (operation == DMA_EP0_STATUS) {
      start_dma(&NRF_USBD->TASKS_EP0STATUS);
      // Queue only after the hardware task, before a newer SETUP can be queued.
      dcd_event_xfer_complete(0, _ep0.status_ep, 0, XFER_RESULT_SUCCESS, is_in_isr());
    } else if (operation == DMA_EP0_RCVOUT) {
      start_dma(&NRF_USBD->TASKS_EP0RCVOUT);
    } else {
      const uint8_t dir   = operation >= DMA_OUT_BASE ? TUSB_DIR_OUT : TUSB_DIR_IN;
      const uint8_t epnum = operation - (dir == TUSB_DIR_OUT ? DMA_OUT_BASE : DMA_IN_BASE) -
                            (CFG_TUD_NRF5X_ISO && operation == USBD_INTEN_ENDISOIN_Pos);
      if (dir == TUSB_DIR_IN || get_td(epnum, dir)->data_received != OUT_HALTED) {
        _dcd.dma_active = bit;
        dma_start_transfer(epnum, dir);
      }
    }
    __set_PRIMASK(saved);
  }
}

static void dma_cancel(uint32_t operations) {
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  _dcd.dma_pending &= ~operations;
  __set_PRIMASK(saved);
}

static void dma_request(uint8_t operation) {
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  if (NRF_USBD->ENABLE) {
    _dcd.dma_pending |= TU_BIT(operation);
  }
  __set_PRIMASK(saved);
  // An ISR must finish accounting its captured END before a new DMA starts.
  if (is_in_isr()) {
    NVIC_SetPendingIRQ(USBD_IRQn);
  } else {
    dma_dispatch();
  }
}

static void xact_out_dma(uint8_t epnum) {
  dma_request(DMA_OUT_BASE + epnum);
}

static void xact_in_dma(uint8_t epnum) {
  dma_request(DMA_IN_BASE + epnum + (CFG_TUD_NRF5X_ISO && epnum == EP_ISO_NUM));
}

// Called with USBD IRQ masked. A retired buffer must be out of hardware's hands
// before its descriptor can be reused. Its END is acknowledged without a stale
// completion, while another endpoint's DMA and pending work remain owned.
static bool dma_retire(uint32_t operations) {
  dma_cancel(operations);
  if (operations & _dcd.dma_active) {
    if (!dma_wait_end(0)) {
      tusb_hal_nrf_power_event(1);
      return false;
    }
    usbd_events(_dcd.dma_active, true);
    edpt_dma_end();
  }
  return true;
}
//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+
void dcd_init (uint8_t rhport)
{
  TU_LOG1("dcd init\r\n");
  (void) rhport;
}

void dcd_int_enable(uint8_t rhport)
{
  (void) rhport;
  NVIC_EnableIRQ(USBD_IRQn);
}

void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ(USBD_IRQn);
}

static __attribute__((noinline)) bool dcd_int_save(void) {
  const bool enabled = NVIC_GetEnableIRQ(USBD_IRQn);
  dcd_int_disable(0);
  return enabled;
}

static __attribute__((noinline)) void dcd_int_restore(bool enabled) {
  if (enabled && NRF_USBD->ENABLE) {
    dcd_int_enable(0);
  }
}

void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;
  (void) dev_addr;
  // Set Address is automatically update by hw controller, nothing to do

  // Enable usbevent for suspend and resume detection
  // Since the bus signal D+/D- are stable now.

  // Clear current pending first
  NRF_USBD->EVENTCAUSE |= NRF_USBD->EVENTCAUSE;
  NRF_USBD->EVENTS_USBEVENT = 0;

  NRF_USBD->INTENSET = USBD_INTEN_USBEVENT_Msk;
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;

  // Bring controller out of low power mode
  // will start wakeup when USBWUALLOWED is set
  NRF_USBD->LOWPOWER = 0;
}

// disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  NRF_USBD->USBPULLUP = 0;
  _ep0.received++;

  // Disable Pull-up does not trigger Power USB Removed, in fact it have no
  // impact on the USB Power status at all -> need to submit unplugged event to the stack.
  dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, false);
}

// connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  NRF_USBD->USBPULLUP = 1;
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
  (void) rhport;

  uint8_t const ep_addr = desc_edpt->bEndpointAddress;
  uint8_t const epnum   = tu_edpt_number(ep_addr);
  uint8_t const dir     = tu_edpt_dir(ep_addr);

#if !CFG_TUD_NRF5X_ISO
  TU_VERIFY(epnum < EP_CBI_COUNT && desc_edpt->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS);
#endif

  _dcd.xfer[epnum][dir].mps = tu_edpt_packet_size(desc_edpt);

#if CFG_TUD_NRF5X_ISO
  if (desc_edpt->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS)
#endif
  {
    if (dir == TUSB_DIR_OUT)
    {
      NRF_USBD->INTENSET = TU_BIT(USBD_INTEN_ENDEPOUT0_Pos + epnum);
      NRF_USBD->EPOUTEN |= TU_BIT(epnum);

      // Write any value to SIZE register will allow nRF to ACK/accept data
      NRF_USBD->SIZE.EPOUT[epnum] = 0;
    }else
    {
      NRF_USBD->INTENSET = TU_BIT(USBD_INTEN_ENDEPIN0_Pos + epnum);
      NRF_USBD->EPINEN  |= TU_BIT(epnum);
    }
  }
#if CFG_TUD_NRF5X_ISO

  else
  {
    TU_ASSERT(epnum == EP_ISO_NUM);
    if (dir == TUSB_DIR_OUT)
    {
      // SPLIT ISO buffer when ISO IN endpoint is already opened.
      if (_dcd.xfer[EP_ISO_NUM][TUSB_DIR_IN].mps) NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_HalfIN;

      // Clear old events
      NRF_USBD->EVENTS_ENDISOOUT = 0;

      // Clear SOF event in case interrupt was not enabled yet.
      if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0) NRF_USBD->EVENTS_SOF = 0;

      // Enable SOF and ISOOUT interrupts, and ISOOUT endpoint.
      NRF_USBD->INTENSET = USBD_INTENSET_ENDISOOUT_Msk | USBD_INTENSET_SOF_Msk;
      NRF_USBD->EPOUTEN |= USBD_EPOUTEN_ISOOUT_Msk;
    }
    else
    {
      NRF_USBD->EVENTS_ENDISOIN = 0;

      // SPLIT ISO buffer when ISO OUT endpoint is already opened.
      if (_dcd.xfer[EP_ISO_NUM][TUSB_DIR_OUT].mps) NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_HalfIN;

      // Clear SOF event in case interrupt was not enabled yet.
      if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0) NRF_USBD->EVENTS_SOF = 0;

      // Enable SOF and ISOIN interrupts, and ISOIN endpoint.
      NRF_USBD->INTENSET = USBD_INTENSET_ENDISOIN_Msk | USBD_INTENSET_SOF_Msk;
      NRF_USBD->EPINEN  |= USBD_EPINEN_ISOIN_Msk;
    }
  }
#endif


  // clear stall and reset DataToggle
  NRF_USBD->EPSTALL = (USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | ep_addr;
  NRF_USBD->DTOGGLE = (USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | ep_addr;

  __ISB(); __DSB();

  return true;
}

void dcd_edpt_close_all(uint8_t rhport) {
  (void)rhport;
  // Retire every non-control buffer before clearing its descriptor.
  const bool enabled = dcd_int_save();
  if (!dma_retire(DMA_ALL_MASK & ~DMA_EP0_MASK)) {
    return;
  }

  // Disable all non-control endpoints, including ISO. TASKS writes of zero
  // cannot stop DMA; only a write of one triggers a Nordic task.
  NRF_USBD->INTENCLR = (0xfeu << USBD_INTEN_ENDEPOUT0_Pos) | (0xfeu << USBD_INTEN_ENDEPIN0_Pos) |
                       USBD_INTENCLR_SOF_Msk |
                       (CFG_TUD_NRF5X_ISO ? (USBD_INTENCLR_ENDISOOUT_Msk | USBD_INTENCLR_ENDISOIN_Msk) : 0);
#if CFG_TUD_NRF5X_ISO
  NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_OneDir;
#endif
  tu_memclr(_dcd.xfer[1], sizeof(_dcd.xfer) - sizeof(_dcd.xfer[0]));

  // de-activate all non-control
  NRF_USBD->EPOUTEN = 1UL;
  NRF_USBD->EPINEN  = 1UL;

  dcd_int_restore(enabled);
}
void dcd_edpt_close (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);
#if !CFG_TUD_NRF5X_ISO
  TU_VERIFY(epnum < EP_CBI_COUNT, );
#endif
  const bool enabled = dcd_int_save();
  if (!dma_retire(TU_BIT(epnum + (dir == TUSB_DIR_OUT ? DMA_OUT_BASE : DMA_IN_BASE) +
                         (CFG_TUD_NRF5X_ISO && dir == TUSB_DIR_IN && epnum == EP_ISO_NUM)))) return;


#if CFG_TUD_NRF5X_ISO
  if (epnum != EP_ISO_NUM)
#endif
  {
    // CBI
    if (dir == TUSB_DIR_OUT)
    {
      NRF_USBD->INTENCLR = TU_BIT(USBD_INTEN_ENDEPOUT0_Pos + epnum);
      NRF_USBD->EPOUTEN &= ~TU_BIT(epnum);
    }
    else
    {
      NRF_USBD->INTENCLR = TU_BIT(USBD_INTEN_ENDEPIN0_Pos + epnum);
      NRF_USBD->EPINEN &= ~TU_BIT(epnum);
    }
  }
#if CFG_TUD_NRF5X_ISO

  else
  {
    _dcd.xfer[EP_ISO_NUM][dir].mps = 0;
    // ISO
    if (dir == TUSB_DIR_OUT)
    {
      NRF_USBD->INTENCLR = USBD_INTENCLR_ENDISOOUT_Msk;
      NRF_USBD->EPOUTEN &= ~USBD_EPOUTEN_ISOOUT_Msk;
      NRF_USBD->EVENTS_ENDISOOUT = 0;
    }
    else
    {
      NRF_USBD->INTENCLR = USBD_INTENCLR_ENDISOIN_Msk;
      NRF_USBD->EPINEN &= ~USBD_EPINEN_ISOIN_Msk;
    }
    // One of the ISO endpoints closed, no need to split buffers any more.
    NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_OneDir;
    // When both ISO endpoint are close there is no need for SOF any more.
    if (_dcd.xfer[EP_ISO_NUM][TUSB_DIR_IN].mps + _dcd.xfer[EP_ISO_NUM][TUSB_DIR_OUT].mps == 0) NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
  }
#endif

  __ISB(); __DSB();
  dcd_int_restore(enabled);
}

bool dcd_edpt0_setup_begin(uint8_t rhport, uint8_t setup_tagid) {
  (void)rhport;
  dma_cancel(DMA_EP0_MASK);
  _ep0.served = setup_tagid;
  return !ep0_superseded();
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);
#if !CFG_TUD_NRF5X_ISO
  TU_VERIFY(epnum < EP_CBI_COUNT, false);
#endif

  if (epnum == 0 && ep0_superseded()) {
    return true;
  }

  xfer_td_t* xfer = get_td(epnum, dir);

  xfer->buffer     = buffer;
  xfer->total_len  = total_bytes;
  xfer->actual_len = 0;

  // Control endpoint with zero-length packet and opposite direction to 1st request byte --> status stage
  const bool control_status = (epnum == 0 && total_bytes == 0 && ep_addr == _ep0.status_ep);

  if ( control_status )
  {
    // Status Phase also requires EasyDMA has to be available as well !!!!
    dma_request(DMA_EP0_STATUS);

  }
  else if ( dir == TUSB_DIR_OUT )
  {
    if ( epnum == 0 )
    {
      // Accept next Control Out packet. TASKS_EP0RCVOUT also require EasyDMA
      dma_request(DMA_EP0_RCVOUT);
    }else
    {
      if (xfer->data_received == 1)
      {
        // Data is already received previously
        // start DMA to copy to SRAM
        xfer->data_received = false;
        xact_out_dma(epnum);
      }
      else
      {
        // nRF auto accept next Bulk/Interrupt OUT packet
        // nothing to do
      }
    }
  }
  else
  {
    // Start DMA to copy data from RAM -> Endpoint
    xact_in_dma(epnum);
  }

  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  (void)rhport;
  const uint8_t epnum = tu_edpt_number(ep_addr);
  const uint8_t dir   = tu_edpt_dir(ep_addr);
#if !CFG_TUD_NRF5X_ISO
  TU_VERIFY(epnum < EP_CBI_COUNT, );
#endif
  const bool enabled = dcd_int_save();

  if (epnum == 0) {
    if (ep0_superseded()) {
      dcd_int_restore(enabled);
      return;
    }
    if (!dma_retire(DMA_EP0_MASK)) {
      return;
    }
    // A SETUP can arrive while retirement waits with only USBD IRQ masked.
    if (!ep0_superseded()) {
      NRF_USBD->TASKS_EP0STALL = 1;
    }
  } else if (epnum != EP_ISO_NUM) {
    NRF_USBD->EPSTALL = (USBD_EPSTALL_STALL_Stall << USBD_EPSTALL_STALL_Pos) | ep_addr;
    if (dir == TUSB_DIR_OUT) {
      xfer_td_t *xfer     = get_td(epnum, dir);
      xfer->data_received = OUT_HALTED;
      if (!dma_retire(TU_BIT(DMA_OUT_BASE + epnum))) {
        return;
      }
      xfer->total_len = xfer->actual_len = 0;
    }
  }
  __ISB();
  __DSB();
  dcd_int_restore(enabled);
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
  (void)rhport;
  const uint8_t epnum = tu_edpt_number(ep_addr);
  const uint8_t dir   = tu_edpt_dir(ep_addr);
#if !CFG_TUD_NRF5X_ISO
  TU_VERIFY(epnum < EP_CBI_COUNT, );
#endif
  if (epnum == 0 || epnum == EP_ISO_NUM) {
    return;
  }

  const bool enabled = dcd_int_save();
  NRF_USBD->DTOGGLE = ep_addr;
  NRF_USBD->DTOGGLE = (USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | ep_addr;
  if (dir == TUSB_DIR_OUT) {
    get_td(epnum, dir)->data_received = 0;
    NRF_USBD->EPDATASTATUS            = TU_BIT(16u + epnum);
    NRF_USBD->SIZE.EPOUT[epnum]       = 0;
  }
  NRF_USBD->EPSTALL = (USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | ep_addr;
  __ISB();
  __DSB();
  dcd_int_restore(enabled);
}

/*------------------------------------------------------------------*/
/* Interrupt Handler
 *------------------------------------------------------------------*/
static bool bus_reset(uint32_t captured) {
  _ep0.received++;
  if (!dma_wait_end(captured)) {
    // A reset may abort DMA without an END. Disable the peripheral before
    // releasing its latch/buffers, and report the same boundary as unplug.
    tusb_hal_nrf_power_event(1);
    return false;
  }
  usbd_events(EDPT_END_ALL_MASK, true);
  edpt_dma_end();
  // 6.35.6 USB controller automatically disabled all endpoints (except control)
  NRF_USBD->EPOUTEN = 1UL;
  NRF_USBD->EPINEN  = 1UL;

  // Clear USB Event Interrupt
  NRF_USBD->EVENTS_USBEVENT = 0;
  NRF_USBD->EVENTCAUSE |= NRF_USBD->EVENTCAUSE;

  // Reset interrupt
  NRF_USBD->INTENCLR = NRF_USBD->INTEN;
  NRF_USBD->INTENSET = USBD_INTEN_USBRESET_Msk | USBD_INTEN_USBEVENT_Msk | USBD_INTEN_EPDATA_Msk |
                       USBD_INTEN_EP0SETUP_Msk | USBD_INTEN_EP0DATADONE_Msk | USBD_INTEN_ENDEPIN0_Msk |
                       USBD_INTEN_ENDEPOUT0_Msk;

  tu_varclr(&_dcd);
  _dcd.xfer[0][TUSB_DIR_IN].mps  = MAX_PACKET_SIZE;
  _dcd.xfer[0][TUSB_DIR_OUT].mps = MAX_PACKET_SIZE;
  return true;
}
void dcd_int_handler(uint8_t rhport)
{
  (void) rhport;

  uint32_t const inten  = NRF_USBD->INTEN;
  uint32_t       int_status = usbd_events(inten, true);

  if ( int_status & USBD_INTEN_USBRESET_Msk )
  {
    if (bus_reset(int_status)) dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
    return; // events captured with the reset belong to the retired transfers
  }

  // ISOIN: Data was moved to endpoint buffer, client will be notified in SOF
  #if CFG_TUD_NRF5X_ISO
if ( int_status & USBD_INTEN_ENDISOIN_Msk )
  {
    xfer_td_t* xfer = get_td(EP_ISO_NUM, TUSB_DIR_IN);

    xfer->actual_len = NRF_USBD->ISOIN.AMOUNT;
    // Data transferred from RAM to endpoint output buffer.
    // Next transfer can be scheduled after SOF.
    xfer->iso_in_transfer_ready = true;
  }
#endif


  if ( int_status & USBD_INTEN_SOF_Msk )
  {
    bool iso_enabled = false;

    // ISOOUT: Transfer data gathered in previous frame from buffer to RAM
    #if CFG_TUD_NRF5X_ISO
if (NRF_USBD->EPOUTEN & USBD_EPOUTEN_ISOOUT_Msk)
    {
      iso_enabled = true;
      xact_out_dma(EP_ISO_NUM);
    }

    // ISOIN: Notify client that data was transferred
    if (NRF_USBD->EPINEN & USBD_EPINEN_ISOIN_Msk)
    {
      iso_enabled = true;

      xfer_td_t* xfer = get_td(EP_ISO_NUM, TUSB_DIR_IN);
      if ( xfer->iso_in_transfer_ready )
      {
        xfer->iso_in_transfer_ready = false;
        dcd_event_xfer_complete(0, EP_ISO_NUM | TUSB_DIR_IN_MASK, xfer->actual_len, XFER_RESULT_SUCCESS, true);
      }
    }
#endif


    if ( !iso_enabled )
    {
      // ISO endpoint is not used, SOF is only enabled one-time for remote wakeup
      // so we disable it now
      NRF_USBD->INTENCLR = USBD_INTENSET_SOF_Msk;
    }

    dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
  }

  if ( int_status & USBD_INTEN_USBEVENT_Msk )
  {
    TU_LOG(2, "EVENTCAUSE = 0x%04lX\r\n", NRF_USBD->EVENTCAUSE);

    enum { EVT_CAUSE_MASK = USBD_EVENTCAUSE_SUSPEND_Msk | USBD_EVENTCAUSE_RESUME_Msk | USBD_EVENTCAUSE_USBWUALLOWED_Msk };
    uint32_t const evt_cause = NRF_USBD->EVENTCAUSE & EVT_CAUSE_MASK;
    NRF_USBD->EVENTCAUSE = evt_cause; // clear interrupt

    if ( evt_cause & USBD_EVENTCAUSE_SUSPEND_Msk )
    {
      // Put controller into low power mode
      // Leave HFXO disable to application, since it may be used by other peripherals
      NRF_USBD->LOWPOWER = 1;

      dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
    }

    if ( evt_cause & USBD_EVENTCAUSE_USBWUALLOWED_Msk )
    {
      // USB is out of low power mode, and wakeup is allowed
      // Initiate RESUME signal
      NRF_USBD->DPDMVALUE = USBD_DPDMVALUE_STATE_Resume;
      NRF_USBD->TASKS_DPDMDRIVE = 1;

      // There is no Resume interrupt for remote wakeup, enable SOF for to report bus ready state
      // Clear SOF event in case interrupt was not enabled yet.
      if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0) NRF_USBD->EVENTS_SOF = 0;
      NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
    }

    if ( evt_cause & USBD_EVENTCAUSE_RESUME_Msk )
    {
      dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
    }
  }

  if (int_status & _dcd.dma_active) {
    edpt_dma_end();
  }

  // Every SETUP aborts the previous control request, even if busy DMA makes
  // its registers unreadable. Retire the old generation before the bounded wait.
  if (int_status & USBD_INTEN_EP0SETUP_Msk) {
    _ep0.received++;
    dma_cancel(DMA_EP0_MASK);
    int_status &= ~(USBD_INTEN_EP0DATADONE_Msk | USBD_INTEN_ENDEPOUT0_Msk);
  }
  // nRF52840 can read zero SETUP registers during an OUT DMA (#4164).
  // On timeout, drop this request rather than deliver a false zero packet.
  if ((int_status & USBD_INTEN_EP0SETUP_Msk) && dma_wait_end(int_status) && NRF_USBD->ENABLE) {
    uint8_t                  setup[8];
    const volatile uint32_t *setup_regs = &NRF_USBD->BMREQUESTTYPE;
    for (uint8_t i = 0; i < sizeof(setup); i++) {
      setup[i] = setup_regs[i];
    }
    if (!dma_retire(DMA_EP0_MASK)) {
      return;
    }
    _dcd.setup_dir = tu_edpt_dir(setup[0]);
    _ep0.status_ep = ((setup[6] | setup[7]) && _dcd.setup_dir == TUSB_DIR_IN) ? 0 : TUSB_DIR_IN_MASK;

    // nrf5x hw auto handle set address, there is no need to inform usb stack
    const tusb_control_request_t *request = (const tusb_control_request_t *)setup;

    if (!(TUSB_REQ_RCPT_DEVICE == request->bmRequestType_bit.recipient &&
          TUSB_REQ_TYPE_STANDARD == request->bmRequestType_bit.type && TUSB_REQ_SET_ADDRESS == request->bRequest)) {
      dcd_event_setup_received_tagid(0, setup, _ep0.received, true);
    }
  }

  //--------------------------------------------------------------------+
  /* Control/Bulk/Interrupt (CBI) Transfer
   *
   * Data flow is:
   *           (bus)              (dma)
   *    Host <-------> Endpoint <-------> RAM
   *
   * For CBI OUT:
   *  - Host -> Endpoint
   *      EPDATA (or EP0DATADONE) interrupted, check EPDATASTATUS.EPOUT[i]
   *      to start DMA. For Bulk/Interrupt, this step can occur automatically (without sw),
   *      which means data may or may not be ready (out_received flag).
   *  - Endpoint -> RAM
   *      ENDEPOUT[i] interrupted, transaction complete, sw prepare next transaction
   *
   * For CBI IN:
   *  - RAM -> Endpoint
   *      ENDEPIN[i] interrupted indicate DMA is complete. HW will start
   *      to move data to host
   *  - Endpoint -> Host
   *      EPDATA (or EP0DATADONE) interrupted, check EPDATASTATUS.EPIN[i].
   *      Transaction is complete, sw prepare next transaction
   *
   * Note: in both Control In and Out of Data stage from Host <-> Endpoint
   * EP0DATADONE will be set as interrupt source
   */
  //--------------------------------------------------------------------+

  /* CBI OUT: Endpoint -> SRAM (aka transaction complete)
   * Note: Since nRF controller auto ACK next packet without SW awareness
   * We must handle this stage before Host -> Endpoint just in case 2 event happens at once
   *
   * ISO OUT: Transaction must fit in single packet, it can be shorter then total
   * len if Host decides to sent fewer bytes, it this case transaction is also
   * complete and next transfer is not initiated here like for CBI.
   */
  for(uint8_t epnum=0; epnum<EP_CBI_COUNT+CFG_TUD_NRF5X_ISO; epnum++)
  {
    if ( tu_bit_test(int_status, USBD_INTEN_ENDEPOUT0_Pos+epnum))
    {
      xfer_td_t* xfer = get_td(epnum, TUSB_DIR_OUT);
      if (xfer->data_received == OUT_HALTED) {
        continue;
      }
      uint8_t const xact_len = NRF_USBD->EPOUT[epnum].AMOUNT;

      xfer->buffer     += xact_len;
      xfer->actual_len += xact_len;

      // Transfer complete if transaction len < Max Packet Size or total len is transferred
      if ( (epnum != EP_ISO_NUM) && (xact_len == xfer->mps) && (xfer->actual_len < xfer->total_len) )
      {
        if ( epnum == 0 )
        {
          // Accept next Control Out packet. TASKS_EP0RCVOUT also require EasyDMA
          dma_request(DMA_EP0_RCVOUT);
        }else
        {
          // nRF auto accept next Bulk/Interrupt OUT packet
          // nothing to do
        }
      }else
      {
        xfer->total_len = xfer->actual_len;

        // CBI OUT complete
        dcd_event_xfer_complete(0, epnum, xfer->actual_len, XFER_RESULT_SUCCESS, true);
      }
    }

    // Ended event for CBI IN : nothing to do
  }

  // Endpoint <-> Host ( In & OUT )
  if ( int_status & (USBD_INTEN_EPDATA_Msk | USBD_INTEN_EP0DATADONE_Msk) )
  {
    uint32_t data_status = NRF_USBD->EPDATASTATUS;
    NRF_USBD->EPDATASTATUS = data_status;
    __ISB(); __DSB();

    // EP0DATADONE is set with either Control Out on IN Data
    // Since EPDATASTATUS cannot be used to determine whether it is control OUT or IN.
    // Use the direction captured before a later DMA can hide the SETUP registers.
    if (int_status & USBD_INTEN_EP0DATADONE_Msk) {
      data_status |= TU_BIT(_dcd.setup_dir == TUSB_DIR_OUT ? 16 : 0);
    }

    // CBI In: Endpoint -> Host (transaction complete)
    for(uint8_t epnum=0; epnum<EP_CBI_COUNT; epnum++)
    {
      if ( tu_bit_test(data_status, epnum) )
      {
        xfer_td_t* xfer = get_td(epnum, TUSB_DIR_IN);
        uint8_t const xact_len = NRF_USBD->EPIN[epnum].AMOUNT;

        xfer->buffer     += xact_len;
        xfer->actual_len += xact_len;

        if ( xfer->actual_len < xfer->total_len )
        {
          // Start DMA to copy next data packet
          xact_in_dma(epnum);
        } else
        {
          // CBI IN complete
          dcd_event_xfer_complete(0, epnum | TUSB_DIR_IN_MASK, xfer->actual_len, XFER_RESULT_SUCCESS, true);
        }
      }
    }

    // CBI OUT: Host -> Endpoint
    for(uint8_t epnum=0; epnum<EP_CBI_COUNT; epnum++)
    {
      if ( tu_bit_test(data_status, 16+epnum) )
      {
        xfer_td_t* xfer = get_td(epnum, TUSB_DIR_OUT);
        if (xfer->data_received == OUT_HALTED) {
          continue;
        }

        if (xfer->actual_len < xfer->total_len)
        {
          xact_out_dma(epnum);
        }else
        {
          // Data overflow !!! Nah, nRF will auto accept next Bulk/Interrupt OUT packet
          // Mark this endpoint with data received
          xfer->data_received = true;
        }
      }
    }
  }
  dma_dispatch();

}

//--------------------------------------------------------------------+
// HFCLK helper
//--------------------------------------------------------------------+
#ifdef SOFTDEVICE_PRESENT

// For enable/disable hfclk with SoftDevice
#include "nrf_mbr.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#ifndef SD_MAGIC_NUMBER
  #define SD_MAGIC_NUMBER   0x51B1E5DB
#endif

static inline bool is_sd_existed(void)
{
  return *((uint32_t*)(SOFTDEVICE_INFO_STRUCT_ADDRESS+4)) == SD_MAGIC_NUMBER;
}

// check if SD is existed and enabled
static inline bool is_sd_enabled(void)
{
  if ( !is_sd_existed() ) return false;

  uint8_t sd_en = false;
  (void) sd_softdevice_is_enabled(&sd_en);
  return sd_en;
}
#endif

static bool hfclk_running(void)
{
#ifdef SOFTDEVICE_PRESENT
  if ( is_sd_enabled() )
  {
    uint32_t is_running = 0;
    (void) sd_clock_hfclk_is_running(&is_running);
    return (is_running ? true : false);
  }
#endif

  return nrf_clock_hf_is_running(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY);
}

static void hfclk_enable(void)
{
  // already running, nothing to do
  if ( hfclk_running() ) return;

#ifdef SOFTDEVICE_PRESENT
  if ( is_sd_enabled() )
  {
    (void)sd_clock_hfclk_request();
    return;
  }
#endif

  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
}

static void hfclk_disable(void)
{
#ifdef SOFTDEVICE_PRESENT
  if ( is_sd_enabled() )
  {
    (void)sd_clock_hfclk_release();
    return;
  }
#endif

  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTOP);
}

  #ifdef NRF52_SERIES
static __attribute__((noinline)) void usbd_errata_write(volatile uint32_t *reg, uint32_t value) {
  volatile uint32_t *const latch = (volatile uint32_t *)0x4006EC00;
  if (*latch == 0) {
    *latch = 0x9375;
    *reg   = value;
    *latch = 0x9375;
  } else {
    *reg = value;
  }
}
  #endif

// Power & Clock Peripheral on nRF5x to manage USB
//
// USB Bus power is managed by Power module, there are 3 VBUS power events:
// Detected, Ready, Removed. Upon these power events, This function will
// enable ( or disable ) usb & hfclk peripheral, set the usb pin pull up
// accordingly to the controller Startup/Standby Sequence in USBD 51.4 specs.
//
// Therefore this function must be called to handle USB power event by
// - nrfx_power_usbevt_init() : if Softdevice is not used or enabled
// - SoftDevice SOC event : if SD is used and enabled
void tusb_hal_nrf_power_event (uint32_t event)
{
  // Value is chosen to be as same as NRFX_POWER_USB_EVT_* in nrfx_power.h
  enum {
    USB_EVT_DETECTED = 0,
    USB_EVT_REMOVED = 1,
    USB_EVT_READY = 2
  };

#if CFG_TUSB_DEBUG >= 2
  const char* const power_evt_str[] = { "Detected", "Removed", "Ready" };
  TU_LOG(2, "Power USB event: %s\r\n", power_evt_str[event]);
#endif

  switch ( event )
  {
    case USB_EVT_DETECTED:
      if ( !NRF_USBD->ENABLE )
      {
        // Prepare for receiving READY event: disable interrupt since we will blocking wait
        NRF_USBD->INTENCLR = USBD_INTEN_USBEVENT_Msk;
        NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
        __ISB(); __DSB(); // for sync

#ifdef NRF52_SERIES // NRF53 does not need this errata
        // ERRATA 171, 187, 166
        if ( nrfx_usbd_errata_187() )
        {
          // CRITICAL_REGION_ENTER();
          usbd_errata_write((volatile uint32_t *)0x4006ED14, 0x00000003);
          // CRITICAL_REGION_EXIT();
        }

        if ( nrfx_usbd_errata_171() )
        {
          // CRITICAL_REGION_ENTER();
          usbd_errata_write((volatile uint32_t *)0x4006EC14, 0x000000C0);
          // CRITICAL_REGION_EXIT();
        }
#endif

        // Enable the peripheral (will cause Ready event)
        NRF_USBD->ENABLE = 1;
        __ISB(); __DSB(); // for sync

        // Enable HFCLK
        hfclk_enable();
      }
    break;

    case USB_EVT_READY:
    {
      // READY is a consumed event, not a persistent status bit. In particular,
      // a post-SoftDevice callback may find USB already attached but HFCLK no
      // longer requested by this context. Restore the clock before testing
      // completion; do not wait for a second peripheral READY event.
      uint32_t remaining = 100000;
      if ( !NRF_USBD->ENABLE ) break;
      hfclk_enable();
      while ( !hfclk_running() )
      {
        if ( !NRF_USBD->ENABLE || !--remaining ) return;
      }

      // Recheck attachment inside the wait: another READY handler can preempt
      // this one and consume EVENTCAUSE before it attaches. Both waits share
      // a finite poll budget, independent of ticks/interrupts being available.
      while ( !(USBD_EVENTCAUSE_READY_Msk & NRF_USBD->EVENTCAUSE) )
      {
        if ( NRF_USBD->USBPULLUP || !NRF_USBD->ENABLE || !--remaining ) return;
      }
      if ( NRF_USBD->USBPULLUP || !NRF_USBD->ENABLE ) break;

      NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
      __ISB(); __DSB(); // for sync

#ifdef NRF52_SERIES
      if ( nrfx_usbd_errata_171() )
      {
        // CRITICAL_REGION_ENTER();
        usbd_errata_write((volatile uint32_t *)0x4006EC14, 0x00000000);

        // CRITICAL_REGION_EXIT();
      }

      if ( nrfx_usbd_errata_187() )
      {
        // CRITICAL_REGION_ENTER();
        usbd_errata_write((volatile uint32_t *)0x4006ED14, 0x00000000);
        // CRITICAL_REGION_EXIT();
      }

      if ( nrfx_usbd_errata_166() )
      {
        *((volatile uint32_t *) (NRF_USBD_BASE + 0x800)) = 0x7E3;
        *((volatile uint32_t *) (NRF_USBD_BASE + 0x804)) = 0x40;

        __ISB(); __DSB();
      }
#endif

      // ISO buffer Lower half for IN, upper half for OUT
#if CFG_TUD_NRF5X_ISO
      NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_HalfIN;

#endif
      // Enable bus-reset interrupt
      NRF_USBD->INTENSET = USBD_INTEN_USBRESET_Msk;

      // Enable interrupt, priorities should be set by application
      NVIC_ClearPendingIRQ(USBD_IRQn);
      // Don't enable USBD interrupt yet, if dcd_init() did not finish yet
      // Interrupt will be enabled by tud_init(), when USB stack is ready
      // to handle interrupts.
      if (tud_inited())
      {
        NVIC_EnableIRQ(USBD_IRQn);
      }

      // Enable pull up
      if ( NRF_USBD->ENABLE ) NRF_USBD->USBPULLUP = 1;
      __ISB(); __DSB(); // for sync
    }
    break;

    case USB_EVT_REMOVED:
      if ( NRF_USBD->ENABLE )
      {
        // Abort all transfers

        // Disable pull up
        NRF_USBD->USBPULLUP = 0;
        __ISB(); __DSB(); // for sync

        // Disable Interrupt
        NVIC_DisableIRQ(USBD_IRQn);

        // disable all interrupt
        NRF_USBD->INTENCLR = NRF_USBD->INTEN;

        (void)dma_wait_end(0);
        NRF_USBD->ENABLE = 0;
        __ISB(); __DSB(); // for sync

        _dcd.dma_pending = 0;
        _ep0.received++;
        edpt_dma_end();

        hfclk_disable();

        dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, is_in_isr());
      }
    break;

    default: break;
  }
}

#endif
