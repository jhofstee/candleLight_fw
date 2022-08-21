/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "can.h"
#include "config.h"

// The STM32F0 only has one CAN interface, define it as CAN1 as
// well, so it doesn't need to be handled separately.
#if !defined(CAN1) && defined(CAN)
#define CAN1 CAN
#endif

// Completely reset the CAN pheriperal, including bus-state and error counters
static void rcc_reset(CAN_TypeDef *instance)
{
#ifdef CAN1
	if (instance == CAN1) {
		__HAL_RCC_CAN1_FORCE_RESET();
		__HAL_RCC_CAN1_RELEASE_RESET();
	}
#endif

#ifdef CAN2
	if (instance == CAN2) {
		__HAL_RCC_CAN2_FORCE_RESET();
		__HAL_RCC_CAN2_RELEASE_RESET();
	}
#endif
}

void can_init(can_data_t *hcan, CAN_TypeDef *instance)
{
	__HAL_RCC_CAN1_CLK_ENABLE();

	GPIO_InitTypeDef itd;
#if defined(STM32F0)
	itd.Pin = GPIO_PIN_8|GPIO_PIN_9;
	itd.Mode = GPIO_MODE_AF_PP;
	itd.Pull = GPIO_NOPULL;
	itd.Speed = GPIO_SPEED_FREQ_HIGH;
	itd.Alternate = GPIO_AF4_CAN;
	HAL_GPIO_Init(GPIOB, &itd);
#elif defined(STM32F4)
	itd.Pin = GPIO_PIN_0|GPIO_PIN_1;
	itd.Mode = GPIO_MODE_AF_PP;
	itd.Pull = GPIO_NOPULL;
	itd.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	itd.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOD, &itd);
#endif

	hcan->instance   = instance;
	hcan->brp        = 6;
	hcan->sjw        = 1;
	hcan->state      = GS_CAN_STATE_STOPPED;
#if defined(STM32F0)
	hcan->phase_seg1 = 13;
	hcan->phase_seg2 = 2;
#elif defined(STM32F4)
	hcan->phase_seg1 = 12;
	hcan->phase_seg2 = 1;
#endif
}

bool can_set_bittiming(can_data_t *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
	if ( (brp>0) && (brp<=1024)
	  && (phase_seg1>0) && (phase_seg1<=16)
	  && (phase_seg2>0) && (phase_seg2<=8)
	  && (sjw>0) && (sjw<=4)
	) {
		hcan->brp = brp & 0x3FF;
		hcan->phase_seg1 = phase_seg1;
		hcan->phase_seg2 = phase_seg2;
		hcan->sjw = sjw;
		return true;
	} else {
		return false;
	}
}

void can_enable(can_data_t *hcan, bool loop_back, bool listen_only, bool one_shot, bool berr_reporting)
{
	CAN_TypeDef *can = hcan->instance;

	uint32_t mcr = CAN_MCR_INRQ
				 | CAN_MCR_TXFP
				 | (one_shot ? CAN_MCR_NART : 0);

	uint32_t btr = ((uint32_t)(hcan->sjw-1)) << 24
				 | ((uint32_t)(hcan->phase_seg1-1)) << 16
				 | ((uint32_t)(hcan->phase_seg2-1)) << 20
				 | (hcan->brp - 1)
				 | (loop_back ? CAN_MODE_LOOPBACK : 0)
				 | (listen_only ? CAN_MODE_SILENT : 0);

	hcan->berr_reporting = berr_reporting;

	// Reset CAN peripheral
	can->MCR |= CAN_MCR_RESET;
	while((can->MCR & CAN_MCR_RESET) != 0); // reset bit is set to zero after reset
	while((can->MSR & CAN_MSR_SLAK) == 0);  // should be in sleep mode after reset

	// Completely reset while being of the bus
	rcc_reset(can);

	can->MCR |= CAN_MCR_INRQ ;
	while((can->MSR & CAN_MSR_INAK) == 0);

	can->MCR = mcr;
	can->BTR = btr;

	can->MCR &= ~CAN_MCR_INRQ;
	while((can->MSR & CAN_MSR_INAK) != 0);

	uint32_t filter_bit = 0x00000001;
	can->FMR |= CAN_FMR_FINIT;
	can->FMR &= ~CAN_FMR_CAN2SB;
	can->FA1R &= ~filter_bit;        // disable filter
	can->FS1R |= filter_bit;         // set to single 32-bit filter mode
	can->FM1R &= ~filter_bit;        // set filter mask mode for filter 0
	can->sFilterRegister[0].FR1 = 0; // filter ID = 0
	can->sFilterRegister[0].FR2 = 0; // filter Mask = 0
	can->FFA1R &= ~filter_bit;       // assign filter 0 to FIFO 0
	can->FA1R |= filter_bit;         // enable filter
	can->FMR &= ~CAN_FMR_FINIT;

	hcan->state = GS_CAN_STATE_ERROR_ACTIVE;

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_PIN_SET);
#endif
}

void can_disable(can_data_t *hcan)
{
	CAN_TypeDef *can = hcan->instance;
#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_PIN_RESET);
#endif
	can->MCR |= CAN_MCR_INRQ ; // send can controller into initialization mode

	hcan->state = GS_CAN_STATE_STOPPED;
}

bool can_is_enabled(can_data_t *hcan)
{
	CAN_TypeDef *can = hcan->instance;
	return (can->MCR & CAN_MCR_INRQ) == 0;
}

bool can_is_rx_pending(can_data_t *hcan)
{
	CAN_TypeDef *can = hcan->instance;
	return ((can->RF0R & CAN_RF0R_FMP0) != 0);
}

bool can_receive(can_data_t *hcan, struct gs_host_frame *rx_frame)
{
	CAN_TypeDef *can = hcan->instance;

	if (can_is_rx_pending(hcan)) {
		CAN_FIFOMailBox_TypeDef *fifo = &can->sFIFOMailBox[0];

		if (fifo->RIR &  CAN_RI0R_IDE) {
			rx_frame->can_id = CAN_EFF_FLAG | ((fifo->RIR >> 3) & 0x1FFFFFFF);
		} else {
			rx_frame->can_id = (fifo->RIR >> 21) & 0x7FF;
		}

		if (fifo->RIR & CAN_RI0R_RTR)  {
			rx_frame->can_id |= CAN_RTR_FLAG;
		}

		rx_frame->can_dlc = fifo->RDTR & CAN_RDT0R_DLC;

		rx_frame->data[0] = (fifo->RDLR >>  0) & 0xFF;
		rx_frame->data[1] = (fifo->RDLR >>  8) & 0xFF;
		rx_frame->data[2] = (fifo->RDLR >> 16) & 0xFF;
		rx_frame->data[3] = (fifo->RDLR >> 24) & 0xFF;
		rx_frame->data[4] = (fifo->RDHR >>  0) & 0xFF;
		rx_frame->data[5] = (fifo->RDHR >>  8) & 0xFF;
		rx_frame->data[6] = (fifo->RDHR >> 16) & 0xFF;
		rx_frame->data[7] = (fifo->RDHR >> 24) & 0xFF;

		can->RF0R |= CAN_RF0R_RFOM0; // release FIFO

		return true;
	} else {
		return false;
	}
}

static CAN_TxMailBox_TypeDef *can_find_free_mailbox(can_data_t *hcan)
{
	CAN_TypeDef *can = hcan->instance;

	uint32_t tsr = can->TSR;
	if ( tsr & CAN_TSR_TME0 ) {
		return &can->sTxMailBox[0];
	} else if ( tsr & CAN_TSR_TME1 ) {
		return &can->sTxMailBox[1];
	} else if ( tsr & CAN_TSR_TME2 ) {
		return &can->sTxMailBox[2];
	} else {
		return 0;
	}
}

bool can_send(can_data_t *hcan, struct gs_host_frame *frame)
{
	CAN_TxMailBox_TypeDef *mb = can_find_free_mailbox(hcan);
	if (mb != 0) {

		/* first, clear transmission request */
		mb->TIR &= CAN_TI0R_TXRQ;

		if (frame->can_id & CAN_EFF_FLAG) { // extended id
			mb->TIR = CAN_ID_EXT | (frame->can_id & 0x1FFFFFFF) << 3;
		} else {
			mb->TIR = (frame->can_id & 0x7FF) << 21;
		}

		if (frame->can_id & CAN_RTR_FLAG) {
			mb->TIR |= CAN_RTR_REMOTE;
		}

		mb->TDTR &= 0xFFFFFFF0;
		mb->TDTR |= frame->can_dlc & 0x0F;

		mb->TDLR =
			  ( frame->data[3] << 24 )
			| ( frame->data[2] << 16 )
			| ( frame->data[1] <<  8 )
			| ( frame->data[0] <<  0 );

		mb->TDHR =
			  ( frame->data[7] << 24 )
			| ( frame->data[6] << 16 )
			| ( frame->data[5] <<  8 )
			| ( frame->data[4] <<  0 );

		/* request transmission */
		mb->TIR |= CAN_TI0R_TXRQ;

		return true;
	} else {
		return false;
	}
}

uint32_t can_get_error_status(can_data_t *hcan)
{
	CAN_TypeDef *can = hcan->instance;
	return can->ESR;
}

static void create_error_frame(struct gs_host_frame *frame, uint32_t err)
{
	frame->echo_id = 0xFFFFFFFF;
	frame->can_id  = CAN_ERR_FLAG | CAN_ERR_CRTL;
	frame->can_dlc = CAN_ERR_DLC;
	frame->data[0] = CAN_ERR_LOSTARB_UNSPEC;
	frame->data[1] = CAN_ERR_CRTL_UNSPEC;
	frame->data[2] = CAN_ERR_PROT_UNSPEC;
	frame->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	frame->data[4] = CAN_ERR_TRX_UNSPEC;
	frame->data[5] = 0;
	frame->data[6] = can_tec(err);
	frame->data[7] = can_rec(err);
}

bool can_berr_report_pending(can_data_t *hcan)
{
	uint8_t lec;

	if (!hcan->berr_reporting)
		return false;
	lec = (can_get_error_status(hcan) >> 4) & 0x07;

	return lec != 0 && lec != 7;
}

static void add_berr_info(can_data_t *hcan, struct gs_host_frame *frame)
{
	CAN_TypeDef *can = hcan->instance;
	uint8_t lec = (can->ESR >> 4) & 0x07;

	switch (lec) {
		case 0x01: /* stuff error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_STUFF;
			break;
		case 0x02: /* form error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_FORM;
			break;
		case 0x03: /* ack error */
			frame->can_id |= CAN_ERR_ACK;
			break;
		case 0x04: /* bit recessive error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_BIT1;
			break;
		case 0x05: /* bit dominant error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_BIT0;
			break;
		case 0x06: /* CRC error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
			break;
		default: /* 0=no error, 7=no change */
			break;
	}

	/* Write 7 (software) to LEC to indicate it has been handled */
	can->ESR |= 7<<4;
}

void can_report_berr(can_data_t *hcan, struct gs_host_frame *frame)
{
	create_error_frame(frame, can_get_error_status(hcan));
	add_berr_info(hcan, frame);
}

bool can_new_state(can_data_t *hcan, enum gs_can_state *state)
{
	uint32_t err = can_get_error_status(hcan);

	// check normal operation state first, to leave early in almost all cases.
	if ((err & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF)) == 0 &&
			hcan->state == GS_CAN_STATE_ERROR_ACTIVE) {
		return false;
	}

	// states with cannot change automatically.
	if (hcan->state >= GS_CAN_STATE_BUS_OFF) {
		return false;
	}

	if (err & CAN_ESR_BOFF) {
		*state = GS_CAN_STATE_BUS_OFF;
	} else if (err & CAN_ESR_EPVF) {
		*state = GS_CAN_STATE_ERROR_PASSIVE;
	} else if (err & CAN_ESR_EWGF) {
		*state = GS_CAN_STATE_ERROR_WARNING;
	} else {
		*state = GS_CAN_STATE_ERROR_ACTIVE;
	}

	return *state != hcan->state;
}

void can_change_state(can_data_t *hcan, struct gs_host_frame *frame, enum gs_can_state new_state)
{
	uint32_t err = can_get_error_status(hcan);
	uint8_t tec = can_tec(err);
	uint8_t rec = can_rec(err);

	create_error_frame(frame, err);

	switch (new_state) {
		case GS_CAN_STATE_STOPPED:
		case GS_CAN_STATE_SLEEPING:
		case GS_CAN_STATE_BUS_OFF:
			frame->can_id |= CAN_ERR_BUSOFF;
			break;

		case GS_CAN_STATE_ERROR_PASSIVE:
			frame->data[1] |= rec >= tec ? CAN_ERR_CRTL_RX_PASSIVE : 0;
			frame->data[1] |= rec <= tec ? CAN_ERR_CRTL_TX_PASSIVE : 0;
			break;

		case GS_CAN_STATE_ERROR_WARNING:
			frame->data[1] |= rec >= tec ? CAN_ERR_CRTL_RX_WARNING : 0;
			frame->data[1] |= rec <= tec ? CAN_ERR_CRTL_TX_WARNING : 0;
			break;

		case GS_CAN_STATE_ERROR_ACTIVE:
			frame->data[1] |= CAN_ERR_CRTL_ACTIVE;
			break;
	}

	hcan->state = new_state;

	if (can_berr_report_pending(hcan))
		add_berr_info(hcan, frame);
}
