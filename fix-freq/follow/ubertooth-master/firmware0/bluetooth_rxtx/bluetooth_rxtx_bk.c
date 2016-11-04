cc2400_rx()
{

	cc2400_set(MANAND,  0x7fff);
	cc2400_set(LMTST,   0x2b22);
//	cc2400_set(MDMTST0, 0x164b); // without PRNG
	cc2400_set(MDMTST0, 0x164b); // without PRNG
	cc2400_set(GRMDM,   0x0301); // un-buffered mode, GFSK
//	cc2400_set(GRMDM,   0x0761); // un-buffered mode, GFSK
//	cc2400_set(GRMDM,   0x6761); // un-buffered mode, GFSK
	// 0 00 00 0 010 00 0 00 0 1
	//      |  | |   |  +--------> CRC off
	//      |  | |   +-----------> sync word: 8 MSB bits of SYNC_WORD
	//      |  | +---------------> 2 preamble bytes of 01010101
	//      |  +-----------------> not packet mode
	//      +--------------------> un-buffered mode
	cc2400_set(FSDIV,   channel - 1); // 1 MHz IF
	cc2400_set(MDMCTRL, 0x0029);

	// Set up CS register
	cs_threshold_calc_and_set();

	while (!(cc2400_status() & XOSC16M_STABLE));
	cc2400_strobe(SFSON);
	while (!(cc2400_status() & FS_LOCK));
	cc2400_strobe(SRX);
#ifdef UBERTOOTH_ONE
	PAEN_SET;
	HGM_SET;
#endif
}

/* start un-buffered rx */
static void cc2400_rx_sync(u32 sync)
{
	u16 grmdm, mdmctrl;

	if (modulation == MOD_BT_BASIC_RATE) {
		mdmctrl = 0x0029; // 160 kHz frequency deviation
		grmdm = 0x0461; // un-buffered mode, packet w/ sync word detection
		// 0 00 00 1 000 11 0 00 0 1
		//   |  |  | |   |  +--------> CRC off
		//   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
		//   |  |  | +---------------> 0 preamble bytes of 01010101
		//   |  |  +-----------------> packet mode
		//   |  +--------------------> un-buffered mode
		//   +-----------------------> sync error bits: 0

	} else if (modulation == MOD_BT_LOW_ENERGY) {
		mdmctrl = 0x0040; // 250 kHz frequency deviation
		grmdm = 0x0561; // un-buffered mode, packet w/ sync word detection
		// 0 00 00 1 010 11 0 00 0 1
		//   |  |  | |   |  +--------> CRC off
		//   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
		//   |  |  | +---------------> 2 preamble bytes of 01010101
		//   |  |  +-----------------> packet mode
		//   |  +--------------------> un-buffered mode
		//   +-----------------------> sync error bits: 0

	} else {
		/* oops */
		return;
	}

	cc2400_set(MANAND,  0x7fff);
	cc2400_set(LMTST,   0x2b22);

	cc2400_set(MDMTST0, 0x124b);
	// 1      2      4b
	// 00 0 1 0 0 10 01001011
	//    | | | | |  +---------> AFC_DELTA = ??
	//    | | | | +------------> AFC settling = 4 pairs (8 bit preamble)
	//    | | | +--------------> no AFC adjust on packet
	//    | | +----------------> do not invert data
	//    | +------------------> TX IF freq 1 0Hz
	//    +--------------------> PRNG off
	//
	// ref: CC2400 datasheet page 67
	// AFC settling explained page 41/42

	cc2400_set(GRMDM,   grmdm);

	cc2400_set(SYNCL,   sync & 0xffff);
	cc2400_set(SYNCH,   (sync >> 16) & 0xffff);

	cc2400_set(FSDIV,   channel - 1); // 1 MHz IF
	cc2400_set(MDMCTRL, mdmctrl);

	// Set up CS register
	cs_threshold_calc_and_set();

	while (!(cc2400_status() & XOSC16M_STABLE));
	cc2400_strobe(SFSON);
	while (!(cc2400_status() & FS_LOCK));
	cc2400_strobe(SRX);
#ifdef UBERTOOTH_ONE
	PAEN_SET;
	HGM_SET;
#endif
}

/*
 * Transmit a BTLE packet with the specified access address.
 *
 * All modulation parameters are set within this function. The data
 * should not be pre-whitened, but the CRC should be calculated and
 * included in the data length.
 */
void le_transmit(u32 aa, u8 len, u8 *data)
{
	unsigned i, j;
	int bit;
	u8 txbuf[64];
	u8 tx_len;
	u8 byte;
	u16 gio_save;

	// first four bytes: AA
	for (i = 0; i < 4; ++i) {
		byte = aa & 0xff;
		aa >>= 8;
		txbuf[i] = 0;
		for (j = 0; j < 8; ++j) {
			txbuf[i] |= (byte & 1) << (7 - j);
			byte >>= 1;
		}
	}

	// whiten the data and copy it into the txbuf
	int idx = whitening_index[btle_channel_index(channel-2402)];
	for (i = 0; i < len; ++i) {
		byte = data[i];
		txbuf[i+4] = 0;
		for (j = 0; j < 8; ++j) {
			bit = (byte & 1) ^ whitening[idx];
			idx = (idx + 1) % sizeof(whitening);
			byte >>= 1;
			txbuf[i+4] |= bit << (7 - j);
		}
	}

	len += 4; // include the AA in len

	// Bluetooth-like modulation
	cc2400_set(MANAND,  0x7fff);
	cc2400_set(LMTST,   0x2b22);    // LNA and receive mixers test register
	cc2400_set(MDMTST0, 0x134b);    // no PRNG

	cc2400_set(GRMDM,   0x0c01);
	// 0 00 01 1 000 00 0 00 0 1
	//      |  | |   |  +--------> CRC off
	//      |  | |   +-----------> sync word: 8 MSB bits of SYNC_WORD
	//      |  | +---------------> 0 preamble bytes of 01010101
	//      |  +-----------------> packet mode
	//      +--------------------> buffered mode

	cc2400_set(FSDIV,   channel);
	cc2400_set(FREND,   0b1011);    // amplifier level (-7 dBm, picked from hat)
	cc2400_set(MDMCTRL, 0x0040);    // 250 kHz frequency deviation
	cc2400_set(INT,     0x0014);	// FIFO_THRESHOLD: 20 bytes

	// sync byte depends on the first transmitted bit of the AA
	if (aa & 1)
		cc2400_set(SYNCH,   0xaaaa);
	else
		cc2400_set(SYNCH,   0x5555);

	// set GIO to FIFO_FULL
	gio_save = cc2400_get(IOCFG);
	cc2400_set(IOCFG, (GIO_FIFO_FULL << 9) | (gio_save & 0x1ff));

	while (!(cc2400_status() & XOSC16M_STABLE));
	cc2400_strobe(SFSON);
	while (!(cc2400_status() & FS_LOCK));
	TXLED_SET;
#ifdef UBERTOOTH_ONE
	PAEN_SET;
#endif
	while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
	cc2400_strobe(STX);

	// put the packet into the FIFO
	for (i = 0; i < len; i += 16) {
		while (GIO6) ; // wait for the FIFO to drain (FIFO_FULL false)
		tx_len = len - i;
		if (tx_len > 16)
			tx_len = 16;
		cc2400_spi_buf(FIFOREG, tx_len, txbuf + i);
	}

	while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
	TXLED_CLR;

	cc2400_strobe(SRFOFF);
	while ((cc2400_status() & FS_LOCK));

#ifdef UBERTOOTH_ONE
	PAEN_CLR;
#endif

	// reset GIO
	cc2400_set(IOCFG, gio_save);
}

void le_jam(void) {
#ifdef TX_ENABLE
	cc2400_set(MANAND,  0x7fff);
	cc2400_set(LMTST,   0x2b22);    // LNA and receive mixers test register
	cc2400_set(MDMTST0, 0x234b);    // PRNG, 1 MHz offset

	cc2400_set(GRMDM,   0x0c01);
	// 0 00 01 1 000 00 0 00 0 1
	//      |  | |   |  +--------> CRC off
	//      |  | |   +-----------> sync word: 8 MSB bits of SYNC_WORD
	//      |  | +---------------> 0 preamble bytes of 01010101
	//      |  +-----------------> packet mode
	//      +--------------------> buffered mode

	// cc2400_set(FSDIV,   channel);
	cc2400_set(FREND,   0b1011);    // amplifier level (-7 dBm, picked from hat)
	cc2400_set(MDMCTRL, 0x0040);    // 250 kHz frequency deviation

	while (!(cc2400_status() & XOSC16M_STABLE));
	cc2400_strobe(SFSON);
	while (!(cc2400_status() & FS_LOCK));
	TXLED_SET;
#ifdef UBERTOOTH_ONE
	PAEN_SET;
#endif
	while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
	cc2400_strobe(STX);
#endif
}

/* TODO - return whether hop happened, or should caller have to keep
 * track of this? */
void hop(void)
{
	do_hop = 0;
	last_hop = clkn;

	// No hopping, if channel is set correctly, do nothing
	if (hop_mode == HOP_NONE) {
		if (cc2400_get(FSDIV) == (channel - 1))
			return;
	}

	/* Slow sweep (100 hops/sec)
	 * only hop to currently used channels if AFH is enabled
	 */
	else if (hop_mode == HOP_SWEEP) {
		TXLED_SET;
		do {
			channel += 32;
			if (channel > 2480)
				channel -= 79;
		} while ( used_channels != 0 && afh_enabled && !( afh_map[(channel-2402)/8] & 0x1<<((channel-2402)%8) ) );
	}

	/* AFH detection
	 * only hop to currently unused channesl
	 */
	else if (hop_mode == HOP_AFH) {
		do {
			channel += 32;
			if (channel > 2480)
				channel -= 79;
		} while( used_channels != 79 && (afh_map[(channel-2402)/8] & 0x1<<((channel-2402)%8)) );
	}

	else if (hop_mode == HOP_BLUETOOTH) {
		TXLED_SET;
		channel = next_hop(clkn);
	}

	else if (hop_mode == HOP_BTLE) {
		TXLED_SET;
		channel = btle_next_hop(&le);
	}

	else if (hop_mode == HOP_DIRECT) {
		TXLED_SET;
		channel = hop_direct_channel;
	}

        /* IDLE mode, but leave amp on, so don't call cc2400_idle(). */
	cc2400_strobe(SRFOFF);
	while ((cc2400_status() & FS_LOCK)); // need to wait for unlock?

	/* Retune */
	cc2400_set(FSDIV, channel - 1);

	/* Update CS register if hopping.  */
	if (hop_mode > 0) {
		cs_threshold_calc_and_set();
	}

	/* Wait for lock */
	cc2400_strobe(SFSON);
	while (!(cc2400_status() & FS_LOCK));

	dma_discard = 1;

	/* RX mode */
	cc2400_strobe(SRX);
}

/* Bluetooth packet monitoring */
void bt_stream_rx()
{
	int8_t rssi;
	int8_t rssi_at_trigger;

	RXLED_CLR;

	queue_init();
	dio_ssp_init();
	dma_init();
	dio_ssp_start();

	cc2400_rx();

	cs_trigger_enable();

	while ( requested_mode == MODE_RX_SYMBOLS || requested_mode == MODE_BT_FOLLOW )
	{

		RXLED_CLR;

		/* Wait for DMA transfer. TODO - need more work on
		 * RSSI. Should send RSSI indications to host even
		 * when not transferring data. That would also keep
		 * the USB stream going. This loop runs 50-80 times
		 * while waiting for DMA, but RSSI sampling does not
		 * cover all the symbols in a DMA transfer. Can not do
		 * RSSI sampling in CS interrupt, but could log time
		 * at multiple trigger points there. The MAX() below
		 * helps with statistics in the case that cs_trigger
		 * happened before the loop started. */
		rssi_reset();
		rssi_at_trigger = INT8_MIN;
		while (!rx_tc) {
			rssi = (int8_t)(cc2400_get(RSSI) >> 8);
			if (cs_trigger && (rssi_at_trigger == INT8_MIN)) {
				rssi = MAX(rssi,(cs_threshold_cur+54));
				rssi_at_trigger = rssi;
			}
			rssi_add(rssi);

			handle_usb(clkn);

			/* If timer says time to hop, do it. */
			if (do_hop) {
				hop();
			} else {
				TXLED_CLR;
			}
			/* TODO - set per-channel carrier sense threshold.
			 * Set by firmware or host. */
			//wpson			
/*			int8_t freqest = (int8_t)(cc2400_get(FREQEST) >> 8);
			int i;

			int8_t bin[8];

			for (i = 0; i < 8; i++)
			{
				bin[i] = (freqest & 0x80) >> 7;
				freqest <<= 1;
			}
			int8_t dec = convert8(bin);
			double offset = (double)dec * 5.2;
			int8_t mod = (int8_t)(offset/15.625);
			u16 temp = (mod & 0x3f)<<7;
			cc2400_set(MDMCTRL, (u16)(temp + 0x0029));
*/		}

		RXLED_SET;

		if (rx_err) {
			status |= DMA_ERROR;
		}

		/* Missed a DMA trasfer? */
		if (rx_tc > 1)
			status |= DMA_OVERFLOW;

		if (dma_discard) {
			status |= DISCARD;
			dma_discard = 0;
		}

		rssi_iir_update();

		/* Set squelch hold if there was either a CS trigger, squelch
		 * is disabled, or if the current rssi_max is above the same
		 * threshold. Currently, this is redundant, but allows for
		 * per-channel or other rssi triggers in the future. */
		if (cs_trigger || cs_no_squelch) {
			status |= CS_TRIGGER;
			cs_trigger = 0;
		}

		if (rssi_max >= (cs_threshold_cur + 54)) {
			status |= RSSI_TRIGGER;
		}

		enqueue(BR_PACKET, (uint8_t*)idle_rxbuf);

	rx_continue:
		handle_usb(clkn);
		rx_tc = 0;
		rx_err = 0;
	}

	/* This call is a nop so far. Since bt_rx_stream() starts the
	 * stream, it makes sense that it would stop it. TODO - how
	 * should setup/teardown be handled? Should every new mode be
	 * starting from scratch? */
	dio_ssp_stop();
	cs_trigger_disable();
}

/* set LE access address */
static void le_set_access_address(u32 aa) {
	u32 aa_rev;

	le.access_address = aa;
	aa_rev = rbit(aa);
	le.syncl = aa_rev & 0xffff;
	le.synch = aa_rev >> 16;
}

/* reset le state, called by bt_generic_le and bt_follow_le() */
void reset_le() {
	le_set_access_address(0x8e89bed6);     // advertising channel access address
	le.crc_init  = 0x555555;               // advertising channel CRCInit
	le.crc_init_reversed = 0xAAAAAA;
	le.crc_verify = 1;
	le.last_packet = 0;

	le.link_state = LINK_INACTIVE;

	le.channel_idx = 0;
	le.channel_increment = 0;

	le.conn_epoch = 0;
	le.interval_timer = 0;
	le.conn_interval = 0;
	le.conn_interval = 0;
	le.conn_count = 0;

	le.win_size = 0;
	le.win_offset = 0;

	le.update_pending = 0;
	le.update_instant = 0;
	le.interval_update = 0;
	le.win_size_update = 0;
	le.win_offset_update;

	do_hop = 0;
}

// reset LE Promisc state
void reset_le_promisc(void) {
	memset(&le_promisc, 0, sizeof(le_promisc));
	le_promisc.smallest_hop_interval = 0xffffffff;
}

/* generic le mode */
void bt_generic_le(u8 active_mode)
{
	u8 *tmp = NULL;
	u8 hold;
	int i, j;
	int8_t rssi, rssi_at_trigger;

	modulation = MOD_BT_LOW_ENERGY;
	mode = active_mode;

	reset_le();

	// enable USB interrupts
	ISER0 = ISER0_ISE_USB;

	RXLED_CLR;

	queue_init();
	dio_ssp_init();
	dma_init();
	dio_ssp_start();
	cc2400_rx();

	cs_trigger_enable();

	hold = 0;

	while (requested_mode == active_mode) {
		if (requested_channel != 0) {
			cc2400_strobe(SRFOFF);
			while ((cc2400_status() & FS_LOCK)); // need to wait for unlock?

			/* Retune */
			cc2400_set(FSDIV, channel - 1);

			/* Wait for lock */
			cc2400_strobe(SFSON);
			while (!(cc2400_status() & FS_LOCK));

			/* RX mode */
			cc2400_strobe(SRX);

			requested_channel = 0;
		}

		if (do_hop) {
			hop();
		} else {
			TXLED_CLR;
		}

		RXLED_CLR;

		/* Wait for DMA. Meanwhile keep track of RSSI. */
		rssi_reset();
		rssi_at_trigger = INT8_MIN;
		while ((rx_tc == 0) && (rx_err == 0))
		{
			rssi = (int8_t)(cc2400_get(RSSI) >> 8);
			if (cs_trigger && (rssi_at_trigger == INT8_MIN)) {
				rssi = MAX(rssi,(cs_threshold_cur+54));
				rssi_at_trigger = rssi;
			}
			rssi_add(rssi);
		}

		if (rx_err) {
			status |= DMA_ERROR;
		}

		/* No DMA transfer? */
		if (!rx_tc)
			goto rx_continue;

		/* Missed a DMA trasfer? */
		if (rx_tc > 1)
			status |= DMA_OVERFLOW;

		rssi_iir_update();

		/* Set squelch hold if there was either a CS trigger, squelch
		 * is disabled, or if the current rssi_max is above the same
		 * threshold. Currently, this is redundant, but allows for
		 * per-channel or other rssi triggers in the future. */
		if (cs_trigger || cs_no_squelch) {
			status |= CS_TRIGGER;
			hold = CS_HOLD_TIME;
			cs_trigger = 0;
		}

		if (rssi_max >= (cs_threshold_cur + 54)) {
			status |= RSSI_TRIGGER;
			hold = CS_HOLD_TIME;
		}

		/* Hold expired? Ignore data. */
		if (hold == 0) {
			goto rx_continue;
		}
		hold--;

		// copy the previously unpacked symbols to the front of the buffer
		memcpy(unpacked, unpacked + DMA_SIZE*8, DMA_SIZE*8);

		// unpack the new packet to the end of the buffer
		for (i = 0; i < DMA_SIZE; ++i) {
			/* output one byte for each received symbol (0x00 or 0x01) */
			for (j = 0; j < 8; ++j) {
				unpacked[DMA_SIZE*8 + i * 8 + j] = (idle_rxbuf[i] & 0x80) >> 7;
				idle_rxbuf[i] <<= 1;
			}
		}

		int ret = data_cb(unpacked);
		if (!ret) break;

	rx_continue:
		rx_tc = 0;
		rx_err = 0;
	}

	// disable USB interrupts
	ICER0 = ICER0_ICE_USB;

	// reset the radio completely
	cc2400_idle();
	dio_ssp_stop();
	cs_trigger_disable();
}


void bt_le_sync(u8 active_mode)
{
	int i;
	int8_t rssi;
	static int restart_jamming = 0;

	modulation = MOD_BT_LOW_ENERGY;
	mode = active_mode;

	le.link_state = LINK_LISTENING;

	// enable USB interrupts
	ISER0 = ISER0_ISE_USB;

	RXLED_CLR;

	queue_init();
	dio_ssp_init();
	dma_init_le();
	dio_ssp_start();

	cc2400_rx_sync(rbit(le.access_address)); // bit-reversed access address

	while (requested_mode == active_mode) {
		if (requested_channel != 0) {
			cc2400_strobe(SRFOFF);
			while ((cc2400_status() & FS_LOCK)); // need to wait for unlock?

			/* Retune */
			cc2400_set(FSDIV, channel - 1);

			/* Wait for lock */
			cc2400_strobe(SFSON);
			while (!(cc2400_status() & FS_LOCK));

			/* RX mode */
			cc2400_strobe(SRX);

			saved_request = requested_channel;
			requested_channel = 0;
		}

		RXLED_CLR;

		/* Wait for DMA. Meanwhile keep track of RSSI. */
		rssi_reset();
		while ((rx_tc == 0) && (rx_err == 0) && (do_hop == 0) && requested_mode == active_mode)
			;

		rssi = (int8_t)(cc2400_get(RSSI) >> 8);
		rssi_min = rssi_max = rssi;

		if (requested_mode != active_mode) {
			goto cleanup;
		}

		if (rx_err) {
			status |= DMA_ERROR;
		}

		if (do_hop)
			goto rx_flush;

		/* No DMA transfer? */
		if (!rx_tc)
			continue;

		/////////////////////
		// process the packet

		uint32_t packet[48/4+1];
		u8 *p = (u8 *)packet;
		packet[0] = le.access_address;

		const uint32_t *whit = whitening_word[btle_channel_index(channel-2402)];
		for (i = 0; i < 4; i+= 4) {
			uint32_t v = rxbuf1[i+0] << 24
					   | rxbuf1[i+1] << 16
					   | rxbuf1[i+2] << 8
					   | rxbuf1[i+3] << 0;
			packet[i/4+1] = rbit(v) ^ whit[i/4];
		}

		unsigned len = (p[5] & 0x3f) + 2;
		if (len > 39)
			goto rx_flush;

		// transfer the minimum number of bytes from the CC2400
		// this allows us enough time to resume RX for subsequent packets on the same channel
		unsigned total_transfers = ((len + 3) + 4 - 1) / 4;
		if (total_transfers < 11) {
			while (DMACC0DestAddr < (uint32_t)rxbuf1 + 4 * total_transfers && rx_err == 0)
				;
		} else { // max transfers? just wait till DMA's done
			while (DMACC0Config & DMACCxConfig_E && rx_err == 0)
				;
		}
		DIO_SSP_DMACR &= ~SSPDMACR_RXDMAE;

		// unwhiten the rest of the packet
		for (i = 4; i < 44; i += 4) {
			uint32_t v = rxbuf1[i+0] << 24
					   | rxbuf1[i+1] << 16
					   | rxbuf1[i+2] << 8
					   | rxbuf1[i+3] << 0;
			packet[i/4+1] = rbit(v) ^ whit[i/4];
		}

		if (le.crc_verify) {
			u32 calc_crc = btle_crcgen_lut(le.crc_init_reversed, p + 4, len);
			u32 wire_crc = (p[4+len+2] << 16)
						 | (p[4+len+1] << 8)
						 | (p[4+len+0] << 0);
			if (calc_crc != wire_crc) // skip packets with a bad CRC
				goto rx_flush;
		}


		RXLED_SET;
		packet_cb((uint8_t *)packet);
		enqueue(LE_PACKET, (uint8_t *)packet);
		le.last_packet = CLK100NS;

	rx_flush:
		cc2400_strobe(SFSON);
		while (!(cc2400_status() & FS_LOCK));

		// flush any excess bytes from the SSP's buffer
		DIO_SSP_DMACR &= ~SSPDMACR_RXDMAE;
		while (SSP1SR & SSPSR_RNE) {
			u8 tmp = (u8)DIO_SSP_DR;
		}

		// timeout - FIXME this is an ugly hack
		u32 now = CLK100NS;
		if (now < le.last_packet)
			now += 3276800000; // handle rollover
		if  ( // timeout
			((le.link_state == LINK_CONNECTED || le.link_state == LINK_CONN_PENDING)
			&& (now - le.last_packet > 50000000))
			// jam finished
			|| (le_jam_count == 1)
			)
		{
			reset_le();
			le_jam_count = 0;
			TXLED_CLR;

			if (jam_mode == JAM_ONCE) {
				jam_mode = JAM_NONE;
				requested_mode = MODE_IDLE;
				goto cleanup;
			}

			// go back to promisc if the connection dies
			if (active_mode == MODE_BT_PROMISC_LE)
				goto cleanup;

			le.link_state = LINK_LISTENING;

			cc2400_strobe(SRFOFF);
			while ((cc2400_status() & FS_LOCK));

			/* Retune */
			channel = saved_request != 0 ? saved_request : 2402;
			restart_jamming = 1;
		}

		cc2400_set(SYNCL, le.syncl);
		cc2400_set(SYNCH, le.synch);

		if (do_hop)
			hop();

		// ♪ you can jam but you keep turning off the light ♪
		if (le_jam_count > 0) {
			le_jam();
			--le_jam_count;
		} else {
			/* RX mode */
			dma_init_le();
			dio_ssp_start();

			if (restart_jamming) {
				cc2400_rx_sync(rbit(le.access_address));
				restart_jamming = 0;
			} else {
				cc2400_strobe(SRX);
			}
		}

		rx_tc = 0;
		rx_err = 0;
	}

cleanup:

	// disable USB interrupts
	ICER0 = ICER0_ICE_USB;

	// reset the radio completely
	cc2400_idle();
	dio_ssp_stop();
	cs_trigger_disable();
}



/* low energy connection following
 * follows a known AA around */
int cb_follow_le() {
	int i, j, k;
	int idx = whitening_index[btle_channel_index(channel-2402)];

	u32 access_address = 0;
	for (i = 0; i < 31; ++i) {
		access_address >>= 1;
		access_address |= (unpacked[i] << 31);
	}

	for (i = 31; i < DMA_SIZE * 8 + 32; i++) {
		access_address >>= 1;
		access_address |= (unpacked[i] << 31);
		if (access_address == le.access_address) {
			for (j = 0; j < 46; ++j) {
				u8 byte = 0;
				for (k = 0; k < 8; k++) {
					int offset = k + (j * 8) + i - 31;
					if (offset >= DMA_SIZE*8*2) break;
					int bit = unpacked[offset];
					if (j >= 4) { // unwhiten data bytes
						bit ^= whitening[idx];
						idx = (idx + 1) % sizeof(whitening);
					}
					byte |= bit << k;
				}
				idle_rxbuf[j] = byte;
			}

			// verify CRC
			if (le.crc_verify) {
				int len		 = (idle_rxbuf[5] & 0x3f) + 2;
				u32 calc_crc = btle_crcgen_lut(le.crc_init_reversed, (uint8_t*)idle_rxbuf + 4, len);
				u32 wire_crc = (idle_rxbuf[4+len+2] << 16)
							 | (idle_rxbuf[4+len+1] << 8)
							 |  idle_rxbuf[4+len+0];
				if (calc_crc != wire_crc) // skip packets with a bad CRC
					break;
			}

			// send to PC
			enqueue(LE_PACKET, (uint8_t*)idle_rxbuf);
			RXLED_SET;

			packet_cb((uint8_t*)idle_rxbuf);

			break;
		}
	}

	return 1;
}

/**
 * Called when we receive a packet in connection following mode.
 */
void connection_follow_cb(u8 *packet) {
	int i;
	u32 aa = 0;

#define ADV_ADDRESS_IDX 0
#define HEADER_IDX 4
#define DATA_LEN_IDX 5
#define DATA_START_IDX 6

	u8 *adv_addr = &packet[ADV_ADDRESS_IDX];
	u8 header = packet[HEADER_IDX];
	u8 *data_len = &packet[DATA_LEN_IDX];
	u8 *data = &packet[DATA_START_IDX];
	u8 *crc = &packet[DATA_START_IDX + *data_len];

	if (le.link_state == LINK_CONN_PENDING) {
		// We received a packet in the connection pending state, so now the device *should* be connected
		le.link_state = LINK_CONNECTED;
		le.conn_epoch = clkn;
		le.interval_timer = le.conn_interval - 1;
		le.conn_count = 0;
		le.update_pending = 0;

		// hue hue hue
		if (jam_mode != JAM_NONE)
			le_jam_count = JAM_COUNT_DEFAULT;

	} else if (le.link_state == LINK_CONNECTED) {
		u8 llid =  header & 0x03;

		// Apply any connection parameter update if necessary
		if (le.update_pending && le.conn_count == le.update_instant) {
			// This is the first packet received in the connection interval for which the new parameters apply
			le.conn_epoch = clkn;
			le.conn_interval = le.interval_update;
			le.interval_timer = le.interval_update - 1;
			le.win_size = le.win_size_update;
			le.win_offset = le.win_offset_update;
			le.update_pending = 0;
		}

		if (llid == 0x03 && data[0] == 0x00) {
			// This is a CONNECTION_UPDATE_REQ.
			// The host is changing the connection parameters.
			le.win_size_update = packet[7];
			le.win_offset_update = packet[8] + ((u16)packet[9] << 8);
			le.interval_update = packet[10] + ((u16)packet[11] << 8);
			le.update_instant = packet[16] + ((u16)packet[17] << 8);
			if (le.update_instant - le.conn_count < 32767)
				le.update_pending = 1;
		}

	} else if (le.link_state == LINK_LISTENING) {
		u8 pkt_type = packet[4] & 0x0F;
		if (pkt_type == 0x05) {
			// This is a connect packet
			// if we have a target, see if InitA or AdvA matches
			if (le.target_set &&
				memcmp(le.target, &packet[6], 6) &&  // Target address doesn't match Initiator.
				memcmp(le.target, &packet[12], 6)) {  // Target address doesn't match Advertiser.
				return;
			}

			le.link_state = LINK_CONN_PENDING;
			le.crc_verify = 0; // we will drop many packets if we attempt to filter by CRC

			for (i = 0; i < 4; ++i)
				aa |= packet[18+i] << (i*8);
			le_set_access_address(aa);

#define CRC_INIT (2+4+6+6+4)
			le.crc_init = (packet[CRC_INIT+2] << 16)
						| (packet[CRC_INIT+1] << 8)
						|  packet[CRC_INIT+0];
			le.crc_init_reversed = rbit(le.crc_init);

#define WIN_SIZE (2+4+6+6+4+3)
			le.win_size = packet[WIN_SIZE];

#define WIN_OFFSET (2+4+6+6+4+3+1)
			le.win_offset = packet[WIN_OFFSET];

#define CONN_INTERVAL (2+4+6+6+4+3+1+2)
			le.conn_interval = packet[CONN_INTERVAL];

#define CHANNEL_INC (2+4+6+6+4+3+1+2+2+2+2+5)
			le.channel_increment = packet[CHANNEL_INC] & 0x1f;
			le.channel_idx = le.channel_increment;

			// Hop to the initial channel immediately
			do_hop = 1;
		}
	}
}

void bt_follow_le() {
	reset_le();
	packet_cb = connection_follow_cb;
	bt_le_sync(MODE_BT_FOLLOW_LE);

	/* old non-sync mode
	data_cb = cb_follow_le;
	packet_cb = connection_follow_cb;
	bt_generic_le(MODE_BT_FOLLOW_LE);
	*/

	mode = MODE_IDLE;
}

// issue state change message
void le_promisc_state(u8 type, void *data, unsigned len) {
	u8 buf[50] = { 0, };
	if (len > 49)
		len = 49;

	buf[0] = type;
	memcpy(&buf[1], data, len);
	enqueue(LE_PROMISC, (uint8_t*)buf);
}

// divide, rounding to the nearest integer: round up at 0.5.
#define DIVIDE_ROUND(N, D) ((N) + (D)/2) / (D)

void promisc_recover_hop_increment(u8 *packet) {
	static u32 first_ts = 0;
	if (channel == 2404) {
		first_ts = CLK100NS;
		hop_direct_channel = 2406;
		do_hop = 1;
	} else if (channel == 2406) {
		u32 second_ts = CLK100NS;
		if (second_ts < first_ts)
			second_ts += 3276800000; // handle rollover
		// Number of channels hopped between previous and current timestamp.
		u32 channels_hopped = DIVIDE_ROUND(second_ts - first_ts,
										   le.conn_interval * LE_BASECLK);
		if (channels_hopped < 37) {
			// Get the hop increment based on the number of channels hopped.
			le.channel_increment = hop_interval_lut[channels_hopped];
			le.interval_timer = le.conn_interval / 2;
			le.conn_count = 0;
			le.conn_epoch = 0;
			do_hop = 0;
			// Move on to regular connection following.
			le.channel_idx = (1 + le.channel_increment) % 37;
			le.link_state = LINK_CONNECTED;
			le.crc_verify = 0;
			hop_mode = HOP_BTLE;
			packet_cb = connection_follow_cb;
			le_promisc_state(3, &le.channel_increment, 1);

			if (jam_mode != JAM_NONE)
				le_jam_count = JAM_COUNT_DEFAULT;

			return;
		}
		hop_direct_channel = 2404;
		do_hop = 1;
	}
	else {
		hop_direct_channel = 2404;
		do_hop = 1;
	}
}

void promisc_recover_hop_interval(u8 *packet) {
	static u32 prev_clk = 0;

	u32 cur_clk = CLK100NS;
	if (cur_clk < prev_clk)
		cur_clk += 3267800000; // handle rollover
	u32 clk_diff = cur_clk - prev_clk;
	u16 obsv_hop_interval; // observed hop interval

	// probably consecutive data packets on the same channel
	if (clk_diff < 2 * LE_BASECLK)
		return;

	if (clk_diff < le_promisc.smallest_hop_interval)
		le_promisc.smallest_hop_interval = clk_diff;

	obsv_hop_interval = DIVIDE_ROUND(le_promisc.smallest_hop_interval, 37 * LE_BASECLK);

	if (le.conn_interval == obsv_hop_interval) {
		// 5 consecutive hop intervals: consider it legit and move on
		++le_promisc.consec_intervals;
		if (le_promisc.consec_intervals == 5) {
			packet_cb = promisc_recover_hop_increment;
			hop_direct_channel = 2404;
			hop_mode = HOP_DIRECT;
			do_hop = 1;
			le_promisc_state(2, &le.conn_interval, 2);
		}
	} else {
		le.conn_interval = obsv_hop_interval;
		le_promisc.consec_intervals = 0;
	}

	prev_clk = cur_clk;
}

void promisc_follow_cb(u8 *packet) {
	int i;

	// get the CRCInit
	if (!le.crc_verify && packet[4] == 0x01 && packet[5] == 0x00) {
		u32 crc = (packet[8] << 16) | (packet[7] << 8) | packet[6];

		le.crc_init = btle_reverse_crc(crc, packet + 4, 2);
		le.crc_init_reversed = 0;
		for (i = 0; i < 24; ++i)
			le.crc_init_reversed |= ((le.crc_init >> i) & 1) << (23 - i);

		le.crc_verify = 1;
		packet_cb = promisc_recover_hop_interval;
		le_promisc_state(1, &le.crc_init, 3);
	}
}

// called when we see an AA, add it to the list
void see_aa(u32 aa) {
	int i, max = -1, killme = -1;
	for (i = 0; i < AA_LIST_SIZE; ++i)
		if (le_promisc.active_aa[i].aa == aa) {
			++le_promisc.active_aa[i].count;
			return;
		}

	// evict someone
	for (i = 0; i < AA_LIST_SIZE; ++i)
		if (le_promisc.active_aa[i].count < max || max < 0) {
			killme = i;
			max = le_promisc.active_aa[i].count;
		}

	le_promisc.active_aa[killme].aa = aa;
	le_promisc.active_aa[killme].count = 1;
}

/* le promiscuous mode */
int cb_le_promisc(char *unpacked) {
	int i, j, k;
	int idx;

	// empty data PDU: 01 00
	char desired[4][16] = {
		{ 1, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
		{ 1, 0, 0, 1, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
		{ 1, 0, 1, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
		{ 1, 0, 1, 1, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
	};

	for (i = 0; i < 4; ++i) {
		idx = whitening_index[btle_channel_index(channel-2402)];

		// whiten the desired data
		for (j = 0; j < (int)sizeof(desired[i]); ++j) {
			desired[i][j] ^= whitening[idx];
			idx = (idx + 1) % sizeof(whitening);
		}
	}

	// then look for that bitsream in our receive buffer
	for (i = 32; i < (DMA_SIZE*8*2 - 32 - 16); i++) {
		int ok[4] = { 1, 1, 1, 1 };
		int matching = -1;

		for (j = 0; j < 4; ++j) {
			for (k = 0; k < (int)sizeof(desired[j]); ++k) {
				if (unpacked[i+k] != desired[j][k]) {
					ok[j] = 0;
					break;
				}
			}
		}

		// see if any match
		for (j = 0; j < 4; ++j) {
			if (ok[j]) {
				matching = j;
				break;
			}
		}

		// skip if no match
		if (matching < 0)
			continue;

		// found a match! unwhiten it and send it home
		idx = whitening_index[btle_channel_index(channel-2402)];
		for (j = 0; j < 4+3+3; ++j) {
			u8 byte = 0;
			for (k = 0; k < 8; k++) {
				int offset = k + (j * 8) + i - 32;
				if (offset >= DMA_SIZE*8*2) break;
				int bit = unpacked[offset];
				if (j >= 4) { // unwhiten data bytes
					bit ^= whitening[idx];
					idx = (idx + 1) % sizeof(whitening);
				}
				byte |= bit << k;
			}
			idle_rxbuf[j] = byte;
		}

		u32 aa = (idle_rxbuf[3] << 24) |
				 (idle_rxbuf[2] << 16) |
				 (idle_rxbuf[1] <<  8) |
				 (idle_rxbuf[0]);
		see_aa(aa);

		enqueue(LE_PACKET, (uint8_t*)idle_rxbuf);

	}

	// once we see an AA 5 times, start following it
	for (i = 0; i < AA_LIST_SIZE; ++i) {
		if (le_promisc.active_aa[i].count > 3) {
			le_set_access_address(le_promisc.active_aa[i].aa);
			data_cb = cb_follow_le;
			packet_cb = promisc_follow_cb;
			le.crc_verify = 0;
			le_promisc_state(0, &le.access_address, 4);
			// quit using the old stuff and switch to sync mode
			return 0;
		}
	}

	return 1;
}

void bt_promisc_le() {
	while (requested_mode == MODE_BT_PROMISC_LE) {
		reset_le_promisc();

		// jump to a random data channel and turn up the squelch
		if ((channel & 1) == 1)
			channel = 2440;

		// if the PC hasn't given us AA, determine by listening
		if (!le.target_set) {
			// cs_threshold_req = -80;
			cs_threshold_calc_and_set();
			data_cb = cb_le_promisc;
			bt_generic_le(MODE_BT_PROMISC_LE);
		}

		// could have got mode change in middle of above
		if (requested_mode != MODE_BT_PROMISC_LE)
			break;

		le_promisc_state(0, &le.access_address, 4);
		packet_cb = promisc_follow_cb;
		le.crc_verify = 0;
		bt_le_sync(MODE_BT_PROMISC_LE);
	}
}

void bt_slave_le() {
	u32 calc_crc;
	int i;

	u8 adv_ind[] = {
		// LL header
		0x00, 0x09,

		// advertising address
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

		// advertising data
		0x02, 0x01, 0x05,

		// CRC (calc)
		0xff, 0xff, 0xff,
	};

	u8 adv_ind_len = sizeof(adv_ind) - 3;

	// copy the user-specified mac address
	for (i = 0; i < 6; ++i)
		adv_ind[i+2] = slave_mac_address[5-i];

	calc_crc = btle_calc_crc(le.crc_init_reversed, adv_ind, adv_ind_len);
	adv_ind[adv_ind_len+0] = (calc_crc >>  0) & 0xff;
	adv_ind[adv_ind_len+1] = (calc_crc >>  8) & 0xff;
	adv_ind[adv_ind_len+2] = (calc_crc >> 16) & 0xff;

	// spam advertising packets
	while (requested_mode == MODE_BT_SLAVE_LE) {
		ICER0 = ICER0_ICE_USB;
		ICER0 = ICER0_ICE_DMA;
		le_transmit(0x8e89bed6, adv_ind_len+3, adv_ind);
		ISER0 = ISER0_ISE_USB;
		ISER0 = ISER0_ISE_DMA;
		msleep(100);
	}
}

/* spectrum analysis */
void specan()
{
	u16 f;
	u8 i = 0;
	u8 buf[DMA_SIZE];

	RXLED_SET;

	queue_init();

#ifdef UBERTOOTH_ONE
	PAEN_SET;
	//HGM_SET;
#endif
	cc2400_set(LMTST,   0x2b22);
	cc2400_set(MDMTST0, 0x134b); // without PRNG
	cc2400_set(GRMDM,   0x0101); // un-buffered mode, GFSK
	cc2400_set(MDMCTRL, 0x0029); // 160 kHz frequency deviation
	//FIXME maybe set RSSI.RSSI_FILT
	while (!(cc2400_status() & XOSC16M_STABLE));
	while ((cc2400_status() & FS_LOCK));

	while (requested_mode == MODE_SPECAN) {
		for (f = low_freq; f < high_freq + 1; f++) {
			cc2400_set(FSDIV, f - 1);
			cc2400_strobe(SFSON);
			while (!(cc2400_status() & FS_LOCK));
			cc2400_strobe(SRX);

			/* give the CC2400 time to acquire RSSI reading */
			volatile u32 j = 500; while (--j); //FIXME crude delay
			buf[3 * i] = (f >> 8) & 0xFF;
			buf[(3 * i) + 1] = f  & 0xFF;
			buf[(3 * i) + 2] = cc2400_get(RSSI) >> 8;
			i++;
			if (i == 16) {
				enqueue(SPECAN, buf);
				i = 0;

				handle_usb(clkn);
			}

			cc2400_strobe(SRFOFF);
			while ((cc2400_status() & FS_LOCK));
		}
	}
	RXLED_CLR;
}

/* LED based spectrum analysis */
void led_specan()
{
	int8_t lvl;
	u8 i = 0;
	u16 channels[3] = {2412, 2437, 2462};
	//void (*set[3]) = {TXLED_SET, RXLED_SET, USRLED_SET};
	//void (*clr[3]) = {TXLED_CLR, RXLED_CLR, USRLED_CLR};

#ifdef UBERTOOTH_ONE
	PAEN_SET;
	//HGM_SET;
#endif
	cc2400_set(LMTST,   0x2b22);
	cc2400_set(MDMTST0, 0x134b); // without PRNG
	cc2400_set(GRMDM,   0x0101); // un-buffered mode, GFSK
	cc2400_set(MDMCTRL, 0x0029); // 160 kHz frequency deviation
	cc2400_set(RSSI,    0x00F1); // RSSI Sample over 2 symbols

	while (!(cc2400_status() & XOSC16M_STABLE));
	while ((cc2400_status() & FS_LOCK));

	while (requested_mode == MODE_LED_SPECAN) {
		cc2400_set(FSDIV, channels[i] - 1);
		cc2400_strobe(SFSON);
		while (!(cc2400_status() & FS_LOCK));
		cc2400_strobe(SRX);

		/* give the CC2400 time to acquire RSSI reading */
		volatile u32 j = 500; while (--j); //FIXME crude delay
		lvl = cc2400_get(RSSI) >> 8;
		if (lvl > rssi_threshold) {
			switch (i) {
				case 0:
					TXLED_SET;
					break;
				case 1:
					RXLED_SET;
					break;
				case 2:
					USRLED_SET;
					break;
			}
		}
		else {
			switch (i) {
				case 0:
					TXLED_CLR;
					break;
				case 1:
					RXLED_CLR;
					break;
				case 2:
					USRLED_CLR;
					break;
			}
		}

		i = (i+1) % 3;

		handle_usb(clkn);

		cc2400_strobe(SRFOFF);
		while ((cc2400_status() & FS_LOCK));
	}
}

void cc2400_txtest(volatile u8 *mod_ptr, volatile u16 *chan_ptr)
{
#ifdef TX_ENABLE

        u8 id_pkt1[9] = {0x54, 0x75, 0xc5, 0x8c, 0xc7, 0x33, 0x45, 0xe7, 0x2a};
// 
/*
        cc2400_set(MANAND, 0x7fff);
        cc2400_set(LMTST,   0x2b22);
        cc2400_set(MDMTST0, 0x164b);
        cc2400_set(FSDIV,   2441 - 1);
//        cc2400_set(SYNCH,   0xf9ae);
//        cc2400_set(SYNCL,   0x1584);
        cc2400_set(MDMCTRL, 0x0040); // bandwidth
        cc2400_set(GRMDM, 0x0f60);

        while (!(cc2400_status() & XOSC16M_STABLE));
        cc2400_strobe(SFSON);
        while (!(cc2400_status() & FS_LOCK));
        RXLED_SET;
        TXLED_CLR;
        USRLED_CLR;
#ifdef UBERTOOTH_ONE
        PAEN_SET;
        HGM_SET;
#endif
        cc2400_strobe(SRX);

        int delay = 1000;
        while (delay) delay--;

//        while (!(cc2400_status() & SYNC_RECEIVED));

        clkn_init();
        clkn = 0x00000000;

        RXLED_CLR;
        cc2400_strobe(SRFOFF);
        while ((cc2400_status() & FS_LOCK));
//
*/
        cc2400_set(LMTST, 0x2b22);
        cc2400_set(MDMTST0, 0x134b); // wo PRNG
        cc2400_set(GRMDM,   0x0f01); // 0 preamble, 8 bit cync word
//        cc2400_set(GRMDM,   0x0f61); // 0 preamble, 8 bit cync word
//        cc2400_set(GRMDM,   0x0f61); // 0 preamble, 8 bit cync word
//        cc2400_set(MDMCTRL, 0x0040); // 250 kHz 
      	cc2400_set(MDMCTRL, 0x0029);  
        cc2400_set(INT, 0x000b);

//        u8 pa = 7;
//      cc2400_set(FREND, pa);

        TXLED_SET;
#ifdef UBERTOOTH_ONE
//        PAEN_SET;
#endif
        cc2400_set (FSDIV, channel);
        while (!(cc2400_status() & XOSC16M_STABLE));
        cc2400_strobe(SFSON);
        while (!(cc2400_status() & FS_LOCK));
 
	for (int i = 0; i < 50000; i++)
//	while (1)
        {
	        cc2400_spi_buf(FIFOREG, 9, id_pkt1);
		while (clkn % 2 == 1); 
                cc2400_strobe (STX);
                while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
        }
        TXLED_CLR;
#ifdef UBERTOOTH_ONE
        PAEN_CLR;
#endif

#endif

}




int main()
{
	ubertooth_init();
	clkn_init();
	ubertooth_usb_init(vendor_request_handler);

	while (1) {
		handle_usb(clkn);
		if(requested_mode != mode) {
			switch (requested_mode) {
				 case MODE_RESET:
					/* Allow time for the USB command to return correctly */
					wait(1);
					reset();
					break;
				case MODE_AFH:
					mode = MODE_AFH;
					bt_stream_rx();
					break;
				case MODE_RX_SYMBOLS:
					mode = MODE_RX_SYMBOLS;
					queue_init();
					bt_stream_rx();
					break;
				case MODE_BT_FOLLOW:
					mode = MODE_BT_FOLLOW;
					bt_stream_rx();
					break;
				case MODE_BT_FOLLOW_LE:
					bt_follow_le();
					break;
				case MODE_BT_PROMISC_LE:
					bt_promisc_le();
					break;
				case MODE_BT_SLAVE_LE:
					bt_slave_le();
					break;
				case MODE_TX_TEST:
					mode = MODE_TX_TEST;
					cc2400_txtest(&modulation, &channel);
					requested_mode = MODE_IDLE;
					break;
				case MODE_RANGE_TEST:
					mode = MODE_RANGE_TEST;
					cc2400_rangetest(&channel);
					requested_mode = MODE_IDLE;
					break;
				case MODE_REPEATER:
					mode = MODE_REPEATER;
					cc2400_repeater(&channel);
					break;
				case MODE_SPECAN:
					specan();
					break;
				case MODE_LED_SPECAN:
					led_specan();
					break;
				case MODE_EGO:
					mode = MODE_EGO;
					ego_main(ego_mode);
					break;
				case MODE_IDLE:
					cc2400_idle();
					break;
				default:
					/* This is really an error state, but what can you do? */
					break;
			}
		}
	}
}
