#include "same70_can_driver.h"

//#define MCAN0_INT1_DEBUG
//#define MCAN1_INT1_DEBUG

void _mcan_configure_rx_fifo_to_accept_all(struct mcan_module* module_inst)
{
	/*  
	 *  Setup rx filtering to accept messages into FIFO1 with extended format
	 *  this accepts all messages
	 */
	struct mcan_extended_message_filter_element et_filter;

	mcan_get_extended_message_filter_element_default(&et_filter);
	et_filter.F0.bit.EFID1 = 0;//id
	et_filter.F1.bit.EFID2 = 0;//mask

	et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF1M_Val;//Put in fifo1
	et_filter.F1.bit.EFT = 2;//classic filter

	mcan_set_rx_extended_filter(module_inst, &et_filter, 0);

	/*  
	 *  Setup rx filtering to accept messages into FIFO0 with standard format
	 *  this accepts all messages
	 */
	struct mcan_standard_message_filter_element sd_filter;
	mcan_get_standard_message_filter_element_default(&sd_filter);
	sd_filter.S0.bit.SFID1 = 0;//ID
	sd_filter.S0.bit.SFID2 = 0;//Mask
	sd_filter.S0.bit.SFEC = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STF0M_Val;//Put in fifo0
	sd_filter.S0.bit.SFT = 2;//classic filter
	mcan_set_rx_standard_filter(module_inst, &sd_filter, 0);
}

void mcan0_configure(uint32_t baudrate, uint32_t rx_buffer_size, uint32_t tx_buffer_size)
{	
	mcan0_standard_receive_index = 0;
	mcan0_extended_receive_index = 0;

	circ_buf_flex_init_buffer(&mcan0_buffer.buffer_rx, rx_buffer_size, sizeof(mcan_timestamped_rx_message_t));
	circ_buf_flex_init_buffer(&mcan0_buffer.buffer_tx, tx_buffer_size, sizeof(mcan_tx_message_t));
	
	mcan0_buffer.adding_in_tx_buffer = false;
	mcan0_buffer.interruption_occurred_while_adding_in_tx_buffer = false;
	mcan0_buffer.buffer_being_emptied_by_interruption = false;

	struct mcan_config config_mcan;
	mcan_get_config_defaults(&config_mcan);
	config_mcan.nonmatching_frames_action_standard = MCAN_NONMATCHING_FRAMES_REJECT;
	config_mcan.nonmatching_frames_action_extended = MCAN_NONMATCHING_FRAMES_REJECT;

	mcan_init(&mcan0_instance, MCAN0, &config_mcan);

	//Enabling the upll clock 
	//NEED TO HAVE THE RIGHT conf_mcan.h !
	pmc_enable_upll_clock();
	// This was firstly changed to "pmc_switch_pck_to_pllack(PMC_PCK_5, PMC_PCK_PRES(9));" in the mcan_init function above.
	//It is recomended in the datasheet to use upllck as it is less subject to change. It is running at 480 MHz.
	pmc_disable_pck(PMC_PCK_5);
	//dividing upll by 6 to get a 80 Mhz signal which is again divided by 8 in the CONF_MCAN_NBTP_NBRP_VALUE in the conf_mcan.h to get a 10 MHz for 500kbits/s default speed.
	pmc_switch_pck_to_upllck(PMC_PCK_5, PMC_PCK_PRES(5));
	pmc_enable_pck(PMC_PCK_5);

	//has to be after set of PMC_PCK_5 as it uses it to calculate the MCAN_NBTP_NBRP_VALUE for the given speed.
	//MCAN_NBTP_NBRP_VALUE = (PMC_PCK_5 frequency)/(baudrate asked)/20
	//Here (PMC_PCK_5 frequency) = 80Mhz
	//Changing default speed.
	mcan_set_baudrate(mcan0_instance.hw, baudrate);


	//choosing what interruption to activate
	mcan_enable_interrupt(&mcan0_instance,
	 	MCAN_RX_FIFO_1_NEW_MESSAGE |
		MCAN_RX_BUFFER_NEW_MESSAGE |
		MCAN_RX_FIFO_0_NEW_MESSAGE |
		MCAN_FORMAT_ERROR |
		MCAN_ACKNOWLEDGE_ERROR |
		MCAN_BUS_OFF | 
		MCAN_TIMESTAMP_COMPLETE |
		MCAN_TX_CANCELLATION_FINISH | MCAN_TX_FIFO_EMPTY |
		MCAN_TX_EVENT_FIFO_NEW_ENTRY |
		MCAN_TX_EVENT_FIFO_WATERMARK |
		MCAN_TX_EVENT_FIFO_FULL |
		MCAN_TX_EVENT_FIFO_ELEMENT_LOST);
	mcan_disable_interrupt(&mcan0_instance, MCAN_TIMESTAMP_WRAPAROUND);
	
	//Interrupt Line Selection making all tx to the second line.
	//using int0 for reception and basic errors
	//using int1 for emission.
	mcan0_instance.hw->MCAN_ILS = 0x0000FF00ul;
	
	//activating interruptions for int0 and int1
	
	irq_register_handler(MCAN0_INT0_IRQn, 1);
	irq_register_handler(MCAN0_INT1_IRQn, 2);
	
	
	_mcan_configure_rx_fifo_to_accept_all(&mcan0_instance);

    mcan_start(&mcan0_instance);
}

void mcan1_configure(uint32_t baudrate, uint32_t rx_buffer_size, uint32_t tx_buffer_size)
{	
	mcan1_standard_receive_index = 0;
	mcan1_extended_receive_index = 0;

	circ_buf_flex_init_buffer(&mcan1_buffer.buffer_rx, rx_buffer_size, sizeof(mcan_timestamped_rx_message_t));
	circ_buf_flex_init_buffer(&mcan1_buffer.buffer_tx, tx_buffer_size, sizeof(mcan_tx_message_t));

	mcan1_buffer.adding_in_tx_buffer = false;
	mcan1_buffer.interruption_occurred_while_adding_in_tx_buffer = false;
	mcan1_buffer.buffer_being_emptied_by_interruption = false;

	struct mcan_config config_mcan;
	mcan_get_config_defaults(&config_mcan);
	config_mcan.nonmatching_frames_action_standard = MCAN_NONMATCHING_FRAMES_REJECT;
	config_mcan.nonmatching_frames_action_extended = MCAN_NONMATCHING_FRAMES_REJECT;
	config_mcan.tx_queue_mode = false;

	mcan_init(&mcan1_instance, MCAN1, &config_mcan);

	//Enabling the upll clock 
	//NEED TO HAVE THE RIGHT conf_mcan.h !
	pmc_enable_upll_clock();
	// This was firstly changed to "pmc_switch_pck_to_pllack(PMC_PCK_5, PMC_PCK_PRES(9));" in the mcan_init function above.
	//It is recomended in the datasheet to use upllck as it is less subject to change. It is running at 480 MHz.
	pmc_disable_pck(PMC_PCK_5);
	//dividing upll by 6 to get a 80 Mhz signal which is again divided by 8 in the CONF_MCAN_NBTP_NBRP_VALUE in the conf_mcan.h to get a 10 MHz for 500kbits/s default speed.
	pmc_switch_pck_to_upllck(PMC_PCK_5, PMC_PCK_PRES(5));
	pmc_enable_pck(PMC_PCK_5);

	//has to be after set of PMC_PCK_5 as it uses it to calculate the MCAN_NBTP_NBRP_VALUE for the given speed.
	//MCAN_NBTP_NBRP_VALUE = (PMC_PCK_5 frequency)/(baudrate asked)/20
	//Here (PMC_PCK_5 frequency) = 80Mhz
	//Changing default speed.
	mcan_set_baudrate(mcan1_instance.hw, baudrate);

	//choosing what interruption to activate
	mcan_enable_interrupt(&mcan1_instance,
	 	MCAN_RX_FIFO_1_NEW_MESSAGE |
		MCAN_RX_BUFFER_NEW_MESSAGE |
		MCAN_RX_FIFO_0_NEW_MESSAGE |
		MCAN_FORMAT_ERROR |
		MCAN_ACKNOWLEDGE_ERROR |
		MCAN_BUS_OFF | 
		MCAN_TIMESTAMP_COMPLETE |
		MCAN_TX_CANCELLATION_FINISH | MCAN_TX_FIFO_EMPTY |
		MCAN_TX_EVENT_FIFO_NEW_ENTRY |
		MCAN_TX_EVENT_FIFO_WATERMARK |
		MCAN_TX_EVENT_FIFO_FULL |
		MCAN_TX_EVENT_FIFO_ELEMENT_LOST);
	
	//TODO understand why I have put this line.
	mcan_disable_interrupt(&mcan1_instance, MCAN_TIMESTAMP_WRAPAROUND);
	
	//Interrupt Line Selection making all tx to the second line.
	//using int0 for reception and basic errors
	//using int1 for emission.
	mcan1_instance.hw->MCAN_ILS = 0x0000FF00ul;
	
	//activating interruptions for int0 and int1
	irq_register_handler(MCAN1_INT0_IRQn, 1);
	irq_register_handler(MCAN1_INT1_IRQn, 2);
	
	_mcan_configure_rx_fifo_to_accept_all(&mcan1_instance);

    mcan_start(&mcan1_instance);
}

void _mcan0_push_message(MCAN_RX_ELEMENT_R0_Type r0, MCAN_RX_ELEMENT_R1_Type r1, uint8_t* data, uint64_t rec_timestamp)
{
	mcan_timestamped_rx_message_t time_message;
	time_message.timestamp = rec_timestamp;

	mcan_rx_message_t message;
	message.message_info.all_8_bits = 0x00;
	message.message_info.bit.is_extended = r0.bit.XTD;
	message.message_info.bit.is_remote_request = r0.bit.RTR;
	message.message_info.bit.is_error = r0.bit.ESI;
	message.message_info.bit.is_fd = r1.bit.EDL;
	message.message_info.bit.is_received_with_bitrate_switch = r1.bit.BRS;

	if(message.message_info.bit.is_extended)
	{
		message.id = r0.bit.ID;
	}
	else
	{
		message.id = GET_BITS(r0.bit.ID, 18, 29);
	}
	message.dlc = r1.bit.DLC;
	for (uint8_t i = 0; i < message.dlc; i++)
	{
		message.data[i] = data[i];
	}
	

	time_message.rx_message = message;

	if(circ_buf_flex_push(&mcan0_buffer.buffer_rx, &time_message) != CBF_SUCCESS)
	{
		//TODO HANDLE ERROR 
	}
}

void _mcan1_push_message(MCAN_RX_ELEMENT_R0_Type r0, MCAN_RX_ELEMENT_R1_Type r1, uint8_t* data, uint64_t rec_timestamp)
{
	mcan_timestamped_rx_message_t time_message;
	time_message.timestamp = rec_timestamp;

	mcan_rx_message_t message;
	message.message_info.all_8_bits = 0x00;
	message.message_info.bit.is_extended = r0.bit.XTD;
	message.message_info.bit.is_remote_request = r0.bit.RTR;
	message.message_info.bit.is_error = r0.bit.ESI;
	message.message_info.bit.is_fd = r1.bit.EDL;
	message.message_info.bit.is_received_with_bitrate_switch = r1.bit.BRS;

	
	if(message.message_info.bit.is_extended)
	{
		message.id = r0.bit.ID;
	}
	else
	{
		message.id = GET_BITS(r0.bit.ID, 18, 29);
	}
	message.dlc = r1.bit.DLC;
	for (uint8_t i = 0; i < message.dlc; i++)
	{
		message.data[i] = data[i];
	}
	

	time_message.rx_message = message;

	if(circ_buf_flex_push(&mcan1_buffer.buffer_rx, &time_message) != CBF_SUCCESS)
	{
		//HANDLE ERROR TODO
	}
}

void MCAN0_INT0_Handler(void)
{
	volatile uint32_t status, i, rx_buffer_index;
	status = mcan_read_interrupt_status(&mcan0_instance);
	
	if (status & MCAN_RX_BUFFER_NEW_MESSAGE)
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_RX_BUFFER_NEW_MESSAGE);

		#ifdef MCAN0_INT0_DEBUG
		printf("CAN0 Message stored to Dedicated Rx Buffer Interrupt\r\n");
		#endif

		for (i = 0; i < CONF_MCAN0_RX_BUFFER_NUM; i++) 
		{
			if (mcan_rx_get_buffer_status(&mcan0_instance, i)) 
			{
				rx_buffer_index = i;
				mcan_rx_clear_buffer_status(&mcan0_instance, i);
				mcan_get_rx_buffer_element(&mcan0_instance, &mcan0_rx_element_buffer, rx_buffer_index);
				
				_mcan0_push_message(mcan0_rx_element_buffer.R0, mcan0_rx_element_buffer.R1, mcan0_rx_element_buffer.data, unix_timestamp_ms);

			}
		}
	}

	if (status & MCAN_RX_FIFO_0_NEW_MESSAGE)
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_RX_FIFO_0_NEW_MESSAGE);

		#ifdef MCAN0_INT0_DEBUG
		printf("CAN0 Rx FIFO 0 New Message Interrupt\r\n");
		#endif

		mcan_get_rx_fifo_0_element(&mcan0_instance, &mcan0_rx_element_fifo_0, mcan0_standard_receive_index);
		mcan_rx_fifo_acknowledge(&mcan0_instance, 0, mcan0_standard_receive_index);

		mcan0_standard_receive_index++;

		if (mcan0_standard_receive_index == CONF_MCAN0_RX_FIFO_0_NUM)
		{
			mcan0_standard_receive_index = 0;
		}

		_mcan0_push_message(mcan0_rx_element_fifo_0.R0, mcan0_rx_element_fifo_0.R1, mcan0_rx_element_fifo_0.data, unix_timestamp_ms);
	}

	if (status & MCAN_RX_FIFO_1_NEW_MESSAGE) 
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_RX_FIFO_1_NEW_MESSAGE);

		#ifdef MCAN0_INT0_DEBUG
		printf("CAN0 Rx FIFO 1 New Message Interrupt\r\n");
		#endif

		mcan_get_rx_fifo_1_element(&mcan0_instance, &mcan0_rx_element_fifo_1, mcan0_extended_receive_index);
		mcan_rx_fifo_acknowledge(&mcan0_instance, 1, mcan0_extended_receive_index);

		mcan0_extended_receive_index++;

		if (mcan0_extended_receive_index == CONF_MCAN0_RX_FIFO_1_NUM) 
		{
			mcan0_extended_receive_index = 0;
		}

		_mcan0_push_message(mcan0_rx_element_fifo_1.R0, mcan0_rx_element_fifo_1.R1, mcan0_rx_element_fifo_1.data, unix_timestamp_ms);
	}
	
	//Usually occurs if can tx pin is not connected to transceiver or to a transceiver not enabled or the ioport has not been set
	if (status & MCAN_BUS_OFF) 
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_BUS_OFF);

		#ifdef MCAN0_INT0_DEBUG
		printf("\n\r CAN0 Bus Off Status \r\n");
		#endif
		
		mcan_stop(&mcan0_instance);
		
	}

	//Usually occurs when sending message with different clock (either bad clock from sending device or SAM E70)
	//This means that no receiver has understood your message and thus they did not ack it
	if (status & MCAN_ACKNOWLEDGE_ERROR) 
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_ACKNOWLEDGE_ERROR);

		#ifdef MCAN0_INT0_DEBUG
		printf("\n\rCAN0 Acknowledge Error \r\n");
		#endif
	}

	//Usually occurs if can rx pin is not connected to transceiver or receiving message with different clock (either bad clock from sending device or SAM E70)
	if (status & MCAN_FORMAT_ERROR) 
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_FORMAT_ERROR);

		#ifdef MCAN0_INT0_DEBUG
		printf("\n\rCAN0 Format Error \r\n");
		#endif
	}
}

void MCAN0_INT1_Handler(void)
{
	volatile uint32_t status;
	status = mcan_read_interrupt_status(&mcan0_instance);
	
	if (status & MCAN_TIMESTAMP_COMPLETE)//This should be called MCAN_TRANSMISSION_COMPLETE, error from ASF. Might be fixed in the future!
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_TIMESTAMP_COMPLETE);
		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Transmission Completed Interrupt\r\n");
		#endif
		if(!mcan0_buffer.adding_in_tx_buffer)
		{
			if(circ_buf_flex_available_elements_to_read(&mcan0_buffer.buffer_tx)>0)
			{
				struct mcan_tx_element tx_elem;

				circ_buf_flex_pop(&mcan0_buffer.buffer_tx, &tx_elem);

				//we have to offset the buffer number in order to write in the fifo memory.
				mcan_set_tx_buffer_element(&mcan0_instance, &tx_elem, CONF_MCAN0_TX_BUFFER_NUM);
				int16_t stat = mcan_tx_transfer_request(&mcan0_instance, 1 << CONF_MCAN0_TX_BUFFER_NUM);
				mcan0_buffer.buffer_being_emptied_by_interruption = true;
			}
			else
			{
				mcan0_buffer.buffer_being_emptied_by_interruption = false;
			}
		}
		else mcan0_buffer.interruption_occurred_while_adding_in_tx_buffer = true;
	}

	if (status & MCAN_TX_CANCELLATION_FINISH)
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_TX_CANCELLATION_FINISH);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Transmission Cancellation Finished Interrupt \r\n");
		#endif
	}

	if (status & MCAN_TX_FIFO_EMPTY)
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_TX_FIFO_EMPTY);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Tx FIFO Empty Interrupt\r\n");
		#endif
	}

	if (status & MCAN_TX_EVENT_FIFO_NEW_ENTRY)
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_TX_EVENT_FIFO_NEW_ENTRY);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Tx Event FIFO New Entry Interrupt \r\n");
		#endif

		//TODO We could get the CAN message from fifo and get information on the message sent such as error and bit rate switch if fd
		//ack event fifo
		uint32_t fifo_status = mcan_tx_get_event_fifo_status(&mcan0_instance);
		//bool is_full = (fifo_status & (0x1u<<25))>>25;
		//uint32_t fifo_put_index = (fifo_status & MCAN_TXEFS_EFPI_Msk)>>MCAN_TXEFS_EFPI_Pos;
		uint32_t fifo_get_index = (fifo_status & MCAN_TXEFS_EFGI_Msk)>>MCAN_TXEFS_EFGI_Pos;
		//uint32_t fifo_fill_level = (fifo_status & MCAN_TXEFS_EFFL_Msk)>>MCAN_TXEFS_EFFL_Pos;
		
		mcan_tx_event_fifo_acknowledge(&mcan0_instance, fifo_get_index);
	}

	if (status & MCAN_TX_EVENT_FIFO_WATERMARK)
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_TX_EVENT_FIFO_WATERMARK);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Tx Event FIFO Watermark Reached Interrupt\r\n");
		#endif
	}

	if (status & MCAN_TX_EVENT_FIFO_FULL)
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_TX_EVENT_FIFO_FULL);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Tx Event FIFO Full Interrupt\r\n");
		#endif
	}

	if (status & MCAN_TX_EVENT_FIFO_ELEMENT_LOST)
	{
		mcan_clear_interrupt_status(&mcan0_instance, MCAN_TX_EVENT_FIFO_ELEMENT_LOST);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Tx Event FIFO Element Lost Interrupt\r\n");
		#endif
	}
}

void MCAN1_INT0_Handler(void)
{
	volatile uint32_t status, i, rx_buffer_index;
	status = mcan_read_interrupt_status(&mcan1_instance);

	if (status & MCAN_RX_BUFFER_NEW_MESSAGE) 
	{	
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_RX_BUFFER_NEW_MESSAGE);
		
		#ifdef MCAN1_INT0_DEBUG
		printf("CAN1 Message stored to Dedicated Rx Buffer Interrupt\r\n");
		#endif

		for (i = 0; i < CONF_MCAN1_RX_BUFFER_NUM; i++) {
			if (mcan_rx_get_buffer_status(&mcan0_instance, i)) 
			{
				rx_buffer_index = i;
				mcan_rx_clear_buffer_status(&mcan1_instance, i);
				mcan_get_rx_buffer_element(&mcan1_instance, &mcan1_rx_element_buffer, rx_buffer_index);
				
				_mcan1_push_message(mcan1_rx_element_buffer.R0, mcan1_rx_element_buffer.R1, mcan1_rx_element_buffer.data, unix_timestamp_ms);

			}
		}
	}

	if (status & MCAN_RX_FIFO_0_NEW_MESSAGE) 
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_RX_FIFO_0_NEW_MESSAGE);

		#ifdef MCAN1_INT0_DEBUG
		printf("CAN1 Rx FIFO 0 New Message Interrupt\r\n");
		#endif

		mcan_get_rx_fifo_0_element(&mcan1_instance, &mcan1_rx_element_fifo_0, mcan1_standard_receive_index);
		mcan_rx_fifo_acknowledge(&mcan1_instance, 0, mcan1_standard_receive_index);
		
		mcan1_standard_receive_index++;
		
		if (mcan1_standard_receive_index == CONF_MCAN1_RX_FIFO_0_NUM)
		{
			mcan1_standard_receive_index = 0;
		}

		_mcan1_push_message(mcan1_rx_element_fifo_0.R0, mcan1_rx_element_fifo_0.R1, mcan1_rx_element_fifo_0.data, unix_timestamp_ms);
	}

	if (status & MCAN_RX_FIFO_1_NEW_MESSAGE) 
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_RX_FIFO_1_NEW_MESSAGE);

		#ifdef MCAN1_INT0_DEBUG
		printf("CAN1 Rx FIFO 1 New Message Interrupt\r\n");
		#endif

		mcan_get_rx_fifo_1_element(&mcan1_instance, &mcan1_rx_element_fifo_1, mcan1_extended_receive_index);
		mcan_rx_fifo_acknowledge(&mcan1_instance, 1, mcan1_extended_receive_index);
		
		mcan1_extended_receive_index++;

		if (mcan1_extended_receive_index == CONF_MCAN1_RX_FIFO_1_NUM) 
		{
			mcan1_extended_receive_index = 0;
		}

		_mcan1_push_message(mcan1_rx_element_fifo_1.R0, mcan1_rx_element_fifo_1.R1, mcan1_rx_element_fifo_1.data, unix_timestamp_ms);
	}
	
	//Usually occurs if can tx pin is not connected to transceiver or to a transceiver not enabled or the ioport has not been set
	if (status & MCAN_BUS_OFF) 
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_BUS_OFF);

		#ifdef MCAN1_INT0_DEBUG
		printf("\n\rCAN1 Bus Off Status \r\n");
		#endif
		
		mcan_stop(&mcan1_instance);
		
	}

	//Usually occurs when sending message with different clock (either bad clock from sending device or SAM E70)
	//This means that no receiver has understood your message and thus they did not ack it
	if (status & MCAN_ACKNOWLEDGE_ERROR) 
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_ACKNOWLEDGE_ERROR);
		
		#ifdef MCAN1_INT0_DEBUG
		printf("\n\rCAN1 Acknowledge Error \r\n");
		#endif
	}
	
	//Usually occurs if can rx pin is not connected to transceiver or receiving message with different clock (either bad clock from sending device or SAM E70)
	if (status & MCAN_FORMAT_ERROR) 
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_FORMAT_ERROR);

		#ifdef MCAN1_INT0_DEBUG
		printf("\n\rCAN1 Format Error \r\n");
		#endif	
	}
}

void MCAN1_INT1_Handler(void)
{
	volatile uint32_t status;
	status = mcan_read_interrupt_status(&mcan1_instance);
	
	if (status & MCAN_TIMESTAMP_COMPLETE)//This should be called MCAN_TRANSMISSION_COMPLETE, error from ASF. Might be fixed in the future!
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_TIMESTAMP_COMPLETE);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Transmission Completed Interrupt\r\n");
		#endif

		if(!mcan1_buffer.adding_in_tx_buffer)
		{
			if(circ_buf_flex_available_elements_to_read(&mcan1_buffer.buffer_tx)>0)
			{
				struct mcan_tx_element tx_elem;
				circ_buf_flex_pop(&mcan1_buffer.buffer_tx, &tx_elem);

				//we have to offset the buffer number in order to write in the fifo memory.
				mcan_set_tx_buffer_element(&mcan1_instance, &tx_elem, CONF_MCAN1_TX_BUFFER_NUM);
				mcan_tx_transfer_request(&mcan1_instance, 1 << CONF_MCAN1_TX_BUFFER_NUM);
				mcan1_buffer.buffer_being_emptied_by_interruption = true;
			}
			else
			{
				mcan1_buffer.buffer_being_emptied_by_interruption = false;
			}
		}
		else mcan1_buffer.interruption_occurred_while_adding_in_tx_buffer = true;
	}

	if (status & MCAN_TX_CANCELLATION_FINISH)
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_TX_CANCELLATION_FINISH);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Transmission Cancellation Finished Interrupt \r\n");
		#endif
	}
	if (status & MCAN_TX_FIFO_EMPTY)
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_TX_FIFO_EMPTY);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Tx FIFO Empty Interrupt\r\n");
		#endif
	}
	if (status & MCAN_TX_EVENT_FIFO_NEW_ENTRY)
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_TX_EVENT_FIFO_NEW_ENTRY);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Tx Event FIFO New Entry Interrupt \r\n");
		#endif

		//TODO We could get the CAN message from fifo and get information on the message sent such as error and bit rate switch if fd
		//ack event fifo
		uint32_t fifo_status = mcan_tx_get_event_fifo_status(&mcan1_instance);
		//bool is_full = (fifo_status & (0x1u<<25))>>25;
		//uint32_t fifo_put_index = (fifo_status & MCAN_TXEFS_EFPI_Msk)>>MCAN_TXEFS_EFPI_Pos;
		uint32_t fifo_get_index = (fifo_status & MCAN_TXEFS_EFGI_Msk)>>MCAN_TXEFS_EFGI_Pos;
		//uint32_t fifo_fill_level = (fifo_status & MCAN_TXEFS_EFFL_Msk)>>MCAN_TXEFS_EFFL_Pos;
		
		mcan_tx_event_fifo_acknowledge(&mcan1_instance, fifo_get_index);
	}
	if (status & MCAN_TX_EVENT_FIFO_WATERMARK)
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_TX_EVENT_FIFO_WATERMARK);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Tx Event FIFO Watermark Reached Interrupt\r\n");
		#endif
	}
	if (status & MCAN_TX_EVENT_FIFO_FULL)
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_TX_EVENT_FIFO_FULL);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Tx Event FIFO Full Interrupt\r\n");
		#endif
	}
	if (status & MCAN_TX_EVENT_FIFO_ELEMENT_LOST)
	{
		mcan_clear_interrupt_status(&mcan1_instance, MCAN_TX_EVENT_FIFO_ELEMENT_LOST);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Tx Event FIFO Element Lost Interrupt\r\n");
		#endif
	}
}

inline uint32_t mcan_get_interrupt(struct mcan_module *const module_inst, const enum mcan_interrupt_source source)
{
	return EXTRACT_X(module_inst->hw->MCAN_IE, source);
}

void _mcan_send_message(struct mcan_module *const module_inst, uint32_t id_value, uint8_t *data, uint32_t data_length, bool is_extended, bool is_remote_transmission)
{
	uint32_t i;
	struct mcan_tx_element tx_element;

	mcan_get_tx_buffer_element_defaults(&tx_element);
	if (is_extended)
	{
		tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_EXTENDED_ID(id_value) |
			MCAN_TX_ELEMENT_T0_XTD;
	}
	else
	{
		tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_STANDARD_ID(id_value);
	}

	if(is_remote_transmission)
	{
		tx_element.T0.bit.RTR = 1;
	}

	tx_element.T1.bit.DLC = data_length;
	for (i = 0; i < data_length; i++) {
		tx_element.data[i] = *data; 
		data++;
	}
	
	#define offsetTX CONF_MCAN1_TX_BUFFER_NUM
	mcan_set_tx_buffer_element(module_inst, &tx_element, offsetTX);
	mcan_tx_transfer_request(module_inst, 1 << offsetTX);
}

uint8_t mcan0_send_message(uint32_t id_value, uint8_t *data, uint32_t data_length, bool is_extended, bool is_remote_transmission)
{
	uint8_t result;

	uint32_t i;
	struct mcan_tx_element tx_element;

	mcan_get_tx_buffer_element_defaults(&tx_element);
	if (is_extended)
	{
		tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_EXTENDED_ID(id_value) |
			MCAN_TX_ELEMENT_T0_XTD;
	}
	else
	{
		tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_STANDARD_ID(id_value);
	}

	if(is_remote_transmission)
	{
		tx_element.T0.bit.RTR = 1;
	}

	tx_element.T1.bit.DLC = data_length;
	for (i = 0; i < data_length; i++) {
		tx_element.data[i] = *data; 
		data++;
	}

	//Set flag to prevent problem with interruption
	mcan0_buffer.adding_in_tx_buffer = true;
	result = circ_buf_flex_push(&mcan0_buffer.buffer_tx, &tx_element);
	mcan0_buffer.adding_in_tx_buffer = false;

	if(mcan0_buffer.interruption_occurred_while_adding_in_tx_buffer | !mcan0_buffer.buffer_being_emptied_by_interruption)
	{
		//No need of this if we've just added a message to the buffer and prevent interruption to deal with messages !
		//We know for sure that there is at least the message in our buffer.
		if(circ_buf_flex_available_elements_to_read(&mcan0_buffer.buffer_tx) > 0)
		{
			struct mcan_tx_element tx_elem;
			circ_buf_flex_pop(&mcan0_buffer.buffer_tx, &tx_elem);
			//we have to offset the buffer number in order to write in the fifo memory.
			mcan_set_tx_buffer_element(&mcan0_instance, &tx_elem, CONF_MCAN0_TX_BUFFER_NUM);
			int16_t stat = mcan_tx_transfer_request(&mcan0_instance, 1 << CONF_MCAN0_TX_BUFFER_NUM);
			
			//reset flags
			mcan0_buffer.interruption_occurred_while_adding_in_tx_buffer = false;
			mcan0_buffer.buffer_being_emptied_by_interruption = true;
		}
	}

	return result;
	
}
uint8_t mcan1_send_message(uint32_t id_value, uint8_t *data, uint32_t data_length, bool is_extended, bool is_remote_transmission)
{
	uint8_t result;

	uint32_t i;
	struct mcan_tx_element tx_element;

	mcan_get_tx_buffer_element_defaults(&tx_element);
	if (is_extended)
	{
		tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_EXTENDED_ID(id_value) |
			MCAN_TX_ELEMENT_T0_XTD;
	}
	else
	{
		tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_STANDARD_ID(id_value);
	}

	if(is_remote_transmission)
	{
		tx_element.T0.bit.RTR = 1;
	}

	tx_element.T1.bit.DLC = data_length;
	for (i = 0; i < data_length; i++) {
		tx_element.data[i] = *data; 
		data++;
	}

	//Set flag to prevent problem with interruption
	mcan1_buffer.adding_in_tx_buffer = true;
	result = circ_buf_flex_push(&mcan1_buffer.buffer_tx, &tx_element);
	mcan1_buffer.adding_in_tx_buffer = false;

	if(mcan1_buffer.interruption_occurred_while_adding_in_tx_buffer | !mcan1_buffer.buffer_being_emptied_by_interruption)
	{
		//No need of this if we've just added a message to the buffer and prevent interruption to deal with messages !
		//We know for sure that there is at least the message in our buffer.
		if(circ_buf_flex_available_elements_to_read(&mcan1_buffer.buffer_tx) > 0)
		{
			struct mcan_tx_element tx_elem;
			circ_buf_flex_pop(&mcan1_buffer.buffer_tx, &tx_elem);
			//we have to offset the buffer number in order to write in the fifo memory.
			mcan_set_tx_buffer_element(&mcan1_instance, &tx_elem, CONF_MCAN1_TX_BUFFER_NUM);
			mcan_tx_transfer_request(&mcan1_instance, 1 << CONF_MCAN1_TX_BUFFER_NUM);

			//reset flags
			mcan1_buffer.interruption_occurred_while_adding_in_tx_buffer = false;
			mcan1_buffer.buffer_being_emptied_by_interruption = true;
		}
	}

	return result;
	
}

uint32_t mcan0_available_message(void)
{
	return circ_buf_flex_available_elements_to_read(&mcan0_buffer.buffer_rx);
}
uint32_t mcan1_available_message(void)
{
	return circ_buf_flex_available_elements_to_read(&mcan1_buffer.buffer_rx);
}

uint8_t mcan0_get_message(mcan_timestamped_rx_message_t* ts_rx_message)
{
	return circ_buf_flex_pop(&mcan0_buffer.buffer_rx, ts_rx_message);
}
uint8_t mcan1_get_message(mcan_timestamped_rx_message_t* ts_rx_message)
{
	return circ_buf_flex_pop(&mcan1_buffer.buffer_rx, ts_rx_message);
}

void mcan0_start()
{
	mcan_start(&mcan0_instance);
}

void mcan1_start()
{
	mcan_start(&mcan1_instance);
}

void mcan0_stop()
{
	mcan_stop(&mcan0_instance);
}

void mcan1_stop()
{
	mcan_stop(&mcan1_instance);
}