#include "same70_can_driver.h"
/*
#define MCAN0_INT0_DEBUG
#define MCAN0_INT1_DEBUG
#define MCAN1_INT0_DEBUG
#define MCAN1_INT1_DEBUG
*/
void _can_configure_rx_fifo_to_accept_all(struct mcan_module* module_inst)
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

void can_configure(enum can_line line, uint32_t baudrate, uint32_t rx_buffer_size, uint32_t tx_buffer_size)
{	
	can_t* can = can_get_can_from_line(line);
	
	Mcan *hw;
	
	switch (line)
	{
		case CAN_LINE_0:
			hw = MCAN0;
			can->buffer_tx_number = CONF_MCAN0_RX_BUFFER_NUM;
			can->standard_filter_number = CONF_MCAN0_RX_STANDARD_ID_FILTER_NUM;
			can->extended_filter_number = CONF_MCAN0_RX_EXTENDED_ID_FILTER_NUM;
			break;
		case CAN_LINE_1:
			hw = MCAN1;
			can->buffer_tx_number = CONF_MCAN1_RX_BUFFER_NUM;
			can->standard_filter_number = CONF_MCAN1_RX_STANDARD_ID_FILTER_NUM;
			can->extended_filter_number = CONF_MCAN1_RX_EXTENDED_ID_FILTER_NUM;
			break;
	}
	 
	can->standard_receive_index = 0;
	can->extended_receive_index = 0;

	switch (line)
	{
		case CAN_LINE_0:
			can->buffer_tx_number = CONF_MCAN0_RX_BUFFER_NUM;
			can->standard_filter_number = CONF_MCAN0_RX_STANDARD_ID_FILTER_NUM;
			can->extended_filter_number = CONF_MCAN0_RX_EXTENDED_ID_FILTER_NUM;
			break;
		case CAN_LINE_1:
			can->buffer_tx_number = CONF_MCAN1_RX_BUFFER_NUM;
			can->standard_filter_number = CONF_MCAN1_RX_STANDARD_ID_FILTER_NUM;
			can->extended_filter_number = CONF_MCAN1_RX_EXTENDED_ID_FILTER_NUM;
			break;
	}

	can->standard_filter_index = 0;
	can->extended_filter_index = 0;

	circ_buf_flex_init_buffer(&can->buffer.buffer_rx, rx_buffer_size, sizeof(can_timestamped_rx_message_t));
	circ_buf_flex_init_buffer(&can->buffer.buffer_tx, tx_buffer_size, sizeof(can_tx_message_t));
	
	can->buffer.adding_in_tx_buffer = false;
	can->buffer.interruption_occurred_while_adding_in_tx_buffer = false;
	can->buffer.buffer_being_emptied_by_interruption = false;

	struct mcan_config config_mcan;
	mcan_get_config_defaults(&config_mcan);
	config_mcan.nonmatching_frames_action_standard = MCAN_NONMATCHING_FRAMES_REJECT;
	config_mcan.nonmatching_frames_action_extended = MCAN_NONMATCHING_FRAMES_REJECT;
	config_mcan.remote_frames_standard_reject = false;
	config_mcan.remote_frames_extended_reject = false;

	mcan_init(&can->instance, hw, &config_mcan);

	//Enabling the upll clock 
	//NEED TO HAVE THE RIGHT conf_mcan.h !
	pmc_enable_upll_clock();
	// This was firstly changed to "pmc_switch_pck_to_pllack(PMC_PCK_5, PMC_PCK_PRES(9));" in the mcan_init function above.
	//It is recommended in the datasheet to use upllck as it is less subject to change. It is running at 480 MHz.
	pmc_disable_pck(PMC_PCK_5);
	//dividing upll by 6 to get a 80 Mhz signal 
	//Which is again by default divided by 8 in the CONF_MCAN_NBTP_NBRP_VALUE in the conf_mcan.h to get a 10 MHz for 500kbits/s default speed.
	pmc_switch_pck_to_upllck(PMC_PCK_5, PMC_PCK_PRES(5));
	pmc_enable_pck(PMC_PCK_5);

	//has to be after set of PMC_PCK_5 as it uses it to calculate the MCAN_NBTP_NBRP_VALUE for the given speed.
	//MCAN_NBTP_NBRP_VALUE = (PMC_PCK_5 frequency)/(baudrate asked)/20
	//Here (PMC_PCK_5 frequency) = 80Mhz
	//Changing default speed.
	mcan_set_baudrate(can->instance.hw, baudrate);


	//choosing what interruption to activate
	mcan_enable_interrupt(&can->instance,
	 	MCAN_RX_FIFO_1_NEW_MESSAGE |
		MCAN_RX_BUFFER_NEW_MESSAGE |
		MCAN_RX_FIFO_0_NEW_MESSAGE |
		MCAN_FORMAT_ERROR |
		MCAN_ACKNOWLEDGE_ERROR |
		MCAN_BUS_OFF |
		MCAN_TIMESTAMP_COMPLETE |
		MCAN_TX_CANCELLATION_FINISH |
		MCAN_TX_FIFO_EMPTY |
		MCAN_TX_EVENT_FIFO_NEW_ENTRY |
		MCAN_TX_EVENT_FIFO_WATERMARK |
		MCAN_TX_EVENT_FIFO_FULL |
		MCAN_TX_EVENT_FIFO_ELEMENT_LOST);
	
	//Interrupt Line Selection making all tx to the second line.
	//using int0 for reception and basic errors
	//using int1 for emission.
	can->instance.hw->MCAN_ILS = 0x0000FF00ul;
	
	//activating interruptions for int0 and int1
	switch (line)
	{
		case CAN_LINE_0:
			irq_register_handler(MCAN0_INT0_IRQn, 1);
			irq_register_handler(MCAN0_INT1_IRQn, 2);
			break;
		case CAN_LINE_1:
			irq_register_handler(MCAN1_INT0_IRQn, 1);
			irq_register_handler(MCAN1_INT1_IRQn, 2);
			break;
		//case ... if more but for SAM E70 only 2 can lines
	}
	
	_can_configure_rx_fifo_to_accept_all(&can->instance);

    mcan_start(&can->instance);
}

void _can_push_message(enum can_line line, MCAN_RX_ELEMENT_R0_Type r0, MCAN_RX_ELEMENT_R1_Type r1, uint8_t* data, uint64_t rec_timestamp)
{
	can_t* can = can_get_can_from_line(line);
	can_timestamped_rx_message_t time_message;
	time_message.timestamp = rec_timestamp;

	can_rx_message_t message;
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

	if(circ_buf_flex_push(&can->buffer.buffer_rx, &time_message) != CBF_SUCCESS)
	{
		//TODO HANDLE ERROR 
	}
}

void MCAN0_INT0_Handler(void)
{
	volatile uint32_t status, i, rx_buffer_index;
	status = mcan_read_interrupt_status(&can0.instance);
	
	if (status & MCAN_RX_BUFFER_NEW_MESSAGE)
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_RX_BUFFER_NEW_MESSAGE);

		#ifdef MCAN0_INT0_DEBUG
		printf("CAN0 Message stored to Dedicated Rx Buffer Interrupt\r\n");
		#endif

		for (i = 0; i < CONF_MCAN0_RX_BUFFER_NUM; i++) 
		{
			if (mcan_rx_get_buffer_status(&can0.instance, i)) 
			{
				rx_buffer_index = i;
				mcan_rx_clear_buffer_status(&can0.instance, i);
				mcan_get_rx_buffer_element(&can0.instance, &can0.rx_element_buffer, rx_buffer_index);
				
				_can_push_message(CAN_LINE_0, can0.rx_element_buffer.R0, can0.rx_element_buffer.R1, can0.rx_element_buffer.data, unix_timestamp_ms);

			}
		}
	}

	if (status & MCAN_RX_FIFO_0_NEW_MESSAGE)
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_RX_FIFO_0_NEW_MESSAGE);

		#ifdef MCAN0_INT0_DEBUG
		printf("CAN0 Rx FIFO 0 New Message Interrupt\r\n");
		#endif

		mcan_get_rx_fifo_0_element(&can0.instance, &can0.rx_element_fifo_0, can0.standard_receive_index);
		mcan_rx_fifo_acknowledge(&can0.instance, 0, can0.standard_receive_index);

		can0.standard_receive_index++;

		if (can0.standard_receive_index == CONF_MCAN0_RX_FIFO_0_NUM)
		{
			can0.standard_receive_index = 0;
		}

		_can_push_message(CAN_LINE_0, can0.rx_element_fifo_0.R0, can0.rx_element_fifo_0.R1, can0.rx_element_fifo_0.data, unix_timestamp_ms);
	}

	if (status & MCAN_RX_FIFO_1_NEW_MESSAGE) 
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_RX_FIFO_1_NEW_MESSAGE);

		#ifdef MCAN0_INT0_DEBUG
		printf("CAN0 Rx FIFO 1 New Message Interrupt\r\n");
		#endif

		mcan_get_rx_fifo_1_element(&can0.instance, &can0.rx_element_fifo_1, can0.extended_receive_index);
		mcan_rx_fifo_acknowledge(&can0.instance, 1, can0.extended_receive_index);

		can0.extended_receive_index++;

		if (can0.extended_receive_index == CONF_MCAN0_RX_FIFO_1_NUM) 
		{
			can0.extended_receive_index = 0;
		}

		_can_push_message(CAN_LINE_0, can0.rx_element_fifo_1.R0, can0.rx_element_fifo_1.R1,can0.rx_element_fifo_1.data, unix_timestamp_ms);
	}
	
	//Usually occurs if can tx pin is not connected to transceiver or to a transceiver not enabled or the ioport has not been set
	if (status & MCAN_BUS_OFF) 
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_BUS_OFF);

		#ifdef MCAN0_INT0_DEBUG
		printf("\n\r CAN0 Bus Off Status \r\n");
		#endif
		
		mcan_stop(&can0.instance);
		
	}

	//Usually occurs when sending message with different clock (either bad clock from sending device or SAM E70)
	//This means that no receiver has understood your message and thus they did not ack it
	if (status & MCAN_ACKNOWLEDGE_ERROR) 
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_ACKNOWLEDGE_ERROR);

		#ifdef MCAN0_INT0_DEBUG
		printf("\n\rCAN0 Acknowledge Error \r\n");
		#endif
	}

	//Usually occurs if can rx pin is not connected to transceiver or receiving message with different clock (either bad clock from sending device or SAM E70)
	if (status & MCAN_FORMAT_ERROR) 
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_FORMAT_ERROR);

		#ifdef MCAN0_INT0_DEBUG
		printf("\n\rCAN0 Format Error \r\n");
		#endif
	}
}

void MCAN0_INT1_Handler(void)
{
	volatile uint32_t status;
	status = mcan_read_interrupt_status(&can0.instance);
	
	if (status & MCAN_TIMESTAMP_COMPLETE)//This should be called MCAN_TRANSMISSION_COMPLETE, error from ASF. Might be fixed in the future!
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_TIMESTAMP_COMPLETE);
		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Transmission Completed Interrupt\r\n");
		#endif
		if(!can0.buffer.adding_in_tx_buffer)
		{
			if(circ_buf_flex_available_elements_to_read(&can0.buffer.buffer_tx)>0)
			{
				struct mcan_tx_element tx_elem;

				circ_buf_flex_pop(&can0.buffer.buffer_tx, &tx_elem);

				//we have to offset the buffer number in order to write in the fifo memory.
				mcan_set_tx_buffer_element(&can0.instance, &tx_elem, CONF_MCAN0_TX_BUFFER_NUM);
				mcan_tx_transfer_request(&can0.instance, 1 << CONF_MCAN0_TX_BUFFER_NUM);
				can0.buffer.buffer_being_emptied_by_interruption = true;
			}
			else
			{
				can0.buffer.buffer_being_emptied_by_interruption = false;
			}
		}
		else can0.buffer.interruption_occurred_while_adding_in_tx_buffer = true;
	}

	if (status & MCAN_TX_CANCELLATION_FINISH)
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_TX_CANCELLATION_FINISH);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Transmission Cancellation Finished Interrupt \r\n");
		#endif
	}

	if (status & MCAN_TX_FIFO_EMPTY)
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_TX_FIFO_EMPTY);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Tx FIFO Empty Interrupt\r\n");
		#endif
	}

	if (status & MCAN_TX_EVENT_FIFO_NEW_ENTRY)
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_TX_EVENT_FIFO_NEW_ENTRY);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Tx Event FIFO New Entry Interrupt \r\n");
		#endif

		//TODO We could get the CAN message from fifo and get information on the message sent such as error and bit rate switch if fd
		//ack event fifo
		uint32_t fifo_status = mcan_tx_get_event_fifo_status(&can0.instance);
		//bool is_full = (fifo_status & (0x1u<<25))>>25;
		//uint32_t fifo_put_index = (fifo_status & MCAN_TXEFS_EFPI_Msk)>>MCAN_TXEFS_EFPI_Pos;
		uint32_t fifo_get_index = (fifo_status & MCAN_TXEFS_EFGI_Msk)>>MCAN_TXEFS_EFGI_Pos;
		//uint32_t fifo_fill_level = (fifo_status & MCAN_TXEFS_EFFL_Msk)>>MCAN_TXEFS_EFFL_Pos;
		
		mcan_tx_event_fifo_acknowledge(&can0.instance, fifo_get_index);
	}

	if (status & MCAN_TX_EVENT_FIFO_WATERMARK)
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_TX_EVENT_FIFO_WATERMARK);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Tx Event FIFO Watermark Reached Interrupt\r\n");
		#endif
	}

	if (status & MCAN_TX_EVENT_FIFO_FULL)
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_TX_EVENT_FIFO_FULL);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Tx Event FIFO Full Interrupt\r\n");
		#endif
	}

	if (status & MCAN_TX_EVENT_FIFO_ELEMENT_LOST)
	{
		mcan_clear_interrupt_status(&can0.instance, MCAN_TX_EVENT_FIFO_ELEMENT_LOST);

		#ifdef MCAN0_INT1_DEBUG
		printf("CAN0 Tx Event FIFO Element Lost Interrupt\r\n");
		#endif
	}
}

void MCAN1_INT0_Handler(void)
{
	volatile uint32_t status, i, rx_buffer_index;
	status = mcan_read_interrupt_status(&can1.instance);

	if (status & MCAN_RX_BUFFER_NEW_MESSAGE) 
	{	
		mcan_clear_interrupt_status(&can1.instance, MCAN_RX_BUFFER_NEW_MESSAGE);
		
		#ifdef MCAN1_INT0_DEBUG
		printf("CAN1 Message stored to Dedicated Rx Buffer Interrupt\r\n");
		#endif

		for (i = 0; i < CONF_MCAN1_RX_BUFFER_NUM; i++) 
		{
			if (mcan_rx_get_buffer_status(&can1.instance, i)) 
			{
				rx_buffer_index = i;
				mcan_rx_clear_buffer_status(&can1.instance, i);
				mcan_get_rx_buffer_element(&can1.instance, &can1.rx_element_buffer, rx_buffer_index);
				
				_can_push_message(CAN_LINE_1, can1.rx_element_buffer.R0, can1.rx_element_buffer.R1, can1.rx_element_buffer.data, unix_timestamp_ms);

			}
		}
	}

	if (status & MCAN_RX_FIFO_0_NEW_MESSAGE) 
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_RX_FIFO_0_NEW_MESSAGE);

		#ifdef MCAN1_INT0_DEBUG
		printf("CAN1 Rx FIFO 0 New Message Interrupt\r\n");
		#endif

		mcan_get_rx_fifo_0_element(&can1.instance, &can1.rx_element_fifo_0, can1.standard_receive_index);
		mcan_rx_fifo_acknowledge(&can1.instance, 0, can1.standard_receive_index);
		
		can1.standard_receive_index++;
		
		if (can1.standard_receive_index == CONF_MCAN1_RX_FIFO_0_NUM)
		{
			can1.standard_receive_index = 0;
		}

		_can_push_message(CAN_LINE_1, can1.rx_element_fifo_0.R0, can1.rx_element_fifo_0.R1, can1.rx_element_fifo_0.data, unix_timestamp_ms);
	}

	if (status & MCAN_RX_FIFO_1_NEW_MESSAGE) 
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_RX_FIFO_1_NEW_MESSAGE);

		#ifdef MCAN1_INT0_DEBUG
		printf("CAN1 Rx FIFO 1 New Message Interrupt\r\n");
		#endif

		mcan_get_rx_fifo_1_element(&can1.instance, &can1.rx_element_fifo_1, can1.extended_receive_index);
		mcan_rx_fifo_acknowledge(&can1.instance, 1, can1.extended_receive_index);
		
		can1.extended_receive_index++;

		if (can1.extended_receive_index == CONF_MCAN1_RX_FIFO_1_NUM) 
		{
			can1.extended_receive_index = 0;
		}

		_can_push_message(CAN_LINE_1, can1.rx_element_fifo_1.R0, can1.rx_element_fifo_1.R1, can1.rx_element_fifo_1.data, unix_timestamp_ms);
	}
	
	//Usually occurs if can tx pin is not connected to transceiver or to a transceiver not enabled or the ioport has not been set
	if (status & MCAN_BUS_OFF) 
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_BUS_OFF);

		#ifdef MCAN1_INT0_DEBUG
		printf("\n\rCAN1 Bus Off Status \r\n");
		#endif
		
		mcan_stop(&can1.instance);
		
	}

	//Usually occurs when sending message with different clock (either bad clock from sending device or SAM E70)
	//This means that no receiver has understood your message and thus they did not ack it
	if (status & MCAN_ACKNOWLEDGE_ERROR) 
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_ACKNOWLEDGE_ERROR);
		
		#ifdef MCAN1_INT0_DEBUG
		printf("\n\rCAN1 Acknowledge Error \r\n");
		#endif
	}
	
	//Usually occurs if can rx pin is not connected to transceiver or receiving message with different clock (either bad clock from sending device or SAM E70)
	if (status & MCAN_FORMAT_ERROR) 
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_FORMAT_ERROR);

		#ifdef MCAN1_INT0_DEBUG
		printf("\n\rCAN1 Format Error \r\n");
		#endif	
	}
}

void MCAN1_INT1_Handler(void)
{
	volatile uint32_t status;
	status = mcan_read_interrupt_status(&can1.instance);
	
	if (status & MCAN_TIMESTAMP_COMPLETE)//This should be called MCAN_TRANSMISSION_COMPLETE, error from ASF. Might be fixed in the future!
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_TIMESTAMP_COMPLETE);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Transmission Completed Interrupt\r\n");
		#endif

		if(!can1.buffer.adding_in_tx_buffer)
		{
			if(circ_buf_flex_available_elements_to_read(&can1.buffer.buffer_tx)>0)
			{
				struct mcan_tx_element tx_elem;
				circ_buf_flex_pop(&can1.buffer.buffer_tx, &tx_elem);

				//we have to offset the buffer number in order to write in the fifo memory.
				mcan_set_tx_buffer_element(&can1.instance, &tx_elem, CONF_MCAN1_TX_BUFFER_NUM);
				mcan_tx_transfer_request(&can1.instance, 1 << CONF_MCAN1_TX_BUFFER_NUM);
				can1.buffer.buffer_being_emptied_by_interruption = true;
			}
			else
			{
				can1.buffer.buffer_being_emptied_by_interruption = false;
			}
		}
		else can1.buffer.interruption_occurred_while_adding_in_tx_buffer = true;
	}

	if (status & MCAN_TX_CANCELLATION_FINISH)
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_TX_CANCELLATION_FINISH);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Transmission Cancellation Finished Interrupt \r\n");
		#endif
	}
	if (status & MCAN_TX_FIFO_EMPTY)
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_TX_FIFO_EMPTY);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Tx FIFO Empty Interrupt\r\n");
		#endif
	}
	if (status & MCAN_TX_EVENT_FIFO_NEW_ENTRY)
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_TX_EVENT_FIFO_NEW_ENTRY);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Tx Event FIFO New Entry Interrupt \r\n");
		#endif

		//TODO We could get the CAN message from fifo and get information on the message sent such as error and bit rate switch if fd
		//ack event fifo
		uint32_t fifo_status = mcan_tx_get_event_fifo_status(&can1.instance);
		//bool is_full = (fifo_status & (0x1u<<25))>>25;
		//uint32_t fifo_put_index = (fifo_status & MCAN_TXEFS_EFPI_Msk)>>MCAN_TXEFS_EFPI_Pos;
		uint32_t fifo_get_index = (fifo_status & MCAN_TXEFS_EFGI_Msk)>>MCAN_TXEFS_EFGI_Pos;
		//uint32_t fifo_fill_level = (fifo_status & MCAN_TXEFS_EFFL_Msk)>>MCAN_TXEFS_EFFL_Pos;
		
		mcan_tx_event_fifo_acknowledge(&can1.instance, fifo_get_index);
	}
	if (status & MCAN_TX_EVENT_FIFO_WATERMARK)
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_TX_EVENT_FIFO_WATERMARK);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Tx Event FIFO Watermark Reached Interrupt\r\n");
		#endif
	}
	if (status & MCAN_TX_EVENT_FIFO_FULL)
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_TX_EVENT_FIFO_FULL);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Tx Event FIFO Full Interrupt\r\n");
		#endif
	}
	if (status & MCAN_TX_EVENT_FIFO_ELEMENT_LOST)
	{
		mcan_clear_interrupt_status(&can1.instance, MCAN_TX_EVENT_FIFO_ELEMENT_LOST);

		#ifdef MCAN1_INT1_DEBUG
		printf("CAN1 Tx Event FIFO Element Lost Interrupt\r\n");
		#endif
	}
}

void _can_send_message(enum can_line line, uint32_t id_value, uint8_t *data, uint32_t data_length, bool is_extended, bool is_remote_transmission)
{
	can_t* can = can_get_can_from_line(line);

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
	
	mcan_set_tx_buffer_element(&can->instance, &tx_element, can->buffer_tx_number);
	mcan_tx_transfer_request(&can->instance, 1 << can->buffer_tx_number);
}

uint8_t can_send_message(enum can_line line, uint32_t id_value, uint8_t *data, uint32_t data_length, bool is_extended, bool is_remote_transmission)
{
	can_t* can = can_get_can_from_line(line);

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
	can->buffer.adding_in_tx_buffer = true;
	result = circ_buf_flex_push(&can->buffer.buffer_tx, &tx_element);
	can->buffer.adding_in_tx_buffer = false;

	if(can->buffer.interruption_occurred_while_adding_in_tx_buffer | !can->buffer.buffer_being_emptied_by_interruption)
	{
		//No need of this if we've just added a message to the buffer and prevent interruption to deal with messages !
		//We know for sure that there is at least the message in our buffer.
		if(circ_buf_flex_available_elements_to_read(&can->buffer.buffer_tx) > 0)
		{
			struct mcan_tx_element tx_elem;
			circ_buf_flex_pop(&can->buffer.buffer_tx, &tx_elem);
			
			//we have to offset the buffer number in order to write in the fifo memory.
			mcan_set_tx_buffer_element(&can->instance, &tx_elem, can->buffer_tx_number);
			mcan_tx_transfer_request(&can->instance, 1 << can->buffer_tx_number);
			
			//reset flags
			can->buffer.interruption_occurred_while_adding_in_tx_buffer = false;
			can->buffer.buffer_being_emptied_by_interruption = true;
		}
	}

	return result;
}

uint32_t can_available_message(enum can_line line)
{
	can_t* can = can_get_can_from_line(line);
	return circ_buf_flex_available_elements_to_read(&can->buffer.buffer_rx);
}

uint8_t can_get_message(enum can_line line, can_timestamped_rx_message_t* ts_rx_message)
{
	can_t* can = can_get_can_from_line(line);
	return circ_buf_flex_pop(&can->buffer.buffer_rx, ts_rx_message);
}

void can_start(enum can_line line)
{
	can_t* can = can_get_can_from_line(line);
	mcan_start(&can->instance);
}

void can_stop(enum can_line line)
{
	can_t* can = can_get_can_from_line(line);
	mcan_stop(&can->instance);
}

enum can_filter_status can_add_id_mask_standard_filter(enum can_line line, uint32_t id, uint32_t mask, bool is_rejecting_filter)
{
	can_t* can = can_get_can_from_line(line);

	struct mcan_standard_message_filter_element sd_filter;
	mcan_get_standard_message_filter_element_default(&sd_filter);

	sd_filter.S0.bit.SFID1 = id;
	sd_filter.S0.bit.SFID2 = mask;
	if(is_rejecting_filter)
	{
		sd_filter.S0.bit.SFEC = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_REJECT_Val;//Reject
	}
	else
	{
		sd_filter.S0.bit.SFEC = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STF0M_Val;//Put in fifo0
	}

	sd_filter.S0.bit.SFT = 2;//classic filter

	if(can->standard_filter_index < can->standard_filter_number)
	{
		mcan_set_rx_standard_filter(&can->instance, &sd_filter, can->standard_filter_index++);
		return CAN_FILTER_OK;
	}

	return CAN_FILTER_STACK_FULL;
}

enum can_filter_status can_add_range_standard_filter(enum can_line line, uint32_t id_min, uint32_t id_max, bool is_rejecting_filter)
{
	can_t* can = can_get_can_from_line(line);

	struct mcan_standard_message_filter_element sd_filter;
	mcan_get_standard_message_filter_element_default(&sd_filter);

	sd_filter.S0.bit.SFID1 = id_min;
	sd_filter.S0.bit.SFID2 = id_max;
	if(is_rejecting_filter)
	{
		sd_filter.S0.bit.SFEC = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_REJECT_Val;//Reject
	}
	else
	{
		sd_filter.S0.bit.SFEC = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STF0M_Val;//Put in fifo0
	}

	sd_filter.S0.bit.SFT = 0;//range filter

	if(can->standard_filter_index < can->standard_filter_number)
	{
		mcan_set_rx_standard_filter(&can->instance, &sd_filter, can->standard_filter_index++);
		return CAN_FILTER_OK;
	}

	return CAN_FILTER_STACK_FULL;
}

enum can_filter_status can_add_dual_id_standard_filter(enum can_line line, uint32_t id1, uint32_t id2, bool is_rejecting_filter)
{
	can_t* can = can_get_can_from_line(line);

	struct mcan_standard_message_filter_element sd_filter;
	mcan_get_standard_message_filter_element_default(&sd_filter);

	sd_filter.S0.bit.SFID1 = id1;
	sd_filter.S0.bit.SFID2 = id2;
	if(is_rejecting_filter)
	{
		sd_filter.S0.bit.SFEC = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_REJECT_Val;//Reject
	}
	else
	{
		sd_filter.S0.bit.SFEC = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STF0M_Val;//Put in fifo0
	}

	sd_filter.S0.bit.SFT = 1;//dual id filter

	if(can->standard_filter_index < can->standard_filter_number)
	{
		mcan_set_rx_standard_filter(&can->instance, &sd_filter, can->standard_filter_index++);
		return CAN_FILTER_OK;
	}

	return CAN_FILTER_STACK_FULL;
}

enum can_filter_status can_add_id_mask_extended_filter(enum can_line line, uint32_t id, uint32_t mask, bool is_rejecting_filter)
{
	can_t* can = can_get_can_from_line(line);

	struct mcan_extended_message_filter_element et_filter;
	mcan_get_extended_message_filter_element_default(&et_filter);

	et_filter.F0.bit.EFID1 = id;
	et_filter.F1.bit.EFID2 = mask;
	
	if(is_rejecting_filter)
	{
		et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_REJECT_Val;//Reject
	}
	else
	{
		et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF1M_Val;//Put in fifo1
	}
	
	et_filter.F1.bit.EFT = 2;//classic filter

	if(can->extended_filter_index < can->extended_filter_number)
	{
		mcan_set_rx_extended_filter(&can->instance, &et_filter, can->extended_filter_index++);
		return CAN_FILTER_OK;
	}

	return CAN_FILTER_STACK_FULL;
}

enum can_filter_status can_add_range_extended_filter(enum can_line line, uint32_t id_min, uint32_t id_max, bool is_rejecting_filter)
{
	if(!(id_max >= id_min)) return CAN_FILTER_BAD_PARAMETERS;
	
	can_t* can = can_get_can_from_line(line);

	struct mcan_extended_message_filter_element et_filter;
	mcan_get_extended_message_filter_element_default(&et_filter);

	et_filter.F0.bit.EFID1 = id_min;
	et_filter.F1.bit.EFID2 = id_max;
	
	if(is_rejecting_filter)
	{
		et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_REJECT_Val;//Reject
	}
	else
	{
		et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF1M_Val;//Put in fifo1
	}
	
	et_filter.F1.bit.EFT = 0;//range filter

	if(can->extended_filter_index < can->extended_filter_number)
	{
		mcan_set_rx_extended_filter(&can->instance, &et_filter, can->extended_filter_index++);
		return CAN_FILTER_OK;
	}

	return CAN_FILTER_STACK_FULL;
}

enum can_filter_status can_add_dual_id_extended_filter(enum can_line line, uint32_t id1, uint32_t id2, bool is_rejecting_filter)
{
	can_t* can = can_get_can_from_line(line);

	struct mcan_extended_message_filter_element et_filter;
	mcan_get_extended_message_filter_element_default(&et_filter);

	et_filter.F0.bit.EFID1 = id1;
	et_filter.F1.bit.EFID2 = id2;
	
	if(is_rejecting_filter)
	{
		et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_REJECT_Val;//Reject
	}
	else
	{
		et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF1M_Val;//Put in fifo1
	}
	
	et_filter.F1.bit.EFT = 1;//range filter

	if(can->extended_filter_index < can->extended_filter_number)
	{
		mcan_set_rx_extended_filter(&can->instance, &et_filter, can->extended_filter_index++);
		return CAN_FILTER_OK;
	}

	return CAN_FILTER_STACK_FULL;
}

void can_reset_standard_filters(enum can_line line)
{
	can_t* can = can_get_can_from_line(line);

	struct mcan_standard_message_filter_element sd_filter;
	mcan_get_standard_message_filter_element_default(&sd_filter);

	sd_filter.S0.bit.SFEC = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_DISABLE_Val;//Disable filter

	for (uint8_t i = 0; i < can->standard_filter_number; i++)
	{
		mcan_set_rx_standard_filter(&can->instance, &sd_filter, i);
	}

	//reset filter index
	can->standard_filter_index = 0;

	//Reuse sd_filter but set it to accept all incoming standard messages.
	sd_filter.S0.bit.SFID1 = 0;//ID
	sd_filter.S0.bit.SFID2 = 0;//Mask
	sd_filter.S0.bit.SFEC = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STF0M_Val;//Put in fifo0
	sd_filter.S0.bit.SFT = 2;//classic filter

	mcan_set_rx_standard_filter(&can->instance, &sd_filter, 0);
}

void can_reset_extended_filters(enum can_line line)
{
	can_t* can = can_get_can_from_line(line);

	struct mcan_extended_message_filter_element et_filter;
	mcan_get_extended_message_filter_element_default(&et_filter);

	et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_DISABLE_Val;//Disable filter

	for (uint8_t i = 0; i < can->standard_filter_number; i++)
	{
		mcan_set_rx_extended_filter(&can->instance, &et_filter, i);
	}

	//reset filter index
	can->extended_filter_index = 0;

	//Reuse et_filter but set it to accept all incoming standard messages.
	et_filter.F0.bit.EFID1 = 0;//id
	et_filter.F1.bit.EFID2 = 0;//mask
	et_filter.F0.bit.EFEC = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF1M_Val;//Put in fifo1
	et_filter.F1.bit.EFT = 2;//classic filter

	//set all message filter to index 0
	mcan_set_rx_extended_filter(&can->instance, &et_filter, 0);

}