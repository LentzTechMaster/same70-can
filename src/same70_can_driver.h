/*
This driver will help you to manage the 2 can lines present on the SAM E70 (mcan0 and mcan1).

This driver has a dependency found at: https://github.com/LentzTechMaster/c-utils.git (it uses the circular buffer flex from it).
And you'll need to import the mcan driver from ASF in your project.


/!\ You need a specific configuration of the conf_mcan.h in order to make this driver work !!!
/!\ Don't forget to enable MCAN0 and MCAN1 inside the conf_board.h !!!

You can find a demo at: https://github.com/LentzTechMaster/same70-canDemo.git




███╗   ███╗ █████╗ ██████╗ ███████╗    ██╗    ██╗██╗████████╗██╗  ██╗    ██╗      ██████╗ ██╗   ██╗███████╗    ██████╗ ██╗   ██╗
████╗ ████║██╔══██╗██╔══██╗██╔════╝    ██║    ██║██║╚══██╔══╝██║  ██║    ██║     ██╔═══██╗██║   ██║██╔════╝    ██╔══██╗╚██╗ ██╔╝
██╔████╔██║███████║██║  ██║█████╗      ██║ █╗ ██║██║   ██║   ███████║    ██║     ██║   ██║██║   ██║█████╗      ██████╔╝ ╚████╔╝ 
██║╚██╔╝██║██╔══██║██║  ██║██╔══╝      ██║███╗██║██║   ██║   ██╔══██║    ██║     ██║   ██║╚██╗ ██╔╝██╔══╝      ██╔══██╗  ╚██╔╝  
██║ ╚═╝ ██║██║  ██║██████╔╝███████╗    ╚███╔███╔╝██║   ██║   ██║  ██║    ███████╗╚██████╔╝ ╚████╔╝ ███████╗    ██████╔╝   ██║   
╚═╝     ╚═╝╚═╝  ╚═╝╚═════╝ ╚══════╝     ╚══╝╚══╝ ╚═╝   ╚═╝   ╚═╝  ╚═╝    ╚══════╝ ╚═════╝   ╚═══╝  ╚══════╝    ╚═════╝    ╚═╝   
                                                                                                                                

██████╗ ███████╗███╗   ██╗     ██╗ █████╗ ███╗   ███╗██╗███╗   ██╗    ██╗     ███████╗███╗   ██╗████████╗███████╗
██╔══██╗██╔════╝████╗  ██║     ██║██╔══██╗████╗ ████║██║████╗  ██║    ██║     ██╔════╝████╗  ██║╚══██╔══╝╚══███╔╝
██████╔╝█████╗  ██╔██╗ ██║     ██║███████║██╔████╔██║██║██╔██╗ ██║    ██║     █████╗  ██╔██╗ ██║   ██║     ███╔╝ 
██╔══██╗██╔══╝  ██║╚██╗██║██   ██║██╔══██║██║╚██╔╝██║██║██║╚██╗██║    ██║     ██╔══╝  ██║╚██╗██║   ██║    ███╔╝  
██████╔╝███████╗██║ ╚████║╚█████╔╝██║  ██║██║ ╚═╝ ██║██║██║ ╚████║    ███████╗███████╗██║ ╚████║   ██║   ███████╗
╚═════╝ ╚══════╝╚═╝  ╚═══╝ ╚════╝ ╚═╝  ╚═╝╚═╝     ╚═╝╚═╝╚═╝  ╚═══╝    ╚══════╝╚══════╝╚═╝  ╚═══╝   ╚═╝   ╚══════╝
                                                                                                                 
licence   : MIT 
*/


#ifndef SAM70_CAN_DRIVER_H_
#define SAM70_CAN_DRIVER_H_

/*
  ___ _  _  ___ _   _   _ ___  ___ ___ 
 |_ _| \| |/ __| | | | | |   \| __/ __|
  | || .` | (__| |_| |_| | |) | _|\__ \
 |___|_|\_|\___|____\___/|___/|___|___/
                                       
*/
#include "compiler.h"

#include <mcan.h>// From module: CAN - Control Area Network Controller from ASF
#include "genclk.h"

#include "circular-buffer-flex.h"

/*
  ___  ___ ___ ___ _  _ ___ ___ 
 |   \| __| __|_ _| \| | __/ __|
 | |) | _|| _| | || .` | _|\__ \
 |___/|___|_| |___|_|\_|___|___/
                                
*/

/*
  ___ _____ ___ _   _  ___ _____ _   _ ___ ___    ___  ___ ___ ___ _  _ ___ _____ ___ ___  _  _ ___ 
 / __|_   _| _ \ | | |/ __|_   _| | | | _ \ __|  |   \| __| __|_ _| \| |_ _|_   _|_ _/ _ \| \| / __|
 \__ \ | | |   / |_| | (__  | | | |_| |   / _|   | |) | _|| _| | || .` || |  | |  | | (_) | .` \__ \
 |___/ |_| |_|_\\___/ \___| |_|  \___/|_|_\___|  |___/|___|_| |___|_|\_|___| |_| |___\___/|_|\_|___/
                                                                                                                                                                                                                                            
*/
typedef struct mcan_buffer_t {
	circ_buf_flex_t					buffer_rx;
	circ_buf_flex_t					buffer_tx;
    //flags
    volatile uint8_t adding_in_tx_buffer;
    //TODO It might be possible to make of these 2 flags only one
    volatile uint8_t interruption_occured_while_adding_in_tx_buffer;
    volatile uint8_t buffer_being_emptied_by_interruption;
}mcan_buffer_t;

typedef struct mcan_rx_message_t 
{
    //message info
	union 
    {
        struct 
        {
            uint8_t reserved 							: 3;
			uint8_t is_received_with_bitrate_switch     : 1;
            uint8_t is_fd      							: 1;
            uint8_t is_error     						: 1;
            uint8_t is_remote_request      				: 1;
            uint8_t is_extended      					: 1;
        } bit;
        uint8_t all_8_bits;
    } message_info;
	
	uint32_t id;

    uint8_t dlc;
    uint8_t data[CONF_MCAN_ELEMENT_DATA_SIZE];
} mcan_rx_message_t;

typedef struct mcan_timestamped_rx_message_t {
	uint64_t					timestamp;
	mcan_rx_message_t			rx_message;
}mcan_timestamped_rx_message_t;


typedef struct mcan_tx_message_t 
{
    //message info
	union {
        struct {
            uint8_t reserved 							: 6;
            uint8_t is_remote_request      				: 1;
            uint8_t is_extended      					: 1;
        } bit;
        uint8_t all_8_bits;
    } message_info;
	
	uint32_t id;

    uint8_t dlc;
    uint8_t data[CONF_MCAN_ELEMENT_DATA_SIZE];
} mcan_tx_message_t;


/*
 __   ___   ___ ___   _   ___ _    ___    ___  ___ ___ ___ _  _ ___ _____ ___ ___  _  _ ___ 
 \ \ / /_\ | _ \_ _| /_\ | _ ) |  | __|  |   \| __| __|_ _| \| |_ _|_   _|_ _/ _ \| \| / __|
  \ V / _ \|   /| | / _ \| _ \ |__| _|   | |) | _|| _| | || .` || |  | |  | | (_) | .` \__ \
   \_/_/ \_\_|_\___/_/ \_\___/____|___|  |___/|___|_| |___|_|\_|___| |_| |___\___/|_|\_|___/
                                                                                            
*/

mcan_buffer_t mcan0_buffer;
mcan_buffer_t mcan1_buffer;

extern volatile uint64_t unix_timestamp_ms;

struct mcan_module mcan0_instance;
/* CAN0 message */
volatile uint32_t mcan0_standard_receive_index;
volatile uint32_t mcan0_extended_receive_index;
struct mcan_rx_element_fifo_0 mcan0_rx_element_fifo_0;
struct mcan_rx_element_fifo_1 mcan0_rx_element_fifo_1;
struct mcan_rx_element_buffer mcan0_rx_element_buffer;

struct mcan_module mcan1_instance;
/* CAN1 message */
volatile uint32_t mcan1_standard_receive_index;
volatile uint32_t mcan1_extended_receive_index;
struct mcan_rx_element_fifo_0 mcan1_rx_element_fifo_0;
struct mcan_rx_element_fifo_1 mcan1_rx_element_fifo_1;
struct mcan_rx_element_buffer mcan1_rx_element_buffer;

/*
  __  __   _   ___ ___  ___  ___ 
 |  \/  | /_\ / __| _ \/ _ \/ __|
 | |\/| |/ _ \ (__|   / (_) \__ \
 |_|  |_/_/ \_\___|_|_\\___/|___/
                                 
*/
#define GET_BYTE(val, idx) ((val >> (idx * 8)) & 0xFF)
#define SHIFT_VALUE(mask) (__builtin_ctz(mask))
#define EXTRACT_X(bin, mask) ((bin & mask) >> SHIFT_VALUE(mask))

/**
 * @brief Takes bits from from a binary word X from bit starting_bit to bit ending_bit INCLUDED.
 * example:
 * int my_bin = 0bQERTYUIO
 * GET_BITS(my_bin, 2, 5) = 0bRTYU
 * 
 */
#define GET_BITS(bin, starting_bit, ending_bit) (((bin) >> (starting_bit)) & ((1 << ((ending_bit) - (starting_bit) + 1)) - 1))

/*
  ___ _   _ _  _  ___ _____ ___ ___  _  _    ___ ___  ___ _____ ___ _______   _____ ___ ___ 
 | __| | | | \| |/ __|_   _|_ _/ _ \| \| |  | _ \ _ \/ _ \_   _/ _ \_   _\ \ / / _ \ __/ __|
 | _|| |_| | .` | (__  | |  | | (_) | .` |  |  _/   / (_) || || (_) || |  \ V /|  _/ _|\__ \
 |_|  \___/|_|\_|\___| |_| |___\___/|_|\_|  |_| |_|_\\___/ |_| \___/ |_|   |_| |_| |___|___/
                                                                                            
*/

/**
 * @brief This sets 2 filters on the given CAN. 
 * One to accept all standard messages and to put them inside FIFO 0
 * Another one to accept all extended messages and to put them inside FIFO 1
 * 
 * @param mcan_mod MCAN instance.
 */
void _mcan_configure_rx_fifo_to_accept_all(struct mcan_module* mcan_mod);

/**
 * @brief Configure clocks, interruptions, initiate variables, ... for MCAN0 
 * 
 * @param rx_buffer_size Number of element the rx buffer can hold.
 * @param tx_buffer_size Number of element the tx buffer can hold.
 */
void mcan0_configure(uint32_t rx_buffer_size, uint32_t tx_buffer_size);

/**
 * @brief Configure clocks, interruptions, initiate variables, ... for MCAN1 
 * 
 * @param rx_buffer_size Number of element the rx buffer can hold.
 * @param tx_buffer_size Number of element the tx buffer can hold.
 */
void mcan1_configure(uint32_t rx_buffer_size, uint32_t tx_buffer_size);

/**
 * @brief Pushes inside the rx buffer the received element composed of r0, r1 and data.
 * 
 * 
 * @param r0 32 bit structure holding message information.
 * @param r1 32 bit structure holding message information.
 * @param data Pointer to data.
 * @param rec_timestamp Timestamp at which the message was received.
 */
void _mcan0_push_message(MCAN_RX_ELEMENT_R0_Type r0, MCAN_RX_ELEMENT_R1_Type r1, uint8_t* data, uint64_t rec_timestamp);

/**
 * @brief Pushes inside the rx buffer the received element composed of r0, r1 and data.
 * 
 * @param r0 32 bit structure holding message information.
 * @param r1 32 bit structure holding message information.
 * @param data Pointer to message data.
 * @param rec_timestamp Timestamp at which the message was received.
 */
void _mcan1_push_message(MCAN_RX_ELEMENT_R0_Type r0, MCAN_RX_ELEMENT_R1_Type r1, uint8_t* data, uint64_t rec_timestamp);

/**
 * @brief This sends a message WITHOUT using tx buffer and interruptions.
 * 
 * @param module_inst MCAN instance.
 * @param id_value Message ID.
 * @param data Pointer to message data.
 * @param data_length Number of message data.
 * @param is_extended 1 if is extended, 0 if standard message.
 * @param is_remote_transmition 1 if is RTR, 0 if not.
 */
void _mcan_send_message(struct mcan_module *const module_inst, uint32_t id_value, uint8_t *data, uint32_t data_length, bool is_extended, bool is_remote_transmition);

/**
 * @brief This sends a message using tx buffer and interruptions.
 * 
 * @param id_value Message ID.
 * @param data Pointer to message data.
 * @param data_length Number of message data.
 * @param is_extended 1 if is extended, 0 if standard message.
 * @param is_remote_transmition 1 if is RTR, 0 if not.
 * @return uint8_t Buffer action status.
 */
uint8_t mcan0_send_message(uint32_t id_value, uint8_t *data, uint32_t data_length, bool is_extended, bool is_remote_transmition);

/**
 * @brief This sends a message using tx buffer and interruptions.
 * 
 * @param id_value Message ID.
 * @param data Pointer to message data.
 * @param data_length Number of message data.
 * @param is_extended 1 if is extended, 0 if standard message.
 * @param is_remote_transmition 1 if is RTR, 0 if not.
 * @return uint8_t Buffer action status.
 */
uint8_t mcan1_send_message(uint32_t id_value, uint8_t *data, uint32_t data_length, bool is_extended, bool is_remote_transmition);

/**
 * @brief Give number of available messages inside the rx buffer.
 * 
 * @return uint32_t Number of available messages.
 */
uint32_t mcan0_available_message(void);

/**
 * @brief Give number of available messages inside the rx buffer.
 * 
 * @return uint32_t Number of available messages.
 */
uint32_t mcan1_available_message(void);

/**
 * @brief Pops message from the rx buffer.
 * 
 * @param ts_rx_message Pointer totimestamped rx message to write to.
 * @return uint8_t Buffer action status.
 */
uint8_t mcan0_get_message(mcan_timestamped_rx_message_t* ts_rx_message);

/**
 * @brief Pops message from the rx buffer.
 * 
 * @param ts_rx_message Pointer to timestamped rx message to write to.
 * @return uint8_t Buffer action status.
 */
uint8_t mcan1_get_message(mcan_timestamped_rx_message_t* ts_rx_message);

#endif /* SAM70_CAN_DRIVER_H_ */