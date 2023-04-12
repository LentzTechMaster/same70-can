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
  ___ _  _ _   _ __  __ 
 | __| \| | | | |  \/  |
 | _|| .` | |_| | |\/| |
 |___|_|\_|\___/|_|  |_|
                        
*/

//Enumeration of most used CAN baudrates.
//This enumeration is non-exhaustive.
enum can_baudrate{
    CAN_BR_1_Mbps=1000000UL,
    CAN_BR_800_kbps=800000UL,
    CAN_BR_500_kbps=500000UL,
    CAN_BR_250_kbps=250000UL,
    CAN_BR_125_kbps=125000UL,
    CAN_BR_100_kbps=100000UL,
    CAN_BR_50_kbps=50000UL,
    CAN_BR_20_kbps=20000UL,
    CAN_BR_10_kbps=10000UL
    };

//Status for can filtering
enum can_filter_status{
    CAN_FILTER_OK,
    CAN_FILTER_STACK_FULL,
    CAN_FILTER_BAD_PARAMETERS
};

//Can line selection
enum can_line{
    CAN_LINE_0,
    CAN_LINE_1
};
/*
  ___ _____ ___ _   _  ___ _____ _   _ ___ ___    ___  ___ ___ ___ _  _ ___ _____ ___ ___  _  _ ___ 
 / __|_   _| _ \ | | |/ __|_   _| | | | _ \ __|  |   \| __| __|_ _| \| |_ _|_   _|_ _/ _ \| \| / __|
 \__ \ | | |   / |_| | (__  | | | |_| |   / _|   | |) | _|| _| | || .` || |  | |  | | (_) | .` \__ \
 |___/ |_| |_|_\\___/ \___| |_|  \___/|_|_\___|  |___/|___|_| |___|_|\_|___| |_| |___\___/|_|\_|___/
                                                                                                                                                                                                                                            
*/
typedef struct
{
	circ_buf_flex_t					buffer_rx;
	circ_buf_flex_t					buffer_tx;
    //flags
    volatile uint8_t adding_in_tx_buffer;
    //TODO It might be possible to make of these 2 flags only one
    volatile uint8_t interruption_occurred_while_adding_in_tx_buffer;
    volatile uint8_t buffer_being_emptied_by_interruption;
}can_buffer_t;

typedef struct 
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
} can_rx_message_t;

typedef struct 
{
	uint64_t					timestamp;
	can_rx_message_t			rx_message;
}can_timestamped_rx_message_t;


typedef struct 
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
} can_tx_message_t;

typedef struct can_t
{
    struct mcan_module instance;

    can_buffer_t buffer;

    //Those are value from conf_mcan.h (set in the can_configure)
    uint8_t buffer_tx_number;
    uint8_t standard_filter_number;
    uint8_t extended_filter_number;
    
    uint8_t standard_filter_index;
    uint8_t extended_filter_index;

    /* CAN message */
    volatile uint32_t standard_receive_index;
    volatile uint32_t extended_receive_index;
    struct mcan_rx_element_fifo_0 rx_element_fifo_0;
    struct mcan_rx_element_fifo_1 rx_element_fifo_1;
    struct mcan_rx_element_buffer rx_element_buffer;
}can_t;

/*
 __   ___   ___ ___   _   ___ _    ___    ___  ___ ___ ___ _  _ ___ _____ ___ ___  _  _ ___ 
 \ \ / /_\ | _ \_ _| /_\ | _ ) |  | __|  |   \| __| __|_ _| \| |_ _|_   _|_ _/ _ \| \| / __|
  \ V / _ \|   /| | / _ \| _ \ |__| _|   | |) | _|| _| | || .` || |  | |  | | (_) | .` \__ \
   \_/_/ \_\_|_\___/_/ \_\___/____|___|  |___/|___|_| |___|_|\_|___| |_| |___\___/|_|\_|___/
                                                                                            
*/

extern volatile uint64_t unix_timestamp_ms;

/*------------------CAN0--------------------*/
can_t can0;
/*------------------CAN1--------------------*/
can_t can1;

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
 * @brief 
 * 
 * @param line Can line to perform action on.
 * @return can_t* Pointer to can structure.
 */
static inline can_t* can_get_can_from_line(enum can_line line)
{
	switch (line)
	{
		case CAN_LINE_0:
			return &can0;
		case CAN_LINE_1:
			return &can1;
		default:
			return NULL;
	}
}

/**
 * @brief 
 * 
 * @param module_inst 
 * @param source 
 * @return uint32_t 
 */
static inline uint32_t mcan_get_interrupt(struct mcan_module *const module_inst, const enum mcan_interrupt_source source)
{
	return EXTRACT_X(module_inst->hw->MCAN_IE, source);
}

/**
 * @brief This sets 2 filters on the given CAN. 
 * One to accept all standard messages and to put them inside FIFO 0
 * Another one to accept all extended messages and to put them inside FIFO 1
 * 
 * @param module_inst MCAN instance.
 */
void _can_configure_rx_fifo_to_accept_all(struct mcan_module* module_inst);

/**
 * @brief Configure clocks, interruptions, initiate variables, ... and start the selected CAN
 * 
 * @param line Can line to perform action on.
 * @param baudrate baudrate in bits/second. Better if use one of the "enum can_baudrate".
 * @param rx_buffer_size Number of element the rx buffer can hold.
 * @param tx_buffer_size Number of element the tx buffer can hold.
 */
void can_configure(enum can_line line, uint32_t baudrate, uint32_t rx_buffer_size, uint32_t tx_buffer_size);

/**
 * @brief Pushes inside the rx buffer the received element composed of r0, r1 and data.
 * 
 * @param line Can line to perform action on.
 * @param r0 32 bit structure holding message information.
 * @param r1 32 bit structure holding message information.
 * @param data Pointer to data.
 * @param rec_timestamp Timestamp at which the message was received.
 */
void _can_push_message(enum can_line line, MCAN_RX_ELEMENT_R0_Type r0, MCAN_RX_ELEMENT_R1_Type r1, uint8_t* data, uint64_t rec_timestamp);


/**
 * @brief This sends a message WITHOUT using tx buffer and interruptions.
 * 
 * @param line Can line to perform action on.
 * @param id_value Message ID.
 * @param data Pointer to message data.
 * @param data_length Number of message data.
 * @param is_extended 1 if is extended, 0 if standard message.
 * @param is_remote_transmission 1 if is RTR, 0 if not.
 */
void _can_send_message(enum can_line line, uint32_t id_value, uint8_t *data, uint32_t data_length, bool is_extended, bool is_remote_transmission);

/**
 * @brief This sends a message using tx buffer and interruptions.
 * 
 * @param line Can line to perform action on.
 * @param id_value Message ID.
 * @param data Pointer to message data.
 * @param data_length Number of message data.
 * @param is_extended 1 if is extended, 0 if standard message.
 * @param is_remote_transmission 1 if is RTR, 0 if not.
 * @return uint8_t Buffer action status.
 */
uint8_t can_send_message(enum can_line line, uint32_t id_value, uint8_t *data, uint32_t data_length, bool is_extended, bool is_remote_transmission);

/**
 * @brief Give number of available messages inside the rx buffer.
 * 
 * @param line Can line to perform action on.
 * @return uint32_t Number of available messages.
 */
uint32_t can_available_message(enum can_line line);

/**
 * @brief Pops message from the rx buffer.
 * 
 * @param line Can line to perform action on.
 * @param ts_rx_message Pointer to timestamped rx message to write to.
 * @return uint8_t Buffer action status.
 */
uint8_t can_get_message(enum can_line line, can_timestamped_rx_message_t* ts_rx_message);

/**
 * @brief Starts CAN.
 * 
 * @param line Can line to perform action on.
 */
void can_start(enum can_line line);

/**
 * @brief Stops CAN.
 * 
 * @param line Can line to perform action on.
 */
void can_stop(enum can_line line);

/**
 * @brief Adds a id/mask filter in the standard filters stack.
 * This filters allows/rejects all IDs that satisfies the following condition: ID_received & filter_mask = filter_id & filter_mask
 * 
 * @param line Can line to perform action on.
 * @param id Filter ID.
 * @param mask Filter Mask.
 * @param is_rejecting_filter True: rejects ID if filter matches, False: allows ID if filter matches.
 * @return enum can_filter_status Filter operation status.
 */
enum can_filter_status can_add_id_mask_standard_filter(enum can_line line, uint32_t id, uint32_t mask, bool is_rejecting_filter);

/**
 * @brief Adds a range filter in the standard filters stack.
 * This filter allows/rejects all if between id_min and id_max.
 * 
 * @param line Can line to perform action on.
 * @param id_min Minimum ID accepted.
 * @param id_max Maximum ID accepted.
 * @param is_rejecting_filter True: rejects ID if filter matches, False: allows ID if filter matches.
 * @return enum can_filter_status Filter operation status.
 */
enum can_filter_status can_add_range_standard_filter(enum can_line line, uint32_t id_min, uint32_t id_max, bool is_rejecting_filter);

/**
 * @brief Adds a dual id filter in the standard filters stack.
 * This filter allows/rejects 2 ID to be filtered. These ID can be the same if only one ID need to be filtered. 
 * 
 * @param line Can line to perform action on.
 * @param id1 First ID accepted.
 * @param id2 Second ID accepted.
 * @param is_rejecting_filter True: rejects ID if filter matches, False: allows ID if filter matches.
 * @return enum can_filter_status Filter operation status.
 */
enum can_filter_status can_add_dual_id_standard_filter(enum can_line line, uint32_t id1, uint32_t id2, bool is_rejecting_filter);

/**
 * @brief Adds a id/mask filter in the extended filters stack.
 * This filters allows/rejects all IDs that satisfies the following condition: ID_received & filter_mask = filter_id & filter_mask
 * 
 * @param line Can line to perform action on.
 * @param id Filter ID.
 * @param mask Filter Mask.
 * @param is_rejecting_filter True: rejects ID if filter matches, False: allows ID if filter matches.
 * @return enum can_filter_status Filter operation status.
 */
enum can_filter_status can_add_id_mask_extended_filter(enum can_line line, uint32_t id, uint32_t mask, bool is_rejecting_filter);

/**
 * @brief Adds a range filter in the extended filters stack.
 * This filter allows/rejects all if between id_min and id_max.
 * 
 * @param line Can line to perform action on.
 * @param id_min Minimum ID accepted.
 * @param id_max Maximum ID accepted.
 * @param is_rejecting_filter True: rejects ID if filter matches, False: allows ID if filter matches.
 * @return enum can_filter_status Filter operation status.
 */
enum can_filter_status can_add_range_extended_filter(enum can_line line, uint32_t id_min, uint32_t id_max, bool is_rejecting_filter);

/**
 * @brief Adds a dual id filter in the extended filters stack.
 * This filter allows/rejects 2 ID to be filtered. These ID can be the same if only one ID need to be filtered. 
 * 
 * @param line Can line to perform action on.
 * @param id1 First ID accepted.
 * @param id2 Second ID accepted.
 * @param is_rejecting_filter True: rejects ID if filter matches, False: allows ID if filter matches.
 * @return enum can_filter_status Filter operation status.
 */
enum can_filter_status can_add_dual_id_extended_filter(enum can_line line, uint32_t id1, uint32_t id2, bool is_rejecting_filter);

/**
 * @brief Reset all standard filters. After this action, all standard messages are accepted.
 * 
 * @param line Can line to perform action on.
 */
void can_reset_standard_filters(enum can_line line);

/**
 * @brief Reset all extended filters. After this action, all extended messages are accepted.
 * 
 * @param line Can line to perform action on.
 */
void can_reset_extended_filters(enum can_line line);

#endif /* SAM70_CAN_DRIVER_H_ */