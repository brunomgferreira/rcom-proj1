#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#include "link_layer.h"

// Enumeration for the different states of the state machine
enum state_machine_state
{
    START,    // Initial state, waiting for FLAG
    FLAG_RCV, // FLAG received, waiting for Address byte
    A_RCV,    // Address byte received, waiting for Control byte
    C_RCV,    // Control byte received, waiting for BCC1
    BCC1_OK,  // BCC1 validated, waiting for data or FLAG
    STP       // STOP state, end of frame processing
};

// Enumeration for the different types of state machines
enum state_machine_type
{
    CONNECTION,   // State machine for connection establishment
    READ,         // State machine for reading data
    WRITE,        // State machine for writing data
    DISCONNECTION // State machine for disconnection
};

// Structure representing a state machine instance
struct state_machine
{
    enum state_machine_type type;                // Type of the state machine (e.g., READ, WRITE)
    unsigned char control_byte;                  // Control byte for the current frame
    unsigned char address_byte;                  // Address byte for the current frame
    enum state_machine_state state;              // Current state of the state machine
    unsigned char buf[MAX_PAYLOAD_SIZE * 2 + 2]; // Buffer for storing incoming data
    int buf_size;                                // Current size of the buffer
    unsigned char BCC1;                          // BCC1 value for error checking
    unsigned char BCC2;                          // BCC2 value for error checking
    unsigned char escape_sequence;               // Flag to indicate if an escape sequence is in progress
    unsigned char REJ;                           // REJ flag to indicate a rejection
    unsigned char ACK;                           // ACK flag to indicate acknowledgment
    unsigned char duplicate;                     // Flag to indicate a duplicate frame
};

// Structure to hold statistics related to link layer operations
struct ll_statistics
{
    int num_SET_sent;              // Number of SET frames sent
    int num_SET_received;          // Number of SET frames received
    int num_UA_sent;               // Number of UA frames sent
    int num_UA_received;           // Number of UA frames received
    int num_RR_sent;               // Number of RR frames sent
    int num_RR_received;           // Number of RR frames received
    int num_REJ_sent;              // Number of REJ frames sent
    int num_REJ_received;          // Number of REJ frames received
    int num_I_frames_sent;         // Number of I frames sent
    int num_I_frames_received;     // Number of I frames received
    int num_DISC_sent;             // Number of DISC frames sent
    int num_DISC_received;         // Number of DISC frames received
    int num_duplicated_frames;     // Number of duplicated frames detected
    int num_retransmissions;       // Number of retransmissions
    int num_timeouts;              // Number of timeouts
    int num_invalid_BCC1_received; // Number of invalid BCC1 received
    int num_invalid_BCC2_received; // Number of invalid BCC2 received
};

// Constants defining special bytes used in the protocol
#define FLAG 0x7E                           // Flag byte to denote start/end of frame
#define TRANSMITTER_ADDRESS 0x03            // Address of the transmitter
#define RECEIVER_ADDRESS 0x01               // Address of the receiver
#define REPLY_FROM_TRANSMITTER_ADDRESS 0x01 // Expected reply address from the transmitter
#define REPLY_FROM_RECEIVER_ADDRESS 0x03    // Expected reply address from the receiver
#define SET 0x03                            // Control byte for SET frame
#define UA 0x07                             // Control byte for UA frame
#define DISC 0x0B                           // Control byte for DISC frame
#define I_FRAME_0 0x00                      // Control byte for I frame 0
#define I_FRAME_1 0x80                      // Control byte for I frame 1
#define ESC 0x7D                            // Escape byte for byte-stuffing
#define ESC_FLAG 0x5E                       // Transformed FLAG byte after escaping
#define ESC_ESC 0x5D                        // Transformed ESC byte after escaping
#define RR0 0xAA                            // Control byte for RR frame 0
#define RR1 0xAB                            // Control byte for RR frame 1
#define REJ0 0x54                           // Control byte for REJ frame 0
#define REJ1 0x55                           // Control byte for REJ frame 1

// Extern declaration of statistics structure
extern struct ll_statistics statistics;

// Function declarations for state machine operations
void create_state_machine(struct state_machine *machine, enum state_machine_type type, unsigned char control_byte, unsigned char address_byte, enum state_machine_state state);
void process_read_BCC1_OK(struct state_machine *machine, unsigned char byte);
void state_machine(struct state_machine *machine, unsigned char byte);
void state_machine_START(struct state_machine *machine, unsigned char byte);
void state_machine_FLAG_RCV(struct state_machine *machine, unsigned char byte);
void state_machine_A_RCV(struct state_machine *machine, unsigned char byte);
void state_machine_C_RCV(struct state_machine *machine, unsigned char byte);
void state_machine_BCC1_OK(struct state_machine *machine, unsigned char byte);

#endif // _STATE_MACHINE_H_