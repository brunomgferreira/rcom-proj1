#include "link_layer.h"

// States of state machine
enum state_machine_state
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    STP // STOP
};

enum state_machine_type
{
    CONNECTION,
    READ,
    WRITE,
    DISCONNECTION
};

struct state_machine
{
    enum state_machine_type type;
    unsigned char control_byte;
    unsigned char address_byte;
    enum state_machine_state state;
    unsigned char buf[MAX_PAYLOAD_SIZE * 2 + 2];
    int buf_size;
    unsigned char BCC1;
    unsigned char BCC2;
    unsigned char escape_sequence;
    unsigned char REJ;
    unsigned char ACK;
    unsigned char duplicate;
};

struct ll_statistics
{
    int num_SET_sent;
    int num_SET_received;
    int num_UA_sent;
    int num_UA_received;
    int num_RR_sent;
    int num_RR_received;
    int num_REJ_sent;
    int num_REJ_received;
    int num_I_frames_sent;
    int num_I_frames_received;
    int num_DISC_sent;
    int num_DISC_received;
    int num_duplicated_frames;
    int num_retransmissions;
    int num_timeouts;
    int num_invalid_BCC1_received;
    int num_invalid_BCC2_received;
};

#define FLAG 0x7E
#define TRANSMITTER_ADDRESS 0x03
#define RECEIVER_ADDRESS 0x01
#define REPLY_FROM_TRANSMITTER_ADDRESS 0x01
#define REPLY_FROM_RECEIVER_ADDRESS 0x03
#define SET 0x03
#define UA 0x07
#define DISC 0x0B
#define I_FRAME_0 0x00
#define I_FRAME_1 0x80
#define ESC 0x7D
#define ESC_FLAG 0x5E // TODO
#define ESC_ESC 0x5D  // TODO
#define RR0 0xAA
#define RR1 0xAB
#define REJ0 0x54
#define REJ1 0x55

extern struct ll_statistics statistics;

void create_state_machine(struct state_machine *machine, enum state_machine_type type, unsigned char control_byte, unsigned char address_byte, enum state_machine_state state);
void process_read_BCC1_OK(struct state_machine *machine, unsigned char byte);
void state_machine(struct state_machine *machine, unsigned char byte);
void state_machine_START(struct state_machine *machine, unsigned char byte);
void state_machine_FLAG_RCV(struct state_machine *machine, unsigned char byte);
void state_machine_A_RCV(struct state_machine *machine, unsigned char byte);
void state_machine_C_RCV(struct state_machine *machine, unsigned char byte);
void state_machine_BCC1_OK(struct state_machine *machine, unsigned char byte);
