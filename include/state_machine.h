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
    WRITE
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

#define FLAG 0x7E
#define TRANSMITTER_ADDRESS 0x03
#define RECEIVER_ADDRESS 0x01
#define REPLY_FROM_TRANSMITTER_ADDRESS 0x01
#define REPLY_FROM_RECEIVER_ADDRESS 0x03
#define SET 0x03
#define UA 0x07
#define I_FRAME_0 0x00
#define I_FRAME_1 0x80
#define ESC 0x7D
#define ESC_FLAG 0x5E // TODO
#define ESC_ESC 0x5D  // TODO
#define RR0 0xAA
#define RR1 0xAB
#define REJ0 0x54
#define REJ1 0x55

void create_state_machine(struct state_machine *machine, enum state_machine_type type, unsigned char control_byte, unsigned char address_byte, enum state_machine_state state);
void process_read_BCC1_OK(struct state_machine *machine, unsigned char byte);
void state_machine(struct state_machine *machine, unsigned char byte);
void state_machine_START(struct state_machine *machine, unsigned char byte);
void state_machine_FLAG_RCV(struct state_machine *machine, unsigned char byte);
void state_machine_A_RCV(struct state_machine *machine, unsigned char byte);
void state_machine_C_RCV(struct state_machine *machine, unsigned char byte);
void state_machine_BCC1_OK(struct state_machine *machine, unsigned char byte);
