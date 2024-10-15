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
    unsigned char buf[512]; // It's bigger than 256 to account for byte stuffing - TODO CHANGE THIS LATER - Make it dynamic ???
    int buf_size;
    unsigned char escape_sequence;
};

#define FLAG 0x7E
#define SENDER_ADDRESS 0x03
#define RECEIVER_ADDRESS 0x01
#define SET 0x03
#define UA 0x07
#define I_FRAME_0 0x00
#define I_FRAME_1 0x80
#define ESC 0x7D
#define RR0 0xAA
#define RR1 0xAB
#define REJ0 0x54
#define REJ1 0x55

int create_state_machine(struct state_machine *machine, enum state_machine_type type, unsigned char control_byte, unsigned char address_byte, enum state_machine_state state);
int state_machine(struct state_machine *machine, unsigned char byte);
int state_machine_START(struct state_machine *machine, unsigned char byte);
int state_machine_FLAG_RCV(struct state_machine *machine, unsigned char byte);
int state_machine_A_RCV(struct state_machine *machine, unsigned char byte);
int state_machine_C_RCV(struct state_machine *machine, unsigned char byte);
int state_machine_BCC1_OK(struct state_machine *machine, unsigned char byte);
