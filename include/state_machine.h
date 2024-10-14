// States of state machine
enum state_machine_state
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STP // STOP
};

struct state_machine
{
    unsigned char control_byte;
    unsigned char address_byte;
    enum state_machine_state state;
};

#define FLAG 0x7E
#define SENDER_ADDRESS 0x03
#define RECEIVER_ADDRESS 0x01
#define SET 0x03
#define UA 0x07

int create_state_machine(struct state_machine *machine, unsigned char control_byte, unsigned char address_byte, enum state_machine_state state);
int state_machine(struct state_machine *machine, unsigned char byte);
int state_machine_START(struct state_machine *machine, unsigned char byte);
int state_machine_FLAG_RCV(struct state_machine *machine, unsigned char byte);
int state_machine_A_RCV(struct state_machine *machine, unsigned char byte);
int state_machine_C_RCV(struct state_machine *machine, unsigned char byte);
int state_machine_BCC_OK(struct state_machine *machine, unsigned char byte);
