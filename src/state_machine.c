#include "state_machine.h"
#include <stdio.h>

int create_state_machine(struct state_machine *machine, unsigned char control_byte, unsigned char address_byte, enum state_machine_state state)
{
    machine->control_byte = control_byte;
    machine->address_byte = address_byte;
    machine->state = state;

    return 0;
}

int state_machine(struct state_machine *machine, unsigned char byte)
{
    switch (machine->state)
    {
    case START:
        state_machine_START(machine, byte);
        break;

    case FLAG_RCV:
        state_machine_FLAG_RCV(machine, byte);
        break;

    case A_RCV:
        state_machine_A_RCV(machine, byte);
        break;

    case C_RCV:
        state_machine_C_RCV(machine, byte);
        break;

    case BCC_OK:
        state_machine_BCC_OK(machine, byte);
        break;

    case STP:
        return 0;

    default:
        printf("state_machine() ERROR!");
        return -1;
    }

    return 0;
}

int state_machine_START(struct state_machine *machine, unsigned char byte)
{
    switch (byte)
    {
    case FLAG:
        machine->state = FLAG_RCV;
        break;
    default:
        machine->state = START;
        break;
    }

    return 0;
}

int state_machine_FLAG_RCV(struct state_machine *machine, unsigned char byte)
{
    if (byte == machine->address_byte)
    {
        machine->state = A_RCV;
    }
    else if (byte == FLAG)
    {
        machine->state = FLAG_RCV;
    }
    else
    {
        machine->state = START;
    }

    return 0;
}

int state_machine_A_RCV(struct state_machine *machine, unsigned char byte)
{
    if (byte == machine->control_byte)
    {
        machine->state = C_RCV;
    }
    else if (byte == FLAG)
    {
        machine->state = FLAG_RCV;
    }
    else
    {
        machine->state = START;
    }

    return 0;
}

int state_machine_C_RCV(struct state_machine *machine, unsigned char byte)
{
    if (byte == (machine->address_byte ^ machine->control_byte))
    {
        machine->state = BCC_OK;
    }
    else if (byte == FLAG)
    {
        machine->state = FLAG_RCV;
    }
    else
    {
        machine->state = START;
    }

    return 0;
}

int state_machine_BCC_OK(struct state_machine *machine, unsigned char byte)
{
    switch (byte)
    {
    case FLAG:
        machine->state = STP;
        break;
    default:
        machine->state = START;
        break;
    }

    return 0;
}
