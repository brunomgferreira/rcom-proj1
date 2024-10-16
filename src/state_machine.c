#include "state_machine.h"
#include <stdio.h>
#include <string.h>

int status = 1;

int create_state_machine(struct state_machine *machine, enum state_machine_type type, unsigned char control_byte, unsigned char address_byte, enum state_machine_state state)
{
    machine->type = type;
    machine->control_byte = control_byte;
    machine->address_byte = address_byte;
    machine->state = state;
    memset(machine->buf, 0, sizeof(machine->buf));

    return 1;
}

int state_machine(struct state_machine *machine, unsigned char byte)
{
    switch (machine->state)
    {
    case START:
        status = 1;
        state_machine_START(machine, byte);
        break;

    case FLAG_RCV:
        status = 1;
        state_machine_FLAG_RCV(machine, byte);
        break;

    case A_RCV:
        status = state_machine_A_RCV(machine, byte);
        break;

    case C_RCV:
        state_machine_C_RCV(machine, byte);
        break;

    case BCC1_OK:
        int temp = state_machine_BCC1_OK(machine, byte);
        status = status == 0 ? status : temp;
        break;

    case STP:
        return status;

    default:
        printf("state_machine() ERROR!");
        return -1;
    }

    return status;
}

int state_machine_START(struct state_machine *machine, unsigned char byte)
{
    switch (byte)
    {
    case FLAG:
        machine->escape_sequence = 0;
        machine->state = FLAG_RCV;
        break;
    default:
        machine->state = START;
        break;
    }

    return 1;
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

    return 1;
}

int state_machine_A_RCV(struct state_machine *machine, unsigned char byte)
{

    if (byte == machine->control_byte)
    {
        machine->state = C_RCV;
    }
    else if (machine->type == WRITE && ((machine->control_byte == RR0 && byte == REJ0) || (machine->control_byte == RR1 && byte == REJ1)))
    {
        machine->state = C_RCV;
        return 0; // REJ received
    }
    else if (byte == FLAG)
    {
        machine->state = FLAG_RCV;
    }
    else
    {
        machine->state = START;
    }

    return 1;
}

int state_machine_C_RCV(struct state_machine *machine, unsigned char byte)
{
    if (byte == ESC)
    {
        machine->escape_sequence = 1;
        return 1;
    }

    if (machine->escape_sequence == 1)
    {
        machine->escape_sequence = 0;

        if (byte == 0x5E) // FLAG
        {
            byte = FLAG;
        }
        else if (byte == 0x5D) // ESC
        {
            byte = ESC;
        }
        else // Invalid escape sequence
        {
            machine->state = START;
        }

        if (machine->type == WRITE)
        {
            if ((machine->control_byte == RR0 && byte == (machine->address_byte ^ REJ0)) || (machine->control_byte == RR1 && byte == (machine->address_byte ^ REJ1)))
            {
                machine->buf_size = 0;
                machine->state = BCC1_OK;
            }
            else
            {
                machine->state = START;
            }
        }
        else if (byte == (machine->address_byte ^ machine->control_byte))
        {
            machine->buf_size = 0;
            machine->state = BCC1_OK;
        }
        else
        {
            machine->state = START;
        }

        return 1;
    }

    if (machine->type == WRITE)
    {
        if (byte == (machine->address_byte ^ machine->control_byte))
        {
            machine->buf_size = 0;
            machine->state = BCC1_OK;
        }
        else if ((machine->control_byte == RR0 && byte == (machine->address_byte ^ REJ0)) || (machine->control_byte == RR1 && byte == (machine->address_byte ^ REJ1)))
        {
            machine->buf_size = 0;
            machine->state = BCC1_OK;
        }
        else if (byte == FLAG)
        {
            machine->state = FLAG_RCV;
        }
        else
        {
            machine->state = START;
        }
    }
    else if (byte == (machine->address_byte ^ machine->control_byte))
    {
        machine->buf_size = 0;
        machine->state = BCC1_OK;
    }
    else if (byte == FLAG)
    {
        machine->state = FLAG_RCV;
    }
    else
    {
        machine->state = START;
    }

    return 1;
}

int state_machine_BCC1_OK(struct state_machine *machine, unsigned char byte)
{
    if (machine->type == READ)
    {
        switch (byte)
        {
        case FLAG:

            machine->buf_size -= 1; // Remove BCC2 byte

            unsigned char BCC2 = machine->buf[0];
            for (int i = 1; i < machine->buf_size; i++)
                BCC2 ^= machine->buf[i];

            // printf("\nBCC2 - 0x%02x\n", BCC2);

            if (machine->buf[machine->buf_size] == BCC2)
            {
                machine->state = STP;
            }
            else
            {
                machine->state = START;
                return 0; // Return 0 to send REJ frame.
            }

            break;
        default:

            if (machine->buf_size < sizeof(machine->buf))
            {
                if (machine->escape_sequence == 1) // Byte Destuffing
                {
                    machine->escape_sequence = 0;

                    if (byte == 0x5E)
                    {
                        machine->buf[machine->buf_size] = FLAG;
                    }
                    else if (byte == 0x5D)
                    {
                        machine->buf[machine->buf_size] = ESC;
                    }
                    else // Invalid escape sequence
                    {
                        // machine->state = START;
                        return 0; // Return 0 to send REJ frame.
                    }

                    machine->buf_size++;
                }
                else if (byte == ESC)
                {
                    machine->escape_sequence = 1;
                }
                else
                {
                    machine->buf[machine->buf_size] = byte;
                    machine->buf_size++;
                }
            }
            else
            {
                // Buffer overflow situation
                machine->state = START;
            }

            break;
        }
    }
    else
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
    }

    return 1;
}
