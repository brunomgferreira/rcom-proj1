#include "state_machine.h"
#include <stdio.h>
#include <string.h>

void create_state_machine(struct state_machine *machine, enum state_machine_type type, unsigned char control_byte, unsigned char address_byte, enum state_machine_state state)
{
    machine->type = type;
    machine->control_byte = control_byte;
    machine->address_byte = address_byte;
    machine->state = state;
    memset(machine->buf, 0, sizeof(machine->buf));
    machine->REJ = 0;
    machine->ACK = 0;
    machine->buf_size = 0;
    machine->escape_sequence = 0;
}

void state_machine(struct state_machine *machine, unsigned char byte)
{
    switch (machine->state)
    {
    case START:
        state_machine_START(machine, byte);
        break;

    case FLAG_RCV:
        machine->REJ = 0;
        machine->ACK = 0;
        machine->BCC2 = 0;
        state_machine_FLAG_RCV(machine, byte);
        break;

    case A_RCV:
        state_machine_A_RCV(machine, byte);
        break;

    case C_RCV:
        state_machine_C_RCV(machine, byte);
        break;

    case BCC1_OK:
        state_machine_BCC1_OK(machine, byte);
        break;

    case STP:
        break;
    }
}

void state_machine_START(struct state_machine *machine, unsigned char byte)
{
    if (byte == FLAG)
    {
        machine->state = FLAG_RCV;
    }
}

void state_machine_FLAG_RCV(struct state_machine *machine, unsigned char byte)
{
    if (byte == machine->address_byte)
    {
        machine->state = A_RCV;
    }
    else if (byte != FLAG)
    {
        machine->state = START;
    }
}

void state_machine_A_RCV(struct state_machine *machine, unsigned char byte)
{
    if (byte == machine->control_byte)
    {
        machine->state = C_RCV;
    }
    else if (machine->type == WRITE &&
             ((machine->control_byte == RR0 && byte == REJ0) ||
              (machine->control_byte == RR1 && byte == REJ1)))
    {
        machine->state = C_RCV;
        machine->REJ = 1; // REJ received
    }
    else if (machine->type == READ && byte == SET)
    {
        machine->state = C_RCV;
        machine->ACK = 1; // SET received
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

void state_machine_C_RCV(struct state_machine *machine, unsigned char byte)
{
    if ((machine->type == WRITE &&
         ((machine->control_byte == RR0 && byte == (machine->address_byte ^ REJ0)) ||
          (machine->control_byte == RR1 && byte == (machine->address_byte ^ REJ1)))) ||
        byte == (machine->address_byte ^ machine->control_byte) ||
        (machine->type == READ && machine->ACK && byte == (machine->address_byte ^ SET)))
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

void state_machine_BCC1_OK(struct state_machine *machine, unsigned char byte)
{
    if (machine->type == READ && !machine->ACK)
    {
        process_read_BCC1_OK(machine, byte);
    }
    else if (byte == FLAG)
    {
        machine->state = STP;
    }
    else
    {
        machine->state = START;
    }
}

void process_read_BCC1_OK(struct state_machine *machine, unsigned char byte)
{
    if (byte == FLAG)
    {
        machine->buf_size--;                              // Remove BCC2 byte
        machine->BCC2 ^= machine->buf[machine->buf_size]; // Remove BCC2 byte

        if (machine->buf[machine->buf_size] == machine->BCC2)
        {
            machine->state = STP;
        }
        else
        {
            machine->state = START;
            machine->REJ = 1; // Send REJ frame.
        }
    }
    else if (machine->buf_size < sizeof(machine->buf))
    {
        if (machine->escape_sequence) // Byte Destuffing
        {
            machine->escape_sequence = 0;
            byte = (byte == 0x5E) ? FLAG : (byte == 0x5D) ? ESC
                                                          : 0;
            if (!byte)
            {
                machine->REJ = 1; // Invalid escape, send REJ
                return;
            }
        }
        else if (byte == ESC)
        {
            machine->escape_sequence = 1;
            return;
        }

        machine->buf[machine->buf_size++] = byte;
        machine->BCC2 ^= byte;
    }
    else
    {
        machine->state = START; // Buffer overflow
    }
}