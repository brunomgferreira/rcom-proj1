#include "state_machine.h"
#include <stdio.h>
#include <string.h>

// Structure to hold statistics for various control frames and events
struct ll_statistics statistics = {
    .num_SET_sent = 0,
    .num_SET_received = 0,
    .num_UA_sent = 0,
    .num_UA_received = 0,
    .num_RR_sent = 0,
    .num_RR_received = 0,
    .num_REJ_sent = 0,
    .num_REJ_received = 0,
    .num_I_frames_sent = 0,
    .num_I_frames_received = 0,
    .num_DISC_sent = 0,
    .num_DISC_received = 0,
    .num_duplicated_frames = 0,
    .num_retransmissions = 0,
    .num_timeouts = 0,
    .num_invalid_BCC1_received = 0,
    .num_invalid_BCC2_received = 0};

// Function to initialize a state machine with given parameters
void create_state_machine(struct state_machine *machine, enum state_machine_type type, unsigned char control_byte, unsigned char address_byte, enum state_machine_state state)
{
    machine->type = type;                          // Set the type of state machine
    machine->control_byte = control_byte;          // Set the control byte
    machine->address_byte = address_byte;          // Set the address byte
    machine->state = state;                        // Initialize state
    memset(machine->buf, 0, sizeof(machine->buf)); // Clear the buffer
    machine->REJ = 0;                              // Initialize REJ flag
    machine->ACK = 0;                              // Initialize ACK flag
    machine->buf_size = 0;                         // Initialize buffer size
    machine->escape_sequence = 0;                  // Initialize escape sequence flag
    machine->duplicate = 0;                        // Initialize duplicate flag
}

// Main function for the state machine processing a byte
void state_machine(struct state_machine *machine, unsigned char byte)
{
    switch (machine->state)
    {
    case START:
        state_machine_START(machine, byte); // Handle START state
        break;

    case FLAG_RCV:
        machine->REJ = 0;                      // Reset REJ flag
        machine->ACK = 0;                      // Reset ACK flag
        machine->BCC2 = 0;                     // Reset BCC2 for new frame
        machine->duplicate = 0;                // Reset duplicate flag
        state_machine_FLAG_RCV(machine, byte); // Handle FLAG_RCV state
        break;

    case A_RCV:
        state_machine_A_RCV(machine, byte); // Handle A_RCV state
        break;

    case C_RCV:
        state_machine_C_RCV(machine, byte); // Handle C_RCV state
        break;

    case BCC1_OK:
        state_machine_BCC1_OK(machine, byte); // Handle BCC1_OK state
        break;

    case STP:
        break; // STP state does nothing
    }
}

// Handle the START state, checking for the FLAG byte
void state_machine_START(struct state_machine *machine, unsigned char byte)
{
    if ((unsigned char)byte == (unsigned char)FLAG)
    {
        machine->state = FLAG_RCV; // Move to FLAG_RCV state
    }
}

// Handle the FLAG_RCV state, checking for the address byte
void state_machine_FLAG_RCV(struct state_machine *machine, unsigned char byte)
{
    if (byte == machine->address_byte)
    {
        machine->state = A_RCV; // Move to A_RCV state
        machine->BCC1 = byte;   // Set BCC1 to address byte
    }
    else if (byte != FLAG)
    {
        machine->state = START; // Invalid byte; reset to START
    }
}

// Handle the A_RCV state, checking for the control byte
void state_machine_A_RCV(struct state_machine *machine, unsigned char byte)
{
    if (byte == machine->control_byte)
    {
        machine->state = C_RCV; // Move to C_RCV state
        machine->BCC1 ^= byte;  // Update BCC1 with control byte
    }
    else if (machine->type == WRITE &&
             ((machine->control_byte == RR0 && byte == REJ1) ||
              (machine->control_byte == RR1 && byte == REJ0)))
    {
        machine->state = C_RCV; // Move to C_RCV for REJ
        machine->REJ = 1;       // REJ received
        machine->BCC1 ^= byte;  // Update BCC1
    }
    else if (machine->type == READ && byte == SET)
    {
        machine->state = C_RCV; // Move to C_RCV for ACK
        machine->ACK = 1;       // SET received
        machine->BCC1 ^= byte;  // Update BCC1
    }
    else if (machine->type == READ &&
             ((machine->control_byte == I_FRAME_0 && byte == I_FRAME_1) ||
              (machine->control_byte == I_FRAME_1 && byte == I_FRAME_0)))
    {
        machine->state = C_RCV; // Move to C_RCV for duplicate
        machine->duplicate = 1; // Duplicate received
        machine->BCC1 ^= byte;  // Update BCC1
    }
    else if (byte == FLAG)
    {
        machine->state = FLAG_RCV; // Return to FLAG_RCV state
    }
    else
    {
        machine->state = START; // Invalid byte; reset to START
    }
}

// Handle the C_RCV state, checking for the BCC1 byte
void state_machine_C_RCV(struct state_machine *machine, unsigned char byte)
{
    if (byte == machine->BCC1)
    {
        machine->buf_size = 0;    // Reset buffer size
        machine->state = BCC1_OK; // Move to BCC1_OK state
    }
    else if (byte == FLAG)
    {
        machine->state = FLAG_RCV; // Return to FLAG_RCV state
    }
    else
    {
        statistics.num_invalid_BCC1_received++; // Increment invalid BCC1 count
        machine->state = START;                 // Invalid byte; reset to START
    }
}

// Handle the BCC1_OK state, processing incoming data
void state_machine_BCC1_OK(struct state_machine *machine, unsigned char byte)
{
    if (machine->type == READ && !machine->ACK)
    {
        process_read_BCC1_OK(machine, byte); // Process read operation
    }
    else if (byte == FLAG)
    {
        machine->state = STP; // Move to STP on FLAG
    }
    else
    {
        machine->state = START; // Invalid byte; reset to START
    }
}

// Process data in the BCC1_OK state for READ state machine type
void process_read_BCC1_OK(struct state_machine *machine, unsigned char byte)
{
    if (byte == FLAG)
    {
        machine->buf_size--;                              // Remove BCC2 byte from buffer
        machine->BCC2 ^= machine->buf[machine->buf_size]; // Update BCC2

        // Check if the received BCC2 matches the expected BCC2
        if (machine->buf[machine->buf_size] == machine->BCC2)
        {
            machine->state = STP; // Valid frame; move to STP state
        }
        else
        {
            statistics.num_invalid_BCC2_received++; // Increment invalid BCC2 count
            machine->state = STP;                   // Invalid BCC2; move to STP
            machine->REJ = 1;                       // Set REJ to indicate error
        }
    }
    else if (machine->buf_size >= 0 && machine->buf_size < sizeof(machine->buf))
    {
        // Handle byte destuffing
        if (machine->escape_sequence) // If escape sequence was initiated
        {
            machine->escape_sequence = 0; // Reset escape sequence
            byte = (byte == ESC_FLAG) ? FLAG : (byte == ESC_ESC) ? ESC
                                                                 : 0; // Map escaped byte
            if (!byte)
            {
                machine->REJ = 1; // Invalid escape; set REJ
                return;           // Exit processing
            }
        }
        else if (byte == ESC)
        {
            machine->escape_sequence = 1; // Set escape sequence flag
            return;                       // Wait for the next byte
        }

        // Add byte to buffer and update BCC2
        machine->buf[machine->buf_size++] = byte;
        machine->BCC2 ^= byte; // Update BCC2 with the new byte
    }
    else
    {
        machine->state = START; // Buffer overflow; reset to START
    }
}