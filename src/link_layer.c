// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "state_machine.h"
#include "alarm.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

int timeout = 0;
int nRetransmissions = 0;
int frame_number = 0;

int safe_write(const unsigned char *bytes, int num_bytes);
int send_SET();
int send_ACK();
int llopen_receiver();
int llopen_transmitter();
int send_data_frame(const unsigned char *buf, int buf_size);
int send_RR();
int send_REJ();

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    timeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;

    switch (connectionParameters.role)
    {
    case LlRx: // Receiver
        if (llopen_receiver() < 0)
            return -1;
        break;
    case LlTx: // Transmitter
        if (llopen_transmitter() < 0)
            return -1;
        break;
    default:
        return -1;
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    struct state_machine machine;
    create_state_machine(&machine, WRITE, (frame_number == 0 ? RR0 : RR1), REPLY_FROM_RECEIVER_ADDRESS, START);

    unsigned int attempt = 0;
    extern int alarm_enabled;
    extern int alarm_count;
    alarm_count = 0; // Reset alarm count

    (void)signal(SIGALRM, alarm_handler);

    while (attempt < nRetransmissions)
    {
        attempt++;
        // printf("Attempt #%d\n", attempt);

        if (send_data_frame(buf, bufSize) < 0) // Failed to send frame
        {
            // printf("Error sending frame %d. Retrying...\n", frame_number);
            continue; // Retry sending frame if it fails
        }

        alarm(timeout);
        alarm_enabled = TRUE;
        machine.state = START;

        while (alarm_enabled)
        {
            unsigned char byte = 0;
            int read_byte = readByteSerialPort(&byte);

            if (read_byte == 0)
            {
                continue; // 0 bytes read
            }
            else if (read_byte < 0)
            {
                printf("Read ERROR!");
                return -1;
            }

            // printf("State: %d, Received byte: 0x%02x\n", machine.state, byte);
            state_machine(&machine, byte);
            if (machine.state == STP && machine.REJ)
            {
                // printf("REJ%d was received!\n", frame_number);
                alarm(0);
                alarm_enabled = FALSE;
                attempt = 0;
                break; // Send frame again
            }
            else if (machine.state == STP)
            {
                // printf("RR%d was received!\n", frame_number);
                alarm(0);
                alarm_enabled = FALSE;
                frame_number = 1 - frame_number;
                return bufSize;
            }
        }
        if (!alarm_enabled && alarm_count > 0)
        {
            // printf("No RR received, retrying...\n");
        }
    }
    printf("Failed to send frame after %d attempts\n", nRetransmissions);
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    struct state_machine machine;
    create_state_machine(&machine, READ, (frame_number == 0 ? I_FRAME_0 : I_FRAME_1), TRANSMITTER_ADDRESS, START);
    int frames_received = 0;
    do
    {
        unsigned char byte = 0;
        int read_byte = readByteSerialPort(&byte);

        if (read_byte == 0)
        {
            continue; // 0 bytes read
        }
        else if (read_byte < 0)
        {
            printf("Read ERROR!");
            return -1;
        }

        // printf("State: %d, Received byte: 0x%02x\n", machine.state, byte);
        state_machine(&machine, byte);

        if (machine.state == STP && machine.ACK && frames_received == 0)
        {
            if (send_ACK() < 0)
                return -1;

            machine.state = START;
        }
        else if (machine.state == STP && machine.REJ)
        {
            if (send_REJ() < 0) // Failed to send REJ command
                return -1;

            machine.state = START; // Reset state for next frame
        }
        else if (machine.state == STP)
        {
            // printf("Frame %d received!\n", frame_number);
            frames_received++;
            break; // Frame received successfully
        }
    } while (machine.state != STP);

    if (send_RR() < 0) // Failed to send RR command
        return -1;

    memcpy(packet, machine.buf, machine.buf_size); // Copy received packet
    frame_number = 1 - frame_number;               // Switch frame number

    return machine.buf_size;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}

////////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////////

int safe_write(const unsigned char *bytes, int num_bytes)
{
    int total_bytes_written = 0;

    while (total_bytes_written < num_bytes)
    {
        int bytes_to_write = num_bytes - total_bytes_written;
        int bytes_written = writeBytesSerialPort(bytes + total_bytes_written, bytes_to_write);

        if (bytes_written < 0)
        {
            printf("ERROR writing to serial port!\n");
            return -1; // Indicate an error occurred
        }

        total_bytes_written += bytes_written;
    }

    return total_bytes_written;
}

int send_SET()
{
    unsigned char buf[5] = {FLAG, TRANSMITTER_ADDRESS, SET, 0, FLAG};
    buf[3] = buf[1] ^ buf[2]; // Calculate BCC1

    if (safe_write(buf, 5) < 0)
    {
        printf("Failed to send SET command.\n");
        return -1;
    }

    // printf("SET sent!\n");

    return 1;
}

int send_ACK()
{
    unsigned char buf[5] = {FLAG, REPLY_FROM_RECEIVER_ADDRESS, UA, 0, FLAG};
    buf[3] = buf[1] ^ buf[2]; // Calculate BCC1

    if (safe_write(buf, 5) < 0)
    {
        printf("Failed to send ACK command.\n");
        return -1;
    }

    // printf("ACK sent!\n");

    return 1;
}

int llopen_receiver()
{
    struct state_machine machine;
    create_state_machine(&machine, CONNECTION, SET, TRANSMITTER_ADDRESS, START);

    do
    {
        unsigned char byte = 0;
        int read_byte = readByteSerialPort(&byte);

        if (read_byte > 0)
        {
            // printf("State: %d, Received byte: 0x%02x\n", machine.state, byte);
            state_machine(&machine, byte);
        }
        else if (read_byte < 0)
        {
            printf("Read ERROR!");
            return -1;
        }
    } while (machine.state != STP);

    return send_ACK();
}

int llopen_transmitter()
{
    struct state_machine machine;
    create_state_machine(&machine, CONNECTION, UA, REPLY_FROM_RECEIVER_ADDRESS, START);

    unsigned int attempt = 0;
    extern int alarm_enabled;
    extern int alarm_count;
    alarm_count = 0; // Reset alarm count
    (void)signal(SIGALRM, alarm_handler);

    while (attempt < nRetransmissions)
    {
        attempt++;
        // printf("Attempt #%d\n", attempt);

        if (send_SET() < 0)
        {
            // printf("Error sending SET. Retrying...\n");
            continue; // Retry sending SET if it fails
        }

        alarm(timeout);
        alarm_enabled = TRUE;
        machine.state = START;

        while (alarm_enabled)
        {
            unsigned char byte = 0;
            int read_byte = readByteSerialPort(&byte);

            if (read_byte == 0)
            {
                continue; // 0 bytes read
            }
            else if (read_byte < 0)
            {
                printf("Read ERROR!");
                return -1;
            }

            // printf("State: %d, Received byte: 0x%02x\n", machine.state, byte);
            state_machine(&machine, byte);

            if (machine.state == STP)
            {
                // printf("ACK was received!\n");
                alarm(0);
                alarm_enabled = FALSE;
                return 1;
            }
        }
        if (!alarm_enabled && alarm_count > 0)
        {
            // printf("No ACK received, retrying...\n");
        }
    }
    // printf("Failed to establish connection after %d attempts\n", nRetransmissions);
    return -1;
}

int send_data_frame(const unsigned char *buf, int buf_size)
{
    unsigned char frame[(buf_size + 1) * 2 + 5]; // (data_size + BCC2) * 2 + (F; A; C; BCC1) + F
    int frame_size = 0;

    frame[frame_size++] = FLAG;
    frame[frame_size++] = TRANSMITTER_ADDRESS;
    frame[frame_size++] = (frame_number == 0) ? I_FRAME_0 : I_FRAME_1;
    frame[frame_size++] = frame[1] ^ frame[2]; // Calculate BCC1

    unsigned char BCC2 = 0; // Initialize BCC2
    for (int i = 0; i < buf_size; i++)
    {
        if (buf[i] == FLAG)
        {
            frame[frame_size++] = ESC;
            frame[frame_size++] = 0x5E; // Escape FLAG
        }
        else if (buf[i] == ESC)
        {
            frame[frame_size++] = ESC;
            frame[frame_size++] = 0x5D; // Escape ESC
        }
        else
        {
            frame[frame_size++] = buf[i];
        }
        BCC2 ^= buf[i]; // Compute BCC2
    }

    // Byte Stuff BCC2
    if (BCC2 == FLAG)
    {
        frame[frame_size++] = ESC;
        frame[frame_size++] = 0x5E; // Escape FLAG
    }
    else if (BCC2 == ESC)
    {
        frame[frame_size++] = ESC;
        frame[frame_size++] = 0x5D; // Escape ESC
    }
    else
    {
        frame[frame_size++] = BCC2;
    }

    frame[frame_size++] = FLAG;

    if (safe_write(frame, frame_size) < 0)
    {
        printf("Failed to send frame %d!\n", frame_number);
        return -1;
    }

    // printf("Frame %d sent!\n", frame_number);
    return 1;
}

int send_RR()
{
    unsigned char buf[5] = {FLAG, REPLY_FROM_RECEIVER_ADDRESS, frame_number == 0 ? RR0 : RR1, 0, FLAG};
    buf[3] = buf[1] ^ buf[2]; // Calculate BCC1

    if (safe_write(buf, 5) < 0)
    {
        printf("Failed to send RR%d command.\n", frame_number);
        return -1;
    }

    // printf("RR%d sent!\n", frame_number);

    return 1;
}

int send_REJ()
{
    unsigned char buf[5] = {FLAG, REPLY_FROM_RECEIVER_ADDRESS, frame_number == 0 ? REJ0 : REJ1, 0, FLAG};
    buf[3] = buf[1] ^ buf[2]; // Calculate BCC1

    if (safe_write(buf, 5) < 0)
    {
        printf("Failed to send REJ%d command.\n", frame_number);
        return -1;
    }

    // printf("REJ%d sent!\n", frame_number);

    return 1;
}
