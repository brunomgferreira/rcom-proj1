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

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int safe_write(const unsigned char *bytes, int num_bytes)
{
    int total_bytes_written = 0;

    while (total_bytes_written < num_bytes)
    {
        int bytesToWrite = num_bytes - total_bytes_written;
        int bytesWritten = writeBytesSerialPort(bytes + total_bytes_written, bytesToWrite);

        if (bytesWritten < 0)
        {
            printf("ERROR writing to serial port!\n");
            return -1; // Indicate an error occurred
        }

        total_bytes_written += bytesWritten;
    }

    return total_bytes_written;
}

int send_set()
{
    const int BUF_SIZE = 5;
    unsigned char buf[BUF_SIZE + 1];
    memset(buf, 0, sizeof(buf)); // Initialize to zero

    // ESTABLISH CONNECTION
    // SEND SET
    buf[0] = FLAG;
    buf[1] = SENDER_ADDRESS;
    buf[2] = SET;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;

    if (safe_write(buf, BUF_SIZE) < 0)
    {
        printf("Failed to send SET command.\n");
        return -1;
    }

    printf("SET sent!\n");

    return 1;
}

int send_ack()
{
    const int BUF_SIZE = 5;
    unsigned char buf[BUF_SIZE + 1];
    memset(buf, 0, sizeof(buf)); // Initialize to zero

    // ESTABLISH CONNECTION
    // SEND UA
    buf[0] = FLAG;
    buf[1] = RECEIVER_ADDRESS;
    buf[2] = UA;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;

    if (safe_write(buf, BUF_SIZE) < 0)
    {
        printf("Failed to send ACK command.\n");
        return -1;
    }

    printf("ACK sent!\n");

    return 1;
}

int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    timeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;

    const int BUF_SIZE = 5;
    unsigned char buf[BUF_SIZE + 1];
    memset(buf, 0, sizeof(buf)); // Initialize to zero

    if (connectionParameters.role == LlRx) // Reader
    {
        struct state_machine machine;
        create_state_machine(&machine, CONNECTION, SET, SENDER_ADDRESS, START);

        do
        {
            unsigned char byte = 0;
            int read_byte = readByteSerialPort(&byte);

            if (read_byte > 0)
            {
                printf("State: %d, Received byte: 0x%02x\n", machine.state, byte);
                state_machine(&machine, byte);
            }
            else if (read_byte < 0)
            {
                printf("Read ERROR!");
                return -1;
            }
        } while (machine.state != STP);

        if (send_ack() < 0) // Failed to send ACK command
            return -1;
    }
    else if (connectionParameters.role == LlTx) // Writer
    {
        struct state_machine machine;
        create_state_machine(&machine, CONNECTION, UA, RECEIVER_ADDRESS, START);

        unsigned int attempt = 0;
        extern int alarmEnabled;
        extern int alarmCount;
        alarmCount = 0; // Reset alarm count

        (void)signal(SIGALRM, alarmHandler);

        while (attempt < connectionParameters.nRetransmissions)
        {
            attempt++;
            printf("Attempt #%d\n", attempt);

            if (send_set() < 0)
            {
                printf("Error sending SET. Retrying...\n");
                continue; // Retry sending SET if it fails
            }

            alarm(connectionParameters.timeout);
            alarmEnabled = TRUE;

            machine.state = START;

            while (alarmEnabled)
            {
                unsigned char byte = 0;
                int read_byte = readByteSerialPort(&byte);

                if (read_byte > 0)
                {
                    printf("State: %d, Received byte: 0x%02x\n", machine.state, byte);
                    state_machine(&machine, byte);

                    if (machine.state == STP)
                    {
                        printf("ACK was received!\n");

                        alarm(0);
                        alarmEnabled = FALSE;

                        return 1;
                    }
                }
                else if (read_byte < 0)
                {
                    printf("Read ERROR!");
                    return -1;
                }
            }

            if (!alarmEnabled && alarmCount > 0)
            {
                printf("No ACK received, retrying...\n");
            }
        }

        printf("Failed to establish connection after %d attempts\n", connectionParameters.nRetransmissions);
        return -1;
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int frame_number = 0;

int send_frame(const unsigned char *buf, int bufSize)
{
    // Is this correct ????
    unsigned char frame[bufSize * 2 + 6]; // Extra space for stuffing 4 header bytes (F; A; C; BCC1) + 2 trailer bytes (BCC2 and F)
    int frame_size = 0;

    frame[frame_size++] = FLAG;
    frame[frame_size++] = SENDER_ADDRESS;
    frame[frame_size++] = (frame_number == 0) ? I_FRAME_0 : I_FRAME_1;
    frame[frame_size++] = frame[1] ^ frame[2];

    unsigned char BCC2 = buf[0];

    switch (buf[0])
    {
    case FLAG:
        frame[frame_size++] = ESC;
        frame[frame_size++] = 0x5E;
        break;
    case ESC:
        frame[frame_size++] = ESC;
        frame[frame_size++] = 0x5D;
        break;
    default:
        frame[frame_size++] = buf[0];
        break;
    }

    for (int i = 1; i < bufSize; i++)
    {
        switch (buf[i])
        {
        case FLAG:
            frame[frame_size++] = ESC;
            frame[frame_size++] = 0x5E;
            break;
        case ESC:
            frame[frame_size++] = ESC;
            frame[frame_size++] = 0x5D;
            break;
        default:
            frame[frame_size++] = buf[i];
            break;
        }

        BCC2 ^= buf[i];
    }

    switch (BCC2)
    {
    case FLAG:
        frame[frame_size++] = ESC;
        frame[frame_size++] = 0x5E;
        break;
    case ESC:
        frame[frame_size++] = ESC;
        frame[frame_size++] = 0x5D;
        break;
    default:
        frame[frame_size++] = BCC2;
        break;
    }

    frame[frame_size++] = FLAG;

    // printf("BCC2: 0x%02x\n", BCC2);

    if (safe_write(frame, frame_size) < 0)
    {
        printf("Failed to send frame %d!\n", frame_number);
        return -1;
    }

    printf("Frame %d sent!\n", frame_number);

    return 1;
}

int llwrite(const unsigned char *buf, int bufSize)
{
    struct state_machine machine;
    create_state_machine(&machine, WRITE, (frame_number == 0 ? RR0 : RR1), RECEIVER_ADDRESS, START);

    unsigned int attempt = 0;
    extern int alarmEnabled;
    extern int alarmCount;
    alarmCount = 0; // Reset alarm count

    (void)signal(SIGALRM, alarmHandler);

    while (attempt < nRetransmissions)
    {
        attempt++;
        printf("Attempt #%d\n", attempt);

        if (send_frame(buf, bufSize) < 0) // Failed to send frame
        {
            printf("Error sending frame %d. Retrying...\n", frame_number);
            continue; // Retry sending frame if it fails
        }

        alarm(timeout);
        alarmEnabled = TRUE;

        machine.state = START;

        while (alarmEnabled)
        {
            unsigned char byte = 0;
            int read_byte = readByteSerialPort(&byte);

            if (read_byte > 0)
            {
                // printf("State: %d, Received byte: 0x%02x - ", machine.state, byte);
                int status = state_machine(&machine, byte);
                // printf("State: %d, Status: %d\n", machine.state, status);

                if (machine.state == STP && status > 0)
                {
                    printf("RR%d was received!\n", frame_number);

                    alarm(0);
                    alarmEnabled = FALSE;

                    frame_number = 1 - frame_number;

                    return bufSize;
                }
                else if (machine.state == STP && status == 0)
                {
                    printf("REJ%d was received!\n", frame_number);

                    alarm(0);
                    alarmEnabled = FALSE;

                    attempt = 1; // TODO Is this correct ????

                    break; // Send frame again
                }
            }
            else if (read_byte < 0)
            {
                printf("Read ERROR!");
                return -1;
            }
        }

        if (!alarmEnabled && alarmCount > 0)
        {
            printf("No ACK received, retrying...\n");
        }
    }

    printf("Failed to send frame after %d attempts\n", nRetransmissions);
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int send_RR()
{
    const int BUF_SIZE = 5;
    unsigned char buf[BUF_SIZE + 1];
    memset(buf, 0, sizeof(buf)); // Initialize to zero

    buf[0] = FLAG;
    buf[1] = RECEIVER_ADDRESS;
    buf[2] = frame_number == 0 ? RR0 : RR1;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;

    if (safe_write(buf, BUF_SIZE) < 0)
    {
        printf("Failed to send RR%d command.\n", frame_number);
        return -1;
    }

    printf("RR%d sent!\n", frame_number);

    return 1;
}

int send_REJ()
{
    const int BUF_SIZE = 5;
    unsigned char buf[BUF_SIZE + 1];
    memset(buf, 0, sizeof(buf)); // Initialize to zero

    buf[0] = FLAG;
    buf[1] = RECEIVER_ADDRESS;
    buf[2] = frame_number == 0 ? REJ0 : REJ1;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;

    if (safe_write(buf, BUF_SIZE) < 0)
    {
        printf("Failed to send REJ%d command.\n", frame_number);
        return -1;
    }

    printf("REJ%d sent!\n", frame_number);

    return 1;
}

int llread(unsigned char *packet)
{
    struct state_machine machine;
    create_state_machine(&machine, READ, (frame_number == 0 ? I_FRAME_0 : I_FRAME_1), SENDER_ADDRESS, START);

    do
    {
        unsigned char byte = 0;
        int read_byte = readByteSerialPort(&byte);

        if (read_byte > 0)
        {
            // printf("State: %d, Received byte: 0x%02x - ", machine.state, byte);
            int status = state_machine(&machine, byte);
            // printf("State: %d, Status %d\n", machine.state, status);

            if (status == 0)
            {
                if (send_REJ() < 0) // Failed to send REJ command
                    return -1;

                machine.state = START;
            }
            else if (machine.state == STP && status > 0)
            {
                printf("Frame %d received!\n", frame_number);
                break;
            }
        }
        else if (read_byte < 0)
        {
            printf("Read ERROR!");
            return -1;
        }

    } while (machine.state != STP);

    if (send_RR() < 0) // Failed to send RR command
        return -1;

    /*
    printf("%d", machine.buf_size);
    for (int i = 0; i < machine.buf_size; i++)
        printf("%c", machine.buf[i]);
    printf("\n");
    */

    memcpy(packet, machine.buf, machine.buf_size);

    frame_number = 1 - frame_number;

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
