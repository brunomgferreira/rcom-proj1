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

    const int BUF_SIZE = 5;
    unsigned char buf[BUF_SIZE + 1];
    memset(buf, 0, sizeof(buf)); // Initialize to zero

    if (connectionParameters.role == LlRx) // Reader
    {
        struct state_machine machine;
        create_state_machine(&machine, SET, SENDER_ADDRESS, START);

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
        create_state_machine(&machine, UA, RECEIVER_ADDRESS, START);

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
                        // alarmEnabled = FALSE;

                        return 0;
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
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
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
