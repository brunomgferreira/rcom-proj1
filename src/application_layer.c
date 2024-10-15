// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>

#define MAX_PACKET_SIZE 256 // Make this dynamic later using the baudRate

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connection_parameters;
    strcpy(connection_parameters.serialPort, serialPort);
    connection_parameters.baudRate = baudRate;
    connection_parameters.nRetransmissions = nTries;
    connection_parameters.timeout = timeout;

    if (strcmp(role, "rx") == 0)
    {
        connection_parameters.role = LlRx;
    }
    else if (strcmp(role, "tx") == 0)
    {
        connection_parameters.role = LlTx;
    }
    else
    {
        printf("Invalid role: %s. Use 'rx' or 'tx'.\n", role);
        return;
    }

    // =====================================================
    // ==================== CONNECTION =====================

    if (llopen(connection_parameters) < 0)
    {
        printf("Failed to open connection.\n");
        return;
    }

    // =====================================================
    // ==================== READ/WRITE =====================

    if (connection_parameters.role == LlRx) // Reader
    {
        // SEE SLIDES 26-27 !!!!
        // READ **START** CONTROL PACKET
        // SAVE FILE TYPE, LENGTH, VALUE
        // LOOP READ PACKETS USING llread() and
        // JOIN PACKETS TOGETHER (REMOVING THE PACKET HEADER)
        // keep doing this until
        // READ **END** CONTROL PACKET

        unsigned char packet[MAX_PACKET_SIZE]; // Define an appropriate size for the packet
        int bytesRead = llread(packet);        // Call llread correctly
        if (bytesRead < 0)
        {
            printf("Failed to read data.\n");
        }
        else
        {
            // Process the received packet (e.g., save to a file)
            printf("Received %d bytes.\n", bytesRead);
        }
    }
    else if (connection_parameters.role == LlTx) // Transmitter
    {
        // SEE SLIDES 26-27 !!!!
        // SEND **START** CONTROL PACKET
        // DIVIDE FILE INTO N PACKETS OF X SIZE
        // ADD APPLICATION HEADER TO PACKETS
        // WHILE FILE IS NOT FULLY SENT LOOP llwrite();
        // SEND **END** CONTROL PACKET

        unsigned char buf[MAX_PACKET_SIZE]; // Buffer for sending data
        /*
        int bufSize = readFileIntoBuffer(filename, buf); // Read file into buffer

        if (bufSize < 0)
        {
            printf("Failed to read the file: %s.\n", filename);
        }
        else
        {
            int bytesWritten = llwrite(buf, bufSize); // Call llwrite correctly
            if (bytesWritten < 0)
            {
                printf("Failed to write data.\n");
            }
            else
            {
                printf("Sent %d bytes.\n", bytesWritten);
            }
        }
        */

        for (int i = 0; i < MAX_PACKET_SIZE; i++)
        {
            buf[i] = 'A' + (i % 26); // ASCII values of 'A' to 'Z' are 65 to 90
        }

        int bytesWritten = llwrite(buf, MAX_PACKET_SIZE); // Call llwrite correctly
        if (bytesWritten < 0)
        {
            printf("Failed to write data.\n");
        }
        else
        {
            printf("Sent %d bytes.\n", bytesWritten);
        }
    }

    llclose(0);
}
