// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>

#define START 1
#define DATA 2
#define END 3
#define FILE_SIZE 0
#define FILE_NAME 1

int send_control_packet(int control_byte, const char *filename, int file_size)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];
    int packet_size = 0;

    packet[packet_size++] = control_byte; // Control field

    // file_size
    packet[packet_size++] = FILE_SIZE;                           // T
    packet[packet_size++] = sizeof(file_size);                   // L (Length of V)
    memcpy(&packet[packet_size], &file_size, sizeof(file_size)); // V
    packet_size += sizeof(file_size);

    // filename
    packet[packet_size++] = FILE_NAME;        // T
    packet[packet_size++] = strlen(filename); // L (Length of V)
    // printf("Filename: %s\n", filename);
    memcpy(&packet[packet_size], filename, strlen(filename)); // V
    packet_size += strlen(filename);

    int bytes_written = llwrite(packet, packet_size);
    if (bytes_written < 0)
    {
        printf("Failed to send control packet.\n");
        return -1;
    }

    printf("Sent %d bytes.\n", bytes_written);
    return 1;
}

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

        unsigned char control_packet[MAX_PAYLOAD_SIZE];
        int bytes_read = llread(control_packet); // Read the start control packet

        if (bytes_read < 0)
        {
            printf("Failed to read start control packet.\n");
            llclose(0);
            return;
        }

        // Process the start control packet
        if (control_packet[0] == START)
        {
            // Extract file size and filename from the control packet
            int file_size = 0;
            char received_filename[100]; // Adjust size as needed
            int index = 1;

            while (index < bytes_read)
            {
                unsigned char type = control_packet[index++];
                unsigned char length = control_packet[index++];
                if (type == FILE_SIZE)
                {
                    memcpy(&file_size, &control_packet[index], length);
                    index += length;
                }
                else if (type == FILE_NAME)
                {
                    memcpy(received_filename, &control_packet[index], length);
                    received_filename[length] = '\0'; // Null-terminate the string
                    index += length;
                }
            }

            printf("Receiving file: %s of size %d bytes\n", received_filename, file_size);

            // Open file to write received data
            FILE *output_file = fopen(filename, "wb");
            if (!output_file)
            {
                printf("Error opening file for writing: %s\n", filename);
                return;
            }

            // Read data packets
            unsigned char data_packet[MAX_PAYLOAD_SIZE];
            int total_received = 0;

            while (1)
            {
                int bytes_read = llread(data_packet); // Read data packet
                if (bytes_read < 0)
                {
                    printf("Failed to read data packet.\n");
                    break;
                }

                // Check if it's an end packet
                if (data_packet[0] == END)
                {
                    printf("Received end packet.\n");
                    break;
                }
                else if (data_packet[0] == DATA)
                {
                    int sequence_number = data_packet[1]; // Is sequence_number verification needed ??????????
                    int L2 = data_packet[2];
                    int L1 = data_packet[3];
                    int K = 256 * L2 + L1; // Number of bytes in data field

                    if (K + 4 > bytes_read)
                    {
                        // If link_layer is correct, this will never happen
                        printf("Packet size mismatch, expected %d bytes but got %d.\n", K + 4, bytes_read);
                    }

                    // Write data to file
                    fwrite(&data_packet[4], 1, K, output_file); // Skip the header
                    total_received += K;
                    printf("Received data packet %d, total received: %d bytes\n", sequence_number, total_received);
                }
            }

            fclose(output_file);
            printf("File received successfully, total bytes: %d\n", total_received);
        }
        else
        {
            printf("Unexpected packet received: %d\n", control_packet[0]);
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

        FILE *file = fopen(filename, "rb");
        if (!file)
        {
            printf("Error opening file: %s\n", filename);
            // return -1;
            llclose(0);
            return;
        }

        fseek(file, 0, SEEK_END);
        int file_size = ftell(file);
        fseek(file, 0, SEEK_SET);

        send_control_packet(START, filename, file_size);

        unsigned char packet[MAX_PAYLOAD_SIZE];
        int bytes_read;
        int sequence_number = 0;

        while ((bytes_read = fread(packet, 1, MAX_PAYLOAD_SIZE - 4, file)) > 0)
        {
            unsigned char data_packet[MAX_PAYLOAD_SIZE];
            int packet_size = 0;

            data_packet[packet_size++] = DATA;
            data_packet[packet_size++] = sequence_number++; // IS IT ONLY 0-99 ????

            // K = 256 * L2 + L1
            int L1 = bytes_read % 256;
            int L2 = bytes_read / 256;

            data_packet[packet_size++] = L2;
            data_packet[packet_size++] = L1;

            memcpy(&data_packet[packet_size], packet, bytes_read);
            packet_size += bytes_read;

            int bytes_written = llwrite(data_packet, packet_size);
            if (bytes_written < 0)
            {
                printf("Failed to send data packet.\n");
                break;
            }

            printf("Sent %d bytes.\n", bytes_written);
        }

        send_control_packet(END, filename, file_size);

        printf("File sent successfully, total bytes: %d\n", file_size);

        fclose(file);
    }

    llclose(0);
}
