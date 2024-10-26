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
#define MAX_SEQUENCE_NUMBER 99
#define MAX_PACKETS (MAX_SEQUENCE_NUMBER + 1)

typedef struct
{
    int sequence_number;
    unsigned char data[MAX_PAYLOAD_SIZE - 4];
    int data_size;
} Packet;

void initialize_packet_buffer(Packet *packet_buffer);
void process_packet(Packet *packet, FILE *output_file);
int send_control_packet(int control_byte, const char *filename, int file_size);
int handle_receiver(const char *filename);
int handle_transmitter(const char *filename);
int read_control_packet(int *file_size, unsigned char *received_filename);
int read_data_packets(FILE *output_file, const unsigned char *received_filename, int file_size);
int send_data_packets(FILE *file, const char *filename, int file_size);
int check_end_packet(unsigned char *control_packet, int data_size, const unsigned char *filename, int file_size);

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connection_parameters;

    // Initialize connection parameters
    strcpy(connection_parameters.serialPort, serialPort);
    connection_parameters.baudRate = baudRate;
    connection_parameters.nRetransmissions = nTries;
    connection_parameters.timeout = timeout;

    // Set role
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

    // Establishment
    if (llopen(connection_parameters) < 0)
    {
        printf("Failed to open connection.\n");
        return;
    }

    // Data Transfer
    if (connection_parameters.role == LlRx) // Receiver
    {
        if (handle_receiver(filename) < 0)
        {
            printf("An error occurred during data transfer.\n");
        }
    }
    else if (connection_parameters.role == LlTx) // Transmitter
    {
        if (handle_transmitter(filename) < 0)
        {
            printf("An error occurred during data transfer.\n");
        }
    }

    // Termination
    if (llclose(1) < 0)
    {
        printf("Failed to close connection.\n");
    }
}

////////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////////

void initialize_packet_buffer(Packet *packet_buffer)
{
    for (int i = 0; i < MAX_PACKETS; i++)
    {
        packet_buffer[i].sequence_number = -1; // Mark as empty
        packet_buffer[i].data_size = 0;
    }
}

void process_packet(Packet *packet, FILE *output_file)
{
    fwrite(packet->data, 1, packet->data_size, output_file);
    // printf("Processed data packet %d\n", packet->sequence_number);
}

int send_control_packet(int control_byte, const char *filename, int file_size)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];
    int packet_size = 0;

    packet[packet_size++] = control_byte; // Control field

    // File size
    packet[packet_size++] = FILE_SIZE;                                            // T
    packet[packet_size++] = (unsigned char)sizeof(file_size);                     // L (Length of V)
    memcpy(&packet[packet_size], (unsigned char *)&file_size, sizeof(file_size)); // V
    packet_size += sizeof(file_size);

    // File name
    packet[packet_size++] = FILE_NAME;                        // T
    packet[packet_size++] = (unsigned char)strlen(filename);  // L (Length of V)
    memcpy(&packet[packet_size], filename, strlen(filename)); // V
    packet_size += strlen(filename);

    int bytes_written = llwrite(packet, packet_size);
    if (bytes_written < 0)
    {
        printf("Failed to send control packet.\n");
        return -1;
    }

    // printf("Sent %d bytes.\n", bytes_written);
    return 1;
}

int read_control_packet(int *file_size, unsigned char *received_filename)
{
    unsigned char control_packet[MAX_PAYLOAD_SIZE];
    int bytes_read = llread(control_packet); // Read the start control packet

    if (bytes_read < 0)
    {
        printf("Failed to read start control packet.\n");
        return -1;
    }

    // Process the start control packet
    if (control_packet[0] == START)
    {
        int index = 1;

        // Extract file size and filename from the control packet
        while (index < bytes_read)
        {
            unsigned char type = control_packet[index++];
            unsigned char length = control_packet[index++];
            if (type == FILE_SIZE)
            {
                memcpy(file_size, &control_packet[index], length);
                index += length;
            }
            else if (type == FILE_NAME)
            {
                memcpy(received_filename, &control_packet[index], length);
                received_filename[length] = '\0'; // Null-terminate
                index += length;
            }
        }

        // printf("Receiving file: %s of size %d bytes\n", received_filename, *file_size);
        return 1;
    }

    printf("Unexpected control packet received: %d\n", control_packet[0]);
    return -1;
}

int read_data_packets(FILE *output_file, const unsigned char *received_filename, int file_size)
{
    // Buffer for out-of-sequence packets
    Packet packet_buffer[MAX_PACKETS];
    initialize_packet_buffer(packet_buffer);

    unsigned char data_packet[MAX_PAYLOAD_SIZE];
    int expected_sequence_number = 0;

    while (1)
    {
        int bytes_read = llread(data_packet); // Read data packet

        if (bytes_read < 0)
        {
            printf("Failed to read data packet.\n");
            return -1;
        }

        // Check for end packet
        if (data_packet[0] == END)
        {
            // TODO Check this
            // printf("Received end packet.\n");
            if (check_end_packet(data_packet, bytes_read, received_filename, file_size) < 0)
            {
                printf("Start and End control packet mismatch!\n");
            }
            return 1;
        }

        if (data_packet[0] == DATA)
        {
            Packet packet;
            packet.sequence_number = data_packet[1];
            packet.data_size = 256 * (int)data_packet[2] + (int)data_packet[3]; // K = 256 * L2 + L1
            memcpy(packet.data, &data_packet[4], packet.data_size);

            if (packet.data_size + 4 != bytes_read)
            {
                // If link_layer is correct, this will never happen
                printf("Packet size mismatch, expected %d bytes but got %d.\n", packet.data_size + 4, bytes_read);
                packet.data_size = bytes_read - 4;
            }

            if (packet.sequence_number == expected_sequence_number)
            {
                process_packet(&packet, output_file);
                expected_sequence_number = (expected_sequence_number + 1) % MAX_PACKETS;

                // Process any buffered packets that are now in order
                while (packet_buffer[expected_sequence_number].sequence_number != -1)
                {
                    process_packet(&packet_buffer[expected_sequence_number], output_file);

                    // Clear the processed packet
                    packet_buffer[expected_sequence_number].sequence_number = -1;
                    packet_buffer[expected_sequence_number].data_size = 0;

                    expected_sequence_number = (expected_sequence_number + 1) % MAX_PACKETS;
                }
            }
            else if (packet_buffer[packet.sequence_number].sequence_number == -1)
            {
                // Store out-of-sequence packets in the buffer
                packet_buffer[packet.sequence_number] = packet;
                // printf("Buffered out-of-sequence packet %d\n", packet.sequence_number);
            }
        }
    }

    return 1;
}

int handle_receiver(const char *filename)
{
    int file_size = 0;
    unsigned char received_filename[MAX_PAYLOAD_SIZE];

    // Read control packet
    if (read_control_packet(&file_size, received_filename) < 0)
    {
        return -1;
    }

    // Open file for writing
    FILE *output_file = fopen(filename, "wb");
    if (!output_file)
    {
        printf("Error opening file for writing: %s\n", filename);
        return -1;
    }

    // Read and process data packets
    if (read_data_packets(output_file, received_filename, file_size) < 0)
    {
        fclose(output_file);
        return -1;
    }

    fclose(output_file);
    // printf("File received successfully!\n");
    return 1;
}

int send_data_packets(FILE *file, const char *filename, int file_size)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];
    int bytes_read;
    int sequence_number = 0;

    while ((bytes_read = fread(packet, 1, MAX_PAYLOAD_SIZE - 4, file)) > 0)
    {
        unsigned char data_packet[MAX_PAYLOAD_SIZE];
        int packet_size = 0;

        // Construct data packet
        data_packet[packet_size++] = DATA;
        data_packet[packet_size++] = sequence_number;

        // K = 256 * L2 + L1
        int L1 = bytes_read % 256;
        int L2 = bytes_read / 256;

        data_packet[packet_size++] = (unsigned char)L2;
        data_packet[packet_size++] = (unsigned char)L1;

        memcpy(&data_packet[packet_size], packet, bytes_read);
        packet_size += bytes_read;

        if (llwrite(data_packet, packet_size) < 0)
        {
            printf("Failed to send data packet.\n");
            return -1;
        }

        // printf("Sent data packet %d, size %d bytes.\n", sequence_number, packet_size);
        sequence_number = (sequence_number + 1) % (MAX_SEQUENCE_NUMBER + 1);
    }

    return 1;
}

int handle_transmitter(const char *filename)
{
    FILE *file = fopen(filename, "rb");
    if (!file)
    {
        printf("Error opening file: %s\n", filename);
        return -1;
    }

    fseek(file, 0, SEEK_END);
    int file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (send_control_packet(START, filename, file_size) < 0)
    {
        fclose(file);
        return -1;
    }

    if (send_data_packets(file, filename, file_size) < 0)
    {
        fclose(file);
        return -1;
    }

    if (send_control_packet(END, filename, file_size) < 0)
    {
        fclose(file);
        return -1;
    };

    // printf("File sent successfully, total bytes: %d\n", file_size);
    fclose(file);
    return 1;
}

int check_end_packet(unsigned char *control_packet, int data_size, const unsigned char *filename, int file_size)
{
    int index = 1;

    int received_file_size = 0;
    unsigned char received_filename[MAX_PAYLOAD_SIZE];

    int end_filename_sent = 0;

    // Extract file size and filename from the control packet
    while (index < data_size)
    {
        unsigned char type = control_packet[index++];
        unsigned char length = control_packet[index++];
        if (type == FILE_SIZE)
        {
            memcpy(&received_file_size, &control_packet[index], length);
            index += length;

            if (received_file_size != file_size)
            {
                // printf("End packet file size does not match start packet file size!\n");
                return -1;
            }
        }
        else if (type == FILE_NAME)
        {
            memcpy(received_filename, &control_packet[index], length);
            received_filename[length] = '\0'; // Null-terminate
            index += length;
            end_filename_sent = 1;

            if (strcmp((const char *)received_filename, (const char *)filename) != 0)
            {
                // printf("End packet filename does not match start packet filename!\n");
                return -1;
            }
        }
    }

    return (strlen((const char *)filename) > 0 && end_filename_sent) ? 1 : -1;
}
