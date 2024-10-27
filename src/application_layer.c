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

// Structure for a data packet
typedef struct
{
    int sequence_number;                      // Sequence number of the packet
    unsigned char data[MAX_PAYLOAD_SIZE - 4]; // Data payload
    int data_size;                            // Size of the data
} Packet;

// Function declarations
void initialize_packet_buffer(Packet *packet_buffer);
void process_packet(Packet *packet, FILE *output_file);
int send_control_packet(int control_byte, const char *filename, int file_size);
int handle_receiver(const char *filename);
int handle_transmitter(const char *filename);
int read_control_packet(int *file_size, unsigned char *received_filename);
int read_data_packets(FILE *output_file, const unsigned char *received_filename, int file_size);
int send_data_packets(FILE *file, const char *filename, int file_size);
int check_end_packet(unsigned char *control_packet, int data_size, const unsigned char *filename, int file_size);

// Main application layer function
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connection_parameters;

    // Initialize connection parameters
    strcpy(connection_parameters.serialPort, serialPort);
    connection_parameters.baudRate = baudRate;
    connection_parameters.nRetransmissions = nTries;
    connection_parameters.timeout = timeout;

    // Set the communication role (receiver or transmitter)
    if (strcmp(role, "rx") == 0)
    {
        connection_parameters.role = LlRx; // Receiver role
    }
    else if (strcmp(role, "tx") == 0)
    {
        connection_parameters.role = LlTx; // Transmitter role
    }
    else
    {
        printf("Invalid role: %s. Use 'rx' or 'tx'.\n", role);
        return;
    }

    // Establish a connection
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

// Initialize packet buffer to mark packets as empty
void initialize_packet_buffer(Packet *packet_buffer)
{
    for (int i = 0; i < MAX_PACKETS; i++)
    {
        packet_buffer[i].sequence_number = -1; // Mark as empty
        packet_buffer[i].data_size = 0;
    }
}

// Process and write the received packet to the output file
void process_packet(Packet *packet, FILE *output_file)
{
    fwrite(packet->data, 1, packet->data_size, output_file); // Write data to file
}

// Send a control packet with file information
int send_control_packet(int control_byte, const char *filename, int file_size)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];
    int packet_size = 0;

    packet[packet_size++] = control_byte; // Control field

    // Include file size in the packet
    packet[packet_size++] = FILE_SIZE;                                            // T
    packet[packet_size++] = (unsigned char)sizeof(file_size);                     // L (Length of V)
    memcpy(&packet[packet_size], (unsigned char *)&file_size, sizeof(file_size)); // V
    packet_size += sizeof(file_size);

    // Include file name in the packet
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

    return 1; // Successfully sent control packet
}

// Read a control packet and extract file information
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
                memcpy(file_size, &control_packet[index], length); // Get file size
                index += length;
            }
            else if (type == FILE_NAME)
            {
                memcpy(received_filename, &control_packet[index], length); // Get filename
                received_filename[length] = '\0';                          // Null-terminate the string
                index += length;
            }
        }

        return 1; // Successfully read control packet
    }

    printf("Unexpected control packet received: %d\n", control_packet[0]);
    return -1; // Unexpected control packet received
}

// Read incoming data packets and write them to the output file
int read_data_packets(FILE *output_file, const unsigned char *received_filename, int file_size)
{
    // Buffer for out-of-sequence packets
    Packet packet_buffer[MAX_PACKETS];
    initialize_packet_buffer(packet_buffer); // Initialize packet buffer

    unsigned char data_packet[MAX_PAYLOAD_SIZE];
    int expected_sequence_number = 0; // Initialize expected sequence number

    while (1)
    {
        int bytes_read = llread(data_packet); // Read data packet

        if (bytes_read < 0)
        {
            printf("Failed to read data packet.\n");
            return -1; // Error reading data packet
        }

        // Check for end packet
        if (data_packet[0] == END)
        {
            if (check_end_packet(data_packet, bytes_read, received_filename, file_size) < 0)
            {
                printf("Start and End control packet mismatch!\n");
            }
            return 1; // End packet received
        }

        if (data_packet[0] == DATA)
        {
            Packet packet;
            packet.sequence_number = data_packet[1];                            // Get sequence number
            packet.data_size = 256 * (int)data_packet[2] + (int)data_packet[3]; // K = 256 * L2 + L1
            memcpy(packet.data, &data_packet[4], packet.data_size);             // Copy data into packet

            if (packet.data_size + 4 != bytes_read)
            {
                // If link_layer is correct, this will never happen
                printf("Packet size mismatch, expected %d bytes but got %d.\n", packet.data_size + 4, bytes_read);
                packet.data_size = bytes_read - 4; // Adjust data size if mismatch
            }

            if (packet.sequence_number == expected_sequence_number)
            {
                process_packet(&packet, output_file);                                    // Process the correctly sequenced packet
                expected_sequence_number = (expected_sequence_number + 1) % MAX_PACKETS; // Update expected sequence number

                // Process any buffered packets that are now in order
                while (packet_buffer[expected_sequence_number].sequence_number != -1)
                {
                    process_packet(&packet_buffer[expected_sequence_number], output_file); // Process buffered packet

                    // Clear the processed packet
                    packet_buffer[expected_sequence_number].sequence_number = -1; // Mark as empty
                    packet_buffer[expected_sequence_number].data_size = 0;

                    expected_sequence_number = (expected_sequence_number + 1) % MAX_PACKETS; // Update expected number
                }
            }
            else if (packet_buffer[packet.sequence_number].sequence_number == -1)
            {
                // Store out-of-sequence packets in the buffer
                packet_buffer[packet.sequence_number] = packet; // Buffer the packet
            }
        }
    }

    return 1; // Successfully processed all packets
}

// Handle the receiver role and manage the file reception
int handle_receiver(const char *filename)
{
    int file_size = 0;
    unsigned char received_filename[MAX_PAYLOAD_SIZE];

    // Read the control packet to get file info
    if (read_control_packet(&file_size, received_filename) < 0)
    {
        return -1; // Error reading control packet
    }

    // Open file for writing
    FILE *output_file = fopen(filename, "wb");
    if (!output_file)
    {
        printf("Error opening file for writing: %s\n", filename);
        return -1; // Error opening output file
    }

    // Read and process data packets
    if (read_data_packets(output_file, received_filename, file_size) < 0)
    {
        fclose(output_file);
        return -1; // Error processing data packets
    }

    fclose(output_file); // Close output file
    return 1;            // Successfully received file
}

// Send data packets from the file
int send_data_packets(FILE *file, const char *filename, int file_size)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];
    int bytes_read;
    int sequence_number = 0; // Initialize sequence number

    while ((bytes_read = fread(packet, 1, MAX_PAYLOAD_SIZE - 4, file)) > 0)
    {
        unsigned char data_packet[MAX_PAYLOAD_SIZE];
        int packet_size = 0;

        // Construct data packet
        data_packet[packet_size++] = DATA;            // Packet type
        data_packet[packet_size++] = sequence_number; // Add sequence number

        // K = 256 * L2 + L1
        int L1 = bytes_read % 256; // Low byte
        int L2 = bytes_read / 256; // High byte

        data_packet[packet_size++] = (unsigned char)L2; // Add high byte
        data_packet[packet_size++] = (unsigned char)L1; // Add low byte

        memcpy(&data_packet[packet_size], packet, bytes_read); // Add data to packet
        packet_size += bytes_read;

        if (llwrite(data_packet, packet_size) < 0)
        {
            printf("Failed to send data packet.\n");
            return -1; // Error sending data packet
        }

        sequence_number = (sequence_number + 1) % (MAX_SEQUENCE_NUMBER + 1); // Update sequence number
    }

    return 1; // Successfully sent all data packets
}

// Handle the transmitter role and manage the file transmission
int handle_transmitter(const char *filename)
{
    FILE *file = fopen(filename, "rb"); // Open file for reading
    if (!file)
    {
        printf("Error opening file: %s\n", filename);
        return -1; // Error opening input file
    }

    fseek(file, 0, SEEK_END);    // Move to the end of the file
    int file_size = ftell(file); // Get the file size
    fseek(file, 0, SEEK_SET);    // Move back to the start of the file

    // Send start control packet with file info
    if (send_control_packet(START, filename, file_size) < 0)
    {
        fclose(file);
        return -1; // Error sending start control packet
    }

    // Send data packets
    if (send_data_packets(file, filename, file_size) < 0)
    {
        fclose(file);
        return -1; // Error sending data packets
    }

    // Send end control packet
    if (send_control_packet(END, filename, file_size) < 0)
    {
        fclose(file);
        return -1; // Error sending end control packet
    };

    fclose(file); // Close input file
    return 1;     // Successfully sent file
}

// Check the end packet for consistency with the start packet
int check_end_packet(unsigned char *control_packet, int data_size, const unsigned char *filename, int file_size)
{
    int index = 1; // Start from first data after control byte

    int received_file_size = 0; // To store received file size
    unsigned char received_filename[MAX_PAYLOAD_SIZE];

    int end_filename_sent = 0; // Flag to check if end filename was sent

    // Extract file size and filename from the control packet
    while (index < data_size)
    {
        unsigned char type = control_packet[index++];
        unsigned char length = control_packet[index++];
        if (type == FILE_SIZE)
        {
            memcpy(&received_file_size, &control_packet[index], length); // Get received file size
            index += length;

            if (received_file_size != file_size)
            {
                return -1; // File size mismatch
            }
        }
        else if (type == FILE_NAME)
        {
            memcpy(received_filename, &control_packet[index], length); // Get received filename
            received_filename[length] = '\0';                          // Null-terminate
            index += length;
            end_filename_sent = 1; // Mark filename sent

            if (strcmp((const char *)received_filename, (const char *)filename) != 0)
            {
                return -1; // Filename mismatch
            }
        }
    }

    return (strlen((const char *)filename) > 0 && end_filename_sent) ? 1 : -1;
}
