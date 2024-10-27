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

// Global variables
LinkLayer connection_parameters; // Connection parameters for link layer
int frame_number = 0;            // Current frame number (0 or 1)
int frames_received = 0;         // Count of frames received

// Statistics structure
extern struct ll_statistics statistics;

// Helper Functions prototypes
int safe_write(const unsigned char *bytes, int num_bytes);
int send_SET();
int send_ACK();
int llopen_receiver();
int llopen_transmitter();
int send_data_frame(const unsigned char *buf, int buf_size);
int send_RR();
int send_REJ();
int send_DISC();
int llclose_receiver();
int llclose_transmitter();
void show_statistics(struct ll_statistics statistics);

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // Open the serial port with specified parameters
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1; // Error opening serial port
    }

    connection_parameters = connectionParameters; // Store connection parameters

    // Handle connection based on role (Receiver or Transmitter)
    switch (connectionParameters.role)
    {
    case LlRx: // Receiver
        if (llopen_receiver() < 0)
            return -1; // Error during receiver connection
        break;
    case LlTx: // Transmitter
        if (llopen_transmitter() < 0)
            return -1; // Error during transmitter connection
        break;
    default:
        return -1; // Invalid role
    }

    return 1; // Connection successful
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    struct state_machine machine;
    // Create a type WRITE state machine
    create_state_machine(&machine, WRITE, (frame_number == 0 ? RR1 : RR0), REPLY_FROM_RECEIVER_ADDRESS, START);

    unsigned int attempt = 0;
    extern int alarm_enabled;
    extern int alarm_count;
    alarm_enabled = FALSE; // Disable alarm initially
    alarm_count = 0;       // Reset alarm count

    (void)signal(SIGALRM, alarm_handler); // Set signal handler for alarm

    // Retry sending data frame based on the number of retransmissions
    while (attempt < connection_parameters.nRetransmissions)
    {
        attempt++;

        // Attempt to send the data frame
        if (send_data_frame(buf, bufSize) < 0) // Failed to send frame
        {
            alarm(0);                         // Disable alarm on failure
            statistics.num_retransmissions++; // Count retransmission
            continue;                         // Retry sending frame
        }

        alarm(connection_parameters.timeout); // Set alarm for timeout
        alarm_enabled = TRUE;                 // Enable alarm
        machine.state = START;                // Reset state machine

        // Wait for response while alarm is enabled
        while (alarm_enabled)
        {
            unsigned char byte = 0;
            int read_byte = readByteSerialPort(&byte); // Read byte from serial port

            if (read_byte == 0)
            {
                continue; // No bytes read, continue waiting
            }
            else if (read_byte < 0)
            {
                printf("Read ERROR!"); // Error reading byte
                return -1;
            }

            // Process the byte through the state machine
            state_machine(&machine, byte);
            if (machine.state == STP && machine.REJ) // REJ received
            {
                statistics.num_REJ_received++; // Count REJ received
                statistics.num_timeouts--;     // No timeout when REJ is received
                alarm(0);                      // Disable alarm
                alarm_enabled = FALSE;         // Disable alarm
                attempt--;                     // Stay on the same attempt
                break;                         // Send frame again
            }
            else if (machine.state == STP) // RR received
            {
                frame_number = 1 - frame_number; // Switch frame number
                statistics.num_RR_received++;    // Count RR received
                alarm(0);                        // Disable alarm
                alarm_enabled = FALSE;           // Disable alarm
                return bufSize;                  // Return size of buffer written
            }
        }
        statistics.num_retransmissions++; // Increment retransmission count
        statistics.num_timeouts++;        // Increment timeout count
    }
    printf("Failed to send frame after %d attempts\n", connection_parameters.nRetransmissions);
    return -1; // Failed to send frame after retries
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    struct state_machine machine;
    // Create a type READ state machine
    create_state_machine(&machine, READ, (frame_number == 0 ? I_FRAME_0 : I_FRAME_1), TRANSMITTER_ADDRESS, START);

    do
    {
        unsigned char byte = 0;
        int read_byte = readByteSerialPort(&byte); // Read byte from serial port

        if (read_byte == 0)
        {
            continue; // No bytes read, continue waiting
        }
        else if (read_byte < 0)
        {
            printf("Read ERROR!"); // Error reading byte
            return -1;
        }

        // Process the byte through the state machine
        state_machine(&machine, byte);

        // Handle received frames based on state machine state
        if (machine.state == STP && machine.ACK && frames_received == 0) // SET received
        {
            statistics.num_SET_received++; // Count SET received

            if (send_ACK() < 0) // Send ACK command
                return -1;      // Error sending ACK

            machine.state = START; // Reset state for next frame
        }
        else if (machine.state == STP && machine.duplicate) // Duplicate received
        {
            statistics.num_I_frames_received++; // Count I frames received
            statistics.num_duplicated_frames++; // Count duplicated frames

            if (send_RR() < 0)     // Send RR command
                return -1;         // Error sending RR
            machine.state = START; // Reset state for next frame
        }
        else if (machine.state == STP && machine.REJ) // New frame with bad data received
        {
            statistics.num_I_frames_received++; // Count I frames received

            if (send_REJ() < 0)    // Send REJ command
                return -1;         // Error sending REJ
            machine.state = START; // Reset state for next frame
        }
        else if (machine.state == STP) // Frame received
        {
            statistics.num_I_frames_received++; // Count I frames received
            frames_received++;                  // Increment frames received count
            break;                              // Frame received successfully
        }
    } while (machine.state != STP);

    frame_number = 1 - frame_number; // Switch frame number
    if (send_RR() < 0)               // Send RR command
        return -1;                   // Error sending RR

    memcpy(packet, machine.buf, machine.buf_size); // Copy received packet to provided buffer

    return machine.buf_size; // Return size of received packet
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    int clstat = 1; // Connection status

    // Handle connection closure based on role
    if (connection_parameters.role == LlRx)
    {
        if (llclose_receiver() < 0)
            clstat = -1; // Error during receiver close
    }
    else if (connection_parameters.role == LlTx)
    {
        if (llclose_transmitter() < 0)
            clstat = -1; // Error during transmitter close
    }

    // Close the serial port
    if (closeSerialPort() < 0)
        clstat = -1; // Error closing serial port

    // Show statistics if requested
    if (showStatistics)
        show_statistics(statistics);

    return clstat; // Return connection status
}

////////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////////

// Safely writes bytes to the serial port, handling partial writes
int safe_write(const unsigned char *bytes, int num_bytes)
{
    int total_bytes_written = 0;

    // Write bytes until all requested bytes are written
    while (total_bytes_written < num_bytes)
    {
        int bytes_to_write = num_bytes - total_bytes_written;
        int bytes_written = writeBytesSerialPort(bytes + total_bytes_written, bytes_to_write);

        if (bytes_written < 0)
        {
            printf("ERROR writing to serial port!\n");
            return -1; // Indicate an error occurred
        }

        total_bytes_written += bytes_written; // Update total written
    }

    return total_bytes_written; // Return total bytes written
}

// Send SET command to establish connection
int send_SET()
{
    unsigned char buf[5] = {FLAG, TRANSMITTER_ADDRESS, SET, 0, FLAG};
    buf[3] = buf[1] ^ buf[2]; // Calculate BCC1

    if (safe_write(buf, 5) < 0)
    {
        printf("Failed to send SET command.\n");
        return -1; // Error sending SET command
    }

    statistics.num_SET_sent++; // Count SET command sent
    return 1;                  // Successful send
}

// Send ACK command to acknowledge receipt
int send_ACK()
{
    unsigned char buf[5] = {FLAG, 0, UA, 0, FLAG};
    if (connection_parameters.role == LlRx)
    {
        buf[1] = REPLY_FROM_RECEIVER_ADDRESS; // Set address for receiver case
    }
    else if (connection_parameters.role == LlTx)
    {
        buf[1] = REPLY_FROM_TRANSMITTER_ADDRESS; // Set address for trasmitter case
    }
    buf[3] = buf[1] ^ buf[2]; // Calculate BCC1

    if (safe_write(buf, 5) < 0)
    {
        printf("Failed to send ACK command.\n");
        return -1; // Error sending UA command
    }

    statistics.num_UA_sent++; // Count UA command sent
    return 1;                 // Successful send
}

// Function to establish a connection on the receiver side
int llopen_receiver()
{
    // Initialize the state machine for connection establishment
    struct state_machine machine;
    create_state_machine(&machine, CONNECTION, SET, TRANSMITTER_ADDRESS, START);

    // Loop until the state machine reaches the STOP state (STP)
    do
    {
        unsigned char byte = 0;                    // Variable to store the byte read from the serial port
        int read_byte = readByteSerialPort(&byte); // Read a byte from the serial port

        if (read_byte == 0)
        {
            continue; // No bytes read, continue waiting
        }
        else if (read_byte < 0)
        {
            printf("Read ERROR!"); // Error reading byte
            return -1;
        }

        state_machine(&machine, byte); // Process the read byte with the state machine

    } while (machine.state != STP);

    statistics.num_SET_received++; // Increment the count of SET frames received
    return send_ACK();             // Send an acknowledgment (ACK) back to the transmitter
}

// Function to establish a connection on the transmitter side
int llopen_transmitter()
{
    // Initialize the state machine for connection establishment
    struct state_machine machine;
    create_state_machine(&machine, CONNECTION, UA, REPLY_FROM_RECEIVER_ADDRESS, START);

    unsigned int attempt = 0;             // Counter for connection attempts
    extern int alarm_enabled;             // Flag indicating if the alarm is enabled
    extern int alarm_count;               // Counter for alarm events
    alarm_enabled = FALSE;                // Initially, alarm is disabled
    alarm_count = 0;                      // Reset alarm count
    (void)signal(SIGALRM, alarm_handler); // Set the signal handler for alarms

    // Loop until the maximum number of retransmissions is reached
    while (attempt < connection_parameters.nRetransmissions)
    {
        attempt++; // Increment the attempt counter

        if (send_SET() < 0) // Attempt to send the SET frame
        {
            alarm(0);                         // Disable the alarm
            statistics.num_retransmissions++; // Increment retransmission count
            continue;                         // Retry sending SET if it fails
        }

        alarm(connection_parameters.timeout); // Set the alarm for timeout duration
        alarm_enabled = TRUE;                 // Enable the alarm
        machine.state = START;                // Reset the state machine state

        // Loop while the alarm is enabled (waiting for UA frame)
        while (alarm_enabled)
        {
            unsigned char byte = 0;                    // Variable to store the byte read from the serial port
            int read_byte = readByteSerialPort(&byte); // Read a byte from the serial port

            if (read_byte == 0)
            {
                continue; // No bytes read, continue waiting
            }
            else if (read_byte < 0)
            {
                printf("Read ERROR!"); // Error reading byte
                return -1;
            }

            state_machine(&machine, byte); // Process the read byte with the state machine

            // Check if the state machine has reached the STOP state (STP)
            if (machine.state == STP)
            {
                alarm(0);                     // Disable the alarm
                alarm_enabled = FALSE;        // Mark alarm as disabled
                statistics.num_UA_received++; // Increment the count of UA frames received
                return 1;                     // Successful connection establishment
            }
        }
        statistics.num_retransmissions++; // Increment retransmission count
        statistics.num_timeouts++;        // Increment timeout count
    }
    printf("Failed to establish connection after %d attempts\n", connection_parameters.nRetransmissions);
    return -1; // Return error if maximum retransmissions are reached without success
}

// Function to send a data frame over the serial connection
int send_data_frame(const unsigned char *buf, int buf_size)
{
    // Allocate memory for the frame:
    // (data_size + BCC2) * 2 for potential byte stuffing + (F; A; C; BCC1) + F
    unsigned char frame[(buf_size + 1) * 2 + 5];
    int frame_size = 0; // Initialize frame size counter

    // Start constructing the frame
    frame[frame_size++] = FLAG;                                        // Start flag
    frame[frame_size++] = TRANSMITTER_ADDRESS;                         // Transmitter address
    frame[frame_size++] = (frame_number == 0) ? I_FRAME_0 : I_FRAME_1; // Frame type (I_FRAME_0 or I_FRAME_1)
    frame[frame_size++] = frame[1] ^ frame[2];                         // Calculate BCC1 (XOR of address and control field)

    unsigned char BCC2 = 0; // Initialize BCC2
    for (int i = 0; i < buf_size; i++)
    {
        if (buf[i] == FLAG) // Check for FLAG byte to perform byte stuffing
        {
            frame[frame_size++] = ESC;      // Add ESC before FLAG
            frame[frame_size++] = ESC_FLAG; // Escape FLAG
        }
        else if (buf[i] == ESC) // Check for ESC byte to perform byte stuffing
        {
            frame[frame_size++] = ESC;     // Add ESC before ESC
            frame[frame_size++] = ESC_ESC; // Escape ESC
        }
        else
        {
            frame[frame_size++] = buf[i]; // Add data byte to frame
        }
        BCC2 ^= buf[i]; // Compute BCC2 using XOR
    }

    // Byte Stuff BCC2
    if (BCC2 == FLAG) // Check if BCC2 is equal to the FLAG byte
    {
        frame[frame_size++] = ESC;      // Add ESC before BCC2
        frame[frame_size++] = ESC_FLAG; // Escape FLAG
    }
    else if (BCC2 == ESC) // Check if BCC2 is equal to the ESC byte
    {
        frame[frame_size++] = ESC;     // Add ESC before BCC2
        frame[frame_size++] = ESC_ESC; // Escape ESC
    }
    else
    {
        frame[frame_size++] = BCC2; // Add BCC2 to frame
    }

    frame[frame_size++] = FLAG; // End flag

    // Attempt to write the frame to the serial port
    if (safe_write(frame, frame_size) < 0)
    {
        printf("Failed to send frame %d!\n", frame_number);
        return -1; // Return -1 on failure
    }

    statistics.num_I_frames_sent++; // Increment the count of I frames sent
    return 1;                       // Return 1 on success
}

// Function to send a RR command
int send_RR()
{
    // Create a buffer to hold the RR frame
    unsigned char buf[5] = {FLAG, REPLY_FROM_RECEIVER_ADDRESS, frame_number == 0 ? RR0 : RR1, 0, FLAG};
    buf[3] = buf[1] ^ buf[2]; // Calculate BCC1

    // Attempt to send the RR command
    if (safe_write(buf, 5) < 0)
    {
        printf("Failed to send RR%d command.\n", frame_number);
        return -1; // Return -1 on failure
    }

    statistics.num_RR_sent++; // Increment the count of RR commands sent
    return 1;                 // Return 1 on success
}

// Function to send a REJ command
int send_REJ()
{
    // Create a buffer to hold the REJ frame
    unsigned char buf[5] = {FLAG, REPLY_FROM_RECEIVER_ADDRESS, frame_number == 0 ? REJ0 : REJ1, 0, FLAG};
    buf[3] = buf[1] ^ buf[2]; // Calculate BCC1

    // Attempt to send the REJ command
    if (safe_write(buf, 5) < 0)
    {
        printf("Failed to send REJ%d command.\n", frame_number);
        return -1; // Return -1 on failure
    }

    statistics.num_REJ_sent++; // Increment the count of REJ commands sent
    return 1;                  // Return 1 on success
}

// Function to send a DISC command
int send_DISC()
{
    // Create a buffer to hold the DISC frame
    unsigned char buf[5] = {FLAG, 0, DISC, 0, FLAG};

    // Set the second byte based on the role (Receiver or Transmitter)
    if (connection_parameters.role == LlRx)
    {
        buf[1] = RECEIVER_ADDRESS; // Receiver's address
    }
    else if (connection_parameters.role == LlTx)
    {
        buf[1] = TRANSMITTER_ADDRESS; // Transmitter's address
    }

    buf[3] = buf[1] ^ buf[2]; // Calculate BCC1

    // Attempt to send the DISC command
    if (safe_write(buf, 5) < 0)
    {
        printf("Failed to send DISC command.\n");
        return -1; // Return -1 on failure
    }

    statistics.num_DISC_sent++; // Increment the count of DISC commands sent
    return 1;                   // Return 1 on success
}

// Function to close the connection from the transmitter's side
int llclose_transmitter()
{
    // Initialize the state machine for the disconnection process
    struct state_machine machine;
    create_state_machine(&machine, DISCONNECTION, DISC, RECEIVER_ADDRESS, START);

    unsigned int attempt = 0;
    extern int alarm_enabled;
    extern int alarm_count;
    alarm_enabled = FALSE; // Disable alarm
    alarm_count = 0;       // Reset alarm count

    (void)signal(SIGALRM, alarm_handler); // Set the alarm handler

    // Try sending the DISC command up to nRetransmissions times
    while (attempt < connection_parameters.nRetransmissions)
    {
        attempt++;

        if (send_DISC() < 0) // Attempt to send DISC
        {
            alarm(0);                         // Disable alarm if sending fails
            statistics.num_retransmissions++; // Increment retransmission count
            continue;                         // Retry sending DISC
        }

        alarm(connection_parameters.timeout); // Set the alarm with the timeout value
        alarm_enabled = TRUE;                 // Enable the alarm
        machine.state = START;                // Reset state machine state

        // Wait for a response while the alarm is enabled
        while (alarm_enabled)
        {
            unsigned char byte = 0;
            int read_byte = readByteSerialPort(&byte); // Read a byte from the serial port

            if (read_byte == 0)
            {
                continue; // No bytes read, continue waiting
            }
            else if (read_byte < 0)
            {
                printf("Read ERROR!");
                return -1; // Return error on read failure
            }

            // Process the received byte with the state machine
            state_machine(&machine, byte);

            if (machine.state == STP) // If in STOP state, DISC received successfully
            {
                statistics.num_DISC_received++; // Increment DISC received count
                alarm(0);                       // Disable alarm
                alarm_enabled = FALSE;          // Disable alarm flag

                if (send_ACK() < 0) // Send ACK in response
                    return -1;

                return 1; // Return success
            }
        }
        statistics.num_retransmissions++; // Increment retransmission count on timeout
        statistics.num_timeouts++;        // Increment timeout count
    }
    printf("Failed to send DISC after %d attempts\n", connection_parameters.nRetransmissions);
    return -1; // Return error after max attempts
}

// Function to close the connection from the receiver's side
int llclose_receiver()
{
    // Initialize the state machine for receiving the DISC frame
    struct state_machine machine;
    create_state_machine(&machine, DISCONNECTION, DISC, TRANSMITTER_ADDRESS, START);

    // Wait for the DISC frame from the transmitter
    do
    {
        unsigned char byte = 0;
        int read_byte = readByteSerialPort(&byte); // Read a byte from the serial port

        if (read_byte == 0)
        {
            continue; // No bytes read, continue waiting
        }
        else if (read_byte < 0)
        {
            printf("Read ERROR!");
            return -1; // Return error on read failure
        }

        // Process the received byte with the state machine
        state_machine(&machine, byte);

        if (machine.state == STP) // If in STOP state, DISC received successfully
        {
            statistics.num_DISC_received++; // Increment DISC received count
            break;                          // Exit the loop
        }
    } while (machine.state != STP);

    // Prepare to reply with a DISC and wait for the UA reply
    create_state_machine(&machine, DISCONNECTION, UA, REPLY_FROM_TRANSMITTER_ADDRESS, START);

    unsigned int attempt = 0;
    extern int alarm_enabled;
    extern int alarm_count;
    alarm_enabled = FALSE; // Disable alarm
    alarm_count = 0;       // Reset alarm count

    (void)signal(SIGALRM, alarm_handler); // Set the alarm handler

    // Try sending the DISC command up to nRetransmissions times
    while (attempt < connection_parameters.nRetransmissions)
    {
        attempt++;

        if (send_DISC() < 0) // Attempt to send DISC
        {
            alarm(0);                         // Disable alarm if sending fails
            statistics.num_retransmissions++; // Increment retransmission count
            continue;                         // Retry sending DISC
        }

        alarm(connection_parameters.timeout); // Set the alarm with the timeout value
        alarm_enabled = TRUE;                 // Enable the alarm
        machine.state = START;                // Reset state machine state

        // Wait for a response while the alarm is enabled
        while (alarm_enabled)
        {
            unsigned char byte = 0;
            int read_byte = readByteSerialPort(&byte); // Read a byte from the serial port

            if (read_byte == 0)
            {
                continue; // No bytes read, continue waiting
            }
            else if (read_byte < 0)
            {
                printf("Read ERROR!");
                return -1; // Return error on read failure
            }

            // Process the received byte with the state machine
            state_machine(&machine, byte);

            if (machine.state == STP) // If in STOP state, UA received successfully
            {
                statistics.num_UA_received++; // Increment UA received count
                alarm(0);                     // Disable alarm
                alarm_enabled = FALSE;        // Disable alarm flag
                return 1;                     // Return success
            }
        }
        statistics.num_retransmissions++; // Increment retransmission count on timeout
        statistics.num_timeouts++;        // Increment timeout count
    }
    printf("Failed to send DISC after %d attempts\n", connection_parameters.nRetransmissions);
    return -1; // Return error after max attempts
}

// Function to display statistics about the communication
void show_statistics(struct ll_statistics statistics)
{
    // Calculate total frames sent and received
    int num_frames_sent = statistics.num_SET_sent +
                          statistics.num_UA_sent +
                          statistics.num_RR_sent +
                          statistics.num_REJ_sent +
                          statistics.num_I_frames_sent +
                          statistics.num_DISC_sent;

    int num_frames_received = statistics.num_SET_received +
                              statistics.num_UA_received +
                              statistics.num_RR_received +
                              statistics.num_REJ_received +
                              statistics.num_I_frames_received +
                              statistics.num_DISC_received;

    // Print out the statistics in a readable format
    printf("\n");
    printf("Total Frames Sent: %d\n", num_frames_sent);
    printf("Total Frames Received: %d\n", num_frames_received);
    printf("Total SET Frames Sent: %d\n", statistics.num_SET_sent);
    printf("Total SET Frames Received: %d\n", statistics.num_SET_received);
    printf("Total UA Frames Sent: %d\n", statistics.num_UA_sent);
    printf("Total UA Frames Received: %d\n", statistics.num_UA_received);
    printf("Total RR Frames Sent: %d\n", statistics.num_RR_sent);
    printf("Total RR Frames Received: %d\n", statistics.num_RR_received);
    printf("Total REJ Frames Sent: %d\n", statistics.num_REJ_sent);
    printf("Total REJ Frames Received: %d\n", statistics.num_REJ_received);
    printf("Total I Frames Sent: %d\n", statistics.num_I_frames_sent);
    printf("Total I Frames Received: %d\n", statistics.num_I_frames_received);
    printf("Total DISC Frames Sent: %d\n", statistics.num_DISC_sent);
    printf("Total DISC Frames Received: %d\n", statistics.num_DISC_received);
    printf("Total Invalid BCC1 Received: %d\n", statistics.num_invalid_BCC1_received);
    printf("Total Invalid BCC2 Received: %d\n", statistics.num_invalid_BCC2_received);
    printf("Total Duplicated Frames Received: %d\n", statistics.num_duplicated_frames);
    printf("Total Timeouts: %d\n", statistics.num_timeouts);
    printf("Total Retransmissions: %d\n", statistics.num_retransmissions);
    printf("\n");
}
