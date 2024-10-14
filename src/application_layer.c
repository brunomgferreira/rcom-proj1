// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // TODO

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

    llopen(connection_parameters);

    llclose(0);
}
