#include "client_packet.h"

#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
   
    
nota* client_receive_packet(int fd, nota* n){

    uint8_t buf[2];
    int ret;
    
    ret = read(fd, buf, 1);
    if (ret == -1 && errno != EINTR) perror("Error reading");
	
    ret = read(fd, buf+1, 1);
    if (ret == -1 && errno != EINTR) perror("Error reading");

    n->pin = buf[0];
    n->tipo_evento = buf[1];
    
    return n;

}
