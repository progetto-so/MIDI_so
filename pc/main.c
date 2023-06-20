#include "client_packet.h"
#include "nota.h"

#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdint.h>

#include <signal.h>
#include <sys/wait.h>
#include <assert.h>
#include <math.h>
#include <alsa/asoundlib.h>

#define S_RATE (44100)
#define BTN0 10
#define BTN1 11
#define BTN2 12

#define handle_error(msg)   \
    do                      \
    {                       \
        perror(msg);        \
        exit(EXIT_FAILURE); \
    } while (0)

/**================================================== *
 * ==========  SERIAL FUNCTIONS  ========== *
 * ================================================== */

int fd;
// pthread_t reader;
int connected = 0;

int serial_set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("Error from tcgetattr");
        return -1;
    }
    switch (speed)
    {
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    default:
        printf("cannot set baudrate %d\n", speed);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    cfmakeraw(&tty);

    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag |= parity;
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
        return -1;
    }
    return 0;
}

int open_serial()
{
    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror("Error opening serial");
        return -1;
    }

    int ret = serial_set_interface_attribs(fd, 57600, 0);
    if (ret != 0)
    {
        perror("Error setting serial interface");
        return -1;
    }
    return 0;
}

/**================================================== *
 * ==========  SOUND  ========== *
 * ================================================== */

int j;
int BUF_SIZE = S_RATE * 5;
short *buffer;

short *playsound(uint8_t pin, short *buffer)
{

    int i;
    float amplitude = 32000;
    float freq_Hz;
    float phase = 0;

    if (pin == BTN0)
    { // DO
        freq_Hz = 261;
    }
    else if (pin == BTN1)
    { // MI
        freq_Hz = 330;
    }
    else if (pin == BTN2)
    { // SOL
        freq_Hz = 392;
    }
    else
    {
        freq_Hz = 0;
    }

    float freq_radians_per_sample = freq_Hz * 2 * 3.14 / S_RATE;

    /* fill buffer with a sine wave */
    for (i = 0; i < BUF_SIZE; i++)
    {
        phase += freq_radians_per_sample;
        // double ph= sin( phase);
        buffer[i] = (int)(amplitude * sin(phase));
    }

    return buffer;
}

snd_pcm_t *pcm;

int suona(nota *n, short *buffer)
{

    snd_pcm_open(&pcm, "default", SND_PCM_STREAM_PLAYBACK, 0);

    snd_pcm_hw_params_t *hw_params;
    snd_pcm_hw_params_alloca(&hw_params);

    snd_pcm_hw_params_any(pcm, hw_params);

    int ret;
    if ((ret = snd_pcm_hw_params_set_access(pcm, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
    {
        handle_error("ERRORE: set_access");
    }
    if ((ret = snd_pcm_hw_params_set_format(pcm, hw_params, SND_PCM_FORMAT_S16_LE)) < 0)
    {
        handle_error("ERRORE: set_format");
    }
    if ((ret = snd_pcm_hw_params_set_channels(pcm, hw_params, 1)) < 0)
    {
        handle_error("ERRORE: set_channels_number");
    }
    if ((ret = snd_pcm_hw_params_set_rate(pcm, hw_params, 48000, 0)) < 0)
    {
        handle_error("ERRORE: set_rate");
    }
    if ((ret = snd_pcm_hw_params(pcm, hw_params)) < 0)
    {
        handle_error("ERRORE: set_harware_parameters");
    }

    if ((ret = snd_pcm_writei(pcm, playsound((n->pin), buffer), BUF_SIZE)) < 0)
    {
        handle_error("ERRORE: write_to_pcm_device");
    }
    else if (ret == -EPIPE)
    {
        snd_pcm_drop(pcm);
    }

    return 0;
}

void stop()
{
    snd_pcm_drop(pcm);
    snd_pcm_drain(pcm);
    snd_pcm_close(pcm);
}

/**================================================== *
 * ==========  THREADS WORK  ========== *
 * ================================================== */

void reader_work(nota *n, short *buffer)
{

    while (connected)
    {

        n = client_receive_packet(fd, n);

        if ((n->tipo_evento) == 0)
        {
            suona(n, buffer);
        }
        else if ((n->tipo_evento) == 1)
        {
            stop();
        }

        printf("stampo nota: pin = %d, tipo_evento: %d\n", n->pin, n->tipo_evento);
        // client_print_packet(n);
    }

    return;
}

/**================================================== *
 * ==========  MAIN  ========== *
 * ================================================== */

int main(int argc, char **argv)
{

    nota *n = (nota *)malloc(sizeof(nota));
    buffer = (short *)malloc(sizeof(short) * BUF_SIZE);

    int ret = open_serial();
    if (ret)
    {
        perror("Error in open_serial");
    }

    connected = 1;

    reader_work(n, buffer);

    free(n);
    free(buffer);

    close(fd);
}
