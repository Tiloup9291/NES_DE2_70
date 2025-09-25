#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stdafx.h"
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include "termios2_compat.h"
#include <linux/joystick.h>
#include <errno.h>

typedef unsigned char byte;
typedef unsigned long DWORD;

unsigned int swapbits(unsigned int a) {
  a &= 0xff;
  unsigned int b = 0;
  for(int i = 0; i < 8; i++, a >>= 1, b <<= 1) b |= (a & 1);
  return b>>1;
}

#define POLY 0x8408
unsigned int crc16(byte *data_p, unsigned short length, unsigned int crc)
{
      unsigned char i;
      unsigned int data;
      if (length == 0)
            return crc;
      do {
        for (i=0, data=(unsigned int)0xff & *data_p++; i < 8; i++, data <<= 1) {
              if ((crc & 0x0001) ^ ((data >> 7) & 0x0001))
                    crc = (crc >> 1) ^ POLY;
              else  crc >>= 1;
        }
      } while (--length);
      return crc;
}

unsigned int crc16b(byte *data_p, unsigned short length, unsigned int crc) {
  byte c[16], newcrc[16];
  byte d[8];
  for(int j = 0; j < 16; j++) c[j] = (crc >> (15-j)) & 1;

  for(int i = 0; i < length; i++) {
    for(int j = 0; j < 8; j++) d[j] = (data_p[i] >> j) & 1;

    newcrc[0] = d[4] ^ d[0] ^ c[8] ^ c[12];
    newcrc[1] = d[5] ^ d[1] ^ c[9] ^ c[13];
    newcrc[2] = d[6] ^ d[2] ^ c[10] ^ c[14];
    newcrc[3] = d[7] ^ d[3] ^ c[11] ^ c[15];
    newcrc[4] = d[4] ^ c[12];
    newcrc[5] = d[5] ^ d[4] ^ d[0] ^ c[8] ^ c[12] ^ c[13];
    newcrc[6] = d[6] ^ d[5] ^ d[1] ^ c[9] ^ c[13] ^ c[14];
    newcrc[7] = d[7] ^ d[6] ^ d[2] ^ c[10] ^ c[14] ^ c[15];
    newcrc[8] = d[7] ^ d[3] ^ c[0] ^ c[11] ^ c[15];
    newcrc[9] = d[4] ^ c[1] ^ c[12];
    newcrc[10] = d[5] ^ c[2] ^ c[13];
    newcrc[11] = d[6] ^ c[3] ^ c[14];
    newcrc[12] = d[7] ^ d[4] ^ d[0] ^ c[4] ^ c[8] ^ c[12] ^ c[15];
    newcrc[13] = d[5] ^ d[1] ^ c[5] ^ c[9] ^ c[13];
    newcrc[14] = d[6] ^ d[2] ^ c[6] ^ c[10] ^ c[14];
    newcrc[15] = d[7] ^ d[3] ^ c[7] ^ c[11] ^ c[15];

    memcpy(c, newcrc, 16);
  }

  unsigned int r = 0;
  for(int j = 0; j < 16; j++) r = r * 2 + c[j];
  return r;
}

size_t FormatPacket(byte *buf, int address, const void *data, int data_size) {
  byte *org = buf;
  while (data_size) {
    int n = data_size > 256 ? 256 : data_size;
    int cksum = address + n;
    buf[1] = address;
    buf[2] = n;
    for(int i = 0; i < n; i++) {
      int v = ((byte*)data)[i];
      buf[i+3] = v;
      cksum += v;
    }
    buf[0] = -cksum;
    buf += n + 3;
    data = (char*)data + n;
    data_size -= n;
  }
  return buf - org;
}

void WritePacket(int h, int address, const void *data, size_t data_size) {
  byte buf[(3+64) * 256];
  size_t n = FormatPacket(buf, address, data, data_size);
  ssize_t written = write(h, data, n);
  if (written < n) {
    printf("WriteFile failed\n");
    return;
  }
}

int set_custom_baudrate(int fd, int baudrate) {
    struct termios2 tio2;

    if (ioctl(fd, TCGETS2, &tio2) < 0) {
        printf("TCGETS2");
        return -1;
    }

    tio2.c_cflag &= ~CBAUD;
    tio2.c_cflag |= BOTHER;   // Activer baudrate personnalisé
    tio2.c_ispeed = baudrate;
    tio2.c_ospeed = baudrate;

    if (ioctl(fd, TCSETS2, &tio2) < 0) {
        printf("TCSETS2");
        return -1;
    }

    return 0;
}

int configure_port(int fd) {
    struct termios options;

    if (tcgetattr(fd, &options) < 0) {
        printf("tcgetattr");
        return -1;
    }

    // 8 bits
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // 1 stop bit
    options.c_cflag &= ~CSTOPB;

    // Pas de parité (équivalent fBinary = TRUE)
    options.c_cflag &= ~PARENB;

    // Mode brut (raw)
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                         | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Activer lecture et ignorer contrôle modem
    options.c_cflag |= (CLOCAL | CREAD);

    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        printf("tcsetattr");
        return -1;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    if (argc < 3) {
        printf("Not Enough Arguments\n");
        return 1;
    }

    int h = open(argv[2], O_RDWR | O_NOCTTY | O_NDELAY);
    if (h < 0) {
        printf("CreateFile failed\n");
        return 0;
    }

    if (set_custom_baudrate(h, 21428571) != 0) {
        printf("error setting baudrate\n");
        close(h);
        return 1;
    }

    if (configure_port(h) != 0) {
        close(h);
        printf("SetCommState failed\n");
        return 1;
    }

    FILE* f = fopen(argv[1], "rb");
    if (!f) { printf("File open fail\n"); close(h); return 1; }

    { char v = 1; WritePacket(h, 0x35, &v, 1); }
    { char v = 0; WritePacket(h, 0x35, &v, 1); }

    size_t total_read = 0xffffff;//10180;
    size_t pos = 0;
    while (pos < total_read) {
        char buf[16384];
        size_t want_read = (total_read - pos) > sizeof(buf) ? sizeof(buf) : (total_read - pos);
        int n = fread(buf, 1, want_read, f);
        if (n <= 0) {
            break;
        }
        WritePacket(h, 0x37, buf, n);
        pos += n;
    }

    int last_keys = -1;
    const char *joydev = "/dev/input/js0";
    int fd = open(joydev, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        printf("open joystick");
        close(h);
        fclose(f);
        f = NULL;
        return 1;
    }

    unsigned char keys = 0;

    struct js_event e;
    while (1) {
        ssize_t r = read(fd, &e, sizeof(e));
        if (r == sizeof(e)) {
            // On reçoit un événement : bouton ou axe
            if (e.type & JS_EVENT_BUTTON) {
                // e.number = numéro du bouton, e.value = 0 ou 1
                // Exemple simple : si bouton 2 pressé, etc.
                // à ajuster selon mapping voulu
                if (e.number == 2 && e.value) keys |= 1;
                if (e.number == 3 && e.value) keys |= 2;
                if (e.number == 8 && e.value) keys |= 4;
                if (e.number == 9 && e.value) keys |= 8;
            }
            else if (e.type & JS_EVENT_AXIS) {
                // e.number = axe (0 = X, 1 = Y typiquement)
                if (e.number == 1) {
                    // axe Y
                    if (e.value < -16384) keys |= 16;
                    if (e.value > 16384) keys |= 32;
                }
                if (e.number == 0) {
                    // axe X
                    if (e.value < -16384) keys |= 64;
                    if (e.value > 16384) keys |= 128;
                }
            }

            if (keys != last_keys) {
                printf("Keys %.2x\n", keys);
                WritePacket(h, 0x40, &keys, 1);
                last_keys = keys;
            }

        } else if (r == -1 && errno != EAGAIN) {
            printf("read joystick");
            break;
        }

        usleep(1000);  // 1 ms
    }
    close(h);
    fclose(f);
    f = NULL;
    close(fd);
    return 0;

}
