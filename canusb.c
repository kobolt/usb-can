#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <asm/termbits.h> /* struct termios2 */
#include <time.h>



#define CANUSB_BAUD_RATE 2000000

typedef enum {
  CANUSB_SPEED_1000000 = 0x01,
  CANUSB_SPEED_800000  = 0x02,
  CANUSB_SPEED_500000  = 0x03,
  CANUSB_SPEED_400000  = 0x04,
  CANUSB_SPEED_250000  = 0x05,
  CANUSB_SPEED_200000  = 0x06,
  CANUSB_SPEED_125000  = 0x07,
  CANUSB_SPEED_100000  = 0x08,
  CANUSB_SPEED_50000   = 0x09,
  CANUSB_SPEED_20000   = 0x0a,
  CANUSB_SPEED_10000   = 0x0b,
  CANUSB_SPEED_5000    = 0x0c,
} CANUSB_SPEED;

typedef enum {
  CANUSB_MODE_NORMAL          = 0x00,
  CANUSB_MODE_LOOPBACK        = 0x01,
  CANUSB_MODE_SILENT          = 0x02,
  CANUSB_MODE_LOOPBACK_SILENT = 0x03,
} CANUSB_MODE;

typedef enum {
  CANUSB_FRAME_STANDARD = 0x01,
  CANUSB_FRAME_EXTENDED = 0x02,
} CANUSB_FRAME;




static int print_traffic = 0;



static CANUSB_SPEED canusb_int_to_speed(int speed)
{
  switch (speed) {
  case 1000000:
    return CANUSB_SPEED_1000000;
  case 800000:
    return CANUSB_SPEED_800000;
  case 500000:
    return CANUSB_SPEED_500000;
  case 400000:
    return CANUSB_SPEED_400000;
  case 250000:
    return CANUSB_SPEED_250000;
  case 200000:
    return CANUSB_SPEED_200000;
  case 125000:
    return CANUSB_SPEED_125000;
  case 100000:
    return CANUSB_SPEED_100000;
  case 50000:
    return CANUSB_SPEED_50000;
  case 20000:
    return CANUSB_SPEED_20000;
  case 10000:
    return CANUSB_SPEED_10000;
  case 5000:
    return CANUSB_SPEED_5000;
  default:
    return 0;
  }
}



static int generate_checksum(unsigned char *data, int data_len)
{
  int i, checksum;

  checksum = 0;
  for (i = 0; i < data_len; i++) {
    checksum += data[i];
  }

  return checksum & 0xff;
}



static int frame_is_complete(unsigned char *frame, int frame_len)
{
  if (frame_len > 0) {
    if (frame[0] != 0xaa) {
      /* Need to sync on 0xaa at start of frames, so just skip. */
      return 1;
    }
  }

  if (frame_len < 2) {
    return 0;
  }

  if (frame[1] == 0x55) { /* Command frame... */
    if (frame_len >= 20) { /* ...always 20 bytes. */
      return 1;
    } else {
      return 0;
    }
  } else if ((frame[1] >> 4) == 0xc) { /* Data frame... */
    if (frame_len >= (frame[1] & 0xf) + 5) { /* ...payload and 5 bytes. */
      return 1;
    } else {
      return 0;
    }
  }

  /* Unhandled frame type. */
  return 1;
}



static int frame_send(int tty_fd, unsigned char *frame, int frame_len)
{
  int result, i;

  if (print_traffic) {
    printf(">>> ");
    for (i = 0; i < frame_len; i++) {
      printf("%02x ", frame[i]);
    }
    printf("\n");
  }

  result = write(tty_fd, frame, frame_len);
  if (result == -1) {
    fprintf(stderr, "write() failed: %s\n", strerror(errno));
    return -1;
  }

  return frame_len;
}



static int frame_recv(int tty_fd, unsigned char *frame, int frame_len_max)
{
  int result, frame_len, checksum;
  unsigned char byte;

  if (print_traffic)
    fprintf(stderr, "<<< ");

  frame_len = 0;
  while (1) {
    result = read(tty_fd, &byte, 1);
    if (result == -1) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        fprintf(stderr, "read() failed: %s\n", strerror(errno));
        return -1;
      }

    } else if (result > 0) {
      if (print_traffic)
        fprintf(stderr, "%02x ", byte);

      if (frame_len == frame_len_max) {
        fprintf(stderr, "frame_recv() failed: Overflow\n");
        return -1;
      }

      frame[frame_len++] = byte;

      if (frame_is_complete(frame, frame_len)) {
        break;
      }
    }

    usleep(10);
  }

  if (print_traffic)
    fprintf(stderr, "\n");

  /* Compare checksum for command frames only. */
  if ((frame_len == 20) && (frame[0] == 0xaa) && (frame[1] == 0x55)) {
    checksum = generate_checksum(&frame[2], 17);
    if (checksum != frame[frame_len - 1]) {
      fprintf(stderr, "frame_recv() failed: Checksum incorrect\n");
      return -1;
    }
  }

  return frame_len;
}



static int command_settings(int tty_fd, CANUSB_SPEED speed, CANUSB_MODE mode, CANUSB_FRAME frame)
{
  int cmd_frame_len;
  unsigned char cmd_frame[20];

  cmd_frame_len = 0;
  cmd_frame[cmd_frame_len++] = 0xaa;
  cmd_frame[cmd_frame_len++] = 0x55;
  cmd_frame[cmd_frame_len++] = 0x12;
  cmd_frame[cmd_frame_len++] = speed;
  cmd_frame[cmd_frame_len++] = frame;
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = mode;
  cmd_frame[cmd_frame_len++] = 0x01;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = generate_checksum(&cmd_frame[2], 17);

  if (frame_send(tty_fd, cmd_frame, cmd_frame_len) < 0) {
    return -1;
  }

  return 0;
}



static void dump_data_frames(int tty_fd)
{
  int i, frame_len;
  unsigned char frame[32];
  struct timespec ts;

  while (1) {
    frame_len = frame_recv(tty_fd, frame, sizeof(frame));

    clock_gettime(CLOCK_MONOTONIC, &ts);
    printf("%lu.%06lu ", ts.tv_sec, ts.tv_nsec / 1000);

    if (frame_len == -1) {
      printf("Frame recieve error!\n");

    } else {

      if ((frame_len >= 6) &&
          (frame[0] == 0xaa) &&
          ((frame[1] >> 4) == 0xc)) {
        printf("Frame ID: %02x%02x, Data: ", frame[3], frame[2]);
        for (i = frame_len - 2; i > 3; i--) {
          printf("%02x ", frame[i]);
        }
        printf("\n");

      } else {
        printf("Unknown:");
        for (i = 0; i <= frame_len; i++) {
          printf("%02x ", frame[i]);
        }
        printf("\n");
      }
    }
  }
}



static int adapter_init(char *tty_device)
{
  int tty_fd, result;
  struct termios2 tio;

  tty_fd = open(tty_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (tty_fd == -1) {
    fprintf(stderr, "open(%s) failed: %s\n", tty_device, strerror(errno));
    return -1;
  }
 
  result = ioctl(tty_fd, TCGETS2, &tio);
  if (result == -1) {
    fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
    close(tty_fd);
    return -1;
  }

  tio.c_cflag &= ~CBAUD;
  tio.c_cflag = BOTHER | CS8 | CSTOPB;
  tio.c_iflag = IGNPAR;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_ispeed = CANUSB_BAUD_RATE;
  tio.c_ospeed = CANUSB_BAUD_RATE;

  result = ioctl(tty_fd, TCSETS2, &tio);
  if (result == -1) {
    fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
    close(tty_fd);
    return -1;
  }

  return tty_fd;
}



static void display_help(char *progname)
{
  fprintf(stderr, "Usage: %s <options>\n", progname);
  fprintf(stderr, "Options:\n"
     "  -h          Display this help and exit.\n"
     "  -t          Print TTY/serial traffic debugging info on stderr.\n"
     "  -d DEVICE   Use TTY DEVICE.\n"
     "  -s SPEED    Set CAN SPEED in bps.\n"
     "\n");
}



int main(int argc, char *argv[])
{
  int c, tty_fd;
  char *tty_device = NULL;
  CANUSB_SPEED speed = 0;

  while ((c = getopt(argc, argv, "htd:s:")) != -1) {
    switch (c) {
    case 'h':
      display_help(argv[0]);
      return EXIT_SUCCESS;

    case 't':
      print_traffic = 1;
      break;

    case 'd':
      tty_device = optarg;
      break;

    case 's':
      speed = canusb_int_to_speed(atoi(optarg));
      break;

    case '?':
    default:
      display_help(argv[0]);
      return EXIT_FAILURE;
    }
  }

  if (tty_device == NULL) {
    fprintf(stderr, "Please specify a TTY!\n");
    display_help(argv[0]);
    return EXIT_FAILURE;
  }

  if (speed == 0) {
    fprintf(stderr, "Please specify a valid speed!\n");
    display_help(argv[0]);
    return EXIT_FAILURE;
  }

  tty_fd = adapter_init(tty_device);
  if (tty_fd == -1) {
    return EXIT_FAILURE;
  }

  command_settings(tty_fd, speed, CANUSB_MODE_NORMAL, CANUSB_FRAME_STANDARD);
  dump_data_frames(tty_fd);

  return EXIT_SUCCESS;
}

