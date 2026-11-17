/* VM-only UART-holder. Does not speak XRCE/DDS or drive hardware. */
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
static volatile sig_atomic_t run=1;
static void quit(int sig){(void)sig;run=0;}
int main(void){int fd=open("/dev/ttyS2",O_RDWR|O_NOCTTY);if(fd<0)return 2;signal(SIGTERM,quit);signal(SIGINT,quit);while(run)pause();close(fd);return 0;}
