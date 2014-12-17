/* beep.c - for Linux and DOS/Windows */

#include <stdio.h>
#include <stdlib.h>
#ifdef __DJGPP__
#include <dos.h>
#include <pc.h>
#endif
#define ESC 27

void beep (int frequency, int duration)
{
#ifdef __DJGPP__
  sound (frequency);
  delay (duration);
  nosound ();
#else  /* Linux */
  FILE *tty;
  if ( NULL == (tty = fopen ("/dev/console", "w")) ) {
    fprintf (stderr, "Cannot write to /dev/console!\n" );
    exit (1);
  }
  fprintf(tty, "%c[10;%d]%c[11;%d]\a", ESC, frequency, ESC, duration);
#endif
}

int main (int argc, char *argv[])
{
  int frequency, duration;
  if (argc != 3) {
    fprintf (stderr, "Usage: beep <frequency> <duration>\n" );
    exit (1);
  }
  frequency = atoi (argv [1]);
  duration = atoi (argv [2]);
  beep (frequency, duration);
  return (0);
}

/* end of beep.c */
