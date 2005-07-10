/*
 * psp_remote : Serial remote software interface for Sony PSP
 * version 1.00
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * 
 * Notes:
 *
 * Most of the code below is based on the information provided by
 * Marcus Comstedt et al. at: http://mc.pp.se/psp/phones.xhtml and 
 * http://forums.ps2dev.org/viewtopic.php?t=986
 *
 */

#include <stdlib.h>
#include <sys/time.h>                   
#include <string.h>
#include <ctype.h>
#include <signal.h>               
#include <termios.h>         
#include <unistd.h>              
#include <fcntl.h>              
#include <sys/ioctl.h>               // serial line status
#include <ncurses.h>                 // console I/O
#include <getopt.h>                  // parameter processing


#define u8  unsigned char            // The usual supsect           
#define u16 unsigned short           // The usual supsect           

#define NAME_SIZE   64               // Maximum device name size
#define DEFAULT_DEV "/dev/ttyS0"     // port the device is plugged in to
#define BAUDRATE    B4800            // baud rate the device spits out at
#define BYTE_DELAY  2084             // Time it takes to send one byte 
                                     // at above baudrate (in us)
#define KB_DELAY    2                // Time spent waiting for keyboard (ms)
#define KEY_TIMEOUT 50               // Duration for a pressed key to be highlighted
#define MAX_BYTES   10               // Maximum number of bytes per frame

// STDOUT functions for ncurses and timestamping
#define POUT(win,args...)            { wprintw(win, ## args); wrefresh(win); }
#define PSTATUS(color, arg)          { wattron(wstatus,COLOR_PAIR(color)); mvwprintw(wstatus, 0, 17, arg); wattroff(wstatus,COLOR_PAIR(color)); wrefresh(wstatus); }
#define PKEYS(color, knum)           { wattron(wkeys,COLOR_PAIR(color)); mvwprintw(wkeys, kd[knum].y, kd[knum].x, "%s", kd[knum].txt); wattroff(wkeys,COLOR_PAIR(color)); wrefresh(wkeys); }
#define PERR(args...)                { sprintf(s, ## args); POUT(werr, "\n[%03.3f] %s", timestamp(), s); }
#define PLOG(args...)                { sprintf(s, ## args); POUT(wlog, "\n[%03.3f] %s", timestamp(), s); }
#define PSENT(arg)                   { mvwprintw(wcommands, 0, 24, "%s [%3.3f]", arg,  timestamp()); wrefresh(wcommands); }
#define PRECVD(arg)                  { mvwprintw(wcommands, 1, 24, "%s [%3.3f]", arg,  timestamp()); wrefresh(wcommands); }
//  
#define FLUSHER			     { while(getchar() != 0x0A); }
#define ERR_EXIT		     { if (fd_serial >= 0) close(fd_serial); fflush(stdin); exit(1); }

// List of known PSP commands
#define CMD_QUERY       0x02
#define CMD_INIT	0x80
#define CMD_ID		0x82
#define CMD_KEYS	0x84

// List of known PSP frame delimiters
#define FRAME_RTS       0xf0         // Request To Send = "I want to speak"
#define FRAME_CTS       0xf8         // Clear To Send = "Go ahead and speak"
#define FRAME_START     0xfd         // Message begins
#define FRAME_STOP      0xfe         // Message ends
#define FRAME_ACK0      0xfa         // Message received ok (phase 0)
#define FRAME_ACK1      0xfb         // Message received ok (phase 1)

// Current processing state
#define STATE_OFFLINE   0x00         // PSP is not powering up the serial port 
#define STATE_ONLINE    0x01         // PPS is powering serial port (2.5V on pin 5)
#define STATE_RESET     0x02         // We just went back on
#define STATE_RTS	0x04         // Request To Send has been received FROM the PSP
#define STATE_CTS       0x08         // Clear To Send has been received FROM the PSP
#define STATE_WAIT_ACK	0x10         // Pending ACK FROM the PSP (after message has been sent)

// Ncurses stuff
#define MAX_W      80                // Max horizontal width
// Our various windows height definition
#define STATUS_H    1
#define KEYS_H      3
#define COMMANDS_H  2
#define LOG_H      10
#define ERR_H       2
// Allows us to highlight the keys
typedef struct {
int x;
int y;
char* txt;
int timeout;
} ktxt;
ktxt kd[10];

// Definition of the commands we will enqueue 
typedef struct {
   u8 command;
   int size;
   u8 data[MAX_BYTES];
} R_Cmd;

// The command queue
R_Cmd cmd_table[16];
int cmd_pos = 0;
int cmd_end = 0;
char s[256];

// Buffer for sending data
u8 write_buffer[MAX_BYTES+3];

// Rotating buffer for input
u8 data_buffer[256];
u8 data_pos = 0;
u8 data_end = 0;

// Rotating buffer is full
int overflow = 0;

// Serial port file descriptor
int fd_serial = 0; 

// Commandline options
int opt_verbose;

// Current processing state
int state = STATE_OFFLINE;

// Sony's weird phase game
u8 inbound_phase = 0;
u8 outbound_phase = 0;

// The origin of times ;)
double t0;

// Keys flags
int keypressed = 0;
int hold = 0;

// ncurses windows
WINDOW *wstatus, *wkeys, *wcommands, *wlog, *werr;

// serial interrupt handler prototype
void serial_handler();

// An inline timestamping function would be better but we don't really care
double timestamp ()
{
struct timeval t;
     gettimeofday(&t, (struct timezone *)0);
     return t.tv_sec * 1.0 + (double)t.tv_usec / 1000000.0 - t0;
}
                  

/*
 *
 * Don't even think about suing!
 *
 */
int print_disclaimer()
{
	char c;
	puts("                       DISCLAIMER");
	puts("");
	puts("THIS PROGRAM IS PROVIDED \"AS IS\" WITHOUT WARRANTY OF ANY KIND,");
	puts("EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO,"); 
	puts("THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A");
	puts("PARTICULAR PURPOSE.");
	puts("");
	puts("THE ENTIRE RISK AS TO THE ABILITY OF THIS PROGRAM TO INTERFACE");
	puts("WITH THE REMOTE PORT OF A SONY PORTABLE PLAYSTATION (PSP) IS");
        puts("WITH YOU. SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE");
        puts("COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.");
	puts("");
	puts("IF YOU UNDERSTAND THE RISKS ASSOCIATED WITH THIS PROGRAM AND");
	puts("DISCHARGE BOTH THE AUTHOR AND SONY CORPORATION FROM ANY");
	puts("DAMAGE OCCURING AS THE RESULT OF ITS USE, PLEASE INDICATE SO");
	puts("BY ANSWERING THE FOLLOWING QUESTION:");
	puts("");
	puts("Do you understand and agree to the statement above (y/n)?");
        fflush(stdin);

	c = (char) getchar();
	FLUSHER;
	if ((c!='y') && (c!='Y'))
	{
		fprintf(stderr, "Operation cancelled by user.\n");
		return -1;
	}
	puts("");
	return 0;
}



/*
 * 
 * enqueue(): bufferize commands to send into a rotating command buffer
 *
 */
int enqueue(u8 command, char* strdata)
{
int i, j, len, size;
char c;
u8 nib = 0;	// init here prevents a warning there

    // Fill in the command
    cmd_table[cmd_end].command = command;

    // Init size
    size = 0;

    // We might as well do the hex -> bin conversion ourselves
    len = strlen(strdata);

    // Quick check
    if ((len%3) != 2)
    {
        PERR("data '%s' is not in format 'xx xx xx .. xx' (trailing space?)", strdata);
        return 1; 
    }

    // Do the hex -> nibble character conversion
    for (i=0; i<len; i+=3)
    {
        // Check that we have spaces as separators
        if ((i+2 < len) && (strdata[i+2] != ' '))
        {
            PERR("data '%s' is not in format 'xx xx xx ... xx'", strdata);
            return 1; 
        }
        
        // hex -> bin
        for (j=0; j<2; j++)
        {
            nib = nib<<4;   // a shift forces LSb to 0
            c = toupper(strdata[i+j]);		// read uppercase char
            if ((c>='0')&&(c<='9'))
                nib |= c&0x0f;
            else if ((c>='A')&&(c<='F'))
                nib |= c-0x37;
            else
            {
                PERR("Invalid hex character: %c", strdata[i]);
                return 1;
            }
        }

        // Write data into the command data table and increment size
        cmd_table[cmd_end].data[size++] = nib;
    }

    // Fill in with computed data size
    cmd_table[cmd_end].size = size;

    // Rotating buffer 
    cmd_end = (cmd_end+1) & 0x0F;

    return 0;
}


/*
 *
 * check_status(): Monitor serial port status
 *
 */
int check_status()
{
int serial_status;

    // Clear RESET flag if set
    if (state & STATE_RESET)
        state &= ~STATE_RESET;

    // Check for any change of RS232_CTS, to indicate whether the device is powered
    ioctl(fd_serial, TIOCMGET, &serial_status);
    if (serial_status & TIOCM_CTS)
    {   // RS323_CTS is on 
        if (!(state & STATE_ONLINE))
        {   // We just went back on
            state = STATE_ONLINE | STATE_RESET; 
            PSTATUS(4, "ONLINE ");

            // Reset data buffer
            data_pos = 0;
            data_end = 0;

            // Reset command buffer
            cmd_pos = 0;
            cmd_end = 0;

            // Enqueue init commands
            enqueue(CMD_INIT, "01 01 01");
            enqueue(CMD_ID, "01 A8 00 47");
        }
    }
    else
    {   // RS232_CTS is off => PSP has cut serial line power
        if (state & STATE_ONLINE)
        {   // We just went offline
            tcflush(fd_serial, TCIFLUSH);   // flush serial port
            PSTATUS(3, "OFFLINE"); 
        }
        state = STATE_OFFLINE;
    }

    // TO_DO: Check for break 

    return 0;
}


/*
 *
 * process_command: Process any inbound command from the PSP
 *
 */
int process_command(u8 command)
{
    // TO_DO: add a size check from associative array

    // Don't bother checking the inbound phase - just accept it
    inbound_phase = command & 0x01;

    switch(command & 0xfe)
    {
        case CMD_QUERY:
            PLOG("Received CMD_QUERY: %02X", data_buffer[data_pos]);
            PRECVD("CMD_QUERY");
            if (data_buffer[data_pos] == 0x01)
            {   // Only answer first time round
                PLOG("enqueue CMD_KEYS");
                enqueue(CMD_KEYS, "00 00");
            }
            data_pos+=2; // TODO: checksum
            if (data_buffer[data_pos++] != 0xfe)
                PERR("Error: missing FE frame end");
            return 0;
        default:
            PLOG("Received UNKNOWN COMMAND %02X", command);
            PRECVD("UNKNOWN");
            // /!\ THIS WILL NOT WORK IF THERE IS AN FE IN THE DATA
            while (data_buffer[data_pos] != 0xfe);
            return 0;
    }
}


/*
 *
 * read_data: incoming
 *
 */
int read_data()
{
u8 frame;
u8 command;

    if (data_pos != data_end)
    {
       frame = data_buffer[data_pos++];

       switch(frame)
       {
           // We are receiving a Request To Send from the PSP => Send Clear To Send
           case FRAME_RTS:
               PLOG("Received: FRAME_RTS");
               state |= STATE_RTS;
               write_buffer[0] = FRAME_CTS;
               if (write(fd_serial, write_buffer, 1) != 1)
                   PERR("Error Sending CTS");
               break;

           // We are receiving CTS on a previous RTS we sent
           case FRAME_CTS:
               PLOG("Received: FRAME_CTS");
               state |= STATE_CTS;
               break;

           // The PSP is ack'ing a previous command we sent
           case FRAME_ACK0:
           case FRAME_ACK1:
               PLOG("Received FRAME_ACK");
               if (!(state & STATE_WAIT_ACK))
                    PERR("Received ACK while not waiting for ACK!");
               state &= ~(STATE_WAIT_ACK | STATE_CTS);
               // Process next command
               cmd_pos = (cmd_pos+1) & 0x0f;
               if ((frame & 0x01) != outbound_phase)
                   PERR("Phase read from PSP on ack does not match our outbound phase!")
               else
                   // Toggle phase
                   outbound_phase = (outbound_phase)? 0:1;
               break;

           // The PSP is sending a command
           case FRAME_START:
               PLOG("FRAME_START");
               // Read command
               if (data_pos == data_end)
               {   // rest of data is not in yet. Try to wait for it
                   PERR("FRAME_START but no command - trying again");
                   usleep(MAX_BYTES*BYTE_DELAY);
                   if (data_pos == data_end)
                   {   // Still nothing
                       PERR("Frame start but no command.");
                       return -1;
                   }
               }
               // Isolate the command byte and process it
               command = data_buffer[data_pos++];
               process_command(command);

               // Acknowledge
               write_buffer[0] = FRAME_ACK0 | inbound_phase;
               if (write(fd_serial, write_buffer, 1) != 1)
                   PERR("Error writing ACK");
               
               state &= ~STATE_RTS;
               break;

           // Who knows...
           default:
               PLOG("Unknown Frame: %02X", frame);
               break;
               
        }
    }
    return 0;
}


/*
 *
 * write_data: outbound
 *
 */
int write_data()
{
u8 checksum;
int len;
int i;

   // Don't do anything if we're not online
   if (!(state & STATE_ONLINE))
       return 0;

   // Check if we have a command to send AND the PSP is not in the process of sending one to us
   // AND we are not waiting for a previous command to be acknowledged
   if ((cmd_pos != cmd_end) && (!(state & STATE_RTS)) && (!(state & STATE_WAIT_ACK)))
   {   // We are good to send
       // Did we receive CTS from PSP yet?
       if (state & STATE_CTS)
       {   
           // Send command 
           PLOG("Sending command %02X", cmd_table[cmd_pos].command);
           len = 0;
           // Frame start delimiter 
           write_buffer[len++] = FRAME_START;
           // We need to compute checksum too => init
           checksum = cmd_table[cmd_pos].command | outbound_phase;
           // Write command
           write_buffer[len++] = checksum;
           // Write data
           for (i=0; i<cmd_table[cmd_pos].size; i++)
           {
               write_buffer[len] = cmd_table[cmd_pos].data[i];
               checksum ^= write_buffer[len++];
           }
           // Write checksum
           write_buffer[len++] = checksum;
           // Frame stop delimiter
           write_buffer[len++] = FRAME_STOP;
           
           // Write to serial port
           if (write(fd_serial, write_buffer, len) != len)
               PERR("Error writing frame");
           // Change the state 
           state |= STATE_WAIT_ACK;
           state &= ~STATE_RTS;

           // Displays the command we just sent in the commands window
           switch(cmd_table[cmd_pos].command)
           {
               case CMD_INIT:
                   PSENT("CMD_INIT ");
                   break;
               case CMD_ID:
                   PSENT("CMD_ID   ");
                   break;
               case CMD_KEYS:
                   PSENT("CMD_KEYS ");
                   break;
               default:
                   PSENT("?????    ");
                   break;
           }
       }
       else
       {   // No CTS received yet => keep sending RTS
           write_buffer[0] = FRAME_RTS;
           if (write(fd_serial, write_buffer, 1) != 1)
              PERR("Error sending RTS");
       }
   }
   return 0;
}


/*
 *
 * ncurses init section
 *
 */
int init_screen()
{
int y = 0;
int i;

     // keys definition and positioning
     kd[0].x = 2;
     kd[0].y = 0;
     kd[0].txt = "0: Play/Pause";
     kd[1].x = 18;
     kd[1].y = 0;
     kd[1].txt = "1: [0x0002]";
     kd[2].x = 32;
     kd[2].y = 0;
     kd[2].txt = "2: Fast Forward";
     kd[3].x = 50;
     kd[3].y = 0;
     kd[3].txt = "3: Rewind";
     kd[4].x = 64;
     kd[4].y = 0;
     kd[4].txt = "4: Vol +";
     kd[5].x = 2;
     kd[5].y = 2;
     kd[5].txt = "5: Vol -";
     kd[6].x = 18;
     kd[6].y = 2;
     kd[6].txt = "6: [0x0040]";
     kd[7].x = 32;
     kd[7].y = 2;
     kd[7].txt = "7: Hold";
     kd[8].x = 50;
     kd[8].y = 2;
     kd[8].txt = "8: [0x0100]";
     kd[9].x = 64;
     kd[9].y = 2;
     kd[9].txt = "9: [0x0200]";

     // ncurses init
     initscr();
     if(has_colors() == FALSE)
     {       endwin();
             printf("Your terminal does not support color\n");
             exit(1);
     }
     cbreak();
     noecho();
     nonl();
     curs_set(0);
     keypad(stdscr, FALSE);    // this allows numpad entry as well
     timeout(KB_DELAY);

     // Use colour for the keys
     start_color();
     init_pair(1, COLOR_BLUE, COLOR_WHITE);
     init_pair(2, COLOR_BLUE, COLOR_GREEN);
     init_pair(3, COLOR_BLACK, COLOR_RED);
     init_pair(4, COLOR_BLACK, COLOR_GREEN);
     
     // Draw the various windows and boxes

     // Top border
     mvaddch(y, 0, ACS_ULCORNER);
     mvhline(y, 1, ACS_HLINE, MAX_W-2);
     mvaddch(y, MAX_W-1, ACS_URCORNER);
     mvaddch(y, 51, ACS_TTEE);
     y++;
     
     // Status window
     wstatus = newwin(STATUS_H, 40, y, 3);
     mvvline(y, 0, ACS_VLINE, STATUS_H);
     mvvline(y, MAX_W-1, ACS_VLINE, STATUS_H);
     mvvline(y, 51, ACS_VLINE, STATUS_H);
     y+=STATUS_H;

     // Separator
     mvaddch(y, 0, ACS_LTEE);
     mvaddch(y, MAX_W-1, ACS_RTEE);
     mvhline(y, 1, ACS_HLINE, MAX_W-2);
     mvaddch(y, 51, ACS_BTEE);
     y++;
 
     // Keys description window
     wkeys = newwin(KEYS_H, MAX_W-2, y, 1);
     mvvline(y, 0, ACS_VLINE, KEYS_H);
     mvvline(y, MAX_W-1, ACS_VLINE, KEYS_H);
     y+=KEYS_H;

     // Separator
     mvaddch(y, 0, ACS_LTEE);
     mvaddch(y, MAX_W-1, ACS_RTEE);
     mvhline(y, 1, ACS_HLINE, MAX_W-2);
     y++;

     // Commands window
     wcommands = newwin(COMMANDS_H, MAX_W-4, y, 3);
     mvvline(y, 0, ACS_VLINE, COMMANDS_H);
     mvvline(y, MAX_W-1, ACS_VLINE, COMMANDS_H);
     y+=COMMANDS_H;

     // Separator
     mvaddch(y, 0, ACS_LTEE);
     mvaddch(y, MAX_W-1, ACS_RTEE);
     mvhline(y, 1, ACS_HLINE, MAX_W-2);
     y++;

     // Log window
     wlog = newwin(LOG_H, MAX_W-4, y, 3);
     mvvline(y, 0, ACS_VLINE, LOG_H);
     mvvline(y, MAX_W-1, ACS_VLINE, LOG_H);
     y+=LOG_H;

     // Separator
     mvaddch(y, 0, ACS_LTEE);
     mvaddch(y, MAX_W-1, ACS_RTEE);
     mvhline(y, 1, ACS_HLINE, MAX_W-2);
     y++;

     // Error window
     werr = newwin(ERR_H, MAX_W-4, y, 3);
     mvvline(y, 0, ACS_VLINE, ERR_H);
     mvvline(y, MAX_W-1, ACS_VLINE, ERR_H);
     y+=ERR_H;

     // Bottom border
     mvaddch(y, 0, ACS_LLCORNER);
     mvhline(y, 1, ACS_HLINE, MAX_W-2);
     mvaddch(y, MAX_W-1, ACS_LRCORNER);
     y++;

     // Some of these windows have to scroll
     scrollok(wlog,TRUE);
     scrollok(werr,TRUE);
    
     // Draw the whole thing
     refresh();
     mvwprintw(stdscr, 1, 56, "Press <Esc> to exit");

     // Populate the status window
     mvwprintw(wstatus, 0, 0, "PSP serial port:"); 
     PSTATUS(3, "OFFLINE");

     // Populate the keys window
     wattron(wkeys,COLOR_PAIR(1));
     for (i=0; i<10; i++)
         mvwprintw(wkeys, kd[i].y, kd[i].x, "%s", kd[i].txt);
     wattroff(wkeys,COLOR_PAIR(1));
     wrefresh(wkeys);

     // Populate the commands window
     mvwprintw(wcommands, 0, 0, "Last command sent     :");
     mvwprintw(wcommands, 1, 0, "Last command received :");
     wrefresh(wcommands);
  
     return 0;
}
 

/*
 *
 * ncurses keyboard processing
 *
 */
int process_keyboard()
{
int ch,num;  
char keys[8];
u16 keyval;

     ch = getch();         // This is where KB_DELAY applies
     // Test for Esc key
     if (ch == 0x1B)
        return -1;
     // Test for a numeric key
     if ( (ch != ERR) && ((ch >= 0x30) && (ch <= 0x39)) )
     {
         num = ch&0x0f;
         keyval = (1<<num);
         keypressed = -1;
         PKEYS(2, num);
         kd[num].timeout = KEY_TIMEOUT;
         sprintf(keys, "%02X %02X", (u8)keyval, (u8)(keyval>>8));
         PLOG("enqueuing CMD_KEYS: %s", keys);
         enqueue(CMD_KEYS,keys);
     }
     // Send the key depress command
     else if (keypressed)
     {
         if (opt_verbose)
            PLOG("enqueuing CMD_KEYS: 00 00 (key depressed)");
         enqueue(CMD_KEYS,"00 00");
         keypressed = 0;
     }
     for (num=0; num<10; num++)
     {
         if (kd[num].timeout != -1)
         {
             kd[num].timeout -= KB_DELAY;
             if (kd[num].timeout < 0)
             {
                 PKEYS(1, num);
                 kd[num].timeout = -1;
             }
         }
     }
     return 0;
}


/*
 *
 *
 *
 */
int main (int argc, char *argv[])
{
struct timeval tm;
struct termios tty;             // will be used for new port settings
struct termios oldtty;          // will be used to save old port settings
struct sigaction saio;          // serial interrupt
char devname[NAME_SIZE] = DEFAULT_DEV;
int quit = 0; 
int opt_error = 0;	// getopt
int i;

     fflush(stdin);

     while ((i = getopt (argc, argv, "hv")) != -1)
     switch (i)
     {
		case 'v':		// Print verbose messages
			opt_verbose++;
			break;
		case 'h':
		default:		// Unknown option
			opt_error++;
			break;
     }

     printf ("\npsp_remote v1.00 : Sony PSP, serial software remote\n");
     printf ("by >NIL:, July 2005\n\n");

     if ( ((argc-optind) > 1) || (opt_error) )
     {
         printf ("usage: psp_emote [-v] [device]\n");
         printf ("If no device is given, psp_remote will use %s\n", DEFAULT_DEV);
         printf ("Options:\n");
         printf ("                -v : verbose\n\n");
         exit (1);
     }

     if (argv[optind] != NULL)
     {
         strncpy (devname, argv[optind], NAME_SIZE);
         devname[NAME_SIZE-1] = 0;
     }

     fd_serial = open(devname, O_RDWR | O_NOCTTY | O_NONBLOCK);
     if (fd_serial < 0) {
          printf("\nUnable to open serial port (%s), are you root?\n", devname);
          ERR_EXIT;
     }

     // Who wants a disclaimer?
     if (print_disclaimer())
         ERR_EXIT;

     tcgetattr(fd_serial, &oldtty);  // save current port settings
     bzero(&tty, sizeof(tty));       // Initialize the port settings structure to all zeros
     tty.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;      // 8N1
     tty.c_iflag = IGNPAR;
     tty.c_oflag = 0;
     tty.c_lflag = 0;
     tty.c_cc[VMIN] = 0;             // 0 means use-vtime
     tty.c_cc[VTIME] = 1;            // time to wait until exiting read (tenths of a second)

     tcflush(fd_serial, TCIFLUSH);   // flush old data and apply new settings
     tcsetattr(fd_serial, TCSANOW, &tty); 
     fcntl(fd_serial, F_SETOWN, getpid());    // enable our PID to receive serial interrupts
     fcntl(fd_serial, F_SETFL, FASYNC);
     
     
     saio.sa_handler = serial_handler;  // set the serial interrupt handler  
     sigemptyset(&saio.sa_mask);        // clear existing settings
     saio.sa_flags = 0;                 // make sure sa_flags is cleared
     saio.sa_restorer = NULL;           // no restorer
     sigaction(SIGIO, &saio, NULL);     // apply new settings

     // ncurses init
     init_screen();

     // Set time origin (for timestamping)
     gettimeofday(&tm, (struct timezone *)0);
     t0 = (double)tm.tv_sec * 1.0 + (double)tm.tv_usec / 1000000.0;

     // Flag indicating that no key is pressed
     keypressed = 0;
   
     // main processing loop
     while (!quit) { 
          // Update RS232 line status
          check_status();
          // Process inbound and outbound data
          read_data();
          write_data();
          // Process keyboard
          quit = process_keyboard();
     }

     // restore the old port settings before quitting
     tcsetattr(fd_serial, TCSANOW, &oldtty); 
     close(fd_serial);

     // Quit ncurses mode
     endwin(); 

     exit(0);
}


/*
 *
 * serial_handler() : called whenever there is inboud data to process
 *
 */
void serial_handler () {
u8 temp_buffer[256];
int len, i;

    // Read the serial data into a temporary buffer
    len = read(fd_serial, temp_buffer, sizeof(temp_buffer));

    // Check that we have space to store it
    if ((data_pos != data_end) && (len >= ((data_pos-data_end)&0xff)))
    {   // We are not addressing overflow for now, just flagging them
        printf("Overflow!\n");
        overflow = -1;
    }
   
    // Copy data into rotating buffer
    for (i=0;i<len;i++)
        data_buffer[data_end++] = temp_buffer[i];
}

