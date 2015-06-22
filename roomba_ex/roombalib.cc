/*
 * roombalib -- Roomba C API
 *
 * http://hackingroomba.com/
 *
 * Copyright (C) 2006, Tod E. Kurt, tod@todbot.com
 *
 * Updates:
 * 14 Dec 2006 - added more functions to roombalib
 */


#include <stdio.h>    /* Standard input/output definitions */
#include <stdint.h>   /* Standard types */
#include <stdlib.h>   /* calloc, strtol */
#include <string.h>   /* strcpy */
//#define __USE_MISC
# include <termios.h>  /* POSIX terminal control definitions */
//#undef __USE_MISC
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */


#include "roombalib.h"

int roombadebug = 0;
struct roomba_struct roomba; 

// internal use only
int roomba_init_serialport( const char* serialport, speed_t baud );

int roomba_init( const char* portpath ) 
{
    int fd = roomba_init_serialport( portpath, B57600 );
  //    int fd = roomba_init_serialport( portpath, 9600 );
    if( fd == -1 ) { 
      fprintf(stderr, "failed to init serial port ");
      return 0;
    }
    uint8_t cmd[1];
    
    cmd[0] = 128;      // START
    int n = write(fd, cmd, 1);
    if( n!=1 ) {
        perror("open_port: Unable to write to port ");
        return 0;
    }
    roomba_delay(COMMANDPAUSE_MILLIS);
    
    cmd[0] = 130;   // CONTROL
    n = write(fd, cmd, 1);
    if( n!=1 ) {
        perror("open_port: Unable to write to port ");
        return 0;
    }
    roomba_delay(COMMANDPAUSE_MILLIS);
    
    roomba.fd = fd;
    strcpy(roomba.portpath, portpath);
    roomba.velocity = DEFAULT_VELOCITY;

    return 1;
}


void roomba_free() 
{
  if( roomba.fd ) 
    roomba_close( );
}

const char* roomba_get_portpath(  )
{
    return roomba.portpath;
}

void roomba_close(  )
{
    close( roomba.fd );
    roomba.fd = 0;
}

void roomba_set_velocity( int vel )
{
    roomba.velocity = vel;
}

int roomba_get_velocity(  )
{
    return roomba.velocity;
}

// send an arbitrary length roomba command
int roomba_xfer( const uint8_t cmd[], int clen, uint8_t*buf, int blen )
{
  if(clen != write( roomba.fd, cmd, clen)){
    perror("roomba_xfer: couldn't write roomba");
    return 1; 
  } 
  if(blen != 0 && blen != read( roomba.fd, buf, blen)){ 
    perror("roomba_xfer: couldn't read roomba");
    return 1; 
  } 
  return 0; // success  
}

int roomba_start()
{
  uint8_t start[] = {128}; 
  return roomba_xfer(start, 1, NULL, 0); 
}

uint8_t roomba_read_IR()
{
  uint8_t cmd[] = { 142, 17 }, val;          // SENSOR, get IR byte
  roomba_xfer(cmd, 2, &val, 1);
  return val; 
}

int roomba_drive2(int rspeed, int lspeed)
{         // left speed, right speed 
  uint8_t cmd[] = { 145, rspeed >> 8,  rspeed & 0xff, lspeed >> 8, lspeed & 0xff };  
  return roomba_xfer(cmd, 5, NULL, 0);
}

// send an arbitrary length roomba command
int roomba_send(const uint8_t* cmd, int len )
{
    int n = write( roomba.fd, cmd, len);
    if( n!=len )
        perror("roomba_send: couldn't write to roomba");
    return (n!=len); // indicate error, can usually ignore
}


// Move Roomba with low-level DRIVE command
void roomba_drive( int velocity, int radius )
{
    uint8_t vhi = velocity >> 8;
    uint8_t vlo = velocity & 0xff;
    uint8_t rhi = radius   >> 8;
    uint8_t rlo = radius   & 0xff;
    if(roombadebug) 
        fprintf(stderr,"roomba_drive: %.2hhx %.2hhx %.2hhx %.2hhx\n",
                vhi,vlo,rhi,rlo);
    uint8_t cmd[] = { 137, vhi,vlo, rhi,rlo };  // DRIVE
    int n = write(roomba.fd, cmd, 5);
    if( n!=5 )
        perror("roomba_drive: couldn't write to roomba");
}

void roomba_stop(  )
{
    roomba_drive( 0, 0 );
}

void roomba_forward(  )
{
    roomba_drive( roomba.velocity, 0x8000 );  // 0x8000 = straight
}
void roomba_forward_at( int velocity )
{
    roomba_drive( velocity, 0x8000 );
}

void roomba_backward(  )
{
    roomba_drive(-roomba.velocity, 0x8000 );
}
void roomba_backward_at( int velocity )
{
    roomba_drive( -velocity, 0x8000 );
}

void roomba_spinleft(  )
{
    roomba_drive( roomba.velocity, 1 );
}
void roomba_spinleft_at(int velocity )
{
    roomba_drive( velocity, 1 );
}

void roomba_spinright(  )
{
    roomba_drive(roomba.velocity,  -1 );
}
void roomba_spinright_at(int velocity )
{
    roomba_drive(velocity,  -1 );
}

void roomba_play_note(uint8_t note, uint8_t duration ) 
{
    uint8_t cmd[] = { 140, 15, 1, note, duration, // SONG, then
                      141, 15 };                  // PLAY
    int n = write( roomba.fd, cmd, 7);
    if( n!=7 )
        perror("roomba_play_note: couldn't write to roomba");
}

// Turns on/off the non-drive motors (main brush, vacuum, sidebrush).
void roomba_set_motors(uint8_t mainbrush, uint8_t vacuum, uint8_t sidebrush)
{
    uint8_t cmd[] = { 138,                        // MOTORS
                      ((mainbrush?0x04:0)|(vacuum?0x02:0)|(sidebrush?0x01:0))};
    int n = write( roomba.fd, cmd, 2);
    if( n!=2 )
        perror("roomba_set_motors: couldn't write to roomba");
}

// Turns on/off the various LEDs.
void roomba_set_leds( uint8_t status_green, uint8_t status_red,
                      uint8_t spot, uint8_t clean, uint8_t max, uint8_t dirt, 
                      uint8_t power_color, uint8_t power_intensity )
{
    uint8_t v = (status_green?0x20:0) | (status_red?0x10:0) |
                (spot?0x08:0) | (clean?0x04:0) | (max?0x02:0) | (dirt?0x01:0);
    uint8_t cmd[] = { 139, v, power_color, power_intensity }; // LEDS
    int n = write( roomba.fd, cmd, 4);
    if( n!=4 )
        perror("roomba_set_leds: couldn't write to roomba");
}

// Turn all vacuum motors on or off according to state
void roomba_vacuum( uint8_t state ) {
    roomba_set_motors( state,state,state);
}

uint16_t roomba_read_ADC()
{
   uint8_t cmd[] = { 142, 33 };          // SENSOR, pin 4 from cargo bay connector
   uint16_t val;   
    int n = write( roomba.fd, cmd, 2);
    if( n!=2 ) {
        if(roombadebug)
            fprintf(stderr,"roomba_read_ADC: not enough write (n=%d)\n",n);
        return -1;
    }
    //    roomba_delay(COMMANDPAUSE_MILLIS);  //hmm, why isn't VMIN & VTIME working?
    n = read( roomba.fd, &val, 2);
    if( n!=2 ) {
        if(roombadebug)
            fprintf(stderr,"roomba_read_ADC: not enough read (n=%d)\n",n);
        return -1;
    }
    return (val>>8) | ((val<<8)&0xff00);
}

int16_t roomba_read_angle()
{
   uint8_t cmd[] = { 142, 20 };          // angle rotated since last query
   uint16_t val;   
    int n = write( roomba.fd, cmd, 2);
    if( n!=2 ) {
        if(roombadebug)
            fprintf(stderr,"roomba_read_angle: not enough write (n=%d)\n",n);
        return -1;
    }
    //    roomba_delay(COMMANDPAUSE_MILLIS);  //hmm, why isn't VMIN & VTIME working?
    n = read( roomba.fd, &val, 2);
    if( n!=2 ) {
        if(roombadebug)
            fprintf(stderr,"roomba_read_angle: not enough read (n=%d)\n",n);
        return -1;
    }
    return (val>>8) | ((val<<8)&0xff00);
}

int16_t roomba_read_distance()
{
   uint8_t cmd[] = { 142, 19 };          // distance traveled (wheel1+wheel2)/2
   uint16_t val;   
    int n = write( roomba.fd, cmd, 2);
    if( n!=2 ) {
        if(roombadebug)
            fprintf(stderr,"roomba_read_distance: not enough write (n=%d)\n",n);
        return -1;
    }
    //    roomba_delay(COMMANDPAUSE_MILLIS);  //hmm, why isn't VMIN & VTIME working?
    n = read( roomba.fd, &val, 2);
    if( n!=2 ) {
        if(roombadebug)
            fprintf(stderr,"roomba_read_distance: not enough read (n=%d)\n",n);
        return -1;
    }
    return (val>>8) | ((val<<8)&0xff00);
}


uint32_t roomba_read_stream_ADC()
{
   uint8_t cmd[] = { 148, 33 };          // SENSOR, pin 4 from cargo bay connector
   uint16_t val;   
    int n = write( roomba.fd, cmd, 2);
    if( n!=2 ) {
        if(roombadebug)
            fprintf(stderr,"roomba_read_ADC: not enough write (n=%d)\n",n);
        return -1;
    }
    roomba_delay(COMMANDPAUSE_MILLIS);  //hmm, why isn't VMIN & VTIME working?
    n = read( roomba.fd, &val, 2);
    if( n!=2 ) {
        if(roombadebug)
            fprintf(stderr,"roomba_read_ADC: not enough read (n=%d)\n",n);
        return -1;
    }
    return (val>>8) | ((val<<8)&0xff00);
}

float read_sonar_mm()
{
  return roomba_read_ADC()*25.4*512/1023; // in mm 
}
float read_sonar_in()
{
  return roomba_read_ADC()*512/1023; // in inch 
}


int roomba_read_sensors(  )
{
    uint8_t cmd[] = { 142, 0 };          // SENSOR, get all sensor data
    int n = write( roomba.fd, cmd, 2);
    roomba_delay(COMMANDPAUSE_MILLIS);  //hmm, why isn't VMIN & VTIME working?
    n = read( roomba.fd, roomba.sensor_bytes, 26);
    if( n!=26 ) {
        if(roombadebug)
            fprintf(stderr,"roomba_read_sensors: not enough read (n=%d)\n",n);
        return -1;
    }
    return 0;
}

void roomba_print_raw_sensors(  )
{
    uint8_t* sb = roomba.sensor_bytes;
    int i;
    for(i=0;i<26;i++) {
        printf("%.2hhx ",sb[i]);
    }
    printf("\n");
}

void roomba_print_sensors(  )
{
    uint8_t* sb = roomba.sensor_bytes;
    printf("bump: %x %x\n", bump_left(sb[0]), bump_right(sb[0]));
    printf("wheeldrop: %x %x %x\n", wheeldrop_left(sb[0]),
           wheeldrop_caster(sb[0]), wheeldrop_right(sb[0]));
    printf("wall: %x\n", sb[1]);
    printf("cliff: %x %x %x %x\n", sb[2],sb[3],sb[4],sb[5] );
    printf("virtual_wall: %x\n", sb[6]);
    printf("motor_overcurrents: %x %x %x %x %x\n", motorover_driveleft(sb[7]),
           motorover_driveright(sb[7]), motorover_mainbrush(sb[7]),
           motorover_sidebrush(sb[7]),  motorover_vacuum(sb[7]));
    printf("dirt: %x %x\n", sb[8],sb[9]);
    printf("remote_opcode: %.2hhx\n", sb[10]);
    printf("buttons: %.2hhx\n", sb[11]);
    printf("distance: %.4x\n",    (sb[12]<<8) | sb[13] );
    printf("angle: %.4x\n",       (sb[14]<<8) | sb[15] );
    printf("charging_state: %.2hhx\n", sb[16]);
    printf("voltage: %d\n",     (sb[17]<<8) | sb[18] );
    printf("current: %d\n",     ((int8_t)sb[19]<<8) | sb[20] );
    printf("temperature: %d\n",    sb[21]);
    printf("charge: %d\n",      (sb[22]<<8) | sb[23] );
    printf("capacity: %d\n",    (sb[24]<<8) | sb[25] );
}

void roomba_delay( int millisecs )
{
    usleep( millisecs * 1000 );
}



// private
// returns valid fd, or -1 on error
int roomba_init_serialport( const char* serialport, speed_t baud)
{
    struct termios toptions;
    int fd;

    if(roombadebug)
        fprintf(stderr,"roomba_init_serialport: opening port %s\n",serialport);

    //    fd = open( serialport, O_RDWR | O_NOCTTY | O_NDELAY );
    fd = open( serialport, O_RDWR | O_NOCTTY );  // BLOCKING 
    if (fd == -1)  {     // Could not open the port.
        perror("roomba_init_serialport: Unable to open port ");
        return -1;
    }
    
    if (tcgetattr(fd, &toptions) < 0) {
        perror("roomba_init_serialport: Couldn't get term attributes");
        return -1;
    }
    
    cfsetispeed(&toptions, baud);
    cfsetospeed(&toptions, baud);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
#define CRTSCTS  020000000000 
    toptions.c_cflag &= ~CRTSCTS;

    toptions.c_cflag    |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag    &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag    &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag    &= ~OPOST; // make raw

    toptions.c_cc[VMIN]  = 26;
    toptions.c_cc[VTIME] = 2;           // FIXME: not sure about this
    
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("roomba_init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}





void roomba_wait_distance(int distance ) // milimeters
{
    uint8_t dhi = (distance >> 8) & 0xff;
    uint8_t dlo = distance & 0xff;
    if(roombadebug) 
      fprintf(stderr,"wait distance: %.2hhx %.2hhx\n",	      dhi,dlo);
    uint8_t cmd[] = { 156, dhi,dlo };  
    int n = write(roomba.fd, cmd, 3);
    if( n != 3 )
      perror("wait_distance: couldn't write to roomba");
}


void roomba_wait_angle(int angle ) // degrees
{
    uint8_t dhi = (angle >> 8) & 0xff;
    uint8_t dlo = angle & 0xff;
    if(roombadebug) 
      fprintf(stderr,"wait angle: %.2hhx %.2hhx\n",	      dhi,dlo);
    uint8_t cmd[] = { 157, dhi,dlo };  
    int n = write(roomba.fd, cmd, 3);
    if( n != 3 )
      perror("wait_angle: couldn't write to roomba");
}
