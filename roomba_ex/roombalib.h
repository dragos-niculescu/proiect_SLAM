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


#include <stdint.h>   /* Standard types */

#define DEFAULT_VELOCITY 200
#define COMMANDPAUSE_MILLIS 10
//#define COMMANDPAUSE_MILLIS 10

// holds all the per-roomba info
// consider it an opaque blob, please
struct roomba_struct {
    int fd;
    char portpath[80];
    uint8_t sensor_bytes[26];
    int velocity;
};

// set to non-zero to see debugging output
extern int roombadebug;
extern struct roomba_struct roomba; 

// given a serial port name, create a Roomba object and return it
// or return NULL on error
int roomba_init( const char* portname );

// frees the memory of the Roomba object created with roomba_init
// will close the serial port if it's open
void roomba_free(  );

// close the serial port connected to the Roomba
void roomba_close(  );

// is this Roomba pointer valid (but not necc connected)
int roomba_valid(  );

// return the serial port path for the given roomba
const char* roomba_get_portpath(  );

// send an arbitrary length roomba command
int roomba_send( const uint8_t* cmd, int len );

// Move Roomba with low-level DRIVE command
void roomba_drive(  int velocity, int radius );
int roomba_drive2( int rspeed, int lspeed); 

// stop the Roomba 
void roomba_stop(  );

// Move Roomba forward at current velocity
void roomba_forward(  );
void roomba_forward_at(  int velocity );

// Move Roomba backward at current velocity
void roomba_backward(  );
void roomba_backward_at(  int velocity );

// Spin Roomba left at current velocity
void roomba_spinleft(  );
void roomba_spinleft_at( int velocity );

// Spin Roomba right at current velocity
void roomba_spinright(  );
void roomba_spinright_at(  int velocity );

// Set current velocity for higher-level movement commands
void roomba_set_velocity(  int velocity );

// Get current velocity for higher-level movement commands
int roomba_get_velocity(  );

// play a musical note
void roomba_play_note( uint8_t note, uint8_t duration );

// Turns on/off the non-drive motors (main brush, vacuum, sidebrush).
void roomba_set_motors( uint8_t mainbrush, uint8_t vacuum, uint8_t sidebrush);

// Turns on/off the various LEDs.
void roomba_set_leds(uint8_t status_green, uint8_t status_red,
                      uint8_t spot, uint8_t clean, uint8_t max, uint8_t dirt, 
                      uint8_t power_color, uint8_t power_intensity );

// Turn all vacuum motors on or off according to state
void roomba_vacuum(uint8_t state );

// Get the sensor data from the Roomba
// returns -1 on failure
int roomba_read_sensors(  );

// print existing sensor data nicely
void roomba_print_sensors(  );

// print existing sensor data as string of hex chars
void roomba_print_raw_sensors(  );

uint16_t roomba_read_ADC( );
float read_sonar_mm(); 
float read_sonar_in(); 
void roomba_wait_distance(int distance ); // milimeters
void roomba_wait_angle(int angle ); // degrees
int16_t roomba_read_distance(); 
int16_t roomba_read_angle(); 
int roomba_start(); 
uint8_t roomba_read_IR(); 

// utility function
void roomba_delay( int millisecs );
#define roomba_wait roomba_delay

// some simple macros of bit manipulations
#define bump_right(b)           ((b & 0x01)!=0)
#define bump_left(b)            ((b & 0x02)!=0)
#define wheeldrop_right(b)      ((b & 0x04)!=0)
#define wheeldrop_left(b)       ((b & 0x08)!=0)
#define wheeldrop_caster(b)     ((b & 0x10)!=0)

#define motorover_sidebrush(b)  ((b & 0x01)!=0) 
#define motorover_vacuum(b)     ((b & 0x02)!=0) 
#define motorover_mainbrush(b)  ((b & 0x04)!=0) 
#define motorover_driveright(b) ((b & 0x08)!=0) 
#define motorover_driveleft(b)  ((b & 0x10)!=0) 

#define IR_LEFT 129
#define IR_FORWARD 130
#define IR_RIGHT 131
#define IR_SPOT 132
#define IR_MAX 133
#define IR_SMALL 134
#define IR_MEDIUM 135
#define IR_LARGE 136
#define IR_CLEAN 136
#define IR_PAUSE 137
#define IR_POWER 138
#define IR_RESVD 240
#define IR_REDBUOY 248
#define IR_GREENBUOY 244
#define IR_FORCE 242
#define IR_REDGREEN 252
#define IR_REDFORCE 250
#define IR_GREENFORCE 246
#define IR_REDGREENFORCE 254

//void usleep(unsigned long usec);
