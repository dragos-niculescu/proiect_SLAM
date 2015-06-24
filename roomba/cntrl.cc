/*
 * Dragos Niculescu dragos@nec-labs.com
 *
 * Based on code by Tod E. Kurt, tod@todbot.com
 *
 */

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <math.h>
#include "roombalib.h"

int main(int nargs, char *argv[]){
	assert(roomba_init( "/dev/ttyUSB0" ));
        int  s, d;
	int c; //contor
	c=0;
	d=0;
	roomba_delay(1000);
	while(c<1){
	        s=0xff;
	        roomba_drive2(s, s);
//		roomba_wait_distance(200);
		roomba_read_sensors();
		if(bump_left(roomba.sensor_bytes[0]) || bump_right(roomba.sensor_bytes[0])){
			roomba_stop();
			roomba_drive2(-s, -s);
			roomba_wait_distance(-50);
			roomba_spinright();
			roomba_wait_angle(-180);
			}
		c=0;
		}
//	printf("%d\n", d);
	exit(0);
}
