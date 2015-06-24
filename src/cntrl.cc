/*
 * Dragoș Niculescu dragos.niculescu@cs.pub.ro 2015
 *
 *
 */

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <math.h>
#include <iostream>
#include "roombalib.h"
#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"

using namespace qrk;
using namespace std;

int main(int nargs, char *argv[]){

  // apel la biblioteca liburg 
  Connection_information information(nargs, argv);
  Urg_driver urg;
  if (!urg.open(information.device_or_ip_name(),
		information.baudrate_or_port_number(),
		information.connection_type())) {
    cout << "Urg_driver::open(): "
	 << information.device_or_ip_name() << ": " << urg.what() << endl;
    return 1;
  }

  // apel la biblioteca libroomba
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
