/* This program simulates communications of two wireless traffic lights at a
 * crossroad. Two devices with microcontrollers MSP430 are required in order 
 * to run this program correctly, otherwise Picomp simulator is an alternative.
 *
 * Author: Zhengyi Liang
 *
 * Course: CMPT364 Assignment 2
 * Date: Feb.15th 2012
 */
#include "sysio.h"
#include "tcvphys.h"
#include "phys_cc1100.h"
#include "plug_null.h"
#include "ser.h"
#include "form.h"

int fd = -1;
int sid = 0;
int channel = 0;
int power = 0;
int size = 20;
int myRoll;
int waitTime = 1024;
int myLight = -1;
int switched = 0;
int resumeTimer = -1;
int lcPID = 0;
int rePID = 0;
char role = NULL;
char boss = 'B';
char listener = 'L';
char *outBuf, *inBuf, *bufCopy = NULL;
boolean triggerSend = NO;
boolean greenLight = NO;
boolean sosMode = NO;
boolean disconnected = true;

#define TRIGGERSEND       (&triggerSend)
#define GREEN_ON          (&greenLight)
#define SOSMODE           (&sosMode)
#define DISCONNECTED      (&disconnected)
#define GREEN             1
#define RED               0
#define YELLOW            2
#define BLINK             2
#define ON                1
#define OFF               0
#define R_ON              leds (RED, ON)
#define R_OFF             leds (RED, OFF)
#define G_ON              leds (GREEN, ON)
#define G_OFF             leds (GREEN, OFF)
#define Y_ON              leds (YELLOW, ON)
#define Y_OFF             leds (YELLOW, OFF)
#define FOUR_WAY_STOP     leds (RED, BLINK)
#define LIGHTS_OFF        R_OFF; G_OFF; Y_OFF
#define GREEN_LIGHT_DURATION  10240
#define TIME_BEFORE_GREEN 1024
#define WAIT_TIME         512
#define RED_MSG           "red"
#define GREEN_MSG         "green"
#define PREFIX            "yi"
#define SOS_MSG           "SOS"

/*
 * light controller for when emergency vehicle is closing
 * in
 */
fsm sosController {
  initial state STARTOVER:
    delay (WAIT_TIME, SOSOVER);    /* time out when receives no sos signal */
    when (SOSMODE, STARTOVER);     /* start over if keeps receiving sos signal */

  state SOSOVER:
    /* resumes the previous buffer the device is sending out and send it out immediately */
    form (outBuf, "%s", bufCopy);
    triggerSend = true;
    trigger (TRIGGERSEND);
    /* free the buffer and set the pointer to null */
    free (bufCopy);
    /* trigger not sos mode to resume the normal light controller */
    bufCopy = NULL;
    sosMode = NO;

    finish;
}

/* regular light controller */
fsm lightController {
  initial state STARTUP:
    /* start the count down when the green light is on or is disconnected */
    greenLight = NO;
    when (GREEN_ON, COUNTDOWN);
    when (DISCONNECTED, PAUSESTATE);
    release;

  state COUNTDOWN:
    /* start counting down the given time */
    myLight = GREEN;
    delay (resumeTimer, REDLIGHT);
    when (DISCONNECTED, PAUSESTATE);
    release;

  state REDLIGHT:
    /* change the lights and roles */
    role = listener;
    LIGHTS_OFF;
    R_ON;
    delay (TIME_BEFORE_GREEN, SENDGREEN);
    release;

  state SENDGREEN:
    /* setup the buffer to out put */
    form (outBuf + 4, "%s", GREEN_MSG);
    triggerSend = true;
    trigger (TRIGGERSEND);
    resumeTimer = GREEN_LIGHT_DURATION;
    proceed (STARTUP);

  state PAUSESTATE:
    /* set the lights to four way stop blinking mode and wait for unblock signal */
    LIGHTS_OFF;
    myLight = -1;
    role = listener;
    FOUR_WAY_STOP;
    when (!DISCONNECTED, STARTUP);
    release;
}

/*
 * receiver that reacts to the receiving signal
 */
fsm receiver {
  address packet;
  int roll;
  int delayTime = WAIT_TIME * 4;

  initial state RECEIVING:
    /* setup a time out */
    delay (delayTime, WAITSTATE);
    packet = tcv_rnp (RECEIVING, fd);
    tcv_read (packet, inBuf, size);
    tcv_endp (packet);
    proceed (RECEIVED);
    release;

  state RECEIVED:
    if (strncmp(inBuf + 2, PREFIX, 2) == 0) {
      /* if receives the role determines by the other device, set the role */
      if (strncmp(inBuf + 4, GREEN_MSG, 5) == 0) {
	if (((role != boss && myLight != GREEN) ||
	    (role == listener && myLight != GREEN)) &&
	    sosMode != true) {
	  form (outBuf + 4, "%s", RED_MSG);
	  role = boss;
	  LIGHTS_OFF;
	  myLight = GREEN;
	  leds (myLight, ON);
	}
      }
      /* if receives a red light signal, change the role to the listener */
      else if (strncmp(inBuf + 4, RED_MSG, 3) == 0) {
	if (((role == listener && myLight != RED) ||
	     role == NULL) && sosMode != true) {
	  role = listener;
	  form (outBuf + 4, "%s", GREEN_MSG);
	  myLight = RED;
	  LIGHTS_OFF;
	  leds (myLight, ON);
	}
      }
      /* read off the roll from the other device */
      else if ((roll = atoi(inBuf + 4)) != 0) {
	if (myRoll != roll) {
	  if (myRoll > roll) {
	    form (outBuf + 4, "%s", RED_MSG);
	  }
	  else {
	    role = listener;
	    myLight = RED;
	    leds (myLight, ON);
	    form (outBuf + 4, "%s", GREEN_MSG);
	    greenLight = NO;
	  }
	}
	/* re-roll if the two numbers are the same */
	else {
	  myRoll = rnd() % 89 + 10;
	  form (outBuf + 4, "%d", myRoll);
	  triggerSend = true;
	  trigger (TRIGGERSEND);
	}
      }
      /* set the disconnected trigger to no */
      if (disconnected != NO) {
	disconnected = NO;
	trigger (!DISCONNECTED);
      }
      /* resets the resumeTimer if there is a new connection */
      if (resumeTimer < 0 || resumeTimer == MAX_UINT)
	resumeTimer = GREEN_LIGHT_DURATION;
      
      if (role == boss && myLight == GREEN) {
	/* trigger the green light duration count down */
	greenLight = true;
	trigger (GREEN_ON);
      }
      /* if there is a legitimate packet */
      if (delayTime != WAIT_TIME * 4)
	delayTime = WAIT_TIME * 4;
    }
    /* if an emergency viechel comes by */
    else if (strncmp(inBuf + 2, SOS_MSG, 3) == 0) {
      sosMode = true;    /* set the sosMode immediately to prevent receiving another green/red signal */

      if (!running(sosController))  /* run the controller if not already started*/
	runfsm sosController;
      
      trigger (SOSMODE);    /* keep the sosmode triggered */
      
      if (bufCopy == NULL)
	form (bufCopy, "%s", outBuf);  /* backup what's originally in the out buffer */
      
      /* put SOS message in the buffer and send the message immediately */
      form (outBuf + 4, "%s", SOS_MSG);  
      triggerSend = true;
      trigger (TRIGGERSEND);
      /* SOS is considered a valid packet */
      if (delayTime != WAIT_TIME *4)
	delayTime = WAIT_TIME * 4;
      
      proceed (WAITSTATE);
    }
    else {
      /* set the delay time to whatever is left if the packet is not legitimate */
      delayTime = dleft(rePID);
    }
    
    proceed (RECEIVING);
    
  /* the state that triggers disconnected flag */
  state WAITSTATE:
    if (disconnected != true) {
      resumeTimer = dleft (lcPID);    /* get the remaining green light duration */
      
      if (resumeTimer == MAX_UINT)
	resumeTimer = GREEN_LIGHT_DURATION;    /* if the remaining time is not available */
      disconnected = true;
      trigger (DISCONNECTED);
      }
    proceed (RECEIVING);
}

fsm sender {
  address packet;
  initial state WAITFORSEND:
    /* sends signals at a certain rate or when triggered */
    delay (waitTime, SENDSIGNAL);
    when (TRIGGERSEND, SENDSIGNAL);
    release;

  state SENDSIGNAL:
    /* fill in the packet */
    packet = tcv_wnp(SENDSIGNAL, fd, size);
    tcv_write(packet, outBuf, size - 2);
    tcv_endp (packet);

  state SENT:
    /* turn off the trigger */
    triggerSend = NO;
    proceed (WAITFORSEND);
}

/*
 * main finite state machine
 */
fsm root {
  initial state SETUP:
    outBuf = (char*) umalloc (size);
    inBuf = (char*) umalloc (size);
    /* check malloc failure */
    if (outBuf == NULL || inBuf == NULL) {
      diag("umalloc failed");
      finish;
    }
    /* initialize the buffer with 0's */
    bzero (outBuf, size);
    /* and then put useful information in */
    form (outBuf + 2, PREFIX, 2);
    myRoll = rnd() % 89 + 10;
    form (outBuf + 4, "%d", myRoll);
    /* setup network */
    phys_cc1100 (0, size);
    tcv_plug (0, &plug_null);
    fd = tcv_open(NONE, 0, 0); 
    if (fd < 0) {
      diag("get file descriptor failed");
      finish;
    }

    tcv_control (fd, PHYSOPT_SETSID, (address) &sid);
    tcv_control (fd, PHYSOPT_SETCHANNEL,(address) &channel);
    tcv_control (fd, PHYSOPT_SETPOWER, (address) &power);
    tcv_control (fd, PHYSOPT_TXON, NULL);
    tcv_control (fd, PHYSOPT_RXON, NULL);
    
    /* run the light controller machine */
    lcPID = runfsm lightController;
    /* run the receiving machin */
    runfsm receiver;
    
    /* put them into four way stop blinking mode */
    trigger (DISCONNECTED);
    /* randomize the starting time for sender */
    delay (rnd() % WAIT_TIME * 4, RUNSENDER);
    release;

  state RUNSENDER:
    runfsm sender;
}
