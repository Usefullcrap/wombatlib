#include <kipr/wombat.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#define handlingLogic1() { \
    moves[length++] = (char)c; \
    break; \
}
/*
	Created by Matteo Tang at a reasonable time of day. WIP. 7/20/2026
	This is a simple program which reads user inputs or returns them
	I played around with this in python but it was shit so I converted it to C and improved it.
	Playback is simple, it reads inputs in order and executes them
	Writing is also pretty simple, it reads inputs, plays them, then writes them. This way, you can do full board runs and replay them.
	Its like writing code but easier.
	Use template 1 code to extend this if you would like.
	THIS TEMPLATE CODE ASSUMES A 2 WHEELED BOT
*/
const char toPlay[2048] = "wsadx";//CHANGE WITH YOUR INPUTS
static int leftPort = 0;
static int rightPort = 1;

// Stops all motors instantly
void resetMotors() {
	for (int i = 0; i < 4; i++) {
		motor(i, 0);
		msleep(1);
	}
}

/*
	Hyper simple move method if you would like to actually test the code
*/
void move(int power1, int power2, int time) {
	motor(leftPort, power1);
	motor(rightPort, power2);
	msleep(time);
	resetMotors();
}

/*
	Returns true if there is at least one character waiting on stdin, without blocking.
	Uses select() with a zero timeout to poll the stdin file descriptor.
*/
int stdinHasInput(void) {
	fd_set fds;
	struct timeval tv = { 0, 0 };

	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);

	return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

void playbackHandling(const char* toPlayback) {

	/*
		This is where you would put things such as:
		Reset
		Wait for light
		Calibrate sensors, gyros, and other
		Initialize potential constants
		Auto shut off in
	*/
	for (int i = 0; i < strlen(toPlayback); i++) {
		switch (toPlayback[i]) {
		case 'W':
		case 'w':
			printf("moved forward");
			move(70, 70, 300);
			break;
		case 'S':
		case 's':
			printf("moved back");
			move(-70, -70, 300);
			break;
		case 'A':
		case 'a':
			printf("moved left");
			move(-70, 70, 300);
			break;
		case 'D':
		case 'd':
			printf("moved right");
			move(70, -70, 300);
			break;
		case 'X':
		case 'x':
			printf("end");
			return;
		default:
			printf("Skipped");
			break;
		}
	}
}

char* writeHandling() {
	int capacity = 16;
	int length = 0;
	char* moves = malloc(capacity * sizeof(char));
	if (moves == NULL) {
		return NULL;
	}
	int c;
	while ((c = getchar()) != EOF) {
		if (length >= capacity - 1) {
			capacity *= 2;
			char* temp = realloc(moves, capacity * sizeof(char));
			if (temp == NULL) {
				free(moves);
				return NULL;
			}
			moves = temp;
		}
		switch (c) {
		case 'W':
		case 'w':
			move(70, 70, 300);
			handlingLogic1();
		case 'S':
		case 's':
			move(-70, -70, 300);
			handlingLogic1();
		case 'A':
		case 'a':
			move(-70, 70, 300);
			handlingLogic1();
		case 'D':
		case 'd':
			move(70, -70, 300);
			handlingLogic1();
		case 'X':
		case 'x':
			//stop
			moves[length++] = (char)c;
			moves[length] = '\0';
			return moves;
		default:
			break;
		}
	}
	moves[length] = '\0';
	return moves;
}

int main(void) {
	printf("Press 0 / A to record a new sequence, or 1 / B to play back an existing one: ");
	int choice = -1;
	while (true) {
		if (stdinHasInput()) {
			choice = getchar();
			break;
		}
		if (a_button() == 1) {
			choice = '0';
			break;
		}
		if (b_button() == 1) {
			choice = '1';
			break;
		}
	}
	if (choice == '0') {
		char* recorded = writeHandling();
		if (recorded != NULL) {
			printf("Recorded sequence: %s\n", recorded);
			free(recorded);
		}
	}
	else if (choice == '1') {
		printf("Playing back, if there is no output, ensure that you have change the input at the top of the file.");
		playbackHandling(toPlay);
	}
	else {
		printf("Invalid input. Please press 0 or 1.\n");
	}
	return 0;
}
