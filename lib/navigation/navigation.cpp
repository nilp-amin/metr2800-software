#include "navigation.h"

float distanceCentre(int x, int y) {
	return (float)sqrt(pow((50-x),2) + pow((50-y),2));
}

float angleCentre(int x, int y) {
    return (float)atan2((50-y), (50-x));
}

/*int turnAngle(int first, int second, int angleSquare) {
	int angle;
	if (first > 50 & second > 50) {
		left turn;
	} else if (first > 50 & second < 50) {
		right turn;
	} else if (first < 50 & second > 50) {
		right turn + 90;
	} else if (fist < 50 & second > 50) {
		left turn + 90;
	}
}*/

void xxlocatexx(Ultrasonic frontSense, Ultrasonic backSense, AccelStepper &left, AccelStepper &right) {
	int moveAngle = 10;
	int dist[180/moveAngle]; // calibrate array size, will be based on move angle 
	int dist2[180/moveAngle];
	for (uint16_t i=0; i<sizeof(dist)/sizeof(dist[0]); i++) {
		rotateCW(left, right, moveAngle);
		dist[i] = frontSense.read() + 7; // +2 accounts for distance to centre of robot
		dist2[i] = backSense.read() + 7;
	}
	
	//int angleToSquare;
	int currentCoord[2];
	for (uint16_t i = 0; i < sizeof(dist)/sizeof(dist[0]); i++) {
        Serial.println(dist[i] + dist2[i]);
		if ((dist[i] + dist2[i]) > 97  && (dist[i] + dist2[i]) < 103) { 
            // TODO: Have to make sure i+90/moveAngle does not overflow
            int perpIndex = i + (90 / moveAngle);
			if ((dist[perpIndex] + dist2[perpIndex] > 98) && (dist2[perpIndex] + dist2[perpIndex]) < 101) {
				//angleToSquare = i * moveAngle;
				currentCoord[0] = (dist[i] < dist2[i]) ? dist[i] : dist2[i];
				currentCoord[1] = (dist[perpIndex] < dist2[perpIndex]) ? dist[perpIndex] : dist2[perpIndex];
				float distCentre = distanceCentre(currentCoord[0], currentCoord[1]);
				float angle = angleCentre(currentCoord[0], currentCoord[1]) + 90; // might need to add 180; (nilp) we need to add 90?


				move(left, right, angle, distCentre); 
				break;
			}
		}
	}
	// might be best to have these values stored in a struct, and alter a struct in main. 
	return;
}

// DEMO FUNCTONS
void locate(Ultrasonic frontSense, Ultrasonic backSense, AccelStepper &left, AccelStepper &right) {
	// move direction of largest until front back are equal
	float front;
	float back;
	int count = 0;
	while(1) {
		front = frontSense.read() + 7.7; // +2 accounts for distance to centre of robot
		back = backSense.read() + 7.7;

		if (abs(front - back) < 3) {
			if (count == 0) {
				count += 1;
				move(left, right, 110, 0);
				continue;
			} else if (count == 1) {
				break;
			}			
		}

		if (front > back) {
			while (1) {
				if (abs(front - back) < 2.5) {
					break;
				}
				move(left, right, 0, (front-back)/2);
				front = frontSense.read() + 7; // +2 accounts for distance to centre of robot
				back = backSense.read() + 7;
			}
			
		} else {
			while(1) {
                if (abs(front-back) < 2.5) {
					break;
				}
                move(left, right, 0, -(back-front)/2);
                front = frontSense.read() + 7;
                back = backSense.read() + 7;
            }
		}
		move(left, right, 150, 0);
	}
}
