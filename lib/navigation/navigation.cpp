#include "navigation.h"

float distanceCentre(int x, int y) {
	float value = sqrt(pow((50-x),2) + pow((50-y),2));
	return value;
}

float angleCentre(int x, int y) {
	float angle = atan((50-y)/(50-x));
	return angle;
}

/*int rotateAngle(int first, int second, int angleSquare) {
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

void locate(Ultrasonic frontSense, Ultrasonic backSense, AccelStepper &left, AccelStepper &right) {
	int dist[36]; // calibrate array size, will be based on move angle 
	int dist2[36];
	int moveAngle = 5;
	for (int i=0; i<50; i++) {
		rotateCW(left, right, 5);
		dist[i] = frontSense.read() + 2; // +2 accounts for distance to centre of robot
		dist2[i] = backSense.read() + 2;
	}
	
	int angleToSquare;
	int currentCoord[2];
	for (int i=0; i < sizeof(dist); i++) {
		if (dist[i] + dist2[i] > 98  && dist[i] + dist2[i] < 101) { 
			if (dist[i+90/moveAngle] + dist2[i+90/moveAngle] > 98 && dist2[i+90/moveAngle] + dist2[i+90/moveAngle] < 101) {
				angleToSquare = i * moveAngle;
				currentCoord[0] = (dist[i] < dist2[i]) ? dist[i] : dist2[i];
				currentCoord[1] = (dist[i+90/moveAngle] < dist2[i+90/moveAngle]) ? dist[i+90/moveAngle] : dist2[i+90/moveAngle];
				float distCentre = distanceCentre(currentCoord[0], currentCoord[1]);
				float angle = angleCentre(currentCoord[0], currentCoord[1]); // might need to add 180;
				
				move(left, right, angle, distCentre); 
				break;
			}
		}
	}
	// might be best to have these values stored in a struct, and alter a struct in main. 
	return;
}