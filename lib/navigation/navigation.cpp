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

bool checkCorner(AccelStepper &left, AccelStepper &right, int front, int back) {
   if (front > 100 || back > 100) {
       move(left, right, 45, 0);
       return true;
   } 
   return false;
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

        if (checkCorner(left, right, front, back)) {
            front = frontSense.read() + 7.7;
            back = backSense.read() + 7.7;
        }

		if (abs(front - back) < 3) {
			if (count == 0) {
                // TODO: will never reach this because of the counts, just for testing
				count += 1;
				move(left, right, 45, 0);
				continue;
			} else if (count >= 1) {
				break;
			}			
		}

		if (front > back) {
            while (1) {
				if (abs(front - back) < 3) {
                    count += 1;
					break;
				}
				move(left, right, 0, (front-back)/2);
				front = frontSense.read() + 7; // +2 accounts for distance to centre of robot
				back = backSense.read() + 7;
                if (checkCorner(left, right, front, back)) {
                    front = frontSense.read() + 7.7;
                    back = backSense.read() + 7.7;
                }       
			}
			
		} else {
			while(1) {
                if (abs(front-back) < 3) {
                    count += 1;
					break;
				}
                move(left, right, 0, -(back-front)/2);
                front = frontSense.read() + 7;
                back = backSense.read() + 7;
                if (checkCorner(left, right, front, back)) {
                    front = frontSense.read() + 7.7;
                    back = backSense.read() + 7.7;
                }
            }
		}
		move(left, right, 95, 0);
	}
}
