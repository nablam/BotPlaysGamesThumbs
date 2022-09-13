/*
 Name:		MainSketch.ino
 Created:	9/8/2022 2:12:38 PM
 Author:	Nabz
*/

#include <Servo.h>
#include <Math.h>
Servo myservo_LB; //left bottom
Servo myservo_LT; //

Servo myservo_RB; //
Servo myservo_RT; //

Servo ALLSERVOS[4] = { myservo_LB ,myservo_LT, myservo_RB,myservo_RT };

#pragma region  PotRegion
int potPin_lr = A0;
int potPin_ud = A1;
const  int midPot_lr = 501;
const  int midPot_ud = 501;
const int deadzone = 20;

bool DeadzoneLR = false;
bool DeadzoneUD = false;

int PotToDir = 0;
int ValPot_lr;
int ValPot_ud;
float ScaledPot_lr = 0;
float ScaledPot_ud = 0;
int XaxisValue;
int YaxisValue;

int myangle = 0;;

int radToDeg(float rad) { return rad * (180 / M_PI); }
int vectorAngle(int x, int y) {
	if (x == 0) // special cases
		return (y > 0) ? 90
		: (y == 0) ? 0
		: 270;
	else if (y == 0) // special cases
		return (x >= 0) ? 0
		: 180;
	int ret = radToDeg(atanf((float)y / x));
	if (x < 0 && y < 0) // quadrant Ⅲ
		ret = 180 + ret;
	else if (x < 0) // quadrant Ⅱ
		ret = 180 + ret; // it actually substracts
	else if (y < 0) // quadrant Ⅳ
		ret = 270 + (90 + ret); // it actually substracts
	return ret;
}

#pragma endregion





#pragma region PinDef

#define RELAY_PIN 3
#define RELAY2_PIN 4

#define BUTTON_PIN 5
#define BUTTON2_PIN 6

#define SVO_RT_PIN 9 //red
#define SVO_RB_PIN 10 //brown
#define SVO_LT_PIN 11 //orange
#define SVO_LB_PIN 12 //yellow 
int pos = 0;
#pragma endregion


#pragma region timing
unsigned long previousMillis = 0;
// constants won't change:
const long interval = 1000;

unsigned long previousTimeLed1 = millis();
long timeIntervalLed1 = 10;
unsigned long previousTimeSerialPrintPotentiometer = millis();
long timeIntervalSerialPrint = 50;

#pragma endregion



#pragma region AngleArrays
//servoL maxLeft 128

//servoR maxRitgh 52
// 
//mid  L111 R69
//N    L94 R86
//nw  84 113
//W  L132 R84
//sw 130 82
//S  L128 R52
//se L96 R50
//E 109 R51
//ne 96 50
//              M0   N1    nw2   W3    sw4   S5    se6   E7   ne8
int araL[] = { 119, 109, 119, 129, 132, 128, 118, 109, 105 };
int araR[] = { 61,  71,  75,  71,  62,  52,  47,  51,  61 };
int curDirIndex = 0;

#pragma endregion



int Arra_LB_LT_RB_RT[4] = { 60,120,127,65 };


double  x;
double  y;
double midY; //for RightHandServos
double Dr, Dl;
double E;
double  Ar, Al, Br, Bl, Cr, Cl;
double  Ir, Il, Jr, Jl, Sr, Sl;
double ANGLE;
double angRads;
double radius;

// the setup function runs once when you press reset or power the board




void setup() {
	Serial.begin(115200);

	pinMode(potPin_lr, INPUT);
	pinMode(potPin_ud, INPUT);
	pinMode(BUTTON_PIN, INPUT);
	pinMode(RELAY_PIN, OUTPUT);

	pinMode(BUTTON2_PIN, INPUT);
	pinMode(RELAY2_PIN, OUTPUT);
	curDirIndex = 1;

	myservo_LB.attach(SVO_LB_PIN);
	myservo_LT.attach(SVO_LT_PIN);
	myservo_RB.attach(SVO_RB_PIN);
	myservo_RT.attach(SVO_RT_PIN);


	//myservo_LB.write(0);
	//myservo_LT.write(0);
	//myservo_RB.write(0);
	//myservo_RT.write(0);





	for (int s = 0; s < 4; s++) {
		ALLSERVOS[s].write(Arra_LB_LT_RB_RT[s]);
	}

	delay(3000);
	//                                
	//       |\ .                        
	//       | \   \       Br               
	//       |  \      \                   
	//       |   \         \                  
	//       |    \            \                
	//       |     \              /             
	//     E |      \ Cr         /                
	//       |       \          /                 
	//       |        \        /  Ar                        
	//       |         \      /                          
	//       |          \    /                         
	//       |________Ir_\Jr/ Sr                               
	//             Dr                       
	radius = 10;
	ANGLE = 90;
	angRads = ANGLE / 360 * 2 * PI;

	x = 0;
	y = 0;

	x = cos(angRads) * radius;
	y = sin(angRads) * radius;
	Serial.println("x= " + String(x, 6) + "  y=" + String(y, 6));


	midY = 50; //for RightHandServos
	Dr = 19.5 - x;
	Dl = 19.5 + x;
	E = midY + y;
	Ar = 32;
	Al = 32;
	Br = 48;
	Bl = 48;
	Cr = sqrt((E * E) + (Dr * Dr));
	Cl = sqrt((E * E) + (Dl * Dl));

	Serial.println("Cr= " + String(Cr, 6) + "  Cl=" + String(Cl, 6));


	Ir = acos(Dr / Cr) / 2 / PI * 360;
	Il = acos(Dl / Cl) / 2 / PI * 360;
	Serial.println("Ir= " + String(Ir, 6) + "  Il=" + String(Il, 6));

	Jr = acos(((Cr * Cr) + (Ar * Ar) - (Br * Br)) / (2 * Ar * Cr)) / 2 / PI * 360;
	Jl = acos(((Cl * Cl) + (Al * Al) - (Bl * Bl)) / (2 * Al * Cl)) / 2 / PI * 360;
	Serial.println("Jr= " + String(Jr, 6) + "  Jl=" + String(Jl, 6));

	Sr = 180 - Ir - Jr;
	Sl = Il + Jl;

	Serial.println("Sr= " + String(Sr, 6) + "  Sl=" + String(Sl, 6));

}

void populateServoangs(double argAng, double argRadius) {
	radius = argRadius;
	ANGLE = argAng;
	angRads = ANGLE / 360 * 2 * PI;

	x = 0;
	y = 0;

	x = cos(angRads) * radius;
	y = sin(angRads) * radius;

	midY = 50; //for RightHandServos
	Dr = 19.5 + x;
	Dl = 19.5 - x;
	E = midY + y;
	Ar = 32;
	Al = 32;
	Br = 48;
	Bl = 48;
	Cr = sqrt((E * E) + (Dr * Dr));
	Cl = sqrt((E * E) + (Dl * Dl));




	Ir = acos(Dr / Cr) / 2 / PI * 360;
	Il = acos(Dl / Cl) / 2 / PI * 360;

	Jr = acos(((Cr * Cr) + (Ar * Ar) - (Br * Br)) / (2 * Ar * Cr)) / 2 / PI * 360;
	Jl = acos(((Cl * Cl) + (Al * Al) - (Bl * Bl)) / (2 * Al * Cl)) / 2 / PI * 360;

	Sr = 180 - Ir - Jr;
	Sl = Il + Jl;

	Arra_LB_LT_RB_RT[2] = int(Sl);
	Arra_LB_LT_RB_RT[3] = int(Sr);



	//***********************************




	radius = argRadius;
	ANGLE = 360 - argAng;
	angRads = ANGLE / 360 * 2 * PI;

	x = 0;
	y = 0;

	x = cos(angRads) * radius;
	y = sin(angRads) * radius;

	midY = 60; //for RightHandServos
	Dr = 19.5 - x;
	Dl = 19.5 + x;
	E = midY + y;
	Ar = 32;
	Al = 32;
	Br = 48;
	Bl = 48;
	Cr = sqrt((E * E) + (Dr * Dr));
	Cl = sqrt((E * E) + (Dl * Dl));




	Ir = acos(Dr / Cr) / 2 / PI * 360;
	Il = acos(Dl / Cl) / 2 / PI * 360;

	Jr = acos(((Cr * Cr) + (Ar * Ar) - (Br * Br)) / (2 * Ar * Cr)) / 2 / PI * 360;
	Jl = acos(((Cl * Cl) + (Al * Al) - (Bl * Bl)) / (2 * Al * Cl)) / 2 / PI * 360;

	Sr = 180 - Ir - Jr;
	Sl = Il + Jl;

	Arra_LB_LT_RB_RT[0] = int(Sr);
	Arra_LB_LT_RB_RT[1] = int(Sl);

}

#pragma region PotReading

void ReadPots() {
	ValPot_lr = analogRead(potPin_lr);
	ValPot_ud = analogRead(potPin_ud);

	if (ValPot_lr > (midPot_lr - deadzone) && ValPot_lr < (midPot_lr + deadzone)) { ValPot_lr = 500; DeadzoneLR = true; }
	else
	{
		DeadzoneLR = false;
	}
	if (ValPot_ud > (midPot_ud - deadzone) && ValPot_ud < (midPot_ud + deadzone)) { ValPot_ud = 500; DeadzoneUD = true; }
	else
	{
		DeadzoneUD = false;
	}

	ScaledPot_lr = map(ValPot_lr, 0, 1023, 0, 512);

	ScaledPot_ud = map(ValPot_ud, 0, 1023, 512, 0);

	if (ScaledPot_lr > 240 && ScaledPot_lr < 247)XaxisValue = 0;
	else
		if (ScaledPot_lr <= 240) XaxisValue = -240 + ScaledPot_lr;
		else
			if (ScaledPot_lr >= 247) XaxisValue = -247 + ScaledPot_lr;


	if (ScaledPot_ud > 240 && ScaledPot_ud < 270)YaxisValue = 0;
	else
		if (ScaledPot_ud <= 240) YaxisValue = -240 + ScaledPot_ud;
		else
			if (ScaledPot_ud >= 270) YaxisValue = -270 + ScaledPot_ud;
}

#pragma endregion


int angcnt = 0;

String readString, servo1, servo2, servo3, servo4;
void testread4() {
	while (Serial.available()) {
		delay(10);
		if (Serial.available() > 0) {
			char c = Serial.read();  //gets one byte from serial buffer
			readString += c; //makes the string readString
		}
	}

	if (readString.length() > 0) {
		Serial.println(readString); //see what was received

		// expect a string like 07002100 containing the two servo positions      
		servo1 = readString.substring(0, 3); //get the first four characters
		servo2 = readString.substring(3, 6); //get the next four characters 
		servo3 = readString.substring(6, 9);
		servo4 = readString.substring(9, 12);

		//Serial.println(servo1);  //print ot serial monitor to see results
		//Serial.println(servo2);
		//Serial.println(servo3);  //print ot serial monitor to see results
		//Serial.println(servo4);
		int n1; //declare as number  
		int n2;
		int n3;
		int n4;

		char carray1[6]; //magic needed to convert string to a number 
		servo1.toCharArray(carray1, sizeof(carray1));
		n1 = atoi(carray1);

		char carray2[6];
		servo2.toCharArray(carray2, sizeof(carray2));
		n2 = atoi(carray2);

		char carray3[6];
		servo3.toCharArray(carray3, sizeof(carray3));
		n3 = atoi(carray3);

		char carray4[6];
		servo4.toCharArray(carray4, sizeof(carray4));
		n4 = atoi(carray4);

		// myservo1.writeMicroseconds(n1); //set servo position 
		// myservo2.writeMicroseconds(n2);


		Serial.println(n1);  //print ot serial monitor to see results
		Serial.println(n2);
		Serial.println(n3);  //print ot serial monitor to see results
		Serial.println(n4);
		readString = "";
	}
}


String commmand, Dir1, Dir2, flag;
void test() {
	while (Serial.available()) {
		delay(10);
		if (Serial.available() > 0) {
			char c = Serial.read();  //gets one byte from serial buffer
			readString += c; //makes the string readString
		}
	}

	if (readString.length() > 0) {
		Serial.println(readString); //see what was received

		// expect a string like 07002100 containing the two servo positions      
		Dir1 = readString.substring(0, 3); //get the first four characters
		Dir2 = readString.substring(3, 6); //get the next four characters 
		commmand = readString.substring(6, 9);
		flag = readString.substring(9, 12);

		//Serial.println(servo1);  //print ot serial monitor to see results
		//Serial.println(servo2);
		//Serial.println(servo3);  //print ot serial monitor to see results
		//Serial.println(servo4);
		int n1; //declare as number  
		int n2;
		int n3;
		int n4;

		char carray1[6]; //magic needed to convert string to a number 
		servo1.toCharArray(carray1, sizeof(carray1));
		n1 = atoi(carray1);

		char carray2[6];
		servo2.toCharArray(carray2, sizeof(carray2));
		n2 = atoi(carray2);

		char carray3[6];
		servo3.toCharArray(carray3, sizeof(carray3));
		n3 = atoi(carray3);

		char carray4[6];
		servo4.toCharArray(carray4, sizeof(carray4));
		n4 = atoi(carray4);

		// myservo1.writeMicroseconds(n1); //set servo position 
		// myservo2.writeMicroseconds(n2);


		Serial.println(n1);  //print ot serial monitor to see results
		Serial.println(n2);
		Serial.println(n3);  //print ot serial monitor to see results
		Serial.println(n4);
		readString = "";
	}
}
// the loop function runs over and over again until power down or reset
void loop() {
	
	return;
	// test();
	 /*
	  myservo_LB.write(pos);
	  myservo_LT.write(pos);
	  myservo_RB.write(pos);
	  myservo_RT.write(pos);
	  */
	unsigned long currentTime = millis();

	if (currentTime % 15 == 0) {
		angcnt++;
		if (angcnt >= 360)angcnt = 0;

		populateServoangs(angcnt, 10);
	}



	for (int s = 0; s < 4; s++) {
		ALLSERVOS[s].write(Arra_LB_LT_RB_RT[s]);
	}

	////task1 run servos
	//TASK1(currentTime);

	////task2 serial read
	//TASK2();

	////task3 read buttons
	TASK3();

	////task4 read pots
	//TASK4();

	////task5 writ console
	//TASK5(currentTime);
}

int cnt = 0;
void RunServoEveryXframe(int argXframe, Servo argServoBot, Servo argServoTop) {
	cnt++;
	if (cnt >= 100009)cnt = 0;
	if (cnt % argXframe == 0) {
		argServoBot.write(araL[curDirIndex]);
		argServoTop.write(araR[curDirIndex]);
	}

}


int GetDirectionFromAngle(int argAngle) {

	int dirIndex_fromAngle = 0;

	//              M0   N1    nw2   W3    sw4   S5    se6   E7   ne8
	// mid
	//if (argAngle ==0 || argAngle == 360) {
	//    dirIndex_fromAngle = 0;
	//}

	if (DeadzoneLR && DeadzoneUD) {
		dirIndex_fromAngle = 0;
		return dirIndex_fromAngle;
	}

	// E7
	if (argAngle >= 0 && argAngle <= 22) {

		if (DeadzoneLR && DeadzoneUD) {
			dirIndex_fromAngle = 0;

		}
		else


			dirIndex_fromAngle = 7;
	}
	else
		if (argAngle > 337 && argAngle <= 360) {
			dirIndex_fromAngle = 7;
		}
		else
			// ne8
			if (argAngle > 22 && argAngle <= 67) {
				dirIndex_fromAngle = 8;
			}
			else
				// N1
				if (argAngle > 67 && argAngle <= 112) {
					dirIndex_fromAngle = 1;
				}
				else
					// nw2
					if (argAngle > 112 && argAngle <= 157) {
						dirIndex_fromAngle = 2;
					}
					else
						// W3
						if (argAngle > 157 && argAngle <= 202) {
							dirIndex_fromAngle = 3;
						}
						else
							// sw4
							if (argAngle > 202 && argAngle <= 247) {
								dirIndex_fromAngle = 4;
							}
							else
								// S5
								if (argAngle > 247 && argAngle <= 292) {
									dirIndex_fromAngle = 5;
								}
								else
									// se6
									if (argAngle > 292 && argAngle <= 337) {
										dirIndex_fromAngle = 6;
									}



	return dirIndex_fromAngle;
}
void servoSweeptest(Servo argservo) {
	for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
   // in steps of 1 degree
		argservo.write(pos);              // tell servo to go to position in variable 'pos'
		delay(15);                       // waits 15ms for the servo to reach the position
	}
	for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
		argservo.write(pos);              // tell servo to go to position in variable 'pos'
		delay(15);                       // waits 15ms for the servo to reach the position
	}
}


void TASK1(long argcurrentTime) {
	// task 1
	if (argcurrentTime - previousTimeLed1 > timeIntervalLed1) {
		previousTimeLed1 = argcurrentTime;
		curDirIndex = PotToDir;
		RunServoEveryXframe(1, myservo_LB, myservo_LT);
		RunServoEveryXframe(1, myservo_RB, myservo_RT);
	}
}

void TASK2() {
	// task 2
	if (Serial.available()) {
		int userInput = Serial.parseInt();
		if (userInput >= 0 && userInput < 10) {
			curDirIndex = userInput;
		}
	}

}
void TASK3() {
	// task 3
	if (digitalRead(BUTTON_PIN) == HIGH) {
		digitalWrite(RELAY_PIN, HIGH);
	}
	else {
		digitalWrite(RELAY_PIN, LOW);
	}

	if (digitalRead(BUTTON2_PIN) == HIGH) {
		digitalWrite(RELAY2_PIN, HIGH);
	}
	else {
		digitalWrite(RELAY2_PIN, LOW);
	}

}
void TASK4() {
	// task 4
	ReadPots();
	//gets ScaledPot_lr and ScaledPot_ud  and gets clean XaxisValue and YaxisValue
	myangle = vectorAngle(XaxisValue, YaxisValue);
	PotToDir = GetDirectionFromAngle(myangle);
}
void TASK5(long argcurrentTime) {
	// task 5
	if (argcurrentTime - previousTimeSerialPrintPotentiometer > timeIntervalSerialPrint) {
		previousTimeSerialPrintPotentiometer = argcurrentTime;

		Serial.print("myangle: ");
		Serial.print(myangle);

		Serial.print(" lr: ");
		Serial.print(ValPot_lr);
		Serial.print(" ud : ");
		Serial.print(ValPot_ud);
		Serial.print(" X : ");
		Serial.print(XaxisValue);
		Serial.print(" Y : ");
		Serial.print(YaxisValue);



		Serial.print("   dir ");
		Serial.println(PotToDir);
		//Serial.print("Value lr: " );
		//Serial.print(ScaledPot_lr);
		//Serial.print(" :ud  ");
		//Serial.println(ScaledPot_ud);
	}
}