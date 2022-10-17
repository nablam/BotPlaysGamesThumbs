/*
 Name:		MainSketch.ino
 Created:	9/8/2022 2:12:38 PM
 Author:	Nabz
*/
//<045108111034115111888012>
#include <Servo.h>
//#include <Math.h>

#include <Arduino.h>
#include <U8x8lib.h>
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);


#pragma region PinDef

#define RELAY_PIN 3
#define RELAY2_PIN 4

#define BUTTON_PIN 5
#define BUTTON2_PIN 6

#define SVO_RT_PIN 9 //red
#define SVO_RB_PIN 10 //brown
#define SVO_LT_PIN 11 //orange
#define SVO_LB_PIN 12 //yellow 
#pragma endregion

Servo myservo_LB; //left bottom
Servo myservo_LT; //

Servo myservo_RB; //
Servo myservo_RT; //

Servo ALLSERVOS[4] = { myservo_LB ,myservo_LT, myservo_RB,myservo_RT };
int Arra_LB_LT_RB_RT[4] = { 60,120,127,65 };
int CURSTATE_svo_LB_LT_RB_RT[4] = { 60,120,127,65 };
bool CURSTATE_SOL2_left_On = false;
bool CURSTATE_SOL5_right_On = false;
int CURSTATE_Command = 0;
int CURSTATE_Debug = 0;
#pragma region DataPars

const byte numChars = 27;
char receivedChars[numChars];
boolean newData = false;

char carray0[4] = "xxx";
char carray1[4] = "xxx";
char carray2[4] = "xxx";
char carray3[4] = "xxx";
char carray4[4] = "xxx";
char carray5[4] = "xxx";
char carray6[4] = "xxx";
char carray7[4] = "xxx";
char* arrofarra[8] = { carray0,carray1,carray2,carray3,carray4,carray5,carray6,carray7 };
int RawInts[8] = { 0,0,0,0,0,0,0,0 };
#pragma endregion

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

#pragma region timing
unsigned long previoustimeInterval_10ms = millis();
long timeInterval_10ms = 10;

unsigned long previoustimeInterval_15ms = millis();
long timeInterval_15ms = 15;

//20fps
unsigned long previoustimeInterval_20ms = millis();
long timeInterval_20ms = 20;

//30fps
unsigned long previoustimeInterval_33ms = millis();
long timeInterval_33ms = 33;


#pragma endregion

void TASK1_10ms(long argcurrentTime) {
	if (argcurrentTime - previoustimeInterval_10ms > timeInterval_10ms) {
		previoustimeInterval_10ms = argcurrentTime;
		//DoHAckyTaskImplementation
	 

		ReceiveWitheReadUnityNewline_andPRocess();
	
	}
}

void TASK1_15ms(long argcurrentTime) {
	if (argcurrentTime - previoustimeInterval_15ms > timeInterval_15ms) {
		previoustimeInterval_15ms = argcurrentTime;
		//DoHAckyTaskImplementation
		for (int s = 0; s < 4; s++) {
			ALLSERVOS[s].write(CURSTATE_svo_LB_LT_RB_RT[s]);
		}

		digitalWrite(RELAY_PIN, CURSTATE_SOL2_left_On);
		digitalWrite(RELAY2_PIN, CURSTATE_SOL5_right_On);



	}
}



void recvWithStartEndMarkers() {
	static boolean recvInProgress = false;
	static byte ndx = 0;
	char startMarker = '<';
	char endMarker = '>';
	char rc;

	while (Serial.available() > 0 && newData == false) {
		rc = Serial.read();

		if (recvInProgress == true) {
			if (rc != endMarker) {
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= numChars) {
					ndx = numChars - 1;
				}
			}
			else {
				receivedChars[ndx] = '\0'; // terminate the string
				recvInProgress = false;
				ndx = 0;
				newData = true;
			}
		}

		else if (rc == startMarker) {
			recvInProgress = true;
		}
	}
}
void processRecivedData() {
	if (newData == true) {
		//Serial.print("data in buffer ");
		//Serial.println(receivedChars);
		int charcount = 0;
		for (int c = 0; c < numChars; c++) {

			if (receivedChars[c] != NULL) {
				charcount++;
			}
		}

		//poulate local variables
		for (int arraRow = 0; arraRow < 8; arraRow++) {
			for (int cindex = 0; cindex < 3; cindex++) {
				int offset = 3 * arraRow;
				arrofarra[arraRow][cindex] = receivedChars[offset + cindex];
			}
			RawInts[arraRow] = atoi(arrofarra[arraRow]);
		}

		CURSTATE_svo_LB_LT_RB_RT[0] = RawInts[0];
		CURSTATE_svo_LB_LT_RB_RT[1] = RawInts[1];
		if (RawInts[2] > 0) {
			CURSTATE_SOL2_left_On = true;
		}
		else {
			CURSTATE_SOL2_left_On = false;
		}

		CURSTATE_svo_LB_LT_RB_RT[2] = RawInts[3];
		CURSTATE_svo_LB_LT_RB_RT[3] = RawInts[4];

		if (RawInts[5] > 0) {
			CURSTATE_SOL5_right_On = true;
		}
		else {
			CURSTATE_SOL5_right_On = false;
		}

		CURSTATE_Command = RawInts[6];
		CURSTATE_Debug = RawInts[7];


		 
		newData = false;//_______________________________the key to finished datatprocessing
	}
}
 
int maxSizeReadBuff = 0;
int maxSizeReadBuff2 = 0;
void ReceiveWitheReadUnityNewline_andPRocess() {
	static byte ndx = 0;
	String strrc;

	while (Serial.available() > 0) {
	
		
	

	

		strrc = Serial.readStringUntil('\n');
		int str_len = strrc.length();
		char str_len_tmp[8];
		itoa(str_len, str_len_tmp, 10);

		u8x8.drawString(4, 6, str_len_tmp);
	/*
		char str_BYTES_tmp[27];
		strrc = Serial.readBytesUntil('#', str_BYTES_tmp,27);
		int str_len = strrc.length();
		char str_len_tmp[8];
		itoa(str_len, str_len_tmp, 10);
	*/

	

		//messages from unity with Sendstring() produce 27 bytes
		//messages from conso,e produce 26
		if (str_len >= 26) {

			char char_array_Zerothletter[2]; //holeds one char and '\n'
			strrc.substring(0, 1).toCharArray(char_array_Zerothletter, 2);

			char char_array_26thletter[2]; //holeds one char and '\n'
			strrc.substring(25, 26).toCharArray(char_array_26thletter, 2);


			if (char_array_Zerothletter[0] == '<' && char_array_26thletter[0] == '>') {

				char char_array_24bytes[25];
				strrc.substring(1, 25).toCharArray(char_array_24bytes, 25);//from char1 to char 24 , but allow 15 bytes for '\n'

				for (int arraRow = 0; arraRow < 8; arraRow++) {
					for (int cindex = 0; cindex < 3; cindex++) {
						int offset = 3 * arraRow;
						arrofarra[arraRow][cindex] = char_array_24bytes[offset + cindex];
					}
					RawInts[arraRow] = atoi(arrofarra[arraRow]);
				}


				u8x8.drawString(0, 6, char_array_Zerothletter);
				u8x8.drawString(2, 6, "s=");
				/*u8x8.drawString(4, 6, str_len_tmp);*/
				u8x8.drawString(7, 6, char_array_26thletter);

				


				CURSTATE_svo_LB_LT_RB_RT[0] = RawInts[0];
				CURSTATE_svo_LB_LT_RB_RT[1] = RawInts[1];
				if (RawInts[2] > 0) {
					CURSTATE_SOL2_left_On = true;
				}
				else {
					CURSTATE_SOL2_left_On = false;
				}

				CURSTATE_svo_LB_LT_RB_RT[2] = RawInts[3];
				CURSTATE_svo_LB_LT_RB_RT[3] = RawInts[4];

				if (RawInts[5] > 0) {
					CURSTATE_SOL5_right_On = true;
				}
				else {
					CURSTATE_SOL5_right_On = false;
				}

				CURSTATE_Command = RawInts[6];
				CURSTATE_Debug = RawInts[7];
			}

		 


		}
		else
		{
			//nothing , just keep old received char
			 
				//Serial.read();
		}
	}
}

void setup_uu8x8(void)
{

	u8x8.begin();
	u8x8.setPowerSave(0);
}

void setup() {
	Serial.begin(115200);
	pinMode(potPin_lr, INPUT);
	pinMode(potPin_ud, INPUT);
	pinMode(BUTTON_PIN, INPUT);
	pinMode(RELAY_PIN, OUTPUT);

	pinMode(BUTTON2_PIN, INPUT);
	pinMode(RELAY2_PIN, OUTPUT);
	 

	myservo_LB.attach(SVO_LB_PIN);
	myservo_LT.attach(SVO_LT_PIN);
	myservo_RB.attach(SVO_RB_PIN);
	myservo_RT.attach(SVO_RT_PIN);


	setup_uu8x8();
	u8x8.setFont(u8x8_font_chroma48medium8_r);
	u8x8.drawString(3, 2, "starting");
	//delay(400);
	

	//for (int s = 0; s < 4; s++) {
	//	ALLSERVOS[s].write(0);
	//}

//	delay(1000);


	//for (int s = 0; s < 4; s++) {
	//	ALLSERVOS[s].write(90);
	//}
//	delay(1000);


	for (int s = 0; s < 4; s++) {
		ALLSERVOS[s].write(Arra_LB_LT_RB_RT[s]);
	}

	delay(500);
	u8x8.clearDisplay();
}


void loop() {

	unsigned long currentTime = millis();
 


	 
	//recvWithStartEndMarkers();
	//processRecivedData();
	 //ReceiveWitheReadUnityNewline_andPRocess();

	TASK1_10ms(currentTime);
	TASK1_15ms(currentTime);

	u8x8.drawString(0, 0, arrofarra[0]); u8x8.drawString(4, 0, arrofarra[1]);  u8x8.drawString(8, 0, arrofarra[3]); 	u8x8.drawString(12, 0, arrofarra[4]);
	u8x8.drawString(0, 1, arrofarra[2]);									   u8x8.drawString(8, 1, arrofarra[5]);

	u8x8.drawString(0, 3, arrofarra[6]); u8x8.drawString(4, 3, arrofarra[7]);






 




	 u8x8.refreshDisplay();
}
void testloop() {

	unsigned long currentTime = millis();
	int ReadBufferSize = Serial.available();
	char readBuff_len__tmp[32];
	itoa(ReadBufferSize, readBuff_len__tmp, 10);
	u8x8.drawString(0, 6, "b=");
	u8x8.drawString(2, 6, readBuff_len__tmp);

	if (ReadBufferSize > maxSizeReadBuff) {
		maxSizeReadBuff = ReadBufferSize;
	}
	char readBuff_Maxlen__tmp[32];
	itoa(maxSizeReadBuff, readBuff_Maxlen__tmp, 10);
	u8x8.drawString(6, 6, "mx");
	u8x8.drawString(8, 6, readBuff_Maxlen__tmp);


	//ReceiveWitheReadUnityNewline_andPRocess();
	//recvWithStartEndMarkers();
	//processRecivedData();
	//ReceiveWitheReadUnityNewline_andPRocess();

	//TASK1_10ms(currentTime);
	//TASK1_15ms(currentTime);

	//u8x8.drawString(0, 0, arrofarra[0]); u8x8.drawString(4, 0, arrofarra[1]);  u8x8.drawString(8, 0, arrofarra[3]); 	u8x8.drawString(12, 0, arrofarra[4]);
	//u8x8.drawString(0, 1, arrofarra[2]);									   u8x8.drawString(8, 1, arrofarra[5]);

	//u8x8.drawString(0, 3, arrofarra[6]); u8x8.drawString(4, 3, arrofarra[7]);







	



	int ReadBufferSize2 = Serial.available();
	char readBuff_len__tmp2[32];
	itoa(ReadBufferSize2, readBuff_len__tmp2, 10);
	u8x8.drawString(0, 7, "b=");
	u8x8.drawString(2, 7, readBuff_len__tmp2);

	if (ReadBufferSize2 > maxSizeReadBuff2) {
		maxSizeReadBuff2 = ReadBufferSize2;
	}
	char readBuff_Maxlen__tmp2[32];
	itoa(maxSizeReadBuff2, readBuff_Maxlen__tmp2, 10);
	u8x8.drawString(6, 7, "mx");
	u8x8.drawString(8, 7, readBuff_Maxlen__tmp2);




	//u8x8.refreshDisplay();
}