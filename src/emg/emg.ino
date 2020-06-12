#include "math.h";

#define EMG_DATA_LENGTH 100

int THRESHOLDS[3] = {50, 200, 500};
int data[EMG_DATA_LENGTH];

int inputPin = A0;
int level;

void setup() {

}

void loop() {
  	// put your main code here, to run repeatedly:
  	for(int i=0;i<EMG_DATA_LENGTH;i++){
  		data[i] = analogRead(inputPin);
  		delay(2);
  	}

  	level = spiking_level(data);
  	Serial.print("level: ");
  	Serial.println(level);


}



double get_var(int* data){
	double rms = 0;
	double var = 0.;
	double mean = 0.;
	for (int i = 0; i < EMG_DATA_LENGTH; ++i)
	{
		mean += data[i];
	}
	mean /= EMG_DATA_LENGTH;
	for(int i=0;i<EMG_DATA_LENGTH;++i)
	{
		rms += (data[i] - mean)*(data[i] - mean);
	}
	rms /= EMG_DATA_LENGTH;
	var = sqrt(rms);
	return var;
}

int spiking_level(int* data){
	// 0: idle, 1: up, 2: push, 3: down?
	double rms = get_var(data);
	Serial.print("amplitude: ");
  	Serial.println(rms);
	if (rms > THRESHOLDS[2]){
		return 3;
	}
	if(rms > THRESHOLDS[1]){
		return 2;
	}
	if (rms > THRESHOLDS[0]){
		return 1;
	}
	return 0;
}

