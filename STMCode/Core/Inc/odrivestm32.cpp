
#include "odrivestm32.h"
void SetPosition(UART_HandleTypeDef *uart_handler, int motor_number, float position){
	SetPositionWithCurrentVelocity(uart_handler, motor_number, position,10.0, 0.0);
}

void SetPositionWithVelocity(UART_HandleTypeDef *uart_handler, int motor_number, float position, float velocity_feedforward){
   char Data[100];
   sprintf(Data, "p %d %f %f %f \n", motor_number, position, velocity_feedforward);
}

void SetPositionWithCurrentVelocity(UART_HandleTypeDef *uart_handler, int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    char Data[100];
	sprintf(Data, "p %d %f %f %f \n\r", motor_number, position, velocity_feedforward, current_feedforward);
	HAL_UART_Transmit(uart_handler, Data, 100, 1000);
}

void SetVelocity(UART_HandleTypeDef *uart_handler, int motor_number, float velocity){
	SetVelocityWithCurrent(uart_handler, motor_number, velocity, 0.0);
}

void SetVelocityWithCurrent(UART_HandleTypeDef *uart_handler, int motor_number, float velocity, float current_feedforward) {
	//make empty array for command
	char data[100];
	int n;

	//fill data array with command and find length of command
	n = sprintf(data, "v %d %f %f\n", motor_number, velocity, current_feedforward);

	//transmit command
	HAL_UART_Transmit(uart_handler,data,n,250);
}

//void SetCurrent(UART_HandleTypeDef *uart_handler, int motor_number, float current) {
////	//Create a holder for characters
////	char data[100];
////
////	//Convert your inputs and other chars you want into a char array and find its length
////	n = Sprintf(data, "c %d %f\n", motor_number, current);
////
////	//Transmit your char array
////	HAL_UART_Transmit(&huart4, data, n, 250);
//
//}

//void TrapezoidalMove(int motor_number, float position) {
////	//Create a holder for characters
////	char data[100];
////
////	//Convert your inputs and other chars you want into a char array and find its length
////	n = Sprintf(data, "t %d %f\n", motor_number, position);
////
////	//Transmit your char array
////	HAL_UART_Transmit(&huart4, data, n, 250);
//
//}

double readFloat(UART_HandleTypeDef *uart_handler) {
    // Old code:
    // return readString().toFloat();

    // Uses atof, but might need to be changed to stof depending on readString()
    char buffer[7];
    readString(uart_handler, buffer, 7, 200);
//    double divider = 1000.00;
    int n = sizeof(buffer) / sizeof(buffer[0]);
    for (int i = 0; i < n - 1; i++) {
        buffer[i] =buffer[i + 1];
    }

    return atof(buffer);
}

float GetVelocity(UART_HandleTypeDef *uart_handler,int motor_number) {        // NEED TO "#include <string.h>" for snprintf()
    // odrive command format:
    // r axis[motor_number].encoder.vel_estimate

    // Output buffer
    char getV[60];
    // snprintf() works like printf() but stores the would-be output into the getV buffer instead
    snprintf(getV, 60, "r axis%d.encoder.vel_estimate\n\r", motor_number);
    //snprintf(getV, 40, "odrv0.axis%d.encoder.vel_estimate\n", motor_number);
    // Transmits the getV command to odrive
    // Size parameter might need to be changed (most likely remove the '+ 1')
    HAL_UART_Transmit(uart_handler,getV,strlen(getV) + 1, 250);      // Need huart object to be declared as well (main.c ???)
    return readFloat(uart_handler);
}
	
double GetPosition(UART_HandleTypeDef *uart_handler,int motor_number){
	char getV[30];
	    // snprintf() works like printf() but stores the would-be output into the getV buffer instead
	snprintf(getV, 30, "r axis%d.encoder.pos_estimate\n", motor_number);
	char buffer[30];
	HAL_UART_Transmit(uart_handler,getV,strlen(getV),250);
	return readFloat(uart_handler);
}

int32_t readInt(UART_HandleTypeDef *uart_handler){
	
	char buffer[40];
	readString(uart_handler, buffer, 40, 1000);
	
	return atoi(buffer);
	
}

//bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout) {
//
//}

void run_state(UART_HandleTypeDef *uart_handler,int axis, int requested_state, bool wait_for_idle, float timeout) {
   int timeout_ctr = (int)(timeout * 10.0f);
   char Rx_data[30];
   snprintf(Rx_data, 30, "w axis%d.requested_state %d\n\r", axis, requested_state);



//   Rx_data[6] = axis;
//   Rx_data[24] = requested_state;
   HAL_UART_Transmit(uart_handler,Rx_data,30,500);
   readString(uart_handler, Rx_data, 30, 1000);
   //serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';

   if (wait_for_idle) {
       do {
           HAL_Delay(100);

           char Rx_data[26] = "r axis0.current_state\n";
           Rx_data[6] = axis;
           HAL_UART_Transmit(uart_handler,Rx_data,6,250);

           //serial_ << "r axis" << axis << ".current_state\n";

       } while (readInt(uart_handler) != 0 && --timeout_ctr > 0);
   }
//   return timeout_ctr > 0;
}

void readString(UART_HandleTypeDef *uart_handler,char* buf, uint16_t len, int timeout) {

	HAL_UART_Receive(uart_handler, buf, len, timeout);


}

//vel_limit 32
//pos_gain 34
//vel_gain 34
//vel_int_gain 52

void set_tuning_parameters(UART_HandleTypeDef *uart_handler, int motor_number, float pos_gain_value, float vel_gain_value, float vel_integrator_gain_value){
	char UART_Output [100] ;
	int sizeOfString;

	//pos_gain
	sizeOfString = sprintf(UART_Output, "w axis%d.controller.config.pos_gain %.5f\n", motor_number, pos_gain_value);
	HAL_UART_Transmit(&huart4, UART_Output, sizeOfString, 100);
	HAL_Delay(100);

	//vel_gain
	sizeOfString = sprintf(UART_Output, "w axis%d.controller.config.vel_gain %.5f\n", motor_number, vel_gain_value);
	HAL_UART_Transmit(&huart4, UART_Output, sizeOfString, 100);
	HAL_Delay(100);
	//vel_integrator_gain
	sizeOfString = sprintf(UART_Output, "w axis%d.controller.config.vel_integrator_gain %.7f\n", motor_number, vel_integrator_gain_value);
	HAL_UART_Transmit(&huart4, UART_Output, sizeOfString, 100);
	HAL_Delay(100);
}

//void RunCalibrationSequence(UART_HandleTypeDef* uart_handler, int motor_number) {
//	//make array for command
//	char data[27];
//	snprintf(data, 27, "w axis%d.requested_state 3\n", motor_number);
//	//transmit command
//	HAL_UART_Transmit(uart_handler,data,27,250);
//}

void ClearErrors(UART_HandleTypeDef *uart_handler){
	char data[10];
	char buffer[40];
	snprintf(data, 10, "sc\n");
	HAL_UART_Transmit(uart_handler,data,10,250);
	readString(uart_handler, buffer, 40, 1000);
	HAL_Delay(100);
}

void Setup_Cur_Lim(UART_HandleTypeDef *uart_handler, double cur_lim){
	char data0[60];
	snprintf(data0, 60, "w axis0.motor.config.current_lim %.1f\n", cur_lim);
	char data1[60];
	snprintf(data1, 60, "w axis1.motor.config.current_lim %.1f\n", cur_lim);

	HAL_UART_Transmit(uart_handler,data0,60,1000);
	HAL_Delay(100);
	HAL_UART_Transmit(uart_handler,data1,60,1000);
	HAL_Delay(100);


}

void Setup_cpr(UART_HandleTypeDef *uart_handler, double cpr){
	char data0[60];
	snprintf(data0, 60, "w axis0.encoder.config.cpr %.1f\n", cpr);
	char data1[60];
	snprintf(data1, 60, "w axis1.encoder.config.cpr %.1f\n", cpr);
	HAL_UART_Transmit(uart_handler,data0,60,1000);
	HAL_Delay(100);
	HAL_UART_Transmit(uart_handler,data1,60,1000);
	HAL_Delay(100);

}

double GetVelocity_custom(UART_HandleTypeDef *uart_handler, int motor_number, double wheel_diameter){
	double vel = 0;
	double pos = 0;
	uint16_t time = 0;
	//TODO: make a calculation that changes the diameter into circumference of the wheel
	double C = 0;
	pos = GetPosition(uart_handler, motor_number);
	time = HAL_GetTick();
	pos = GetPosition(uart_handler, motor_number) - pos;
	time = HAL_GetTick() - time;

	vel = pos * C / time;
	return vel;


}
void Setup_vel_limit(UART_HandleTypeDef *uart_handler, double RPS_limit){
	char data0[60];
		snprintf(data0, 60, "w axis0.controller.config.vel_limit %.1f\n", RPS_limit);
		char data1[60];
		snprintf(data1, 60, "w axis0.controller.config.vel_limit %.1f\n", RPS_limit);
		HAL_UART_Transmit(uart_handler,data0,60,1000);
		HAL_Delay(100);
		HAL_UART_Transmit(uart_handler,data1,60,1000);
		HAL_Delay(100);
}
//double valuefilter(double input, int position_state){
//	double filtered = fmod(input, position_state);
//	if ()
//}
