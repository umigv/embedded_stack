
#include "odrivestm32.h"
void SetPosition(int motor_number, float position){
//	char Data[100];
//	int n;
//	n = sprintf(Data, "p %d %f 0.0 0.0 \n", motor_number, position);
//	HAL_UART_Transmit(&huart4, Data, 40, 1000);
	SetPositionWithCurrentVelocity(motor_number, position,0.0, 0.0);
}

void SetPositionWithVelocity(int motor_number, float position, float velocity_feedforward){
   char Data[100];
   int n;
   n = sprintf(Data, "p %d %f %f %f \n", motor_number, position, velocity_feedforward);
}

void SetPositionWithCurrentVelocity(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    char Data[100];
	int n;
	n = sprintf(Data, "p %d %f %f %f \n", motor_number, position, velocity_feedforward, current_feedforward);
	HAL_UART_Transmit(&huart4, Data, n, 250);
}

void SetVelocity(int motor_number, float velocity){
	SetVelocityWithCurrent(motor_number, velocity, 0.0);
}

void SetVelocityWithCurrent(int motor_number, float velocity, float current_feedforward) {
	//make empty array for command
	char data[100];
	int n;

	//fill data array with command and find length of command
	n = sprintf(data, "v %d %f %f\n", motor_number, velocity, current_feedforward);

	//transmit command
	HAL_UART_Transmit(&huart4,data,n,250);
}

void SetCurrent(int motor_number, float current) {
//	//Create a holder for characters
//	char data[100];
//
//	//Convert your inputs and other chars you want into a char array and find its length
//	n = Sprintf(data, "c %d %f\n", motor_number, current);
//
//	//Transmit your char array
//	HAL_UART_Transmit(&huart4, data, n, 250);

}

void TrapezoidalMove(int motor_number, float position) {
//	//Create a holder for characters
//	char data[100];
//
//	//Convert your inputs and other chars you want into a char array and find its length
//	n = Sprintf(data, "t %d %f\n", motor_number, position);
//
//	//Transmit your char array
//	HAL_UART_Transmit(&huart4, data, n, 250);

}

double readFloat() {
    // Old code:
    // return readString().toFloat();

    // Uses atof, but might need to be changed to stof depending on readString()
    char buffer[8];
    readString(buffer, 8, 1000);
    return atof(buffer);
}

float GetVelocity(int motor_number) {        // NEED TO "#include <string.h>" for snprintf()
    // odrive command format:
    // r axis[motor_number].encoder.vel_estimate

    // Output buffer
    char getV[30];
    // snprintf() works like printf() but stores the would-be output into the getV buffer instead
    snprintf(getV, 30, "r axis%d.encoder.vel_estimate\n", motor_number);
    //snprintf(getV, 40, "odrv0.axis%d.encoder.vel_estimate\n", motor_number);
    // Transmits the getV command to odrive
    // Size parameter might need to be changed (most likely remove the '+ 1')
    HAL_UART_Transmit(&huart4,getV,strlen(getV) + 1, 250);      // Need huart object to be declared as well (main.c ???)
    return readFloat();
}
	
double GetPosition(int motor_number){
//	char send_data[40] = "r axis ";
//	char numbuffer[3];
//	snprintf(numbuffer, "%d", motor_number);
//	strcat(send_data,numbuffer);
//	char suffix[26] =".encoder.pos_estimate\n";
//	strcat(send_data, suffix);
	char getV[40];
	    // snprintf() works like printf() but stores the would-be output into the getV buffer instead
	snprintf(getV, 40, "r axis%d.encoder.pos_estimate\n", motor_number);
	char buffer[40];
	HAL_UART_Transmit(&huart4,getV,strlen(getV),250);
//	HAL_UART_Receive(&huart4, &buffer[0], 40, 1000);
	return readFloat();
}

int32_t readInt(){
	
	char buffer[40];
	readString(buffer, 40, 1000);
	
	return atoi(buffer);
	
}

//bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout) {
//
//}

bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout) {
//   int timeout_ctr = (int)(timeout * 10.0f);
//
//   char Rx_data[26] = "w axis0.requested_state 3\n";
//   Rx_data[6] = axis;
//   Rx_data[24] = requested_state;
//   HAL_UART_Transmit(&huart4,Rx_data,26,250);
//
//   //serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
//
//   if (wait_for_idle) {
//       do {
//           HAL_Delay(100);
//
//           char Rx_data[26] = "r axis0.current_state\n";
//           Rx_data[6] = axis;
//           HAL_UART_Transmit(&huart4,Rx_data,26,250);
//
//           //serial_ << "r axis" << axis << ".current_state\n";
//
//       } while (readInt() != 0 && --timeout_ctr > 0);
//   }
//   return timeout_ctr > 0;
}

void readString(char* buf, uint16_t len, int timeout) {
//    static const unsigned long timeout = 1000;
//    unsigned long timeout_start = millis();
//    int i = 0;
//
//    for (;;) {
//        while (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) == SET) {
//            if (millis() - timeout_start >= timeout) {
//                break;
//            }
//        }
//        char c;
//        HAL_UART_Receive(&huart4, c, 1, 1000);
//        buf[i] = c;
//        if (c == '\n') {
//            break;
//        }
//        i++;
//    }
	HAL_UART_Receive(&huart4, buf, len, timeout);


}

void set_tuning_parameters(int motor_number, float pos_gain_value, float vel_gain_value, float vel_integrator_gain_value){
//	char[100] UART_Output;
//	int sizeOfString;
//
//	//pos_gain
//	sizeOfString = sprintf(UART_Output, "%s%i%s%d\n", "w axis", motor_number,
//	    ".controller.config.pos_gain ", pos_gain_value);
//	HAL_UART_Transmit(&huart4, UART_Output, sizeOfString, 100);
//
//	//vel_gain
//	sizeOfString = sprintf(UART_Output, "%s%i%s%d\n", "w axis", motor_number,
//	    ".controller.config.vel_gain ", vel_gain_value);
//	HAL_UART_Transmit(&huart4, UART_Output, sizeOfString, 100);
//
//	//vel_integrator_gain
//	sizeOfString = sprintf(UART_Output, "%s%i%s%d\n", "w axis", motor_number,
//	    ".controller.config.vel_integrator_gain ", vel_integrator_gain_value);
//	HAL_UART_Transmit(&huart4, UART_Output, sizeOfString, 100);
}

void RunCalibrationSequence() {
	//make array for command
	char data[26] = "w axis0.requested_state 3\n";

	//transmit command
	HAL_UART_Transmit(&huart4,data,26,250);
}

