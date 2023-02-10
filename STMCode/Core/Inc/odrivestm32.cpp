
#include "odrivestm32.h"
void SetDefaultPosition(int motor_number, float position){
	char Data[100];
	int n;
	n = sprintf(Data, "p %d %f \n", motor_number, position);
	HAL_UART_Transmit(&huart4, Data, n, 1000)
}

void SetPositionWithVelocity(int motor_number, float position, float velocity_feedforward){

}

void SetPositionWithCurrentVelocity(int motor_number, float position, float velocity_feedforward, float current_feedforward) {

}

void SetVelocity(int motor_number, float velocity){

}

void SetVelocityWithCurrent(int motor_number, float velocity, float current_feedforward) {

}

void SetCurrent(int motor_number, float current) {

}

void TrapezoidalMove(int motor_number, float position) {


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
