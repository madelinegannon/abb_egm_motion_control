/** 
*********************************************************************
	Author        : Mohith Sakthivel
	Date          : 04.06.2019
	Description   : Windows console application to implement 
					EGM control Robot and Track Setup
*********************************************************************
**/

#include <thread>
#include "egm_control.h"
#include <json/json.h>

#define TEST_C11
#define TEST_C21

using namespace std;

SOCKET sock;
bool receive_external = true;
float position_targets_rob_1[6] = { 0,0,0,0,0,0 };
float position_targets_rob_2[6] = { 0,0,0,0,0,0 };
float position_targets_rob_3[6] = { 0,0,0,0,0,0 };
float position_targets_rob_4[6] = { 0,0,0,0,0,0 };

Json::Value value;

#ifdef TEST_C11
const std::string ROB1_Name = "Robot 1";
const int ROB1_PORT = 6510;

const std::string ROB2_Name = "Robot 2";
const int ROB2_PORT = 6511;
#endif // TEST_C11

#ifdef TEST_C21
const std::string ROB3_Name = "Robot 3";
const int ROB3_PORT = 6512;

const std::string ROB4_Name = "Robot 4";
const int ROB4_PORT = 6513;
#endif // TEST_C21

bool IsReadable = true;

// Input file parameters
int msg_count = 1482;
int sampling_time = 4;

std::chrono::time_point<std::chrono::steady_clock> StartTime, EndTime, CycleStartTime, PrevCycleStartTime;

float time_data[10] = { 0,0,0,0,0,0,0,0,0,0 };

inline void update_val(Robot* robot, float position_targets[6]) {
	robot->RobotJoint[0] = position_targets[0];
	robot->RobotJoint[1] = position_targets[1];
	robot->RobotJoint[2] = position_targets[2];
	robot->RobotJoint[3] = position_targets[3];
	robot->RobotJoint[4] = position_targets[4];
	robot->RobotJoint[5] = position_targets[5];
}


inline void update_val (int count, Robot* sysRobot1) {
	sysRobot1->RobotJoint[0] = position_targets_rob_1[0];
	sysRobot1->RobotJoint[1] = position_targets_rob_1[1];
	sysRobot1->RobotJoint[2] = position_targets_rob_1[2];
	sysRobot1->RobotJoint[3] = position_targets_rob_1[3];
	sysRobot1->RobotJoint[4] = position_targets_rob_1[4];
	sysRobot1->RobotJoint[5] = position_targets_rob_1[5];
}

inline void initUDPReceiver(int port) {
	// create socket to listen from Robot
	sock = ::socket(AF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in serverAddr;
	memset(&serverAddr, sizeof(serverAddr), 0);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddr.sin_port = htons(port);
	// listen on all interfaces
	::bind(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr));

	std::cout << "UDP Receiver setup on port (" << port << ")." << std::endl;
}

inline void receive_external_data() {
	sockaddr_in Addr;
	int len_receive = sizeof(sockaddr);
	char buffer[1400];
	while (receive_external){
		// receive message from external apps
		int n = recvfrom(sock, buffer, 1400, 0, (struct sockaddr*)&Addr, &len_receive);
		if (n < 0)
		{
			printf("Error receiving message\n");
		}
		else {
			// parse data
			//std::cout << ".";
			
			Json::Reader reader;
			Json::Value value;
			
			if (reader.parse(buffer, value)) {
				auto position_targets_gantry = value["gantry"];
				auto position_targets_ABB_0 = value["/World/ABB_0"];
				auto position_targets_ABB_1 = value["/World/ABB_1"];
				auto position_targets_ABB_2 = value["/World/ABB_2"];
				auto position_targets_ABB_3 = value["/World/ABB_3"];

				int i = 0;
				//cout << "[";
				for (auto val : position_targets_ABB_0) {
					position_targets_rob_1[i] = val.asFloat();
					//cout << position_targets_rob_1[i] << ",";
					i++;					
				}
				//cout << "]" << endl;
				i = 0;
				for (auto val : position_targets_ABB_1) {
					position_targets_rob_2[i] = val.asFloat();
					i++;
				}
				i = 0;
				for (auto val : position_targets_ABB_2) {
					position_targets_rob_3[i] = val.asFloat();
					i++;
				}
				i = 0;
				for (auto val : position_targets_ABB_3) {
					position_targets_rob_4[i] = val.asFloat();
					i++;
				}
				
			}
			else {
				cout << reader.getFormattedErrorMessages();
			}
		}
	}
}


int main(int argc, char* argv[]) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;

	initWinSock();

	initUDPReceiver(11999);
	std::thread UDPReceiveThread(&receive_external_data);

#ifdef TEST_C11
	Robot* Rob1 = new  Robot(ROB1_Name, ROB1_PORT);
	Robot* Rob2 = new  Robot(ROB2_Name, ROB2_PORT);
	std::thread Robot1InitThread(&Robot::initRobot, Rob1);
	std::thread Robot2InitThread(&Robot::initRobot, Rob2);
	Robot1InitThread.join();
	Robot2InitThread.join();
	while ((std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() -
		Rob1->StreamStartTime).count() / 1e6) <= sampling_time) {
	}
#endif // TEST_C11	

#ifdef TEST_C21
	Robot* Rob3 = new  Robot(ROB3_Name, ROB3_PORT);
	Robot* Rob4 = new  Robot(ROB4_Name, ROB4_PORT);
	std::thread Robot3InitThread(&Robot::initRobot, Rob3);
	std::thread Robot4InitThread(&Robot::initRobot, Rob4);
	Robot3InitThread.join();
	Robot4InitThread.join();
	while ((std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() -
		Rob3->StreamStartTime).count() / 1e6) <= sampling_time) {
	}
#endif // TEST_C21

	std::cout << "Done Initializing Robot Threads." << std::endl;
	
	PrevCycleStartTime = CycleStartTime = StartTime = std::chrono::steady_clock::now();

	int i = 0;
	while (true){

		// Read feedback through protobuf
		//std::thread ReadRobot1([](Robot* Rob1, int i, float* TimeData, bool IsReadable) {Rob1->FeedbackCycleJoint(i, TimeData, IsReadable); },Rob1,i,&time_data[1],IsReadable);
		
		// wait for readthreads to join
		//ReadRobot1.join();

		CycleStartTime = std::chrono::steady_clock::now();
		time_data[0] = float(std::chrono::duration_cast<std::chrono::nanoseconds>(CycleStartTime - PrevCycleStartTime).count() / 1e6);
		PrevCycleStartTime = CycleStartTime;

#ifdef TEST_C11
		update_val(Rob1, position_targets_rob_1);
		update_val(Rob2, position_targets_rob_2);
#endif // TEST_C11

#ifdef TEST_C21
		update_val(Rob3, position_targets_rob_3);
		update_val(Rob4, position_targets_rob_4);
#endif // TEST_C21

		// Send data through Protobuf
#ifdef TEST_C11
		std::thread WriteRobot1([](Robot* Rob1, float* TimeData) {Rob1->WriteCycleJoint(TimeData); }, Rob1, &time_data[2]);
		std::thread WriteRobot2([](Robot* Rob2, float* TimeData) {Rob2->WriteCycleJoint(TimeData); }, Rob2, &time_data[2]);
#endif // TEST_C11

#ifdef TEST_C21
		std::thread WriteRobot3([](Robot* Rob3, float* TimeData) {Rob3->WriteCycleJoint(TimeData); }, Rob3, &time_data[2]);
		std::thread WriteRobot4([](Robot* Rob4, float* TimeData) {Rob4->WriteCycleJoint(TimeData); }, Rob4, &time_data[2]);
#endif // TEST_C21
		

		// wait for read and write threads to join
#ifdef TEST_C11
		WriteRobot1.join();
		WriteRobot2.join();
#endif // TEST_C11		
		
#ifdef TEST_C21
		WriteRobot3.join();
		WriteRobot4.join();
#endif // TEST_C21		

		i++;
	}
	receive_external = false;
	UDPReceiveThread.join();

	EndTime = std::chrono::steady_clock::now();
	std::cout << std::endl <<"Expected Runtime:" << ((msg_count*sampling_time)/1000.0) << std::endl <<
		"Execution Time:" << std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime).count() / 1e6 << " seconds" << std::endl;

#ifdef TEST_C11
	delete Rob1;
	delete Rob2;
#endif // TEST_C11	

#ifdef TEST_C21
	delete Rob3;
	delete Rob4;
#endif // TEST_C21

	exit(0);
	return 0;
}