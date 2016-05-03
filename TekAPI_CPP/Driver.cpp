#include "Driver.h"
#include <iostream>
#include <cmath>
using namespace sp;

static const double pi = acos(-1.0);

sp::Motor::Motor(){

}

sp::Motor::~Motor(){

}

void sp::Motor::setInterpolation(int Pmin, int Pmax, double Rmin, double Rmax){
	_Pmin = Pmin;
	_Pmax = Pmax;
	_Rmin = Rmin;
	_Rmax = Rmax;
}
void sp::Motor::operator =(const double& Rref){
	if (Rref > _Rmax){
		_Pref = interpolation(_Rmax);
	}
	else if (Rref < _Rmin){
		_Pref = interpolation(_Rmin);
	}
	else{
		_Pref = interpolation(Rref);
	}
}
int sp::Motor::interpolation(const double& Rref){
	return (int)(((Rref-_Rmin)*(_Pmax-_Pmin)/(_Rmax-_Rmin))+_Pmin);
}

const int& sp::Motor::getPref() const{
	return _Pref;
}

sp::Driver::Driver(){
}

sp::Driver::Driver(int i){
	_motor.reserve(i);
	for (int j = 0; j < i; ++j){
		_motor.push_back(new Motor());
	}
}

sp::Driver::~Driver(){
	for (int i = 0, n = _motor.size(); i < n; ++i){
		delete _motor[i];
	}
}
int sp::Driver::size() const{
	return _motor.size();
}
void sp::Driver::addMotor(){
	_motor.push_back(new Motor());
}

const Motor& sp::Driver::operator[](int i) const{
	return *_motor[i];
}

Motor& sp::Driver::operator[](int i){
	return *_motor[i];
}

sp::Arduino::Arduino(){
	_serial = 0;
}
sp::Arduino::~Arduino(){
	for (int i = 0, n = _driver.size(); i < n; ++i){
		delete _driver[i];
	}
	delete _serial;
}
sp::Arduino::Arduino(char* comPort){
	_serial = new Serial(comPort);
}

bool sp::Arduino::setComPort(char* comPort){
	_serial = new Serial(comPort);
	return _serial->IsConnected();
}

bool sp::Arduino::isConnected() const{
	return _serial->IsConnected();
}

int sp::Arduino::driverSize() const{
	return _driver.size();
}

int sp::Arduino::motorSize() const{
	int total = 0;
	for (int i = 0, n = _driver.size(); i < n; ++i){
		total += _driver[i]->size();
	}
	return total;
}

Motor& sp::Arduino::operator()(int i){
	int j = 0;
	for (int k = 0, n = _driver.size(); k < n; ++k){
		j = _driver[k]->size();
		if (j > i){
			return (*this)[k][i];
		}
		else{
			i = i - j;
		}
	}
	return (*this)[0][0];
}

const Driver& sp::Arduino::operator[](int i) const{
	return *_driver[i];
}

Driver& sp::Arduino::operator[](int i){
	return *_driver[i];
}

void sp::Arduino::addDriver(int i){
	_driver.push_back(new Driver(i));
}

void sp::Arduino::sendData(unsigned char* buffer, unsigned int nbByte){
	_serial->WriteData(buffer, nbByte);
}

DWORD sp::spThreadID;
HANDLE sp::spHandle;
int ePathIndex = 0;

DWORD WINAPI sp::spThread(LPVOID lpParameter){
	TIMECAPS tc;
	
	// 取得硬體timer設定資源
	if (timeGetDevCaps(&tc, sizeof(TIMECAPS)) != TIMERR_NOERROR)
	{
		// Error; application can't continue.  
		return 0;
	}
	
	unsigned int wTimerRes = min(max(tc.wPeriodMin, TARGET_RESOLUTION), tc.wPeriodMax);

	timeBeginPeriod(wTimerRes);

	unsigned int Period_time = 5; // 觸發週期 (millisecond)

	unsigned int timeSetEventRslt = timeSetEvent(
		Period_time, //觸發的週期時間
		TARGET_RESOLUTION, //timer最小時間精度
		sp::spTimer, //Callback function 名稱
		wTimerRes, //存放用戶提供的回調數據
		TIME_PERIODIC | TIME_CALLBACK_FUNCTION | TIME_KILL_SYNCHRONOUS // 設定timer的相關參數flag
		);

	if (timeSetEventRslt == NULL)
		MessageBox(NULL, "TimeSetEvent fail. Maybe delay is not in the range of the minimum and maximum ", "錯誤", MB_OK);
	
	while (1){

	}

	timeEndPeriod(wTimerRes);         //對應timeBeginPeriod  
	timeKillEvent(timeSetEventRslt);  //對應timeSetEventRslt  
	// index & flag 歸零
	ePathIndex = 0;
	threadUsingFrag = 0;
	return 0;
}

void CALLBACK sp::spTimer(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2){
	static unsigned char buffer[14] = { 0 };
	for (int i = 0, n = ePath_L.size(); i < n; ++i){
		buffer[2 * i] = ePath_L[i] >> 8;
		buffer[2 * i + 1] = ePath_L[i];
	}
	mega.sendData(buffer, 14);
}

Arduino sp::mega;

void sp::initMega(){
	mega.addDriver(1);
	mega[0][0].setInterpolation(0, 2522, 0, pi / 2);
	mega.addDriver(2);
	mega[1][0].setInterpolation(991, 680, 0, pi / 2);
	mega[1][1].setInterpolation(0, 2126, 0, pi * 99 / 180);
	mega.addDriver(2);
	mega[2][0].setInterpolation(962, 558, 0, pi / 2);
	mega[2][1].setInterpolation(0, 2328, 0, pi * 99 / 180);
	mega.addDriver(2);
	mega[3][0].setInterpolation(978, 622, 0, pi / 2);
	mega[3][1].setInterpolation(0, 2170, 0, pi * 99 / 180);
}

void sp::pathR2E(const std::vector<std::vector<double>>& path, std::vector<std::vector<int>>& ePath){
	ePath.clear();
	for (int i = 0, n = path.size(); i < n; ++i){
		ePath.push_back(std::vector<int>());
		ePath[i].reserve(mega.motorSize());
		for (int j = 0, m = path[i].size(); j < m; ++j){
			mega(j) = path[i][j];
			ePath[i].push_back(mega(j).getPref());
		}
	}
}

void sp::pathR2E(const Eigen::MatrixXd& path, std::vector<std::vector<int>>& ePath){
	ePath.clear();
	for (int i = 0, n = path.rows(); i < n; ++i){
		ePath.push_back(std::vector<int>());
		ePath[i].reserve(mega.motorSize());
		for (int j = 0, m = path.cols(); j < m; ++j){
			mega(j) = path(i, j);
			ePath[i].push_back(mega(j).getPref());
		}
	}
}

std::vector<std::vector<int>> sp::ePath;

void sp::megaSendPath(){
	if (threadUsingFrag == 0){
		spHandle = CreateThread(0, 0, spThread, 0, 0, &spThreadID);
		threadUsingFrag = 1;
	}
}

bool sp::threadUsingFrag = 0;

std::vector<double> sp::path_Ref;			////軌跡1*7 (角度)
std::vector<int> sp::ePath_L;			////軌跡1*14
std::vector<double> sp::Pressure_References;	////參考壓力 1*7

void sp::InitPar()
{
	for (int i = 0; i < 7; i++){
		sp::Pressure_References.push_back(0.002);
		sp::path_Ref.push_back(0);
	}
	for (int i = 0; i < 7; i++){
		sp::ePath_L.push_back(0);
	}
	sp::path_Ref[0] = pi / 180 * 90;	//// 角度換弧度
}

void sp::pathR2E_1D(std::vector<double>& path_Ref, std::vector<int>& ePath_L){
	ePath_L.clear();
	for (int i = 0, n = sp::path_Ref.size(); i < n; ++i){
			mega(i) = path_Ref[i];
			ePath_L.push_back(mega(i).getPref());
	}
}