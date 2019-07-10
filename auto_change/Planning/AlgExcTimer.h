#pragma once
#include <chrono>

class AlgExcTimer
{
public:
	AlgExcTimer();
	void StartTimer();		// 启动计时器
	void StopTimer();		// 停止计时器
	double GetAlgExcTime();	// 获取本次计时时间

private:
	typedef std::chrono::high_resolution_clock timeclock;
	typedef std::chrono::microseconds res;

	timeclock::time_point m_begen;
	timeclock::time_point m_end;
};
