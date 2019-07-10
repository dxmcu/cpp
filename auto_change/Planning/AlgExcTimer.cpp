#include "AlgExcTimer.h"

AlgExcTimer::AlgExcTimer() :m_begen(res::zero()),
m_end(res::zero())
{

}

void AlgExcTimer::StartTimer()
{
	m_begen = timeclock::now();
}

void AlgExcTimer::StopTimer()
{
	m_end = timeclock::now();
}

double AlgExcTimer::GetAlgExcTime()
{
	int64_t time;
	time = std::chrono::duration_cast<res>(m_end - m_begen).count();
	return (double)time / 1000;
}
