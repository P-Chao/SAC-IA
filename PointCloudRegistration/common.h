#pragma once

#include <Windows.h>

class Timer{
public:
	void StartWatchTimer(){
		QueryPerformanceFrequency(&m_liPerfFreq);
		QueryPerformanceCounter(&m_liPerfStart);
	}

	float ReadWatchTimer(){
		char temp[64];
		QueryPerformanceCounter(&liPerfNow);
		dfTim = (((liPerfNow.QuadPart - m_liPerfStart.QuadPart) * 1000.0f) / m_liPerfFreq.QuadPart);//µ¥Î»Îªms
		return dfTim;
	}

	operator float(){
		return dfTim;
	}

protected:
	LARGE_INTEGER m_liPerfFreq;
	LARGE_INTEGER m_liPerfStart;
	LARGE_INTEGER liPerfNow;
	double dfTim;
};