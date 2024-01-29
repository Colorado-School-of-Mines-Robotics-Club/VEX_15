#pragma once

class Time {
	public:
	Time()
		: m_Time(0){};
	Time(long long time)
		: m_Time(time){};

	float AsSeconds() const
	{
		return (float)m_Time / 1000000;
	}

	float AsMilliseconds() const
	{
		return (float)m_Time / 1000;
	}

	float AsMicroseconds() const
	{
		return (float)m_Time;
	}

	private:
	long long m_Time; //Time in microseconds
};