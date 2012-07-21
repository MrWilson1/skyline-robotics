#include "logging.h"

template <class T, class E>
void EhLog(T value, E end)
{
	std::ofstream file;
	file.open("spartabots_log.txt", std::ios::app);
	file << value << end;
	file.close();
}

template <class T>
void EhLog(T value)
{
	Log(value, "\n");
}

