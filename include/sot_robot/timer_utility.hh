#ifndef TIMER_UTILITY_HH
#define TIMER_UTILITY_HH

#include <cstdio>
#include <cstddef>
#include <fstream>
#include <sys/time.h>



namespace timer
{
class Timer
{
public:
  Timer();
  ~Timer();

  void start();                               ///< start timer
  void stop();                                ///< stop the timer
  double getElapsedTime();                    ///< get elapsed time in milli-second
  double getElapsedTimeInSec();               ///< get elapsed time in second (same as getElapsedTime)
  double getElapsedTimeInMilliSec();          ///< get elapsed time in milli-second
  double getElapsedTimeInMicroSec();          ///< get elapsed time in micro-second

private:
  double startTimeInMicroSec;                 ///< starting time in micro-second
  double endTimeInMicroSec;                   ///< ending time in micro-second
  int stopped;                                ///< stop flag
#ifdef _WIN32
  LARGE_INTEGER frequency;                    ///< ticks per second
  LARGE_INTEGER startCount;
  LARGE_INTEGER endCount;
#else
  timeval startCount;
  timeval endCount;
#endif
};

}
#endif
