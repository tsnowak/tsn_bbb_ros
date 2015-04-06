/*
 * SimpleTimer.h
 *
 *  Created on: Mar 30, 2014
 *      Author: nagnanamus
 */

#ifndef SIMPLETIMER_H_
#define SIMPLETIMER_H_

#include <time.h>
#include <sys/time.h>
class SimpleTimer {
private:
	double Tsec,T0sec;
	struct timeval tv;
	struct timezone tz;
public:
	void ResetTimer(){
		gettimeofday(&tv,&tz);
		T0sec=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;
	}
	void SetTsec(double tsec){
		Tsec=tsec;
	}
	/*
	 *  returns false if it crosses the set timer value
	 *  so use it as while(CheckTimer()){}
	 */
	bool CheckTimer(){
		double tt;
		gettimeofday(&tv,&tz);
		tt=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;
		if ((tt-T0sec)>=Tsec)
		{return false;}
		else
		{return true;}
	}

	double GetTime_from_T0sec(){
		double tt;
		gettimeofday(&tv,&tz);
		tt=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;
	    return (tt-T0sec);
	}
};


#endif /* SIMPLETIMER_H_ */
