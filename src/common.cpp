/**
  ******************************************************************************
  * @file    src/common.cpp
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    10 января 2018
  * @brief   Общие классы и функции
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <new>
#include <stdio.h>
#include "common.h"
#include "hardware.h"


void *operator new(size_t size) {
	void *p = pvPortMalloc(size);
	return p;
}

void operator delete(void *p) {
	vPortFree(p);
}

void vApplicationTickHook() {

}


char *FloatToString(char *outstr, double value, int places, int minwidth, bool rightjustify) {
// this is used to write a float value to string, outstr.  oustr is also the return value.
int digit;
double tens = 0.1f;
int tenscount = 0;
double tempfloat = value;
int c = 0;
int charcount = 1;
int extra = 0;

	// make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
	// if this rounding step isn't here, the value  54.321 prints as 54.3209

	// calculate rounding term d:   0.5/pow(10,places)
	double d = 0.5;
	if (value < 0)
		d *= -1.0;
	// divide by ten for each decimal place
	for (int i = 0; i < places; i++)
		d/= 10.0;
	// this small addition, combined with truncation will round our values properly
	tempfloat +=  d;

	// first get value tens to be the large power of ten less than value
	if (value < 0)
		tempfloat *= -1.0;
	while ((tens * 10.0) <= tempfloat) {
		tens *= 10.0;
		tenscount += 1;
	}

	if (tenscount > 0)
		charcount += tenscount;
	else
		charcount += 1;

	if (value < 0)
		charcount += 1;
	charcount += 1 + places;

	minwidth += 1; // both count the null final character
	if (minwidth > charcount){
		extra = minwidth - charcount;
		charcount = minwidth;
	}

	if ((extra > 0) && rightjustify) {
		for (int i = 0; i< extra; i++) {
			outstr[c++] = ' ';
		}
	}

	// write out the negative if needed
	if (value < 0)
		outstr[c++] = '-';

	if (tenscount == 0)
		outstr[c++] = '0';

	for (int i = 0; i < tenscount; i++) {
		digit = (int) (tempfloat / tens);
		sprintf(&outstr[c++], "%d", digit);
		tempfloat = tempfloat - ((float)digit * tens);
		tens /= 10.0;
	}

	// if no places after decimal, stop now and return

	// otherwise, write the point and continue on
	if (places > 0)
		outstr[c++] = '.';


	// now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
	for (int i = 0; i < places; i++) {
		tempfloat *= 10.0;
		digit = (int) tempfloat;
		sprintf(&outstr[c++], "%d", digit);

		// once written, subtract off that digit
		tempfloat = tempfloat - (float) digit;
	}
	if (extra > 0 && (!rightjustify)) {
		for (int i = 0; i < extra; i++) {
			outstr[c++] = ' ';
		}
	}

	outstr[c++] = '\0';
	return outstr;
}
