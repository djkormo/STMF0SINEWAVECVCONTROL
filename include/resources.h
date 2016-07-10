#ifndef RESOURCES_H_
#define RESOURCES_H_

#include <stdint.h>

extern const uint16_t Sine32_12bit [32];

extern const uint16_t Sine1024_12bit [1024];


extern int rangeScaleLinear
(int x,
		int x_min,
		int x_max,
		int y_min,
		int y_max);


extern float rangeScaleVoltPerOclave
(float v,
		float v_min,
		float x_max,
		float y_min,
		float y_max);




#endif /* RESOURCES_H_ */
