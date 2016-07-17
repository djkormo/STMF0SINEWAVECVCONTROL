#ifndef ALGORITHM_H_
#define ALGORITHM_H_



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

#endif /* ALGORITHM_H_ */
