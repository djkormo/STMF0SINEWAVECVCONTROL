/*
 * algorithm.c
 *
 *  Created on: 11 lip 2016
 *      Author: kormo
 */
//#include "algorithm.h"

/*
Zak�adaj�c, �e mamy warto�� x z przedzia�u [x_min, x_max]
i chcemy j� przenie�� do [y_min, y_max], mo�na skorzysta� ze wzoru:

*/
int rangeScaleLinear
(int x,
		int x_min,
		int x_max,
		int y_min,
		int y_max)

{
	return (int) y_min+(x-x_min)*(y_max-y_min)/(x_max-x_min);
};


extern float rangeScaleVoltPerOclave
(float v,
		float v_min,
		float x_max,
		float y_min,
		float y_max)

{

	return 0.0;

};
