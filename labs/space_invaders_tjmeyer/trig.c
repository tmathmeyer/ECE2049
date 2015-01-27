/*
 * trig.c
 *
 *  Created on: Jan 26, 2015
 *      Author: tjmeyer
 */

#define absd(x, y) (y>x?y-x:x-y)
#define mosc(i, s, m) (m*(1-absd(1, 2*(i%s)/s)))

