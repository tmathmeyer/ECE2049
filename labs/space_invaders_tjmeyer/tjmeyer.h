/*
 * THIS CODE IS GPL!!!
 *
 * tjmeyer.h
 *
 *  Created on: never
 *      Author: TED MEYER
 */

#ifndef TJMEYER_H_
#define TJMEYER_H_

#include "grlib.h"

#define CHAR_ROW char
#define uint unsigned int

#define NORMAL 1
#define INVERTED 0
#define SCREENWIDTH 102
#define PIXMAP_HEIGHT 8

// custom character set defs
struct G_FIG {
	CHAR_ROW pixmap[5];
};

// custom graphics library interaction
void draw_buffer(struct G_FIG *figures, uint strlen, uint tlcx, uint tlcy, int draw_method, int kspace);
void fill_region(uint x, uint y, uint w, uint h, uint color);
void set_display_context(const tDisplay *display_context);

// math shit
int oscilate(uint *inp);

#endif
