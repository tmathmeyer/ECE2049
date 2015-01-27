/*
 * GPL BBY
 *
 * tjmeyer.c
 *
 *  Created on: the birth of the universe
 *      Author: TED MEYER
 */

#include "grlib.h"
#include "tjmeyer.h"

const tDisplay *display = 0;

#define write(dsp, X, Y, char_row) (dsp->pfnPixelDraw(dsp->pvDisplayData, tlcx+X, tlcy+Y, dm^((char_row>>Y)&1)))
#define writeln(dsp, X, cr) write(dsp,X,0,cr);write(dsp,X,1,cr);write(dsp,X,2,cr);write(dsp,X,3,cr); \
							write(dsp,X,4,cr);write(dsp,X,5,cr);write(dsp,X,6,cr);write(dsp,X,7,cr);

void draw_figure(struct G_FIG fig, uint tlcx, uint tlcy, int dm) {
	writeln(display, 0, (fig.pixmap)[0]);
	writeln(display, 1, (fig.pixmap)[1]);
	writeln(display, 2, (fig.pixmap)[2]);
	writeln(display, 3, (fig.pixmap)[3]);
	writeln(display, 4, (fig.pixmap)[4]);
}


void draw_buffer(struct G_FIG *figures, uint strlen, uint tlcx, uint tlcy, int draw_method, int kspace) {
	if (!display) {
		return;
	}
	int i = 0;
	while(i < strlen) {
		draw_figure(figures[i++], tlcx, tlcy, draw_method);
		tlcx += (kspace+5);
	}
}

// range of -16 to 16
int oscilate(uint *inp) {
	switch((*inp) & 0x0F) {
		case 1: case 15: return 4;
		case 2: case 14: return 8;
		case 3: case 13: return 12;
		case 4: case 12: return 16;
		case 5: case 11: return 20;
		case 6: case 10: return 24;
		case 7: case 9: return 28;
		case 8: return 32;
	}
	return 0;
}

void fill_region(uint x, uint y, uint w, uint h, uint color) {
	if (!display) {
		return;
	}

	// TODO: loop unroll this
	while(w --> 0) {
		uint hp = h;
		while(hp --> 0) {
			display->pfnPixelDraw(display->pvDisplayData, x+w, y+hp, color);
		}
	}
}



void set_display_context(const tDisplay *display_context) {
	display = display_context;
}

