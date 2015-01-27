#include <msp430.h>
#include <stdint.h>
#include "inc\hw_memmap.h"
#include "driverlibHeaders.h"
#include "CTS_Layer.h"
#include "grlib.h"
#include "LcdDriver/Dogs102x64_UC1701.h"
#include "peripherals.h"
#include "tjmeyer.h"

void shitty_function(struct G_FIG *figures, char A, char B, char FUCK, char D, char E);
char *scroll_text(char *text, int chars);
void display_menu(int which);
void countdown();
void game();
void shitty_timer(int sex);



char ctd[2] = {'3', 0};
int state;


struct G_FIG SPACE_INVADER_1[2];
struct G_FIG BLANK_SPACE[2];


void init_figures() {
	// space invader 1
	shitty_function(SPACE_INVADER_1+0, 30, 184, 125, 54, 62);
	shitty_function(SPACE_INVADER_1+1, 62, 54, 125, 184, 30);

	// empty space
	shitty_function(BLANK_SPACE+0, 0, 0, 0, 0, 0);
	shitty_function(BLANK_SPACE+1, 0, 0, 0, 0, 0);
}



void main(void) {
    state = 0;
	WDTCTL = WDTPW | WDTHOLD;

	init_figures();

    configDisplay();
	set_display_context((&g_sContext)->pDisplay);


    configTouchPadLEDs();
    configCapButtons();



    P1SEL = P1SEL & ~BIT0;
    P1DIR |= BIT0;













    while(1) {
    	switch(state/2) {
    		case 0: display_menu(state%2); break;
    		case 1: countdown(); break;
    		case 2: game(); break;
    	}


		GrFlush(&g_sContext);
    }
}



uint b_p = 0;
void draw_row_of_invaders(uint ylevel, char enemy_map) {
	int ost = 32-oscilate(&b_p);
	fill_region(0, ylevel, SCREENWIDTH, PIXMAP_HEIGHT, NORMAL);

	if (enemy_map & 0x01) {
		draw_buffer(SPACE_INVADER_1, 2, 00+ost, ylevel, NORMAL, 0);
	}

	if (enemy_map & 0x02) {
		draw_buffer(SPACE_INVADER_1, 2, 15+ost, ylevel, NORMAL, 0);
	}

	if (enemy_map & 0x04) {
		draw_buffer(SPACE_INVADER_1, 2, 30+ost, ylevel, NORMAL, 0);
	}

	if (enemy_map & 0x08) {
		draw_buffer(SPACE_INVADER_1, 2, 45+ost, ylevel, NORMAL, 0);
	}

	if (enemy_map & 0x10) {
		draw_buffer(SPACE_INVADER_1, 2, 60+ost, ylevel, NORMAL, 0);
	}
}

char enemies[3] = {31, 31, 31};
void game() {
	b_p++;

	switch(b_p / 32) {
		case 2: draw_row_of_invaders(30, enemies[2]);
		case 1: draw_row_of_invaders(20, enemies[1]);
		case 0: draw_row_of_invaders(10, enemies[0]);
	}

	if (CapButtonRead()<<1 == LED8) {
		enemies[0] = enemies[0]? enemies[0]<<1 : 31;
	}



}

void countdown() {
	if (ctd[0] > '0') {
		GrStringDrawCentered(&g_sContext, ctd, AUTO_STRING_LENGTH, 51, 16, 1);
		shitty_timer(2);
		ctd[0]--;
		return;
	}
	GrClearDisplay(&g_sContext);
	state = 4;
}

uint flipflop = 0;
void display_menu(int which) {
	if (!which) {
		P1OUT |= LED8;   // turn on the start LED

		GrStringDrawCentered(&g_sContext, "space invaders", AUTO_STRING_LENGTH, 51, 16, 1);
		GrStringDrawCentered(&g_sContext, "press X", AUTO_STRING_LENGTH, 51, 25, 1);
		GrStringDrawCentered(&g_sContext, "to start", AUTO_STRING_LENGTH, 51, 34, 1);
	}

	if (flipflop = 1-flipflop) {
		draw_buffer(BLANK_SPACE, 2, 00, 0, NORMAL, 0);
		draw_buffer(BLANK_SPACE, 2, 72, 0, NORMAL, 0);
		draw_buffer(SPACE_INVADER_1, 2, 10, 0, NORMAL, 0);
		draw_buffer(SPACE_INVADER_1, 2, 82, 0, NORMAL, 0);
	} else {
		draw_buffer(BLANK_SPACE, 2, 10, 0, NORMAL, 0);
		draw_buffer(BLANK_SPACE, 2, 82, 0, NORMAL, 0);
		draw_buffer(SPACE_INVADER_1, 2, 00, 0, NORMAL, 0);
		draw_buffer(SPACE_INVADER_1, 2, 72, 0, NORMAL, 0);
	}

	state = 1;
	if (CapButtonRead()<<1 == LED8) {
		GrClearDisplay(&g_sContext);
		state = 4;
	}
}

// loads data into a pixmap
void shitty_function(struct G_FIG *figures, char A, char B, char FUCK, char D, char E) {
		(figures->pixmap)[0] = A;
		(figures->pixmap)[1] = B;
		(figures->pixmap)[2] = FUCK;
		(figures->pixmap)[3] = D;
		(figures->pixmap)[4] = E;
}

// make text rotate
char *scroll_text(char *text, int chars) {
	char * dbglook = text;
	char buffer = *text;
	while(chars-->1) {
		text[0] = text[1];
	    text++;
	}
	*text = buffer;
	return dbglook;
}


void shitty_timer(int sex) {
	volatile uint i;
	while(sex --> 0) {
		for(i=0;i<50000;i++);
	}
}





