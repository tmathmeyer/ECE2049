#include "msp430.h"

int main() {
	setupP4();
	setupP5();

	while(1) {
		InOut();
	}
}