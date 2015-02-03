void InOut() {
	char val = (P4IN >> 3) & 0x0F;
	if (val >= 8) {
		P3OUT |= (~(val<<1) & 0x1E);
	} else {
		P3OUT |= ((val<<1) & 0x1E);
	}
}