// assume timefmt is 8 chars, [ss.mmmm '\0']
void fill_time(char *timefmt, uint16_t timer) {
    timefmt[0] = '0'+(timer / 25000);
    timer %= 25000;
    timefmt[1] = '0'+(timer / 2500);
    timer %= 2500;

    timefmt[3] = '0'+(timer / 250);
    timer %= 250;
    timefmt[4] = '0'+(timer / 25);
    timer %= 25;
    timefmt[5] = '0'+(timer / 2);
    timer %= 2;
    timefmt[5] = '0';
}

