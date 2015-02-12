void write_time(uint16_t *day, uint16_t *hour, uint16_t *min, uint16_t *sec, uint16_t timer) {
    *day  = (timer/86400) + 1;
    *hour = (timer%86400) / 3600;
    *min  = (timer%3600)  / 60;
    *sec  = (timer%60);
}
