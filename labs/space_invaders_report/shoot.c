void shoot_enemies(char mask) {
    char a = enemies[0] & (~mask);
    char b = enemies[1] & (~mask | enemies[0]);
    char c = enemies[2] & (~mask | enemies[1]);

    enemies[0] = a;
    enemies[1] = b;
    enemies[2] = c;
}

