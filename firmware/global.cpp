efitick_t getTimeNowNt() {
    return chVTGetTimeStamp();
}

efitimeus_t getTimeNowUs() {
    return NT2US(getTimeNowNt());
}