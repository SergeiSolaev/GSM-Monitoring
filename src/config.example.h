#pragma once
#include <Arduino.h>

// Основной номер
const char PHONE_NUMBER[] = "+1234567890";

// Белый список
const char *WHITELISTED_NUMBERS[] = {
    "+1234567890",
    "+2345678901",
};

const uint8_t WHITELISTED_NUMBERS_COUNT =
    sizeof(WHITELISTED_NUMBERS) /
    sizeof(WHITELISTED_NUMBERS[0]);