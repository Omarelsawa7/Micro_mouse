// Micromouse V8 - Arduino wrapper (thin, C++ only here)
// All logic is pure C in main.c / modules. No delay() used.

#include "main.h"

extern "C" {
#include "config.h"
}

void setup()
{
    robot_setup();
}

void loop()
{
    robot_loop();
}
