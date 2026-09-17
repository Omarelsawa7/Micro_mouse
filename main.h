#ifndef MAIN_H
#define MAIN_H

/* Entry points called from the thin Arduino wrapper.
 * Pure C, non-blocking, no delay(). */
void robot_setup(void);
void robot_loop(void);

#endif /* MAIN_H */
