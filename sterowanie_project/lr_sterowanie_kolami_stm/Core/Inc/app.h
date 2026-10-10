/**
  * @file    app.h
  * @brief   Wheel-control application. Called from USER CODE blocks in main.c.
  */
#ifndef APP_H
#define APP_H

/* Call once after all MX_*_Init(). */
void app_init(void);
/* Call on every pass of the main loop. */
void app_loop(void);

#endif /* APP_H */
