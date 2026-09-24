/**
 * @file    app.h
 * @brief   Application entry: module init and the cooperative task scheduler.
 */
#ifndef APP_H
#define APP_H

void App_Init(void);    // Call once after all MX_..._Init()
void App_Loop(void);    // Call forever from the main while(1)

#endif /* APP_H */
