#ifndef _YUV_OUTPUT_H_
#define _YUV_OUTPUT_H_

#include <cstdint>

void set_resolution(uint32_t width, uint32_t height);
void feeding_data(char *pyuv, uint32_t length);
int output_main(void) ;
void output_main_stop(void) ;

#endif /* _YUV_OUTPUT_H_ */
