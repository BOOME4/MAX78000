// #include <stdio.h>
// #include <stdint.h>
// #include <string.h>

// #include "cnn.h"
// #include "led.h"


// int16_t Max, Min;
// #define NUM_OUTPUTS CNN_NUM_OUTPUTS // number of classes

// /* Set of detected words */
// const char keywords[NUM_OUTPUTS][10] = { "UP",    "DOWN", "LEFT",   "RIGHT", "STOP",  "GO",
//                                          "YES",   "NO",   "ON",     "OFF",   "ONE",   "TWO",
//                                          "THREE", "FOUR", "FIVE",   "SIX",   "SEVEN", "EIGHT",
//                                          "NINE",  "ZERO", "Unknown" };

// int main(void)
// {


//     //识别关键字
//         /* find detected class with max probability */
//     ret = check_inference(ml_softmax, ml_data, &out_class, &probability);

//     PR_DEBUG("----------------------------------------- \n");
//     /* Treat low confidence detections as unknown*/
//     if (!ret || out_class == NUM_OUTPUTS - 1) {
//         PR_DEBUG("Detected word: %s", "Unknown");
//     } else {
//         if(!strcmp(keywords[out_class],"YES"))  LED_On(LED_RED);
//         if(!strcmp(keywords[out_class],"NO"))  LED_Off(LED_RED);
//         PR_DEBUG("Detected word: %s (%0.1f%%)", keywords[out_class], probability);
//     }
//     PR_DEBUG("\n----------------------------------------- \n");

//     Max = 0;
//     Min = 0;
//     //------------------------------------------------------------
//     /* code */
//     return 0;
// }
