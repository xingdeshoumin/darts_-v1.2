#ifndef DARTS_LIST_H
#define DARTS_LIST_H

#include "struct_typedef.h"
#include "chassis_behavior.h"

#define distance_list_lence 7

#define R_1 {1, 1, -75.0, 7350.0f, 0.0}
#define R_4 {1, 4, -250.0, 7450.0f, 0.0}
#define R_5 {1, 5, 100.0, 7390.0f, 0.0}		// ok -
#define R_7 {1, 7, -550.0, 7400.0f, 0.0}	// 464
#define B_1 {0, 1, -200.0, 7410.0f, 0.0}	// ok	// -180
#define B_2 {0, 2, 0.0, 7380.0f, 0.0}		// ok - 
#define B_4 {0, 4, -300.0, 7550.0f, 0.0}	// -264.0

const dart_struct dart_list[3] = {B_1, B_2, R_5};

#define B_0_16265 {0, 1, 6.2, 16.265, 6000.0f}

#define R_1_15828 {0, 1, 6.0, 15.828, 5900.0f} // Êµ²âÊý¾Ý
#define R_1_16030 {0, 1, 6.0, 16.030, 5940.0f}
#define R_1_16267 {0, 1, 6.0, 16.267, 5950.0f}
#define R_1_16463 {0, 1, 6.0, 16.460, 6000.0f}
#define R_1_16650 {0, 1, 6.0, 16.650, 6060.0f}
#define R_1_16852 {0, 1, 6.0, 16.852, 6100.0f}
#define R_1_17077 {0, 1, 6.0, 17.077, 6150.0f}
#define R_1_19969 {0, 1, 6.0, 19.969, 6650.0f}

const dart_struct outpost_distance_list[][distance_list_lence] = {{R_1_15828, R_1_16030, R_1_16267, R_1_16463, R_1_16650, R_1_16852, R_1_17077},
                                                            };

#define B_1_25410 {0, 1, -75.0, 25.410, 7450.0f}
#define B_1_25610 {0, 1, -75.0, 25.610, 7550.0f}
#define B_1_25810 {0, 1, -75.0, 25.810, 7650.0f}

const dart_struct base_distance_list[][distance_list_lence] = {{B_1_25410, B_1_25610, B_1_25810, B_1_25810, B_1_25810, B_1_25810, B_1_25810},
                                                         };

#endif // !DARTS_LIST_H