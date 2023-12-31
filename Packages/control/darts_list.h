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

#define R_1_15828 {0, 1, 6.0, 15.828, 5900.0f} // 实测数据
#define R_1_16030 {0, 1, 6.0, 16.030, 5940.0f}
#define R_1_16267 {0, 1, 6.0, 16.267, 5950.0f}
#define R_1_16463 {0, 1, 6.0, 16.460, 6000.0f}
#define R_1_16650 {0, 1, 6.0, 16.650, 6060.0f}
#define R_1_16852 {0, 1, 6.0, 16.852, 6100.0f}
#define R_1_17077 {0, 1, 6.0, 17.077, 6150.0f}
#define R_1_19969 {0, 1, 6.0, 19.969, 6650.0f}

#define C_1_15828 {0, 1, -0.3, 15.828, 5840.0f} // 实测数据
#define C_1_16030 {0, 1, -0.2, 16.020, 5880.0f}
#define C_1_16267 {0, 1, -0.2, 16.263, 5920.0f}
#define C_1_16463 {0, 1, -0.0, 16.462, 5960.0f}
#define C_1_16650 {0, 1, -0.1, 16.655, 6000.0f}
#define C_1_16852 {0, 1, -0.1, 16.854, 6040.0f}
#define C_1_17077 {0, 1, -0.1, 17.077, 6080.0f}

const dart_struct outpost_distance_list[][distance_list_lence] = {{C_1_15828, C_1_16030, C_1_16267, C_1_16463, C_1_16650, C_1_16852, C_1_17077},
                                                            };

#define B_2_25318 {1, 2, -75.0, 25.277, 7800.0f} // 实测数据 20:41
#define B_0_25423 {1, 0, -0.5, 25.423, 7500.0f}
#define B_1_25423 {1, 1, -0.3, 25.423, 7680.0f}
#define R_0_25423 {0, 0, -0.3, 25.423, 7570.0f} // 仅该项估计
#define R_1_25423 {0, 1, -0.4, 25.423, 7520.0f}
#define R_2_25423 {0, 2, -0.3, 25.423, 7800.0f} // pass // 可能是湿度过大导致需要更高转速
#define R_3_25423 {0, 3, -0.9, 25.423, 7230.0f} // ABS

#define R_1_25223 {0, 1, -0.4, 25.223, 7490.0f} // 解算数据
#define R_1_25423 {0, 1, -0.4, 25.423, 7520.0f}
#define R_1_25623 {0, 1, -0.4, 25.623, 7560.0f}
#define R_1_25823 {0, 1, -0.4, 25.823, 7590.0f}
#define R_1_26023 {0, 1, -0.4, 26.023, 7630.0f}
#define R_1_26223 {0, 1, -0.4, 26.023, 7670.0f}
#define R_1_26423 {0, 1, -0.4, 26.023, 7700.0f}

#define C_1_25223 {0, 1, -1.0, (25.223-9.046), 7360.0f} // 实测数据
#define C_1_25423 {0, 1, -1.0, (25.426-9.046), 7410.0f}
#define C_1_25623 {0, 1, -1.0, (25.626-9.046), 7440.0f}
#define C_1_25823 {0, 1, -0.8, (25.820-9.046), 7470.0f}
#define C_1_26023 {0, 1, -0.8, (26.027-9.046), 7500.0f}
#define C_1_26223 {0, 1, -0.8, (26.224-9.046), 7530.0f}
#define C_1_26423 {0, 1, -0.7, (26.427-9.046), 7560.0f}

const dart_struct base_distance_list[][distance_list_lence] = {{C_1_25223, C_1_25423, C_1_25623, C_1_25823, C_1_26023, C_1_26223, C_1_26423},
                                                         };

#endif // !DARTS_LIST_H