//
// Created by bismarck on 11/15/22.
//

#ifndef RM2022ENGINEER_CONFIG_H
#define RM2022ENGINEER_CONFIG_H

// 注：所有误差类参数均使用两值中较小值进行归一化（ diff = (v1 - v2) / min(v1, v2) ）

// 三角形识别参数
#define MIN_ANGLE 70            // 三角形中最大角的最小角度值
#define MAX_LENGTH_DIFF 1       // 三角形两腰最大允许误差
#define MIN_AERO 30             // 三角形最小面积

// 辅助点识别参数
#define MIN_AUX_POINT_AERO 4    // 辅助点最小面积

// 辅助点匹配参数
#define AUX_MIN_AERO 0.15       // 辅助点面积与该角面积的比值的最小值
#define AUX_MAX_AERO 0.5        // 辅助点面积与该角面积的比值的最大值
#define AUX_MAX_ANGLE 20        // 辅助点到角点连线与该边夹角的最大值
#define AUX_MIN_LENGTH 3        // 辅助点到角点的长度的最小值（除以`sqrt(aero)`归一化）
#define AUX_MAX_LENGTH 7        // 辅助点到角点的长度的最大值

// 边识别参数
#define MAX_TRIANGLE_EDGE_ANGLE_DIFF 20     // 两三角形直角边间最大允许角度
#define MAX_EDGE_TRIANGLE_DIFF 15           // 三角形直角边与构成边线间最大允许角度
#define MAX_AERO_DIFF 4.5                   // 两三角形contour面积的差的最大允许值

// 正方形投影识别参数
#define MAX_EDGE_ANGLE_DIFF 10  // 正方形投影至少有两边平行，两平行边间的夹角最大值
#define MIN_EDGE_RATE 0.6       // 两平行边间最大长度与最小长度比值的最小值
#define MIN_AUX_FOUND 1         // 至少需要匹配到的辅助点个数
#define MIN_SQUARE_AERO 200     // 最小正方形投影面积

#endif //RM2022ENGINEER_CONFIG_H
