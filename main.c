// File main.c (Nâng cấp kiểm thử dải giá trị biên)
#include "Quaternion.h"

/* Khai báo hàm đặc biệt của Frama-C để tạo dải số thực biến thiên */
float Frama_C_float_interval(float min, float max);

int main() {
    Quaternion_t q1, q2, res;
    Euler_t angle;
    float v_in[3];
    float v_out[3];

    /* Giả lập q1 và q2 biến thiên liên tục từ -2.0 đến 2.0 
       Dải này chắc chắn đi qua điểm 0.0 cực kỳ nguy hiểm */
    q1.w = Frama_C_float_interval(-2.0f, 2.0f);
    q1.x = Frama_C_float_interval(-2.0f, 2.0f);
    q1.y = Frama_C_float_interval(-2.0f, 2.0f);
    q1.z = Frama_C_float_interval(-2.0f, 2.0f);

    q2.w = Frama_C_float_interval(-2.0f, 2.0f);
    q2.x = Frama_C_float_interval(-2.0f, 2.0f);
    q2.y = Frama_C_float_interval(-2.0f, 2.0f);
    q2.z = Frama_C_float_interval(-2.0f, 2.0f);

    /* Giả lập vector đầu vào biến thiên từ -10.0 đến 10.0 */
    v_in[0] = Frama_C_float_interval(-10.0f, 10.0f);
    v_in[1] = Frama_C_float_interval(-10.0f, 10.0f);
    v_in[2] = Frama_C_float_interval(-10.0f, 10.0f);

    // Chạy kiểm thử toàn bộ luồng thuật toán với dữ liệu biến thiên
    Quat_Normalize(&q1);
    Quat_Normalize(&q2);
    Quat_Conjugate(q1);
    Quat_Conjugate(q2);
    Quat_Reciprocal(&q1, &res);
    Quat_Reciprocal(&q2, &res);
    Quat_Multiply(&q1, &q2, &res);
    Quat_ToEuler(&res, &angle);
    Quat_RotateVector(&q1, v_in, v_out);

    return 0;
}