```cpp
#include <iostream>
#include "Filter.h"

int main() {
    // LPF 설정
    double lpf_alpha = 0.1;
    LPF lpf(lpf_alpha);

    // HPF 설정
    double hpf_alpha = 0.1;
    HPF hpf(hpf_alpha);

    // KalmanFilter 설정
    double process_noise = 1e-5;
    double measurement_noise = 1e-1;
    double estimated_error = 1.0;
    double initial_value = 0.0;
    KalmanFilter kf(process_noise, measurement_noise, estimated_error, initial_value);

    // 필터링할 샘플 데이터
    double data[] = {1.0, 2.0, 3.0, 2.0, 1.0, 0.0, -1.0, -2.0, -3.0};
    int dataSize = sizeof(data) / sizeof(data[0]);

    std::cout << "Input Data: ";
    for (int i = 0; i < dataSize; ++i) {
        std::cout << data[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "LPF Output: ";
    for (int i = 0; i < dataSize; ++i) {
        double filteredValue = lpf.apply(data[i]);
        std::cout << filteredValue << " ";
    }
    std::cout << std::endl;

    std::cout << "HPF Output: ";
    for (int i = 0; i < dataSize; ++i) {
        double filteredValue = hpf.apply(data[i]);
        std::cout << filteredValue << " ";
    }
    std::cout << std::endl;

    std::cout << "Kalman Filter Output: ";
    for (int i = 0; i < dataSize; ++i) {
        double filteredValue = kf.apply(data[i]);
        std::cout << filteredValue << " ";
    }
    std::cout << std::endl;

    return 0;
}
```
