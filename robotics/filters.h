#ifndef FILTER_H
#define FILTER_H

class Filter {
	public:
	virtual double apply(double input) = 0;
};

class LPF : public Filter {
	private:
	double alpha;
	double prev_output;

	public:
	LPF(double alpha) : alpha(alpha), prev_output(0.0) {}

	double apply(double input) override {
		double output = alpha * input + (1 - alpha) * prev_output;
		prev_output = output;
		return output;
	}
};

class HPF : public Filter {
	private:
	double alpha;
	double prev_input;
	double prev_output;

	public:
	HPF(double alpha) : alpha(alpha), prev_input(0.0), prev_output(0.0) {}

	double apply(double input) override {
		double output = alpha * (prev_output + input - prev_input);
		prev_input = input;
		prev_output = output;
		return output;
	}
};

class KalmanFilter : public Filter {
	private:
	double Q; // Process noise covariance
	double R; // Measurement noise covariance
	double P; // Error covariance
	double K; // Kalman gain
	double X; // Value

	public:
	KalmanFilter(double process_noise, double measurement_noise, double estimated_error, double initial_value)
	: Q(process_noise), R(measurement_noise), P(estimated_error), X(initial_value) {}

	double apply(double measurement) override {
		// Prediction update
		P = P + Q;

		// Measurement update
		K = P / (P + R);
		X = X + K * (measurement - X);
		P = (1 - K) * P;

		return X;
	}
};

#endif // FILTER_H
