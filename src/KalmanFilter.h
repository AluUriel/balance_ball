class KalmanFilter {
private:
    float xhat, P, xhatminus, Pminus, K;
    float q, r;  // Process noise and measurement noise variance

public:
    KalmanFilter(float process_noise, float measurement_noise)
        : q(process_noise), r(measurement_noise) {
        xhat = 0.0;
        P = 1.0;
    }

    void setProcessNoise(float process_noise) {
        q = process_noise;
    }

    void setMeasurementNoise(float measurement_noise) {
        r = measurement_noise;
    }

    float update(float measurement) {
        // Time update
        xhatminus = xhat;
        Pminus = P + q;

        // Measurement update
        K = Pminus / (Pminus + r);
        xhat = xhatminus + K * (measurement - xhatminus);
        P = (1 - K) * Pminus;

        return xhat;
    }

    float getCurrentEstimate() const {
        return xhat;
    }
};