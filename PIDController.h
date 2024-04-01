class PIDController {
private:
    float Kp, Ki, Kd;
    float prevError;
    float integral;
    unsigned long prevTime;
    unsigned long sampleTime;

public:
    PIDController(float Kp, float Ki, float Kd, unsigned long sampleTime) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->sampleTime = sampleTime;
        prevError = 0;
        integral = 0;
        prevTime = millis();
    }

    float compute(float setpoint, float input) {
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - prevTime;
        if (elapsedTime >= sampleTime) {
            float error = setpoint - input;
            integral += error * elapsedTime;
            float derivative = (error - prevError) / elapsedTime;
            float output = Kp * error + Ki * integral + Kd * derivative;
            prevError = error;
            prevTime = currentTime;
            return output;
        }
        return 0; // If sample time not reached, return zero output
    }
};
