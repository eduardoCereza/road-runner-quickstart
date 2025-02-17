package org.firstinspires.ftc.teamcode;

public class PIDFController {
    private double Kp, Ki, Kd, Kf;
    private double setPoint = 0, minInput = 0, maxInput = 0, minOutput = 0, maxOutput = 0;
    private double integralSum = 0, lastError = 0;
    private double theresholdPercent = 0;
    private long lastTime = System.nanoTime();

    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }

    public void setThresholdPercent(double theresholdPercent) {
        this.theresholdPercent = theresholdPercent;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public double getComputedOutput(double input) {
        double error = setPoint - input;
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1e9; // Convertendo para segundos

        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        double computedOutput = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * setPoint);

        // Limitando a saída
        computedOutput = Math.max(minOutput, Math.min(computedOutput, maxOutput));

        lastError = error;
        lastTime = currentTime;

        return computedOutput;
    }

    public boolean hasPIDFControllerReachedTarget() {
        double error = Math.abs(setPoint - lastError);
        double percentDifferenceFromTarget = (error / (maxInput - minInput)) * 100;
        return percentDifferenceFromTarget < theresholdPercent;
    }
}