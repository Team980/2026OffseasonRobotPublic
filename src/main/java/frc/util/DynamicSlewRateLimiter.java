package frc.util;

import java.util.function.DoubleSupplier;

public class DynamicSlewRateLimiter {
    Stopwatch stopwatch = new Stopwatch();
    DoubleSupplier upRate;
    DoubleSupplier downRate;
    double currentVal;

    public DynamicSlewRateLimiter(DoubleSupplier upRate, DoubleSupplier downRate, double startVal) {
        this.downRate = downRate;
        this.upRate = upRate;
        this.currentVal = startVal;
        this.stopwatch.start();
    }
    public DynamicSlewRateLimiter(DoubleSupplier upRate, DoubleSupplier downRate) {
        this(upRate, downRate, 0);
    }
    public DynamicSlewRateLimiter(DoubleSupplier upRate, double startVal) {
        this(upRate, upRate, startVal);
    }
    public DynamicSlewRateLimiter(DoubleSupplier upRate) {
        this(upRate, upRate, 0);
    }

    public double calculate(double targetVal){
        double diff = Math.abs(currentVal-targetVal);

        double elapsedSeconds = this.stopwatch.elapsedSeconds();
        if (elapsedSeconds > 0.1) elapsedSeconds = 0.02;

        stopwatch.start();

        if (targetVal >= currentVal) {
            this.currentVal += Math.min(diff, upRate.getAsDouble()*elapsedSeconds);
        } else {
            this.currentVal -= Math.min(diff, downRate.getAsDouble()*elapsedSeconds);
        }

        return this.currentVal;
    }

    public double calculate(double targetVal, double mod) {
        this.currentVal = this.currentVal % mod + mod;
        targetVal = targetVal % mod + mod;
        // Mod numbers arent ordered so we will only use upRate
        double[] targetVals = {targetVal, targetVal+mod, targetVal-mod};
        double[] diffs = {Math.abs(targetVals[0]-currentVal), Math.abs(targetVals[1]-currentVal), Math.abs(targetVals[2]-currentVal)};

        targetVal = targetVals[0];
        double diff = diffs[0];

        if (diffs[1] < diff) {diff = diffs[1]; targetVal = targetVals[1];}
        if (diffs[2] < diff) {diff = diffs[2]; targetVal = targetVals[2];}

        double elapsedSeconds = this.stopwatch.elapsedSeconds();
        if (elapsedSeconds > 0.1) { // elapsedSeconds = 0.02;
            currentVal = targetVal;
        }

        stopwatch.start();

        if (targetVal >= currentVal) {
            this.currentVal += Math.min(diff, upRate.getAsDouble()*elapsedSeconds);
        } else {
            this.currentVal -= Math.min(diff, upRate.getAsDouble()*elapsedSeconds);
        }

        while (this.currentVal < 0) this.currentVal += mod;
        this.currentVal %= mod;

        return this.currentVal;
    }
    

    public double prevVal() {
        return this.currentVal;
    }

    public void reset(double val) {
        this.stopwatch.start();
        this.currentVal = val;
    }

    public void reset() {
        this.reset(currentVal);
    }
}