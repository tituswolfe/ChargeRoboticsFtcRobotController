package org.firstinspires.ftc.teamcode.util.math;

public class RollingAverage {
    private int sampleSize;
    private double[] samples;
    private double totalSum = 0;
    private int latestIndex = -1;
    private int count = 0;

    public RollingAverage(int sampleSize) {
        this.sampleSize = sampleSize;
        this.samples = new double[sampleSize];
    }

    public void update(double newSample) {
        latestIndex = (latestIndex + 1) % sampleSize;

        totalSum -= samples[latestIndex];

        samples[latestIndex] = newSample;
        totalSum += newSample;

        if (count < sampleSize) {
            count++;
        }
    }

    public double getAverage() {
        if (count == 0) return 0.0;
        return totalSum / count;
    }

    public int getSampleSize() {
        return sampleSize;
    }

    public void setSampleSize(int sampleSize) {
        this.sampleSize = sampleSize;
        this.samples = new double[sampleSize];
        this.totalSum = 0;
        this.latestIndex = -1;
        this.count = 0;
    }
}