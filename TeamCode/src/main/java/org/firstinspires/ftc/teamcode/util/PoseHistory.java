package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public class PoseHistory {
    private final int capacity; // BY TIME RATHER THAN SAMPLES?

    private final Pose[] poses;
    private final long[] timestamps;

    private int latestIndex = -1;
    private int count = 0;

    public PoseHistory(int capacity) {
        this.capacity = capacity;
        this.poses = new Pose[capacity];
        this.timestamps = new long[capacity];
    }

    public void update(Pose pose) {
        latestIndex = (latestIndex + 1) % capacity;

        poses[latestIndex] = pose;
        timestamps[latestIndex] = System.currentTimeMillis();

        if (count < capacity) {
            count++;
        }
    }

    public Pose getPoseAt(long deltaMillis) {
        if (count == 0) return new Pose();

        long targetTime = System.currentTimeMillis() - deltaMillis;
        int oldestIdx = (latestIndex - count + 1 + capacity) % capacity;

        if (targetTime >= timestamps[latestIndex]) return poses[latestIndex];
        if (targetTime <= timestamps[oldestIdx]) return poses[oldestIdx];

        int olderIdx = MathUtil.findFloorIndex(timestamps, targetTime, oldestIdx, count);
        int newerIdx = (olderIdx + 1) % capacity;

        double t = (double) (targetTime - timestamps[olderIdx]) / (timestamps[newerIdx] - timestamps[olderIdx]);

        return MathUtil.lerp(poses[olderIdx], poses[newerIdx], t);
    }
}