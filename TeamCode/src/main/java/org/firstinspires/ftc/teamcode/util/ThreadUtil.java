package org.firstinspires.ftc.teamcode.util;

public class ThreadUtil {
    public static void runAsync(Runnable runnable) {
        Thread thread = new Thread(runnable);
        thread.start();
    }

    public static void sleep(int mills) {
        try {
            Thread.sleep(mills);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}