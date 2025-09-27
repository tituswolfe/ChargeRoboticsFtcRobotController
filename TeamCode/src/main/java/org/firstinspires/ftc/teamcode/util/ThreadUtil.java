package org.firstinspires.ftc.teamcode.util;

public class ThreadUtil {
    // TODO: Figure out how to use static cashed thread pools (tried before, many problems)

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