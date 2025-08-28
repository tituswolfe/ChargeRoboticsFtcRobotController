package org.firstinspires.ftc.teamcode.util;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ThreadUtil {
    // Make the executorService final and initialize it in a static block
    private static ExecutorService executorService;


    public static void init() {
        executorService = Executors.newCachedThreadPool();
    }


    public static void runAsync(Runnable runnable) {
        // This can now never be null
        executorService.submit(runnable);
    }

    public static void shutdown() {
        if (executorService != null && !executorService.isShutdown()) {
            executorService.shutdownNow();
            System.out.println("ExecutorService has been shut down.");
        }
    }

    public static void sleep(int mills) {
        try {
            Thread.sleep(mills);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}