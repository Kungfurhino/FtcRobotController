package org.firstinspires.ftc.teamcode.util;

public class SleepUtils {
    public static void sleep(long millis)
    {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.interrupted();
            e.printStackTrace();
        }
    }
}
