package org.firstinspires.ftc.teamcode.util;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FtcDashboardUtil {
    private final static FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    public static MultipleTelemetry getMultipleTelemetry(Telemetry telemetry) {
        return new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());
    }

    public static void sendImage(Bitmap bitmap) {
        ftcDashboard.sendImage(bitmap);
    }

    // TODO: Add prams
    public static void updateCurrentPosition() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .fillRect(0, 0, 24, 24);
        ftcDashboard.sendTelemetryPacket(packet);
    }

}
