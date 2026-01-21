package org.firstinspires.ftc.teamcode.hardware.controllers;


import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Abstract HardwareController for closed loop control
 */
public abstract class HardwareController<Device extends HardwareDevice> {
    protected final Device device;
    private final String name;

    public HardwareController(Device device, String name) {
        this.device = device;
        this.name = name;
    }

    public abstract void update(long deltaTime);


    public void addTelemetry(TelemetryManager telemetry) {
        telemetry.addData("Name", name);
    }

    public Device getDevice() {
        return device;
    }
}
