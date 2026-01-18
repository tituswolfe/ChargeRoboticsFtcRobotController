/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.robots.base;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.util.math.Angle;

// - Dedication -
// In memory of DraculaBase & Mr. Miller.
// It wasn't pretty, but it worked.

/**
 * {@link RobotBase} contains the base functions & hardware classes for a robot.
 * @author Titus Wolfe
 */
@Configurable
public abstract class RobotBase {
    public enum FieldType {
        DIAMOND,
        SQUARE,
        SQUARE_INVERTED_ALLIANCE
    }
    private FieldType fieldType;

    // Drivetrain
    protected Follower follower;
    public double speedFactor = 1;
    public boolean isRobotCentric = false;
    public static boolean isSlowMode = false;
    public Angle fieldCentricOffset;
    public static double slowSpeedFactor = 0.3;



    /**
     * Called in {@link OpModeBase#init()}
     * @param hardwareMap hardware map
     */
    public void init(HardwareMap hardwareMap, Pose startPose, OpModeBase.AllianceColor allianceColor) {
        fieldType = instantiateFieldType();
        follower = instantiateFollower(hardwareMap);
        initHardware(hardwareMap);
        setFieldCentricOffset(allianceColor);

        if (follower != null) {
            follower.setStartingPose(startPose);
            follower.update();
        }
    }

    public void setFieldCentricOffset(OpModeBase.AllianceColor allianceColor) {
        fieldCentricOffset = switch (allianceColor) {
            case RED -> new Angle(90, false);
            case BLUE -> new Angle((fieldType == FieldType.DIAMOND) ? 180 : -90, false);
        };
    }

    /**
     * Initiate robot season specific hardware here.
     * @param hardwareMap hardwareMap
     */
    public abstract void initHardware(HardwareMap hardwareMap);

    /**
     * Activate any hardware to set or hold a starting configuration.
     * This method is called in {@link Autonomous} during {@link #init}.
     * This method is NOT called in {@link TeleOp}.
     * You cannot have any CONTINUOUS movement during init.
     */
    public abstract void startConfiguration();

    // public abstract void teleopStartHardware();

    /**
     * Update hardware states and telemetry. Do not {@link TelemetryManager#update()} in this method. Robot uses auto caching. do not get sensor data twice.
     */
    public void update(long deltaTimeMs, TelemetryManager telemetry) {
        if (follower != null) {
            follower.update();

            telemetry.addLine("- DRIVETRAIN -");
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.addData("isRobotCentric", isRobotCentric);
            telemetry.addData("Field Centric Offset (Deg)", fieldCentricOffset.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
            telemetry.addData("isSlowMode", isSlowMode);
            telemetry.addData("Velocity Magnitude", follower.getVelocity().getMagnitude());
            telemetry.addData("Velocity Magnitude", follower.getAcceleration().getMagnitude());
            telemetry.addLine("");

            StaticData.lastPose = follower.getPose();
        }
    }


    protected abstract FieldType instantiateFieldType();

    /**
     * @param hardwareMap hardwareMap
     * @return {@link Follower}
     * @see Constants#createFollower(HardwareMap)
     */
    public abstract Follower instantiateFollower(HardwareMap hardwareMap);

    public FieldType getFieldType() {
        return fieldType;
    }

    public Follower getFollower() {
        return follower;
    }
}
