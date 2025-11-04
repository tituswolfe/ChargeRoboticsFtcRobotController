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

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;

// - Dedication -
// In memory of DraculaBase & Mr. Miller.
// It wasn't pretty, but it worked.

/**
 * {@link RobotBase} contains the base functions & hardware classes for a robot.
 *
 * @author Titus Wolfe
 */
public abstract class RobotBase {
    public enum FieldType {
        DIAMOND,
        SQUARE,
        SQUARE_INVERTED_ALLIANCE
    }
    private FieldType fieldType;

    private Follower follower;
    private Limelight3A limelight3A;

    public boolean allowDrive = false;

    /**
     * Called in {@link OpModeBase#init()}
     * @param hardwareMap
     * @param startPose
     */
    public void init(HardwareMap hardwareMap, Pose startPose) {
        fieldType = instantiateFieldType();
        follower = instantiateFollower(hardwareMap);
        limelight3A = instantiateLimelight3A(hardwareMap);
        initHardware(hardwareMap);

        if (follower != null) {
            follower.setStartingPose(startPose != null ? startPose : new Pose(0, 0, 0)); // TODO: Move over static pose
            follower.update();
            buildPaths(follower.pathBuilder());
        }
    }


    /**
     * Activate any powered or unpowered hardware to set or hold a starting configuration for {@link com.qualcomm.robotcore.eventloop.opmode.Autonomous} before {@link OpMode#start()} during {@link OpMode#init()}.
     * You cannot have any CONTINUOUS movement during init.
     * This method is only called during init in auto.
     */
    public abstract void startConfiguration();

    public abstract void buildPaths(PathBuilder pathBuilder);

    /**
     * Initiate robot season specific hardware here.
     * @param hardwareMap
     */
    public abstract void initHardware(HardwareMap hardwareMap);

    protected abstract FieldType instantiateFieldType();

    /**
     * @param hardwareMap
     * @return {@link Follower}
     * @see Constants#createFollower(HardwareMap)
     */
    public abstract Follower instantiateFollower(HardwareMap hardwareMap);

    /**
     * @param hardwareMap
     * @return {@link Limelight3A}
     */
    public abstract Limelight3A instantiateLimelight3A(HardwareMap hardwareMap);


    public FieldType getFieldType() {
        return fieldType;
    }

    public Follower getFollower() {
        return follower;
    }

    public Limelight3A getLimelight3A() {
        return limelight3A;
    }
}
