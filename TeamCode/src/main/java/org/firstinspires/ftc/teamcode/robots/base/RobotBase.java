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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;

// - Dedication -
// In memory of DraculaBase & Mr. Miller.
// It wasn't pretty, but it worked.

/**
 * {@link RobotBase} contains the base functions & hardware for a robot.
 *
 * @author Titus Wolfe
 */
public abstract class RobotBase {
    private Follower follower;
    private Limelight3A limelight3A;

    /**
     * Called in {@link BaseOpMode#init()}
     * @param hardwareMap
     */
    public void init(HardwareMap hardwareMap) {
        follower = instantiateFollower(hardwareMap);
        limelight3A = instantiateLimelight3A(hardwareMap);
        initHardware(hardwareMap);
    }

    /**
     * Initiate robot season specific hardware here.
     * @param hardwareMap
     */
    public abstract void initHardware(HardwareMap hardwareMap);

    /**
     * @param hardwareMap
     * @return {@link Follower}
     * @see org.firstinspires.ftc.teamcode.pedroPathing.Constants#createFollower(HardwareMap)
     */
    public abstract Follower instantiateFollower(HardwareMap hardwareMap);

    /**
     * @param hardwareMap
     * @return {@link Limelight3A}
     */
    public abstract Limelight3A instantiateLimelight3A(HardwareMap hardwareMap);

    public Follower getFollower() {
        return follower;
    }

    public Limelight3A getLimelight3A() {
        return limelight3A;
    }
}
