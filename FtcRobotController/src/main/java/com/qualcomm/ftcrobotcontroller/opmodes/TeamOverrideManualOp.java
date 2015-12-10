package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class TeamOverrideManualOp extends OpMode {

    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor rightForward;
    DcMotor leftForward;
    DcMotor rightBack;
    DcMotor leftBack;
    OpticalDistanceSensor ODS;
    double threshold = 0.1;

    @Override
    public void init() {
        rightForward = hardwareMap.dcMotor.get("rightForward");
        leftForward = hardwareMap.dcMotor.get("leftForward");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        ODS = hardwareMap.opticalDistanceSensor.get("opticalSensor");
    }

    @Override
    public void loop() {
        //control drive train with joysticks
        if (Math.abs(gamepad1.left_stick_y) > threshold) {
            //move motor in response to joystick value
            leftForward.setPower(gamepad1.left_stick_y);
            //move motor in response to joystick value
            leftBack.setPower(gamepad1.left_stick_y);
        } else {
            //stop motor
            leftForward.setPower(0.0);
            leftBack.setPower(0.0);
        }
        if (Math.abs(gamepad1.right_stick_y) > threshold) {
            rightForward.setPower(gamepad1.right_stick_y);
            rightBack.setPower(gamepad1.right_stick_y);
        } else {
            //stop robot
            rightForward.setPower(0.0);
            rightBack.setPower(0.0);
        }


        //print out ODS values to driver station

        telemetry.addData("Robot Data", "**********");
        telemetry.addData("ODO: %d", ODS.getLightDetected());
        telemetry.addData("ODO", "ODS:" + Double.toString(ODS.getLightDetected()));
        telemetry.addData("right:", "Right:" + Double.toString(rightForward.getPower()));
        telemetry.addData("left:", "Left: " + Double.toString(leftForward.getPower()));
        telemetry.addData("right:", "Right:" + Double.toString(rightBack.getPower()));
        telemetry.addData("left:", "Left: " + Double.toString(leftBack.getPower()));
    }
}
