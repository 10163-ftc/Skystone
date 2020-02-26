/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Notre:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_wheel"
 * Servo channel:  Servo to open right claw: "right_wheel"
 */
public class HardwareBaymaxx
{
    //motors
    public DcMotor  FleftDrive  = null;
    public DcMotor  BleftDrive  = null;
    public DcMotor  FrightDrive = null;
    public DcMotor  BrightDrive = null;
    public DcMotor  leftwheel   = null;
    public DcMotor  rightwheel  = null;
    public DcMotor leftlift = null;
    public DcMotor rightlift = null;

    //servos
    public Servo forward = null;
    public Servo backward = null;
    //public Servo clamp = null;
    //public Servo turn = null;
    public Servo push = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareBaymaxx(){
    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        FleftDrive  = hwMap.get(DcMotor.class, "FleftDrive");
        BleftDrive  = hwMap.get(DcMotor.class, "BleftDrive");
        FrightDrive = hwMap.get(DcMotor.class, "FrightDrive");
        BrightDrive = hwMap.get(DcMotor.class, "BrightDrive");
        leftwheel   = hwMap.get(DcMotor.class, "leftwheel");
        rightwheel  = hwMap.get(DcMotor.class, "rightwheel");
        leftlift  = hwMap.get(DcMotor.class, "leftlift");
        rightlift = hwMap.get(DcMotor.class, "rightlift");

        BleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BrightDrive.setPower(0);
        leftwheel.setPower(0);
        rightwheel.setPower(0);
        leftlift.setPower(0);
        rightlift.setPower(0);

        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        FleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftwheel.setDirection(DcMotor.Direction.FORWARD);
        rightwheel.setDirection(DcMotor.Direction.FORWARD);
        leftlift.setDirection(DcMotor.Direction.FORWARD);
        rightlift.setDirection(DcMotor.Direction.REVERSE);

        //clamp = hwMap.get(Servo.class, "clamp");
        //turn = hwMap.get(Servo.class, "turn");
        forward = hwMap. get(Servo.class, "forward");
        backward = hwMap.get(Servo.class, "backward");
        push = hwMap.get(Servo.class, "push");

        forward.setPosition(0.5);
        backward.setPosition(0.51);
        //turn.setPosition(0.5);
    }
 }
