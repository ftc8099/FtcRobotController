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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.concurrent.TimeUnit;


public class George
{
    /* Public OpMode members. */
    private BNO055IMU imu;
    private DcMotor wheel1;
    private DcMotor wheel3;
    private DcMotor wheel2;
    private DcMotor wheel4;
    private DcMotor Ramp;
    private DcMotor LeftShooter;
    private DcMotor RightShooter;
    private Servo Flick;
    private DcMotor BaseArm;
    private DcMotor motorTopArm;
    private Servo thumbServo;
    private DcMotor motorArm;
    float gyro;
    double speed = 1;
    double stickAngle;
    double rotateSpeed = 0.5;

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public George(HardwareMap hardwareMap){
        // Save reference to Hardware map
        hwMap = hardwareMap;

        // Define and Initialize Motors
        BNO055IMU.Parameters IMU_Parameters;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        wheel1 = hardwareMap.get(DcMotor.class, "wheel1");
        wheel3 = hardwareMap.get(DcMotor.class, "wheel3");
        wheel2 = hardwareMap.get(DcMotor.class, "wheel2");
        wheel4 = hardwareMap.get(DcMotor.class, "wheel4");
//        Ramp = hardwareMap.get(DcMotor.class, "Ramp");
//        LeftShooter = hardwareMap.get(DcMotor.class, "LeftShooter");
//        RightShooter = hardwareMap.get(DcMotor.class, "RightShooter");
//        Flick = hardwareMap.get(Servo.class, "Flick");
//        BaseArm = hardwareMap.get(DcMotor.class, "BaseArm");
//        motorTopArm = hardwareMap.get(DcMotor.class, "motorTopArm");
//        thumbServo = hardwareMap.get(Servo.class, "thumbServo");
//        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        // Put initialization blocks here.
        //Flick.setDirection(Servo.Direction.REVERSE);
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel3.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        wheel4.setDirection(DcMotorSimple.Direction.FORWARD);
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(IMU_Parameters);
    }

    public boolean wheelCheck(){
        return (wheel1 != null && wheel2 != null && wheel3 != null && wheel4 != null);
    }

    public void drive(double x, double y) {
        // Idea: ability to 'reset' the starting angle of the robot during play?
        // sets bot to angle from starting position
        gyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        // idea: ability to change speed during play?


        double eqX;
        double eqY;
        // if the gamepad left stick is moved:
        if (x != 0 || y != 0) {
            // calculate angle of the stick from x & y values
            stickAngle = Math.atan2(y, x) / Math.PI * 180;
            // remove the gyro angle & translate to x & y
            eqX = Math.cos((stickAngle - gyro) / 180 * Math.PI);
            eqY = Math.sin((stickAngle - gyro) / 180 * Math.PI);
            // Set powers of wheels to move in intended direction
            wheel1.setPower((eqY + eqX) * speed);
            wheel3.setPower((eqY + eqX) * speed);
            wheel2.setPower((eqY - eqX) * speed);
            wheel4.setPower((eqY - eqX) * speed);
        }

        wheel1.setPower(0);
        wheel3.setPower(0);
        wheel2.setPower(0);
        wheel4.setPower(0);
    }

    public void drive(double x, double y, int time) {
         ElapsedTime timer  = new ElapsedTime();
         timer.reset();
         while(timer.time(TimeUnit.SECONDS) < time) {
             drive(x, y);
         }
    }

    public void rotate(double x) {
        wheel1.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
        wheel4.setPower(0);
        // if the gamepad right stick is moved:
        if (x < 0) {
            // if it's to the right, rotate clockwise
            wheel1.setPower(rotateSpeed);
            wheel4.setPower(rotateSpeed);
            wheel2.setPower(-rotateSpeed);
            wheel3.setPower(-rotateSpeed);
        } else if (x > 0) {
            // if it's to the left, rotate counter-clockwise
            wheel1.setPower(-rotateSpeed);
            wheel4.setPower(-rotateSpeed);
            wheel2.setPower(rotateSpeed);
            wheel3.setPower(rotateSpeed);
        }
    }

    public void intake(boolean right_bumper) {
        Ramp.setPower(0);
        if (right_bumper) {
            Ramp.setPower(1);
        }
    }

    public void shoot(boolean a, boolean b) {
        if (a) {
            LeftShooter.setPower(1);
            RightShooter.setPower(1);
        }
        if (b) {
            LeftShooter.setPower(0);
            RightShooter.setPower(0);
        }

//        if (gamepad1.x) {
//            Flick.setPosition(5);
//        }
    }
 }

