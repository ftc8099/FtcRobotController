package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Drive2021 (George)")
public class Drive2021_2 extends LinearOpMode {
    George george;


    @Override
    public void runOpMode() {
        george = new George(hardwareMap);
        //george.init(hardwareMap);
        telemetry.addData("Wheels", george.wheelCheck());
        telemetry.addData("Gyro", george.gyro);
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.addData("Wheels",george.wheelCheck());
                telemetry.addData("Gyro", george.gyro);
                telemetry.addData("Joystick x", gamepad1.left_stick_x);
                telemetry.addData("Joystick y", gamepad1.left_stick_y);
                telemetry.addData("Flick position", george.Flick.getPosition());
                telemetry.update();
                george.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                george.rotate(gamepad1.right_stick_x);
                george.flick(gamepad1.x);
                george.shoot(gamepad1.a, gamepad1.b, false);
                george.intake(gamepad1.right_bumper);
                george.arm(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left);
//                line below: press left bumper to activate shooters and then flick
                //uncomment to test
//                if (gamepad1.left_bumper){
//                    george.shoot(3);
//                }

                telemetry.update();
            }
        }
    }
}