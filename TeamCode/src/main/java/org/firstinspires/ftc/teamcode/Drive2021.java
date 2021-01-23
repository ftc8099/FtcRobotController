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
@TeleOp(name = "Drive2021 (Blocks to Java)", group = "")
public class Drive2021 extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor wheel1;
    private DcMotor wheel3;
    private DcMotor wheel2;
    private DcMotor wheel4;
    private DcMotor motorArm;
    private DcMotor motorTopArm;
    private Servo thumbServo;
    double stickAngle;
    float gyro;
    double speed;
    double rotateSpeed;
    /**
     * Drive & rotate
     */
    private void drive() {
        // Idea: ability to 'reset' the starting angle of the robot during play?
        // sets bot to angle from starting position
        gyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        // idea: ability to change speed during play?
        // sets speeds of robot
        speed = 0.8;
        rotateSpeed = 0.6;
        // Wheel power to zero
        wheel1.setPower(0);
        wheel3.setPower(0);
        wheel2.setPower(0);
        wheel4.setPower(0);
        move();
        rotate();
        // Send info to the phone
        telemetry.addData("RightStick", gamepad1.right_stick_x);
        telemetry.addData("LeftStickAngle", stickAngle);
        telemetry.addData("EquationAngle", stickAngle + gyro);
        telemetry.addData("gyro", gyro);
        telemetry.addData("LeftStickX", -gamepad1.left_stick_x);
        telemetry.addData("LeftStickY", gamepad1.left_stick_y);
    }
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters IMU_Parameters;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        wheel1 = hardwareMap.get(DcMotor.class, "wheel1");
        wheel3 = hardwareMap.get(DcMotor.class, "wheel3");
        wheel2 = hardwareMap.get(DcMotor.class, "wheel2");
        wheel4 = hardwareMap.get(DcMotor.class, "wheel4");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorTopArm = hardwareMap.get(DcMotor.class, "motorTopArm");
        thumbServo = hardwareMap.get(Servo.class, "thumbServo");
        // Put initialization blocks here.
        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel4.setDirection(DcMotorSimple.Direction.REVERSE);
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(IMU_Parameters);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                drive();
                arm();
                telemetry.update();
            }
        }
    }
    /**
     * Move the arm
     */
    private void arm() {
    }
    /**
     * Describe this function...
     */
    private void move() {
        double eqX;
        double eqY;
        // if the gamepad left stick is moved:
        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
            // calculate angle of the stick from x & y values
            stickAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) / Math.PI * 180;
            // remove the gyro angle & translate to x & y
            eqX = -Math.cos((stickAngle - gyro) / 180 * Math.PI);
            eqY = Math.sin((stickAngle - gyro) / 180 * Math.PI);
            // Set powers of wheels to move in intended direction
            wheel1.setPower((eqY + eqX) * speed);
            wheel3.setPower((eqY + eqX) * speed);
            wheel2.setPower((eqY - eqX) * speed);
            wheel4.setPower((eqY - eqX) * speed);
        }
    }
    /**
     * Describe this function...
     */
    private void rotate() {
        // if the gamepad right stick is moved:
        if (gamepad1.right_stick_x > 0) {
            // if it's to the right, rotate clockwise
            wheel1.setPower(rotateSpeed);
            wheel4.setPower(rotateSpeed);
            wheel2.setPower(-rotateSpeed);
            wheel3.setPower(-rotateSpeed);
        } else if (gamepad1.right_stick_x < 0) {
            // if it's to the left, rotate counter-clockwise
            wheel1.setPower(-rotateSpeed);
            wheel4.setPower(-rotateSpeed);
            wheel2.setPower(rotateSpeed);
            wheel3.setPower(rotateSpeed);
        }
    }
}
