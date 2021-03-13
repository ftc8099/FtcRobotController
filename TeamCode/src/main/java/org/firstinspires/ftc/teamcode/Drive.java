package org.firstinspires.ftc.teamcode;

public class Drive {
}
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

@TeleOp(name = "Drive2021 (Blocks to Java)")
public class Drive2021 extends LinearOpMode {

    private BNO055IMU imuAsBNO055IMU;
    private DcMotor wheel1AsDcMotor;
    private DcMotor wheel3AsDcMotor;
    private DcMotor wheel2AsDcMotor;
    private DcMotor wheel4AsDcMotor;
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
        gyro = imuAsBNO055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        // idea: ability to change speed during play?
        // sets speeds of robot
        speed = 0.8;
        rotateSpeed = 0.6;
        // Wheel power to zero
        wheel1AsDcMotor.setPower(0);
        wheel3AsDcMotor.setPower(0);
        wheel2AsDcMotor.setPower(0);
        wheel4AsDcMotor.setPower(0);
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

        imuAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imuAsBNO055IMU");
        wheel1AsDcMotor = hardwareMap.get(DcMotor.class, "wheel1AsDcMotor");
        wheel3AsDcMotor = hardwareMap.get(DcMotor.class, "wheel3AsDcMotor");
        wheel2AsDcMotor = hardwareMap.get(DcMotor.class, "wheel2AsDcMotor");
        wheel4AsDcMotor = hardwareMap.get(DcMotor.class, "wheel4AsDcMotor");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorTopArm = hardwareMap.get(DcMotor.class, "motorTopArm");
        thumbServo = hardwareMap.get(Servo.class, "thumbServo");

        // Put initialization blocks here.
        wheel1AsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel4AsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuAsBNO055IMU.initialize(IMU_Parameters);
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
            wheel1AsDcMotor.setPower((eqY + eqX) * speed);
            wheel3AsDcMotor.setPower((eqY + eqX) * speed);
            wheel2AsDcMotor.setPower((eqY - eqX) * speed);
            wheel4AsDcMotor.setPower((eqY - eqX) * speed);
        }
    }

    /**
     * Describe this function...
     */
    private void rotate() {
        // if the gamepad right stick is moved:
        if (gamepad1.right_stick_x > 0) {
            // if it's to the right, rotate clockwise
            wheel1AsDcMotor.setPower(rotateSpeed);
            wheel4AsDcMotor.setPower(rotateSpeed);
            wheel2AsDcMotor.setPower(-rotateSpeed);
            wheel3AsDcMotor.setPower(-rotateSpeed);
        } else if (gamepad1.right_stick_x < 0) {
            // if it's to the left, rotate counter-clockwise
            wheel1AsDcMotor.setPower(-rotateSpeed);
            wheel4AsDcMotor.setPower(-rotateSpeed);
            wheel2AsDcMotor.setPower(rotateSpeed);
            wheel3AsDcMotor.setPower(rotateSpeed);
        }
    }
}