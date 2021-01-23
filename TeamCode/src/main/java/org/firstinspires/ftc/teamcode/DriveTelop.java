package org.firstinspires.ftc.teamcode;

public class DriveTelop {
}
    Java Code:
        package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "_2020Teleop (Blocks to Java)")
public class _2020Teleop extends LinearOpMode {

    private DcMotor motorArmAsDcMotor;
    private DcMotor motorTopArmAsDcMotor;
    private Servo thumbServo;
    private Servo leftServo;
    private Servo rightServo;
    private DcMotor motorRightAsDcMotor;
    private DcMotor motorLeftAsDcMotor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int armPosition;
        int TopArmPosition;
        int TopArmLimit;
        int ArmLimit;
        double ServoPosition;
        double ServoSpeed;

        motorArmAsDcMotor = hardwareMap.get(DcMotor.class, "motorArmAsDcMotor");
        motorTopArmAsDcMotor = hardwareMap.get(DcMotor.class, "motorTopArmAsDcMotor");
        thumbServo = hardwareMap.get(Servo.class, "thumbServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        motorRightAsDcMotor = hardwareMap.get(DcMotor.class, "motorRightAsDcMotor");
        motorLeftAsDcMotor = hardwareMap.get(DcMotor.class, "motorLeftAsDcMotor");

        // Put initialization blocks here.
        // #### Put Motor Arm Stuff Here ####
        motorArmAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTopArmAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArmAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTopArmAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTopArmAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Assume 0 for the lower limit (where the motor is)
        // Set upper limits
        armPosition = 0;
        TopArmPosition = 0;
        ArmLimit = -545;
        TopArmLimit = 335;
        motorArmAsDcMotor.setTargetPosition(armPosition);
        motorTopArmAsDcMotor.setTargetPosition(TopArmPosition);
        motorArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTopArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorTopArmAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorArmAsDcMotor.setPower(1);
        motorTopArmAsDcMotor.setPower(1);
        // #### Put Thumb Servo Stuff Here ####
        ServoPosition = 0.5;
        ServoSpeed = 0.03;
        thumbServo.setPosition(0.5);
        // #### Servo #####
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setPosition(0);
        rightServo.setPosition(0);
        // #### Put Drive Stuff Here ####
        motorRightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // #### Put Motor Arm Stuff Here ####
                if (gamepad1.y) {
                    armPosition += -7;
                    if (armPosition < ArmLimit) {
                        armPosition = ArmLimit;
                    }
                }
                if (gamepad1.a) {
                    armPosition += 7;
                    if (armPosition > 0) {
                        armPosition = 0;
                    }
                }
                motorArmAsDcMotor.setTargetPosition(armPosition);
                if (gamepad1.dpad_up) {
                    TopArmPosition += 5;
                    if (TopArmPosition > TopArmLimit) {
                        TopArmPosition = TopArmLimit;
                    }
                }
                if (gamepad1.dpad_down) {
                    TopArmPosition += -5;
                    if (TopArmPosition < 0) {
                        TopArmPosition = 0;
                    }
                }
                motorTopArmAsDcMotor.setTargetPosition(TopArmPosition);
                telemetry.addData("armPosition", armPosition);
                telemetry.addData("TopArmPosition", TopArmPosition);
                telemetry.addData("Bottom", motorArmAsDcMotor.getCurrentPosition());
                telemetry.addData("Top", motorTopArmAsDcMotor.getCurrentPosition());
                // #### Servo ####
                if (gamepad1.dpad_left) {
                    leftServo.setPosition(0);
                    rightServo.setPosition(0);
                }
                if (gamepad1.dpad_right) {
                    leftServo.setPosition(0.42);
                    rightServo.setPosition(0.42);
                }
                // #### Put Motor Arm Stuff Here ####
                if (gamepad1.b) {
                    ServoPosition += ServoSpeed;
                }
                if (gamepad1.x) {
                    ServoPosition += -ServoSpeed;
                }
                if (gamepad1.right_bumper) {
                    ServoPosition = 0.5;
                }
                ServoPosition = Math.min(Math.max(ServoPosition, 0.25), 0.75);
                thumbServo.setPosition(ServoPosition);
                telemetry.addData("Servo", ServoPosition);
                // ####
                // #### Put Drive Stuff Here ####
                motorRightAsDcMotor.setPower(gamepad1.right_stick_y);
                motorLeftAsDcMotor.setPower(gamepad1.left_stick_y);
                telemetry.addData("Right", gamepad1.right_stick_y);
                telemetry.addData("Left", gamepad1.left_stick_y);
                telemetry.update();
            }
        }
    }
}