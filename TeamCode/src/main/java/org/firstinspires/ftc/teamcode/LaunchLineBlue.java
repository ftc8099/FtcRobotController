package org.firstinspires.ftc.teamcode;

public class LaunchLineBlue {
}
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "launchlineblue (Blocks to Java)")
public class launchlineblue extends LinearOpMode {

    private VuforiaCurrentGame vuforiaUltimateGoal;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        vuforiaUltimateGoal = new VuforiaCurrentGame();

        // Initialize Vuforia (use default settings).
        vuforiaUltimateGoal.initialize(
                "", // vuforiaLicenseKey
                VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
                true, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                0, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
        // Prompt user to push start button.
        telemetry.addData("VuMark Example", "Press start to continue...");
        telemetry.update();
        // Wait until user pushes start button.
        waitForStart();
        if (opModeIsActive()) {
            // Activate Vuforia software.
            vuforiaUltimateGoal.activate();
            while (opModeIsActive()) {
                // Get the tracking results.
                // Turn to the left 90 degrees
                while (!vuforiaUltimateGoal.track("Blue Alliance Target").isVisible) {
                    // Move forwards, pushing wobble
                }
                telemetry.addData("VuMark", "Blue side wall goal is visible");
                // Move right, turn to face forwards
                while (!vuforiaUltimateGoal.track("Blue Tower Goal Target").isVisible) {
                    // Move to the right
                }
                telemetry.addData("VuMark", "Blue Tower goal is visible");
                // Place rings
                // use encoders to return to  launchline
            }
            // Deactivate before exiting.
            vuforiaUltimateGoal.deactivate();
        }

        vuforiaUltimateGoal.close();
    }
}