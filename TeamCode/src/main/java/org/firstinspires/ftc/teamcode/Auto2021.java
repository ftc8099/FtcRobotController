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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous(name="Autonomous 2021", group="Java")

public class Auto2021 extends LinearOpMode {

    // put our hardware here
    George george;

    @Override
    public void runOpMode() {

        // Initialize hardware
        george = new George(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            //ToDo: edit the numbers based on testing
            //Put wobble goal in square A:
            //Move forwards at angle
            george.speed = 0.75;
            george.drive(0,1, 3);
//            george.arm(50); //hopefully this works --> lower arm and open gripper
//            george.gripper(true);
            //back up a little bit an angle
            george.drive(1, 0,.5);
            george.drive(0,1,0.05);
            george.drive(0,-1, 1);

                while (george.getAngle() > 0.2 || george.getAngle() < -0.2) {
                    if (george.getAngle() > 0.2)
                        george.rotate(0.1);
                    else if (george.getAngle() < -0.2)
                        george.rotate(-0.1);
                    else
                        george.rotate(0);
                    telemetry.addData("angle", george.getAngle());
                    telemetry.update();
                }

            george.rotate(0);


            //activate flick servo, and shooter motors
            george.shoot(5);
            george.shoot(false, true, false);
            george.intake(5);
            george.shoot(5);
            george.shoot(false, true, false);
            george.drive(0,1,.001);

            telemetry.update();
        }


    }
}
