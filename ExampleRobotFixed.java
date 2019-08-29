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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp Basic Example", group="Iterative Opmode")
public class ExampleRobotFixed extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor ML = null;
    private DcMotor MR = null;
    private String DM = "";
    double PL=0;
    double PR=0;
    double drive=0;
    double turn=0;

    /*                                            *\
    |* Code to run ONCE when the driver hits INIT *|
    \*                                            */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        ML = hardwareMap.get(DcMotor.class, "ML");
        MR = hardwareMap.get(DcMotor.class, "MR");

        // Since the motors don't know what direction they're 'facing', they all spin
        // the same direction. To counteract this, we set one to flip its values for
        // backwards and forwards by performing these two lines of code. 
        ML.setDirection(DcMotor.Direction.FORWARD);
        MR.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*                                                                             *\
    |* Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY *|
    \*                                                                             */
    @Override
    public void init_loop() {
        if(gamepad1.a){
            DM = "POV";
        }
        else if (gamepad1.b){
            DM = "TANK";
        }
        telemetry.addData("Use the A/B button to select drive mode: ",DM);
        telemetry.update();
        
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        if(DM==""){
            DM="POV";
        }
        runtime.reset();
    }

    /*                                                                            *\
    |* Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP *|
    \*                                                                            */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        
        // Uses the left/right trigger for throttle/reverse
        // Uses the left stick x-axis to determine turning speed
        if(DM=="POV"){
            drive = gamepad1.left_trigger-gamepad1.right_trigger;
            turn  = gamepad1.left_stick_x;
            PL = Range.clip(drive - ((turn)*-drive), -1.0, 1.0);
            PR = Range.clip(drive + ((turn)*-drive), -1.0, 1.0);
        }

        // Tank Mode uses the Y-axis position of either joystick to power the motors
        if (DM=="TANK"){
            
            PL = -gamepad1.left_stick_y;
            PR = -gamepad1.right_stick_y;
        }

        // Send calculated power to wheels
        ML.setPower(PL);
        MR.setPower(PR);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", PL, PR);
    }

    /*                                             *\
    |* Code to run ONCE after the driver hits STOP *|
    \*                                             */
    @Override
    public void stop() {
        // Making sure motors stop no matter what
        ML.setPower(0);
        MR.setPower(0);
        
    }

}
