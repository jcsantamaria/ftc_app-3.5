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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Back Servos Calibration", group="Linear Opmode")
//@Disabled
public class BackServosCalibration extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveFrt = null;
    private DcMotor leftDriveBck = null;
    private DcMotor rightDriveFrt = null;
    private DcMotor rightDriveBck = null;
    private DcMotor Elevator = null;

    private Servo leftGrip = null;
    private Servo rightGrip = null;
    //private Servo JouleArm = null;
    private Servo balance = null;

    double leftGripOpenPosition = .5;
    double leftGripClosePosition = .7;
    double rightGripOpenPosition = .7;
    double rightGripClosePosition = .5;

    @Override
    public void runOpMode() {
        try {

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            leftDriveFrt = hardwareMap.get(DcMotor.class, "left_drive0");
            rightDriveFrt = hardwareMap.get(DcMotor.class, "right_drive2");
            leftDriveBck  = hardwareMap.get(DcMotor.class, "left_drive1");
            rightDriveBck = hardwareMap.get(DcMotor.class, "right_drive3");
            Elevator = hardwareMap.get(DcMotor.class, "elevator");
            rightGrip = hardwareMap.get(Servo.class, "right_grip");
            leftGrip = hardwareMap.get(Servo.class, "left_grip");
            //JouleArm = hardwareMap.get(Servo.class, "joule_arm");
            balance = hardwareMap.get(Servo.class, "balance");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftDriveFrt.setDirection(DcMotor.Direction.FORWARD);
            rightDriveFrt.setDirection(DcMotor.Direction.REVERSE);
            leftDriveBck.setDirection(DcMotor.Direction.FORWARD);
            rightDriveBck.setDirection(DcMotor.Direction.REVERSE);
            Elevator.setDirection(DcMotorSimple.Direction.FORWARD);

            telemetry.addData("test","in here!");

            //float leftopen = 0;
            //float rightopen = 0;

            float balanceopen = 1f;



            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();
//"Im sorry, That is not a valid extension! :D"
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                // Setup a variable for each drive wheel to save power level for telemetry
                double leftPower;
                double rightPower;
                double epower;

                // Choose to drive using either Tank Mode, or POV Mode
                // Comment out the method that's not used.  The default below is POV.

                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.

                //If dario changes his mind :D
                //double forward = -gamepad1.left_stick_y;
                //double right   = gamepad1.left_stick_x;
                //double turn  =  gamepad1.right_stick_x;
                double forward = -gamepad1.left_stick_y;
                double right = -gamepad1.right_stick_x;
                double turn = gamepad1.left_stick_x;
                double lift = -gamepad2.left_stick_y;
                //double servo = gamepad2.right_stick_y;
                double JouleArm = gamepad2.right_stick_y;

                double leftFrontPower = forward - right + turn;
                double leftBackPower = forward + right + turn;
                double rightFrontPower = forward + right - turn;
                double rightBackPower = forward - right - turn;
                double elevatorPower = lift;
                //double servoPower = servo;


                if (gamepad1.dpad_down){
                    balanceopen -= 0.05f;
                    balanceopen = Range.clip(balanceopen, -1f,1f);
                    sleep(250);
                }
                if (gamepad1.dpad_up){
                    balanceopen += 0.05f;
                    balanceopen = Range.clip(balanceopen, 0f,1f);
                    sleep(250);
                }
             /*   if (gamepad2.dpad_down) {
                    rightopen -= 0.1f;
                    rightopen = Range.clip(rightopen, 0f,1f);
                    sleep(250);
                }
                if (gamepad2.dpad_up){
                    rightopen += 0.1f;
                    rightopen = Range.clip(rightopen, 0f,1f);
                    sleep(250);
                }
*/

                leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
                rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);
                leftBackPower = Range.clip(leftBackPower, -1.0, 1.0);
                rightBackPower = Range.clip(rightBackPower, -1.0, 1.0);
                elevatorPower = Range.clip(elevatorPower, -1.0, 1.0);
                //servoPower = Range.clip(servoPower. -1.0, 1.0);

                // Send calculated power to wheels
                //leftDriveFrt is Motor 0
                leftDriveFrt.setPower(leftFrontPower);
                //rightDriveFrt is Motor 2
                rightDriveFrt.setPower(rightFrontPower);
                //leftDriveBck is Motor 1
                leftDriveBck.setPower(leftBackPower);
                //leftDriveBck is Motor 3
                rightDriveBck.setPower(rightBackPower);
                Elevator.setPower(elevatorPower);

                //servo - grip
/*                if (Math.abs(gamepad2.right_stick_y)  > 0.06){
                    double gripPos = ((-gamepad2.right_stick_y+1.0)/2);
                    leftGrip.setPosition(gripPos);
                    rightGrip.setPosition(1.0 - gripPos);
                }
*/

                //Copyright Carlos (C)
                //Moves servos to a certain position on keypress
                //leftGrip.setPosition(-0.4);
                //rightGrip.setPosition(1.0);
                if(gamepad1.a){
                    balance.setPosition(balanceopen);
                }
                if(gamepad1.b){
                    balance.setPosition(0.2);
                }



                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front", "%5.2f  %5.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back ", "%5.2f  %5.2f", leftBackPower, rightBackPower);
//                telemetry.addData("gripper ", "lo:%5.2f  ro:%5.2f lc:%5.2f  rc:%5.2f", leftGripOpenPosition, rightGripOpenPosition, leftGripClosePosition, rightGripClosePosition);
                telemetry.addData("balance", "balance:%5.2f", balanceopen);
                telemetry.update();
            }

        }
        catch(Exception ex)
        {
            telemetry.addData( "error","error: " + ex.getMessage());
            //stop();
        }


    }

    void openGripper()
    {
        leftGrip.setPosition(Range.clip(leftGripOpenPosition, 0.0, 1.0));
        rightGrip.setPosition(Range.clip(rightGripOpenPosition, -1.0, 1.0));

    }
    void closeGripper()
    {
        leftGrip.setPosition(Range.clip(leftGripClosePosition, 0.0, 1.0));
        rightGrip.setPosition(Range.clip(rightGripClosePosition, -1.0, 1.0));

    }
}