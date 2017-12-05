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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


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

@TeleOp(name="Pattern Test", group="Linear Opmode")
//@Disabled
public class PatternTest extends LinearOpMode {

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
    private Servo Balance = null;

    double leftGripOpenPosition = .5;
    double leftGripClosePosition = .7;
    double rightGripOpenPosition = .7;
    double rightGripClosePosition = .5;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

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
            Balance = hardwareMap.get(Servo.class, "balance");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftDriveFrt.setDirection(DcMotor.Direction.FORWARD);
            rightDriveFrt.setDirection(DcMotor.Direction.REVERSE);
            leftDriveBck.setDirection(DcMotor.Direction.FORWARD);
            rightDriveBck.setDirection(DcMotor.Direction.REVERSE);
            Elevator.setDirection(DcMotorSimple.Direction.FORWARD);

            telemetry.addData("test","in here!");

            float leftopen = 0;
            float rightopen = 0;

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            // OR...  Do Not Activate the Camera Monitor View, to save power
            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
            parameters.vuforiaLicenseKey = "AfoTndf/////AAAAGUkP1/cU1k2/ncKRQNPTlUhusBVRu77+YJvjQjkwz0/KJkHKHPgptit/RUbZoF8q3HztGCS2zyHZ2IinfJYzre858HGS2z84gfFEBPvRi2GvrZy3/BMJYXWIMOL1eyj9NRLvjh0OQ1nEqM0noVvOIVyfpHU/bLQWPDXSV6ABhEvQ/tfyMeRq/5uRx+YCoujw8pKll6aatWlVWOkIOczSl81JVC50yLsagkyolgyWMh36C7AJtflq2B6JU5VEpDZMY8MRpDoo5C8GO3YwUwYNATRgz/VwfqQmZsD9mGlRUpivCBiZbQNqsRkPfeROF7Bpmyzvdomb2lkMyboUcdYUcQgh6m9/UHDDLi6bBdPz1CPc";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            /**
             * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
             * in this data set: all three of the VuMarks in the game were created from this one template,
             * but differ in their instance id information.
             * @see VuMarkInstanceId
             */
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            telemetry.addData(">", "Press Play to start");
            telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            relicTrackables.activate();

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

                /**
                 * See if any of the instances of {@link relicTemplate} are currently visible.
                 * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                 * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                 * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
                 */
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                    telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;
                    }
                }
                else {
                    telemetry.addData("VuMark", "not visible");
                }

                telemetry.update();


                // Show the elapsed game time and wheel power.
                //telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.addData("Front", "%5.2f  %5.2f", leftFrontPower, rightFrontPower);
                //telemetry.addData("Back ", "%5.2f  %5.2f", leftBackPower, rightBackPower);
//                telemetry.addData("gripper ", "lo:%5.2f  ro:%5.2f lc:%5.2f  rc:%5.2f", leftGripOpenPosition, rightGripOpenPosition, leftGripClosePosition, rightGripClosePosition);
                //telemetry.addData("pattern ", "%s", pattern);
                //telemetry.update();
            }

        }
        catch(Exception ex)
        {
            telemetry.addData( "error","error: " + ex.getMessage());
            //stop();
        }


    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
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