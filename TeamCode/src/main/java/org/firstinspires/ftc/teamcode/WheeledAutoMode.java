package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
 * Created by jcsanta on 11/9/2017.
 */

@Autonomous(name = "Auto", group = "Cronos")
@Disabled
public class WheeledAutoMode extends WheeledBotHardware {

    /**
     * The offset is the position of the relic template relative to the starting position
     * of the robot.
     */
    public double OffsetX = 0.0;
    public double OffsetY = 0.0;

    public enum BehaviorState {
        LowerArm,   // lower joule arm
        Check,      // check color of ball
        Bump,       // move a bit to bump the ball
        Wait,
        RaiseArm,   // raise joule arm
        Stop                // stop the robot
    }

    public boolean redTeam;   // true if we are red team; false otherwise
    public BehaviorState state;
    public double timestamp;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;

    @Override
    public void start() {

        // do what our parent says first
        super.start();

        //Set drive mode
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set drive mode
        setElevatorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set gripper to open
        openWideGripper();

        //Raise joule arm
        jouleArmUp();

        //Raise the balance
        balanceUp();

        // set initial state
        state = BehaviorState.LowerArm;

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
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        /**
         * activate the patterns
         */
        relicTrackables.activate();

        timestamp = getRuntime();
        telemetry.addData("joule", String.format("%.2f", joule.getPosition()));
        telemetry.addData("state", "%s", state);
    }

    @Override
    public void loop() {

        // do what our parent says first
        super.loop();

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
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

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

                positionX = tX - OffsetX;
                positionY = tY - OffsetY;
            }
        } else {
            positionX = Double.NaN;
            positionY = Double.NaN;
        }

        if (gamepad2.dpad_up) {
            jouleArmUp();
        }
        if (gamepad2.dpad_down) {
            jouleArmDown();
        }

        boolean redAhead = (colorSensor.red() - colorSensor.blue()) > 10;

        switch (state) {
            case LowerArm: {
                jouleArmDown();
                closeGripper();
                if (joule.getPosition() > .85 && (getRuntime() - timestamp) > 2) {
                    state = BehaviorState.Check;
                    timestamp = getRuntime();   // mark current time
                }
            }
            break;
/*
            case Wait:
            {
                try
                {
                    Thread.sleep(1000);
                }
                catch(InterruptedException ex)
                {
                    Thread.currentThread().interrupt();
                }
            }
            break;
*/
            case Check: {
                // bumpPower sets the direction of the bump: + is forward, - is backwards
                double bumpPower = redTeam ? -0.2 : 0.2;

                if (redAhead) {
                    // move in direction to knock down opposite color
                    setDrivePower( bumpPower, 0.0, 0.0);
                } else {
                    // move opposite to knock down opposite color
                    setDrivePower(-bumpPower, 0.0, 0.0);
                }

                /*during competition 11/11/17, the time and power works fine when red = true
                when red = false, it does not seem to move far enough to knock the ball off every  time
                 */

                state = BehaviorState.Bump;
                timestamp = getRuntime();   // mark current time
            }
            break;

            case Bump: {
                // let motors run for a period of time
                if ( getRuntime() - timestamp > 1.5) {
                    state = BehaviorState.Stop;
                    timestamp = getRuntime();   // mark current time
    }
            }
            break;

            case Stop: {
                setDrivePower(0.0, 0.0, 0.0);
                jouleArmUp();
            }
            break;
        }

        telemetry.addData("state", "%s", state);
        //telemetry.addData("color", String.format("r:%d g:%d b:%d", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
        telemetry.addData("isRed", String.format("%b", redAhead));
        telemetry.addData("joule", joule.getPosition());
        telemetry.addData("pos  ", String.format("x:%4.0f y:%4.0f h:%3.0f", positionX, positionY, Math.toDegrees(heading)));


        //telemetry.addData("joy", String.format("%.2f %.2f",  xValue, yValue));
        //telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f", positionX, positionY, Math.toDegrees(heading)));
        //telemetry.addData("rot", String.format("p:%3.0f r:%3.0f h:%3.0f", orientation.getPitch(), orientation.getRoll(), orientation.getHeading()));
        //telemetry.addData("touch", armTouch != null ? armTouch.isPressed() : "null");
        //telemetry.addData("arm", String.format("%s %.2f %d %s", onArmReset, elvMotor.getPower(), elvMotor.getCurrentPosition(), elvMotor.getMode().toString()));
        //telemetry.addData("distance", String.format("%.2f", opticalDistanceSensor.getLightDetected()));
    }

}
