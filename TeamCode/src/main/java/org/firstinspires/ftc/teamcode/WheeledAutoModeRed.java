package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jcsanta on 11/9/2017.
 */

@Autonomous(name="Auto Red", group="Cronos")
//@Disabled
public class WheeledAutoModeRed extends WheeledAutoMode {

    @Override
    public void start() {
        super.start();

        // we are red team
        redTeam = true;
    }

    @Override
    public void loop() {
        super.loop();
    }

}
