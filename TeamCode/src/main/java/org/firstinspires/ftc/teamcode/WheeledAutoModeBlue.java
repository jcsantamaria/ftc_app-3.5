package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by jcsanta on 11/30/2017.
 */

@Autonomous(name="Auto Blue", group="Cronos")
//@Disabled
public class WheeledAutoModeBlue extends WheeledAutoMode {

    @Override
    public void start() {
        super.start();

        // we are nlue team
        redTeam = false;
    }

    @Override
    public void loop() {
        super.loop();
    }
}
