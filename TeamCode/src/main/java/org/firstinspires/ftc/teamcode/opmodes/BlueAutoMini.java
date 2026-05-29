package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoMini;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Blue Auto Mini", group = "Autonomous")
public class BlueAutoMini extends AutoMini {
    public BlueAutoMini() {
        super(Alliance.BLUE);
    }
}
