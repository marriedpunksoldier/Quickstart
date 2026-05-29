package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoShort;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Blue Auto Short", group = "Autonomous")
public class BlueAutoShort extends AutoShort {
    public BlueAutoShort() {
        super(Alliance.BLUE);
    }
}
