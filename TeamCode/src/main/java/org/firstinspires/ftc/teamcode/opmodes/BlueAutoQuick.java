package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoQuick;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Blue Auto Quick", group = "Autonomous")
public class BlueAutoQuick extends AutoQuick {
    public BlueAutoQuick() {
        super(Alliance.BLUE);
    }
}
