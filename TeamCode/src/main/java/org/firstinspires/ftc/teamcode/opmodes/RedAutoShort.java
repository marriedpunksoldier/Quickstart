package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoShort;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Red Auto Short", group = "Autonomous")
public class RedAutoShort extends AutoShort {
    public RedAutoShort() {
        super(Alliance.RED);
    }
}
