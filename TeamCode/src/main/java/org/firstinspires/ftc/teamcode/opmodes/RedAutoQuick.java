package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoQuick;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Red Auto Quick", group = "Autonomous")
public class RedAutoQuick extends AutoQuick {
    public RedAutoQuick() {
        super(Alliance.RED);
    }
}
