package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoLong;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Red Auto Long", group = "Autonomous")
public class RedAutoLong extends AutoLong {
    public RedAutoLong() {
        super(Alliance.RED);
    }
}
