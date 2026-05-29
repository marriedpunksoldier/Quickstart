package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoLong;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Blue Auto Long", group = "Autonomous")
public class BlueAutoLong extends AutoLong {
    public BlueAutoLong() {
        super(Alliance.BLUE);
    }
}
