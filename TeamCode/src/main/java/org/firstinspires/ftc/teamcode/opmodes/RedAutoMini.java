package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoMini;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Red Auto Mini", group = "Autonomous")
public class RedAutoMini extends AutoMini {
    public RedAutoMini() {
        super(Alliance.RED);
    }
}
