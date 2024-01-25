package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;

public class SetRoller extends Command {
    Intake intake;
    Intake.RollerState rollerState;

    public SetRoller(Intake intake, Intake.RollerState rollerState) {
        this.intake = intake;
        this.rollerState = rollerState;
    }

    @Override
    public void init() {
        intake.setRoller(rollerState);
    }

    @Override
    public void periodic() {}

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {

    }
}
