package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;

public class SetRoller extends Command {
    Intake intake;
    Intake.RollerStates rollerStates;
    boolean hasFinished = false;

    public SetRoller(Intake intake, Intake.RollerStates rollerStates) {
        this.intake = intake;
        this.rollerStates = rollerStates;
    }

    @Override
    public void init() {
        intake.setRoller(rollerStates);
    }

    @Override
    public void periodic() {
        intake.setRoller(rollerStates);
        hasFinished = true;
    }

    @Override
    public boolean completed() {
        return hasFinished;
    }

    @Override
    public void shutdown() {

    }
}
