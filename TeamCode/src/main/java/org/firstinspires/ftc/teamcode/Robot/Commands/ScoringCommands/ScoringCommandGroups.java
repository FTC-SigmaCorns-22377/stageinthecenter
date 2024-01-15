package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetArm;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetClawBlack;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetClawPurple;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetDrone;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetHang;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetHolder;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetRoller;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetSlides;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetWrist;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Drone;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Hang;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.MainScoringMechanism;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;


public class ScoringCommandGroups {
    Drone drone;
    Hang hang;
    Intake intake;
    Output output;
    Slides slides;
    Drivetrain drivetrain;
    int savedHeight = 0;
    int stepTime = 100;

    public ScoringCommandGroups(MainScoringMechanism mechanism, Drivetrain drivetrain) {
        this.intake = mechanism.intake;
        this.output = mechanism.output;
        this.slides = mechanism.slides;
        this.drone = mechanism.drone;
        this.hang = mechanism.hang;
        this.drivetrain = drivetrain;
    }

    // Useful
    public Command intakeDown() { return setHolder(Intake.HolderStates.INTAKE); }
    public Command rollerOn() { return setRoller(Intake.RollerStates.ON); }
    public Command rollerOff() { return setRoller(Intake.RollerStates.OFF); }
    public Command transfer() {
        return setHolder(Intake.HolderStates.TRANSFER)
                .addNext(new MultipleCommand(setClawBlack(Output.ClawBlackStates.CLOSED), setClawPurple(Output.ClawPurpleStates.CLOSED)))
                .addNext(setArm(Output.ArmStates.SCORE));
    }
    public Command scoringPosition() throws InterruptedException {
        return setSlides(savedHeight*stepTime);
    }
    public Command clawBlackOpen() { return setClawBlack(Output.ClawBlackStates.OPEN); }
    public Command clawPurpleOpen() { return setClawPurple(Output.ClawPurpleStates.OPEN); }
    public Command clawsBothOpen() throws InterruptedException {
        return new MultipleCommand(setClawBlack(Output.ClawBlackStates.OPEN), setClawPurple(Output.ClawPurpleStates.OPEN))
                .addNext(setSlides(-savedHeight*stepTime))
                .addNext(setArm(Output.ArmStates.TRANSFER));
    }
    public Command slidesDown() throws InterruptedException {
        return setSlides(-savedHeight*stepTime)
                .addNext(setArm(Output.ArmStates.TRANSFER));
    }

    // Primitive
    public SetArm setArm(Output.ArmStates armStates) {
        return new SetArm(output, armStates);
    }
    public SetClawPurple setClawPurple(Output.ClawPurpleStates clawPurpleStates) {
        return new SetClawPurple(output, clawPurpleStates);
    }
    public SetClawBlack setClawBlack(Output.ClawBlackStates clawBlackStates) {
        return new SetClawBlack(output, clawBlackStates);
    }
    public SetDrone setDrone(Drone.DroneStates droneStates) { return new SetDrone(drone, droneStates); }
    public SetHang setHang(Double power) { return new SetHang(hang, power); }
    public SetSlides setSlides(int time) { return new SetSlides(slides, time); }
    public SetHolder setHolder(Intake.HolderStates holderStates) { return new SetHolder(intake, holderStates); }
    public SetRoller setRoller(Intake.RollerStates rollerStates) { return new SetRoller(intake, rollerStates); }
    public SetWrist setWrist(Output.WristStates wristStates) { return new SetWrist(output, wristStates); }
}
