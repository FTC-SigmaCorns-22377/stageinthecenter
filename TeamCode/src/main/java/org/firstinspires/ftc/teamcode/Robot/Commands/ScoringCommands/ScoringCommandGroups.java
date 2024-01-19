package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetArm;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetClawBlack;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetClawPurple;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetHang;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetTransfer;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetRoller;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetSlides;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.ToggleTransfer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Drone;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Hang;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.MainScoringMechanism;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;
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
//        this.drone = mechanism.drone;
//        this.hang = mechanism.hang;
        this.drivetrain = drivetrain;
    }

    // Useful
    public Command intakeDown() { return setTransfer(Intake.TransferState.INTAKE); }

    public Command intakeUp() { return setTransfer(Intake.TransferState.TRANSFER); }

    public Command intakeToggle() { return new ToggleTransfer(intake); }

    public Command rollerOn() { return setRoller(Intake.RollerState.INTAKE); }
    public Command rollerOff() { return setRoller(Intake.RollerState.OFF); }
//    public Command transfer() {
//        return setHolder(Intake.HolderStates.TRANSFER)
//                .addNext(new MultipleCommand(setClawBlack(Output.ClawBlackStates.CLOSED), setClawPurple(Output.ClawPurpleStates.CLOSED)))
//                .addNext(setArm(Output.ArmStates.SCORE));
//    }
//    public Command scoringPosition() {
//        return setSlides(savedHeight*stepTime);
//    }
//    public Command clawBlackOpen() { return setClawBlack(Output.ClawBlackStates.OPEN); }
//    public Command clawPurpleOpen() { return setClawPurple(Output.ClawPurpleStates.OPEN); }
//    public Command clawsBothOpen() throws InterruptedException {
//        return new MultipleCommand(setClawBlack(Output.ClawBlackStates.OPEN), setClawPurple(Output.ClawPurpleStates.OPEN))
//                .addNext(setSlides(-savedHeight*stepTime))
//                .addNext(setArm(Output.ArmStates.TRANSFER));
//    }
//    public Command slidesDown() throws InterruptedException {
//        return setSlides(-savedHeight*stepTime)
//                .addNext(setArm(Output.ArmStates.TRANSFER));
//    }

    public Command slidesDown() {
        return setSlides(Slides.SlideHeight.LOW);
    }

    public Command slidesUp() {
        return setSlides(Slides.SlideHeight.MID);
    }

    // Primitive
    public SetArm setArm(Output.ArmState armState) {
        return new SetArm(output, armState);
    }

    public SetHang setHang(Hang.Hanging hanging){
        return new SetHang(hang, hanging);
    }

    public Command hangUp(){
        return setHang(Hang.Hanging.UP);
    }

    public Command hangDown(){
        return setHang(Hang.Hanging.DOWN);
    }

    public SetClawPurple setClawPurple(Output.ClawState clawPurpleStates) {
        return new SetClawPurple(output, clawPurpleStates);
    }
    public SetClawBlack setClawBlack(Output.ClawState clawBlackStates) {
        return new SetClawBlack(output, clawBlackStates);
    }
//    public SetDrone setDrone(Drone.DroneStates droneStates) { return new SetDrone(drone, droneStates); }
    //public SetHang setHang(Double power) { return new SetHang(hang, power); }
    public SetSlides setSlides(Slides.SlideHeight slideHeight) { return new SetSlides(slides, slideHeight); }
    public SetTransfer setTransfer(Intake.TransferState transferState) { return new SetTransfer(intake, transferState); }
    public SetRoller setRoller(Intake.RollerState rollerState) { return new SetRoller(intake, rollerState); }
//    public SetWrist setWrist(Output.WristStates wristStates) { return new SetWrist(output, wristStates); }
}
