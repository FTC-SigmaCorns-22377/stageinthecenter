package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.MoveHang;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.MoveSlides;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetArm;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetClaw;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetWrist;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetClawBlack;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetClawPurple;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetTransfer;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetRoller;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetSlides;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetTransferSafe;
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
        this.hang = mechanism.hang;
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

    public Command rollerReverse() { return setRoller(Intake.RollerState.OUTTAKE);}

    public Command hangOn() { return new MoveHang(hang, Hang.HangState.INTAKE); }
    public Command hangOff() { return new MoveHang(hang, Hang.HangState.OFF); }

    public Command hangReverse() { return new MoveHang(hang, Hang.HangState.OUTTAKE); }


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
        return setSlides(Slides.SlideHeight.L0);
    }

    public Command slidesUp() {
        return setSlides(Slides.SlideHeight.L2);
    }

    public Command moveSlides(double inc){
        return new MoveSlides(slides, inc);
    }

    // Primitive
    public SetArm setArm(Output.ArmState armState) {
        return new SetArm(output, armState);
    }

    public SetClaw setClaw(Output.ClawState clawStates) {
        return new SetClaw(output, clawStates);
    }
    public SetSlides setSlides(Slides.SlideHeight slideHeight) { return new SetSlides(slides, slideHeight); }
    public SetTransfer setTransfer(Intake.TransferState transferState) {
        return new SetTransfer(intake, transferState);
    }

    public SetTransferSafe setTransferSafe(Intake.TransferState transferState){
        return new SetTransferSafe(intake, transferState, output, output.getArmState());
    }

    public Command newSetTransfer(Intake.TransferState transferState){
        return new SetTransfer(intake, transferState)
                .addNext(setArm(Output.ArmState.TRANSFER))
                .addNext(slidesDown())
                .addNext(setClaw(Output.ClawState.OPEN));

    }

    public SetRoller setRoller(Intake.RollerState rollerState) { return new SetRoller(intake, rollerState); }



    /*public Command hangUp(){
        return setHang(Hang.Hanging.UP);
    }

    public Command hangDown(){
        return setHang(Hang.Hanging.DOWN);
    }*/
    public Command intakePos(){
        return intakeDown()
                .addNext((setSlides(Slides.SlideHeight.L0)))
                .addNext(setArm(Output.ArmState.TRANSFER))
                .addNext(setClaw(Output.ClawState.OPEN));
    }

    public Command transferPos(){
        return intakeUp()
                .addNext(setSlides(Slides.SlideHeight.L0))
                .addNext(setArm(Output.ArmState.TRANSFER));
                //.addNext(setClaw(Output.ClawState.CLOSED));
    }

    public Command score(){
        return setClaw(Output.ClawState.OPEN);
    }

    public Command postScore(){
        return setTransfer(Intake.TransferState.INTAKE)
                .addNext(setSlides(Slides.SlideHeight.L0))
                .addNext(setArm(Output.ArmState.TRANSFER))
                .addNext(setClaw(Output.ClawState.OPEN));
    }

    public Command inTransUp(){
        return setTransfer(Intake.TransferState.TRANSFER);
    }

    public Command scorePos(){
        return setClaw(Output.ClawState.CLOSED)
                .addNext(setSlides(Slides.SlideHeight.L2))
                .addNext(setArm(Output.ArmState.SCORE))
                .addNext(setClaw(Output.ClawState.CLOSED));

    }
    public SetWrist setWrist(Output.WristState wristStates) { return new SetWrist(output, wristStates); }
}
