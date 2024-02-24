package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.FireDroneSequence;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.MoveHang;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.RetractDrone;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetArm;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetClaw;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetRollerPosition;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SetWrist;
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
        this.drone = mechanism.drone;
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
        return new MultipleCommand(
                new SetTransfer(intake, transferState),
                setArm(Output.ArmState.TRANSFER),
                slidesDown(),
                setClaw(Output.ClawState.OPEN)
        );
    }

    public Command setRollerPosition(Intake.TransferState transferState) {
        return new SetRollerPosition(intake, transferState);
    }

    public Command autoScorePos(){
        return setClaw(Output.ClawState.CLOSED)
                .addNext(setSlides(Slides.SlideHeight.L2))
                .addNext(setArm(Output.ArmState.SCORE))
                .addNext(setClaw(Output.ClawState.CLOSED));

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

        return setClaw(Output.ClawState.BOTHPOSTSCORE)
                .addNext(setArm(Output.ArmState.POST));
    }
    public Command blackscore(){

        return setClaw(Output.ClawState.BLACKPOSTSCORE);
    }
    public Command purplescore(){

        return setClaw(Output.ClawState.PURPLEPOSTSCORE);
    }

    public Command postScore(){
        return new MultipleCommand(
                setTransfer(Intake.TransferState.TRAVEL),
                setSlides(Slides.SlideHeight.L0),
                setArm(Output.ArmState.TRAVEL),
                setClaw(Output.ClawState.OPEN),
                setWrist(Output.WristState.DEG0)
        );
    }

    public Command Intake(){
        return setTransfer(Intake.TransferState.INTAKE)
              .addNext(setClaw(Output.ClawState.OPEN));
    }

    public Command postScore5(){
        return setTransfer(Intake.TransferState.FIVE)
                .addNext(setSlides(Slides.SlideHeight.L0))
                .addNext(setArm(Output.ArmState.TRANSFER))
                .addNext(setClaw(Output.ClawState.OPEN));
    }

    public Command postScore3(){
        return setTransfer(Intake.TransferState.THREE)
                .addNext(setSlides(Slides.SlideHeight.L0))
                .addNext(setArm(Output.ArmState.TRANSFER))
                .addNext(setClaw(Output.ClawState.OPEN));
    }

    public Command inTransUp(){
        return setTransfer(Intake.TransferState.TRANSFER);
    }

    public Command scorePos(){
        return setClaw(Output.ClawState.CLOSED)
                .addNext(setSlides(Slides.SlideHeight.L1))
                .addNext(setArm(Output.ArmState.SCORE))
                .addNext(setClaw(Output.ClawState.CLOSED));

    }

    public Command scorePosNoLift(){
        return setClaw(Output.ClawState.CLOSED)
                .addNext(setArm(Output.ArmState.SCORE))
                .addNext(setClaw(Output.ClawState.CLOSED));
    }

    public SetWrist setWrist(Output.WristState wristStates) { return new SetWrist(output, wristStates); }

    public Command retractDrone() {
        return new RetractDrone(drone);
    }

    public Command fireDroneSequence() {
        return new FireDroneSequence(drone);
    }
}
