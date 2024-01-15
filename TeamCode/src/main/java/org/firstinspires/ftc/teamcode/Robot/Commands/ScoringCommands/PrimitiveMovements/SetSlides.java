package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;

public class SetSlides extends Command {
    Slides slides;
    int time;
    boolean hasFinished = false;

    public SetSlides(Slides slides, int time) {
        this.slides = slides;
        this.time = time;
    }

    @Override
    public void init() throws InterruptedException {
        slides.setSlides(time);
    }

    @Override
    public void periodic() {
//      slides.setSlides(time);
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
