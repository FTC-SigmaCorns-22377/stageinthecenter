package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;

public class SetSlides extends Command {
    Slides slides;

    Slides.SlideHeight slideHeight;

    boolean has_set = false;

    public SetSlides(Slides slides, Slides.SlideHeight slideHeight) {
        this.slides = slides;
        this.slideHeight = slideHeight;
    }

    @Override
    public void init() {
        slides.setSlideHeight(slideHeight);
        has_set = true;
    }

    @Override
    public void periodic() {
    }

    @Override
    public boolean completed() {
        return has_set;
    }

    @Override
    public void shutdown() {

    }
}
