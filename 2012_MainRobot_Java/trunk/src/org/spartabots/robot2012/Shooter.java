/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.spartabots.robot2012;

import edu.wpi.first.wpilibj.Jaguar;

public class Shooter {
    Jaguar mTopLeftSpeedController;
    Jaguar mTopRightSpeedController;
    Jaguar mBottomLeftSpeedController;
    Jaguar mBottomRightSpeedController;
    public static float kReductionFactor = 0.9f;

    public Shooter(Jaguar topLeftSpeedController, Jaguar topRightSpeedController,
            Jaguar bottomLeftSpeedController, Jaguar bottomRightSpeedController) {
	mTopLeftSpeedController = topLeftSpeedController;
	mTopRightSpeedController = topRightSpeedController;
	mBottomLeftSpeedController = bottomLeftSpeedController;
	mBottomRightSpeedController = bottomRightSpeedController;
    }

    public void setSpeed(float speed) {
            float slowSpeed = speed * kReductionFactor;

            mTopLeftSpeedController.set(slowSpeed);
            mTopRightSpeedController.set(-1 * slowSpeed);
            mBottomLeftSpeedController.set(-1 * speed);
            mBottomRightSpeedController.set(speed);
    }

    public void setSpeed(float upperSpeed, float lowerSpeed) {
            mTopLeftSpeedController.set(upperSpeed);
            mTopRightSpeedController.set(upperSpeed * -1);
            mBottomLeftSpeedController.set(lowerSpeed * -1);
            mBottomRightSpeedController.set(lowerSpeed);
    }
}