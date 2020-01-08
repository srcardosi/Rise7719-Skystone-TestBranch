package org.firstinspires.ftc.teamcode.SeansSpace.SeanThreading;

/**
 * Created by Sean Cardosi on 2019-11-30.
 */
public class SeanThread implements Runnable {

    private boolean isRunning = true;

    @Override
    public void run() {

        while (isRunning) {

            //noinspection EmptyTryBlock This just stops the inspection from yelling at me
            try {

                //Do Stuff

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
    public void stop() {
        isRunning = false;
    }
}
