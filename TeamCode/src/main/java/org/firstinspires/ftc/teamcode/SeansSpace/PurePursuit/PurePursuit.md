Pure pursuit is a path following algorithm that works by heading towards a point on the path
further ahead of the robot. The pure pursuit concept can be useful for having fast and accurate
autonomous.


For a path like this:
    _______________________
   /
  /
 /
/

The robots actual path would be like this:
        ___________________
     __/
  __/
 /
/

You first create the path by giving the robot a set of points called waypoints. The robot then
injects more points into the path to make it a lot smoother. The path then gets curved based on
a given look ahead distance. If the look ahead distance is too small,the robot will overshoot
the path and end up following the path in a zig-zag pattern. If the look ahead distance is too
large, the robot will curve the path too much and may end up running into obstacles that could
have been avoided. Typically you want a large look ahead distance that is not too large. Your
look ahead distance will change based on the type of path you are following. To utilize pure
pursuit, the robot will need a very good odometry system to accurately track its location.



This list is not accurate anymore and I am too lazy to update it.
- Sean

-----------=+(Things To Fix)+=-------------------------------------------------------------------

    -   If point1 is closer to the robot than point0, then the robot will target point1.
        This only happens with point0-1.

    -   The robot can't stop at the end of the path because it can never exactly
        reach the final point.





-----------=+(Things To Implement)+=-------------------------------------------------------------

    -   Create a way for the robot to stop at the end of its path.

    -   Implement PID. This shouldn't be too hard. Just take the motor outputs from
        PurePursuitDrivetrain and use them as input for the PID controller.

    -   Put the odometry updater into a thread for greater accuracy.

    -   Put the motor updater into a thread for greater accuracy.

    -   Rewrite Pure Pursuit D:
        - This is done. It needs testing and revisions.






Scroll Down...






















































Please help me.