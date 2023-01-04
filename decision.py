import numpy as np


# Calculate steering angle
def calc_steering_angle(Rover):
    nav_angle = np.mean(Rover.nav_angles * 180/np.pi)
    if Rover.rock_angles is None or len(Rover.rock_angles) == 0:
        return nav_angle
    else:
        return np.clip(
            0.3 * nav_angle +
            0.7 * np.mean(Rover.rock_angles * 180/np.pi),
            -Rover.max_steer, Rover.max_steer
        )

# Check if the rover is stuck


def is_stuck(Rover):
    print(f"stuck count: {Rover.stuck_count}")
    if Rover.vel < 0.2:
        Rover.stuck_count += 1
    elif Rover.vel >= 0.2 and Rover.stuck_count >= 30:
        Rover.stuck_count = 0
        Rover.mode = 'forward'
    return Rover.stuck_count >= 30


def balance_to_right(Rover):
    return np.mean(Rover.nav_angles) > 0.


def closeto(this, that, howmuch=0.1):
    return (np.abs(np.abs(this)-np.abs(that)) < howmuch)

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function


def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    nav_dist = np.mean(Rover.nav_dists)
    nav_angle = np.mean(Rover.nav_angles * 180/np.pi)

    print(f"nav dist: {nav_dist}")
    print(f"nav angle: {nav_angle}")
    print(f"Rover vel: {Rover.vel}")

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:  # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = calc_steering_angle(Rover)

                if Rover.picking_up == 0 and Rover.near_sample == 0 and is_stuck(Rover):
                    Rover.mode = 'backward'

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            else:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                # Rover.steer = 0
                Rover.mode = 'stop'

        # we're stuck, back out...
        elif Rover.mode == 'backward':
            if (Rover.backup_count > Rover.backup_threshold) | ((Rover.backup_count > (Rover.backup_threshold/2)) & closeto(Rover.vel, 0.0, 0.2)):
                Rover.backup_count = 0
                Rover.stuck_count = 0
                Rover.steer = -Rover.max_steer
                if balance_to_right(Rover):
                    Rover.steer = Rover.max_steer
                Rover.mode = 'stop'
                Rover.throttle = 0
                print("Stopping", Rover.backup_count,
                      Rover.steer, Rover.brake, Rover.throttle, Rover.vel)
            else:
                Rover.steer = Rover.max_steer
                if balance_to_right(Rover):
                    Rover.steer = -Rover.max_steer
                Rover.brake = 0
                Rover.throttle = -Rover.throttle_set
                Rover.backup_count += 1
                if Rover.backup_count == Rover.backup_threshold:
                    print("Backing up", Rover.backup_count,
                          Rover.steer, Rover.brake, Rover.throttle, Rover.vel)

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                # Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    if (Rover.steer != -Rover.max_steer) & (Rover.steer < 0.):
                        Rover.steer = Rover.max_steer
                    elif (Rover.steer != Rover.max_steer) & (Rover.steer > 0.):
                        Rover.steer = -Rover.max_steer
                    else:
                        Rover.steer = -Rover.max_steer
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    Rover.mode = 'forward'

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    if Rover.near_sample == 1:
        Rover.brake = Rover.brake_set

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover
