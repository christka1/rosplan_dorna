(define (problem task)
(:domain dummy_domain)
(:objects
    start_location shared_location scan_location take_location remove_location red_location green_location blue_location unkwn - location
    red_container - container
    scanner conv - robot
    cube - cube
)
(:init
    (allowed_at conv start_location)
    (allowed_at conv shared_location)
    (allowed_at conv take_location)
    (allowed_at conv remove_location)
    (allowed_at scanner shared_location)
    (allowed_at scanner scan_location)
    (allowed_at scanner red_location)
    (allowed_at scanner green_location)
    (allowed_at scanner blue_location)

    (is_scan_location scan_location)

    (is_start_location start_location)

    (is_shared_location shared_location)

    (is_take_location take_location)

    (is_remove_location remove_location)

    (is_scanner scanner)

    (is_conv conv)

    (not (cube_at cube shared_location))
    (not (cube_at cube blue_location))
    (not (cube_at cube red_location))

    (cube_stored cube blue_location)
    (cube_stored cube red_location)

    (robot_at conv shared_location)
    (not (robot_at conv red_location))
    (not (robot_at scanner remove_location))
    (not (robot_at scanner shared_location))
    (not (robot_at conv green_location))
    (not (robot_at scanner start_location))
    (not (robot_at conv start_location))
    (not (robot_at scanner unkwn))
    (not (robot_at scanner take_location))
    (not (robot_at conv take_location))
    (not (robot_at conv scan_location))
    (robot_at scanner scan_location)
    (not (robot_at scanner blue_location))
    (not (robot_at conv remove_location))
    (not (robot_at conv blue_location))
    (not (robot_at conv unkwn))
    (not (robot_at scanner green_location))
    (not (robot_at scanner red_location))

    (holding scanner cube)
    (not (holding conv cube))

    (not (not_holding scanner cube))
    (not_holding conv cube)

    (gripper_closed scanner)

    (not (gripper_open scanner))

    (scanned)

    (not (no_cube_exist))

    (not (cube_removed))

    (= (result) 2)

)
(:goal (and
    (cube_stored cube green_location)
))
)
