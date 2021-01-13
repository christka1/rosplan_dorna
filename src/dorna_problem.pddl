(define (problem dummy_problem) (:domain dummy_domain)
(:objects 
    start_location shared_location scan_location take_location remove_location red_location green_location blue_location unkwn - location
    scanner conv - robot
    cube - cube
    red_container - container
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

    (is_start_location start_location)
    (is_shared_location shared_location)
    (is_scan_location scan_location)
    (is_take_location take_location)
    (is_remove_location remove_location)
    (is_scanner scanner)
    (is_conv conv)
    (= (result) 0)

    ;(robot_at conv start_location)
    ;(robot_at scanner start_location)
    ;(cube_at cube shared_location)
    ;(not_holding conv cube)
    (no_cube_exist)
    ;(not_holding scanner cube)
    ;(gripper_open scanner)

    ;todo: put the initial state's facts and numeric values here
)

(:goal (and
    ;(holding scanner cube)
    ;(robot_at conv start_location)
    ;(robot_at cube scan_location)

    ;(robot_at conv shared_location)
    ;(robot_at scanner shared_location)
    ;(gripper_closed scanner)

    ;(holding scanner cube)
    ;(robot_at scanner scan_location)
    
    ;(scanned)

    ;(holding conv cube)
    ;(cube_at cube remove_location)
    ;(no_cube_exist)

    ;todo: put the goal condition here
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
