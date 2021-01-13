;Header and description

(define (domain dummy_domain)

;remove requirements that are not needed
(:requirements :strips :typing :fluents)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    location locatable container - object
    robot cube - locatable
)

; un-comment following line if constants are needed
;(:constants )

(:predicates ;todo: define predicates here
    (allowed_at ?obj - locatable ?loc - location)
    (is_scan_location ?loc - location)
    (is_start_location ?loc - location)
    (is_shared_location ?loc - location)
    (is_take_location ?loc - location)
    (is_remove_location ?loc - location)
    (is_scanner ?robot - robot)
    (is_conv ?robot - robot)

    (cube_at ?cube - cube ?loc - location)
    (cube_stored ?cube - cube ?loc - location)
    (robot_at ?robot - robot ?loc - location)
    (holding ?robot - robot ?cube - cube)
    (not_holding ?robot - robot ?cube - cube)
    (gripper_closed ?robot - robot)
    (gripper_open ?robot - robot)
    (scanned)
    (no_cube_exist)
    (cube_removed)
)


(:functions
    (result)- number
)

(:action store_cube
    :parameters (
        ?cube - cube
        ?loc - location
    )
    :precondition (and 
        (cube_at ?cube ?loc)
        (scanned)
    )
    :effect (and 
        (not (cube_at ?cube ?loc))
        (cube_stored ?cube ?loc)
        (no_cube_exist)
        (not (scanned))
    )
)

(:action remove_cube
    :parameters (
        ?cube - cube
        ?loc - location
    )
    :precondition (and 
        (is_remove_location ?loc)
        (cube_at ?cube ?loc)
        (scanned)
    )
    :effect (and 
        (not (cube_at ?cube ?loc))
        (no_cube_exist)
        (not (scanned))
        (cube_removed)
    )
)

(:action prepare_removal
    :parameters (
        ?robot - robot
        ?cube - cube
        ?shared_loc - location
        ?take_loc - location
    )
    :precondition (and 
        (is_shared_location ?shared_loc)
        (is_take_location ?take_loc)
        (is_conv ?robot)
        (cube_at ?cube ?shared_loc)
        (robot_at ?robot ?take_loc)
        (scanned)
    )
    :effect (and 
        (not (cube_at ?cube ?shared_loc))
        (holding ?robot ?cube)
        (not (not_holding ?robot ?cube))
    )
)


(:action drop
    :parameters (
        ?robot - robot
        ?cube - cube
        ?loc - location
    )
    :precondition (and 
        (is_scanner ?robot)
        (robot_at ?robot ?loc)
        (gripper_closed ?robot)
        (holding ?robot ?cube)
    )
    :effect (and 
        (cube_at ?cube ?loc)
        (not (gripper_closed ?robot))
        (gripper_open ?robot)
        (not (holding ?robot ?cube))
        (not_holding ?robot ?cube)
    )
)

(:action open_gripper
    :parameters (
        ?robot - robot
    )
    :precondition (and
        (is_scanner ?robot)
        (gripper_closed ?robot)
    )
    :effect (and
        (gripper_open ?robot)
        (not (gripper_closed ?robot))
    )
)

(:action scan
    :parameters (
        ?loc - location
        ?cube - cube
        ?robot - robot
    )
    :precondition (and 
        (is_scan_location ?loc)
        (is_scanner ?robot)
        (robot_at ?robot ?loc)
        (holding ?robot ?cube)
    )
    :effect (and 
        (scanned)
    )
)

(:action pick-up
    :parameters
    (
        ?robot - robot
        ?cube - cube
        ?loc - location
    )
    :precondition
     (and 
        (is_scanner ?robot)
        (robot_at ?robot ?loc)
        (cube_at ?cube ?loc)
        (gripper_open ?robot)
        (not_holding ?robot ?cube)
     )
    :effect 
    (and 
        (not (cube_at ?cube ?loc))
        (holding ?robot ?cube)
        (not (gripper_open ?robot))
        (gripper_closed ?robot)
        (not (not_holding ?robot ?cube))
    )       
)

(:action move_robot
    :parameters
    (
        ?robot - robot
        ?from - location
        ?to - location
    )
    :precondition
    (and
        (allowed_at ?robot ?to)
        (robot_at ?robot ?from)
    )
    :effect
    (and
        (not (robot_at ?robot ?from))
        (robot_at ?robot ?to)
    )
)

(:action move_conveyor
    :parameters (
        ?from - location
        ?to - location
        ?robot - robot
        ?cube - cube
    )
    :precondition (and 
        (is_conv ?robot)
        (robot_at ?robot ?from)
        (holding ?robot ?cube)
        (allowed_at ?robot ?to)
    )
    :effect (and 
        (cube_at ?cube ?to)
        (not (robot_at ?robot ?from))
        (robot_at ?robot ?to)
        (not (holding ?robot ?cube))
        (not_holding ?robot ?cube)
    )
)

;(:action get_cube
;    :parameters (
;        ?cube - cube
;        ?robot - robot
;        ?from - location
;        ?to - location
;    )
;    :precondition (and 
;        (not_holding ?robot ?cube)
;        (is_conv ?robot)
;        (cube_at ?cube ?to)
;        (robot_at ?robot ?from)
;        (is_start_location ?to)
;        (allowed_at ?robot ?to)
;    )
;    :effect (and 
;        (holding ?robot ?cube)
;        (not (not_holding ?robot ?cube))
;        (not (cube_at ?cube ?to))
;        (robot_at ?robot ?to)
;    )
;)

(:action make_cube
    :parameters (
        ?cube - cube
        ?loc - location
        ?robot - robot
    )
    :precondition (and 
        (is_conv ?robot)
        (no_cube_exist)
        (robot_at ?robot ?loc)
        (is_start_location ?loc)
    )
    :effect (and 
        (not (no_cube_exist))
        (holding ?robot ?cube)
        (not (not_holding ?robot ?cube))
        (not (cube_removed))
    )
)

)