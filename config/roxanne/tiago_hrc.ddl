DOMAIN TIAGO_HRC {

    /**
     * [0, 1500] -> min and max time of execution of the whole domain.
     * 1000 -> time slots (unused)
     */ 
    TEMPORAL_MODULE temporal_module = [0, 10000], 1000;

    // Enumerators

    PAR_TYPE EnumerationParameterType location = {
        home,
        starting_point,
        table_1,
        table_2
    };

    PAR_TYPE EnumerationParameterType motion = {
        // default motions
        home,
        unfold_arm,
        reach_floor,
        reach_max,
        head_tour,
        wave,
        pregrasp_weight,
        do_weights,
        pick_from_floor,
        shake_hands,
        open_hand,
        close_hand,
        pointing_hand,
        gun_hand,
        thumb_up_hand,
        pinch_hand,

        // pick motions
        pregrasp,
        pick_final_pose

    };

    PAR_TYPE NumericParameterType head_pos = [-100, 100];

    PAR_TYPE NumericParameterType torso_pos = [0, 35];

    // Component types

    /**
     * Definition of the component BaseController used to move the robot.
     */
    COMP_TYPE SingletonStateVariable BaseController(At(location), _MovingTo(location)) {

        /**
         * At is the state of the robot when it's not moving. 
         * Remains stationary at 'location'.
         * 
         * Duration: 1 to +INF time units
         * Adjacent states: _MovingTo
         */
        VALUE At(?location) [1, +INF]
		MEETS {
			_MovingTo(?dest_location);
		}

        /**
         * _MovingTo is the state of the robot when it's moving.
         * Moves to 'dest_location'.
         * 
         * Duration: 1 to 300 time units
         * Adjacent states: At
         * Constraints: end location must be equal to destination.
         */
		VALUE _MovingTo(?dest_location) [1, 300] // States that starts with '_' are partially controllable.
		MEETS {
			At(?location);
            
            // location constraint
			//?location = ?dest_location;
		}
    }

    /**
     * Definition of the component HeadController used to move head of the robot.
     */
    COMP_TYPE SingletonStateVariable HeadController(Idle(), Moving(head_pos, head_pos)) {

        /**
         * Idle state of head.
         * 
         * Duration: 1 to +INF time units
         * Adjacente states: Moving
         */
        VALUE Idle() [1, +INF]
        MEETS {
            Moving(?to_h_pos, ?to_v_pos);
        }

        /**
         * Moving is the state in which the robot head is moving.
         * The horizontal destination position of the head is 'to_h_pos'.
         * The vertical destination position of the head is 'to_v_pos'.
         * 
         * Duration: 1 to 5 time units
         * Adjacent states: LookingAt
         */
        VALUE Moving(?to_h_pos, ?to_v_pos) [1, 5]
        MEETS {
            Idle();
        }
    }

    /**
     * Definition of the component TorsoController used to lift or lower the robot's torso.
     */
    COMP_TYPE SingletonStateVariable TorsoController(Idle(), Moving(torso_pos)) {

        /**
         * Idle state of torso.
         * 
         * Duration: 1 to +INF time units
         * Adjacente states: Moving
         */
        VALUE Idle() [1, +INF]
        MEETS {
            Moving(?pos);
        }

        /**
         * Moving is the state in which the torso is lifting or lowering.
         * The destination height is 'pos'.
         * 
         * Duration: 1 to 15 time units
         * Adjacent states: Idle
         */
        VALUE Moving(?pos) [1, 15]
        MEETS {
            Idle();
        }
    }

    /**
     * Definition of the component GraspController used for picking and placing objects.
     */
    COMP_TYPE SingletonStateVariable GraspController(Idle(), Picking(), Placing()) {

        VALUE Idle() [1, +INF]
        MEETS {
            Picking();
            Placing();
        }

        VALUE Picking() [1, 30]
        MEETS {
            Idle();
        }

        VALUE Placing() [1, 30]
        MEETS {
            Idle();
        }
    }

    /**
     * Definition of the component PlayMotionController used to reproduce predefined movements 
     */
    COMP_TYPE SingletonStateVariable PlayMotionController(Idle(), Playing(motion)) {

        /**
         * Idle state.
         * 
         * Duration: 1 to +INF time units
         * Adjacent states: Playing
         */
        VALUE Idle() [1, +INF]
        MEETS {
            Playing(?motion);
        }

        /**
         * Playing is the state in which the robot is executing a predefined motion.
         * Executing the 'motion' motion.
         * 
         * Duration: 1 to 10 time units
         * Adjacent states: Idle
         */
        VALUE Playing(?motion) [1, 10]
        MEETS {
            Idle();
        }
    }

    COMP_TYPE SingletonStateVariable RobotController(Idle(), GoHome(), MoveTo(location), Operation(location, motion)) {

        VALUE Idle() [1, +INF]
        MEETS {
            MoveTo(?to);
            Operation(?location, ?motion);
            GoHome();
        }

        VALUE MoveTo(?to) [1, +INF]
        MEETS {
            Idle();
        }

        VALUE Operation(?location, ?motion) [1, +INF]
        MEETS {
            Idle();
        }

        VALUE GoHome() [1, +INF]
        MEETS {
            Idle();
        }
    }

    COMP_TYPE SingletonStateVariable GuestController(None(), Arrive(location), Leave(location)) {

        VALUE None() [0, +INF]
        MEETS {
            Arrive(?table);
            Leave(?table);
        }

        VALUE Arrive(?table) [1, 1]
        MEETS {
            None();
        }

        VALUE Leave(?table) [1, 1]
        MEETS {
            None();
        }
    }

    // Components 

    COMPONENT base        {FLEXIBLE positions(primitive)} : BaseController;
    COMPONENT head        {FLEXIBLE head_goals(primitive)} : HeadController;
    COMPONENT torso       {FLEXIBLE torso_goals(primitive)} : TorsoController;
    COMPONENT grasp       {FLEXIBLE grasp_goals(primitive)} : GraspController;
    COMPONENT play_motion {FLEXIBLE pm_goals(primitive)} : PlayMotionController;
    COMPONENT robot       {FLEXIBLE robot_goals(primitive)} : RobotController;
    COMPONENT guest       {FLEXIBLE arrivals(primitive)} : GuestController;

    SYNCHRONIZE guest.arrivals {
        VALUE Arrive(?location) {
            d0 <!> robot.robot_goals.Operation(?loc, ?motion);
            d1 robot.robot_goals.GoHome();

            d0 BEFORE [0, +INF] d1;

            BEFORE [0, +INF] d0;

            ?motion = wave;
            ?loc = ?location;
        }

        VALUE Leave(?location) {
            d0 <!> robot.robot_goals.Operation(?loc, ?motion);
            d1 robot.robot_goals.GoHome();

            d0 BEFORE [0, +INF] d1;
            
            BEFORE [0, +INF] d0;

            ?motion = shake_hands;
            ?loc = ?location;
        }
    }

    SYNCHRONIZE robot.robot_goals {

        VALUE MoveTo(?location) {
		    d0 <!> base.positions._MovingTo(?l0);

            CONTAINS [0, +INF] [0, +INF] d0;

            ?l0 = ?location;
		}

        VALUE Operation(?location, ?motion) {
            d1 <!> base.positions._MovingTo(?l1);
            d2 play_motion.pm_goals.Playing(?m0);

            CONTAINS [0, +INF] [0, +INF] d1;
            CONTAINS [0, +INF] [0, +INF] d2;

            d1 BEFORE [0, +INF] d2;

            ?l1 = ?location;
            ?m0 = ?motion;
        }

        VALUE GoHome() {
            d0 <!> base.positions._MovingTo(?l0);

            CONTAINS [0, +INF] [0, +INF] d0;

            ?l0 = home;
        }
    }

}