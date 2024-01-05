DOMAIN tiago_roxanne {

    TEMPORAL_MODULE temporal_module = [0, 5000], 1000;

    // Enumerators

    PAR_TYPE EnumerationParameterType location = {
        home,
        starting_point,
        table_0,
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
         * Duration: 1 to 100 time units
         * Adjacent states: At
         * Constraints: end location must be equal to destination.
         */
		VALUE _MovingTo(?dest_location) [1, 100] // States that starts with '_' are partially controllable.
		MEETS {
			At(?location);
            
            // location constraint
			?location = ?dest_location;
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
         * Adjacent states: Idle
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
         * Duration: 1 to 10 time units
         * Adjacent states: Idle
         */
        VALUE Moving(?pos) [1, 10]
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
         * Duration: 5 to 30 time units
         * Adjacent states: Idle
         */
        VALUE Playing(?motion) [5, 30]
        MEETS {
            Idle();
        }
    }

    COMP_TYPE SingletonStateVariable Robot(Idle(), Home(), MoveTo(location), Operation(location, motion)) {

        VALUE Idle() [1, +INF]
        MEETS {
            MoveTo(?to);
            Operation(?location, ?motion);
            Home();
        }

        VALUE MoveTo(?to) [1, +INF]
        MEETS {
            Idle();
        }

        VALUE Operation(?location, ?motion) [1, +INF]
        MEETS {
            Idle();
        }

        VALUE Home() [1, +INF]
        MEETS {
            Idle();
        }
    }

    COMP_TYPE SingletonStateVariable Guest(None(), Arrive(location), Leave(location)) {

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

    COMPONENT base        {FLEXIBLE positions(primitive)}   : BaseController;
    COMPONENT head        {FLEXIBLE head_goals(primitive)}  : HeadController;
    COMPONENT torso       {FLEXIBLE torso_goals(primitive)} : TorsoController;
    COMPONENT grasp       {FLEXIBLE grasp_goals(primitive)} : GraspController;
    COMPONENT play_motion {FLEXIBLE pm_goals(primitive)}    : PlayMotionController;
    COMPONENT robot       {FLEXIBLE robot_goals(primitive)} : Robot;
    COMPONENT guest       {FLEXIBLE behavior(functional)}   : Guest;


    SYNCHRONIZE guest.behavior {
        VALUE Arrive(?location) {
            d1 robot.robot_goals.Operation(?loc, ?mot);

            BEFORE [0, +INF] d1;

            ?loc = ?location;
            ?mot = shake_hands;
        }

        VALUE Leave(?location) {
            d1 robot.robot_goals.Operation(?loc, ?mot);
            
            BEFORE [0, +INF] d1;

            ?loc = ?location;
            ?mot = wave;
        }
    }

    SYNCHRONIZE robot.robot_goals {

        VALUE Home() {
            d0 <!> base.positions.At(?l0);

            CONTAINS [0, +INF] [0, +INF] d0;

            ?l0 = home;
        }

        VALUE MoveTo(?location) {
		    d0 <!> base.positions.At(?l0);

            CONTAINS [0, +INF] [0, +INF] d0;

            ?l0 = ?location;
		}

        VALUE Operation(?location, ?motion) {
            d0 <!> base.positions._MovingTo(?loc);
            d1 base.positions.At(?loc);
            d2 play_motion.pm_goals.Playing(?mot);
            d3 play_motion.pm_goals.Playing(?home_mot);
            d4 <!> base.positions._MovingTo(?home);

            CONTAINS [0, +INF] [0, +INF] d0;
            CONTAINS [0, +INF] [0, +INF] d1;
            CONTAINS [0, +INF] [0, +INF] d2;
            CONTAINS [0, +INF] [0, +INF] d3;

            d0 BEFORE [0, +INF] d1;

            d2 DURING [0, +INF] [0, +INF] d1;
            d3 DURING [0, +INF] [0, +INF] d1;

            MEETS d4;

            ?loc = ?location;
            ?mot = ?motion;

            ?home_mot = home;
            ?home = home;
        }
    }

}