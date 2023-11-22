DOMAIN TIAGO_HRC {

    /**
     * [0, 1500] -> min and max time of execution of the whole domain.
     * 1000 -> time slots (unused)
     */ 
    TEMPORAL_MODULE temporal_module = [0, 1500], 1000;

    // Enumerators

    PAR_TYPE EnumerationParameterType location = {
        home,
        test
    };

    // Component types

    /**
     * Definition of the component BaseController used to move the robot.
     */
    COMP_TYPE SingletonStateVariable BaseController(At(location), _MovingTo(dest_location)) {

        /**
         * At is the state of the robot when it's not moving. 
         * Remains stationary at 'location'.
         * 
         * Duration: 1 to +INF time units
         * Adjacent states: _MovingTo
         */
        VALUE At(?location) [1, +INF]
        // see Allen's Relations (BEFORE, AFTER, MEETS, DURING, ...) https://dl.acm.org/doi/10.1145/182.358434
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
			?location = ?dest_location;
		}
    }

    /**
     * Definition of the component HeadController used to move head of the robot.
     */
    COMP_TYPE SingletonStateVariable HeadController(LookingAt(at_h_pos, at_v_pos), Moving(to_h_pos, to_v_pos)) {

        /**
         * LookingAt is the state in which the robot head is not moving.
         * The horizontal position of the head is 'at_h_pos'.
         * The vertical position of the head is 'at_v_pos'.
         * 
         * Duration: 1 to +INF time units
         * Adjacent states: Moving
         */
        VALUE LookingAt(?at_h_pos, ?at_v_pos) [1, +INF]
        MEETS {
            Moving(?to_h_pos, ?to_v_pos);
        }

        /**
         * Moving is the state in which the robot head is moving.
         * The horizontal destination position of the head is 'to_h_pos'.
         * The vertical destination position of the head is 'to_v_pos'.
         * 
         * Duration: 1 to 30 time units
         * Adjacent states: LookingAt
         * Constraints: vertical and horizontal positions of destination are the same as the corresponding ones of LookingAt.
         */
        VALUE Moving(?to_h_pos, ?to_v_pos) [1, 30]
        MEETS {
            LookingAt(?at_h_pos, ?at_v_pos);

            // head positions constraints
            ?at_h_pos = ?to_h_pos;
            ?at_v_pos = ?to_v_pos;
        }
    }

    /**
     * Definition of the component TorsoController used to lift or lower the robot's torso.
     */
    COMP_TYPE SingletonStateVariable TorsoController(Idle(), Moving(pos)) {

        /**
         * Idle state of torso.
         * Is standing still at the height 'pos'.
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
         * Duration: 1 to 30 time units
         * Adjacent states: Idle
         */
        VALUE Moving(?pos) [1, 30]
        MEETS {
            Idle();
        }
    }

    /**
     * Definition of the component TTSController (Text To Speech) used to reproduce sentences.
     */
    COMP_TYPE SingletonStateVariable TTSController(Idle(), Talking(text)) {

        /**
         * Idle is the state in which the robot is not talking.
         * 
         * Duration: 1 to +INF time units
         * Adjacent states: Talking
         */
        VALUE Idle() [1, +INF]
        MEETS {
            Talking(?text);
        }

        /**
         * Talking is the state in which the robot is reproducing a sentence.
         * Reporducing the 'text' sentence.
         * 
         * Duration: 1 to 60 time units
         * Adjacent states: Idle
         */
        VALUE Talking(?text) [1, 60]
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
         * Duration: 1 to 50 time units
         * Adjacent states: Idle
         */
        VALUE Playing(?motion) [1, 50]
        MEETS {
            Idle();
        }
    }

    COMP_TYPE SingletonStateVariable RobotController(Idle(), MoveTo(from, to), Test()) {

        VALUE Idle() [1, +INF]
        MEETS {
            MoveTo(?from, ?to);
            Test();
        }

        VALUE MoveTo(?from, ?to) [1, +INF]
        MEETS {
            Idle();
        }

        VALUE Test() [1, +INF] 
        MEETS {
            Idle();

        }
    }

    // Components 

    COMPONENT base        {FLEXIBLE positions(primitive)} : BaseController;
    COMPONENT head        {FLEXIBLE goals(primitive)} : HeadController;
    COMPONENT tts         {FLEXIBLE goals(primitive)} : TTSController;
    COMPONENT grasp       {FLEXIBLE goals(primitive)} : GraspController;
    COMPONENT play_motion {FLEXIBLE goals(primitive)} : PlayMotionController;
    COMPONENT robot       {FLEXIBLE goals(functional)} : RobotController;

    SYNCHRONIZE robot.goals {

        VALUE Test() {
            d0 <!> base.positions._MovingTo(?l0);

            CONTAINS [0, +INF] [0, +INF] d0;

            ?l0 = test;
        }

        VALUE MoveTo(?location) {

		    d0 <!> base.positions._MovingTo(?l0);

		    CONTAINS [0, +INF] [0, +INF] d0;

		    ?l0 = ?location;
		}
    }

}