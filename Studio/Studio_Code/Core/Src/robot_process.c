/* =============================================================================
 * robot_process.c — robot state-machine implementation (runs in the main loop).
 *
 * HOW TO FINISH THE MOVE (do this in your IDE so the compiler checks you):
 *   1. In main.c, add:  #include "robot_process.h"
 *   2. In main.c, DELETE the SeqState / TestState / JoyEdges typedefs
 *      (they now live in robot_process.h). Keep the variable definitions
 *      (e.g. `SeqState seq = {0};`).
 *   3. CUT these ten functions out of main.c and PASTE them below, in order:
 *        static void Robot_State_Homing(void)
 *        static void Robot_State_Waiting(void)
 *        static void Robot_Running_Sequence(void)
 *        static void Robot_Running_BaseSystem(void)
 *        static uint8_t Robot_HandleEmergency(void)
 *        static uint8_t Robot_HandleSoftStop(void)
 *        static void Robot_UpdateTrustMode(void)
 *        static uint8_t Robot_HandleHomeIntercept(void)
 *        static void Robot_HandleGripper(void)
 *        static void Robot_HandleColorHold(void)
 *        void Robot_State_Process(void)          (this one stays non-static)
 *   4. Build. The compiler will list any symbol it can't see — for each one,
 *      add an `extern` for it in robot_process.h (most are already there).
 *
 * Everything the functions call (PWM, TRAJ_Plan, Gripper_*, PilotRamp_*,
 * SYSTEM_STATE_*, HAL_*) is reached through the headers included below.
 * ============================================================================= */

#include "robot_process.h"
#include <math.h>
#include <stdlib.h>   /* abs() */
#include <stdbool.h>

/* ---- Local wrapper: arm a move and latch the target into sys_state.cur_pos ---- */
static void TRAJ_Plan(int profile, float32_t target_deg,
                      float32_t p1, float32_t p2, float32_t p3)
{
    TrajManager_Plan(&traj_mgr, profile, target_deg, p1, p2, p3, encoder.encoder_rad);
    sys_state.cur_pos = traj_mgr.cur_pos_rad;
}

/* ---- Sequence slot → trajectory target (degrees) ----------------------------
 * Per README §3.8:
 *   magnitude = absolute hole index (0 … 71, wraps if ≥ 72)
 *   sign      = + CCW, − CW
 *
 * The hole's canonical position is always:
 *   base = home_deg + hole × DEG_PER_HOLE   (CCW side of home)
 *
 * Then we adjust so the trajectory travels in the requested direction from
 * the robot's CURRENT position.  Without this, raw = −71 would produce a
 * target of home − 355° (355° CW spin) instead of home − 5° (5° CW spin).
 *
 *   CCW (+): if base ≤ cur_deg, add 360° (target must be ahead)
 *   CW  (−): if base ≥ cur_deg, sub 360° (target must be behind)
 *
 * A 0.5° dead-band prevents a spurious full-revolution when already on-hole.
 * ---------------------------------------------------------------------------*/
static float32_t Seq_SlotToTarget(int16_t raw)
{
    float32_t home_deg = SYSTEM_STATE_convert_rad2degree(sys_state.basesystem_home_pos);
    float32_t cur_deg  = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos);
    int16_t   hole     = (int16_t)(abs(raw) % HOLE_COUNT);          /* wrap 0 … 71 */
    float32_t tgt      = home_deg + (float32_t)hole * DEG_PER_HOLE; /* CCW absolute */

    if (raw >= 0)   /* CCW: target must be strictly ahead of current position */
    {
        if (tgt <= cur_deg + 0.5f)
            tgt += 360.0f;
    }
    else            /* CW:  target must be strictly behind current position   */
    {
        if (tgt >= cur_deg - 0.5f)
            tgt -= 360.0f;
    }
    return tgt;
}

/* ---- STATE_HOMING: proximity sweep, then seed reference + start ISR ---- */
static void Robot_State_Homing(void)
{
    PilotRamp_SetPower(&robot_ramp, 1);
    PilotRamp_SetAuto(&robot_ramp, 0);   /* Manual ON during homing */

    /* Proximity sensor homing — owns PWM directly, ISR is off */
    SYSTEM_STATE_Homing(&ref_pos);
    PWM(ref_pos.vout, 0.0f);

    /* flag_ready=1 means sensor found center, encoder counter corrected */
    if (ref_pos.flag_ready == 1)
    {
        /* Sync encoder then seed Kalman from real position */
        SYSTEM_STATE_Encoder_Compute(&encoder);
        Kalman.X[0]         = encoder.encoder_rad;
        Kalman.X[1]         = 0.0f;
        Kalman.X[2]         = 0.0f;
        Kalman.X[3]         = 0.0f;
        sys_state.cur_pos   = 0.0f;
        sys_state.home_pos  = 0.0f;
        sys_state.cur_state = STATE_WAITING_COMMAND;
        PWM(0, 0);
        /* Start control loop ISR now that reference is established */
        HAL_TIM_Base_Start_IT(&htim20);
    }
}

/* ---- STATE_WAITING_COMMAND: idle, listening for joystick / BaseSystem cmds ---- */
static void Robot_State_Waiting(void)
{
    PilotRamp_SetPower(&robot_ramp, 1);

    /* ---- Joystick mode ---- */
    if (sys_state.trust == Trust_Joystick)
    {
        /* White buttons — step size depends on mode:
         * mode 0 = 5 holes per press, other modes = 1 hole per press */
        float32_t white_step = (robot_joy.mode == 0) ? (5.0f * DEG_PER_HOLE) : DEG_PER_HOLE;
        if (robot_joy.raw_btn_white_left == 0 && joy_prev.white_left == 1)
        {
            float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos) + white_step;
            TRAJ_Plan(PROFILE_SCURVE, tgt, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
            sys_state.cur_state = STATE_RUNNING;
        }
        else if (robot_joy.raw_btn_white_right == 0 && joy_prev.white_right == 1)
        {
            float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos) - white_step;
            TRAJ_Plan(PROFILE_SCURVE, tgt, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
            sys_state.cur_state = STATE_RUNNING;
        }

        /* ---- Mode 0: direct gripper control ---- */
        if (robot_joy.mode == 0)
        {
            /* Black  — toggle grip open/close */
            if (robot_joy.raw_btn_black  == 0 && joy_prev.black  == 1)
                Gripper_SetOpen(&robot_gripper, robot_gripper.grip_open ? 0 : 1);

            /* Blue   — toggle up / down */
            if (robot_joy.raw_btn_blue   == 0 && joy_prev.blue   == 1)
                Gripper_SetUp(&robot_gripper, robot_gripper.grip_up ? 0 : 1);

            /* Yellow — Reset (also used to recover from emergency — handled below) */
            if (robot_joy.raw_btn_yellow == 0 && joy_prev.yellow == 1)
            {
                /* In normal operation: reset controller integrals */
                Inner.integral   = 0.0f;
                Outer.integral   = 0.0f;
                Kalman.X[2]      = 0.0f;   /* clear disturbance estimate */
                sys_state.cur_pos = encoder.encoder_rad;
            }

            /* Red    — (unassigned) */
        }

        /* ---- Mode 1: home / pick / place ---- */
        if (robot_joy.mode == 1)
        {
            /* Black — set home to current position */
            if (robot_joy.raw_btn_black == 0 && joy_prev.black == 1)
                sys_state.home_pos = sys_state.cur_pos;

            /* Blue — go to home position */
            if (robot_joy.raw_btn_blue == 0 && joy_prev.blue == 1)
            {
                float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.home_pos);
                TRAJ_Plan(PROFILE_SCURVE, tgt, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
                sys_state.cur_state = STATE_RUNNING;
            }

            /* Red — start gripper pick */
            if (robot_joy.raw_btn_red == 0 && joy_prev.red == 1)
            {
                joy_picking          = 1;
                joy_placing          = 0;
                robot_gripper.action_phase = 0;
            }

            /* Yellow — start gripper place */
            if (robot_joy.raw_btn_yellow == 0 && joy_prev.yellow == 1)
            {
                joy_placing          = 1;
                joy_picking          = 0;
                robot_gripper.action_phase = 0;
            }

            /* pick/place state machine runs at top level each tick */
        }

        /* edge-detection history updated below, outside state machine */
    }

    /* BaseSystem mode commands */
    if (sys_state.trust == Trust_Basesystem)
    {
        /* HOME is handled at top level — no action needed here */
        if (BaseCmd.Target_Mode == CMD_MODE_JOG)
        {
            PilotRamp_SetAuto(&robot_ramp, 1);   /* Manual ON */
            if (BaseCmd.flag_jog_execute == 1)
            {
                /* Don't clear flag here — STATE_RUNNING clears it after executing */
                sys_state.cur_state = STATE_RUNNING;
            }
        }
        else if (BaseCmd.Target_Mode == CMD_MODE_AUTO)
        {
            PilotRamp_SetAuto(&robot_ramp, 0);   /* Auto ON */
            if (BaseCmd.flag_p2p_execute == 1 || BaseCmd.flag_seq_execute == 1)
                sys_state.cur_state = STATE_RUNNING;
        }
        else if (BaseCmd.Target_Mode == CMD_MODE_TEST)
        {
            PilotRamp_SetAuto(&robot_ramp, 1);   /* Manual ON */
            if (BaseCmd.flag_test_execute == 1)
                sys_state.cur_state = STATE_RUNNING;
        }
    }
}

/* ---- STATE_RUNNING helper: pick/place sequence step-advance (any trust mode) ---- */
static void Robot_Running_Sequence(void)
{
    if (!seq.active) return;

    switch (seq.phase)
    {
        /* ---- 0: trajectory to pick position ---- */
        case 0:
            if (traj_mgr.state.Complete == 1 && !traj_mgr.running)
            {
                /* Trajectory done — enter settle phase */
                seq.settle_phase_start = HAL_GetTick();
                seq.settle_ok_start    = 0;
                seq.phase = 1;
            }
            break;

        /* ---- 1: settle at pick — error < 0.1 deg for 0.5 s, or 1.5 s timeout ---- */
        case 1:
        {
            float32_t err_deg = fabsf(Kalman.X[0] - sys_state.cur_pos)
                                * (180.0f / 3.14159265f);
            if (err_deg < 0.1f)
            {
                if (seq.settle_ok_start == 0)
                    seq.settle_ok_start = HAL_GetTick();
                if (HAL_GetTick() - seq.settle_ok_start >= 500)
                    seq.phase = 2;   /* settled — go to gripper */
            }
            else
            {
                seq.settle_ok_start = 0;   /* reset continuous-ok timer */
            }
            if (HAL_GetTick() - seq.settle_phase_start >= 3000)
                seq.phase = 2;
            break;
        }

        /* ---- 2: gripper pick ---- */
        case 2:
        {
            if (robot_gripper.delay_start == 0)
                robot_gripper.delay_start = HAL_GetTick();

            uint8_t pick_done = 0;
            if (BaseCmd.Gripper_Auto_En)
                pick_done = Gripper_Pick(&robot_gripper);
            else if (HAL_GetTick() - robot_gripper.delay_start >= 1500)
                pick_done = 1;

            if (pick_done || HAL_GetTick() - robot_gripper.delay_start >= GRIPPER_TIMEOUT_MS)
            {
                robot_gripper.delay_start  = 0;
                robot_gripper.action_phase = 0;
                uint8_t   slot = seq.step * 2 + 1;
                int16_t   raw  = (slot < 10) ? BaseCmd.PickPlace_Sequence[slot] : 0;
                float32_t tgt  = Seq_SlotToTarget(raw);   /* sign=direction, wrap hole index */
                TRAJ_Plan(PROFILE_SCURVE, tgt, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
                seq.phase = 3;
            }
            break;
        }

        /* ---- 3: trajectory to place position ---- */
        case 3:
            if (traj_mgr.state.Complete == 1 && !traj_mgr.running)
            {
                seq.settle_phase_start = HAL_GetTick();
                seq.settle_ok_start    = 0;
                seq.phase = 4;
            }
            break;

        /* ---- 4: settle at place ---- */
        case 4:
        {
            float32_t err_deg = fabsf(Kalman.X[0] - sys_state.cur_pos)
                                * (180.0f / 3.14159265f);
            if (err_deg < 0.1f)
            {
                if (seq.settle_ok_start == 0)
                    seq.settle_ok_start = HAL_GetTick();
                if (HAL_GetTick() - seq.settle_ok_start >= 500)
                    seq.phase = 5;
            }
            else
            {
                seq.settle_ok_start = 0;
            }
            if (HAL_GetTick() - seq.settle_phase_start >= 3000)
                seq.phase = 5;
            break;
        }

        /* ---- 5: gripper place ---- */
        case 5:
        {
            if (robot_gripper.delay_start == 0)
                robot_gripper.delay_start = HAL_GetTick();

            uint8_t place_done = 0;
            if (BaseCmd.Gripper_Auto_En)
                place_done = Gripper_Place(&robot_gripper);
            else if (HAL_GetTick() - robot_gripper.delay_start >= 1500)
                place_done = 1;

            if (place_done || HAL_GetTick() - robot_gripper.delay_start >= GRIPPER_TIMEOUT_MS)
            {
                robot_gripper.delay_start  = 0;
                robot_gripper.action_phase = 0;
                place_done = 1;
            }

            if (place_done)
            {
                seq.step++;
                seq.phase = 0;
                if ((uint16_t)seq.step >= BaseCmd.PickPlace_Pairs)
                {
                    seq.active = 0;
                    float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.home_pos);
                    TRAJ_Plan(PROFILE_SCURVE, tgt, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
                }
                else
                {
                    uint8_t   slot = seq.step * 2;
                    int16_t   raw  = (slot < 10) ? BaseCmd.PickPlace_Sequence[slot] : 0;
                    float32_t tgt  = Seq_SlotToTarget(raw);   /* sign=direction, wrap hole index */
                    TRAJ_Plan(PROFILE_SCURVE, tgt, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
                }
            }
            break;
        }
    }
}

/* ---- STATE_RUNNING helper: BaseSystem command dispatch (JOG/P2P/SEQ/TEST) ---- */
static void Robot_Running_BaseSystem(void)
{
    if (sys_state.trust != Trust_Basesystem) return;

    /* Lamp reflects active mode: Auto=0, Manual=1 */
    if (BaseCmd.Target_Mode == CMD_MODE_AUTO || BaseCmd.Target_Mode == CMD_MODE_HOME)
        PilotRamp_SetAuto(&robot_ramp, 0);   /* Auto ON  */
    else
        PilotRamp_SetAuto(&robot_ramp, 1);   /* Manual ON (JOG / TEST / SET_HOME) */

    /* SET_HOME handled at top level via flag_sethome_execute */
    /* HOME handled at top level — preempts all states */
    /* ---- JOG: one step from current position ---- */
    if (BaseCmd.Target_Mode == CMD_MODE_JOG
             && traj_mgr.state.Complete == 1
             && BaseCmd.flag_jog_execute == 1)
    {
        BaseCmd.flag_jog_execute = 0;
        registerFrame[0x05].U16 = 0;
        float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos) + BaseCmd.Jog_Degree;
        TRAJ_Plan(PROFILE_SCURVE, tgt, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
        BaseCmd.Jog_Degree = 0.0f;
    }
    /* ---- P2P: single point-to-point move ---- */
    else if (BaseCmd.Target_Mode == CMD_MODE_AUTO
             && BaseCmd.flag_p2p_execute == 1
             && !traj_mgr.running)
    {
        BaseCmd.flag_p2p_execute = 0;
        float32_t home_deg = SYSTEM_STATE_convert_rad2degree(sys_state.basesystem_home_pos);
        float32_t target   = (BaseCmd.P2P_Unit == 1)
                             ? home_deg + BaseCmd.P2P_Target * DEG_PER_HOLE
                             : home_deg + BaseCmd.P2P_Target;
#if (DEBUG_MODE == 1)
        /* Trajectory-tuning: profile + params come from the P2P_TUNE_* #defines above. */
        #if   (P2P_TUNE_PROFILE == PROFILE_SCURVE)
            TRAJ_Plan(PROFILE_SCURVE, target, P2P_TUNE_VMAX, P2P_TUNE_AMAX, P2P_TUNE_JMAX);
        #elif (P2P_TUNE_PROFILE == PROFILE_TRAPEZOID)
            TRAJ_Plan(PROFILE_TRAPEZOID, target, P2P_TUNE_TIME, P2P_TUNE_ACCT, 0.0f);
        #else  /* PROFILE_MINJERK */
            TRAJ_Plan(PROFILE_MINJERK, target, P2P_TUNE_TIME, 0.0f, 0.0f);
        #endif
        p2p_grip_pending = 1;   /* fire gripper once this move arrives */
#else
        TRAJ_Plan(PROFILE_SCURVE, target, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
#endif
    }
    /* ---- SEQUENCE: pick-place pairs ---- */
    else if (BaseCmd.Target_Mode == CMD_MODE_AUTO
             && BaseCmd.flag_seq_execute == 1
             && !seq.active)
    {
        BaseCmd.flag_seq_execute = 0;
        seq.active = 1;
        seq.step   = 0;
        seq.phase  = 0;
        {
            int16_t   raw = BaseCmd.PickPlace_Sequence[0];
            float32_t tgt = Seq_SlotToTarget(raw);   /* sign=direction, wrap hole index */
            TRAJ_Plan(PROFILE_SCURVE, tgt, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
        }
    }
    /* ---- TEST mode trigger ---- */
    else if (BaseCmd.Target_Mode == CMD_MODE_TEST
             && BaseCmd.flag_test_execute == 1
             && !test.active
             && traj_mgr.state.Complete == 1)
    {
        BaseCmd.flag_test_execute = 0;
        PilotLamp_SetWhite(1);
        test.active      = 1;
        test.phase       = 0;
        test.repeat      = 0;
        test.delay_start = 0;

        /* Decode unit from sign of Test_Repeat_Count:
         * negative = index mode, positive = degree mode */
        uint8_t   is_index   = (BaseCmd.Test_Repeat_Count < 0.0f);
        float32_t test_final = is_index
                               ? BaseCmd.Test_Final_Pos * DEG_PER_HOLE
                               : BaseCmd.Test_Final_Pos;

        if (BaseCmd.Test_Type == CMD_TEST_PERFORM)
        {
            /* --- PERFORM: S-Curve, ref → ref+360° → delay 0.5s → ref --- */
            float32_t vmax    = BaseCmd.Test_Velocity > 0.01f ? BaseCmd.Test_Velocity : 1.0f;
            float32_t amax    = BaseCmd.Test_Accel    > 0.01f ? BaseCmd.Test_Accel    : 2.0f;
            if (vmax > 4.0f) vmax = 4.0f;
            if (amax > 8.0f) amax = 8.0f;
            float32_t ref_deg = SYSTEM_STATE_convert_rad2degree(sys_state.basesystem_home_pos);
            TRAJ_Plan(PROFILE_SCURVE, ref_deg + 360.0f, vmax, amax, 2.5f * amax);
        }
        else
        {
            /* --- PRECISION: MinJerk, Init → Final → 1s → back → repeat --- */
            TRAJ_Plan(PROFILE_SCURVE, test_final, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
        }
    }
    /* ---- TEST step advance ---- */
    else if (test.active)
    {
        /* Recalculate converted positions — sign of repeat encodes unit */
        uint8_t   t_is_index = (BaseCmd.Test_Repeat_Count < 0.0f);
        float32_t t_init     = t_is_index
                              ? BaseCmd.Test_Init_Pos  * DEG_PER_HOLE
                              : BaseCmd.Test_Init_Pos;
        float32_t t_final    = t_is_index
                              ? BaseCmd.Test_Final_Pos * DEG_PER_HOLE
                              : BaseCmd.Test_Final_Pos;
        int t_repeat_count   = (int)fabsf(BaseCmd.Test_Repeat_Count);
        float32_t ref_deg = SYSTEM_STATE_convert_rad2degree(sys_state.basesystem_home_pos);

        if (BaseCmd.Test_Type == CMD_TEST_PERFORM)
        {
            float32_t vmax = BaseCmd.Test_Velocity > 0.01f ? BaseCmd.Test_Velocity : 1.0f;
            float32_t amax = BaseCmd.Test_Accel    > 0.01f ? BaseCmd.Test_Accel    : 2.0f;
            if (vmax > 4.0f) vmax = 4.0f;
            if (amax > 8.0f) amax = 8.0f;

            switch (test.phase)
            {
                case 0: /* going ref+360° — wait done */
                    if (traj_mgr.state.Complete == 1 && !traj_mgr.running)
                    {
                        test.delay_start = HAL_GetTick();
                        test.phase = 1;
                    }
                    break;

                case 1: /* 0.5s delay */
                    if (HAL_GetTick() - test.delay_start >= 500)
                    {
                        test.delay_start = 0;
                        test.phase = 2;
                        TRAJ_Plan(PROFILE_SCURVE, ref_deg, vmax, amax, 2.5f * amax);
                    }
                    break;

                case 2: /* going back to ref — wait done */
                    if (traj_mgr.state.Complete == 1 && !traj_mgr.running)
                    {
                        test.active = 0;
                        PilotLamp_SetWhite(0);
                    }
                    break;
            }
        }
        else /* PRECISION */
        {
            switch (test.phase)
            {
                case 0: /* going to Final (go) */
                    if (traj_mgr.state.Complete == 1 && !traj_mgr.running)
                    {
                        test.delay_start = HAL_GetTick();
                        test.phase = 1;
                    }
                    break;

                case 1: /* 1s delay after go */
                    if (HAL_GetTick() - test.delay_start >= 1000)
                    {
                        test.delay_start = 0;
                        test.phase = 2;
                        TRAJ_Plan(PROFILE_SCURVE, t_init, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
                    }
                    break;

                case 2: /* going to Init (back) */
                    if (traj_mgr.state.Complete == 1 && !traj_mgr.running)
                    {
                        test.delay_start = HAL_GetTick();
                        test.phase = 3;
                    }
                    break;

                case 3: /* 1s delay after back — count round trip */
                    if (HAL_GetTick() - test.delay_start >= 1000)
                    {
                        test.repeat++;
                        if (test.repeat >= t_repeat_count)
                        {
                            test.active = 0;
                            PilotLamp_SetWhite(0);
                        }
                        else
                        {
                            test.delay_start = 0;
                            test.phase = 0;
                            TRAJ_Plan(PROFILE_SCURVE, t_final, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
                        }
                    }
                    break;
            }
        }
    }
}

/* ---- Emergency button (PA4). Returns 1 if emergency active (skip rest). ---- */
/* emergency_latch:
 *   0 = normal
 *   1 = emergency button held down
 *   2 = button released, waiting for joystick Yellow reset */
static uint8_t Robot_HandleEmergency(void)
{
    uint8_t emer_pin = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

    if (emer_pin == 0 && emergency_latch == 0)
    {
        /* Button just pressed — emergency triggered */
        emergency_latch = 1;
        HAL_TIM_Base_Stop_IT(&htim20);
        traj_mgr.running = 0;
        seq.active       = 0;
        PWM(0.0f, 0.0f);
        PilotRamp_SetPower(&robot_ramp, 0);
    }
    else if (emer_pin == 1 && emergency_latch == 1)
    {
        /* Button released — wait for joystick Yellow reset before recovering */
        emergency_latch = 2;
    }

    /* latch == 2: Yellow edge triggers recovery (home memory preserved) */
    if (emergency_latch == 2
        && robot_joy.raw_btn_yellow == 0 && joy_prev.yellow == 1)
    {
        emergency_latch = 0;

        SYSTEM_STATE_Encoder_Compute(&encoder);
        Kalman.X[0]  = encoder.encoder_rad;
        Kalman.X[1]  = 0.0f;
        Kalman.X[2]  = 0.0f;
        Kalman.X[3]  = 0.0f;
        Inner.integral = 0.0f;
        Outer.integral = 0.0f;
        sys_state.cur_pos = encoder.encoder_rad;
        /* NOTE: home_pos and basesystem_home_pos are intentionally NOT touched */

        ref_pos.flag_ready      = 0;
        ref_pos.state_detection = 0;
        ref_pos.proximity_flag  = 0;
        sys_state.cur_state     = STATE_HOMING;
    }

    return (emergency_latch != 0);   /* non-zero = skip rest of state machine */
}

/* ---- Soft stop (joystick or BaseSystem). Returns 1 if triggered. ---- */
static uint8_t Robot_HandleSoftStop(void)
{
    bool soft_stop = (robot_joy.soft_stop == 1) || (BaseCmd.Soft_Stop_Req == 1);
    if (soft_stop)
    {
        traj_mgr.state.Complete = 1;
        traj_mgr.running        = 0;
        seq.active          = 0;
        test.active         = 0;
        PilotLamp_SetWhite(0);
        PilotRamp_SetPower(&robot_ramp, 0);   /* Power OFF */
        sys_state.cur_state = STATE_WAITING_COMMAND;
        return 1;
    }
    return 0;
}

/* ---- Trust-mode select (PB0) + manual lamp ---- */
static void Robot_UpdateTrustMode(void)
{
    int sw = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    sys_state.trust = (sw == 0) ? Trust_Joystick : Trust_Basesystem;
    if (sys_state.trust == Trust_Joystick)
        PilotRamp_SetAuto(&robot_ramp, 1);   /* Manual ON */
}

/* ---- BaseSystem HOME intercept. Returns 1 if it preempted this tick. ---- */
static uint8_t Robot_HandleHomeIntercept(void)
{
    if (sys_state.trust == Trust_Basesystem && BaseCmd.Target_Mode == CMD_MODE_HOME)
    {
        /* Cancel everything */
        traj_mgr.running = 0;
        seq.active       = 0;
        test.active      = 0;

        /* Sync cur_pos so the ISR holds cleanly for one tick */
        sys_state.cur_pos = Kalman.X[0];

        float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.home_pos);
        TRAJ_Plan(PROFILE_SCURVE, tgt, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);
        BaseCmd.Target_Mode  = CMD_MODE_IDLE;
        PilotRamp_SetAuto(&robot_ramp, 0);   /* Auto ON */
        sys_state.cur_state  = STATE_RUNNING;
        return 1;
    }
    return 0;
}

/* ---- Gripper: manual cmds, pick/place sequence start, SET_HOME (any state) ---- */
static void Robot_HandleGripper(void)
{
    /* GRIPPER MANUAL — works in any state / any trust mode */
    if (BaseCmd.flag_gripper_manual == 1)
    {
        BaseCmd.flag_gripper_manual = 0;
        switch (BaseCmd.Gripper_Manual)
        {
            case CMD_GRIPPER_UP:    Gripper_SetUp  (&robot_gripper, 1); break;
            case CMD_GRIPPER_DOWN:  Gripper_SetUp  (&robot_gripper, 0); break;
            case CMD_GRIPPER_OPEN:  Gripper_SetOpen(&robot_gripper, 0); break;
            case CMD_GRIPPER_CLOSE: Gripper_SetOpen(&robot_gripper, 1); break;
        }
    }

    /* GRIPPER SEQ (pick / place) — 1 = pick, 2 = place */
    if (BaseCmd.flag_gripper_seq == 1)
    {
        BaseCmd.flag_gripper_seq   = 0;
        robot_gripper.action_phase = 0;
        if (BaseCmd.Gripper_Seq == 1) { joy_picking = 1; joy_placing = 0; }
        if (BaseCmd.Gripper_Seq == 2) { joy_placing = 1; joy_picking = 0; }
    }

    /* Run pick/place state machine (shared with joystick mode 1) */
    if (joy_picking)
    {
        if (Gripper_Pick(&robot_gripper))  joy_picking = 0;
    }
    else if (joy_placing)
    {
        if (Gripper_Place(&robot_gripper)) joy_placing = 0;
    }

#if (DEBUG_MODE == 1)
    /* DEBUG_MODE 1: after a P2P move arrives, fire the selected gripper action */
    if (p2p_grip_pending
        && traj_mgr.state.Complete == 1
        && !traj_mgr.running
        && fabsf(encoder.encoder_rad - sys_state.cur_pos) <= 0.001745f)   /* ~0.1 deg */
    {
        p2p_grip_pending = 0;
        robot_gripper.action_phase = 0;
        if      (BaseCmd.Gripper_Seq == 1) { joy_picking = 1; joy_placing = 0; }  /* Pick  */
        else if (BaseCmd.Gripper_Seq == 2) { joy_placing = 1; joy_picking = 0; }  /* Place */
    }
#endif

    /* SET_HOME from BaseSystem — snapshot current pos as BS coordinate origin */
    if (BaseCmd.flag_sethome_execute == 1)
    {
        BaseCmd.flag_sethome_execute  = 0;
        sys_state.basesystem_home_pos = encoder.encoder_rad;
    }
}

/* ---- Color button 2s hold → move to robot reference (0 deg), any mode ---- */
static void Robot_HandleColorHold(void)
{
    uint8_t any_color_held = (robot_joy.raw_btn_black  == 0)
                           || (robot_joy.raw_btn_blue   == 0)
                           || (robot_joy.raw_btn_red    == 0)
                           || (robot_joy.raw_btn_yellow == 0);

    if (any_color_held)
    {
        if (color_hold_start == 0)
            color_hold_start = HAL_GetTick();

        if (HAL_GetTick() - color_hold_start >= 2000)
        {
            color_hold_start = 0;
            traj_mgr.running = 0;
            seq.active       = 0;
            test.active      = 0;
            TRAJ_Plan(PROFILE_SCURVE, 0.0f, TRAJ_VMAX, TRAJ_AMAX, TRAJ_JMAX);   /* reference = 0 deg */
            sys_state.cur_state = STATE_RUNNING;
        }
    }
    else
    {
        color_hold_start = 0;   /* reset if released */
    }
}

/* =========================================================================
 * Robot_State_Process — runs in main loop (non-real-time)
 * ========================================================================= */
void Robot_State_Process(void)
{
    SYSTEM_STATE_Joystick_Update(&robot_joy);

    if (Robot_HandleEmergency()) return;   /* emergency latch — skip rest */
    if (Robot_HandleSoftStop())  return;   /* soft stop — back to WAITING */

    Robot_UpdateTrustMode();               /* PB0 select + manual lamp */

    if (Robot_HandleHomeIntercept()) return;   /* BaseSystem HOME preempts */

    Robot_HandleGripper();                 /* manual + seq + pick/place + SET_HOME */
    Robot_HandleColorHold();               /* 2s color hold → reference */

    switch (sys_state.cur_state)
    {
        /* ------------------------------------------------------------------ */
        case STATE_HOMING:
            Robot_State_Homing();
            break;


        /* ------------------------------------------------------------------ */
        case STATE_WAITING_COMMAND:
            Robot_State_Waiting();
            break;

        /* ------------------------------------------------------------------ */
        case STATE_RUNNING:
            PilotRamp_SetPower(&robot_ramp, 1);
            /* Lamp set per-mode inside handlers — do NOT override here */

            /* ---- BaseSystem command dispatch (JOG/P2P/SEQ/TEST) ---- */
            Robot_Running_BaseSystem();

            /* ---- Sequence step-advance (runs regardless of trust mode) ---- */
            Robot_Running_Sequence();
            /* ---- Auto-return to WAITING after move completes (Basesystem) ---- */
            if (sys_state.trust == Trust_Basesystem
                && traj_mgr.state.Complete == 1
                && !traj_mgr.running
                && !seq.active
                && !test.active)
            {
                PilotLamp_SetWhite(0);
                sys_state.cur_state = STATE_WAITING_COMMAND;
            }

            /* ---- Joystick: auto-return to WAITING when move completes ---- */
            if (sys_state.trust == Trust_Joystick
                && traj_mgr.state.Complete == 1 && !traj_mgr.running
                && !seq.active)
            {
                sys_state.cur_state = STATE_WAITING_COMMAND;
            }

            break;

        /* ------------------------------------------------------------------ */
        default:
            sys_state.cur_state = STATE_WAITING_COMMAND;
            break;
    }

    /* Always update edge-detection history regardless of state (raw values) */
    joy_prev.white_left  = robot_joy.raw_btn_white_left;
    joy_prev.white_right = robot_joy.raw_btn_white_right;
    joy_prev.black       = robot_joy.raw_btn_black;
    joy_prev.blue        = robot_joy.raw_btn_blue;
    joy_prev.yellow      = robot_joy.raw_btn_yellow;
    joy_prev.red         = robot_joy.raw_btn_red;
}
