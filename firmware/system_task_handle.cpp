// =====================================================================================
//  NexArm ACCURACY & REPEATABILITY TEST  -  pick-and-place between two taught stations
//
//  Industrial task: pick-and-place between two taught stations A and B. The arm picks the
//  object at A and places it at B, then picks it at B and places it back at A, again and
//  again ("run N" = N trips, no operator needed).
//
//  Positions are TAUGHT, like on an industrial robot with a teach pendant: you move the arm
//  with small jog commands and save the pose (A, B). Nothing is measured with a ruler.
//
//  Measurement (ISO 9283 style): every time the gripper arrives at A or B (to pick or to
//  place) the program reads the position the arm itself reports (its servo encoders through
//  Hiwonder's forward kinematics, 1 mm resolution) and prints a LOG line. "stats" gives,
//  per station: accuracy AP = distance between the mean arrival and the taught point,
//  repeatability RP = mean spread + 3 standard deviations (ISO 9283 formula).
//  The real placement of the object is checked by hand on paper outlines (see README).
//
//  Built on Hiwonder's official NexArm example (inverse kinematics in the arm, screen,
//  buttons, buzzer). This file replaces the example's system_task_handle.cpp; every other
//  file in this folder is Hiwonder's, unchanged.
//
//  Console 115200 - type  help .   KEY1 = one trip,  KEY2 = stop.
//
//  PC LINK (used by the Python FSM + HMI on the laptop): lines starting with '@'.
//    @ping                      -> PONG,nexarm-pclink,1
//    @tel N                     PC mode on, a TEL line every N ms (0 = PC mode off)
//    @mv x y z pitch roll claw ms   one short segment of the trajectory planned by the PC
//    @estop / @reset            latch / clear the E-stop (mv refused while latched)
//  The arm sends:
//    TEL,ms,x,y,z,pitch,roll,claw,claw_raw,estop,feedback_age_ms   (measured, not commanded)
//    EVT,KEY1   EVT,ESTOP,KEY2|PC   EVT,RESET   ERR,...
//  In PC mode KEY2 is a hardware E-stop handled HERE, so it works even if the PC freezes.
// =====================================================================================
#include "Arduino.h"
#include "system_task_handle.h"
#include "Robot_Arm.h"          // official Hiwonder library: provides `arm` and `servo`
#include <Preferences.h>
#include <string.h>
#include <math.h>

// =============================== 1. SETTINGS =========================================
static float CLAW_OPEN = -50.0f;        // official example values (open / closed)
static float CLAW_GRIP = 0.0f;          // change with:  grip A   (saved)
static const float APPROACH = 80.0f;    // come down to a station from this far above (mm)
static const float PLACE_LIFT = 2.0f;   // release the object this much above the taught height (mm)
static const float Z_SAFE_MAX = 250.0f; // never travel higher than this
static const uint16_t T_MOVE = 1500, T_SLOW = 900, T_CLAW = 700, T_JOG = 500;
static const uint8_t  MOVE_ACC = 16;    // acceleration, as in the official examples

// =============================== 2. OFFICIAL PLUMBING ================================
// (from Hiwonder's example: receive the arm's position feedback from the AT32 co-processor)
static CommProtocol_t at32_protocol;
static int16_t  last_servo_positions[6] = {2048, 2048, 2048, 2048, 2048, 2048};
static uint32_t last_servo_feedback_ms = 0;

void at32_packet_callback(PacketTypeDef* rx) {
    if (rx->elements.id != 0x5A || rx->elements.cmd != CMD_GET_CUR_COORDS) return;
    if (rx->elements.length >= 14) {
        const uint8_t* a = rx->elements.args;
        arm.current_pose.x     = (float)(int16_t)((a[1] << 8) | a[0]);
        arm.current_pose.y     = (float)(int16_t)((a[3] << 8) | a[2]);
        arm.current_pose.z     = (float)(int16_t)((a[5] << 8) | a[4]);
        arm.current_pose.pitch = (float)(int16_t)((a[7] << 8) | a[6]) / 10.0f;
        arm.current_pose.roll  = (float)(int16_t)((a[9] << 8) | a[8]);
        arm.current_pose.claw  = (float)(int16_t)((a[11] << 8) | a[10]);
    }
    if (rx->elements.length >= 26) {
        for (int i = 0; i < 6; ++i) {
            int k = 12 + i * 2;
            last_servo_positions[i] = (int16_t)((rx->elements.args[k + 1] << 8) | rx->elements.args[k]);
        }
        last_servo_feedback_ms = millis();
    }
}

void pump_at32_feedback(void) {
    while (servo.uart->available()) {
        uint8_t c = servo.uart->read();
        at32_protocol.parsing(&c, 1);
    }
}

bool sync_arm_feedback(uint32_t timeout_ms) {
    arm.update_status();
    uint32_t t0 = millis(), before = last_servo_feedback_ms;
    while (millis() - t0 < timeout_ms) {
        pump_at32_feedback();
        if (last_servo_feedback_ms != 0 && last_servo_feedback_ms != before) return true;
        delay(2);
    }
    return last_servo_feedback_ms != 0;
}

bool get_last_servo_positions(int16_t out_pos[6]) {
    if (last_servo_feedback_ms == 0) return false;
    memcpy(out_pos, last_servo_positions, sizeof(last_servo_positions));
    return true;
}

// =============================== 3. TAUGHT DATA ======================================
struct Pose { float x, y, z, pitch, roll; bool ok; };   // roll = gripper turned around its own axis
static Pose P_A = {0, 0, 0, -90.0f, 0.0f, false};   // station A: gripper around the object
static Pose P_B = {0, 0, 0, -90.0f, 0.0f, false};   // station B: gripper around the object

static Pose jog = {200.0f, 0.0f, 200.0f, -90.0f, 0.0f, true};   // the pose you are moving now
static float roll_cmd = 0.0f;                                     // gripper turn used by the next move
static float claw_now = -50.0f;

static Preferences prefs;
static bool busy = false;        // the task is running: only 'stop' is accepted
static bool stop_req = false;    // KEY2 or 'stop': stop after the current motion
static int  trips_done = 0;
static char obj_at = 'A';        // where the object is now (tell it with:  at A  /  at B)

static void handle_command(char* line);

static void save_all(void) {
    prefs.begin("abtest", false);
    prefs.putBytes("A", &P_A, sizeof(Pose));
    prefs.putBytes("B", &P_B, sizeof(Pose));
    prefs.putFloat("grip", CLAW_GRIP);
    prefs.end();
}

static void load_all(void) {
    prefs.begin("abtest", true);
    if (prefs.getBytesLength("A") == sizeof(Pose)) prefs.getBytes("A", &P_A, sizeof(Pose));
    if (prefs.getBytesLength("B") == sizeof(Pose)) prefs.getBytes("B", &P_B, sizeof(Pose));
    CLAW_GRIP = prefs.getFloat("grip", CLAW_GRIP);
    prefs.end();
}

static bool all_taught(void) { return P_A.ok && P_B.ok; }

// =============================== 4. SCREEN, SERIAL, WAITING ==========================
static void screen(const char* line3) {
    char l2[24], l4[24];
    snprintf(l2, sizeof(l2), "A:%s B:%s obj at %c", P_A.ok ? "ok" : "--", P_B.ok ? "ok" : "--", obj_at);
    snprintf(l4, sizeof(l4), "Trips:%d KEY1 go KEY2", trips_done);
    arm.board.oled.set_custom_text(0, "A<->B Repeatability");
    arm.board.oled.set_custom_text(1, l2);
    arm.board.oled.set_custom_text(2, line3);
    arm.board.oled.set_custom_text(3, l4);
    arm.board.oled.show_custom();
}

static bool ok_req = false;     // KEY1 or 'ok' while the task waits for you

static void read_serial(void) {
    static char buf[64];
    static int n = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c != '\r' && c != '\n') { if (n < (int)sizeof(buf) - 1) buf[n++] = c; continue; }
        // Enter: the console may send CR, LF or both - each ends the line (empty lines are skipped)
        buf[n] = 0;
        char line[64];
        strcpy(line, buf);
        n = 0;
        if (line[0] == 0) continue;
        if (busy) {
            if (!strcmp(line, "stop") || !strcmp(line, "s")) { stop_req = true; Serial.println("[STOP] stopping after this motion"); }
            else if (!strcmp(line, "ok")) ok_req = true;
            else Serial.println("[BUSY] the task is running - only 'stop' (or 'ok' when asked) works now");
        } else {
            handle_command(line);
        }
    }
}

// Waits while keeping everything alive: arm feedback, buttons, buzzer, serial (stop)
static void wait_ms(uint32_t ms) {
    static uint32_t last_sync = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        pump_at32_feedback();
        arm.board.buzzer.update();
        arm.board.button.update();
        if (busy && arm.board.button.is_clicked(1)) { stop_req = true; Serial.println("[STOP] KEY2"); }
        if (busy && arm.board.button.is_clicked(0)) ok_req = true;
        if (millis() - last_sync >= 500) { last_sync = millis(); arm.update_status(); }
        if (busy) read_serial();
        delay(2);
    }
}

// =============================== 5. MOTION ===========================================
// Moves the gripper tip to (x, y, z) with the given pitch. False if a stop was requested.
static bool move_arm(float x, float y, float z, float pitch, float claw, uint16_t ms) {
    if (stop_req) return false;
    x = constrain(x, arm.limit_x_min, arm.limit_x_max);
    y = constrain(y, arm.limit_y_min, arm.limit_y_max);
    z = constrain(z, arm.limit_z_min, arm.limit_z_max);
    arm.set_torque(true);
    arm.set_move_acc(MOVE_ACC);
    arm.move(x, y, z, pitch, roll_cmd, claw, ms);
    jog = {x, y, z, pitch, roll_cmd, true};
    claw_now = claw;
    wait_ms(ms + 250);
    return !stop_req;
}

static bool move_pose(const Pose& p, float claw, uint16_t ms) { roll_cmd = p.roll; return move_arm(p.x, p.y, p.z, p.pitch, claw, ms); }

static float safe_z(float a, float b) { return fminf(fmaxf(a, b) + APPROACH, Z_SAFE_MAX); }

// ---------- measurement: where does the arm REALLY arrive? ----------
struct Sample { float x, y, z; };
static const int MAX_SAMPLES = 100;
static Sample samples[2][MAX_SAMPLES];          // [0] = station A, [1] = station B
static int    n_samples[2] = {0, 0};

// Reads the position reported by the arm (average of 3 fresh readings), stores it, prints a LOG line.
static void measure_arrival(int st, const Pose& t, const char* action) {
    wait_ms(300);                                // let the arm settle
    float sx = 0, sy = 0, sz = 0;
    int k = 0;
    for (int i = 0; i < 3; ++i) {
        if (sync_arm_feedback(200)) { sx += arm.current_pose.x; sy += arm.current_pose.y; sz += arm.current_pose.z; ++k; }
    }
    if (k == 0) { Serial.println("[MEASURE] no position feedback from the arm"); return; }
    Sample m = {sx / k, sy / k, sz / k};
    if (n_samples[st] < MAX_SAMPLES) samples[st][n_samples[st]++] = m;
    const float tz = (strcmp(action, "place") == 0) ? t.z + PLACE_LIFT : t.z;
    const float ex = m.x - t.x, ey = m.y - t.y, ez = m.z - tz;
    // LOG,trip,station,action,target_x,target_y,target_z,real_x,real_y,real_z,error_mm
    Serial.printf("LOG,%d,%c,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f\n", trips_done + 1, 'A' + st, action,
                  t.x, t.y, tz, m.x, m.y, m.z, sqrtf(ex * ex + ey * ey + ez * ez));
}

// ISO 9283: AP = distance from the taught point to the mean arrival (accuracy)
//           RP = mean(l) + 3*std(l), l = distance of each arrival from the mean (repeatability)
static void print_stats(void) {
    Serial.println("\n=== RESULTS (arm's own position feedback, 1 mm resolution) ===");
    for (int st = 0; st < 2; ++st) {
        const int n = n_samples[st];
        const Pose& t = st ? P_B : P_A;
        if (n == 0) { Serial.printf(" Station %c: no data yet\n", 'A' + st); continue; }
        float mx = 0, my = 0, mz = 0;
        for (int i = 0; i < n; ++i) { mx += samples[st][i].x; my += samples[st][i].y; mz += samples[st][i].z; }
        mx /= n; my /= n;  mz /= n;
        const float ap_xy = hypotf(mx - t.x, my - t.y);
        float lsum = 0, lmax = 0, l[MAX_SAMPLES];
        for (int i = 0; i < n; ++i) {
            l[i] = hypotf(samples[st][i].x - mx, samples[st][i].y - my);
            lsum += l[i]; lmax = fmaxf(lmax, l[i]);
        }
        const float lbar = lsum / n;
        float var = 0;
        for (int i = 0; i < n; ++i) var += (l[i] - lbar) * (l[i] - lbar);
        const float sd = (n > 1) ? sqrtf(var / (n - 1)) : 0.0f;
        Serial.printf(" Station %c (%d arrivals): taught (%.0f, %.0f)  mean (%.1f, %.1f)\n", 'A' + st, n, t.x, t.y, mx, my);
        Serial.printf("   Accuracy      AP = %.2f mm   (mean offset x %+.2f, y %+.2f)\n", ap_xy, mx - t.x, my - t.y);
        Serial.printf("   Repeatability RP = %.2f mm   (mean spread %.2f, std %.2f, max %.2f)\n", lbar + 3 * sd, lbar, sd, lmax);
    }
    Serial.println(" (x/y in the table plane; LOG lines above give every arrival for the report)\n");
}

// Pick at a station: above it, straight down, MEASURE, close, straight up
static bool pick(const Pose& s, int st) {
    roll_cmd = s.roll;
    const float up = safe_z(s.z, jog.z - APPROACH);
    if (!(move_arm(s.x, s.y, up,  s.pitch, CLAW_OPEN, T_MOVE) &&
          move_arm(s.x, s.y, s.z, s.pitch, CLAW_OPEN, T_SLOW))) return false;
    measure_arrival(st, s, "pick");
    return move_arm(s.x, s.y, s.z, s.pitch, CLAW_GRIP, T_CLAW) &&
           move_arm(s.x, s.y, up,  s.pitch, CLAW_GRIP, T_SLOW);
}

// Place at a station: above it, straight down, MEASURE, open, straight up
static bool place(const Pose& s, int st) {
    roll_cmd = s.roll;
    const float up = safe_z(s.z, jog.z - APPROACH);
    if (!(move_arm(s.x, s.y, up,  s.pitch, CLAW_GRIP, T_MOVE) &&
          move_arm(s.x, s.y, s.z + PLACE_LIFT, s.pitch, CLAW_GRIP, T_SLOW))) return false;
    measure_arrival(st, s, "place");
    return move_arm(s.x, s.y, s.z + PLACE_LIFT, s.pitch, CLAW_OPEN, T_CLAW) &&
           move_arm(s.x, s.y, up,  s.pitch, CLAW_OPEN, T_SLOW);
}

// =============================== 6. THE TASK =========================================
// One trip: pick where the object is (A or B), place it at the other station.
static bool one_trip(void) {
    const int from = (obj_at == 'A') ? 0 : 1, to = 1 - from;
    const Pose& F = from ? P_B : P_A;
    const Pose& T = to   ? P_B : P_A;
    char msg[40];
    snprintf(msg, sizeof(msg), "Trip %d: %c -> %c", trips_done + 1, 'A' + from, 'A' + to);
    Serial.printf("[TRIP %d] %c -> %c\n", trips_done + 1, 'A' + from, 'A' + to);
    screen(msg);
    if (!pick(F, from)) return false;
    if (!place(T, to)) return false;
    obj_at = 'A' + to;
    ++trips_done;
    return true;
}

// N trips back and forth, no operator needed. Results at the end.
static void run_trips(int count) {
    if (!all_taught()) { Serial.println("[TASK] teach first: save A, save B (type  list)"); return; }
    busy = true; stop_req = false;
    Serial.printf("[TASK] %d trip(s), object starts at %c\n", count, obj_at);
    bool ok = true;
    for (int i = 1; i <= count && ok; ++i) ok = one_trip();
    if (!ok) {
        Serial.printf("[TASK] stopped by you. The object should be at %c or in the gripper (type  open  if needed,\n"
                      "       then put it on a station and type  at A  or  at B)\n", obj_at);
        screen("STOPPED");
    } else {
        Serial.printf("[TASK] DONE - %d trip(s) in total, object now at %c\n", trips_done, obj_at);
        arm.reset_all(2000);
        wait_ms(2200);
        arm.board.buzzer.set(80, 80, 2, 2200);
        screen("DONE - type stats");
        print_stats();
    }
    busy = false;
}

// =============================== 7. TEACHING COMMANDS ================================
static void print_pose(const char* name, const Pose& p) {
    if (p.ok) Serial.printf("   %-2s x=%.0f y=%.0f z=%.0f pitch=%.0f roll=%.0f\n", name, p.x, p.y, p.z, p.pitch, p.roll);
    else      Serial.printf("   %-2s NOT taught\n", name);
}

static void print_list(void) {
    Serial.println("[LIST] taught stations:");
    print_pose("A", P_A);
    print_pose("B", P_B);
    Serial.printf("   grip %.0f   object at %c   -> %s\n", CLAW_GRIP, obj_at,
                  all_taught() ? "READY: type  run 10  (or KEY1 for one trip)" : "teach the missing ones");
}

static void print_help(void) {
    Serial.printf(
        "\n=== NexArm A<->B Accuracy & Repeatability ===\n"
        " TEACHING (once):\n"
        "   x+ 10 | x- 10 | y+ 10 | y- 10 | z+ 10 | z- 10   move the gripper (mm; number optional)\n"
        "   p+ 5  | p- 5      tilt the gripper (degrees, -90 = straight down)\n"
        "   r+ 10 | r- 10     turn the gripper around its own axis (degrees)\n"
        "   where             show the position\n"
        "   open | close      gripper        grip A   holding angle (now %.0f; -50 open .. 0 closed)\n"
        "   save A | save B   remember this position as station A / B\n"
        "   go A | go B       test a taught station (comes from above)\n"
        "   list              what is taught\n"
        " TEST:\n"
        "   at A | at B       tell the arm where the object is now\n"
        "   start  (KEY1)     one trip (object moves to the other station)\n"
        "   run N             N trips back and forth, then the results\n"
        "   stats | clear     show / erase the results\n"
        "   stop   (KEY2)     stop after the current motion\n"
        "   home              Hiwonder home pose\n", CLAW_GRIP);
}

static Pose* pose_by_name(const char* s) {
    if (!strcmp(s, "A") || !strcmp(s, "a")) return &P_A;
    if (!strcmp(s, "B") || !strcmp(s, "b")) return &P_B;
    return nullptr;
}
// =============================== 7b. PC LINK (Python FSM + HMI) ======================
static bool     pc_mode = false;      // the laptop is in control
static bool     fw_estop = false;     // latched E-stop: no motion until @reset
static uint16_t tel_period = 100;     // ms between TEL lines

static void send_telemetry(void) {
    const int age = last_servo_feedback_ms ? (int)(millis() - last_servo_feedback_ms) : 9999;
    Serial.printf("TEL,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d\n", (unsigned long)millis(),
                  arm.current_pose.x, arm.current_pose.y, arm.current_pose.z, arm.current_pose.pitch,
                  arm.current_pose.roll, arm.current_pose.claw, (int)last_servo_positions[5],
                  fw_estop ? 1 : 0, age);
}

static void pc_estop(const char* source) {
    fw_estop = true;
    Serial.printf("EVT,ESTOP,%s\n", source);
    screen("E-STOP (reset on PC)");
    arm.board.buzzer.set(100, 100, 3, 1500);
}

static void pc_command(char* line) {
    char cmd[12] = {0};
    sscanf(line, "%11s", cmd);
    if (!strcmp(cmd, "ping")) { Serial.println("PONG,nexarm-pclink,1"); }
    else if (!strcmp(cmd, "tel")) {
        int ms = 100;
        sscanf(line, "%*s %d", &ms);
        pc_mode = (ms > 0);
        if (pc_mode) { arm.set_torque(true); arm.set_move_acc(MOVE_ACC); }
        if (pc_mode) tel_period = (uint16_t)constrain(ms, 50, 1000);
        Serial.printf("ACK,tel,%d\n", pc_mode ? tel_period : 0);
        screen(pc_mode ? "PC control (HMI)" : "Ready");
    }
    else if (!strcmp(cmd, "mv")) {
        float x, y, z, p, r, c, t;
        if (fw_estop) { Serial.println("ERR,estop"); return; }
        if (sscanf(line, "%*s %f %f %f %f %f %f %f", &x, &y, &z, &p, &r, &c, &t) != 7) { Serial.println("ERR,mv args"); return; }
        x = constrain(x, arm.limit_x_min, arm.limit_x_max);
        y = constrain(y, arm.limit_y_min, arm.limit_y_max);
        z = constrain(z, arm.limit_z_min, arm.limit_z_max);
        arm.move(x, y, z, constrain(p, -90.0f, 0.0f), constrain(r, -90.0f, 90.0f),
                 constrain(c, -50.0f, 10.0f), (uint32_t)constrain(t, 20.0f, 5000.0f));   // returns at once
        jog = {x, y, z, p, r, true};
        roll_cmd = r; claw_now = c;
    }
    else if (!strcmp(cmd, "estop")) pc_estop("PC");
    else if (!strcmp(cmd, "reset")) { fw_estop = false; Serial.println("EVT,RESET"); screen("PC control (HMI)"); }
    else Serial.println("ERR,unknown");
}

static void handle_command(char* line) {
    if (line[0] == '@') { pc_command(line + 1); return; }   // machine commands from the Python HMI
    char cmd[16] = {0}, arg[16] = {0};
    float a = 0;
    int n = sscanf(line, "%15s %15s", cmd, arg);
    Serial.printf("> %s\n", line);

    // jog:  x+ 10, z- 5, p+ 5, r+ 10 ...  (default step 10 mm / 5 deg / 10 deg)
    if (strlen(cmd) == 2 && strchr("xyzpr", cmd[0]) && (cmd[1] == '+' || cmd[1] == '-')) {
        float step = (n == 2 && sscanf(arg, "%f", &a) == 1) ? fabsf(a) : (cmd[0] == 'p' ? 5.0f : 10.0f);   // r: 10 deg
        if (cmd[1] == '-') step = -step;
        Pose t = jog;
        if (cmd[0] == 'x') t.x += step; else if (cmd[0] == 'y') t.y += step;
        else if (cmd[0] == 'z') t.z += step;
        else if (cmd[0] == 'p') t.pitch = constrain(t.pitch + step, -90.0f, 0.0f);
        else t.roll = constrain(t.roll + step, -90.0f, 90.0f);
        roll_cmd = t.roll;
        const float dist = (cmd[0] == 'p' || cmd[0] == 'r') ? 2.0f * fabsf(step) : fabsf(step);   // bigger step = slower
        move_arm(t.x, t.y, t.z, t.pitch, claw_now, (uint16_t)fmaxf(T_JOG, 12.0f * dist));
        Serial.printf("   now x=%.0f y=%.0f z=%.0f pitch=%.0f roll=%.0f\n", jog.x, jog.y, jog.z, jog.pitch, jog.roll);
    }
    else if (!strcmp(cmd, "where")) {
        sync_arm_feedback(200);
        Serial.printf("   commanded x=%.0f y=%.0f z=%.0f pitch=%.0f | arm reports x=%.0f y=%.0f z=%.0f\n",
                      jog.x, jog.y, jog.z, jog.pitch, arm.current_pose.x, arm.current_pose.y, arm.current_pose.z);
    }
    else if (!strcmp(cmd, "save")) {
        Pose* p = pose_by_name(arg);
        if (!p) Serial.println("   save what?  A | B");
        else { *p = jog; p->ok = true; save_all(); print_pose(arg, *p); Serial.println("   (saved)"); screen("Teaching"); }
    }
    else if (!strcmp(cmd, "go")) {
        Pose* p = pose_by_name(arg);
        if (!p || !p->ok) Serial.println("   go where?  A | B  (must be taught)");
        else {                                      // come from above, like the task does
            roll_cmd = p->roll;
            move_arm(p->x, p->y, safe_z(p->z, jog.z - APPROACH), p->pitch, claw_now, T_MOVE);
            move_pose(*p, claw_now, T_SLOW);
        }
    }
    else if (!strcmp(cmd, "open"))  move_arm(jog.x, jog.y, jog.z, jog.pitch, CLAW_OPEN, T_CLAW);
    else if (!strcmp(cmd, "close")) move_arm(jog.x, jog.y, jog.z, jog.pitch, CLAW_GRIP, T_CLAW);
    else if (!strcmp(cmd, "grip") && n == 2 && sscanf(arg, "%f", &a) == 1) {
        CLAW_GRIP = constrain(a, -50.0f, 10.0f); save_all();
        Serial.printf("   grip = %.0f (saved) - type  close  to try it\n", CLAW_GRIP);
    }
    else if (!strcmp(cmd, "list"))  print_list();
    else if (!strcmp(cmd, "help"))  print_help();
    else if (!strcmp(cmd, "start")) run_trips(1);
    else if (!strcmp(cmd, "run"))   { int k = (n == 2) ? atoi(arg) : 1; run_trips(k < 1 ? 1 : (k > 50 ? 50 : k)); }
    else if (!strcmp(cmd, "at") && pose_by_name(arg)) {
        obj_at = (pose_by_name(arg) == &P_A) ? 'A' : 'B';
        Serial.printf("   object is at %c\n", obj_at); screen("Ready");
    }
    else if (!strcmp(cmd, "stats")) print_stats();
    else if (!strcmp(cmd, "clear")) { n_samples[0] = n_samples[1] = 0; trips_done = 0; Serial.println("   results erased"); screen("Ready"); }
    else if (!strcmp(cmd, "home"))  { arm.reset_all(2000); wait_ms(2200); sync_arm_feedback(200);
                                      jog = {arm.current_pose.x, arm.current_pose.y, arm.current_pose.z, arm.current_pose.pitch, 0.0f, true}; roll_cmd = 0.0f; }
    else if (!strcmp(cmd, "stop") || !strcmp(cmd, "ok")) Serial.println("   nothing is running");
    else Serial.println("   unknown command - type  help");
    stop_req = false;
}

// =============================== 8. START-UP AND MAIN LOOP ===========================
void register_system_task(esp_event_loop_handle_t* event_loop) {
    (void)event_loop;
    setCpuFrequencyMhz(240);
    Serial.begin(115200);
    servo.begin(Serial1, 1000000, 16, 17);      // as in the official example
    arm.begin();
    arm.set_torque(true);
    delay(1000);
    arm.reset_all(1500);
    at32_protocol.begin();
    at32_protocol.register_success_callback(at32_packet_callback);
    wait_ms(1600);
    if (sync_arm_feedback(300)) {               // start jogging from where the arm really is
        jog = {arm.current_pose.x, arm.current_pose.y, arm.current_pose.z, arm.current_pose.pitch, 0.0f, true};
        roll_cmd = 0.0f;
    }

    load_all();
    screen(all_taught() ? "Ready" : "Teach: type help");
    arm.board.button.update();
    arm.board.button.is_clicked(0);             // clear clicks from power-up
    arm.board.button.is_clicked(1);
    print_help();
    print_list();
}

void system_loop_handler(void) {
    static uint32_t last_sync = 0;
    pump_at32_feedback();
    arm.board.button.update();
    arm.board.buzzer.update();
    static uint32_t last_tel = 0;
    const uint32_t sync_period = pc_mode ? 80 : 500;     // PC mode: fresh position ~12 times/s
    if (millis() - last_sync >= sync_period) { last_sync = millis(); arm.update_status(); arm.board.bat.update(); }
    read_serial();
    if (pc_mode) {                                        // the laptop runs the FSM
        if (millis() - last_tel >= tel_period) { last_tel = millis(); send_telemetry(); }
        if (arm.board.button.is_clicked(0)) Serial.println("EVT,KEY1");
        if (arm.board.button.is_clicked(1)) pc_estop("KEY2");   // hardware E-stop, handled locally
        delay(2);
        return;
    }
    if (arm.board.button.is_clicked(0)) run_trips(1);    // KEY1 = one trip
    arm.board.button.is_clicked(1);                       // KEY2 when idle: nothing to stop
    delay(5);
}
