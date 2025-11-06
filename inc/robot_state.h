#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

// ===================== ROBOT STATES =====================
enum State {
  INIT, IDLE, LINE_FOLLOW,
  AVOID_PREPARE, AVOID_PATH, MERGE_SEARCH,
  TURN_LEFT_PREPARE, SLOW_DOWN, STOP,
  LOST_LINE
};

#endif // ROBOT_STATE_H