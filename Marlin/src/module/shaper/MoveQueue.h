#pragma once

#include "../planner.h"
#include "TimeDouble.h"

#define MOVE_SIZE 64
#define MOVE_MOD(n) ((n + MOVE_SIZE)%MOVE_SIZE)

#define EMPTY_TIME 100

#define MOVE_FLAG_NORMAL 0
#define MOVE_FLAG_START 1
#define MOVE_FLAG_END 2

class Move {
  public:
    float start_v;
    float end_v;
    float t;
    time_double_t start_t = 0;
    time_double_t end_t = 0;
    float accelerate;

    float start_pos[AXIS_SIZE] = {0};
    float end_pos[AXIS_SIZE] = {0};
    float axis_r[4] = {0};

    float distance;

    uint8_t flag = 0;
};

class MoveQueue {
  public:
    Move moves[MOVE_SIZE];
    volatile uint8_t move_tail;
    volatile uint8_t move_head;

    bool is_start = true;

    constexpr uint8_t nextMoveIndex(const uint8_t block_index) { return MOVE_MOD(block_index + 1);};
    constexpr uint8_t prevMoveIndex(const uint8_t block_index) { return MOVE_MOD(block_index - 1);};
    bool isBetween(const uint8_t block_index) {
        return (MOVE_MOD(move_head - block_index) + MOVE_MOD(block_index - move_tail)) == MOVE_MOD(move_head - move_tail);
    };
    int getMoveSize() {
        return MOVE_MOD(move_head - move_tail);
    };

    void calculateMoves(block_t &block);

    uint8_t addEmptyMove(float time);
    uint8_t addMoveStart();
    uint8_t addMoveEnd();
    void setMoveEnd();

    uint8_t calculateMoveStart(uint8_t index, float delta_window);
    uint8_t calculateMoveEnd(uint8_t index, float delta_window);

    void updateMoveTail(uint8_t index);

    void initMoveTimeAndPos(uint8_t move_shaped_start, uint8_t move_start, uint8_t move_shaped_end);

    float getAxisPositionAcrossMoves(int move_index,int axis, time_double_t time, int move_shaped_start, int move_shaped_end);
    float getAxisPosition(int move_index,int axis, time_double_t time);

    void setMove(uint8_t move_index, float start_v, float end_v, float accelerate, float distance, xyze_float_t axis_r, float t, uint8_t flag = 0);
    uint8_t addMove(float start_v, float end_v, float accelerate, float distance, xyze_float_t axis_r, float t, uint8_t flag = MOVE_FLAG_NORMAL);

  private:
};

extern MoveQueue moveQueue;