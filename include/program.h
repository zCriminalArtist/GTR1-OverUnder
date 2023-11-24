#ifndef PROGRAM_H
#define PROGRAM_H

#include "vex.h"
#include "drivetrain/odometry.h"
#include <functional>

class autonomousProgram {
  public:
    autonomousProgram(int quadrants[], Vec start_pos, std::function<void()> callback) : routine(callback), pos(start_pos) { 
      for (int i = 0; i < sizeof(quads)/sizeof(quads[0]); i++) quads[i] = quadrants[i];
    }
    const void run() const { routine(); }
    const Vec getStartingPosition() { return pos; }
    int* getQuadrant() { return quads; }
    void chooseQuadrant(unsigned int quad) { chosenQuadrant = quad; }
    unsigned int getChosenQuadrant() { return chosenQuadrant; }
  private:
    std::function<void()> routine;
    Vec pos;
    int quads[4];
    unsigned int chosenQuadrant;
};

#endif PROGRAM_H