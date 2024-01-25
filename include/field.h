#include "autonomous.h"
#include "drivetrain/motionPath.h"
#include <algorithm>
#include "vex.h"

using namespace vex;

class robot;

struct field {
  class Visual {
    protected:
      unsigned int x, y, width, height;
    public:
      Visual(unsigned int _x = 0, unsigned int _y = 0, unsigned int _width = 0, unsigned int _height = 0) : x(_x), y(_y), width(_width), height(_height) {}
      const unsigned int& getX() const { return x; }
      const unsigned int& getY() const { return y; }
      const unsigned int& getWidth() const{ return width; }
      const unsigned int& getHeight() const{ return height; }
      const unsigned int& getLength() const { return width; }
  };

  class button : public Visual {
    protected:
      bool isTouched;
    public:
      button(unsigned int x, unsigned int y, unsigned int width, unsigned int height) : Visual(x, y, width, height) {}
  };

  class buttonVisual : public button {
    public:
      buttonVisual(unsigned int x, unsigned int y, unsigned int width, unsigned int height, const char* _text) : button(x, y, width, height), text(_text) {}
      void render();
      void render_button();
      void render_text();
      const bool isTouched();
    private:
      const char* text;
  };

  class triangularButtonVisual : public button {
    public:
      triangularButtonVisual(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned int& _orientation) : button(x, y, width, height), orientation(_orientation) {}
      triangularButtonVisual(Visual& _button, unsigned int _orientation) : button(_button.getX(), _button.getY(), _button.getWidth(), _button.getHeight()), orientation(_orientation) {}
      void render();
      const unsigned int& getOrientation() { return orientation; }
      const bool isTouched();
    private:
      unsigned int orientation;
  };

  class RectangularButtonVisual : public button {
    public:
      RectangularButtonVisual(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned int& _orientation) : button(x, y, width, height), orientation(_orientation) {}
      RectangularButtonVisual(Visual& _button, unsigned int _orientation) : button(_button.getX(), _button.getY(), _button.getWidth(), _button.getHeight()), orientation(_orientation) {}
      void render();
      const unsigned int& getOrientation() { return orientation; }
      const bool isTouched();
    private:
      unsigned int orientation;
  };

  class RobotVisual : public Visual {
    public:
      RobotVisual(unsigned int _size) : size(_size) {}
      void render(unsigned int x, unsigned int y, double angle);
    private:
      unsigned int size;
      unsigned int prev_x;
      unsigned int prev_y;
      double angle;
      double prev_angle;
  };

  class FieldVisual : public Visual {
    public:
      FieldVisual(unsigned int x, unsigned int y, unsigned int length, int _selectedQuadrant) : Visual(x, y, length, length), selectedQuadrant(_selectedQuadrant) {}
      const unsigned int& getSelectedQuadrant() const { return selectedQuadrant; }
      void selectQuadrant(const unsigned int& i) { selectedQuadrant = i; }
      const double* FieldCentricCoordtoAbs(double _x, double _y) const;
      const double FieldCentricHeadingtoAbs(double heading) const;
      void render();
      void drawGameObject(int x, int y, int sideLength, int rotat);
      void render_MotionPath(MotionPath& path, const Vec& starting_pos) {
        for (unsigned int i = 0; i < path.newPath.size() - 1; i++) {
          std::cout << path.newPath[i].velocity << " " << path.newPath[i].acceleration << std::endl;
          Brain.Screen.drawLine(FieldCentricCoordtoAbs(starting_pos.x + path.newPath[i].x, starting_pos.y + path.newPath[i].y)[0], FieldCentricCoordtoAbs(starting_pos.x + path.newPath[i].x, starting_pos.y + path.newPath[i].y)[1], FieldCentricCoordtoAbs(starting_pos.x + path.newPath[i+1].x, starting_pos.y + path.newPath[i+1].y)[0], FieldCentricCoordtoAbs(starting_pos.x + path.newPath[i+1].x, starting_pos.y + path.newPath[i+1].y)[1]);
        }
      }
      void render_MotionPathPts(MotionPath& path, const Vec& starting_pos) {
        for (unsigned int i = 0; i < path.path.size(); i++) Brain.Screen.drawCircle(FieldCentricCoordtoAbs(starting_pos.x + path.path[i].x, starting_pos.y + path.path[i].y)[0], FieldCentricCoordtoAbs(starting_pos.x + path.path[i].x, starting_pos.y + path.path[i].y)[1], 1);
      }
      void render_AutonomousRoutine(autonomous& Autonomous) {
        Autonomous.Robot.autonomousControl.disable();
        Autonomous.run();
        for (MotionPath path : Autonomous.Robot.Drive.getMotionPaths()) {
          Brain.Screen.setPenColor(path.isReversed ? red : orange);
          render_MotionPath(path, Autonomous.getSelectedAutonomous().getStartingPosition());
          Brain.Screen.setPenColor(ClrAquamarine);
          render_MotionPathPts(path, Autonomous.getSelectedAutonomous().getStartingPosition());
        }
        Autonomous.Robot.autonomousControl.enable();
        std::cout << "EXIT" << std::endl;
        Autonomous.Robot.Drive.getMotionPaths().clear();
      }
      RectangularButtonVisual quadrant1 = RectangularButtonVisual(*this, 1);
      RectangularButtonVisual quadrant2 = RectangularButtonVisual(*this, 2);
      RectangularButtonVisual quadrant3 = RectangularButtonVisual(*this, 3);
      RectangularButtonVisual quadrant4 = RectangularButtonVisual(*this, 4);
      RobotVisual robot = RobotVisual(15);
    private:
      unsigned int selectedQuadrant;
      vex::color redAlliance = vex::color(204, 71, 71);
      vex::color blueAlliance = vex::color(30, 128, 214);
  };

  field() {}
  field(unsigned int x, unsigned int y, unsigned int length, int _selectedQuadrant, vex::competition& _c, autonomous& _autonomous) 
    : fieldElement(new FieldVisual(x, y, length, _selectedQuadrant)), c(&_c), Autonomous(&_autonomous) { 
      render(); 
      Autonomous->getSelectedAutonomous().chooseQuadrant(_selectedQuadrant);
    }
  void render();
  autonomousProgram& getAutonomous() { return *Autonomous->selectedAutonomous; };
  void chooseQuadrantPrompt();
  void chooseAutonomousPrompt();
  void updateRobotPositionOnField();
  void autoChooseAutonomous();
  
  private:
    FieldVisual* fieldElement;
    buttonVisual routine1 = buttonVisual(265, 20, 50, 50, "1");
    buttonVisual routine2 = buttonVisual(265 + routine1.getWidth() + 20, 20, 50, 50, "2");
    buttonVisual routine3 = buttonVisual(routine2.getX() + routine2.getWidth() + 20, 20, 50, 50, "3");
    buttonVisual routine4 = buttonVisual(265, 20 + routine1.getHeight() + 20, 50, 50, "4");
    buttonVisual routine5 = buttonVisual(265 + routine4.getWidth() + 20, routine4.getY(), 50, 50, "5");
    buttonVisual routine6 = buttonVisual(routine5.getX() + routine5.getWidth() + 20, routine4.getY(), 50, 50, "6");
    buttonVisual skills = buttonVisual(265 + routine4.getWidth() + 20, routine4.getY() + routine4.getHeight() + 20, routine4.getWidth()*2 + 20*1, 50, "Skills");
    vex::competition* c;
    autonomous* Autonomous;
};

const double* field::FieldVisual::FieldCentricCoordtoAbs(double _x, double _y) const {
  if (getSelectedQuadrant() == 1) return (new double[2]{ getX() + getLength() - _x*getLength()/144, getY() + _y*getLength()/144 }); 
  if (getSelectedQuadrant() == 2) return (new double[2]{ getX() - _x*getLength()/144, getY() + _y*getLength()/144 }); 
  if (getSelectedQuadrant() == 3) return (new double[2]{ getX() + _x*getLength()/144, getY() + getLength() - _y*getLength()/144 });
  if (getSelectedQuadrant() == 4) return (new double[2]{ getX() + getLength() + _x*getLength()/144, getY() + getLength() - _y*getLength()/144 });  

  // if (getSelectedQuadrant() == 1) return (new double[2]{ getX() + getLength() - _y*getLength()/144, getY() + _x*getLength()/144 }); 
  // if (getSelectedQuadrant() == 2) return (new double[2]{ getX() + _y*getLength()/144, getY() + _x*getLength()/144 }); 
  // if (getSelectedQuadrant() == 3) return (new double[2]{ getX() + _y*getLength()/144, getY() + getLength()/2 + _x*getLength()/144 }); 
  // if (getSelectedQuadrant() == 4) return (new double[2]{ getX() + getLength() - _y*getLength()/144, getY() + getLength()/2 + _x*getLength()/144 }); 
  return (new double[2]{_x, _y});
}

const double field::FieldVisual::FieldCentricHeadingtoAbs(double heading) const {
  if (getSelectedQuadrant() == 1) return (-PI/2 - heading); 
  if (getSelectedQuadrant() == 2) return (-PI/2 - heading); 
  if (getSelectedQuadrant() == 3) return (PI/2 - heading); 
  if (getSelectedQuadrant() == 4) return (PI/2 - heading); 
  return heading;
}

void field::updateRobotPositionOnField() {
  while (!Autonomous->Robot.isRunningSlpashscreen()) {
    if (!Autonomous->Robot.Odometer.isCalibrating()) {
      double x = Autonomous->selectedAutonomous->getStartingPosition().x + Autonomous->Robot.Odometer.getPosition().x;
      double y = Autonomous->selectedAutonomous->getStartingPosition().y + Autonomous->Robot.Odometer.getPosition().y;
      double h = Autonomous->Robot.Odometer.getPosition().h;
      
      fieldElement->robot.render(fieldElement->FieldCentricCoordtoAbs(x, y + 2)[0], fieldElement->FieldCentricCoordtoAbs(x, y + 2)[1], fieldElement->FieldCentricHeadingtoAbs(h));
    }
  }
}

void field::RobotVisual::render(unsigned int x, unsigned int y, double angle) {
  if (prev_x != 0 && prev_y != 0) {
    Brain.Screen.setPenWidth(2);
    // Brain.Screen.setPenColor(vex::color(77, 77, 77));
    Brain.Screen.setPenColor(vex::color(130, 130, 130));
    Brain.Screen.drawLine(prev_x, prev_y, prev_x + size*cos(prev_angle), (prev_y) - size*sin(prev_angle));
    Brain.Screen.drawLine(prev_x, prev_y, prev_x + (size/2)*cos(prev_angle - PI/2), prev_y - (size/2)*sin(prev_angle - PI/2));
    Brain.Screen.drawLine(prev_x + (size/2)*cos(prev_angle - PI/2), (prev_y) - (size/2)*sin(prev_angle - PI/2), prev_x + size*cos(prev_angle), prev_y - size*sin(prev_angle));
    Brain.Screen.drawLine(prev_x, prev_y, prev_x + (size/2)*cos(prev_angle + PI/2), prev_y - (size/2)*sin(prev_angle + PI/2));
    Brain.Screen.drawLine(prev_x + (size/2)*cos(prev_angle + PI/2), prev_y - (size/2)*sin(prev_angle + PI/2), prev_x + size*cos(prev_angle), prev_y - size*sin(prev_angle));
  }
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(x, y, x + size*cos(angle), y - size*sin(angle));
  Brain.Screen.drawLine(x, y, x + (size/2)*cos(angle - PI/2), y - (size/2)*sin(angle - PI/2));
  Brain.Screen.drawLine(x + (size/2)*cos(angle - PI/2), y - (size/2)*sin(angle - PI/2), x + size*cos(angle), y - size*sin(angle));
  Brain.Screen.drawLine(x, y, x + (size/2)*cos(angle + PI/2), y - (size/2)*sin(angle + PI/2));
  Brain.Screen.drawLine(x + (size/2)*cos(angle + PI/2), y - (size/2)*sin(angle + PI/2), x + size*cos(angle), y - size*sin(angle));
  prev_x = x;
  prev_y = y;
  prev_angle = angle;
  vex::task::sleep(100);
}

const bool field::buttonVisual::isTouched() {
  int X = Brain.Screen.xPosition();
  int Y = Brain.Screen.yPosition();
  return (getX() <= X) && (X <= (getX() + getWidth())) && (getY() <= Y) && (Y <= (getY() + getHeight()));
}

void field::buttonVisual::render() {
  render_button();
  render_text();
}

void field::buttonVisual::render_button() {
  Brain.Screen.drawRectangle(getX(), getY(), getWidth(), getHeight());
}

void field::buttonVisual::render_text() {
  Brain.Screen.printAt(getX() + getWidth()/2 - 5*strlen(text), getY() + getHeight()/2 + 5, text);
}

const bool field::triangularButtonVisual::isTouched() {
  int X = Brain.Screen.xPosition();
  int Y = Brain.Screen.yPosition();
  if (getOrientation() == 1) {
    return ((x <= X)) && (X <= (x + getLength()/3)) && ((y + getLength()/3) <= Y) && (Y <= (y + 2*getLength()/3));
  } else if (getOrientation() == 2) {
    return ((x + getLength()/3) <= X) && (X <= (x + 2*getLength()/3)) && ((y) <= Y) && (Y <= (y + getLength()/3));
  } else if (getOrientation() == 3) {
    return ((x + 2*getLength()/3) <= X) && (X <= (x + getLength())) && ((y + getLength()/3) <= Y) && (Y <= (y + 2*getLength()/3));
  } else if (getOrientation() == 4) {
    return ((x + getLength()/3) <= X) && (X <= (x + 2*getLength()/3)) && ((y + 2*getLength()/3) <= Y) && (Y <= (y + getLength()));
  }
  return false;
}

const bool field::RectangularButtonVisual::isTouched() {
  int X = Brain.Screen.xPosition();
  int Y = Brain.Screen.yPosition();
  if (getOrientation() == 1) {
    return ((x + getLength()/2) <= X) && (X <= (x + getLength())) && ((y) <= Y) && (Y <= (y + getLength()/2));
  } else if (getOrientation() == 2) {
    return (x <= X) && (X <= (x + getLength()/2)) && ((y) <= Y) && (Y <= (y + getLength()/2));
  } else if (getOrientation() == 3) {
    return ((x) <= X) && (X <= (x + getLength()/2)) && ((y + getLength()/2) <= Y) && (Y <= (getLength()));
  } else if (getOrientation() == 4) {
    return ((x + getLength()/2) <= X) && (X <= (getLength())) && ((y + getLength()/2) <= Y) && (Y <= (getLength()));
  }
  return false;
}

void field::autoChooseAutonomous() {
  if (std::find(getAutonomous().getQuadrant(), getAutonomous().getQuadrant() + 3, fieldElement->getSelectedQuadrant()) != getAutonomous().getQuadrant() + 3) return;
  if (std::find(Autonomous->routine1->getQuadrant(), Autonomous->routine1->getQuadrant() + 3, fieldElement->getSelectedQuadrant()) != Autonomous->routine1->getQuadrant() + 3) Autonomous->selectAutonomous(Autonomous->routine1);
  if (std::find(Autonomous->routine2->getQuadrant(), Autonomous->routine2->getQuadrant() + 3, fieldElement->getSelectedQuadrant()) != Autonomous->routine2->getQuadrant() + 3) Autonomous->selectAutonomous(Autonomous->routine2);
  if (std::find(Autonomous->routine3->getQuadrant(), Autonomous->routine3->getQuadrant() + 3, fieldElement->getSelectedQuadrant()) != Autonomous->routine3->getQuadrant() + 3) Autonomous->selectAutonomous(Autonomous->routine3);
  if (std::find(Autonomous->routine4->getQuadrant(), Autonomous->routine4->getQuadrant() + 3, fieldElement->getSelectedQuadrant()) != Autonomous->routine4->getQuadrant() + 3) Autonomous->selectAutonomous(Autonomous->routine4);
  if (std::find(Autonomous->routine5->getQuadrant(), Autonomous->routine5->getQuadrant() + 3, fieldElement->getSelectedQuadrant()) != Autonomous->routine5->getQuadrant() + 3) Autonomous->selectAutonomous(Autonomous->routine5);
  if (std::find(Autonomous->routine6->getQuadrant(), Autonomous->routine6->getQuadrant() + 3, fieldElement->getSelectedQuadrant()) != Autonomous->routine6->getQuadrant() + 3) Autonomous->selectAutonomous(Autonomous->routine6);
  if (std::find(Autonomous->skills->getQuadrant(), Autonomous->skills->getQuadrant() + 3, fieldElement->getSelectedQuadrant()) != Autonomous->skills->getQuadrant() + 3) Autonomous->selectAutonomous(Autonomous->skills);
}

void field::chooseAutonomousPrompt() {
  while (!c->isAutonomous()) {
    if (Brain.Screen.pressing()) {
      if (Autonomous->selectedAutonomous != Autonomous->routine1 && routine1.isTouched()) {
        Autonomous->selectAutonomous(Autonomous->routine1);
        render();
        continue;
      }
      if (Autonomous->selectedAutonomous != Autonomous->routine2 && routine2.isTouched()) {
        Autonomous->selectAutonomous(Autonomous->routine2);
        render();
        continue;
      }
      if (Autonomous->selectedAutonomous != Autonomous->routine3 && routine3.isTouched()) {
        Autonomous->selectAutonomous(Autonomous->routine3);
        render();
        continue;
      }
      if (Autonomous->selectedAutonomous != Autonomous->routine4 && routine4.isTouched()) {
        Autonomous->selectAutonomous(Autonomous->routine4);
        render();
        continue;
      }
      if (Autonomous->selectedAutonomous != Autonomous->routine5 && routine5.isTouched()) {
        Autonomous->selectAutonomous(Autonomous->routine5);
        render();
        continue;
      }
      if (Autonomous->selectedAutonomous != Autonomous->routine6 && routine6.isTouched()) {
        Autonomous->selectAutonomous(Autonomous->routine6);
        render();
        continue;
      }
      if (Autonomous->selectedAutonomous != Autonomous->skills && skills.isTouched()) {
        Autonomous->selectAutonomous(Autonomous->skills);
        render();
        continue;
      }
    }
  }
}

void field::chooseQuadrantPrompt() {
  while (!c->isAutonomous()) {
    if (Brain.Screen.pressing()) {
      if (fieldElement->getSelectedQuadrant() != 1 && fieldElement->quadrant1.isTouched()) {
          fieldElement->selectQuadrant(1);
          Autonomous->getSelectedAutonomous().chooseQuadrant(1);
          autoChooseAutonomous();
          render();
          continue;
      }
      if (fieldElement->getSelectedQuadrant() != 2 && fieldElement->quadrant2.isTouched()) {
          fieldElement->selectQuadrant(2);
          Autonomous->getSelectedAutonomous().chooseQuadrant(2);
          autoChooseAutonomous();
          render();
          continue;
      }
      if (fieldElement->getSelectedQuadrant() != 3 && fieldElement->quadrant3.isTouched()) {
          fieldElement->selectQuadrant(3);
          Autonomous->getSelectedAutonomous().chooseQuadrant(3);
          autoChooseAutonomous();
          render();
          continue;
      }
      if (fieldElement->getSelectedQuadrant() != 4 && fieldElement->quadrant4.isTouched()) {
          fieldElement->selectQuadrant(4);
          Autonomous->getSelectedAutonomous().chooseQuadrant(4);
          autoChooseAutonomous();
          render();
          continue;
      }
    }
  }
}

void field::triangularButtonVisual::render() {
  if (getOrientation() == 1) {
    for (int i = 0; i < getLength()/2; i++) {
      Brain.Screen.drawLine(x, y + i, x + i, y + i);
      Brain.Screen.drawLine(x, y + getLength()/2 + i, x + (getLength()/2-i), y + i + getLength()/2);
    }
  } else if (getOrientation() == 2) {
    for (int i = 0; i < getLength()/2; i++) {
      Brain.Screen.drawLine(x + i, y, x + i, y + i);
      Brain.Screen.drawLine(x + getLength()/2 + i, y, x + getLength()/2 + i, y - i + getLength()/2);
    }
  } else if (getOrientation() == 3) {
    for (int i = 0; i < getLength()/2; i++) {
      Brain.Screen.drawLine(x + getLength() - i, y + i, x + getLength(), y + i);
      Brain.Screen.drawLine(x + getLength() - getLength()/2 + i, y + getLength()/2 + i, x + getLength(), y + getLength()/2 + i);
    }
  } else if (getOrientation() == 4) {
    for (int i = 0; i < getLength()/2; i++) {
      Brain.Screen.drawLine(x - i, y + 1, x, y + i);
      Brain.Screen.drawLine(x + getLength()/2 - i, y + getLength()/2 + i, x + getLength()/2 + i, y + getLength()/2 + i);
    }
  }
}

void field::RectangularButtonVisual::render() {
  if (getOrientation() == 1) {
    Brain.Screen.drawRectangle(x + getLength()/2, y, getLength()/2, getLength()/2);
  } else if (getOrientation() == 2) {
    Brain.Screen.drawRectangle(x, y, getLength()/2, getLength()/2);
  } else if (getOrientation() == 3) {
    Brain.Screen.drawRectangle(x, y + getLength()/2, getLength()/2, getLength()/2);
  } else if (getOrientation() == 4) {
    Brain.Screen.drawRectangle(x + getLength()/2, y + getLength()/2, getLength()/2, getLength()/2);
  }
}

void field::render() {
  fieldElement->render();

  // for (MotionPath::Pt pt : Autonomous->path1.newPath) {
  //   std::cout << pt.curvature << " " << pt.velocity << std::endl;
  // }
  std::cout << "RENDER" << std::endl;
  // Autonomous->path1.getLookAheadPt(Autonomous->Robot.Odometer.getPosition());

  //Brain.Screen.drawCircle(fieldElement->FieldCentricCoordtoAbs(getAutonomous().getStartingPosition().x + Autonomous->path1.getClosestPt(Autonomous->Robot.Odometer.getPosition()).x, getAutonomous().getStartingPosition().y + Autonomous->path1.getClosestPt(Autonomous->Robot.Odometer.getPosition()).y)[0]
  //  , fieldElement->FieldCentricCoordtoAbs(getAutonomous().getStartingPosition().x + Autonomous->path1.getClosestPt(Autonomous->Robot.Odometer.getPosition()).x, getAutonomous().getStartingPosition().y + Autonomous->path1.getClosestPt(Autonomous->Robot.Odometer.getPosition()).y)[1], 2);
  //// std::cout << Autonomous->path1.getClosestPt(Autonomous->Robot.Odometer.getPosition()).x << " " << Autonomous->path1.getClosestPt(Autonomous->Robot.Odometer.getPosition()).y << std::endl;
  //Brain.Screen.setPenColor(cyan);
  //MotionPath::Pt lookAheadPt = Autonomous->path1.getLookAheadPt(Autonomous->Robot.Odometer.getPosition());
  //Brain.Screen.drawCircle(fieldElement->FieldCentricCoordtoAbs(getAutonomous().getStartingPosition().x + lookAheadPt.x, getAutonomous().getStartingPosition().y + lookAheadPt.y)[0]
  //  , fieldElement->FieldCentricCoordtoAbs(getAutonomous().getStartingPosition().x + lookAheadPt.x, getAutonomous().getStartingPosition().y + lookAheadPt.y)[1], 3);
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(std::find(Autonomous->routine1->getQuadrant(), Autonomous->routine1->getQuadrant() + 3, fieldElement->getSelectedQuadrant()) != Autonomous->routine1->getQuadrant() + 3 ? vex::color(150, 150, 150) : vex::color(45, 45, 45));
  routine1.render();
  Brain.Screen.setPenColor(std::find(Autonomous->routine2->getQuadrant(), Autonomous->routine2->getQuadrant() + 4, fieldElement->getSelectedQuadrant()) != Autonomous->routine2->getQuadrant() + 4 ? vex::color(150, 150, 150) : vex::color(45, 45, 45));
  routine2.render();
  Brain.Screen.setPenColor(std::find(Autonomous->routine3->getQuadrant(), Autonomous->routine3->getQuadrant() + 4, fieldElement->getSelectedQuadrant()) != Autonomous->routine3->getQuadrant() + 4 ? vex::color(150, 150, 150) : vex::color(45, 45, 45));
  routine3.render();

  Brain.Screen.setPenColor(std::find(Autonomous->routine4->getQuadrant(), Autonomous->routine4->getQuadrant() + 4, fieldElement->getSelectedQuadrant()) != Autonomous->routine4->getQuadrant() + 4 ? vex::color(150, 150, 150) : vex::color(45, 45, 45));
  routine4.render();
  Brain.Screen.setPenColor(std::find(Autonomous->routine5->getQuadrant(), Autonomous->routine5->getQuadrant() + 4, fieldElement->getSelectedQuadrant()) != Autonomous->routine5->getQuadrant() + 4 ? vex::color(150, 150, 150) : vex::color(45, 45, 45));
  routine5.render();
  Brain.Screen.setPenColor(std::find(Autonomous->routine6->getQuadrant(), Autonomous->routine6->getQuadrant() + 4, fieldElement->getSelectedQuadrant()) != Autonomous->routine6->getQuadrant() + 4 ? vex::color(150, 150, 150) : vex::color(45, 45, 45));
  routine6.render();

  Brain.Screen.setPenColor(std::find(Autonomous->skills->getQuadrant(), Autonomous->skills->getQuadrant() + 4, fieldElement->getSelectedQuadrant()) != Autonomous->skills->getQuadrant() + 4 ? vex::color(150, 150, 150) : vex::color(45, 45, 45));
  skills.render();
  
  if (fieldElement->getSelectedQuadrant() == 2 || fieldElement->getSelectedQuadrant() == 3) {
    Brain.Screen.setFillColor(vex::color(158, 114, 114));
    Brain.Screen.setPenColor(vex::color(158, 114, 114));
  } else if (fieldElement->getSelectedQuadrant() == 1 || fieldElement->getSelectedQuadrant() == 4) {
    Brain.Screen.setFillColor(vex::color(143, 159, 196));
    Brain.Screen.setPenColor(vex::color(143, 159, 196));
  } else {
    Brain.Screen.setFillColor(vex::color(130, 130, 130));
    Brain.Screen.setPenColor(vex::color(130, 130, 130));
  }
  if (Autonomous->selectedAutonomous == Autonomous->routine1) {
    routine1.render_button();
    Brain.Screen.setPenColor(white);
    Brain.Screen.setPenWidth(4);
    routine1.render_text();
  } else if (Autonomous->selectedAutonomous == Autonomous->routine2) {
    routine2.render_button();
    Brain.Screen.setPenColor(white);
    Brain.Screen.setPenWidth(4);
    routine2.render_text();
  } else if (Autonomous->selectedAutonomous == Autonomous->routine3) {
    routine3.render_button();
    Brain.Screen.setPenColor(white);
    Brain.Screen.setPenWidth(4);
    routine3.render_text();
  } else if (Autonomous->selectedAutonomous == Autonomous->routine4) {
    routine4.render_button();
    Brain.Screen.setPenColor(white);
    Brain.Screen.setPenWidth(4);
    routine4.render_text();
  } else if (Autonomous->selectedAutonomous == Autonomous->routine5) {
    routine5.render_button();
    Brain.Screen.setPenColor(white);
    Brain.Screen.setPenWidth(4);
    routine5.render_text();
  } else if (Autonomous->selectedAutonomous == Autonomous->routine6) {
    routine6.render_button();
    Brain.Screen.setPenColor(white);
    Brain.Screen.setPenWidth(4);
    routine6.render_text();
  } else if (Autonomous->selectedAutonomous == Autonomous->skills) {
    skills.render_button();
    Brain.Screen.setPenColor(white);
    Brain.Screen.setPenWidth(4);
    skills.render_text();
  }

  Brain.Screen.setPenWidth(3);
  Brain.Screen.setFillColor(transparent);
  fieldElement->render_AutonomousRoutine(*Autonomous);
}

void field::FieldVisual::drawGameObject(int x, int y, int sideLength, int rotat) {
  Brain.Screen.setPenColor(vex::color(57, 164, 36));
  Brain.Screen.setPenWidth(5);
  double rotationRadians = rotat * 3.14159265359 / 180.0;

  // Calculate the coordinates of the three vertices of the equilateral triangle
  double x1 = x + sideLength * cos(rotationRadians);
  double y1 = y + sideLength * sin(rotationRadians);

  double x2 = x + sideLength * cos(rotationRadians + 2.09439510239); // 2π/3 radians
  double y2 = y + sideLength * sin(rotationRadians + 2.09439510239);

  double x3 = x + sideLength * cos(rotationRadians - 2.09439510239);
  double y3 = y + sideLength * sin(rotationRadians - 2.09439510239);

  // Draw the triangle using lines
  Brain.Screen.drawLine(x1, y1, x2, y2);
  Brain.Screen.drawLine(x2, y2, x3, y3);
  Brain.Screen.drawLine(x3, y3, x1, y1);
}

void field::FieldVisual::render() {
  int r = getLength()*0.015;
  
  //Field
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(vex::color(77, 77, 77));
  Brain.Screen.drawRectangle(getX(), getY(), getLength(), getLength());

  //Quadrant Selection
  Brain.Screen.setFillColor(vex::color(130, 130, 130));
  Brain.Screen.setPenColor(vex::color(130, 130, 130)); 
  if (this->selectedQuadrant == 1) {
    quadrant1.render();
  } else if (this->selectedQuadrant == 2) {
    quadrant2.render();
  } else if (this->selectedQuadrant == 3) {
    quadrant3.render();
  } else if (this->selectedQuadrant == 4) {
    quadrant4.render();
  }
  //Draw lines
  Brain.Screen.setFillColor(vex::color(77, 77, 77));
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawLine(getX(), getY() + getLength()/2 - 2, getX() + getLength() + 1, getY() + getLength()/2 - 2);
  Brain.Screen.drawLine(getX(), getY() + getLength()/2 + 3, getX() + getLength() + 1, getY() + getLength()/2 + 3);

  Brain.Screen.drawLine(getX() + getLength()/2, getY() + getLength()/2 - getLength()/12, getX() + getLength() + 1, getY() + getLength()/2 - getLength()/12);
  Brain.Screen.drawLine(getX(), getY() + getLength()/2 + getLength()/12, getX() + getLength()/2, getY() + getLength()/2 + getLength()/12);

  Brain.Screen.setPenColor(redAlliance); //Net
  Brain.Screen.setPenWidth(1);
  // Vertical
  Brain.Screen.drawLine(x + getLength() - getLength()/6 + 10, y + getLength()/2 - getLength()/6 + 7, x + getLength() - getLength()/6 + 10, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength() - getLength()/6 + 10 + getLength()/48, y + getLength()/2 - getLength()/6 + 7, x + getLength() - getLength()/6 + 10 + getLength()/48, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength() - getLength()/6 + 10 + 2*getLength()/48, y + getLength()/2 - getLength()/6 + 7, x + getLength() - getLength()/6 + 10 + 2*getLength()/48, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength() - getLength()/6 + 10 + 3*getLength()/48, y + getLength()/2 - getLength()/6 + 7, x + getLength() - getLength()/6 + 10 + 3*getLength()/48, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength() - getLength()/6 + 10 + 4*getLength()/48, y + getLength()/2 - getLength()/6 + 7, x + getLength() - getLength()/6 + 10 + 4*getLength()/48, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength() - getLength()/6 + 10 + 5*getLength()/48, y + getLength()/2 - getLength()/6 + 7, x + getLength() - getLength()/6 + 10 + 5*getLength()/48, y + getLength()/2 + getLength()/6 - 7);
  // Horizontal
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15, x + getLength(), y + getLength()/2 - getLength()/6 + 15);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15 + getLength()/48, x + getLength(), y + getLength()/2 - getLength()/6 + 15 + getLength()/48);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 2*getLength()/48, x + getLength(), y + getLength()/2 - getLength()/6 + 15 + 2*getLength()/48);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 3*getLength()/48, x + getLength(), y + getLength()/2 - getLength()/6 + 15 + 3*getLength()/48);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 4*getLength()/48, x + getLength(), y + getLength()/2 - getLength()/6 + 15 + 4*getLength()/48);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 5*getLength()/48, x + getLength(), y + getLength()/2 - getLength()/6 + 15 + 5*getLength()/48);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 6*getLength()/48, x + getLength(), y + getLength()/2 - getLength()/6 + 15 + 6*getLength()/48);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 7*getLength()/48, x + getLength(), y + getLength()/2 - getLength()/6 + 15 + 7*getLength()/48);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 8*getLength()/48, x + getLength(), y + getLength()/2 - getLength()/6 + 15 + 8*getLength()/48);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 9*getLength()/48, x + getLength(), y + getLength()/2 - getLength()/6 + 15 + 9*getLength()/48);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 10*getLength()/48, x + getLength(), y + getLength()/2 - getLength()/6 + 15 + 10*getLength()/48);
  
  Brain.Screen.setPenColor(blueAlliance); //Net
  Brain.Screen.setPenWidth(1);
  // Vertical
  Brain.Screen.drawLine(x + getLength()/6 - 10, y + getLength()/2 - getLength()/6 + 7, x + getLength()/6 - 10, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength()/6 - 10 - getLength()/48, y + getLength()/2 - getLength()/6 + 7, x + getLength()/6 - 10 - getLength()/48, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength()/6 - 10 - 2*getLength()/48, y + getLength()/2 - getLength()/6 + 7, x + getLength()/6 - 10 - 2*getLength()/48, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength()/6 - 10 - 3*getLength()/48, y + getLength()/2 - getLength()/6 + 7, x + getLength()/6 - 10 - 3*getLength()/48, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength()/6 - 10 - 4*getLength()/48, y + getLength()/2 - getLength()/6 + 7, x + getLength()/6 - 10 - 4*getLength()/48, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength()/6 - 10 - 5*getLength()/48, y + getLength()/2 - getLength()/6 + 7, x + getLength()/6 - 10 - 5*getLength()/48, y + getLength()/2 + getLength()/6 - 7);
  // Horizontal
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15, x, y + getLength()/2 - getLength()/6 + 15);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15 + getLength()/48, x, y + getLength()/2 - getLength()/6 + 15 + getLength()/48);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 2*getLength()/48, x, y + getLength()/2 - getLength()/6 + 15 + 2*getLength()/48);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 3*getLength()/48, x, y + getLength()/2 - getLength()/6 + 15 + 3*getLength()/48);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 4*getLength()/48, x, y + getLength()/2 - getLength()/6 + 15 + 4*getLength()/48);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 5*getLength()/48, x, y + getLength()/2 - getLength()/6 + 15 + 5*getLength()/48);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 6*getLength()/48, x, y + getLength()/2 - getLength()/6 + 15 + 6*getLength()/48);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 7*getLength()/48, x, y + getLength()/2 - getLength()/6 + 15 + 7*getLength()/48);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 8*getLength()/48, x, y + getLength()/2 - getLength()/6 + 15 + 8*getLength()/48);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 9*getLength()/48, x, y + getLength()/2 - getLength()/6 + 15 + 9*getLength()/48);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 15 + 10*getLength()/48, x, y + getLength()/2 - getLength()/6 + 15 + 10*getLength()/48);


  Brain.Screen.setPenColor(vex::color(85,85,85));
  Brain.Screen.setPenWidth(2);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6 + 6, x + getLength(), y + getLength()/2 - getLength()/6 + 6);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 + getLength()/6 - 6, x + getLength(), y + getLength()/2 + getLength()/6 - 6);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6 + 6, x, y + getLength()/2 - getLength()/6 + 6);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 + getLength()/6 - 6, x, y + getLength()/2 + getLength()/6 - 6);
  Brain.Screen.setPenWidth(4);
  Brain.Screen.setPenColor(vex::color(60,60,60));
  Brain.Screen.drawLine(getX() + getLength()/2, getY(), getX() + getLength()/2, getY() + getLength());
  Brain.Screen.drawLine(getX() + getLength()/2 - getLength()/6, getY() + getLength()/6, getX() + getLength()/2 + getLength()/6, getY() + getLength()/6);
  Brain.Screen.drawLine(getX() + getLength()/2 - getLength()/6, getY() + getLength() - getLength()/6, getX() + getLength()/2 + getLength()/6, getY() + getLength() - getLength()/6);
  Brain.Screen.drawLine(x + getLength() - getLength()/6 + 10, y + getLength()/2 - getLength()/6 + 7, x + getLength() - 5, y + getLength()/2 - getLength()/6 + 7);
  Brain.Screen.drawLine(x + getLength() - getLength()/6 + 10, y + getLength()/2 + getLength()/6 - 7, x + getLength() - 5, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.drawLine(x + getLength()/6 - 10, y + getLength()/2 - getLength()/6 + 7, x + 5, y + getLength()/2 - getLength()/6 + 7);
  Brain.Screen.drawLine(x + getLength()/6 - 10, y + getLength()/2 + getLength()/6 - 7, x + 5, y + getLength()/2 + getLength()/6 - 7);
  Brain.Screen.setPenWidth(3);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6, x + getLength() - getLength()/6, y + getLength()/2 + getLength()/6);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6, x + getLength()/6, y + getLength()/2 + getLength()/6);

  Brain.Screen.setPenColor(redAlliance);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.setPenWidth(4);
  Brain.Screen.drawLine(x + getLength()/2, y + getLength() - getLength()/6 + 3, x + getLength()/2, y + getLength());
  Brain.Screen.setPenWidth(4);
  Brain.Screen.drawLine(x, y + getLength() - getLength()/6, x + getLength()/6, y + getLength());
  Brain.Screen.drawLine(x, y + getLength()/6, x + getLength()/6, y);
  Brain.Screen.setPenWidth(2);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6, x + getLength(), y + getLength()/2 - getLength()/6);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength()/2 + getLength()/6, x + getLength(), y + getLength()/2 + getLength()/6);
  Brain.Screen.drawCircle(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6, r*3);
  Brain.Screen.drawCircle(x + getLength() - getLength()/6, y + getLength()/2 + getLength()/6, r*3);
  Brain.Screen.setPenWidth(4);
  Brain.Screen.drawCircle(x + getLength() - getLength()/6, y + getLength()/2 - getLength()/6, r*1.5);
  Brain.Screen.drawCircle(x + getLength() - getLength()/6, y + getLength()/2 + getLength()/6, r*1.5);
  Brain.Screen.setPenColor(blueAlliance);
  Brain.Screen.setPenWidth(4);
  Brain.Screen.drawLine(x + getLength()/2, y, x + getLength()/2, y + getLength()/6 - 3);
  Brain.Screen.setPenWidth(4);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y, x + getLength(), y + getLength()/6);
  Brain.Screen.drawLine(x + getLength() - getLength()/6, y + getLength(), x + getLength(), y + getLength() - getLength()/6);
  Brain.Screen.setPenWidth(2);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 - getLength()/6, x, y + getLength()/2 - getLength()/6);
  Brain.Screen.drawLine(x + getLength()/6, y + getLength()/2 + getLength()/6, x, y + getLength()/2 + getLength()/6);
  Brain.Screen.drawCircle(x + getLength()/6, y + getLength()/2 - getLength()/6, r*3);
  Brain.Screen.drawCircle(x + getLength()/6, y + getLength()/2 + getLength()/6, r*3);
  Brain.Screen.setPenWidth(4);
  Brain.Screen.drawCircle(x + getLength()/6, y + getLength()/2 - getLength()/6, r*1.5);
  Brain.Screen.drawCircle(x + getLength()/6, y + getLength()/2 + getLength()/6, r*1.5);
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.drawRectangle(x, y, getLength(), getLength());

  drawGameObject(x + getLength()/2 - 10, y + getLength()/2 + 2, 4, 270);
  drawGameObject(x + getLength()/2 - 10, y + getLength()/2 - getLength()/6, 4, 90);
  drawGameObject(x + getLength()/2 - getLength()/6, y + getLength()/2, 4, 90);
  drawGameObject(x + 10, y + 10, 4, 90);
  drawGameObject(x + 10, y + getLength() - 10, 4, 180);

  drawGameObject(x + getLength()/2 + 10, y + getLength()/2 - 1, 4, 90);
  drawGameObject(x + getLength()/2 + 10, y + getLength()/2 + getLength()/6, 4, 270);
  drawGameObject(x + getLength()/2 + getLength()/6, y + getLength()/2, 4, 270);
  drawGameObject(x + getLength() - 10, y + 10, 4, 180);
  drawGameObject(x + getLength() - 10, y + getLength() - 10, 4, 270);
}