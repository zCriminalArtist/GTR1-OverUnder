#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "vex.h"
#include "odometry.h"
#include <iostream>
#include <vector>
#include <cmath>

struct MotionPath {
  struct Vector {
    double length;
    double angle;

    Vector(double mag, double dir) : length(mag), angle(dir) {}
    const double magnitude() const { return length; }
    const double direction() const { return angle; }
    Vector normalize() { return Vector(1, this->angle); }
    friend Vector operator* (const Vector& lhs, double rhs) {
      return Vector(lhs.length * rhs, lhs.angle);
    }
    float dot(const Vector& rhs) {
      return (magnitude()*rhs.magnitude()*cos(rhs.direction() - direction()));
    }
  };

  struct Pt {
    double x;
    double y;
    double distance = 0.0;
    double curvature = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    Pt(double _x, double _y) : x(_x), y(_y) {}
    friend Vector operator-(Pt& lhs, const Pt& rhs) {
      return Vector(sqrt(pow((lhs.x - rhs.x), 2) + pow((lhs.y - rhs.y), 2)), atan2((lhs.y - rhs.y), (lhs.x - rhs.x)));
    }
    Pt operator*(const double& scalar) const {
      return {x * scalar, y * scalar};
    }
    double operator*(const Pt& rhs) const {
      return x * rhs.x + y * rhs.y;
    }
    Pt operator+(const Pt& rhs) const {
    return {x + rhs.x, y + rhs.y};
    }
    double distTo(const Pt& rhs) { return hypot((rhs.x - x), (rhs.y - y)); }
  };

  std::vector<Pt> path;
  std::vector<Pt> newPath;
  bool isReversed = false;
  unsigned int spacing = 3;
  unsigned int lastClosestPtIndex = 0;
  unsigned int prevLookAheadIndex = 0;
  double prevLookAheadTValue = 0;
  int viewDistance = 15;
  double maxAcceleration = 80.0;
  double maxDeceleration = 80.0;
  double maxVelocity = 6.0;
  double curveVelocity = 0.3;

  double smoothWeight = 0.8;
  double tolerance = 0.001;

  MotionPath() {}
  MotionPath(std::vector<Pt> pts) : path(pts) {}
  MotionPath& smooth();
  MotionPath& reverse();
  MotionPath& trajectory(const double acceleration, const double speed, const double deceleration, const double angularVelocity);
  MotionPath& setCurveVelocity(const double speed);
  void inject();
  void findCurvature();
  void calculatePtVelocity();
  Pt& getClosestPt(const Vec& position);
  Pt getLookAheadPt(const Vec& position);
  double getTValue(Pt& start, Pt& end, const Vec& position);
  double calculateCurvature(const Vec& position, const Pt& lookAheadPt);
  std::pair<double, double> calculateVelocity(const double& curvature, const Pt& closestPt);
  std::pair<double, double> calculateAcceleration(const double& curvature, const Pt& closestPt);
};

MotionPath& MotionPath::trajectory(const double acceleration, const double speed, double deceleration, const double angularVelocity) {
  maxVelocity = speed;
  maxAcceleration = acceleration;
  maxDeceleration = fabs(deceleration);
  curveVelocity = angularVelocity;
  smooth();
  return *this;
}

MotionPath& MotionPath::setCurveVelocity(const double speed) {
  curveVelocity = speed;
  return *this;
}

MotionPath& MotionPath::reverse() {
  isReversed = true;
  return *this;
}

void MotionPath::inject() {
  for (unsigned int i = 0; i < path.size() - 1; i++) {
    Pt start_point = path[i];
    Pt end_point = path[i+1];
    Vector v = Vector(end_point - start_point);
    unsigned int num_points_that_fit = ceil(v.magnitude() / spacing);
    v = v.normalize() * spacing;
    for (unsigned int j = 0; j < num_points_that_fit; j++) {
      newPath.push_back(Pt(start_point.x + v.magnitude() * cos(v.direction()) * j, start_point.y + v.magnitude() * sin(v.direction()) * j));
    }
  }
}

MotionPath& MotionPath::smooth() { //How many neighbours to smooth
  inject();

  std::vector<Pt> tmp = newPath;
  double change = tolerance;

  double a = 1 - smoothWeight, b = smoothWeight;

  while (change >= tolerance){
    change = 0.0;
    for (int i = 1; i < tmp.size() - 1; i++){
      Pt aux = newPath[i];

      newPath[i].x += a * (tmp[i].x - newPath[i].x) + b * (newPath[i-1].x + newPath[i+1].x - (2.0 * newPath[i].x));
      newPath[i].y += a * (tmp[i].y - newPath[i].y) + b * (newPath[i-1].y + newPath[i+1].y - (2.0 * newPath[i].y));

      change += fabs(aux.x + aux.y - newPath[i].x - newPath[i].y);
    }   
  }

  calculatePtVelocity();
  return *this;
}

void MotionPath::findCurvature() {
  for (unsigned int i = 1; i < newPath.size(); i++) {
    newPath[i].distance = newPath[i-1].distance + hypot((newPath[i].x - newPath[i-1].x), (newPath[i].y - newPath[i-1].y));
    if (i < newPath.size() - 1) {
      Pt A = newPath[i-1];
      Pt B = newPath[i];
      Pt C = newPath[i+1];

      double a = B.distTo(C);
      double b = A.distTo(C);
      double c = A.distTo(B);

      auto a2 = a * a, b2 = b * b, c2 = c * c;

      Pt pa = A * (a2 * (b2 + c2 - a2) / ((b+c)*(b+c)-a2) / (a2-(b-c)*(b-c)));
      Pt pb = B * (b2 * (a2 + c2 - b2) / ((a+c)*(a+c)-b2) / (b2-(a-c)*(a-c)));
      Pt pc = C * (c2 * (a2 + b2 - c2) / ((a+b)*(a+b)-c2) / (c2-(a-b)*(a-b)));

      Pt center = pa + pb + pc;

      double radius = center.distTo(A);
      newPath[i].curvature = 1/radius;
    }
  }
}

void MotionPath::calculatePtVelocity() {
  findCurvature();
  newPath[newPath.size()-1].velocity = 0.0;
  for (int i = 1; i < newPath.size() - 1; i++) {
    std::cout << "Max: " << maxVelocity << " " << curveVelocity / newPath[i].curvature << std::endl;
    newPath[i].velocity = (fmin(maxVelocity, curveVelocity / newPath[i].curvature));
  }
  newPath[newPath.size()-1].velocity = 0.0;
  for (int i = newPath.size() - 1; i >= 0; i--) { // Acceleration < 0
    newPath[i].velocity = fmin(newPath[i].velocity, sqrt(newPath[i+1].velocity * newPath[i+1].velocity + 2.0 * maxDeceleration * newPath[i].distTo(newPath[i+1])));
  }
  for (int i = 0; i < newPath.size() - 1; i++) { // Acceleration > 0
    double dist = newPath[i].distTo(newPath[i+1]);
    newPath[i+1].velocity = fmin(newPath[i+1].velocity, sqrt(newPath[i].velocity * newPath[i].velocity + 2.0 * maxAcceleration * dist));
    newPath[i].acceleration = (newPath[i+1].velocity * newPath[i+1].velocity - newPath[i].velocity * newPath[i].velocity) / 2.0 / dist;
  }
}

MotionPath::Pt& MotionPath::getClosestPt(const Vec& position) {
  unsigned int closestPtIndex = lastClosestPtIndex;
  for (unsigned int i = lastClosestPtIndex; i < newPath.size(); i++) {
    if (hypot(newPath[i].x - position.x, newPath[i].y - position.y) < hypot(newPath[closestPtIndex].x - position.x, newPath[closestPtIndex].y - position.y)) {
      closestPtIndex = i;
    }
  }
  lastClosestPtIndex = closestPtIndex;
  return newPath[closestPtIndex];
}

double MotionPath::getTValue(Pt& start, Pt& end, const Vec& position) {
  Pt d = Pt(end.x - start.x, end.y - start.y);
  Pt f = Pt(start.x - position.x, start.y - position.y);

  auto a = d * d;
  auto b = 2 * (f * d);
  auto c = f * f - viewDistance * viewDistance;
  auto discriminant = b * b - 4 * a * c; 

  if (discriminant >= 0)  {
    discriminant = sqrt(discriminant);
    double t1 = ((-b - discriminant) / (2 * a));
    double t2 = ((-b + discriminant) / (2 * a));

    if(t2 >= 0 && t2 <= 1) {
      return t2;
    } else if (t1 >= 0 && t1 <= 1) {
      return t1;
    }   
  }
  return -1;
}

MotionPath::Pt MotionPath::getLookAheadPt(const Vec& position) {
  unsigned int closestIndex = lastClosestPtIndex;
  // std::cout << "lastClosestIndex: " << lastClosestPtIndex << std::endl;;

  for (unsigned int i = 0; i < newPath.size() - 1; i++) {
    // std::cout << "i: " << i << " / " << newPath.size() - 1 << std::endl;;
    Pt start = newPath[i];
    Pt end = newPath[i+1];
    // std::cout << "    start: " << start.x << " , " << start.y << std::endl;
    // std::cout << "    end: " << end.x << " , " << end.y << std::endl;
    auto t = getTValue(start, end, position);
    // std::cout << "    t: " << t << std::endl;
    if (t >= 0) {
      // std::cout << "  VALID T " << t << std::endl;
      if (i + prevLookAheadTValue > prevLookAheadIndex) {
        prevLookAheadIndex = i;
        prevLookAheadTValue = t;
        // std::cout << "  CHOSEN T " << ((double) prevLookAheadIndex + prevLookAheadTValue) << std::endl;
        break;
      }
    }
  }
  return Pt(newPath[prevLookAheadIndex].x + (newPath[prevLookAheadIndex + 1].x - newPath[prevLookAheadIndex].x) * prevLookAheadTValue, newPath[prevLookAheadIndex].y + (newPath[prevLookAheadIndex + 1].y - newPath[prevLookAheadIndex].y) * prevLookAheadTValue);
}

double MotionPath::calculateCurvature(const Vec& position, const Pt& lookAheadPt) {
  double a = -1/tan(position.h);
  double b = 1;
  double c = ((1/tan(position.h))*position.x) - position.y;

  double x = fabs(lookAheadPt.x * a + lookAheadPt.y * b + c) / sqrt(a * a + b * b);
  double side = (cos(position.h) * (lookAheadPt.x - position.x) - sin(position.h) * (lookAheadPt.y - position.y));
  side = side < 0 ? -1.0 : 1.0;

  // std::cout << lookAheadPt.x << " * " << a << " + " << lookAheadPt.y << " *  " << b << " + " << c << "  =  "  << x << std::endl;

  return (2 * x) / (viewDistance * viewDistance) * side;
}

std::pair<double, double>  MotionPath::calculateVelocity(const double& curvature, const Pt& closestPt) {
  double vel = isReversed ? -closestPt.velocity : closestPt.velocity;
  double leftVelocity =  vel * (2.0 + curvature * 25.0) / 2.0;
  double rightVelocity = vel * (2.0 - curvature * 25.0) / 2.0;
  if (isReversed) {
    return { leftVelocity, rightVelocity };
  } else {
    return { leftVelocity, rightVelocity };
  }
}

std::pair<double, double>  MotionPath::calculateAcceleration(const double& curvature, const Pt& closestPt) {
  double accel = isReversed ? -closestPt.acceleration : closestPt.acceleration;
  double leftAcceleration =  accel * (2.0 + curvature * 25.0) / 2.0;
  double rightAcceleration = accel * (2.0 - curvature * 25.0) / 2.0;
  if (isReversed) {
    return { leftAcceleration, rightAcceleration };
  } else {
    return { leftAcceleration, rightAcceleration };
  }
}

#endif AUTONOMOUS_H