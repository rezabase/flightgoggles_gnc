#pragma once

#include "Math/Quaternion.h"
#include "Utility/FixedQueue.h"
#include <vector>

using namespace SLR;

#define MAX_TRAJECTORY_POINTS 1000




// Struct for holding all of the data related to a single trajectory point
struct TrajectoryPoint {
  float time;
  V3F position;
  V3F velocity;
  V3F omega;
  V3F accel;
  Quaternion<float> attitude;

  // Initialise all fields to zero when declared
  TrajectoryPoint() :
    time(0.f),
    position(0.f, 0.f, 0.f),
    velocity(0.f, 0.f, 0.f),
    omega(0.f, 0.f, 0.f),
    attitude(0.f, 0.f, 0.f, 0.f)
  {
  }
};



class Trajectory {
public:
  Trajectory();
  Trajectory(const string& filename);
  ~Trajectory();
  bool ReadFile(const string& filename);
  void ParseLine(const string& filename, const string& s);
  void Clear();
  void SetLogFile(const string& filename);
  void AddTrajectoryPoint(TrajectoryPoint traj_pt);
  TrajectoryPoint NextTrajectoryPoint(float time);
  void WriteTrajectoryPointToFile(FILE* f, TrajectoryPoint traj_pt);

  vector<TrajectoryPoint> traj; // vector containing the trajectory points

  int GetCurTrajectoryPoint() const { return _curTrajPoint; }


private:
  string _log_filename;
  FILE* _log_file;
  int _curTrajPoint;
};
