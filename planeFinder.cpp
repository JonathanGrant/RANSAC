#include "SimplePly.h"
#include <iostream>
#include <Eigen/Dense>
#include <algorithm>

void swap(PlyPoint p1, PlyPoint p2) {
  PlyPoint temp = p1;
  p1 = p2;
  p2 = temp;
}

Eigen::Matrix<double, 3, 1> getNormalUnitVectorFromThreePoints(PlyPoint p1, PlyPoint p2, PlyPoint p3) {
  Eigen::Matrix<double, 3, 1> unitVector = (p3.location - p1.location).cross(p2.location - p1.location);
  return unitVector / unitVector.norm();
}

double distanceFromPlane(Eigen::Matrix<double, 3, 1> normalUnitVector, PlyPoint pointOnPlane, PlyPoint otherPoint) {
  return normalUnitVector.dot(pointOnPlane.location - otherPoint.location);
}

void RansacAndColor(SimplePly ply, int nPlanes, double threshold, int nTrials, SimplePly output) {
  // Get the colors ready
  std::vector<Eigen::Vector3i> colors;
  colors.push_back(Eigen::Vector3i(0,0,0));
  colors.push_back(Eigen::Vector3i(255,0,0));
  colors.push_back(Eigen::Vector3i(0,255,0));
  colors.push_back(Eigen::Vector3i(0,0,255));
  colors.push_back(Eigen::Vector3i(255,255,0));
  colors.push_back(Eigen::Vector3i(255,0,255));
  colors.push_back(Eigen::Vector3i(0,255,255));
  colors.push_back(Eigen::Vector3i(255,255,255));
  colors.push_back(Eigen::Vector3i(127,0,0));
  colors.push_back(Eigen::Vector3i(0,127,0));
  colors.push_back(Eigen::Vector3i(0,0,127));
  colors.push_back(Eigen::Vector3i(127,127,0));
  colors.push_back(Eigen::Vector3i(127,0,127));
  colors.push_back(Eigen::Vector3i(0,127,127));
  colors.push_back(Eigen::Vector3i(127,127,127));

  std::cout << "Starting RANSAC" << std::endl;

  for(int nPlanesFound = 0; nPlanesFound < nPlanes; nPlanesFound++) {
    std::cout << "RANSAC loop " << nPlanesFound + 1 << std::endl;
    std::vector<int> mostPointIndexesOnPlane;
    for(int currTrial = 0; currTrial < nTrials; currTrial++) {
      std::cout << "RANSAC trial " << currTrial + 1 << std::endl;
      // Get three random points from ply
      PlyPoint p1 = ply[rand() % ply.size()];
      Eigen::Matrix<double, 3, 1> currNormalVectorForPlane = getNormalUnitVectorFromThreePoints(p1, ply[rand() % ply.size()], ply[rand() % ply.size()]);
      std::vector<int> pointIndexesOnPlane;
      for(int pointIndex = 0; pointIndex < ply.size(); pointIndex++) {
        if (distanceFromPlane(currNormalVectorForPlane, p1, ply[pointIndex]) <= threshold) {
          pointIndexesOnPlane.push_back(pointIndex);
        }
      }
      if(pointIndexesOnPlane.size() > mostPointIndexesOnPlane.size()) {
        mostPointIndexesOnPlane = pointIndexesOnPlane;
      }
    }
    // Color the points, add them to the output, and remove them from ply
    for(int pointIndex = mostPointIndexesOnPlane.size() - 1; pointIndex >= 0; pointIndex--) {
      // Color them from one of 15 distinct colors.
      ply[pointIndex].colour = colors[nPlanesFound % colors.size()];

      // Add to the output
      output.push_back(ply[pointIndex]);

      // Fastest way to remove
      swap(ply[pointIndex], ply.back());
      ply.pop_back();
    }
  }
}

int main (int argc, char *argv[]) {

  // Check the commandline arguments.
  if (argc != 6) {
    std::cout << "Usage: planeFinder <input file> <output file> <number of planes> <point-plane threshold> <number of RANSAC trials>" << std::endl;
    return -1;
  }
  int nPlanes = atoi(argv[3]);
  double threshold = atof(argv[4]);
  int nTrials = atoi(argv[5]);

  std::cout << "Searching for " << nPlanes << " planes" << std::endl;
  std::cout << "Using a point-plane threshold of " << threshold << " units" << std::endl;
  std::cout << "Applying RANSAC with " << nTrials << " trials" << std::endl;  

  // Storage for the point cloud.
  SimplePly ply;

  // Read in the data from a PLY file
  std::cout << "Reading PLY data from " << argv[1] << std::endl;
  if (!ply.read(argv[1])) {
    std::cout << "Could not read PLY data from file " << argv[1] << std::endl;
    return -1;
  }
  std::cout << "Read " << ply.size() << " points" << std::endl;

  // Do the Ransac
  SimplePly output;
  RansacAndColor(ply, nPlanes, threshold, nTrials, output);

  // Write the resulting (re-coloured) point cloud to a PLY file.
  std::cout << "Writing PLY data to " << argv[2] << std::endl;
  if (!output.write(argv[2])) {
    std::cout << "Could not write PLY data to file " << argv[2] << std::endl;
    return -2;
  }

  return 0;
}
