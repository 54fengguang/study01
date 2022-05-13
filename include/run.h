#pragma once
#include "common.h"
#include "directseed.h"
#include "plough.h"

namespace routing
{
      void running(std::vector<PointXYZ> &m_FNode, std::vector<Obstacle> &obstacles, PointXYZ *locationNode, Machine *mac, Tractor *tra, double interSp, double typeSeeding, double ChangeOperation, double AccOverlap, double sd);
      std::vector<PointXYZ> getNode();
}