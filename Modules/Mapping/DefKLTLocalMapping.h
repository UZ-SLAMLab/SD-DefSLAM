/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

#ifndef DEFKLTLOCALMAPPING_H
#define DEFKLTLOCALMAPPING_H

#include "DefLocalMapping.h"
#include "WarpDatabase.h"
namespace ORB_SLAM2
{
  class KeyFrame;
}
namespace defSLAM
{
  using ORB_SLAM2::KeyFrame;
  using ORB_SLAM2::Map;
  using ORB_SLAM2::MapDrawer;

  class DefKLTLocalMapping : public DefLocalMapping
  {
  public:
    DefKLTLocalMapping(Map *pMap, const string &strSettingPath);

    DefKLTLocalMapping(Map *pMap, const SettingsLoader &settingLoader);

    ~DefKLTLocalMapping() = default;

    /*********************************
   * Create the new map points. They are extracted from the surface 
   * estimated for the keyframe with the Isometric NRSfM. KLT variation
   ********************************/
    virtual void CreateNewMapPoints() override;
  };

} // namespace defSLAM

#endif // LOCALMAPPING_H
