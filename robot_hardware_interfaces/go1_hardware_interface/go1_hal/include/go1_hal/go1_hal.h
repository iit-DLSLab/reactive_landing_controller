#ifndef _DLS_GO1_HAL_H_
#define _DLS_GO1_HAL_H_

#include <array>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
// using namespace UNITREE_LEGGED_SDK;
namespace unitree = UNITREE_LEGGED_SDK;

namespace go1hal 
{


inline namespace unitree { using namespace ::unitree; }


class LowLevelInterface
{
public:
  LowLevelInterface();

  LowState ReceiveObservation();

  void SendLowCmd(LowCmd& motorcmd);
  void SendCommand(std::array<float, 60> motorcmd);
  void InitCmdData(LowCmd& motorcmd);

  UDP udp_;
  Safety safe_;

  LowCmd lowcmd_ = {0};
  LowState lowstate_ = {0};

};


class HighLevelInterface
{
public:
  HighLevelInterface();

  HighState ReceiveObservation();

  void SendHighCmd(HighCmd& descmd);
  // void SendCommand(std::array<float, 60> motorcmd);

  UDP udp_;
  Safety safe_;

  HighCmd highcmd_ = {0};
  HighState highstate_ = {0};

};


}


#endif
