#ifndef __V2X_LEADER_TIMESYNCHRONIZER_H_
#define __V2X_LEADER_TIMESYNCHRONIZER_H_

#include <omnetpp.h>
using namespace omnetpp;
class TimeSynchronizer : public cSimpleModule
{
private:
  bool waitingForAdvance;        // Simulation paused until client allows
  simtime_t stepSize;            // How much time to advance per step
  cMessage *stepEvent = nullptr; // Self-message to trigger steps

protected:
  virtual void initialize() override;
  virtual void handleMessage(cMessage *msg) override;
  virtual void finish();

public:
  void advanceStep();
  bool isWaiting() const;
  simtime_t getTimeStep();
};

#endif
