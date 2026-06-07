#include "headers/timeSynchronizer.h"

Define_Module(TimeSynchronizer);

void TimeSynchronizer::initialize()
{

    if (hasPar("stepSize"))
    {
        stepSize = par("stepSize");
    }
    else
    {
        stepSize = 1.0; // Use the default value
    }
    stepEvent = new cMessage("stepEvent");
    waitingForAdvance = true; // nothing scheduled until external trigger
}

void TimeSynchronizer::handleMessage(cMessage *msg)
{
    if (msg == stepEvent)
    {
        // step completed → stop scheduling further
        waitingForAdvance = true;
        EV << "[TimeSynchronizer] Step completed at " << simTime() << "\n";
        // no scheduleAt() here → simulation halts naturally
    }
    else
    {
        delete msg;
    }
}

void TimeSynchronizer::advanceStep()
{
    if (!waitingForAdvance)
    {
        return;
    }
    waitingForAdvance = false;
    scheduleAt(simTime() + stepSize, stepEvent);
    std::cout << "[TimeSynchronizer] Advancing by " << stepSize << " from " << simTime() << "\n";
}

bool TimeSynchronizer::isWaiting() const
{
    return waitingForAdvance;
}
simtime_t TimeSynchronizer::getTimeStep()
{
    return stepSize;
}
void TimeSynchronizer::finish()
{
    cancelAndDelete(stepEvent);
}
