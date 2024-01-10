#ifndef SIGNALSTATS_H
#define SIGNALSTATS_H

class SignalStats {
public:
    int numCuts;
    double distance;
    double fraction;
    double factor;

    SignalStats(int numCuts_, double distance_, double fraction_,  double factor_)
        : numCuts(numCuts_), distance(distance_), fraction(fraction_), factor(factor_) {};

    SignalStats()
        : numCuts(0), distance(0), fraction(0), factor(0) {};
};

#endif // SIGNALSTATS_H