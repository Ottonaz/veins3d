#include "Statistics.h"
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"

Define_Module(Statistics);

void Statistics::initialize(int stage) {

    run = getSimulation()->getActiveEnvir()->getConfigEx()->getActiveRunNumber();

    std::stringstream a;
    a << par("outputFilePath").stringValue() << "-#" << run << ".csv";
    outputFilePath =  a.str();
    EV_TRACE << "Output file path: " << outputFilePath << endl;

    file.open(outputFilePath, std::ios_base::ate);

    if (!file) {
        std::cerr << "can't open the file." << std::endl;
        return;
    }

    file << "run,"  << "sim_time," << "src_addr," << "dst_addr," << "dst_posx," << "dst_posy," << "h_bs," << "rssi," << "snr," << "dst_module," << "distance,"<< "numCuts,"<< "fractionInObstacle" "\n";

}


void Statistics::writeDataToCSV(double src, double dst, double dst_posx, double dst_posy, double snr, double rssi, double sim_time, double h_bs, const std::string& module, SignalStats signalStats) {


    if (!file) {
        return;
    }

    file << run << "," << sim_time << "," << src << "," 
         << dst << "," << dst_posx << "," << dst_posy << "," 
         << h_bs << "," << rssi << "," << snr  << "," << module 
         << "," << signalStats.distance << "," <<  signalStats.numCuts  << "," << signalStats.fraction <<  "\n";

}

void Statistics::finish() {

    file.close();

}
