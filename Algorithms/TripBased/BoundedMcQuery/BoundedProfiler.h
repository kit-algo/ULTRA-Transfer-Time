#pragma once

#include <iostream>

#include "../../../Helpers/String/String.h"
#include "../../../Helpers/Timer.h"

namespace TripBased {

class BoundedProfiler {
public:
    inline void reset() noexcept {
        numQueries = 0;
        addJourneyCount = 0;
        enqueueCount = 0;
        scannedTripsCount = 0;
        scannedStopsCount = 0;
        roundCount = 0;
        chTime = 0.0;
        clearTime = 0.0;
        forwardTime = 0.0;
        backwardTime = 0.0;
        mainTime = 0.0;
        queryTime = 0.0;
    }

    inline void start() noexcept {
        numQueries++;
        queryTimer.restart();
    }

    inline void done() noexcept {
        queryTime += queryTimer.elapsedMicroseconds();
    }

    inline void printStatistics() const noexcept {
        std::cout << "Number of enqueued trips: " << String::prettyDouble(enqueueCount / numQueries, 0) << std::endl;
        std::cout << "Number of scanned trips: " << String::prettyDouble(scannedTripsCount / numQueries, 0) << std::endl;
        std::cout << "Number of scanned stops: " << String::prettyDouble(scannedStopsCount / numQueries, 0) << std::endl;
        std::cout << "Number of rounds: " << String::prettyDouble(static_cast<double>(roundCount) / numQueries, 2) << std::endl;
        std::cout << "Number of found journeys: " << String::prettyDouble(addJourneyCount / numQueries, 0) << std::endl;
        std::cout << "Bucket-CH query time: " << String::musToString(chTime / numQueries) << std::endl;
        std::cout << "Clear time: " << String::musToString(clearTime / numQueries) << std::endl;
        std::cout << "Forward pruning search time: " << String::musToString(forwardTime / numQueries) << std::endl;
        std::cout << "Backward pruning search time: " << String::musToString(backwardTime / numQueries) << std::endl;
        std::cout << "Main search time: " << String::musToString(mainTime / numQueries) << std::endl;
        std::cout << "Total query time: " << String::musToString(queryTime / numQueries) << std::endl;
    }

public:
    size_t numQueries{0};
    size_t addJourneyCount{0};
    size_t enqueueCount{0};
    size_t scannedTripsCount{0};
    size_t scannedStopsCount{0};
    size_t roundCount{0};
    Timer chTimer;
    Timer phaseTimer;
    Timer queryTimer;
    double chTime{0.0};
    double clearTime{0.0};
    double forwardTime{0.0};
    double backwardTime{0.0};
    double mainTime{0.0};
    double queryTime{0.0};
};

}
