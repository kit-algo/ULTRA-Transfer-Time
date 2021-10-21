#pragma once

#include <iostream>
#include <algorithm>
#include <random>
#include <vector>
#include <string>
#include <cmath>

#include "../../Shell/Shell.h"

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"

#include "../../Algorithms/RAPTOR/BoundedMcRAPTOR/BoundedMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/BoundedULTRAMcRAPTOR/BoundedULTRAMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/DijkstraMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../../Algorithms/RAPTOR/McRAPTOR.h"
#include "../../Algorithms/RAPTOR/RAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRAMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRARAPTOR.h"
#include "../../Algorithms/TripBased/BoundedMcQuery/BoundedMcQuery.h"
#include "../../Algorithms/TripBased/Query/McQuery.h"
#include "../../Algorithms/TripBased/Query/Query.h"
#include "../../Algorithms/TripBased/Query/TransitiveQuery.h"

using namespace Shell;

class RunTransitiveRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveRAPTORQueries", "Runs the given number of random transitive RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false> raptor(raptorData);

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{StopId(rand() % raptorData.numberOfStops()), StopId(rand() % raptorData.numberOfStops()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            raptor.run(queries[i].source, queries[i].departureTime, queries[i].target);
            numJourneys += raptor.getJourneys().size();
        }
        raptor.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;

    }

private:
    struct Query {
        StopId source;
        StopId target;
        int departureTime;
    };
};

class RunDijkstraRAPTORQueries : public ParameterizedCommand {

public:
    RunDijkstraRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runDijkstraRAPTORQueries", "Runs the given number of random Dijkstra RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> dijkstraRaptor(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{Vertex(rand() % ch.numVertices()), Vertex(rand() % ch.numVertices()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            dijkstraRaptor.run(queries[i].source, queries[i].departureTime, queries[i].target);
            numJourneys += dijkstraRaptor.getJourneys().size();
        }
        dijkstraRaptor.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }

private:
    struct Query {
        Vertex source;
        Vertex target;
        int departureTime;
    };
};

class RunULTRARAPTORQueries : public ParameterizedCommand {

public:
    RunULTRARAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRARAPTORQueries", "Runs the given number of random ULTRA-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::ULTRARAPTOR<true, RAPTOR::BucketCHInitialTransfers, RAPTOR::AggregateProfiler, false> ultraRaptor(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{Vertex(rand() % ch.numVertices()), Vertex(rand() % ch.numVertices()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            ultraRaptor.run(queries[i].source, queries[i].departureTime, queries[i].target);
            numJourneys += ultraRaptor.getJourneys().size();
        }
        ultraRaptor.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }

private:
    struct Query {
        Vertex source;
        Vertex target;
        int departureTime;
    };
};

class RunTransitiveMcRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveMcRAPTORQueries", "Runs the given number of random transitive McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        RAPTOR::McRAPTOR<true, true, RAPTOR::AggregateProfiler> mcRaptor(raptorData);

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{StopId(rand() % raptorData.numberOfStops()), StopId(rand() % raptorData.numberOfStops()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            mcRaptor.run(queries[i].source, queries[i].departureTime, queries[i].target);
            numJourneys += mcRaptor.getJourneys().size();
        }
        mcRaptor.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }

private:
    struct Query {
        StopId source;
        StopId target;
        int departureTime;
    };
};

class RunTransitiveBoundedMcRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveBoundedMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveBoundedMcRAPTORQueries", "Runs the given number of random transitive Bounded McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{StopId(rand() % raptorData.numberOfStops()), StopId(rand() % raptorData.numberOfStops()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            algorithm.run(queries[i].source, queries[i].departureTime, queries[i].target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }

private:
    struct Query {
        StopId source;
        StopId target;
        int departureTime;
    };
};

class RunDijkstraMcRAPTORQueries : public ParameterizedCommand {

public:
    RunDijkstraMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runDijkstraMcRAPTORQueries", "Runs the given number of random Dijkstra McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::DijkstraMcRAPTOR<true, RAPTOR::AggregateProfiler> dijkstraRaptor(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{Vertex(rand() % ch.numVertices()), Vertex(rand() % ch.numVertices()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            dijkstraRaptor.run(queries[i].source, queries[i].departureTime, queries[i].target);
            numJourneys += dijkstraRaptor.getJourneys().size();
        }
        dijkstraRaptor.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }

private:
    struct Query {
        Vertex source;
        Vertex target;
        int departureTime;
    };
};

class RunULTRAMcRAPTORQueries : public ParameterizedCommand {

public:
    RunULTRAMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRAMcRAPTORQueries", "Runs the given number of random ULTRA-McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::ULTRAMcRAPTOR<true, RAPTOR::AggregateProfiler> ultraRaptor(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{Vertex(rand() % ch.numVertices()), Vertex(rand() % ch.numVertices()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            ultraRaptor.run(queries[i].source, queries[i].departureTime, queries[i].target);
            numJourneys += ultraRaptor.getJourneys().size();
        }
        ultraRaptor.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }

private:
    struct Query {
        Vertex source;
        Vertex target;
        int departureTime;
    };
};

class RunBoundedULTRAMcRAPTORQueries : public ParameterizedCommand {

public:
    RunBoundedULTRAMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runBoundedULTRAMcRAPTORQueries", "Runs the given number of random Bounded ULTRA-McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::BoundedULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{Vertex(rand() % ch.numVertices()), Vertex(rand() % ch.numVertices()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            algorithm.run(queries[i].source, queries[i].departureTime, queries[i].target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }

private:
    struct Query {
        Vertex source;
        Vertex target;
        int departureTime;
    };
};

class RunULTRATripBasedQueries : public ParameterizedCommand {

public:
    RunULTRATripBasedQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRATripBasedQueries", "Runs the given number of random ULTRA-TripBased queries.") {
        addParameter("Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        CH::CH ch(getParameter("CH data"));
        TripBased::Query<TripBased::ReachedIndexSmall, true> algorithm(tripBasedData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{Vertex(rand() % ch.numVertices()), Vertex(rand() % ch.numVertices()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            algorithm.run(queries[i].source, queries[i].departureTime, queries[i].target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }

private:
    struct Query {
        Vertex source;
        Vertex target;
        int departureTime;
    };
};

class RunULTRAMcTripBasedQueries : public ParameterizedCommand {

public:
    RunULTRAMcTripBasedQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRAMcTripBasedQueries", "Runs the given number of random ULTRA-McTripBased queries.") {
        addParameter("Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        CH::CH ch(getParameter("CH data"));
        TripBased::McQuery<true> algorithm(tripBasedData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{Vertex(rand() % ch.numVertices()), Vertex(rand() % ch.numVertices()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            algorithm.run(queries[i].source, queries[i].departureTime, queries[i].target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }

private:
    struct Query {
        Vertex source;
        Vertex target;
        int departureTime;
    };
};

class RunBoundedULTRAMcTripBasedQueries : public ParameterizedCommand {

public:
    RunBoundedULTRAMcTripBasedQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runBoundedULTRAMcTripBasedQueries", "Runs the given number of random Bounded ULTRA-McTripBased queries.") {
        addParameter("Trip-Based input file");
        addParameter("Bounded forward Trip-Based input file");
        addParameter("Bounded backward Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        TripBased::Data forwardBoundedData(getParameter("Bounded forward Trip-Based input file"));
        forwardBoundedData.printInfo();
        TripBased::Data backwardBoundedData(getParameter("Bounded backward Trip-Based input file"));
        backwardBoundedData.printInfo();
        CH::CH ch(getParameter("CH data"));
        TripBased::BoundedMcQuery<true> algorithm(tripBasedData, forwardBoundedData, backwardBoundedData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Query{Vertex(rand() % ch.numVertices()), Vertex(rand() % ch.numVertices()), rand() % (24 * 60 * 60)});
        }

        double numJourneys = 0;
        for (size_t i = 0; i < n; i++) {
            algorithm.run(queries[i].source, queries[i].departureTime, queries[i].target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }

private:
    struct Query {
        Vertex source;
        Vertex target;
        int departureTime;
    };
};

class ComputeTransferTimeSavings : public ParameterizedCommand {

public:
    ComputeTransferTimeSavings(BasicShell& shell) :
        ParameterizedCommand(shell, "computeTransferTimeSavings", "Computes the savings in transfer time of a 3-criteria (bounded) Pareto set compared to a 2-criteria one.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::BoundedULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        srand(42);
        std::vector<Query> queries;
        for (size_t i = 0; i < n; i++) {
            queries.emplace_back(Vertex(rand() % ch.numVertices()), Vertex(rand() % ch.numVertices()), rand() % (24 * 60 * 60));
        }

        IO::OFStream outputFile(getParameter("Output file"));
        outputFile << "ArrivalSlack";
        for (const double tripSlack : tripSlacks) {
            const int slackAsInt = tripSlack * 100 - 100;
            for (const double threshold : thresholds) {
                const int thresholdAsInt = threshold * 100;
                outputFile << "\tTripSlack" << slackAsInt << "Savings" << thresholdAsInt;
            }
        }
        outputFile << "\n";
        outputFile.flush();

        for (const double arrivalSlack : arrivalSlacks) {
            outputFile << arrivalSlack;
            for (const double tripSlack : tripSlacks) {
                std::cout << "Arrival slack: " << arrivalSlack << ", trip slack: " << tripSlack << std::endl;
                std::vector<double> transferTimeSavings;
                for (const Query& query : queries) {
                    algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
                    const std::vector<RAPTOR::WalkingParetoLabel> fullLabels = algorithm.getResults();
                    const std::vector<RAPTOR::ArrivalLabel>& anchorLabels = algorithm.getAnchorLabels();
                    RAPTOR::WalkingParetoLabel bestLabel;
                    RAPTOR::WalkingParetoLabel bestAnchorLabel;
                    for (const RAPTOR::WalkingParetoLabel& label : fullLabels) {
                        if (label.walkingDistance <= bestLabel.walkingDistance) {
                            bestLabel = label;
                        }
                        if (label.walkingDistance <= bestAnchorLabel.walkingDistance && isAnchorLabel(label, anchorLabels)) {
                            bestAnchorLabel = label;
                        }
                    }
                    if (bestAnchorLabel.walkingDistance == 0) {
                        transferTimeSavings.emplace_back(0);
                    } else {
                        transferTimeSavings.emplace_back((bestAnchorLabel.walkingDistance - bestLabel.walkingDistance)/static_cast<double>(bestAnchorLabel.walkingDistance));
                    }
                }
                std::sort(transferTimeSavings.begin(), transferTimeSavings.end(), [&](const double a, const double b) {
                    return a > b;
                });
                size_t j = 0;
                std::vector<size_t> savingsCount(thresholds.size(), 0);
                for (const double s : transferTimeSavings) {
                    while (s < thresholds[j]) {
                        j++;
                        if (j == thresholds.size()) break;
                    }
                    if (j == thresholds.size()) break;
                    savingsCount[j]++;
                }
                for (const size_t c : savingsCount) {
                    const double ratio = c/static_cast<double>(transferTimeSavings.size());
                    outputFile << "\t" << ratio;
                }

            }
            outputFile << "\n";
            outputFile.flush();
        }
    }

private:
    std::vector<double> thresholds { 0.75, 0.5, 0.25 };
    std::vector<double> arrivalSlacks { 1, 1.1, 1.2, 1.3, 1.4, 1.5 };
    std::vector<double> tripSlacks { 1, 1.25, 1.5 };

    struct Query {
        Query(const Vertex source, const Vertex target, const int departureTime) :
            source(source),
            target(target),
            departureTime(departureTime) {
        }

        Vertex source;
        Vertex target;
        int departureTime;
    };

    inline bool isAnchorLabel(const RAPTOR::WalkingParetoLabel& label, const std::vector<RAPTOR::ArrivalLabel>& anchorLabels) const noexcept {
        for (const RAPTOR::ArrivalLabel& anchorLabel : anchorLabels) {
            if (label.arrivalTime != anchorLabel.arrivalTime) continue;
            if (label.numberOfTrips != anchorLabel.numberOfTrips) continue;
            return true;
        }
        return false;
    }
};
