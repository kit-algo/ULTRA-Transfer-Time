#pragma once

#include <iostream>
#include <string>

#include "../../Algorithms/RAPTOR/ULTRA/Builder.h"
#include "../../Algorithms/RAPTOR/ULTRA/McBuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/McULTRABuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/ULTRABuilder.h"

#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"

#include "../../Helpers/MultiThreading.h"

#include "../../Shell/Shell.h"

using namespace Shell;

class ComputeStopToStopShortcuts : public ParameterizedCommand {

public:
    ComputeStopToStopShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeStopToStopShortcuts", "Computes stop-to-stop transfer shortcuts using ULTRA.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Witness limit");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
        addParameter("Prune with existing shortcuts?", "true");
        addParameter("Require direct transfer?", "false");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const size_t witnessLimit = getParameter<size_t>("Witness limit");
        const size_t numberOfThreads = getNumberOfThreads();
        const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");
        const bool pruneWithExistingShortcuts = getParameter<bool>("Prune with existing shortcuts?");
        const bool requireDirectTransfer = getParameter<bool>("Require direct transfer?");

        RAPTOR::Data data = RAPTOR::Data::FromBinary(inputFile);
        data.useImplicitDepartureBufferTimes();
        data.printInfo();
        choosePrune(data, numberOfThreads, pinMultiplier, witnessLimit, requireDirectTransfer, pruneWithExistingShortcuts);
        data.dontUseImplicitDepartureBufferTimes();
        Graph::printInfo(data.transferGraph);
        data.transferGraph.printAnalysis();
        data.serialize(outputFile);
    }

private:
    inline size_t getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

    inline void choosePrune(RAPTOR::Data& data, const size_t numberOfThreads, const size_t pinMultiplier, const size_t witnessLimit, const bool requireDirectTransfer, const bool pruneWithExistingShortcuts) const noexcept {
        if (pruneWithExistingShortcuts) {
            chooseRequireDirectTransfer<true>(data, numberOfThreads, pinMultiplier, witnessLimit, requireDirectTransfer);
        } else {
            chooseRequireDirectTransfer<false>(data, numberOfThreads, pinMultiplier, witnessLimit, requireDirectTransfer);
        }
    }

    template<bool PRUNE_WITH_EXISTING_SHORTCUTS>
    inline void chooseRequireDirectTransfer(RAPTOR::Data& data, const size_t numberOfThreads, const size_t pinMultiplier, const size_t witnessLimit, const bool requireDirectTransfer) const noexcept {
        if (requireDirectTransfer) {
            run<PRUNE_WITH_EXISTING_SHORTCUTS, true>(data, numberOfThreads, pinMultiplier, witnessLimit);
        } else {
            run<PRUNE_WITH_EXISTING_SHORTCUTS, false>(data, numberOfThreads, pinMultiplier, witnessLimit);
        }
    }

    template<bool PRUNE_WITH_EXISTING_SHORTCUTS, bool REQUIRE_DIRECT_TRANSFER>
    inline void run(RAPTOR::Data& data, const size_t numberOfThreads, const size_t pinMultiplier, const size_t witnessLimit) const noexcept {
        RAPTOR::ULTRA::Builder<false, PRUNE_WITH_EXISTING_SHORTCUTS, REQUIRE_DIRECT_TRANSFER> shortcutGraphBuilder(data);
        std::cout << "Computing stop-to-stop ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), witnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);
    }
};

class ComputeMcStopToStopShortcuts : public ParameterizedCommand {

public:
    ComputeMcStopToStopShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeMcStopToStopShortcuts", "Computes stop-to-stop transfer shortcuts using ULTRA.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Witness limit");
        addParameter("Use tiebreaking key?");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        if (getParameter<bool>("Use tiebreaking key?")) {
            run<true>();
        } else {
            run<false>();
        }
    }

private:
    inline size_t getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

    template<bool USE_TIEBREAKING_KEY>
    inline void run() const noexcept {
        const std::string inputFile = getParameter("Input file");
        const size_t witnessLimit = getParameter<size_t>("Witness limit");
        const std::string outputFile = getParameter("Output file");
        const size_t numberOfThreads = getNumberOfThreads();
        const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");

        RAPTOR::Data data = RAPTOR::Data::FromBinary(inputFile);
        data.useImplicitDepartureBufferTimes();
        data.printInfo();

        RAPTOR::ULTRA::McBuilder<false, USE_TIEBREAKING_KEY> shortcutGraphBuilder(data);
        std::cout << "Computing multicriteria stop-to-stop ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), witnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);

        data.dontUseImplicitDepartureBufferTimes();
        Graph::printInfo(data.transferGraph);
        data.transferGraph.printAnalysis();
        data.serialize(outputFile);
    }
};

class ComputeEventToEventShortcuts : public ParameterizedCommand {

public:
    ComputeEventToEventShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeEventToEventShortcuts", "Computes event-to-event transfer shortcuts using ULTRA and saves the resulting network in Trip-Based format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Witness limit");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const int witnessLimit = getParameter<int>("Witness limit");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");

        RAPTOR::Data raptor = RAPTOR::Data::FromBinary(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        TripBased::ULTRABuilder shortcutGraphBuilder(data);
        std::cout << "Computing event-to-event ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), witnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getStopEventGraph()), data.stopEventGraph);

        data.printInfo();
        data.serialize(outputFile);
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

};

class ComputeMcEventToEventShortcuts : public ParameterizedCommand {

public:
    ComputeMcEventToEventShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeMcEventToEventShortcuts", "Computes multicriteria event-to-event transfer shortcuts using ULTRA and saves the resulting network in Trip-Based format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Intermediate witness limit");
        addParameter("Final witness limit");
        addParameter("Use tiebreaking key?");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        if (getParameter<bool>("Use tiebreaking key?")) {
            run<true>();
        } else {
            run<false>();
        }
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

    template<bool USE_TIEBREAKING_KEY>
    inline void run() const noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");
        const int intermediateWitnessLimit = getParameter<int>("Intermediate witness limit");
        const int finalWitnessLimit = getParameter<int>("Final witness limit");

        RAPTOR::Data raptor = RAPTOR::Data::FromBinary(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        TripBased::McULTRABuilder<false, USE_TIEBREAKING_KEY> shortcutGraphBuilder(data);
        std::cout << "Computing multicriteria event-to-event ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), intermediateWitnessLimit, finalWitnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getStopEventGraph()), data.stopEventGraph);

        data.printInfo();
        data.serialize(outputFile);
    }

};

class AugmentTripBasedShortcuts : public ParameterizedCommand {

public:
    AugmentTripBasedShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "augmentTripBasedData", "Augments Trip-Based shortcuts for bounded multicriteria search.") {
        addParameter("Input file");
        addParameter("Forward output file");
        addParameter("Backward output file");
    }

    virtual void execute() noexcept {
        TripBased::Data data(getParameter("Input file"));
        data.printInfo();
        TripBased::Data reverseData = data.reverseNetwork();
        data.augmentShortcuts();
        reverseData.augmentShortcuts();
        data.serialize(getParameter("Forward output file"));
        reverseData.serialize(getParameter("Backward output file"));
    }
};
