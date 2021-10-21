#pragma once

#include "ReachedIndexSmall.h"
#include "Query.h"

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/Container/Set.h"

namespace TripBased {

template<typename REACHED_INDEX, bool DEBUG = false>
class TransitiveQuery {

public:
    using ReachedIndex = REACHED_INDEX;
    static constexpr bool Debug = DEBUG;
    using Type = Query<ReachedIndex, Debug>;

private:
    struct TripLabel {
        TripLabel(const u_int32_t begin, const u_int32_t end) :
            begin(begin),
            end(end) {
        }
        u_int32_t begin;
        u_int32_t end;
    };

    struct EdgeLabel {
        EdgeLabel(const StopEventId stopEvent = noStopEvent, const TripId trip = noTripId, const StopEventId firstEvent = noStopEvent) :
            stopEvent(stopEvent),
            trip(trip),
            firstEvent(firstEvent) {
        }
        StopEventId stopEvent;
        TripId trip;
        StopEventId firstEvent;
    };

    struct RouteLabel {
        RouteLabel() :
            numberOfTrips(0) {
        }
        inline StopIndex end() const noexcept {
            return StopIndex(departureTimes.size() / numberOfTrips);
        }
        u_int32_t numberOfTrips;
        std::vector<int> departureTimes;
    };

public:
    TransitiveQuery(const Data& data) :
        data(data),
        reverseTransferGraph(data.raptorData.transferGraph),
        transferFromSource(data.numberOfStops(), INFTY),
        transferToTarget(data.numberOfStops(), INFTY),
        lastSource(Vertex(0)),
        lastTarget(Vertex(0)),
        reachedRoutes(data.numberOfRoutes()),
        reachedIndex(data),
        edgeLabels(data.stopEventGraph.numEdges()),
        routeLabels(data.numberOfRoutes()) {
        reverseTransferGraph.revert();
        for (const Edge edge : data.stopEventGraph.edges()) {
            edgeLabels[edge].stopEvent = StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
            edgeLabels[edge].trip = data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
            edgeLabels[edge].firstEvent = data.firstStopEventOfTrip[edgeLabels[edge].trip];
        }
        for (const RouteId route : data.raptorData.routes()) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const size_t numberOfTrips = data.raptorData.numberOfTripsInRoute(route);
            const RAPTOR::StopEvent* stopEvents = data.raptorData.firstTripOfRoute(route);
            routeLabels[route].numberOfTrips = numberOfTrips;
            routeLabels[route].departureTimes.resize((numberOfStops - 1) * numberOfTrips);
            for (size_t trip = 0; trip < numberOfTrips; trip++) {
                for (size_t stopIndex = 0; stopIndex + 1 < numberOfStops; stopIndex++) {
                    routeLabels[route].departureTimes[(stopIndex * numberOfTrips) + trip] = stopEvents[(trip * numberOfStops) + stopIndex].departureTime;
                }
            }
        }
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target) noexcept {
        if constexpr (Debug) profiler.start();
        clear();
        computeInitialAndFinalTransfers(source, departureTime, target);
        evaluateInitialTransfers(source, departureTime);
        scanTrips();
        if constexpr (Debug) profiler.done();
    }

    inline int getEarliestArrivalTime() const noexcept {
        return minArrivalTimeByMaxNumberOfUsedVehicles.back();
    }

    inline int getEarliestArrivalNumberOfTrips() const noexcept {
        const int eat = minArrivalTimeByMaxNumberOfUsedVehicles.back();
        for (size_t i = 0; i < minArrivalTimeByMaxNumberOfUsedVehicles.size(); i++) {
            if (minArrivalTimeByMaxNumberOfUsedVehicles[i] == eat) return i;
        }
        return -1;
    }

    inline std::vector<RAPTOR::ArrivalLabel> getArrivals() const noexcept {
        std::vector<RAPTOR::ArrivalLabel> result;
        for (size_t i = 0; i < minArrivalTimeByMaxNumberOfUsedVehicles.size(); i++) {
            if (minArrivalTimeByMaxNumberOfUsedVehicles[i] >= INFTY) continue;
            if ((result.size() >= 1) && (result.back().arrivalTime == minArrivalTimeByMaxNumberOfUsedVehicles[i])) continue;
            result.emplace_back(minArrivalTimeByMaxNumberOfUsedVehicles[i], i);
        }
        return result;
    }

    inline Profiler& getProfiler() noexcept {
        return profiler;
    }

private:
    inline void clear() noexcept {
        currentQueue.clear();
        nextQueue.clear();
        reachedIndex.clear();
        numberOfUsedVehicles = 0;
        minArrivalTime = INFTY;
        std::vector<int>(1, INFTY).swap(minArrivalTimeByMaxNumberOfUsedVehicles);
    }

    inline void computeInitialAndFinalTransfers(const Vertex source, const int departureTime, const Vertex target) noexcept {
        if constexpr (Debug) profiler.phaseTimer.restart();
        for (const Edge edge : data.raptorData.transferGraph.edgesFrom(lastSource)) {
            const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
            if (!data.isStop(stop)) continue;
            transferFromSource[stop] = INFTY;
        }
        for (const Edge edge : reverseTransferGraph.edgesFrom(lastTarget)) {
            const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
            if (!data.isStop(stop)) continue;
            transferToTarget[stop] = INFTY;
        }
        for (const Edge edge : data.raptorData.transferGraph.edgesFrom(source)) {
            const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
            if (!data.isStop(stop)) continue;
            transferFromSource[stop] = data.raptorData.transferGraph.get(TravelTime, edge);
        }
        for (const Edge edge : reverseTransferGraph.edgesFrom(target)) {
            const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
            if (stop == source) addJourney(departureTime + reverseTransferGraph.get(TravelTime, edge));
            if (!data.isStop(stop)) continue;
            transferToTarget[stop] = reverseTransferGraph.get(TravelTime, edge);
        }
        lastSource = source;
        lastTarget = target;
        if constexpr (Debug) profiler.chTime += profiler.phaseTimer.elapsedMicroseconds();
    }

    inline void evaluateInitialTransfers(const Vertex source, const int departureTime) noexcept {
        if constexpr (Debug) profiler.phaseTimer.restart();
        reachedRoutes.clear();
        for (const Edge edge : data.raptorData.transferGraph.edgesFrom(source)) {
            const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
            if (!data.isStop(stop)) continue;
            for (const RAPTOR::RouteSegment& route : data.raptorData.routesContainingStop(StopId(stop))) {
                reachedRoutes.insert(route.routeId);
            }
        }
        reachedRoutes.sort();
        for (const RouteId route : reachedRoutes) {
            const RouteLabel& label = routeLabels[route];
            const StopIndex endIndex = label.end();
            const TripId firstTrip = data.firstTripOfRoute[route];
            const StopId* stops = data.raptorData.stopArrayOfRoute(route);
            TripId tripIndex = noTripId;
            for (StopIndex stopIndex(0); stopIndex < endIndex; stopIndex++) {
                const int timeFromSource = transferFromSource[stops[stopIndex]];
                if (timeFromSource == INFTY) continue;
                const u_int32_t labelIndex = stopIndex * label.numberOfTrips;
                if (tripIndex >= label.numberOfTrips) {
                    tripIndex = std::lower_bound(TripId(0), TripId(label.numberOfTrips), departureTime + timeFromSource, [&](const TripId trip, const int time) {
                        return label.departureTimes[labelIndex + trip] < time;
                    });
                    if (tripIndex >= label.numberOfTrips) continue;
                } else {
                    const int stopDepartureTime = departureTime + timeFromSource;
                    if (label.departureTimes[labelIndex + tripIndex - 1] < stopDepartureTime) continue;
                    tripIndex--;
                    while ((tripIndex > 0) && (label.departureTimes[labelIndex + tripIndex - 1] >= stopDepartureTime)) {
                        tripIndex--;
                    }
                }
                enqueue(firstTrip + tripIndex, stopIndex);
                if (tripIndex == 0) break;
            }
        }
        if constexpr (Debug) profiler.initialTransfersTime += profiler.phaseTimer.elapsedMicroseconds();
    }

    inline void scanTrips() noexcept {
        while (!nextQueue.empty()) {
            if constexpr (Debug) profiler.roundCount++;
            currentQueue.swap(nextQueue);
            numberOfUsedVehicles++;
            for (const TripLabel& label : currentQueue) { // Evaluate final transfers in order to check if the target is reachable
                if constexpr (Debug) profiler.scannedTripsCount++;
                for (StopEventId i(label.begin); i < label.end; i++) {
                    if constexpr (Debug) profiler.scannedStopsCount++;
                    if (data.arrivalEvents[i].arrivalTime >= minArrivalTime) break;
                    const int timeToTarget = transferToTarget[data.arrivalEvents[i].stop];
                    if (timeToTarget != INFTY) addJourney(data.arrivalEvents[i].arrivalTime + timeToTarget);
                }
            }
            for (TripLabel& label : currentQueue) { // Find the range of transfers for each trip
                for (StopEventId i(label.begin); i < label.end; i++) {
                    if (data.arrivalEvents[i].arrivalTime >= minArrivalTime) label.end = i;
                }
                label.begin = data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
                label.end = data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
            }
            for (const TripLabel& label : currentQueue) { // Relax the transfers for each trip
                for (Edge edge(label.begin); edge < label.end; edge++) {
                    enqueue(edge);
                }
            }
            currentQueue.clear();
        }
    }

    inline void enqueue(const TripId trip, const StopIndex index) noexcept {
        if constexpr (Debug) profiler.enqueueCount++;
        if (reachedIndex.alreadyReached(trip, index + 1)) return;
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        nextQueue.emplace_back(firstEvent + index + 1, firstEvent + reachedIndex(trip));
        reachedIndex.update(trip, index);
    }

    inline void enqueue(const Edge edge) noexcept {
        if constexpr (Debug) profiler.enqueueCount++;
        const EdgeLabel& label = edgeLabels[edge];
        if (reachedIndex.alreadyReached(label.trip, label.stopEvent - label.firstEvent)) return;
        nextQueue.emplace_back(label.stopEvent, StopEventId(label.firstEvent + reachedIndex(label.trip)));
        reachedIndex.update(label.trip, StopIndex(label.stopEvent - label.firstEvent));
    }

    inline void addJourney(const int newArrivalTime) noexcept {
        if constexpr (Debug) profiler.addJourneyCount++;
        if (numberOfUsedVehicles >= minArrivalTimeByMaxNumberOfUsedVehicles.size()) {
            minArrivalTimeByMaxNumberOfUsedVehicles.resize(numberOfUsedVehicles + 1, minArrivalTimeByMaxNumberOfUsedVehicles.back());
        }
        AssertMsg(numberOfUsedVehicles + 1 == minArrivalTimeByMaxNumberOfUsedVehicles.size(), "Wrong number of used vehicles!");
        minArrivalTimeByMaxNumberOfUsedVehicles[numberOfUsedVehicles] = std::min(minArrivalTimeByMaxNumberOfUsedVehicles[numberOfUsedVehicles], newArrivalTime);
        minArrivalTime = minArrivalTimeByMaxNumberOfUsedVehicles[numberOfUsedVehicles];
    }

private:
    const Data& data;

    TransferGraph reverseTransferGraph;
    std::vector<int> transferFromSource;
    std::vector<int> transferToTarget;
    Vertex lastSource;
    Vertex lastTarget;

    IndexedSet<false, RouteId> reachedRoutes;

    std::vector<TripLabel> currentQueue;
    std::vector<TripLabel> nextQueue;
    ReachedIndex reachedIndex;

    int minArrivalTime;
    u_int32_t numberOfUsedVehicles;
    std::vector<int> minArrivalTimeByMaxNumberOfUsedVehicles;

    std::vector<EdgeLabel> edgeLabels;
    std::vector<RouteLabel> routeLabels;

    Profiler profiler;

};

}
