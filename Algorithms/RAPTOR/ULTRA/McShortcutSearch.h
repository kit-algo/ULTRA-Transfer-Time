#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../Dijkstra/Dijkstra.h"

#include "../../../Helpers/Meta.h"
#include "../../../Helpers/Helpers.h"
#include "../../../Helpers/Types.h"

#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/RAPTOR/Data.h"

namespace RAPTOR::ULTRA {

template<bool DEBUG = false, bool USE_TIEBREAKING_KEY = true>
class McShortcutSearch {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool UseTiebreakingKey = USE_TIEBREAKING_KEY;
    using Type = McShortcutSearch<Debug, UseTiebreakingKey>;

public:
    struct ShortcutInfo {
        ShortcutInfo(const StopId origin, const StopId destination, const StopEventId target) :
            origin(origin),
            destination(destination) {
            targets.insert(target);
        }

        StopId origin;
        StopId destination;
        Set<StopEventId> targets;
    };

    struct Shortcut {
        Shortcut(const ShortcutInfo& info, const int travelTime) :
            origin(info.origin),
            destination(info.destination),
            travelTime(travelTime) {
        }

        StopId origin;
        StopId destination;
        int travelTime;
    };

    struct ArrivalLabel : public ExternalKHeapElement {
        ArrivalLabel() : arrivalTime(never) {}
        int arrivalTime;
        inline bool hasSmallerKey(const ArrivalLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }
    };

    struct OneTripLabel : public ExternalKHeapElement {
        OneTripLabel(const int arrivalTime = never, const int walkingDistance = INFTY, const StopId shortcutOrigin = noStop, const size_t timestamp = -1) :
            arrivalTime(arrivalTime),
            walkingDistance(walkingDistance),
            shortcutOrigin(shortcutOrigin),
            timestamp(timestamp) {
        }

        OneTripLabel(const OneTripLabel& parentLabel, const int walkingDistance) :
            arrivalTime(parentLabel.arrivalTime + walkingDistance),
            walkingDistance(parentLabel.walkingDistance + walkingDistance),
            shortcutOrigin(parentLabel.shortcutOrigin),
            timestamp(parentLabel.timestamp) {
        }

        int arrivalTime;
        int walkingDistance;
        StopId shortcutOrigin; //Only valid for candidates
        size_t timestamp;

        template<typename LABEL>
        inline bool dominates(const LABEL& other) const noexcept {
            return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
        }

        inline bool isCandidate() const noexcept {
            return shortcutOrigin != noStop;
        }

        inline void witnessify() noexcept {
            shortcutOrigin = noStop;
        }

        inline int getKey() const noexcept {
            if constexpr (UseTiebreakingKey) {
                return arrivalTime;
            } else {
                return arrivalTime + walkingDistance;
            }
        }

        inline bool hasSmallerKey(const OneTripLabel* const other) const noexcept {
            if constexpr (UseTiebreakingKey) {
                return arrivalTime < other->arrivalTime || (arrivalTime == other->arrivalTime && walkingDistance < other->walkingDistance);
            } else {
                return getKey() < other->getKey();
            }
        }
    };

    struct TwoTripsLabel : public ExternalKHeapElement {
        TwoTripsLabel(const int arrivalTime = never, const int walkingDistance = INFTY, const StopEventId finalStopEvent = noStopEvent) :
            arrivalTime(arrivalTime),
            walkingDistance(walkingDistance),
            finalStopEvent(finalStopEvent) {
        }

        TwoTripsLabel(const TwoTripsLabel& parentLabel, const int walkingDistance) :
            arrivalTime(parentLabel.arrivalTime + walkingDistance),
            walkingDistance(parentLabel.walkingDistance + walkingDistance),
            finalStopEvent(noStopEvent) {
        }

        TwoTripsLabel(const OneTripLabel& oneTripLabel) :
            arrivalTime(oneTripLabel.arrivalTime),
            walkingDistance(oneTripLabel.walkingDistance),
            finalStopEvent(noStopEvent) {
        }

        int arrivalTime;
        int walkingDistance;
        StopEventId finalStopEvent; //Only valid for candidates

        template<typename LABEL>
        inline bool dominates(const LABEL& other) const noexcept {
            return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
        }

        inline bool isCandidate() const noexcept {
            return finalStopEvent != noStopEvent;
        }

        inline void witnessify() noexcept {
            finalStopEvent = noStopEvent;
        }

        inline int getKey() const noexcept {
            if constexpr (UseTiebreakingKey) {
                return arrivalTime;
            } else {
                return arrivalTime + walkingDistance;
            }
        }

        inline bool hasSmallerKey(const TwoTripsLabel* const other) const noexcept {
            if constexpr (UseTiebreakingKey) {
                return arrivalTime < other->arrivalTime || (arrivalTime == other->arrivalTime && walkingDistance < other->walkingDistance);
            } else {
                return getKey() < other->getKey();
            }
        }
    };

    template<typename LABEL>
    class Bag : public ExternalKHeapElement {
    public:
        using Label = LABEL;
        using Iterator = typename std::vector<Label>::const_iterator;
        static constexpr int logK = 2;
        static constexpr int K = 1 << logK;

        Bag() : heapSize(0), timestamp(0) {}

        inline Label& operator[](const size_t i) noexcept {
            return labels[i];
        }

        inline const Label& operator[](const size_t i) const noexcept {
            return labels[i];
        }

        inline Iterator begin() const noexcept {
            return labels.begin();
        }

        inline Iterator end() const noexcept {
            return labels.end();
        }

        inline bool empty() const noexcept {
            return labels.empty();
        }

        inline bool heapEmpty() const noexcept {
            return heapSize == 0;
        }

        inline const Label& front() const noexcept {
            AssertMsg(!empty(), "An empty heap has no front!");
            AssertMsg(heapSize > 0, "An empty heap has no front!");
            return labels[0];
        }

        inline int getKey() const noexcept {
            return front().getKey();
        }

        inline bool hasSmallerKey(const Bag* const other) const noexcept {
            return front().hasSmallerKey(&(other->front()));
        }

        inline const Label& extractFront() noexcept {
            AssertMsg(!empty(), "An empty heap has no front!");
            AssertMsg(heapSize > 0, "An empty heap has no front!");
            heapSize--;
            if (heapSize > 0) {
                std::swap(labels[0], labels[heapSize]);
                siftDown(0);
            }
            return labels[heapSize];
        }

        inline bool prune(const Label& newLabel) noexcept {
            return internalMerge<false>(newLabel);
        }

        template<typename OTHER_BAG>
        inline bool prune(const OTHER_BAG& otherBag) noexcept {
            bool pruned = false;
            for (const typename OTHER_BAG::Label& label : otherBag) {
                pruned |= internalMerge<false>(label);
            }
            return pruned;
        }

        template<typename FUNCTION = NO_OPERATION>
        inline bool merge(const Label& newLabel, const FUNCTION& processRemovedHeapLabel = NoOperation) noexcept {
            return internalMerge<true>(newLabel, processRemovedHeapLabel);
        }

        template<typename OTHER_LABEL>
        inline bool dominates(const OTHER_LABEL& other) noexcept {
            for (const Label& label : labels) {
                if (label.dominates(other)) return true;
            }
            return false;
        }

        inline void witnessify() noexcept {
            for (size_t i = 0; i < heapSize; i++) {
                labels[i].witnessify();
            }
        }

    private:
        template<bool ONTO_HEAP, typename FUNCTION = NO_OPERATION>
        inline bool internalMerge(const Label& newLabel, const FUNCTION& processRemovedHeapLabel = NoOperation) noexcept {
            size_t removedLabels = 0;
            size_t removedHeapLabels = 0;
            for (size_t i = 0; i < labels.size(); i++) {
                if (labels[i].dominates(newLabel)) return false;
                if (newLabel.dominates(labels[i])) {
                    removedLabels++;
                    if (i < heapSize) {
                        processRemovedHeapLabel(labels[i]);
                        removedHeapLabels++;
                    }
                    continue;
                }
                labels[i - removedLabels] = labels[i];
            }
            heapSize -= removedHeapLabels;
            labels.resize(labels.size() - removedLabels + 1);
            labels.back() = newLabel;
            if constexpr (ONTO_HEAP) {
                std::swap(labels.back(), labels[heapSize]);
                heapSize++;
                heapify();
            } else {
                if (removedHeapLabels > 0) heapify();
            }
            if constexpr (ONTO_HEAP) {
                return true;
            } else {
                return (removedHeapLabels > 0);
            }
        }

        inline void heapify() noexcept {
            if (heapSize <= 1) return;
            for (size_t i = parent(heapSize - 1); i != size_t(-1); i--) {
                siftDown(i);
            }
        }

        inline void siftDown(size_t i) noexcept {
            AssertMsg(i < heapSize, "siftDown index out of range!");
            while (true) {
                size_t minIndex = i;
                const size_t childrenStart = firstChild(i);
                const size_t childrenEnd = std::min(childrenStart + K, heapSize);
                for (size_t j = childrenStart; j < childrenEnd; j++) {
                    if (labels[j].hasSmallerKey(&labels[minIndex])) {
                        minIndex = j;
                    }
                }
                if (minIndex == i) break;
                std::swap(labels[i], labels[minIndex]);
                i = minIndex;
            }
        }

        inline size_t parent(const size_t i) const noexcept {
            return (i - 1) >> logK;
        }

        inline size_t firstChild(const size_t i) const noexcept {
            return (i << logK) + 1;
        }

        std::vector<Label> labels;
        size_t heapSize;

    public:
        size_t timestamp;
    };

    using OneTripBag = Bag<OneTripLabel>;
    using TwoTripsBag = Bag<TwoTripsLabel>;

    struct RouteLabel {
        const StopEvent* trip;
        int walkingDistance;
        StopId shortcutOrigin;
        StopIndex parentStopIndex;

        inline const StopEvent* parentStopEvent() const noexcept {
            return trip + parentStopIndex;
        }

        inline bool dominates(const RouteLabel& other) const noexcept {
            return trip <= other.trip && walkingDistance <= other.walkingDistance;
        }

        inline bool isCandidate() const noexcept {
            return shortcutOrigin != noStop;
        }
    };

    struct RouteBag {
        using Iterator = typename std::vector<RouteLabel>::const_iterator;

        inline bool merge(const RouteLabel& newLabel) noexcept {
            size_t removedLabels = 0;
            for (size_t i = 0; i < labels.size(); i++) {
                if (labels[i].dominates(newLabel)) return false;
                if (newLabel.dominates(labels[i])) {
                    removedLabels++;
                    continue;
                }
                labels[i - removedLabels] = labels[i];
            }
            labels.resize(labels.size() - removedLabels + 1);
            labels.back() = newLabel;
            return true;
        }

        inline Iterator begin() const noexcept {
            return labels.begin();
        }

        inline Iterator end() const noexcept {
            return labels.end();
        }

        std::vector<RouteLabel> labels;
    };

    struct DepartureLabel {
        DepartureLabel(const RouteId routeId = noRouteId, const StopIndex stopIndex = noStopIndex, const int departureTime = never) : route(routeId, stopIndex), departureTime(departureTime) {}
        RouteSegment route;
        int departureTime;
        inline bool operator<(const DepartureLabel& other) const noexcept {
            return (departureTime > other.departureTime) || ((departureTime == other.departureTime) && (route.routeId < other.route.routeId));
        }
    };

    struct ConsolidatedDepartureLabel {
        ConsolidatedDepartureLabel(const int departureTime = never) : departureTime(departureTime) {}
        std::vector<RouteSegment> routes;
        int departureTime;
        inline bool operator<(const ConsolidatedDepartureLabel& other) const noexcept {
            return departureTime > other.departureTime;
        }
    };

    struct Station {
        Station() : representative(noStop) {}
        StopId representative;
        std::vector<StopId> stops;
        inline void add(const StopId stop) noexcept {
            if (representative > stop) {
                representative = stop;
            }
            stops.emplace_back(stop);
        }
    };

public:
    McShortcutSearch(const Data& data, DynamicTransferGraph& shortcutGraph, const int witnessTransferLimit) :
        data(data),
        shortcutGraph(shortcutGraph),
        stationOfStop(data.numberOfStops()),
        sourceStation(),
        sourceDepartureTime(0),
        shortcutCandidatesInQueue(0),
        shortcutDestinationCandidates(data.numberOfStopEvents()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        witnessTransferLimit(witnessTransferLimit),
        earliestDepartureTime(data.getMinDepartureTime()),
        timestamp(0) {
        AssertMsg(data.hasImplicitBufferTimes(), "Shortcut search requires implicit departure buffer times!");
        Dijkstra<TransferGraph, false> dijkstra(data.transferGraph);
        for (const StopId stop : data.stops()) {
            dijkstra.run(stop, noVertex, [&](const Vertex u) {
                if (!data.isStop(u)) return;
                stationOfStop[stop].add(StopId(u));
            }, NoOperation, [&](const Vertex, const Edge edge) {
                return data.transferGraph.get(TravelTime, edge) > 0;
            });
        }
    }

    inline void run(const StopId source, const int minTime, const int maxTime) noexcept {
        AssertMsg(data.isStop(source), "source (" << source << ") is not a stop!");
        if (stationOfStop[source].representative != source) return;
        setSource(source);
        for (const ConsolidatedDepartureLabel& label : collectDepartures(minTime, maxTime)) {
            runForDepartureTime(label);
            for (const Shortcut& shortcut : shortcuts) {
                if (!shortcutGraph.hasEdge(shortcut.origin, shortcut.destination)) {
                    shortcutGraph.addEdge(shortcut.origin, shortcut.destination).set(TravelTime, shortcut.travelTime);
                } else {
                    AssertMsg(shortcutGraph.get(TravelTime, shortcutGraph.findEdge(shortcut.origin, shortcut.destination)) == shortcut.travelTime, "Edge from " << shortcut.origin << " to " << shortcut.destination << " has inconclusive travel time (" << shortcutGraph.get(TravelTime, shortcutGraph.findEdge(shortcut.origin, shortcut.destination)) << ", " << shortcut.travelTime << ")");
                }
            }
        }
    }

private:
    inline void setSource(const StopId sourceStop) noexcept {
        AssertMsg(directTransferQueue.empty(), "Queue for round 0 is not empty!");
        AssertMsg(stationOfStop[sourceStop].representative == sourceStop, "Source " << sourceStop << " is not representative of its station!");
        clear();
        sourceStation = stationOfStop[sourceStop];
        initialDijkstra();
        sort(stopsReachedByDirectTransfer);
        if constexpr (Debug) {
            std::cout << "   Source stop: " << sourceStop << std::endl;
            std::cout << "   Number of stops reached by direct transfer: " << String::prettyInt(stopsReachedByDirectTransfer.size()) << std::endl;
        }
    }

    inline void runForDepartureTime(const ConsolidatedDepartureLabel& label) noexcept {
        if constexpr (Debug) std::cout << "   Running search for departure time: " << label.departureTime << " (" << String::secToTime(label.departureTime) << ")" << std::endl;

        timestamp++;
        shortcutCandidatesInQueue = 0;
        shortcutDestinationCandidates.clear();
        shortcuts.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();

        sourceDepartureTime = label.departureTime;
        for (const StopId stop : stopsReachedByDirectTransfer) {
            stopsUpdatedByTransfer.insert(stop);
        }
        collectRoutesServingUpdatedStops(label.routes);
        //scanRoutes1Witnesses();
        scanRoutes1(false);
        for (const StopId stop : sourceStation.stops) {
            stopsUpdatedByTransfer.insert(stop);
        }
        collectRoutesServingUpdatedStops<1>();
        scanRoutes1(true);
        intermediateDijkstra();
        collectRoutesServingUpdatedStops<2>();
        scanRoutes2();
        finalDijkstra();
    }

    inline std::vector<ConsolidatedDepartureLabel> collectDepartures(const int minTime, const int maxTime) noexcept {
        AssertMsg(directTransferArrivalLabels[sourceStation.representative].arrivalTime == 0, "Direct transfer for source " << sourceStation.representative << " is incorrect!");
        const int cutoffTime = std::max(minTime, earliestDepartureTime);
        std::vector<DepartureLabel> departureLabels;
        for (const RouteId route : data.routes()) {
            const StopId* stops = data.stopArrayOfRoute(route);
            const size_t tripSize = data.numberOfStopsInRoute(route);
            int minimalTransferTime = never;
            for (size_t stopIndex = 0; stopIndex + 1 < tripSize; stopIndex++) {
                if (directTransferArrivalLabels[stops[stopIndex]].arrivalTime > minimalTransferTime) continue;
                minimalTransferTime = directTransferArrivalLabels[stops[stopIndex]].arrivalTime;
                for (const StopEvent* trip = data.firstTripOfRoute(route); trip <= data.lastTripOfRoute(route); trip += tripSize) {
                    const int departureTime = trip[stopIndex].departureTime - minimalTransferTime;
                    if (departureTime < cutoffTime) continue;
                    if (departureTime > maxTime) break;
                    if (stationOfStop[stops[stopIndex]].representative == sourceStation.representative) {
                        departureLabels.emplace_back(noRouteId, noStopIndex, departureTime);
                    } else {
                        departureLabels.emplace_back(route, StopIndex(stopIndex), departureTime);
                    }
                }
            }
        }
        sort(departureLabels);
        std::vector<ConsolidatedDepartureLabel> result(1);
        for (const DepartureLabel& label : departureLabels) {
            if (label.route.routeId == noRouteId) {
                if (label.departureTime == result.back().departureTime) continue;
                result.back().departureTime = label.departureTime;
                result.emplace_back(label.departureTime);
            } else {
                result.back().routes.emplace_back(label.route);
            }
        }
        result.pop_back();
        return result;
    }

private:
    inline void clear() noexcept {
        sourceStation = Station();

        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(directTransferArrivalLabels);
        stopsReachedByDirectTransfer.clear();

        oneTripQueue.clear();
        std::vector<OneTripBag>(data.transferGraph.numVertices()).swap(oneTripBags);

        twoTripsQueue.clear();
        std::vector<TwoTripsBag>(data.transferGraph.numVertices()).swap(twoTripsBags);

        std::vector<StopEventId>(data.numberOfStopEvents(), noStopEvent).swap(twoTripsRouteParent);

        shortcutCandidatesInQueue = 0;
        shortcutDestinationCandidates.clear();
        shortcuts.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
    }

    template<int CURRENT>
    inline void collectRoutesServingUpdatedStops() noexcept {
        static_assert((CURRENT == 1) | (CURRENT == 2), "Invalid round!");
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if constexpr (CURRENT == 1) {
                    if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < initialArrivalTime(stop)) continue;
                }
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    inline void collectRoutesServingUpdatedStops(const std::vector<RouteSegment>& routes) noexcept {
        for (const RouteSegment& route : routes) {
            AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
            AssertMsg(route.stopIndex + 1 < data.numberOfStopsInRoute(route.routeId), "RouteSegment " << route << " is not a departure event!");
            AssertMsg(data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime >= initialArrivalTime(data.stopOfRouteSegment(route)), "RouteSegment " << route << " is not reachable!");
            if (routesServingUpdatedStops.contains(route.routeId)) {
                routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
            } else {
                routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
            }
        }
    }

    //Not worth the effort. Only saves 4% of shortcuts, but adds 33% running time
    /*inline void scanRoutes1Witnesses() noexcept {
        shortcutCandidatesInQueue = 0;
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            StopId stop = stops[stopIndex];

            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            const StopEvent* lastTrip = data.lastTripOfRoute(route);

            RouteBag routeBag;

            while (stopIndex < tripSize - 1) {
                if (stopsUpdatedByTransfer.contains(stop)) {
                    const int walkingDistance = directTransferArrivalLabels[stop].arrivalTime;
                    const int arrivalTime = initialArrivalTime(stop);
                    const StopEvent* trip = firstTrip;
                    while ((trip < lastTrip) && (trip[stopIndex].departureTime < arrivalTime)) {
                        trip += tripSize;
                    }
                    if (trip[stopIndex].departureTime >= arrivalTime) {
                        RouteLabel newLabel;
                        newLabel.trip = trip;
                        newLabel.walkingDistance = walkingDistance;
                        routeBag.merge(newLabel);
                    }
                }
                stopIndex++;
                stop = stops[stopIndex];
                for (const RouteLabel& label : routeBag) {
                    const OneTripLabel newLabel(label.trip[stopIndex].arrivalTime, label.walkingDistance, noStop, timestamp);
                    arrivalByRoute1(stop, newLabel);
                }
            }
        }
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
    }*/

    inline void scanRoutes1(const bool forCandidates) noexcept {
        shortcutCandidatesInQueue = 0;
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            const StopIndex stopIndex = routesServingUpdatedStops[route];
            TripIterator tripIterator = data.getTripIterator(route, stopIndex);
            StopIndex parentIndex = stopIndex;
            int walkingDistance = directTransferArrivalLabels[tripIterator.stop()].arrivalTime;
            while (tripIterator.hasFurtherStops()) {
                //Find earliest trip that can be entered
                if (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= initialArrivalTime(tripIterator.stop()))) {
                    do {
                        tripIterator.previousTrip();
                    } while (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= initialArrivalTime(tripIterator.stop())));
                    if (!stopsUpdatedByTransfer.contains(tripIterator.stop())) {
                        //Trip was improved by an arrival that was found during a previous RAPTOR iteration.
                        //We already explored this trip during that iteration.
                        //Fast forward to the next stop that was updated in the current iteration.
                        if (!tripIterator.hasEarlierTrip()) break;
                        do {
                            tripIterator.nextStop();
                        } while (tripIterator.hasFurtherStops() && ((!stopsUpdatedByTransfer.contains(tripIterator.stop())) || (tripIterator.previousDepartureTime() < initialArrivalTime(tripIterator.stop()))));
                        continue;
                    }
                    parentIndex = tripIterator.getStopIndex();
                    walkingDistance = directTransferArrivalLabels[tripIterator.stop()].arrivalTime;
                }
                tripIterator.nextStop();
                const OneTripLabel label(tripIterator.arrivalTime(), walkingDistance, forCandidates ? tripIterator.stop() : noStop, timestamp);
                arrivalByRoute1(tripIterator.stop(), label);
            }
        }
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
    }

    inline void scanRoutes2() noexcept {
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            StopId stop = stops[stopIndex];

            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            const StopEvent* lastTrip = data.lastTripOfRoute(route);

            RouteBag routeBag;

            while (stopIndex < tripSize - 1) {
                for (const OneTripLabel& label : oneTripBags[stop]) {
                    if (label.timestamp != timestamp) continue;
                    const StopEvent* trip = firstTrip;
                    while ((trip < lastTrip) && (trip[stopIndex].departureTime < label.arrivalTime)) {
                        trip += tripSize;
                    }
                    if (trip[stopIndex].departureTime < label.arrivalTime) continue;

                    RouteLabel newLabel;
                    newLabel.trip = trip;
                    newLabel.walkingDistance = label.walkingDistance;
                    newLabel.shortcutOrigin = label.shortcutOrigin;
                    newLabel.parentStopIndex = stopIndex;
                    routeBag.merge(newLabel);
                }
                stopIndex++;
                stop = stops[stopIndex];
                for (const RouteLabel& label : routeBag) {
                    TwoTripsLabel newLabel(label.trip[stopIndex].arrivalTime, label.walkingDistance);
                    const StopEventId finalStopEvent = StopEventId(label.trip + stopIndex - &(data.stopEvents[0]));
                    arrivalByRoute2(stop, newLabel, label, stops[label.parentStopIndex], finalStopEvent);
                }
            }
        }
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
    }

    inline void initialDijkstra() noexcept {
        directTransferArrivalLabels[sourceStation.representative].arrivalTime = 0;
        directTransferQueue.update(&(directTransferArrivalLabels[sourceStation.representative]));
        while (!directTransferQueue.empty()) {
            ArrivalLabel* currentLabel = directTransferQueue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(directTransferArrivalLabels[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                if (newArrivalTime < directTransferArrivalLabels[neighborVertex].arrivalTime) {
                    directTransferArrivalLabels[neighborVertex].arrivalTime = newArrivalTime;
                    directTransferQueue.update(&(directTransferArrivalLabels[neighborVertex]));
                }
            }
            if (data.isStop(currentVertex) && stationOfStop[currentVertex].representative != sourceStation.representative) {
                stopsReachedByDirectTransfer.emplace_back(StopId(currentVertex));
            }
        }
    }

    inline void intermediateDijkstra() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        for (const StopId stop : stopsUpdatedByRoute) {
            AssertMsg(oneTripBags[stop].timestamp == timestamp, "Bag of stop " << stop << " was not updated!");
            oneTripQueue.update(&(oneTripBags[stop]));
        }
        if (shortcutCandidatesInQueue == 0) {
            stopsUpdatedByRoute.clear();
            return;
        }

        int transferLimit = intMax;
        while (!oneTripQueue.empty()) {
            OneTripBag* currentBag = oneTripQueue.extractFront();
            const OneTripLabel& currentLabel = currentBag->extractFront();
            if (!currentBag->heapEmpty()) oneTripQueue.update(currentBag);
            const Vertex currentVertex = Vertex(currentBag - &(oneTripBags[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const OneTripLabel newLabel(currentLabel, data.transferGraph.get(TravelTime, edge));
                arrivalByEdge1(neighborVertex, newLabel);
            }
            if (currentLabel.isCandidate()) {
                shortcutCandidatesInQueue--;
            }
            if (shortcutCandidatesInQueue == 0) {
                //Once all candidates have been settled, leave the Dijkstra search running until witnessTransferLimit is met.
                //Note that witnesses above the limit may be pruned, leading to superfluous shortcuts.
                shortcutCandidatesInQueue = -1;
                transferLimit = currentLabel.getKey() + witnessTransferLimit;
                if (transferLimit < currentLabel.getKey()) transferLimit = intMax;
                if constexpr (Debug) std::cout << "   Transfer limit in round 1: " << String::secToString(transferLimit - sourceDepartureTime) << ", travel time: " << (currentLabel.arrivalTime - sourceDepartureTime) << ", walking distance: " << currentLabel.walkingDistance << std::endl;
            }
            if (data.isStop(currentVertex)) {
                stopsUpdatedByTransfer.insert(StopId(currentVertex));
            }
            if (currentLabel.getKey() > transferLimit) break;
        }

        stopsUpdatedByRoute.clear();
    }

    inline void finalDijkstra() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        for (const StopId stop : stopsUpdatedByRoute) {
            AssertMsg(twoTripsBags[stop].timestamp == timestamp, "Bag of stop " << stop << " was not updated!");
            twoTripsQueue.update(&(twoTripsBags[stop]));
        }

        while (!twoTripsQueue.empty()) {
            TwoTripsBag* currentBag = twoTripsQueue.extractFront();
            const TwoTripsLabel& currentLabel = currentBag->extractFront();
            if (!currentBag->heapEmpty()) twoTripsQueue.update(currentBag);
            const Vertex currentVertex = Vertex(currentBag - &(twoTripsBags[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const TwoTripsLabel newLabel(currentLabel, data.transferGraph.get(TravelTime, edge));
                arrivalByEdge2(neighborVertex, newLabel);
            }
            if (data.isStop(currentVertex) && currentLabel.isCandidate()) {
                const StopEventId shortcutDestination = twoTripsRouteParent[currentLabel.finalStopEvent];
                if (shortcutDestination != noStopEvent && shortcutDestinationCandidates.contains(shortcutDestination)) {
                    //No witness dominates this candidate journey => insert shortcut
                    //Because currentLabel represents a candidate, the walking distance is equal to the shortcut length
                    const ShortcutInfo& info = shortcutDestinationCandidates[shortcutDestination];
                    shortcuts.emplace_back(info, currentLabel.walkingDistance);
                    AssertMsg(info.targets.contains(currentLabel.finalStopEvent), "Stop event " << currentLabel.finalStopEvent << " is not contained in shortcutDestinationCandidates list of " << shortcutDestination << "!");
                    //Unmark other candidates using this shortcut, since we don't need them anymore
                    for (const StopEventId obsoleteCandidate : info.targets) {
                        twoTripsRouteParent[obsoleteCandidate] = noStopEvent;
                    }
                    shortcutDestinationCandidates.remove(shortcutDestination);
                }
            }
            if (shortcutDestinationCandidates.empty()) break;
        }
        //Clean up finalStopEvent pointers of obsolete candidates
        for (TwoTripsBag* unsettledBag : twoTripsQueue.data()) {
            unsettledBag->witnessify();
        }

        AssertMsg(shortcutDestinationCandidates.empty(), "There are still shortcut destination candidates left (" << shortcutDestinationCandidates.size() << ")!");
        stopsUpdatedByRoute.clear();
    }

    inline void arrivalByRoute1(const StopId stop, const OneTripLabel& label) noexcept {
        if (!updateOneTripBag(stop, label)) return;
        if (label.isCandidate()) shortcutCandidatesInQueue++;
        //If the bag was improved, remove it from the queue - it will be re-added with the correct key later.
        if (oneTripBags[stop].isOnHeap()) {
            oneTripQueue.remove(&(oneTripBags[stop]));
        }
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByRoute2(const StopId stop, TwoTripsLabel& label, const RouteLabel& routeLabel, const StopId parentStop, const StopEventId finalStopEvent) noexcept {
        const bool isCandidate = routeLabel.isCandidate() && (routeLabel.shortcutOrigin != parentStop) && !shortcutGraph.hasEdge(routeLabel.shortcutOrigin, parentStop);
        if (isCandidate) {
            label.finalStopEvent = finalStopEvent;
        }
        if (!updateTwoTripsBag(stop, label)) return;
        if (twoTripsBags[stop].isOnHeap()) {
            twoTripsQueue.remove(&(twoTripsBags[stop]));
        }
        stopsUpdatedByRoute.insert(stop);
        AssertMsg(!twoTripsBags[stop].heapEmpty(), "Inserting empty bag into queue!");
        if (isCandidate) {
            const StopEventId shortcutDestination = StopEventId(routeLabel.parentStopEvent() - &(data.stopEvents[0]));
            twoTripsRouteParent[finalStopEvent] = shortcutDestination;
            if (!shortcutDestinationCandidates.contains(shortcutDestination)) {
                shortcutDestinationCandidates.insert(shortcutDestination, ShortcutInfo(routeLabel.shortcutOrigin, parentStop, finalStopEvent));
            } else {
                shortcutDestinationCandidates[shortcutDestination].targets.insert(finalStopEvent);
            }
        } else {
            twoTripsRouteParent[finalStopEvent] = noStopEvent;
        }
    }

    inline void arrivalByEdge1(const Vertex vertex, const OneTripLabel& label) noexcept {
        if (!updateOneTripBag(vertex, label)) return;
        if (label.isCandidate()) shortcutCandidatesInQueue++;
        oneTripQueue.update(&(oneTripBags[vertex]));
    }

    inline void arrivalByEdge2(const Vertex vertex, const TwoTripsLabel& label) noexcept {
        if (!updateTwoTripsBag(vertex, label)) return;
        twoTripsQueue.update(&(twoTripsBags[vertex]));
    }

    inline int initialArrivalTime(const Vertex vertex) const noexcept {
        return directTransferArrivalLabels[vertex].arrivalTime + sourceDepartureTime;
    }

    inline void refreshOneTripBag(const Vertex vertex, const OneTripLabel& walkingLabel) noexcept {
        if (oneTripBags[vertex].timestamp != timestamp) {
            oneTripBags[vertex].timestamp = timestamp;
            if (oneTripBags[vertex].prune(walkingLabel)) {
                pruneBagInQueue(oneTripBags[vertex], oneTripQueue);
            }
        }
    }

    inline bool updateOneTripBag(const Vertex vertex, const OneTripLabel& newLabel) noexcept {
        const OneTripLabel walkingLabel = getWalkingLabel(vertex);
        if (walkingLabel.dominates(newLabel)) return false;
        refreshOneTripBag(vertex, walkingLabel);
        return oneTripBags[vertex].merge(newLabel, [&](const OneTripLabel& removedLabel) {
            if (removedLabel.isCandidate()) shortcutCandidatesInQueue--;
        });
    }

    inline bool updateTwoTripsBag(const Vertex vertex, const TwoTripsLabel& newLabel) noexcept {
        const OneTripLabel walkingLabel = getWalkingLabel(vertex);
        if (walkingLabel.dominates(newLabel)) return false;
        refreshOneTripBag(vertex, walkingLabel);
        if (oneTripBags[vertex].dominates(newLabel)) return false;
        if (twoTripsBags[vertex].timestamp != timestamp) {
            twoTripsBags[vertex].timestamp = timestamp;
            if (twoTripsBags[vertex].prune(TwoTripsLabel(walkingLabel))) {
                pruneBagInQueue(oneTripBags[vertex], oneTripQueue);
            }
            if (twoTripsBags[vertex].prune(oneTripBags[vertex])) {
                pruneBagInQueue(twoTripsBags[vertex], twoTripsQueue);
            }
        }
        return twoTripsBags[vertex].merge(newLabel, [&](const TwoTripsLabel& removedLabel) {
            if (!removedLabel.isCandidate()) return;
            const StopEventId shortcutDestination = twoTripsRouteParent[removedLabel.finalStopEvent];
            if (shortcutDestination == noStopEvent || !shortcutDestinationCandidates.contains(shortcutDestination)) return;
            AssertMsg(shortcutDestinationCandidates[shortcutDestination].targets.contains(removedLabel.finalStopEvent), "Stop event " << removedLabel.finalStopEvent << " is not contained in shortcutDestinationCandidates list of " << shortcutDestination << "!");
            shortcutDestinationCandidates[shortcutDestination].targets.erase(removedLabel.finalStopEvent);
            if (shortcutDestinationCandidates[shortcutDestination].targets.empty()) {
                shortcutDestinationCandidates.remove(shortcutDestination);
            }
            twoTripsRouteParent[removedLabel.finalStopEvent] = noStopEvent;
        });
    }

    template<typename BAG_TYPE, typename QUEUE_TYPE>
    inline void pruneBagInQueue(BAG_TYPE& bag, QUEUE_TYPE& queue) noexcept {
        AssertMsg(bag.isOnHeap(), "Heap labels were removed from bag, but bag is not in queue!");
        if (bag.heapEmpty()) {
            queue.remove(&bag);
        } else {
            queue.update(&bag);
        }
    }

    inline OneTripLabel getWalkingLabel(const Vertex vertex) noexcept {
        const int walkingDistance = directTransferArrivalLabels[vertex].arrivalTime;
        return OneTripLabel(walkingDistance == INFTY ? INFTY : sourceDepartureTime + walkingDistance, walkingDistance);
    }

private:
    const Data& data;
    DynamicTransferGraph& shortcutGraph;
    std::vector<Station> stationOfStop;

    Station sourceStation;
    int sourceDepartureTime;

    std::vector<ArrivalLabel> directTransferArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> directTransferQueue;
    std::vector<StopId> stopsReachedByDirectTransfer;

    std::vector<OneTripBag> oneTripBags;
    ExternalKHeap<2, OneTripBag> oneTripQueue;

    std::vector<TwoTripsBag> twoTripsBags;
    ExternalKHeap<2, TwoTripsBag> twoTripsQueue;

    //For each stop event. If a candidate leads there, points to the corresponding shortcut destination stop event
    std::vector<StopEventId> twoTripsRouteParent;

    size_t shortcutCandidatesInQueue;
    IndexedMap<ShortcutInfo, false, StopEventId> shortcutDestinationCandidates;
    std::vector<Shortcut> shortcuts;

    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;
    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;

    int witnessTransferLimit;

    int earliestDepartureTime;

    size_t timestamp;

};

}
