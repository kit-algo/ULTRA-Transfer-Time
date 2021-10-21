#pragma once

#include "../../../Helpers/Console/Progress.h"
#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class StopEventGraphBuilder {

private:
    class StopLabel {

    public:
        StopLabel() :
            arrivalTime(INFTY),
            timeStamp(0) {
        }

    public:
        inline void checkTimeStamp(const int newTimeStamp) noexcept {
            arrivalTime = (timeStamp != newTimeStamp) ? INFTY : arrivalTime;
            timeStamp = newTimeStamp;
        }

        inline void update(const int newTimeStamp, const int newArrivalTime) noexcept {
            checkTimeStamp(newTimeStamp);
            arrivalTime = std::min(arrivalTime, newArrivalTime);
        }

    public:
        int arrivalTime;
        int timeStamp;
    };

public:
    StopEventGraphBuilder(const Data& data) :
        data(data),
        labels(data.numberOfStops()),
        timeStamp(0) {
        stopEventGraph.addVertices(data.numberOfStopEvents());
    }

public:
    inline void scanTrip(const TripId trip) noexcept {
        const StopId* stops = data.stopArrayOfTrip(trip);
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        for (StopIndex i = StopIndex(1); i < data.numberOfStopsInTrip(trip); i++) {
            const int arrivalTime = data.raptorData.stopEvents[firstEvent + i].arrivalTime;
            scanRoutes(trip, i, stops[i], arrivalTime);
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stops[i])) {
                scanRoutes(trip, i, StopId(data.raptorData.transferGraph.get(ToVertex, edge)), arrivalTime + data.raptorData.transferGraph.get(TravelTime, edge));
            }
        }
    }

    inline void scanRoutes(const TripId trip, const StopIndex i, const StopId stop, const int arrivalTime) noexcept  {
        const RouteId originalRoute = data.routeOfTrip[trip];
        for (const RAPTOR::RouteSegment& route : data.raptorData.routesContainingStop(stop)) {
            const TripId other = data.getEarliestTrip(route, arrivalTime);
            if (other == noTripId) continue;
            if ((route.routeId == originalRoute) && (other >= trip) && (route.stopIndex >= i)) continue;
            if (isUTransfer(trip, i, other, route.stopIndex)) continue;
            stopEventGraph.addEdge(Vertex(data.firstStopEventOfTrip[trip] + i), Vertex(data.firstStopEventOfTrip[other] + route.stopIndex));
        }
    }

    inline bool isUTransfer(const TripId fromTrip, const StopIndex fromIndex, const TripId toTrip, const StopIndex toIndex) const noexcept {
        if (fromIndex < 2) return false;
        if (toIndex + 1 >= data.numberOfStopsInTrip(toTrip)) return false;
        if (data.getStop(fromTrip, StopIndex(fromIndex - 1)) != data.getStop(toTrip, StopIndex(toIndex + 1))) return false;
        if (data.getStopEvent(fromTrip, StopIndex(fromIndex - 1)).arrivalTime > data.getStopEvent(toTrip, StopIndex(toIndex + 1)).departureTime) return false;
        return true;
    }

    inline void reduceTransfers(const TripId trip) noexcept {
        timeStamp++;
        const StopId* stops = data.stopArrayOfTrip(trip);
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        for (StopIndex i = StopIndex(data.numberOfStopsInTrip(trip) - 1); i > 0; i--) {
            const int arrivalTime = data.raptorData.stopEvents[firstEvent + i].arrivalTime;
            labels[stops[i]].update(timeStamp, arrivalTime);
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stops[i])) {
                labels[data.raptorData.transferGraph.get(ToVertex, edge)].update(timeStamp, arrivalTime + data.raptorData.transferGraph.get(TravelTime, edge));
            }

            std::vector<Edge> transfers;
            const Vertex StopEventVertex = Vertex(data.firstStopEventOfTrip[trip] + i);
            for (const Edge edge : stopEventGraph.edgesFrom(StopEventVertex)) {
                transfers.emplace_back(edge);
            }
            std::sort(transfers.begin(), transfers.end(), [&](const Edge a, const Edge b){
                return data.raptorData.stopEvents[stopEventGraph.get(ToVertex, a)].arrivalTime < data.raptorData.stopEvents[stopEventGraph.get(ToVertex, b)].arrivalTime;
            });

            std::vector<Edge> deleteTransfers;
            for (const Edge transfer : transfers) {
                bool keep = false;
                const StopEventId transferTarget = StopEventId(stopEventGraph.get(ToVertex, transfer));
                const StopIndex transferTargetIndex = data.indexOfStopEvent[transferTarget];
                const TripId transferTargetTrip = data.tripOfStopEvent[transferTarget];
                const StopId* transferTargetStops = data.stopArrayOfTrip(transferTargetTrip) + transferTargetIndex;
                for (size_t j = data.numberOfStopsInTrip(transferTargetTrip) - transferTargetIndex - 1; j > 0; j--) {
                    const StopId transferTargetStop = transferTargetStops[j];
                    const int transferTargetTime = data.raptorData.stopEvents[transferTarget + j].arrivalTime;
                    labels[transferTargetStop].checkTimeStamp(timeStamp);
                    if (labels[transferTargetStop].arrivalTime > transferTargetTime) {
                        labels[transferTargetStop].arrivalTime = transferTargetTime;
                        keep = true;
                    }
                    for (const Edge edge : data.raptorData.transferGraph.edgesFrom(transferTargetStop)) {
                        const StopId arrivalStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                        const int arrivalTime = transferTargetTime + data.raptorData.transferGraph.get(TravelTime, edge);
                        labels[arrivalStop].checkTimeStamp(timeStamp);
                        if (labels[arrivalStop].arrivalTime > arrivalTime) {
                            labels[arrivalStop].arrivalTime = arrivalTime;
                            keep = true;
                        }
                    }
                }
                if (!keep) deleteTransfers.emplace_back(transfer);
            }

            std::sort(deleteTransfers.begin(), deleteTransfers.end(), [](const Edge a, const Edge b){
                return a > b;
            });
            for (const Edge transfer : deleteTransfers) {
                stopEventGraph.deleteEdge(StopEventVertex, transfer);
            }
        }
    }

public:
    const Data& data;

    SimpleDynamicGraph stopEventGraph;

    std::vector<StopLabel> labels;
    int timeStamp;

};

inline void ComputeStopEventGraph(Data& data) noexcept {
    Progress progress(data.numberOfTrips());
    StopEventGraphBuilder builder(data);
    for (const TripId trip : data.trips()) {
        builder.scanTrip(trip);
        builder.reduceTransfers(trip);
        progress++;
    }
    Graph::move(std::move(builder.stopEventGraph), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

inline void ComputeStopEventGraph(Data& data, const int numberOfThreads, const int pinMultiplier = 1) noexcept {
    Progress progress(data.numberOfTrips());
    SimpleEdgeList stopEventGraph;
    stopEventGraph.addVertices(data.numberOfStopEvents());

    const int numCores = numberOfCores();

    omp_set_num_threads(numberOfThreads);
    #pragma omp parallel
    {
        int threadId = omp_get_thread_num();
        pinThreadToCoreId((threadId * pinMultiplier) % numCores);
        AssertMsg(omp_get_num_threads() == numberOfThreads, "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

        StopEventGraphBuilder builder(data);
        const size_t numberOfTrips = data.numberOfTrips();

        #pragma omp for schedule(dynamic,1)
        for (size_t i = 0; i < numberOfTrips; i++) {
            const TripId trip = TripId(i);
            builder.scanTrip(trip);
            builder.reduceTransfers(trip);
            progress++;
        }

        #pragma omp critical
        {
            for (const auto [edge, from] : builder.stopEventGraph.edgesWithFromVertex()) {
                stopEventGraph.addEdge(from, builder.stopEventGraph.get(ToVertex, edge));
            }
        }
    }

    Graph::move(std::move(stopEventGraph), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

}
