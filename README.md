[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Fast Multimodal Journey Planning for Three Criteria
This C++ framework contains code for the ALENEX22 publication "Fast Multimodal Journey Planning for Three Criteria". It provides algorithms for efficient journey planning in multimodal networks with three optimization criteria: arrival time, number of public transit trips, and time spent using the transfer mode (e.g., walking, cycling). This framework was developed at [KIT](https://www.kit.edu) in the [group of Prof. Dorothea Wagner](https://i11www.iti.kit.edu/).

## Usage
All query algorithms and preprocessing steps are available via the console application ``ULTRA``, which can be compiled using the ``Makefile`` located in the ``Runnables`` folder. It includes the following commands:

* CH computation:
    - ``buildCH`` computes a normal CH needed for the (Mc)ULTRA query algorithms.
    - ``coreCH`` computes a core-CH, which is used as input for the (Mc)ULTRA shortcut computation.
* (Mc)ULTRA shortcut computation:
    - ``computeStopToStopShortcuts`` computes stop-to-stop ULTRA shortcuts for use with ULTRA-RAPTOR.
    - ``computeEventToEventShortcuts`` computes event-to-event ULTRA shortcuts for use with ULTRA-TB.
    - ``computeMcStopToStopShortcuts`` computes stop-to-stop McULTRA shortcuts for use with ULTRA-McRAPTOR and ULTRA-BM-RAPTOR.
    - ``computeMcEventToEventShortcuts`` computes event-to-event McULTRA shortcuts for use with ULTRA-McTB and ULTRA-BM-TB.
    - ``augmentTripBasedShortcuts`` performs the shortcut augmentation step that is required for ULTRA-BM-TB.
* Query algorithms:
    - ``runULTRAMcRAPTORQueries`` runs random queries with ULTRA-McRAPTOR, which computes full Pareto sets.
    - ``runULTRAMcTripBasedQueries`` runs random queries with ULTRA-McTB, which computes full Pareto sets.
    - ``runBoundedULTRAMcRAPTORQueries`` runs random queries with ULTRA-BM-RAPTOR, which computes restricted Pareto sets.
    - ``runBoundedULTRAMcTripBasedQueries`` runs random queries with ULTRA-BM-TB, which computes restricted Pareto sets.
    - Similar commands are available for the two-criteria and transitive variants of these algorithms, which are used for baseline comparisons.

## Networks
We use custom data formats for loading the public transit network and the transfer graph. The Intermediate format allows for easy network manipulation, while the RAPTOR format is required by the preprocessing and query algorithms. The Switzerland and London networks used in our experiments are available at [https://i11www.iti.kit.edu/PublicTransitData/ULTRA/](https://i11www.iti.kit.edu/PublicTransitData/ULTRA/) in the required formats. Unfortunately, we cannot provide the Germany network because it is proprietary.

The ``Network`` application provides commands for manipulating the network data and for converting public transit data to our custom format. It includes the following commands:
* ``parseGTFS`` converts GFTS data in CSV format to a binary format.
* ``gtfsToIntermediate`` converts GFTS binary data to the Intermediate network format.
* ``intermediateToRAPTOR`` converts a network in Intermediate format to RAPTOR format.
* ``loadDimacsGraph`` converts a graph in the format used by the [9th DIMACS Implementation Challenge](http://users.diag.uniroma1.it/challenge9/download.shtml) to our custom binary graph format.
* ``duplicateTrips`` duplicates all trips in the network and shifts them by a specified time offset. This is used to extend networks that only comprise a single day to two days, in order to allow for overnight journeys.
* ``addGraph`` adds a transfer graph to a network in Intermediate format. Existing transfer edges in the network are preserved.
* ``replaceGraph`` replaces the transfer graph of a network with a specified transfer graph.
* ``reduceGraph`` contracts all vertices with degree less than 3 in the transfer graph.
* ``reduceToMaximumConnectedComponent`` reduces a network to its largest connected component.
* ``applyBoundingBox`` removes all parts of a network that lie outside a predefined bounding box.
* ``applyCustomBoundingBox`` removes all parts of a network that lie outside a specified bounding box.
* ``makeOneHopTransfers`` computes one-hop transfers for all stops whose distance is below a specified threshold. This is used to create a transitively closed network for comparison with non-multi-modal algorithms.
* ``applyMaxTransferSpeed`` applies a maximum transfer speed to all edges in the transfer graph.
* ``applyConstantTransferSpeed`` applies a constant transfer speed to all edges in the transfer graph and computes the travel times accordingly.

An example script that combines all steps necessary to load a public transit network is provided at ``Runnables/BuildNetworkExample.script``. It can be run from the ``Network`` application using ``runScript BuildNetworkExample.script``. It takes as input GFTS data in CSV format located at ``Networks/Switzerland/GTFS/`` and a road graph in DIMACS format located at ``Networks/Switzerland/OSM/dimacs``.
