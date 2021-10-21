#include "Commands/CH.h"
#include "Commands/Preprocessing.h"
#include "Commands/Queries.h"

#include "../Helpers/Console/CommandLineParser.h"

#include "../Shell/Shell.h"
using namespace Shell;

int main(int argc, char** argv) {
    CommandLineParser clp(argc, argv);
    pinThreadToCoreId(clp.value<int>("core", 1));
    checkAsserts();
    ::Shell::Shell shell;
    new BuildCH(shell);
    new CoreCH(shell);

    new ComputeStopToStopShortcuts(shell);
    new ComputeMcStopToStopShortcuts(shell);
    new ComputeEventToEventShortcuts(shell);
    new ComputeMcEventToEventShortcuts(shell);
    new AugmentTripBasedShortcuts(shell);

    new RunTransitiveRAPTORQueries(shell);
    new RunDijkstraRAPTORQueries(shell);
    new RunULTRARAPTORQueries(shell);
    new RunTransitiveMcRAPTORQueries(shell);
    new RunTransitiveBoundedMcRAPTORQueries(shell);
    new RunDijkstraMcRAPTORQueries(shell);
    new RunULTRAMcRAPTORQueries(shell);
    new RunBoundedULTRAMcRAPTORQueries(shell);
    new RunULTRATripBasedQueries(shell);
    new RunULTRAMcTripBasedQueries(shell);
    new RunBoundedULTRAMcTripBasedQueries(shell);

    new ComputeTransferTimeSavings(shell);
    shell.run();
    return 0;
}
