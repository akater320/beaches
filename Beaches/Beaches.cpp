#include <iostream>

#include "beaches/graph.h"
#include "cpl_conv.h"

void printUsage() {
    std::cout << "Usage: Beaches input_file_name [OPTIONS] output_file_name\n";
    std::cout << "Options:\n";
    std::cout << "\t -d \t\tExport debug vectors. Note: This can be a lot of vectors and should only be done on extracts.\n";
    std::cout << "\t -b ##\t\tBuffer the vectors by this distance. e.g. \"-b .001 -b .005\" \n";
    std::cout << "\t -g rows cols\tThe rows and columns in the polygonm grid. Default is \"-g 180 360\"\n";
    std::cout << "\t -p ##\t\tThe number of degrees to pad across +/-180 longitude. Default is 0.\n";
    std::cout << "Example:\n";
    std::cout << "\tBeaches planet.osm.pbf -b .001 -b .002 -b .003 -p 20.0 -g 180 400 planet_mask\n";
}

int main(int argc, const char* args[])
{
    //CPLSetConfigOption("CPL_DEBUG", "ON");
    //CPLSetConfigOption("CPL_LOG_ERRORS", "ON");

    if (argc < 3 || strcmp(args[1], "-h") == 0 || strcmp(args[1], "--help") == 0) {
        printUsage();
        std::exit(0);
    }

    const char* inputFileName = args[1];
    const char* outputFileName = args[argc - 1];
    
    bool exportDebugVectors = false;
    std::vector<double> bufferSizes;
    int rows = 180;
    int columns = 360;
    double padding = 0.0;

    int curArgIndex = 2;
    while (curArgIndex < (argc - 1)) {
        const char* curArg = args[curArgIndex];
        curArgIndex++;
        if (strcmp(curArg, "-d") == 0) {
            exportDebugVectors = true;
        }
        else if (strcmp(curArg, "-b") == 0) {
            bufferSizes.push_back(std::atof(args[curArgIndex++]));
        }
        else if (strcmp(curArg, "-g") == 0) {
            rows = atoi(args[curArgIndex++]);
            columns = atoi(args[curArgIndex++]);
        }
        else if (strcmp(curArg, "-p") == 0) {
            padding = atof(args[curArgIndex++]);
        }
        else {
            std::cout << "Unexpected argument :" << curArg << "\n";
            std::exit(-1);
        }
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    beaches::Graph graph;
    graph.load(args[1]);
    auto endTime = std::chrono::high_resolution_clock::now();
    std::cout << "Took " << std::chrono::duration_cast<std::chrono::minutes>(endTime - startTime).count() << " minutes.\n";

    if (exportDebugVectors) {
        graph.exportEdges(inputFileName, outputFileName);
    }
    else {
        graph.exportBufferedLinestrings(inputFileName, bufferSizes, padding, rows, columns, outputFileName);
    }
}
