﻿

#include <iostream>

#include "osmium/osm/types.hpp"
#include "beaches/graph.h"

#include "cpl_conv.h"

int main(int argc, const char* args[])
{
    CPLSetConfigOption("CPL_DEBUG", "ON");
    CPLSetConfigOption("CPL_LOG_ERRORS", "ON");

    if (argc < 2) {
        std::cout << "First parameter must be a file name.\n";
        std::exit(-1);
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    beaches::Graph graph;
    graph.load(args[1], args[2], args[3]);

    auto endTime = std::chrono::high_resolution_clock::now();

    std::cout << "Took " << std::chrono::duration_cast<std::chrono::minutes>(endTime - startTime).count() << " minutes.\n";

    std::vector<beaches::Edge*> incidentEdges;
    graph.getEdges(4271711924, incidentEdges);

    osmium::object_id_type myId = 1;
}
