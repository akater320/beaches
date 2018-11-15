#pragma once
#ifndef BEACHES_GRAPH_H
#define BEACHES_GRAPH_H

#include <numeric>
#include <algorithm>

#include "osmium/visitor.hpp"
#include "osmium/io/pbf_input.hpp"

#include "beaches/edge.h"
#include "beaches/node_location_manager.h"
#include "beaches/water_polygon_relation_manager.h"
#include "beaches/way_edges_handler.h"
#include "beaches/reachable_ways_handler.h"
#include "beaches/islands_handler.h"
#include "beaches/aggregate_geom.h"
#include "beaches/grid_geom_aggregator.h"

#include "ogr_feature.h"
#include "ogrsf_frmts.h"
#include "gdal_priv.h"

namespace beaches {
    class Graph {
    public:
        void load(const char* fileName);
        void exportBufferedLinestrings(const char* osmFile, const std::vector<double>& bufferDistance, float padding, int rows, int columns, std::string fileName) noexcept;
        void exportEdges(const char* osmFile, std::string fileName) noexcept;
    private:
        void getEdges(osmium::object_id_type nodeId, std::vector<Edge*>& incidentEdges);
        void initalizeEdgeVectors() noexcept;
        void findCoastlineRing();
        void findReachableWater();
        void populateBranchingNodes(const char* fileName) noexcept;
        void populateUsedNodes(const char* fileName) noexcept;
        void propagateCoastRing(Edge* startEdge, bool waterOnRight) noexcept;
        void propagateReachableWaterPath(Edge* startEdge, bool waterOnRight) noexcept;

        float getDirection(osmium::object_id_type node1, osmium::object_id_type node2) {
            const auto& n1 = m_nodeManager.GetNode(node1);
            const auto& n2 = m_nodeManager.GetNode(node2);
            return std::atan2f(
                n2.Lat() - n1.Lat(),
                n2.Lon() - n1.Lon()
            );
        }

        float getPositiveAngle(float baseDirection, float direction2) {
            constexpr float pi_times_2 = M_PI * 2.0;
            float positiveBase = baseDirection > 0 ? baseDirection : (pi_times_2 + baseDirection);
            float dir2Base = direction2 > 0 ? direction2 : (pi_times_2 + direction2);
            float angle = dir2Base - positiveBase;
            if (angle < 0.0) {
                angle = pi_times_2 + angle;
            }

            return angle;
        }

    private:
        std::vector<Edge> m_edges;
        std::vector<uint32_t> m_reversedEdgeIndices;
        NodeLocationManger m_nodeManager;
    };

    inline void beaches::Graph::load(const char * fileName)
    {
        { //scope for relationManager. Want to release this memory when we're done.
            auto relationManager = WaterPolygonRelationManager();
            std::cout << "Reading relations....\n";
            {
                osmium::io::Reader reader(fileName, osmium::osm_entity_bits::relation);
                while (const auto buffer = reader.read()) {
                    osmium::apply(buffer, relationManager);
                }
            }
            relationManager.sortMembersForLookup();

            std::cout << "Reading ways...\n";
            {
                WayEdgesHandler handler(m_edges, relationManager);
                osmium::io::Reader reader(fileName, osmium::osm_entity_bits::way);
                while (const auto buffer = reader.read()) {
                    osmium::apply(buffer, handler);
                }
            }

            std::cout << "Edges: " << m_edges.size() << "\n";

            initalizeEdgeVectors();

            populateBranchingNodes(fileName);

            findCoastlineRing();

            findReachableWater();

            std::cout << "Ways pass for islands - 1...\n";
            {
                ReachableWaysHandler handler(relationManager, m_edges);
                osmium::io::Reader reader(fileName, osmium::osm_entity_bits::way);
                while (const auto buffer = reader.read()) {
                    osmium::apply(buffer, handler);
                }
            }

            relationManager.updateReachableFlags();

            std::cout << "Ways pass for islands - 2...\n";
            {
                IslandsHandler handler(relationManager, m_edges);
                osmium::io::Reader reader(fileName, osmium::osm_entity_bits::way);
                while (const auto buffer = reader.read()) {
                    osmium::apply(buffer, handler);
                }
            }
        }

        //Find the reachable water inside the islands.
        findReachableWater();
    }

    inline void beaches::Graph::populateBranchingNodes(const char* fileName) noexcept
    {
        m_nodeManager.clear();

        std::cout << "Adding nodes to tracker...\n";

        std::vector<Edge*> incidentEdges;
        osmium::object_id_type prevNode = -1;
        for (const auto& edge : m_edges) {
            if (edge.Node1() != prevNode) {
                prevNode = edge.Node1();
                incidentEdges.clear();
                getEdges(prevNode, incidentEdges);

                if (incidentEdges.size() > 2) {
                    //branching node.
                    for (const auto pEdge : incidentEdges) {
                        m_nodeManager.trackNode(pEdge->Node1());
                        m_nodeManager.trackNode(pEdge->Node2());
                    }
                }

            }
        }

        prevNode = -1;
        for (const auto edgeIndex : m_reversedEdgeIndices) {
            const Edge& edge = m_edges[edgeIndex];
            if (edge.Node2() != prevNode) {
                prevNode = edge.Node2();
                incidentEdges.clear();
                getEdges(prevNode, incidentEdges);

                if (incidentEdges.size() > 2) {
                    //branching node.
                    for (const auto pEdge : incidentEdges) {
                        m_nodeManager.trackNode(pEdge->Node1());
                        m_nodeManager.trackNode(pEdge->Node2());
                    }
                }

            }
        }

        m_nodeManager.prepareHandler();

        std::cout << "Populating node locations...\n";
        {
            osmium::io::Reader reader(fileName, osmium::osm_entity_bits::node);
            while (const auto buffer = reader.read()) {
                osmium::apply(buffer, m_nodeManager);
            }
        }
        std::cout << "Nodes done.\n";
    }

    inline void beaches::Graph::populateUsedNodes(const char* fileName) noexcept {
        m_nodeManager.clear();

        std::cout << "Adding nodes to tracker...\n";

        //For debug add everything? Including inland water?

        osmium::object_id_type prevNode = -1;
        for (const auto& edge : m_edges) {
            if (edge.Node1() != prevNode) {
                prevNode = edge.Node1();
                m_nodeManager.trackNode(prevNode);
            }
        }

        //Sort the nodes so we can avoid adding duplicates. (Even tho they are already sorted!)
        std::cout << "\tsorting...";
        m_nodeManager.prepareHandler();

        prevNode = -1;
        for (const auto edgeIndex : m_reversedEdgeIndices) {
            const Edge& edge = m_edges[edgeIndex];
            if (edge.Node2() != prevNode) {
                prevNode = edge.Node2();
                m_nodeManager.trackNode(prevNode);
            }
        }

        std::cout << "\tsorting...\n";
        m_nodeManager.prepareHandler();

        std::cout << "Nodes pass. Populating locations...\n";
        {
            osmium::io::Reader reader(fileName, osmium::osm_entity_bits::node);
            while (const auto buffer = reader.read()) {
                osmium::apply(buffer, m_nodeManager);
            }
        }
        std::cout << "Nodes done.\n";
    }

    inline void beaches::Graph::initalizeEdgeVectors() noexcept {
        std::cout << "Preparing edges...\n";

        auto forwardOrder = [](const Edge& left, const Edge& right) {
            return (left.Node1() < right.Node1()) ||
                (left.Node1() == right.Node1() && left.Node2() < right.Node2());
        };

        std::cout << "\tSorting " << m_edges.size() << "\n";
        std::sort(std::execution::par, m_edges.begin(), m_edges.end(), forwardOrder);

        //Consolidate parallel edges.
        std::cout << "\tConsolidating edges..\n";
        auto mainEdge = &m_edges.front();
        for (Edge& edge : m_edges) {
            if (mainEdge->Node1() != edge.Node1() || mainEdge->Node2() != edge.Node2()) {
                mainEdge = &edge;
                continue;
            }
            mainEdge->unionFlags(edge.Flags());
            edge.setFlag(EdgeFlags::Erased, true);
        }
        std::cout << "\tRemoving erased...\n";
        m_edges.erase(
            std::remove_if(m_edges.begin(), m_edges.end(), [](const Edge& pEdge) { return pEdge.hasFlag(EdgeFlags::Erased); }),
            m_edges.end());

        std::cout << "\tSorting " << m_edges.size() << "\n";
        std::sort(std::execution::par, m_edges.begin(), m_edges.end(), forwardOrder);
        m_edges.shrink_to_fit();

        //Create the reversed index vector.
        //These are 32bit indexes into the edges vector.
        //Modifying either vector invalidates everything.
        std::cout << "\tCopying reversed edges...\n";
        m_reversedEdgeIndices.resize(m_edges.size());
        std::iota(m_reversedEdgeIndices.begin(), m_reversedEdgeIndices.end(), 0);
        std::cout << "\tSorting reversed edges...\n";
        std::sort(std::execution::par, m_reversedEdgeIndices.begin(), m_reversedEdgeIndices.end(),
            [&edges = m_edges](uint32_t indLeft, uint32_t indRight) {
            const Edge& left = edges[indLeft];
            const Edge& right = edges[indRight];
            return left.Node2() < right.Node2();
        });
        std::cout << "Sort done.\n";
        m_reversedEdgeIndices.shrink_to_fit();
    }

    inline void beaches::Graph::getEdges(osmium::object_id_type nodeId, std::vector<Edge*>& incidentEdges)
    {
        incidentEdges.clear();

        //Get the forward edges.
        {
            auto iter = std::lower_bound(m_edges.begin(), m_edges.end(), nodeId,
                [](const Edge& pEdge, const osmium::object_id_type pId) {
                return pEdge.Node1() < pId;
            });
            for (; iter != m_edges.end() && iter->Node1() == nodeId; iter++) {
                if (!iter->hasFlag(EdgeFlags::Erased)) {
                    incidentEdges.push_back(&(*iter));
                }
            }
        }

        //Get the reversed edges.
        {
            auto iter = std::lower_bound(m_reversedEdgeIndices.begin(), m_reversedEdgeIndices.end(), nodeId,
                [&edges = m_edges](const uint32_t pIndex, const osmium::object_id_type pId) {
                return edges[pIndex].Node2() < pId;
            }
            );
            for (; iter != m_reversedEdgeIndices.end() && m_edges[*iter].Node2() == nodeId; iter++) {
                if (!(m_edges[*iter]).hasFlag(EdgeFlags::Erased)) {
                    incidentEdges.push_back(&m_edges[*iter]);
                }
            }
        }
    }

    inline void beaches::Graph::propagateCoastRing(Edge * startEdge, bool waterOnRight) noexcept
    {
        //Find the outer ring that fully contains the coastline.
        struct CandidateEdge {
            Edge* edge;
            float angle;
        };

        std::vector<Edge*> incidentEdges;
        incidentEdges.reserve(10);
        std::vector<CandidateEdge> candidates;
        candidates.reserve(10);

        Edge* curEdge = startEdge;
        bool done = false;

        //For the forward direction, must keep water on the right.
        osmium::object_id_type prevNode = (curEdge->hasFlag(EdgeFlags::WaterOnRight) || !waterOnRight) ? curEdge->Node2() : curEdge->Node1();

        //std::cout << "Path starting: " << prevNode << "\n";

        while (!done) {
            curEdge->setFlag(EdgeFlags::CoastOuterRing, true);
            osmium::object_id_type headNode = (prevNode == curEdge->Node1()) ? curEdge->Node2() : curEdge->Node1();
            //std::cout << "\t" << headNode << "\n";

            getEdges(headNode, incidentEdges);
            candidates.clear();

            //Node locations are only avilable for degree 2+ nodes and their neighbors.
            const bool computeAngles = incidentEdges.size() > 2;
            float curEdgeDirection = computeAngles ? getDirection(headNode, prevNode) : 0;

            for (Edge* e : incidentEdges) {
                if (e->hasFlag(EdgeFlags::CoastOuterRing))
                    continue; //already visited.

                osmium::object_id_type farNode = (e->Node1() == headNode) ? e->Node2() : e->Node1();

                candidates.push_back(
                    CandidateEdge{
                        e,
                        computeAngles ? getPositiveAngle(curEdgeDirection, getDirection(headNode, farNode)) : 0 //Direction is reversed.
                    }
                );
            }

            if (candidates.size() == 0) {
                done = true;
                continue;
            }

            auto elemIter = waterOnRight ?
                std::max_element(candidates.begin(), candidates.end(),
                    [](const CandidateEdge& left, const CandidateEdge& right) {return left.angle < right.angle; }) :
                std::min_element(candidates.begin(), candidates.end(),
                    [](const CandidateEdge& left, const CandidateEdge& right) {return left.angle < right.angle; });

            curEdge = elemIter->edge;

            prevNode = headNode;

        }
    }

    inline void Graph::propagateReachableWaterPath(Edge * startEdge, bool waterOnRight) noexcept
    {
        //Keep turning "inland", avoiding any edges that border water on both sides.
        struct CandidateEdge {
            Edge* edge;
            float angle;
        };

        std::vector<Edge*> incidentEdges;
        incidentEdges.reserve(10);
        std::vector<CandidateEdge> candidates;
        candidates.reserve(10);

        Edge* curEdge = startEdge;
        bool done = false;

        //For the forward direction, must keep water on the right.
        osmium::object_id_type prevNode = (curEdge->hasFlag(EdgeFlags::WaterOnRight) || !waterOnRight) ? curEdge->Node2() : curEdge->Node1();

        //std::cout << "Path starting: " << prevNode << "\n";

        while (!done) {
            curEdge->setFlag(EdgeFlags::ReachableFromSea, true);
            osmium::object_id_type headNode = (prevNode == curEdge->Node1()) ? curEdge->Node2() : curEdge->Node1();
            //std::cout << headNode << " ";

            getEdges(headNode, incidentEdges);
            candidates.clear();

            bool computeAngles = incidentEdges.size() > 2;
            float curEdgeDirection = computeAngles ? getDirection(headNode, prevNode) : 0; //Direction is reversed. (Where we came from.)

            for (Edge* e : incidentEdges) {
                if (e->hasFlag(EdgeFlags::ReachableFromSea))
                    continue; //already visited.

                if (e->hasFlag(EdgeFlags::CoastOuterRing) && e->hasFlag(EdgeFlags::OuterRing)) {
                    continue; //borders water on both sides.
                }

                osmium::object_id_type farNode = (e->Node1() == headNode) ? e->Node2() : e->Node1();

                candidates.push_back(
                    CandidateEdge{
                        e,
                        computeAngles ? getPositiveAngle(curEdgeDirection, getDirection(headNode, farNode)) : 0 //Direction is reversed.
                    }
                );
            }

            if (candidates.size() == 0) {
                done = true;
                continue;
            }

            auto elemIter = waterOnRight ?
                std::min_element(candidates.begin(), candidates.end(),
                    [](const CandidateEdge& left, const CandidateEdge& right) {return left.angle < right.angle; }) :
                std::max_element(candidates.begin(), candidates.end(),
                    [](const CandidateEdge& left, const CandidateEdge& right) {return left.angle < right.angle; });

            curEdge = elemIter->edge;

            prevNode = headNode;
        }

        //std::cout << "\n";
    }

    inline void beaches::Graph::findCoastlineRing()
    {
        //Find the outer ring that fully contains the coastline.
        //Any edges in this ring that also border a water polygon are water-water intersections.

        for (auto& outerEdge : m_edges) {
            if (outerEdge.hasFlag(EdgeFlags::CoastOuterRing)) {
                //already visited.
                continue;
            }

            if ((outerEdge.hasFlag(EdgeFlags::WaterOnLeft) || outerEdge.hasFlag(EdgeFlags::WaterOnRight)) &&
                !(outerEdge.hasFlag(EdgeFlags::InnerRing) || outerEdge.hasFlag(EdgeFlags::OuterRing))) {
                //This edge has water on a known side and the other side (probably) doesn't border water.
                //We can start path propagation from here.
                Edge* curEdge = &outerEdge;

                propagateCoastRing(curEdge, true);
                propagateCoastRing(curEdge, false);
            }
        }
    }

    inline void Graph::findReachableWater()
    {
        //Find "left-turn" paths beginning with edges that are:
        // - in the coast outer ring
        // - do not border a water polygon.

        for (auto& outerEdge : m_edges) {
            if (outerEdge.hasFlag(EdgeFlags::ReachableFromSea)) {
                //already visited.
                continue;
            }

            if (outerEdge.hasFlag(EdgeFlags::CoastOuterRing) && !outerEdge.hasFlag(EdgeFlags::OuterRing)) {
                //This edge is in the outer coast ring and does not border the outside of a water polygon.
                //We can start a path from here.
                Edge* curEdge = &outerEdge;

                propagateReachableWaterPath(curEdge, true);
                propagateReachableWaterPath(curEdge, false);
            }
        }
    }

    //inline void Graph::populateGrid(GridGeomAggregator & grid, size_t edgeStart, size_t edgeEnd) const
    //{
    //	//auto grid = GridGeomAggregator(1000, 1000, -180.0, -90.0, 180.0, 90.0);
    //	std::cout << "Adding edges to grid...\n";
    //	for (size_t i = edgeStart; i < edgeEnd; i++) {
    //		const auto& edge = m_edges[i];
    //		if (!(edge.hasFlag(EdgeFlags::ReachableFromSea)))
    //			continue;

    //		OGRLineString line;
    //		const auto& node1 = m_nodeManager.GetNode(edge.Node1());
    //		const auto& node2 = m_nodeManager.GetNode(edge.Node2());
    //		line.addPoint(node1.Lon(), node1.Lat());
    //		line.addPoint(node2.Lon(), node2.Lat());
    //		OGRGeometry* bufferedLine = line.Buffer(.01, 4);
    //		grid.Add(*bufferedLine);
    //		OGRGeometryFactory::destroyGeometry(bufferedLine);
    //	}

    //	std::cout << "Consolidating " << edgeStart << "\n";
    //	grid.ConsoldiateCells();
    //	std::cout << "Thread done. " << edgeStart << "\n";
    //}

    /*inline void Graph::exportGrid(const char * fileName) const noexcept
    {
        auto startTime = std::chrono::high_resolution_clock::now();

        long numberThreads = std::thread::hardware_concurrency() * 2;
        std::vector<std::thread> threads;
        std::vector<GridGeomAggregator> grids;
        std::vector<long> intervals;

        long stepSize = m_edges.size() / numberThreads;
        for (int i = 0; i < numberThreads; i++) {
            grids.emplace_back(1000, 1000, -180.0, -90.0, 180.0, 90.0);
            intervals.push_back(i * stepSize);
        }
        intervals.push_back(m_edges.size());

        threads.reserve(numberThreads);
        for (int i = 0; i < numberThreads; i++) {
            std::thread t(&Graph::populateGrid, this, std::ref(grids[i]), intervals[i], intervals[i + 1]);
            threads.push_back(std::move(t));
        }

        std::for_each(threads.begin(), threads.end(), [](std::thread& t) { t.join(); });

        for (int i = 1; i < grids.size(); i++) {
            std::cout << "Merging " << i << "\n";
            grids[0].AddAll(grids[i]);
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        std::cout << "Grid export took " << std::chrono::duration_cast<std::chrono::minutes>(endTime - startTime).count() << " minutes.\n";

        std::cout << "Exporting grid...\n";
        grids[0].ConsoldiateCells();
        grids[0].Export(fileName);
    }*/

    inline void beaches::Graph::exportBufferedLinestrings(const char* osmFile, const std::vector<double>& bufferSizes, float padding, int rows, int columns, std::string fileName) noexcept
    {
        //Dump all the memory we won't need.
        //Remove all the edges that are not reachable. We don't want to search through these 
        //or track their node locations.
        m_edges.erase(
            std::remove_if(m_edges.begin(), m_edges.end(), [](const Edge& pEdge) {
            return !pEdge.hasFlag(EdgeFlags::ReachableFromSea); }),
            m_edges.end());
        initalizeEdgeVectors();

        //Only load locations for nodes that we'll need for the buffer.
        populateUsedNodes(osmFile); 

        std::cout << "Extracting rings...\n";

        //Translations to modulo the geom across +/- 180 degrees longitude. (Alaska, Siberia, Fiji, etc.)
        std::vector<std::tuple<double, double>> offsets{ {-360.0, 0.0}, {0.0, 0.0}, {360.0, 0} };

        for (const double bufferDistance : bufferSizes) {
            std::cout << "\nBuffer size: " << bufferDistance << "\n";
            std::for_each(m_edges.begin(), m_edges.end(), [](Edge& pEdge) { pEdge.setFlag(EdgeFlags::Visited, false); });
            //4326 coordinates.
            auto aggregator = GridGeomAggregator(rows, columns, -180.0f - padding, -90.0f, 180.0f + padding, 90.0f);

            auto insertLinestringFunction = [&nodeManager = m_nodeManager, &aggregator, bufferDistance](const std::vector<osmium::object_id_type>& thePath, const double dx, const double dy) {
                if (thePath.size() < 2) {
                    std::cout << "Degenerate line.\n";
                    return; //Not a line.
                }

                OGRLineString* lineString = static_cast<OGRLineString*>(OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbLineString));

                std::for_each(thePath.begin(), thePath.end(),
                    [&line = lineString, &nodes = nodeManager, dx, dy](const osmium::object_id_type nodeId) {
                    const auto node = nodes.GetNode(nodeId);
                    line->addPoint(node.Lon() + dx, node.Lat() + dy);
                });

                OGRGeometry* simplifiedLine = lineString->Simplify(.001);
                OGRGeometryFactory::destroyGeometry(lineString);
                OGRPolygon* bufferedLine = static_cast<OGRPolygon*>(simplifiedLine->Buffer(bufferDistance, 4));
                OGRGeometryFactory::destroyGeometry(simplifiedLine);

                aggregator.Add(*bufferedLine);
                OGRGeometryFactory::destroyGeometry(bufferedLine);
            };

            auto insertPathsFunc = [&insertLinestringFunction, &offsets, &aggregator](std::vector<std::vector<osmium::object_id_type>>& paths) {
                std::cout << "_";
                std::for_each(std::execution::par, paths.begin(), paths.end(),
                    [&insertLinestringFunction, &offsets](const std::vector<osmium::object_id_type>& thePath)
                {
                    std::for_each(offsets.begin(), offsets.end(), [&thePath, &insertLinestringFunction](const std::tuple<double, double>& translation) {
                        auto[dx, dy] = translation;
                        insertLinestringFunction(thePath, dx, dy);
                    });
                });

                paths.clear();

                std::cout << "x";
                aggregator.ConsoldiateCells();
                std::cout << "|";
            };

            {
                std::vector<Edge*> incidentEdges;
                std::vector<std::vector<osmium::object_id_type>> paths;
                std::vector<osmium::object_id_type> pathForward;
                for (auto& outerEdge : m_edges) {
                    if (outerEdge.hasFlag(EdgeFlags::ReachableFromSea) && !outerEdge.hasFlag(EdgeFlags::Visited)) {
                        //Start a path (hopefully a ring) from here.

                        osmium::object_id_type headNode = outerEdge.Node2();
                        Edge* nextEdge = &outerEdge;

                        while (true) {
                            pathForward.push_back(headNode);
                            nextEdge->setFlag(EdgeFlags::Visited, true);

                            incidentEdges.clear();
                            getEdges(headNode, incidentEdges);

                            auto candidateIter = std::find_if(incidentEdges.begin(), incidentEdges.end(),
                                [](const Edge* pEdge) {
                                return pEdge->hasFlag(EdgeFlags::Visited) == false &&
                                    pEdge->hasFlag(EdgeFlags::ReachableFromSea) == true;
                            });

                            if (candidateIter == incidentEdges.end()) {
                                break; //break out of the while loop.
                            }
                            else {
                                nextEdge = (*candidateIter);
                                headNode = nextEdge->Node1() == headNode ? nextEdge->Node2() : nextEdge->Node1();
                            }
                        }

                        if (pathForward.size() >= 2) {

                            //Push back a copy.
                            //Split the path to some maximum size for performance.
                            size_t indexStart = 0;
                            size_t indexEnd = 0;
                            while (indexStart < pathForward.size()) {
                                indexEnd = std::min(indexStart + 1000, pathForward.size());
                                if (indexEnd == pathForward.size() - 1) {
                                    indexEnd++; //don't orphan the final point.
                                }

                                paths.emplace_back(pathForward.begin() + indexStart, pathForward.begin() + indexEnd);
                                indexStart = indexEnd;
                            }
                            if (paths.size() >= 50'000) {
                                insertPathsFunc(paths);
                            }
                        }
                        pathForward.clear();
                    }
                }

                insertPathsFunc(paths);
            } //scope for paths vector.

            std::cout << "Exporting...\n";
            aggregator.Export((fileName + "_" + std::to_string(bufferDistance) + ".shp").c_str());
        }
    }

    inline void beaches::Graph::exportEdges(const char* osmFile, std::string fileName) noexcept
    {
        //Track all of the nodes! This could be a bunch.
        populateUsedNodes(osmFile);

        const char *pszDriverName = "ESRI Shapefile";
        GDALDriver *poDriver;
        GDALAllRegister();
        poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
        if (poDriver == NULL)
        {
            printf("%s driver not available.\n", pszDriverName);
            exit(1);
        }
        GDALDataset *poDS = poDriver->Create(fileName.c_str(), 0, 0, 0, GDT_Unknown, NULL);
        if (poDS == NULL)
        {
            printf("Creation of output file failed.\n");
            exit(1);
        }
        OGRLayer *m_poLayer = poDS->CreateLayer("paths", NULL, wkbLineString, NULL);
        if (m_poLayer == NULL)
        {
            printf("Layer creation failed.\n");
            exit(1);
        }
        {
            OGRFieldDefn oField("node1_id", OGRFieldType::OFTInteger64);
            if (m_poLayer->CreateField(&oField) != OGRERR_NONE)
            {
                printf("Creating 'node_1' field failed.\n");
                exit(1);
            }
        }
        {
            OGRFieldDefn oField("node2_id", OGRFieldType::OFTInteger64);
            if (m_poLayer->CreateField(&oField) != OGRERR_NONE)
            {
                printf("Creating 'node_2' field failed.\n");
                exit(1);
            }
        }
        {
            OGRFieldDefn oField("coast_ring", OGRFieldType::OFTInteger);
            if (m_poLayer->CreateField(&oField) != OGRERR_NONE)
            {
                printf("Creating 'coast_ring' field failed.\n");
                exit(1);
            }
        }
        {
            OGRFieldDefn oField("reachable", OGRFieldType::OFTInteger);
            if (m_poLayer->CreateField(&oField) != OGRERR_NONE)
            {
                printf("Creating 'reachable' field failed.\n");
                exit(1);
            }
        }

        for (long i = 0; i < m_edges.size(); i++) {
            const Edge& edge = m_edges[i];

            OGRFeature *poFeature;
            poFeature = OGRFeature::CreateFeature(m_poLayer->GetLayerDefn());
            poFeature->SetField("node1_id", edge.Node1());
            poFeature->SetField("node2_id", edge.Node2());
            poFeature->SetField("coast_ring", edge.hasFlag(EdgeFlags::CoastOuterRing));
            poFeature->SetField("reachable", edge.hasFlag(EdgeFlags::ReachableFromSea));


            OGRLineString line;
            const auto& node1 = m_nodeManager.GetNode(edge.Node1());
            const auto& node2 = m_nodeManager.GetNode(edge.Node2());
            line.addPoint(node1.Lon(), node1.Lat());
            line.addPoint(node2.Lon(), node2.Lat());
            poFeature->SetGeometry(&line);
            if (m_poLayer->CreateFeature(poFeature) != OGRERR_NONE)
            {
                printf("Failed to create feature in shapefile.\n");
                exit(1);
            }
            OGRFeature::DestroyFeature(poFeature);
        }

        if (poDS) {
            GDALClose(poDS);
        }
    }
}

#endif // !BEACHES_GRAPH_H
