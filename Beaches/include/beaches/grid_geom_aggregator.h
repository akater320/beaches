#pragma once
#ifndef COASTLINE_GRID_GEOM_AGGREGATOR_H
#define COASTLINE_GRID_GEOM_AGGREGATOR_H

#include "ogr_geometry.h"
#include <vector>
#include <algorithm>
#include <memory>
#include <assert.h>

#include "ogr_feature.h"
#include "ogrsf_frmts.h"
#include "gdal_priv.h"

namespace beaches {
	namespace {
		struct GridArea {
			int minRow;
			int minCol;
			int maxRow;
			int maxCol;
		};
	}

	class GridGeomAggregator {
	public:
		GridGeomAggregator(int rows, int columns, float minX, float minY, float maxX, float maxY);
		~GridGeomAggregator();
		void Add(OGRGeometry& geom);
		void Export(const char* fileName) const;
		void ConsoldiateCells();
		void AddAll(const GridGeomAggregator& other);
	private:
		void consolidateCell(int row, int col);
		GridArea overlappingCells(const OGREnvelope& envelope) const;
		OGREnvelope cellBoundary(int row, int col) const;
	private:
		std::mutex m_writeLock;
		int m_rows;
		int m_cols;
		float m_minX;
		float m_minY;
		float m_maxX;
		float m_maxY;
		double m_cellWidth;
		double m_cellHeight;
		std::vector<std::vector<OGRGeometry*>> m_geomRows;
	};

	inline beaches::GridGeomAggregator::GridGeomAggregator(int rows, int columns, float minX, float minY, float maxX, float maxY)
		: m_minX{ minX }, m_minY{ minY }, m_maxX{ maxX }, m_maxY{ maxY }, m_rows{ rows }, m_cols{ columns }
	{
		m_cellWidth = (maxX - minX) / m_cols;
		m_cellHeight = (maxY - minY) / m_rows;
		//Lay out the row and cols.
		//Lots of geoms could be null, so these are lazy-loaded.
		m_geomRows.resize(rows);
		std::for_each(m_geomRows.begin(), m_geomRows.end(), [columns](std::vector<OGRGeometry*>& row) { row.resize(columns); });
	}

	inline beaches::GridGeomAggregator::~GridGeomAggregator()
	{
		//Destroy any valid geoms.
		for (const auto& row : m_geomRows) {
			for (OGRGeometry* ptr : row) {
				if (ptr) {
					OGRGeometryFactory::destroyGeometry(ptr);
				}
			}
		}
	}

	inline void beaches::GridGeomAggregator::Add(OGRGeometry & geom)
	{
		auto envelope = OGREnvelope{};
		geom.getEnvelope(&envelope);
		auto subGrid = overlappingCells(envelope);

		for (int row = subGrid.minRow; row < subGrid.maxRow; row++) {
			auto& rowVector = m_geomRows[row];
			for (int col = subGrid.minCol; col < subGrid.maxCol; col++) {

				//Clip the geom to the boundary of the cell.
				OGREnvelope cellEnvelope = cellBoundary(row, col);
				OGRLinearRing* boundaryRing = (OGRLinearRing*)OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbLinearRing);
				boundaryRing->addPoint(cellEnvelope.MinX, cellEnvelope.MinY);
				boundaryRing->addPoint(cellEnvelope.MaxX, cellEnvelope.MinY);
				boundaryRing->addPoint(cellEnvelope.MaxX, cellEnvelope.MaxY);
				boundaryRing->addPoint(cellEnvelope.MinX, cellEnvelope.MaxY);
				boundaryRing->closeRings();

				OGRPolygon* cellBoundary = (OGRPolygon*)OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbPolygon);
				cellBoundary->addRing(boundaryRing); //assumes ownership of ring.

				/*OGRGeometry* clippedGeom = cellBoundary->Intersection(&geom);*/
				OGRGeometry* clippedGeom = geom.Intersection(cellBoundary);

				if (clippedGeom->IsEmpty()) {
					//The rough intersection is done with the envelope (AABB).
					//The actual intersection can be empty.

					OGRGeometryFactory::destroyGeometry(cellBoundary);
					OGRGeometryFactory::destroyGeometry(clippedGeom);

					continue;
				}

				{
					std::unique_lock lock(m_writeLock);
					if (!rowVector[col]) {
						rowVector[col] = OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbMultiPolygon);
					}
					OGRMultiPolygon* aggregateGeom = ((OGRMultiPolygon*)rowVector[col]);

					OGRGeometryCollection* clippedMultiPoly = dynamic_cast<OGRGeometryCollection*>(clippedGeom);
					if (clippedMultiPoly != nullptr) {
						for (const auto child : clippedMultiPoly) {
							aggregateGeom->addGeometry(child);
						}
						OGRGeometryFactory::destroyGeometry(clippedGeom);
					}
					else {
						aggregateGeom->addGeometryDirectly(clippedGeom);
					}
				}
				OGRGeometryFactory::destroyGeometry(cellBoundary);

				//int numGeoms = ((OGRMultiPolygon*)rowVector[col])->getNumGeometries();
				//if (numGeoms >= 100) {
				//	consolidateCell(row, col);
				//}
			}
		}
	}

	inline void beaches::GridGeomAggregator::Export(const char * fileName) const
	{
		const char *pszDriverName = "ESRI Shapefile";
		GDALDriver *poDriver;
		GDALAllRegister();
		poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
		if (poDriver == NULL)
		{
			printf("%s driver not available.\n", pszDriverName);
			exit(1);
		}
		GDALDataset *poDS = poDriver->Create(fileName, 0, 0, 0, GDT_Unknown, NULL);
		if (poDS == NULL)
		{
			printf("Creation of output file failed.\n");
			exit(1);
		}
		OGRLayer *m_poLayer = poDS->CreateLayer("paths", NULL, wkbPolygon, NULL);
		if (m_poLayer == NULL)
		{
			printf("Layer creation failed.\n");
			exit(1);
		}
		{
			OGRFieldDefn oField("row", OGRFieldType::OFTInteger);
			if (m_poLayer->CreateField(&oField) != OGRERR_NONE)
			{
				printf("Creating Degree field failed.\n");
				exit(1);
			}
		}
		{
			OGRFieldDefn oField("col", OGRFieldType::OFTInteger);
			if (m_poLayer->CreateField(&oField) != OGRERR_NONE)
			{
				printf("Creating Degree field failed.\n");
				exit(1);
			}
		}


		//TODO: Make Parallel.
		for (int iRow = 0; iRow < m_geomRows.size(); iRow++) {
			const auto& row = m_geomRows[iRow];
			for (int iCol = 0; iCol < row.size(); iCol++) {
				const OGRGeometry* ptr = row[iCol];
				if (ptr) {

					OGRFeature *poFeature;
					poFeature = OGRFeature::CreateFeature(m_poLayer->GetLayerDefn());
					poFeature->SetField("row", iRow);
					poFeature->SetField("col", iCol);

					std::cout << ".";

					poFeature->SetGeometry(ptr);

					if (m_poLayer->CreateFeature(poFeature) != OGRERR_NONE)
					{
						printf("Failed to create feature in shapefile.\n");
						exit(1);
					}
					OGRFeature::DestroyFeature(poFeature);

				}
			}
		}

		if (poDS) {
			GDALClose(poDS);
		}
	}

	inline void GridGeomAggregator::ConsoldiateCells()
	{
		std::vector<long> rowIndices(m_geomRows.size());
		std::iota(rowIndices.begin(), rowIndices.end(), 0);

		std::for_each(std::execution::par, rowIndices.begin(), rowIndices.end(),
			[this](const long rowIndex) {
			const auto& row = m_geomRows[rowIndex];
			for (int iCol = 0; iCol < row.size(); iCol++) {
				consolidateCell(rowIndex, iCol);
			}
		});
	}

	inline void GridGeomAggregator::AddAll(const GridGeomAggregator & other)
	{
		assert(this->m_rows == other.m_rows);
		assert(this->m_cols == other.m_cols);

		int i = 0;

		for (int iRow = 0; iRow < this->m_geomRows.size(); iRow++) {
			auto& row1 = this->m_geomRows[iRow];
			auto& row2 = other.m_geomRows[iRow];
			for (int iCol = 0; iCol < row1.size(); iCol++) {
				OGRGeometry* geom1 = row1[iCol];
				OGRGeometry* geom2 = row2[iCol];

				if (geom1 == nullptr) {
					if (geom2 != nullptr) {
						row1[iCol] = geom2->clone();
						i++;
					}
				}
				else {
					if (geom2 != nullptr) {
						OGRGeometryCollection* poly2 = (OGRGeometryCollection*)geom2;
						//GDAL 2.3 introduced iterators.
						int numChildren = poly2->getNumGeometries();
						for (int i = 0; i < numChildren; i++) {
							OGRGeometry* child = poly2->getGeometryRef(i);
							OGRErr err = ((OGRGeometryCollection*)geom1)->addGeometry(child); //clone and add.
							if (err) {
								std::cout << "Error adding geom: " << err << "\n";
							}
						}
						i++;
					}
				}
			}
		}

		std::cout << "\tAdded " << i << " geoms.\n";
	}

	inline void GridGeomAggregator::consolidateCell(int row, int col)
	{
		OGRGeometry* oldGeom = m_geomRows[row][col];
		if (!oldGeom) {
			return;
		}

		int numGeoms = ((OGRMultiPolygon*)oldGeom)->getNumGeometries();
		if (numGeoms < 2) {
			return;
		}
		//std::cout << "Consolidating " << numGeoms << " geometries...";
		OGRGeometry* unionGeom = oldGeom->UnionCascaded();

		//If all the geometries touch then the union will be a polygon. 
		//We need to change this back to a multipolygon in order to add more geometries to it.
		if (unionGeom->getGeometryType() != OGRwkbGeometryType::wkbMultiPolygon) {
			OGRGeometry* child = unionGeom;
			unionGeom = OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbMultiPolygon);
			((OGRMultiPolygon*)unionGeom)->addGeometryDirectly(child); //ownership assumed
		}

		m_geomRows[row][col] = unionGeom;
		OGRGeometryFactory::destroyGeometry(oldGeom);
		//std::cout << "Done. Now: " << ((OGRMultiPolygon*)m_geomRows[row][col])->getNumGeometries() << "\n";
	}

	inline GridArea GridGeomAggregator::overlappingCells(const OGREnvelope & envelope) const
	{
		int minCol = static_cast<int>(std::floor((envelope.MinX - m_minX) / m_cellWidth));
		int maxCol = static_cast<int>(std::ceil((envelope.MaxX - m_minX) / m_cellWidth));
		int minRow = static_cast<int>(std::floor((envelope.MinY - m_minY) / m_cellHeight));
		int maxRow = static_cast<int>(std::ceil((envelope.MaxY - m_minY) / m_cellHeight));

		GridArea subGrid;
		subGrid.minRow = std::max(0, minRow);
		subGrid.maxRow = std::min(m_rows - 1, maxRow);
		subGrid.minCol = std::max(0, minCol);
		subGrid.maxCol = std::min(m_cols - 1, maxCol);

		return subGrid;
	}

	inline OGREnvelope GridGeomAggregator::cellBoundary(int row, int col) const
	{
		double minX = m_minX + m_cellWidth * col;
		double minY = m_minY + m_cellHeight * row;

		OGREnvelope envelope;
		envelope.MinX = minX;
		envelope.MinY = minY;
		envelope.MaxX = minX + m_cellWidth;
		envelope.MaxY = minY + m_cellHeight;

		return envelope;
	}
}

#endif // !COASTLINE_GRID_GEOM_AGGREGATOR_H
