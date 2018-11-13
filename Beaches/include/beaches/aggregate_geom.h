#pragma once
#ifndef BEACHES_AGGREGATE_GEOM_H
#define BEACHES_AGGREGATE_GEOM_H

#include "ogr_geometry.h"
#include <vector>
#include <algorithm>
#include <memory>
#include <assert.h>
#include <iostream>
#include <numeric>
#include <execution>

#include "ogr_feature.h"
#include "ogrsf_frmts.h"
#include "gdal_priv.h"

namespace beaches {
	class AggregateGeom {
	public:
		explicit AggregateGeom(
			double minX, double minY, double maxX, double maxY
		);
		~AggregateGeom();

		//Ownership of polygon is not assumed.
		void Add(OGRPolygon* polygon);
		void ExportAsGrid(const char* fileName, double rows, double cols);
	private:
	private:
		const double m_minX;
		const double m_minY;
		const double m_maxX;
		const double m_maxY;
		OGRPolygon* m_clippingRegion;
		OGRMultiPolygon* m_unionGeom;
		std::mutex m_unionGeomLock;
	};

	inline AggregateGeom::AggregateGeom(double minX, double minY, double maxX, double maxY) :
		m_minX{ minX }, m_minY{ minY }, m_maxX{ maxX }, m_maxY{ maxY }
	{
		OGRLinearRing* boundaryRing = (OGRLinearRing*)OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbLinearRing);
		boundaryRing->addPoint(minX, minY);
		boundaryRing->addPoint(minX, maxY);
		boundaryRing->addPoint(maxX, maxY);
		boundaryRing->addPoint(maxX, minY);
		boundaryRing->closeRings();

		m_clippingRegion = static_cast<OGRPolygon*>(OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbPolygon));
		m_clippingRegion->addRing(boundaryRing); //assumes ownership of ring.

		m_unionGeom = static_cast<OGRMultiPolygon*>(OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbMultiPolygon));
	}

	inline AggregateGeom::~AggregateGeom()
	{
		OGRGeometryFactory::destroyGeometry(m_clippingRegion);
		OGRGeometryFactory::destroyGeometry(m_unionGeom);
	}

	inline void AggregateGeom::Add(OGRPolygon * polygon)
	{
		OGRGeometry* intersection = polygon->Intersection(m_clippingRegion);
		if (intersection->IsEmpty()) {
			OGRGeometryFactory::destroyGeometry(intersection);
		}
		else {
			std::unique_lock insertLock(m_unionGeomLock);
			m_unionGeom->addGeometryDirectly(intersection);
		}
	}

	inline void AggregateGeom::ExportAsGrid(const char * fileName, double rows, double cols)
	{
		std::cout << "Full intersection...\n";
		auto fullGeom = m_unionGeom->UnionCascaded();
		//TODO: Simplify?
		std::cout << "Done.\n";

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
		OGRLayer *poLayer = poDS->CreateLayer("paths", NULL, wkbMultiPolygon, NULL);
		if (poLayer == NULL)
		{
			printf("Layer creation failed.\n");
			exit(1);
		}
		{
			OGRFieldDefn oField("row", OGRFieldType::OFTInteger);
			if (poLayer->CreateField(&oField) != OGRERR_NONE)
			{
				printf("Creating Degree field failed.\n");
				exit(1);
			}
		}
		{
			OGRFieldDefn oField("col", OGRFieldType::OFTInteger);
			if (poLayer->CreateField(&oField) != OGRERR_NONE)
			{
				printf("Creating Degree field failed.\n");
				exit(1);
			}
		}

		double gridWidth = m_maxX - m_minX;
		double gridHeight = m_maxY - m_minY;
		double cellWidth = gridWidth / cols;
		double cellHeight = gridHeight / rows;

		std::mutex writeLock;
		auto procesCellLambda = [this, cellWidth, cellHeight, &poLayer, &writeLock, &fullGeom](int iRow, int iCol) {			
			//Find the cell boundary.
			double minX = m_minX + (iCol * cellWidth);
			double minY = m_minY + (iRow * cellHeight);
			double maxX = minX + cellWidth;
			double maxY = minY + cellHeight;

			//Build up the clipping geom for this cell.
			OGRLinearRing* boundaryRing = (OGRLinearRing*)OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbLinearRing);
			boundaryRing->addPoint(minX, minY);
			boundaryRing->addPoint(minX, maxY);
			boundaryRing->addPoint(maxX, maxY);
			boundaryRing->addPoint(maxX, minY);
			boundaryRing->closeRings();
			OGRPolygon* cellGeom = static_cast<OGRPolygon*>(OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbPolygon));
			cellGeom->addRing(boundaryRing); //assumes ownership of ring.

			OGRGeometry* cellIntersection = fullGeom->Intersection(cellGeom);
			OGRGeometryFactory::destroyGeometry(cellGeom);

			{
				//Scope for the multiPoly since it will be invalid after this block.
				OGRMultiPolygon* multiPoly = dynamic_cast<OGRMultiPolygon*>(cellIntersection);
				if (multiPoly != nullptr) {
					auto unioned = multiPoly->UnionCascaded();
					OGRGeometryFactory::destroyGeometry(cellIntersection);
					cellIntersection = unioned;
				}
			}

			if (cellIntersection == nullptr) {
				return;
			}

			if (cellIntersection->IsEmpty()) {
				//std::cout << "x";
				OGRGeometryFactory::destroyGeometry(cellIntersection);
				return;
			}

			OGRFeature *poFeature;
			poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
			poFeature->SetField("row", iRow);
			poFeature->SetField("col", iCol);

			std::cout << ".";

			poFeature->SetGeometry(cellIntersection);

			{ //scope for the lock.
				std::unique_lock lockObj(writeLock); //take the lock.
				if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
				{
					printf("Failed to create feature in shapefile.\n");
					exit(1);
				}
			}
			OGRFeature::DestroyFeature(poFeature);
		};

		//Some column numbers we can iterate over for parallel execution.
		std::vector<int> columnNumbers(cols);
		std::iota(columnNumbers.begin(), columnNumbers.end(), 0);

		for (int iRow = 0; iRow < rows; iRow++) {
			std::cout << "Row " << iRow << "\n";
			std::for_each(std::execution::par, columnNumbers.begin(), columnNumbers.end(), 
				[&procesCellLambda, iRow](const int iCol) { return procesCellLambda(iRow, iCol); });
		}

		OGRGeometryFactory::destroyGeometry(fullGeom);

		if (poDS) {
			GDALClose(poDS);
		}
	}
}

#endif // !BEACHES_AGGREGATE_GEOM_H
