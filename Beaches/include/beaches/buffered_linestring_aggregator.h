#pragma once
#ifndef BEACHES_BUFFERED_LINESTRING_AGGREGATOR_H
#define BEACHES_BUFFERED_LINESTRING_AGGREGATOR_H

#include "ogr_geometry.h"
#include <vector>
#include <algorithm>
#include <memory>
#include <assert.h>

#include "ogr_feature.h"
#include "ogrsf_frmts.h"
#include "gdal_priv.h"

namespace beaches {
	class BufferedLinestringAggregator {
	public:
		explicit BufferedLinestringAggregator(
			double simplificationTolerance, double bufferDist, int quadSegments=4);
		~BufferedLinestringAggregator();
		void Add(OGRLineString& lineString);
		void Export(const char* fileName) const;
		void ExportAsGrid(const char* fileName, double minX, double minY, double maxX, double maxY, int rows, int columns, int colPadding) const;
	private:
		
	private:
		const double m_simplificationTolerance;
		const double m_bufferDist;
		const int m_quadSegs;
		OGRMultiPolygon* m_geom;
	};

	inline BufferedLinestringAggregator::BufferedLinestringAggregator(double simplificationTolerance, double bufferDist, int quadSegments) : 
		m_simplificationTolerance{simplificationTolerance}, 
		m_bufferDist{ bufferDist }, 
		m_quadSegs{quadSegments}
	{
		m_geom = static_cast<OGRMultiPolygon*>(OGRGeometryFactory::createGeometry(OGRwkbGeometryType::wkbMultiPolygon));
	}

	inline BufferedLinestringAggregator::~BufferedLinestringAggregator()
	{
		OGRGeometryFactory::destroyGeometry(m_geom);
	}

	inline void BufferedLinestringAggregator::Add(OGRLineString & lineString)
	{
		//PreserveTopology? Performance hit?
		std::cout << "LineString: Points: " << lineString.getNumPoints() << "\n";

		OGRGeometry* simplified = lineString.SimplifyPreserveTopology(m_simplificationTolerance);
		OGRGeometry* buffered = simplified->Buffer(m_bufferDist, m_quadSegs);
		m_geom->addGeometryDirectly(buffered); //assumes ownership
		m_geom->addGeometryDirectly(simplified);
	}

	inline void BufferedLinestringAggregator::Export(const char * fileName) const
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

		for (const auto poly : m_geom) {
			//	continue;

			OGRFeature *poFeature;
			poFeature = OGRFeature::CreateFeature(m_poLayer->GetLayerDefn());
			poFeature->SetGeometry(poly);
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

	inline void BufferedLinestringAggregator::ExportAsGrid(const char * fileName, double minX, double minY, double maxX, double maxY, int numRows, int numColumns, int colPadding) const
	{
		double gridWidth = maxX - minX;
		double gridHeight = maxY - minY;

		double cellWidth = gridWidth / numColumns;
		double cellHeight = gridHeight / numRows;

		int minCol = 0 - colPadding;
		int maxCol = (numColumns - 1) + colPadding;

		OGRSpatialReference refZero;
		refZero.SetTOWGS84(0.0, 0.0, 0.0);

		for (int row = 0; row < numRows; row++) {
			for (int col = minCol; col <= maxCol; col++) {

				int noramlizedCol = std::abs(col) % numColumns;

				double cellMinX = minX + (cellWidth * col);
				double noramlizedCellMinX = minX + (cellWidth * noramlizedCol);
				double cellMinY = minY + (cellHeight * row);



			}
		}


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

		for (const auto poly : m_geom) {
			//	continue;

			OGRFeature *poFeature;
			poFeature = OGRFeature::CreateFeature(m_poLayer->GetLayerDefn());
			poFeature->SetGeometry(poly);
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



#endif // !BEACHES_BUFFERED_LINESTRING_AGGREGATOR_H

