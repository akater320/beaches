# Beaches

Beaches produces a set of "coastline" vectors from an OSM file. Rather than directly extracting rings of ways labeled with "natural"="coastline", Beaches makes an effort to remove edges that are bordered by water on two sides. This allows the coastline to extend into bays and larger rivers that may be navigable.  

Optionally, the vectors can be buffered and a polygon representing litoral zones can be exported.

## Building
To build beaches you'll need a C++17 compiler that supports parallel stl. Specifically you'll need the [`<execution>`](https://en.cppreference.com/w/cpp/header/execution) header. MSVC 2017 (Wine) should work. Check your compiler for compatibility here. https://godbolt.org/z/wJdoYc

Additionally you'll need GDAL 2.3 or higher. The easiest way to get this on windows is to install QGIS or use the OSGEO4W installer. This also works on Linux. Note: The versions of GDAL included in many Linux package managers are quite dated. 
Recent versions of GDAL are also available from Conda.

### Windows
You'll need Visual Studio or Visual Studio Build Tools. If you've installed something with native package support - e.g. Node.js or Python3 - you may already have the build tools.

To build on the command line, open the `x64 Native Tools Command Prompt` and type:
```
set GDAL_INCLUDE=/path/to/gdal_and_org/headersset 
set GDAL_LIB=/path/to/gdal/libs
MSBuild /t:Rebuild /p:Configuration=Release
```

## Running
Example - create 2 sizes of buffered masks:  
`Beaches /osm/planet.osm.pbf -b .001 -b .005 /output/coast_mask`  
For all options run:  
`Beaches -h`  
For debug level output:  
`Beaches /osm/small_extract.osm.pbf -d /output/debug_vectors.shp`

Beaches is designed to be fast and memory-friendly but planet.osm.pbf is huge. For reference, processing the current 44.3GB planet.osm.pbf and outputting a single coast mask takes about 45 minutes on a Core i7-6700. The time is dominated by decoding/decompressing the protobuf format then by finding the intersection of polygons. Fortunately both of these processes parallelize well and more cores should improve run times.

## OSM files
The planet.osm.pbf is avialable form https://planet.openstreetmap.org/. It usually includes changes made more than 5 days ago.  
Extracts are available from http://download.geofabrik.de/. These are usally updated every day. The country-scale and smaller extracts can be very useful when debugging as continet-scale files can quicly overflow the limits of the shapefiles that Beaches outputs.  

## About the algorithm  
In OSM, coastline is labeled with the tag "natural"="coastline". This classification can somewhat arbitrary - e.g. some river-banks are considered coastline while other, similar ones are not - but provides a good starting point. Ways labeled as "natural"="coastline" have the convenient property that they will always have on their right. (It should not be assumed that land is on the left.) Beaches begins with the OSM shoreline then adds all polygons that are connected to the sea/ocean. Unlike other tools that deal with OSM coastlines, beaches internally uses vectors rather than polygons. This allows edges that are bordered by water on both sides to be discarded.  

One of the major challenges for the algorithm is that polygons in OSM are primarily used for rendering basemaps. Overlapping polygons look fine when rendered. ([osmcoastlines](https://github.com/osmcode/osmcoastline) even inflates polygons so that they do overlap to improve rendering quality.) When OSM contributors are editing polygons they often don't notice if the polygons are topologically inconsistent or don't strictly adhere to the OSM guidelines for multipolygon relations.

Another common issue is that, by inspecting the tags alone, it can't be determine whether a way/edge borders water on both sides or borders water on one side and land on the other. Both cases are commonly used and championed in OSM.

### Basic procedure
Storing the location of every node is not memory (or cache!) friendly so Beaches makes several passes through the source osm file and only loads locations for a small number of nodes.

- Relations describing water polygons are parsed and cached.
- All ways bordering water are added to a connectivity graph.
- Lat/Lon is loaded into memory for nodes with degree > 2 and their neighbors.
- Edges reachable from the coast are identified by "turning left" at branching nodes.
- Edges that are not reachable from the coast are discarded.
- Edges that are bordered by water on both sides are discarded.
- The entire process is repeated for islands within reachable water polygons.
- The vectors or polygons are constructed and exported.