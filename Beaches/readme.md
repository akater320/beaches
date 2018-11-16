# Beaches

Beaches produces a set of "coastline" vectors from an OSM file. Rather than directly extracting rings of ways labeled with "natural"="coastline", Beaches makes an effort to remove edges that are bordered by water on two sides. This allows the coastline to extend into bays and larger rivers that may be navigable.  

Optionally, the vectors can be buffered and a polygon representing litoral zones can be exported.

## Building
To build beaches you're need a c++17 compiler that supports parallel stl. Specifically you'll need the [`<execution>`](https://en.cppreference.com/w/cpp/header/execution) header. 

MSVC 2017 (Wine) should both work. Check your compiler for compatibility here. https://godbolt.org/z/wJdoYc

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
