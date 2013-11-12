##### Files #####
set( gridSources
    adaptivegridcontainer.cpp
	grid.cpp
    gridcontainer.cpp
    numagridcontainer.cpp
    localcachegrid.cpp
	localstaticgrid.cpp
	multigrid.cpp
	passthroughgrid.cpp
	simplegridcontainer.cpp
        threadhandler.cpp 
        numalocalstaticgrid.cpp
        numalocalcachegrid.cpp
        numagrid.cpp
        )
if( NOT NOMPI )
	set( gridSources
		${gridSources}
		distcachegrid.cpp
		diststaticgrid.cpp )
endif( NOT NOMPI )

##### Add to library #####
add_sources( ${gridSources} )