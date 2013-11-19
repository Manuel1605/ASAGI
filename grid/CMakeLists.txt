##### Files #####
set( gridSources
    adaptivegridcontainer.cpp
	grid.cpp
    gridcontainer.cpp
    localcachegrid.cpp
	localstaticgrid.cpp
	multigrid.cpp
	passthroughgrid.cpp
	simplegridcontainer.cpp
        )
if( NOT NOMPI )
	set( gridSources
		${gridSources}
		distcachegrid.cpp
		diststaticgrid.cpp )
endif( NOT NOMPI )
if ( NUMA_SUPPORT ) 
    add_subdirectory( numa )
endif( NUMA_SUPPORT )
##### Add to library #####
add_sources( ${gridSources} )