##### Files #####
if ( NUMA_SUPPORT ) 
    add_subdirectory( numa )
else()

 set( gridSources
        grid.cpp
        gridcontainer.cpp
        localcachegrid.cpp
        localstaticgrid.cpp
        multigrid.cpp
        passthroughgrid.cpp 
        simplegridcontainer.cpp
        adaptivegridcontainer.cpp )
    if( NOT NOMPI )
            set( gridSources
                    ${gridSources}
                    distcachegrid.cpp
                    diststaticgrid.cpp )
    endif( NOT NOMPI )

endif( NUMA_SUPPORT )

##### Add to library #####
add_sources( ${gridSources} )