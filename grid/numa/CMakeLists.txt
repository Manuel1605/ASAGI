 
##### Files #####
set( gridSources
        threadhandler.cpp 
        numalocalstaticgrid.cpp
        numalocalcachegrid.cpp
        numagrid.cpp
        numagridcontainer.cpp
        numasimplegridcontainer.cpp
        )
if( NOT NOMPI )
	set( gridSources
		${gridSources}
		numadiststaticgrid.cpp )
endif( NOT NOMPI )

##### Add to library #####
add_sources( ${gridSources} )