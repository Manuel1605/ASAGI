/* 
 * File:   threadgridcontainer.h
 * Author: root
 *
 * Created on 7. November 2013, 15:58
 */

#ifndef NUMAGRIDCONTAINER_H
#define	NUMAGRIDCONTAINER_H
#include "gridcontainer.h"
#include "numagrid.h"

namespace grid
{

/**
 * Simple grid container that stores one grid for each level. Each grid has to
 * cover the hole domain.
 */
class NumaGridContainer : public GridContainer
{
private:
	/** All grids we control */
	grid::Grid **m_grids;
public:
	NumaGridContainer(Type type, bool isArray = false, unsigned int hint = NO_HINT,
		unsigned int levels = 1);
	virtual ~NumaGridContainer();
	
	Error setParam(const char* name, const char* value,
		unsigned int level = 0);
	Error open(const char* filename, unsigned int level = 0);
	
	unsigned char getByte3D(double x, double y = 0, double z = 0,
		unsigned int level = 0);
	int getInt3D(double x, double y = 0, double z = 0,
		unsigned int level = 0);
	long getLong3D(double x, double y = 0, double z = 0,
		unsigned int level = 0);
	float getFloat3D(double x, double y = 0, double z = 0,
		unsigned int level = 0);
	double getDouble3D(double x, double y = 0, double z = 0,
		unsigned int level = 0);
	void getBuf3D(void* buf, double x, double y = 0, double z = 0,
		unsigned int level = 0);
	
	bool exportPng(const char* filename, unsigned int level = 0);
        asagi::Grid::Error setComm(MPI_Comm comm);

	unsigned long getCounter(const char* name, unsigned int level = 0);
        grid::Grid* createGrid(unsigned int hint, unsigned int id) const;
};

}


#endif	/* NUMAGRIDCONTAINER_H */

