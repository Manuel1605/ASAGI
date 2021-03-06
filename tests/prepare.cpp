/**
 * @file
 *  This file is part of ASAGI.
 * 
 *  ASAGI is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ASAGI is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with ASAGI.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Diese Datei ist Teil von ASAGI.
 *
 *  ASAGI ist Freie Software: Sie koennen es unter den Bedingungen
 *  der GNU General Public License, wie von der Free Software Foundation,
 *  Version 3 der Lizenz oder (nach Ihrer Option) jeder spaeteren
 *  veroeffentlichten Version, weiterverbreiten und/oder modifizieren.
 *
 *  ASAGI wird in der Hoffnung, dass es nuetzlich sein wird, aber
 *  OHNE JEDE GEWAEHELEISTUNG, bereitgestellt; sogar ohne die implizite
 *  Gewaehrleistung der MARKTFAEHIGKEIT oder EIGNUNG FUER EINEN BESTIMMTEN
 *  ZWECK. Siehe die GNU General Public License fuer weitere Details.
 *
 *  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 *  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
 * 
 * @copyright 2012 Sebastian Rettenberger <rettenbs@in.tum.de>
 */

#include <fstream>
#include <iostream>
#include <netcdf.h>

#define NC_1D "1dgrid.nc"
#define NC_1DPSEUDO "1dpseudogrid.nc"
#define NC_2D "2dgrid.nc"
#define NC_3D "3dgrid.nc"
#define NC_PERFTEST NC_2D
#define HEADER_FILENAME "tests.h"
#ifdef NUMA_SUPPORT
#define THREADS 4
#endif

#define WIDTH 51
#define LENGTH 51
#define HEIGHT 51

int main()
{
	int file;
	int var;
	int dim[3];
	
	// Create the nc files
	
	// 1d
	nc_create(NC_1D, NC_NETCDF4, &file);
	
	nc_def_dim(file, "x", WIDTH, &dim[0]);
	
	nc_def_var(file, "z", NC_FLOAT, 1, dim, &var);
	
	float values_1d[WIDTH];
	
	for (unsigned int i = 0; i < WIDTH; i++)
		values_1d[i] = i;
	
	nc_put_var_float(file, var, values_1d);

	nc_close(file);
	
	// pseudo 1d
	nc_create(NC_1DPSEUDO, NC_NETCDF4, &file);

	nc_def_dim(file, "y", 1, &dim[0]);
	nc_def_dim(file, "x", WIDTH, &dim[1]);
	
	nc_def_var(file, "z", NC_FLOAT, 2, dim, &var);
	
	nc_put_var_float(file, var, values_1d); // reuse values
	
	nc_close(file);
	
	// 2d
	nc_create(NC_2D, NC_NETCDF4, &file);
	
	nc_def_dim(file, "y", LENGTH, &dim[0]);
	nc_def_dim(file, "x", WIDTH, &dim[1]);
	
	nc_def_var(file, "z", NC_FLOAT, 2, dim, &var);
	
	float values_2d[LENGTH][WIDTH];
	
	for (unsigned int i = 0; i < LENGTH; i++)
		for (unsigned int j = 0; j < WIDTH; j++)
			values_2d[i][j] = i * WIDTH + j;
	
	nc_put_var_float(file, var, reinterpret_cast<float*>(values_2d));

	nc_close(file);
	
	// 3d
	nc_create(NC_3D, NC_NETCDF4, &file);
	
	nc_def_dim(file, "t", HEIGHT, &dim[0]); // can't set this to z ...
	nc_def_dim(file, "y", LENGTH, &dim[1]);
	nc_def_dim(file, "x", WIDTH, &dim[2]);
	
	nc_def_var(file, "z", NC_FLOAT, 3, dim, &var);
	
	size_t offset_3d[3] = {0, 0, 0};
	size_t size_3d[3] = {1, LENGTH, WIDTH};
	
	for (unsigned int i = 0; i < HEIGHT; i++) {
		offset_3d[0] = i;
		
		for (unsigned int j = 0; j < LENGTH; j++)
			for (unsigned int k = 0; k < WIDTH; k++)
				values_2d[j][k] = (i * LENGTH + j) * WIDTH + k;
		
		nc_put_vara_float(file, var, offset_3d, size_3d, reinterpret_cast<float*>(values_2d));
	}
	
	nc_close(file);

	// Create the header file
	std::ofstream headerFile;
	headerFile.open(HEADER_FILENAME);
	headerFile << "#ifndef TESTS_H\n";
	headerFile << "#define TESTS_H\n";
	
	headerFile << "#define NC_1D \"" << NC_1D << "\"\n";
	headerFile << "#define NC_1DPSEUDO \"" << NC_1DPSEUDO << "\"\n";
	headerFile << "#define NC_2D \"" << NC_2D << "\"\n";
	headerFile << "#define NC_3D \"" << NC_3D << "\"\n";
	headerFile << "#define NC_PERFTEST \"" << NC_PERFTEST << "\"\n";
	headerFile << "#define NC_WIDTH " << WIDTH << "\n";
	headerFile << "#define NC_LENGTH " << LENGTH << "\n";
	headerFile << "#define NC_HEIGHT " << HEIGHT << "\n";
#ifdef NUMA_SUPPORT
        headerFile << "#define THREADS " << THREADS << "\n";
#endif
	
	headerFile << "#endif // TESTS_H\n";
	headerFile.close();
	
	return 0;
}
