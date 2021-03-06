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
 * @copyright 2013 Manuel Fasching <manuel.fasching@tum.de>
 */
#include <asagi.h>
#include <mpi.h>
#include <omp.h>

#define DEBUG_ABORT MPI_Abort(MPI_COMM_WORLD, 1)
#include "utils/logger.h"

#include "tests.h"

using namespace asagi;

int main(int argc, char** argv)
{
        int failure = 0;
        
        int rank;
	
	MPI_Init(&argc, &argv);
	
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	
        Grid* grid = Grid::createThreadHandler(asagi::Grid::FLOAT, 0x0, 1, THREADS);
#pragma omp parallel num_threads(THREADS)
{
       if (grid->open(NC_1D) != Grid::SUCCESS)
           failure = 1;
       if (failure != 1)
                for (int i = 0; i < NC_WIDTH; i++) 
                        if (grid->getInt1D(i) != i) {
                            logError() << "Test failed on rank " << rank << " and thread " << omp_get_thread_num() << std::endl
                                    << "Value at" << i << "should be"
                                    << i << "but is" << grid->getInt1D(i);
                            failure = 1;
                        }
}
        delete grid;
        MPI_Finalize();
        return failure;
}