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

#include <asagi.h>
#include <mpi.h>

#define DEBUG_ABORT MPI_Abort(MPI_COMM_WORLD, 1)
#include "utils/logger.h"

#include "tests.h"

using namespace asagi;

int main(int argc, char** argv)
{
        int rank;
        
        MPI_Init(&argc, &argv);
        
        MPI_Comm_rank(MPI_COMM_WORLD, &rank);
        
        Grid* grid = Grid::create(); // FLOAT is default
        
        if (grid->open(NC_3D) != Grid::SUCCESS)
                return 1;
        
        long value;
        
        for (int i = 0; i < NC_WIDTH; i++) {
                for (int j = 0; j < NC_LENGTH; j++) {
                        for (int k = 0; k < NC_HEIGHT; k++) {
                                value = (k * NC_LENGTH + j) * NC_WIDTH + i;
                                if (grid->getFloat3D(i, j, k) != value) {
                                        logError() << "Test failed on rank" << rank << std::endl
                                                << "Value at" << i << j << k << "should be"
                                                << value << "but is" << grid->getInt3D(i, j, k);
                                        return 1;
                                }
                        }
                }
        }
        
        delete grid;
        
        MPI_Finalize();
        
        return 0;
}