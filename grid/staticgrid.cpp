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

#include "staticgrid.h"

/**
 * @see Grid::Grid()
 */
grid::StaticGrid::StaticGrid(const GridContainer &container,
	unsigned int hint)
	: Grid(container, hint)
{
	m_data = 0L;
}

grid::StaticGrid::~StaticGrid()
{
	freeLocalMem(m_data);
}

asagi::Grid::Error grid::StaticGrid::init()
{
	unsigned long blockSize = getTotalBlockSize();
	size_t block[3];
	unsigned long masterBlockCount = getLocalBlockCount();
	asagi::Grid::Error error;
	
	error = allocLocalMem(getType().getSize() * blockSize * masterBlockCount, m_data);
	if (error != asagi::Grid::SUCCESS)
		return error;
	
	// Load the blocks from the file, which we control
	for (unsigned long i = 0; i < masterBlockCount; i++) {
		if (getGlobalBlock(i) >= getBlockCount())
			// Last process(es) may control less blocks
			break;
		
		// Get x, y and z coordinates of the block
		getBlockPos(getGlobalBlock(i), block);
		
		// Get x, y and z coordinates of the first value in the block
		for (unsigned char i = 0; i < 3; i++)
			block[i] *= getBlockSize(i);
		
		getType().load(getInputFile(),
			block, getBlockSize(),
			&m_data[getType().getSize() * blockSize * i]);
	}
	
	return asagi::Grid::SUCCESS;
}

void grid::StaticGrid::getAt(void* buf, types::Type::converter_t converter,
	long unsigned int x, long unsigned int y, long unsigned int z)
{
	unsigned long blockSize = getTotalBlockSize();
	unsigned long block = getBlockByCoords(x, y, z);
	int remoteRank = getBlockRank(block); NDBG_UNUSED(remoteRank);
	unsigned long offset = getBlockOffset(block);
	
	// Offset inside the block
	x %= getBlockSize(0);
	y %= getBlockSize(1);
	z %= getBlockSize(2);
	
	assert(remoteRank == getMPIRank());
		
	(getType().*converter)(&m_data[getType().getSize() *
		(blockSize * offset // jump to the correct block
		+ (z * getBlockSize(1) + y) * getBlockSize(0) + x) // correct value inside the block
		],
		buf);
}