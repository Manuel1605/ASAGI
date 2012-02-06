#include "types/type.h"

#include "simplegrid.h"

SimpleGrid::SimpleGrid(Type type)
	: Grid(type)
{
	masterData = 0L;
	slaveData = 0L;
	
	window = MPI_WIN_NULL;
}

SimpleGrid::~SimpleGrid()
{
	MPI_Win_free(&window);
	
	MPI_Free_mem(masterData);
	free(slaveData);
}

bool SimpleGrid::init()
{
	unsigned long blockSize = getXBlockSize() * getYBlockSize();
	unsigned long blockX, blockY;
	
	masterBlockCount = (getBlockCount() + m_mpiSize - 1) / m_mpiSize;

	MPI_Alloc_mem(m_type->getSize() * blockSize * masterBlockCount,
		MPI_INFO_NULL, &masterData);
	
	// Load the blocks from the file, which we control
	for (unsigned long i = 0; i < masterBlockCount; i++) {
		if (i + m_mpiRank * masterBlockCount >= getBlockCount())
			// Last process may controll less blocks
			break;
		
		// Get x and y coordinates of the block
		getBlockPos(i + m_mpiRank * masterBlockCount, blockX, blockY);
		
		// Get x and y coordinates of the first value in the block
		blockX = blockX * getXBlockSize();
		blockY = blockY * getYBlockSize();
		
		m_type->load(*file,
			blockX, blockY,
			getXBlockSize(), getYBlockSize(),
			&masterData[m_type->getSize() * blockSize * i]);
	}
	
	// Create the mpi window for the master data
	if (MPI_Win_create(masterData,
		m_type->getSize() * blockSize * masterBlockCount,
		m_type->getSize(),
		MPI_INFO_NULL,
		communicator,
		&window) != MPI_SUCCESS)
		return false;
	
	// Allocate memory for slave blocks
	slaveData = static_cast<unsigned char*>(
		malloc(m_type->getSize() * blockSize * getBlocksPerNode()));
	blockManager.init(getBlocksPerNode());
	
	return true;
}

void* SimpleGrid::getAt(unsigned long x, unsigned long y)
{
	unsigned long blockSize = getXBlockSize() * getYBlockSize();
	unsigned long block = getBlockByCoords(x, y);
	int remoteRank;
	unsigned long remoteOffset;
	
	// Offset inside the block
	x %= getXBlockSize();
	y %= getYBlockSize();
	
	if ((block >= m_mpiRank * masterBlockCount)
		&& (block < (m_mpiRank + 1) * masterBlockCount)) {
		// Nice, this is a block where we are the master
		
		block -= m_mpiRank * masterBlockCount;
	
		return &masterData[m_type->getSize() *
			(blockSize * block // jump to the correct block
			+ y * getXBlockSize() + x) // correct value inside the block
			];
	}
	
	if (!blockManager.getIndex(block)) {
		// We do not have this block, transfer it first
	
		// Where do we find the block?
		remoteRank = block / masterBlockCount;
		remoteOffset = block % masterBlockCount;
		
		// Index where we store the block
		block = blockManager.getFreeIndex(block);
		
		// Transfer data
		
		// I think we can use nocheck, because we only read
		// -> no conflicting locks
		// TODO check this
		MPI_Win_lock(MPI_LOCK_SHARED, remoteRank,
			MPI_MODE_NOCHECK, window);
		
		MPI_Get(&slaveData[m_type->getSize() * blockSize * block],
			blockSize,
			m_type->getMPIType(),
			remoteRank,
			remoteOffset * blockSize,
			blockSize,
			m_type->getMPIType(),
			window);
		
		MPI_Win_unlock(remoteRank, window);
	}
		
	return &slaveData[m_type->getSize() *
		(blockSize * block // correct block
		+ y * getXBlockSize() + x) // correct value inside the block
		];
}