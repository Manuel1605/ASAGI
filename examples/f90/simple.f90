! Simple library test
program simple
  use asagi
  use mpi
  implicit none
  
  integer :: grid_id
  integer :: error

  call MPI_Init( error )
  write (*,*) 'hier' 
  grid_id = grid_create_threadhandler( GRID_FLOAT, GRID_NO_HINT, 1, 1  )
  write (*,*) 'da auch noch'
  if( grid_open( grid_id, "/home/asagi/test_program/1dgrid.nc" ) /= GRID_SUCCESS ) then
    write (*,*) 'Could not load file'
    call exit(1)
  end if
  write (*,*) 'tod'

  write (*,*) "Range X:", grid_min_x( grid_id ), grid_max_x( grid_id )
  write (*,*) "Range Y:", grid_min_y( grid_id ), grid_max_y( grid_id )

  write (*,*) "Value at 5x10:", grid_get_float( grid_id, 5.d+0, 10.d+0 )
  write (*,*) "Value at 5x10.1:", grid_get_float_2d( grid_id, 5.d+0, 10.1d+0 )
  write (*,*) "Value at -1x-5005.32:", grid_get_float_2d( grid_id, -1.d+0, -5005.32d+0 )

  call grid_close( grid_id )

  call MPI_Finalize( error )
  
end program simple
