
SET(SLAMCLASS_SRCS
  Camera.cpp
  Frame.cpp
  MapPoint.cpp
  Map.cpp
  Input_Handler.cpp
  Map_Viewer.cpp
  Triangulation_Solver.cpp
  Pnp_Solver.cpp
  Map_Builder.cpp
  
)

# shared library
ADD_LIBRARY(recons_slam SHARED ${SLAMCLASS_SRCS})
target_link_libraries( recons_slam
    ${THIRD_PARTY_LIBS}
)
