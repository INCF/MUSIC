bin_PROGRAMS = connect_adapter ros_sensor_adapter ros_command_adapter 


# look for ros in the common paths
LD_ROS_PATH = -L/opt/ros/kinetic/lib/ \
			  -L/opt/ros/jade/lib/

INC_ROS_PATH = -I/opt/ros/kinetic/include \
			   -I/opt/ros/jade/include

AM_CXXFLAGS = $(INC_ROS_PATH) -I$(top_srcdir) -I$(top_srcdir)/src -I$(top_builddir)/src @MPI_CXXFLAGS@
AM_LDFLAGS = $(top_builddir)/mpidep/libmpidep.la @MPI_LDFLAGS@ \
			 $(top_builddir)/src/libmusic.la \
			 $(top_builddir)/src/libmusic-c.la \
			 $(LD_ROS_PATH) 
						

connect_adapter_SOURCES = connect.h connect.cpp 
connect_adapter_LDADD = -lgsl -lgslcblas -ljsoncpp

ros_sensor_adapter_SOURCES = ros_sensor_adapter.h ros_sensor_adapter.cpp rtclock.h rtclock.cpp
ros_sensor_adapter_LDADD = -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lgsl -lgslcblas

ros_command_adapter_SOURCES = ros_command_adapter.h ros_command_adapter.cpp rtclock.h rtclock.cpp
ros_command_adapter_LDADD = -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lgsl -lgslcblas -ljsoncpp



