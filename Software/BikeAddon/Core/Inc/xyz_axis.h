#ifndef GYRO_H
#define GYRO_H

typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
} xyz_axis_t;

#define IMPLEMENT_READ_XYZ_VALUES(__ID) void read_xyz_values_##__ID(xyz_axis_t* inout_xyz_axis)

#endif
