#ifndef _GRIDIDEA_GLOBAL_H
#define _GRIDIDEA_GLOBAL_H

enum DataType
{
	U8,
	U16,
	U32,
	I8,
	I16,
	I32,
	F32,
	H8,
	H16,
	H32,
	MIXED,
	BIT
};

#define PARAM_U8                (0x0 << 12)
#define PARAM_U16               (0x1 << 12)
#define PARAM_U32               (0x2 << 12)
#define PARAM_I8                (0x3 << 12)
#define PARAM_I16               (0x4 << 12)
#define PARAM_I32               (0x5 << 12)
#define PARAM_F32               (0x6 << 12)
#define PARAM_TYPE(x)           ((x & 0x7000)>>12)
#define PARAM_INDEX(x)          (x & 0x0FFF)
#define PARAM_LIVE              (0x8000)
#define PARAM_IS_LIVE(x)        ((x & PARAM_LIVE) != 0)
#endif