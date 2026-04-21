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

#define PARAM_U8                (U8 << 12)
#define PARAM_U16               (U16 << 12)
#define PARAM_U32               (U32 << 12)
#define PARAM_I8                (I8 << 12)
#define PARAM_I16               (I16 << 12)
#define PARAM_I32               (I32 << 12)
#define PARAM_F32               (F32 << 12)
#define PARAM_TYPE(x)           ((x & 0x7000)>>12)
#define PARAM_INDEX(x)          (x & 0x0FFF)
#define PARAM_LIVE              (0x8000)
#define PARAM_IS_LIVE(x)        ((x & PARAM_LIVE) != 0)


#define PARAM_U8_SZ_MAX		256
#define PARAM_I8_SZ_MAX		256
#define PARAM_U16_SZ_MAX	128
#define PARAM_I16_SZ_MAX	128
#define PARAM_U32_SZ_MAX	128
#define PARAM_I32_SZ_MAX	128
#define PARAM_F4_SZ_MAX		128

#endif