#ifndef COLOR_MASK_H
#define COLOR_MASK_H

#define RMASK 0xff000000
#define GMASK 0x00ff0000
#define BMASK 0x0000ff00

#define SET_RED(P, C) (P = ((P & ~RMASK) | (C << 24)))
#define SET_GREEN(P, C) (P = ((P & ~GMASK) | (C << 16)))
#define SET_BLUE(P, C) (P = ((P & ~BMASK) | (C << 8)))
#define GET_RED(P) ((P & RMASK) >> 24)
#define GET_GREEN(P) ((P & GMASK) >> 16)
#define GET_BLUE(P) ((P & BMASK) >> 8)

#endif

