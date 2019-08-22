

#ifndef INIT_H
#define INIT_H

// SR04 tasks use a C struct named LAYER for their output commands:


typedef struct layer {
  int cmd;        /* assertion command */
  int arg;        /* assertion argument */
  int flag;
  int state;      /* layer state, used by AFSM tasks */
} LAYER;

// SR04 has 9 subsumption behaviors, so define 9 LAYERS:

extern LAYER bump,motion,ir,boundary,sonar,photo,xlate,prowl,stop;

// layers[] is an array of pointers to the LAYER struct for each task:

#ifndef LAYERS
#define LAYERS 9
#endif



/* --------------------------------------------------------------- */
// The arbitration flags on SR04 are implemented as bit positions in a
// 16 bit flag word (hence only 16 possible layers), defined as follows:

#ifndef BUMP
#define BUMP    1
#endif

#ifndef MOTION
#define MOTION  2
#endif

#ifndef IR
#define IR      4
#endif

#ifndef BOUNDARY
#define BOUNDARY 8
#endif

#ifndef SONAR
#define SONAR 0x10
#endif

#ifndef PHOTO
#define PHOTO 0x20
#endif

#ifndef XLATE
#define XLATE 0x40
#endif

#ifndef PROWL
#define PROWL 0x80
#endif

#ifndef DEFAULT
#define DEFAULT 0x100
#endif

// These bits definition apply to a set of global masks:

//int srat_flags;                 /* bit = layer asserting */
//int srat_avoid;                 /* bit = invert response */
//int srat_enable;                /* bit = enable this layer */
//int srat_suppress;              /* bit = ignore this layer */

/* --------------------------------------------------------------- */

#endif
