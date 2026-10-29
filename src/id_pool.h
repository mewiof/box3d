// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "container.h"

typedef struct b3IdPool
{
	b3Array( int ) freeArray;
	int nextIndex;
	int retiredCount;
} b3IdPool;

b3IdPool b3CreateIdPool( void );
void b3DestroyIdPool( b3IdPool* pool );

int b3AllocId( b3IdPool* pool );
void b3FreeId( b3IdPool* pool, int id );
void b3ValidateFreeId( const b3IdPool* pool, int id );

#define B3_MAX_GENERATION UINT16_MAX

static inline void b3RetireOrFreeId( b3IdPool* pool, int id, uint16_t generation )
{
	if ( generation >= B3_MAX_GENERATION )
	{
		pool->retiredCount += 1;
		return;
	}

	b3FreeId( pool, id );
}

static inline int b3GetIdCount( const b3IdPool* pool )
{
	return pool->nextIndex - pool->freeArray.count - pool->retiredCount;
}

static inline int b3GetIdCapacity( const b3IdPool* pool )
{
	return pool->nextIndex;
}

static inline int b3GetIdBytes( const b3IdPool* pool )
{
	return b3Array_ByteCount( pool->freeArray );
}
