#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* Directions: 0=N(+y) 1=E(+x) 2=S(-y) 3=W(-x) */
typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
} Direction;

#define WALL_N 0x01
#define WALL_E 0x02
#define WALL_S 0x04
#define WALL_W 0x08

typedef struct {
    uint8_t walls;    /* known SOLID walls (bitmask WALL_*) */
    uint8_t seen;     /* known OPEN passages (bitmask WALL_*) */
    bool    visited;
} Cell;

typedef struct {
    Cell cells[MAZE_SIZE][MAZE_SIZE];
    int   dist[MAZE_SIZE][MAZE_SIZE];
    int   px;         /* robot x 0..7 */
    int   py;         /* robot y 0..7 */
    Direction heading;
} Maze;

void maze_init(Maze *m);
bool maze_in_bounds(int x, int y);
bool maze_is_goal(int x, int y);
bool maze_is_start(int x, int y);

/* Record one wall side + mirror to neighbor. solid=true -> wall, false -> open. */
void maze_set_wall(Maze *m, int x, int y, Direction d, bool solid);

/* Flood-fill distances.
 * pessimistic=false: unknown = open (fast explore to goal).
 * pessimistic=true : unknown = wall (strict safe return, only mapped path). */
void maze_flood(Maze *m, int tx, int ty, bool pessimistic);
void maze_flood_to_goal(Maze *m, bool pessimistic);
void maze_flood_to_start(Maze *m, bool pessimistic);

/* Best neighbor direction from (x,y) per dist[]. Returns false if trapped.
 * pessimistic=true: unknown edges (not in seen) are treated as walls and
 * never chosen (strict safe return: mapped path only).
 * Tie-break prefers less rotation from heading (straight > side > back). */
bool maze_next_dir(const Maze *m, int x, int y, Direction heading,
                   bool pessimistic, Direction *out_dir);

/* Fuse IR snapshot into map at current pose. */
void maze_observe(Maze *m, bool wall_left, bool wall_right, bool wall_front);

int  maze_dist_at(const Maze *m, int x, int y);
void maze_print_dist(const Maze *m);

#endif /* MAZE_H */
