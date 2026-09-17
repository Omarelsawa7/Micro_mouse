#include "maze.h"
#include <stdio.h> /* printf for maze_print_dist; keeps module pure C (no Arduino/C++ dependency) */

static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {1, 0, -1, 0};

static uint8_t dir_bit(Direction d)
{
    switch (d) {
    case DIR_NORTH: return WALL_N;
    case DIR_EAST:  return WALL_E;
    case DIR_SOUTH: return WALL_S;
    case DIR_WEST:  return WALL_W;
    }
    return 0;
}

static Direction dir_opposite(Direction d)
{
    return (Direction)((d + 2) & 0x03);
}

void maze_init(Maze *m)
{
    int x;
    int y;

    if (m == 0) {
        return;
    }
    for (y = 0; y < MAZE_SIZE; y++) {
        for (x = 0; x < MAZE_SIZE; x++) {
            m->cells[x][y].walls   = 0;
            m->cells[x][y].seen    = 0;
            m->cells[x][y].visited = false;
            m->dist[x][y]          = 255;
        }
    }
    /* Perimeter walls are always known solid. */
    for (x = 0; x < MAZE_SIZE; x++) {
        maze_set_wall(m, x, 0, DIR_SOUTH, true);
        maze_set_wall(m, x, MAZE_SIZE - 1, DIR_NORTH, true);
    }
    for (y = 0; y < MAZE_SIZE; y++) {
        maze_set_wall(m, 0, y, DIR_WEST, true);
        maze_set_wall(m, MAZE_SIZE - 1, y, DIR_EAST, true);
    }
    /* Start cell: east wall known open per classic maze (entry). Keep west shut. */
    m->px = START_X;
    m->py = START_Y;
    m->heading = DIR_NORTH;
    m->cells[START_X][START_Y].visited = true;
}

bool maze_in_bounds(int x, int y)
{
    return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

bool maze_is_goal(int x, int y)
{
    return x >= MAZE_GOAL_MIN && x <= MAZE_GOAL_MAX
        && y >= MAZE_GOAL_MIN && y <= MAZE_GOAL_MAX;
}

bool maze_is_start(int x, int y)
{
    return x == START_X && y == START_Y;
}

void maze_set_wall(Maze *m, int x, int y, Direction d, bool solid)
{
    int nx;
    int ny;
    uint8_t b;
    uint8_t nb;

    if (m == 0 || !maze_in_bounds(x, y)) {
        return;
    }
    b = dir_bit(d);
    nb = dir_bit(dir_opposite(d));

    if (solid) {
        m->cells[x][y].walls |= b;
        m->cells[x][y].seen  |= b;
    } else {
        m->cells[x][y].walls &= (uint8_t)(~b);
        m->cells[x][y].seen  |= b;
    }
    nx = x + DX[d];
    ny = y + DY[d];
    if (maze_in_bounds(nx, ny)) {
        if (solid) {
            m->cells[nx][ny].walls |= nb;
            m->cells[nx][ny].seen  |= nb;
        } else {
            m->cells[nx][ny].walls &= (uint8_t)(~nb);
            m->cells[nx][ny].seen  |= nb;
        }
    }
}

static bool passable(const Maze *m, int x, int y, Direction d, bool pessimistic)
{
    int nx;
    int ny;
    uint8_t b;

    if (!maze_in_bounds(x, y)) {
        return false;
    }
    b = dir_bit(d);
    if (m->cells[x][y].walls & b) {
        return false; /* known solid */
    }
    nx = x + DX[d];
    ny = y + DY[d];
    if (!maze_in_bounds(nx, ny)) {
        return false;
    }
    if (pessimistic) {
        /* Strict safe return: the NEIGHBOR CELL must have been physically
         * visited. An edge merely seen open (e.g. IR looking into the next
         * cell) is NOT enough to drive into it. */
        if (!m->cells[nx][ny].visited) {
            return false;
        }
        if (!(m->cells[x][y].seen & b)) {
            return false; /* edge never mapped */
        }
        return true;
    }
    if (m->cells[x][y].seen & b) {
        return true; /* known open */
    }
    /* Unknown edge: optimistic = open. */
    return true;
}

void maze_flood(Maze *m, int tx, int ty, bool pessimistic)
{
    int qx[MAZE_SIZE * MAZE_SIZE];
    int qy[MAZE_SIZE * MAZE_SIZE];
    int head = 0;
    int tail = 0;
    int x;
    int y;
    int d;

    if (m == 0) {
        return;
    }
    for (y = 0; y < MAZE_SIZE; y++) {
        for (x = 0; x < MAZE_SIZE; x++) {
            m->dist[x][y] = 255;
        }
    }

    if (tx == -1) {
        /* Multi-target: all goal cells distance 0 */
        for (y = MAZE_GOAL_MIN; y <= MAZE_GOAL_MAX; y++) {
            for (x = MAZE_GOAL_MIN; x <= MAZE_GOAL_MAX; x++) {
                m->dist[x][y] = 0;
                qx[tail] = x;
                qy[tail] = y;
                tail++;
            }
        }
    } else {
        if (!maze_in_bounds(tx, ty)) {
            return;
        }
        m->dist[tx][ty] = 0;
        qx[tail] = tx;
        qy[tail] = ty;
        tail++;
    }

    while (head < tail) {
        int cx = qx[head];
        int cy = qy[head];
        head++;

        for (d = 0; d < 4; d++) {
            int nx = cx + DX[d];
            int ny = cy + DY[d];
            Direction dir = (Direction)d;
            if (!maze_in_bounds(nx, ny)) {
                continue;
            }
            /* Edge cx->nx must be passable from cx side.
             * Check from neighbor back to cx equivalently; use cx side. */
            if (!passable(m, cx, cy, dir, pessimistic)) {
                continue;
            }
            if (m->dist[nx][ny] == 255) {
                m->dist[nx][ny] = m->dist[cx][cy] + 1;
                qx[tail] = nx;
                qy[tail] = ny;
                tail++;
            }
        }
    }
}

void maze_flood_to_goal(Maze *m, bool pessimistic)
{
    maze_flood(m, -1, -1, pessimistic);
}

void maze_flood_to_start(Maze *m, bool pessimistic)
{
    maze_flood(m, START_X, START_Y, pessimistic);
}

bool maze_next_dir(const Maze *m, int x, int y, Direction heading,
                   bool pessimistic, Direction *out_dir)
{
    int best = 1000;
    int best_rot = 99;
    int d;
    bool found = false;
    Direction bd = DIR_NORTH;

    if (m == 0 || !maze_in_bounds(x, y)) {
        return false;
    }
    for (d = 0; d < 4; d++) {
        int nx = x + DX[d];
        int ny = y + DY[d];
        uint8_t bit = dir_bit((Direction)d);
        int rot;
        if (!maze_in_bounds(nx, ny)) {
            continue;
        }
        if (m->cells[x][y].walls & bit) {
            continue; /* known solid wall */
        }
        if (pessimistic) {
            /* Strict safe return: enter only physically visited cells via
             * mapped edges. Unknown edges AND unvisited cells are walls. */
            if (!(m->cells[x][y].seen & bit)) {
                continue;
            }
            if (!m->cells[nx][ny].visited) {
                continue;
            }
        }
        rot = (d - (int)heading + 4) & 0x03;
        if (rot > 2) {
            rot = 4 - rot; /* 0=straight, 1=side, 2=back */
        }
        if (m->dist[nx][ny] < best ||
            (m->dist[nx][ny] == best && rot < best_rot)) {
            best = m->dist[nx][ny];
            best_rot = rot;
            bd = (Direction)d;
            found = true;
        }
    }
    /* If all neighbors are 255 (unreachable under pessimistic map),
     * report trapped so caller can stop safely instead of wandering. */
    if (!found || best >= 255) {
        return false;
    }
    if (out_dir) {
        *out_dir = bd;
    }
    return true;
}

void maze_observe(Maze *m, bool wall_left, bool wall_right, bool wall_front)
{
    Direction dl;
    Direction dr;
    Direction df;

    if (m == 0) {
        return;
    }
    dl = (Direction)((m->heading + 3) & 0x03);
    dr = (Direction)((m->heading + 1) & 0x03);
    df = m->heading;

    maze_set_wall(m, m->px, m->py, dl, wall_left);
    maze_set_wall(m, m->px, m->py, dr, wall_right);
    maze_set_wall(m, m->px, m->py, df, wall_front);
    m->cells[m->px][m->py].visited = true;
}

int maze_dist_at(const Maze *m, int x, int y)
{
    if (m == 0 || !maze_in_bounds(x, y)) {
        return 255;
    }
    return m->dist[x][y];
}

void maze_print_dist(const Maze *m)
{
    int x;
    int y;

    if (m == 0) {
        return;
    }
    printf("--- flood distances (y=%d top) ---\n", MAZE_SIZE - 1);
    for (y = MAZE_SIZE - 1; y >= 0; y--) {
        for (x = 0; x < MAZE_SIZE; x++) {
            if (m->dist[x][y] >= 255) {
                printf(" ##");
            } else {
                printf("%3d", m->dist[x][y]);
            }
        }
        printf("\n");
    }
}
