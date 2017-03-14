import java.util.Collections;

class cell implements Comparable<cell> {
  public int screenX;
  public int screenY;   
  public boolean occupied;
  public color c;
  public int fcost, gcost, hcost;
  public void drawCell(int cellWidth) {
    if (occupied) {
      fill(0, 0, 0);
    } else {
      fill(c);
    }
    rect(screenX, screenY, cellWidth, cellWidth);
  }

  int compareTo(cell o) {
    if (fcost == o.fcost) { 
      return 0;
    } else if (fcost > o.fcost) { 
      return 1;
    } else { 
      return -1;
    }
  }

  cell(int i, int j, boolean o) {
    screenX = i;
    screenY = j;
    occupied = o;
    c = color(255, 255, 255);
  }
}
float distance(int x1, int x2, int y1, int y2) {
  float dist = sqrt(sq((x1 - x2)) + sq((y1 - y2)));
  return dist;
}



int cellWidth = 10, // width and height should both be evenly divisable by cellWidth  
  numCellsX, numCellsY;
float dist2Target, dist2start;
int TARGETx, TARGETy, STARTx, STARTy;
int obstacleCount = 7;
cell map[][]; 

void aStarNoDiagonal(int Tx, int Ty, int Sx, int Sy) {
  int Cx = Sx, Cy = Sy;//current location
  boolean isReached = false;
  ArrayList<cell> openCell = new ArrayList<cell>(8);// cells available for exploring
  ArrayList<cell> closedCell = new ArrayList<cell>(40);// already visited cells
  while (!isReached) {
    closedCell.add(map[Cx][Cy]);
    for (int i = -1; i < 2; i++) {
      for (int j = -1; j < 2; j++) {

        if ((i == -1 && j == -1) || (i == -1 && j == 1)|| (i == 1 && j == 1) || (i == 1 && j ==-1)) { // no diagonal movement doesnt check corners of 3x3 matrix
          continue;
        }
        if (!(openCell.contains(map[Cx + i][Cy + j])) && !(closedCell.contains(map[Cx + i][Cy + j]))) {
          closedCell.add(map[i][j]);
        }
      }
    }
  }
}


void aStarWithDiagonal(int Tx, int Ty, int Sx, int Sy) {
  int Cx = Sx, Cy = Sy;//current location
  boolean isReached = false;
  ArrayList<cell> openCell = new ArrayList<cell>(8);
  ArrayList<cell> closedCell = new ArrayList<cell>(40);
  while (!isReached) {
    closedCell.add(map[Cx][Cy]);
    for (int i = -1; i < 2; i++) {  
      for (int j = -1; j < 2; j++) {
        if (!(openCell.contains(map[Cx + i][Cy + j])) && !(closedCell.contains(map[Cx + i][Cy + j]))) { // checks all surrounding cells
          
        }
      }
    }
  }
}






void setup() {
  size(500, 500);
  numCellsX = width/cellWidth;
  numCellsY = height/cellWidth;
  map = new cell[numCellsX][numCellsY];
  int scrXPos = 0;
  int scrYPos = 0;
  for (int i = 0; i < numCellsX; i++) {
    for (int j = 0; j < numCellsY; j++) {
      map[i][j] = new cell(scrXPos, scrYPos, ((int)(random(20) % 15) == 1 ? true : false)); // random locations in the map are occupied locations "walls"
      map[i][j].drawCell(cellWidth);
      scrXPos += cellWidth;
    }
    scrXPos = 0;
    scrYPos += cellWidth;
  }
  // draws target location near the both right corner
  TARGETx = (int)((numCellsX - cellWidth)+random(numCellsX/cellWidth));
  TARGETy = (int)((numCellsY - cellWidth)+random(numCellsY/cellWidth));
  map[TARGETx][TARGETy].c = color(0, 255, 0);
  map[TARGETx][TARGETy].occupied = false;

  // draws start location near the top left corner
  STARTx = (int)random(numCellsX/cellWidth);
  STARTy = (int)random(numCellsY/cellWidth);
  map[STARTx][STARTy].c = color(255, 10, 15);
  map[STARTx][STARTy].occupied = false;
}
void draw() {
  for (int i = 0; i < numCellsX; i++) {
    for (int j = 0; j < numCellsY; j++) {
      map[i][j].drawCell(cellWidth);
    }
  }


  dist2Target = distance(STARTx, TARGETx, STARTy, TARGETy);
  //println(dist2Target, abs((STARTx - TARGETx)), abs((STARTy - TARGETy)));
}