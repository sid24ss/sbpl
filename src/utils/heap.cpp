/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sbpl/sbpl_exception.h>
#include <sbpl/utils/heap.h>

using namespace std;

void heaperror(const char* ErrorString)
{
    //need to send a message from here somehow
    SBPL_PRINTF("%s\n", ErrorString);
    throw new SBPL_Exception();
}

//returns an infinite key
CKey InfiniteKey()
{
    CKey key;
    key.SetKeytoInfinity();
    return key;
}

//---------------------------------normal (multi-priority) CHeap class--------------------------------------------------

//constructors and destructors
CHeap::CHeap()
{
    percolates = 0;
    currentsize = 0;
    allocated = HEAPSIZE_INIT;

    heap = new heapelement[allocated];

}

CHeap::~CHeap()
{
    int i;
    for (i = 1; i <= currentsize; ++i)
        heap[i].heapstate->heapindex = 0;

    delete[] heap;
}

void CHeap::percolatedown(int hole, heapelement tmp)
{
    int child;

    if (currentsize != 0) {
        for (; 2 * hole <= currentsize; hole = child) {
            child = 2 * hole;

            if (child != currentsize && heap[child + 1].key < heap[child].key) ++child;
            if (heap[child].key < tmp.key) {
                percolates += 1;
                heap[hole] = heap[child];
                heap[hole].heapstate->heapindex = hole;
            }
            else
                break;
        }
        heap[hole] = tmp;
        heap[hole].heapstate->heapindex = hole;
    }
}

void CHeap::percolateup(int hole, heapelement tmp)
{
    if (currentsize != 0) {
        for (; hole > 1 && tmp.key < heap[hole / 2].key; hole /= 2) {
            percolates += 1;
            heap[hole] = heap[hole / 2];
            heap[hole].heapstate->heapindex = hole;
        }
        heap[hole] = tmp;
        heap[hole].heapstate->heapindex = hole;
    }
}

void CHeap::percolateupordown(int hole, heapelement tmp)
{
    if (currentsize != 0) {
        if (hole > 1 && heap[hole / 2].key > tmp.key)
            percolateup(hole, tmp);
        else
            percolatedown(hole, tmp);
    }
}

bool CHeap::emptyheap()
{
    return currentsize == 0;
}

bool CHeap::fullheap()
{
    return currentsize == HEAPSIZE - 1;
}

bool CHeap::inheap(AbstractSearchState *AbstractSearchState)
{
    return (AbstractSearchState->heapindex != 0);
}

CKey CHeap::getkeyheap(AbstractSearchState *AbstractSearchState)
{
    if (AbstractSearchState->heapindex == 0) heaperror("GetKey: AbstractSearchState is not in heap");

    return heap[AbstractSearchState->heapindex].key;
}

void CHeap::makeemptyheap()
{
    int i;

    for (i = 1; i <= currentsize; ++i)
        heap[i].heapstate->heapindex = 0;
    currentsize = 0;
}

void CHeap::makeheap()
{
    int i;

    for (i = currentsize / 2; i > 0; i--) {
        percolatedown(i, heap[i]);
    }
}

void CHeap::growheap()
{
    heapelement* newheap;
    int i;

    SBPL_PRINTF("growing heap size from %d ", allocated);

    allocated = 2 * allocated;
    if (allocated > HEAPSIZE) allocated = HEAPSIZE;

    SBPL_PRINTF("to %d\n", allocated);

    newheap = new heapelement[allocated];

    for (i = 0; i <= currentsize; ++i)
        newheap[i] = heap[i];

    delete[] heap;

    heap = newheap;
}

void CHeap::sizecheck()
{
    if (fullheap())
        heaperror("insertheap: heap is full");
    else if (currentsize == allocated - 1) {
        growheap();
    }
}

void CHeap::insertheap(AbstractSearchState *AbstractSearchState, CKey key)
{
    heapelement tmp;
    char strTemp[100];

    sizecheck();

    if (AbstractSearchState->heapindex != 0) {
        sprintf(strTemp, "insertheap: AbstractSearchState is already in heap");
        heaperror(strTemp);
    }
    tmp.heapstate = AbstractSearchState;
    tmp.key = key;
    percolateup(++currentsize, tmp);
}

void CHeap::deleteheap(AbstractSearchState *AbstractSearchState)
{
    if (AbstractSearchState->heapindex == 0) heaperror("deleteheap: AbstractSearchState is not in heap");
    percolateupordown(AbstractSearchState->heapindex, heap[currentsize--]);
    AbstractSearchState->heapindex = 0;
}

void CHeap::updateheap(AbstractSearchState *AbstractSearchState, CKey NewKey)
{
    if (AbstractSearchState->heapindex == 0) heaperror("Updateheap: AbstractSearchState is not in heap");
    if (heap[AbstractSearchState->heapindex].key != NewKey) {
        heap[AbstractSearchState->heapindex].key = NewKey;
        percolateupordown(AbstractSearchState->heapindex, heap[AbstractSearchState->heapindex]);
    }
}

void CHeap::insert_unsafe(AbstractSearchState* AbstractSearchState, CKey key)
{
    heapelement tmp;
    char strTemp[100];

    sizecheck();

    if (AbstractSearchState->heapindex != 0) {
        sprintf(strTemp, "insertheap: AbstractSearchState is already in heap");
        heaperror(strTemp);
    }
    tmp.heapstate = AbstractSearchState;
    tmp.key = key;

    ++currentsize;
    heap[currentsize] = tmp;
    heap[currentsize].heapstate->heapindex = currentsize;
}

void CHeap::deleteheap_unsafe(AbstractSearchState* AbstractSearchState)
{
    if (AbstractSearchState->heapindex == 0) {
        heaperror("deleteheap: AbstractSearchState is not in heap");
    }

    heap[AbstractSearchState->heapindex] = heap[currentsize];
    --currentsize;

    heap[AbstractSearchState->heapindex].heapstate->heapindex = AbstractSearchState->heapindex;
    AbstractSearchState->heapindex = 0;
}

void CHeap::updateheap_unsafe(AbstractSearchState* AbstractSearchState, CKey NewKey)
{
    if (AbstractSearchState->heapindex == 0) {
        heaperror("updateheap: AbstractSearchState is not in heap");
    }
    if (heap[AbstractSearchState->heapindex].key != NewKey) {
        heap[AbstractSearchState->heapindex].key = NewKey;
    }
}

AbstractSearchState* CHeap::getminheap()
{
    if (currentsize == 0) heaperror("GetMinheap: heap is empty");
    return heap[1].heapstate;
}

AbstractSearchState* CHeap::getminheap(CKey& ReturnKey)
{
    if (currentsize == 0) {
        heaperror("GetMinheap: heap is empty");
        ReturnKey = InfiniteKey();
    }
    ReturnKey = heap[1].key;
    return heap[1].heapstate;
}

CKey CHeap::getminkeyheap()
{
    CKey ReturnKey;
    if (currentsize == 0) return InfiniteKey();
    ReturnKey = heap[1].key;
    return ReturnKey;
}

AbstractSearchState* CHeap::deleteminheap()
{
    AbstractSearchState *AbstractSearchState;

    if (currentsize == 0) heaperror("DeleteMin: heap is empty");

    AbstractSearchState = heap[1].heapstate;
    AbstractSearchState->heapindex = 0;
    percolatedown(1, heap[currentsize--]);
    return AbstractSearchState;
}

//---------------------------------end of normal (multi-priority) CHeap class-------------------------------------------

//---------------------------------single-priority CIntHeap class---------------------------------------------------

//constructors and destructors
CIntHeap::CIntHeap()
{
    percolates = 0;
    currentsize = 0;
    allocated = HEAPSIZE_INIT;

    heap = new heapintelement[allocated];
}

CIntHeap::CIntHeap(int initial_size)
{
    percolates = 0;
    currentsize = 0;
    allocated = initial_size;

    heap = new heapintelement[allocated];
}

CIntHeap::~CIntHeap()
{
    int i;
    for (i = 1; i <= currentsize; ++i)
        heap[i].heapstate->heapindex = 0;

    delete[] heap;
}

void CIntHeap::percolatedown(int hole, heapintelement tmp)
{
    int child;

    if (currentsize != 0) {
        for (; 2 * hole <= currentsize; hole = child) {
            child = 2 * hole;

            if (child != currentsize && heap[child + 1].key < heap[child].key) ++child;
            if (heap[child].key < tmp.key) {
                percolates += 1;
                heap[hole] = heap[child];
                heap[hole].heapstate->heapindex = hole;
            }
            else
                break;
        }
        heap[hole] = tmp;
        heap[hole].heapstate->heapindex = hole;
    }
}

void CIntHeap::percolateup(int hole, heapintelement tmp)
{
    if (currentsize != 0) {
        for (; hole > 1 && tmp.key < heap[hole / 2].key; hole /= 2) {
            percolates += 1;
            heap[hole] = heap[hole / 2];
            heap[hole].heapstate->heapindex = hole;
        }
        heap[hole] = tmp;
        heap[hole].heapstate->heapindex = hole;
    }
}

void CIntHeap::percolateupordown(int hole, heapintelement tmp)
{
    if (currentsize != 0) {
        if (hole > 1 && heap[hole / 2].key > tmp.key)
            percolateup(hole, tmp);
        else
            percolatedown(hole, tmp);
    }
}

bool CIntHeap::emptyheap()
{
    return currentsize == 0;
}

bool CIntHeap::fullheap()
{
    return currentsize == HEAPSIZE - 1;
}

bool CIntHeap::inheap(AbstractSearchState *AbstractSearchState)
{
    return (AbstractSearchState->heapindex != 0);
}

int CIntHeap::getkeyheap(AbstractSearchState *AbstractSearchState)
{
    if (AbstractSearchState->heapindex == 0) heaperror("GetKey: AbstractSearchState is not in heap");

    return heap[AbstractSearchState->heapindex].key;
}

void CIntHeap::makeemptyheap()
{
    int i;

    for (i = 1; i <= currentsize; ++i)
        heap[i].heapstate->heapindex = 0;
    currentsize = 0;
}

void CIntHeap::makeheap()
{
    int i;

    for (i = currentsize / 2; i > 0; i--) {
        percolatedown(i, heap[i]);
    }
}

void CIntHeap::growheap()
{
    heapintelement* newheap;
    int i;

    SBPL_PRINTF("growing heap size from %d ", allocated);

    allocated = 2 * allocated;
    if (allocated > HEAPSIZE) allocated = HEAPSIZE;

    SBPL_PRINTF("to %d\n", allocated);

    newheap = new heapintelement[allocated];

    for (i = 0; i <= currentsize; ++i)
        newheap[i] = heap[i];

    delete[] heap;

    heap = newheap;
}

void CIntHeap::sizecheck()
{
    if (fullheap())
        heaperror("insertheap: heap is full");
    else if (currentsize == allocated - 1) {
        growheap();
    }
}

void CIntHeap::insertheap(AbstractSearchState *AbstractSearchState, int key)
{
    heapintelement tmp;
    char strTemp[100];

    sizecheck();

    if (AbstractSearchState->heapindex != 0) {
        sprintf(strTemp, "insertheap: AbstractSearchState is already in heap");
        heaperror(strTemp);
    }
    tmp.heapstate = AbstractSearchState;
    tmp.key = key;
    percolateup(++currentsize, tmp);
}

void CIntHeap::deleteheap(AbstractSearchState *AbstractSearchState)
{
    if (AbstractSearchState->heapindex == 0) heaperror("deleteheap: AbstractSearchState is not in heap");
    percolateupordown(AbstractSearchState->heapindex, heap[currentsize--]);
    AbstractSearchState->heapindex = 0;
}

void CIntHeap::updateheap(AbstractSearchState *AbstractSearchState, int NewKey)
{
    if (AbstractSearchState->heapindex == 0) heaperror("Updateheap: AbstractSearchState is not in heap");
    if (heap[AbstractSearchState->heapindex].key != NewKey) {
        heap[AbstractSearchState->heapindex].key = NewKey;
        percolateupordown(AbstractSearchState->heapindex, heap[AbstractSearchState->heapindex]);
    }
}

AbstractSearchState* CIntHeap::getminheap()
{
    if (currentsize == 0) heaperror("GetMinheap: heap is empty");
    return heap[1].heapstate;
}

AbstractSearchState* CIntHeap::getminheap(int& ReturnKey)
{
    if (currentsize == 0) {
        heaperror("GetMinheap: heap is empty");
    }
    ReturnKey = heap[1].key;
    return heap[1].heapstate;
}

int CIntHeap::getminkeyheap()
{
    int ReturnKey;
    if (currentsize == 0) return INFINITECOST;
    ReturnKey = heap[1].key;
    return ReturnKey;
}

AbstractSearchState* CIntHeap::deleteminheap()
{
    AbstractSearchState *AbstractSearchState;

    if (currentsize == 0) heaperror("DeleteMin: heap is empty");

    AbstractSearchState = heap[1].heapstate;
    AbstractSearchState->heapindex = 0;
    percolatedown(1, heap[currentsize--]);
    return AbstractSearchState;
}

//---------------------------------end of single-priority CIntHeap class------------------------------------------------

CHeapArr::CHeapArr(int num)
{
  percolates = 0;
  if (num > MAX_NUM) {
    heaperror("Max number exceeded");
  } 
  for (int j=0; j < num; j++) {
  	currentsize[j] = 0;
  	allocated[j] = HEAPSIZE_INIT;
	heap[j] = new heapelement[allocated[j]];
  }
}

CHeapArr::~CHeapArr()
{
  int i;
  for (int j=0; j < this->num; j++) {
	  for (i=1; i<=currentsize[j]; ++i)
		  heap[j][i].heapstate->heapind[j] = 0;
  	delete [] heap[j];
  }
  delete [] heap;
}


void CHeapArr::percolatedown(int hole, heapelement tmp, int i)
{
  int child;

  if (currentsize[i] != 0)
  {

    for (; 2*hole <= currentsize[i]; hole = child)
	{
	  child = 2*hole;

	  if (child != currentsize[i] && heap[i][child+1].key < heap[i][child].key)
	    ++child;
	  if (heap[i][child].key < tmp.key)
	    {
	      percolates += 1;
	      heap[i][hole] = heap[i][child];
	      heap[i][hole].heapstate->heapind[i] = hole;
	    }
	  else
	    break;
	}
      heap[i][hole] = tmp;
      heap[i][hole].heapstate->heapind[i] = hole;
   }
}

void CHeapArr::percolateup(int hole, heapelement tmp, int i)
{
  if (currentsize[i] != 0)
    {
      for (; hole > 1 && tmp.key < heap[i][hole/2].key; hole /= 2)
	  {
		percolates += 1;
		heap[i][hole] = heap[i][hole/2];
		heap[i][hole].heapstate->heapind[i] = hole;
	  }  
      heap[i][hole] = tmp;
      heap[i][hole].heapstate->heapind[i] = hole;
    }
}

void CHeapArr::percolateupordown(int hole, heapelement tmp, int i)
{
  if (currentsize != 0)
    {
      if (hole > 1 && heap[i][hole/2].key > tmp.key)
		percolateup(hole, tmp, i);
      else
		percolatedown(hole, tmp, i);
    }
}

bool CHeapArr::emptyheap(int i)
{
  return currentsize[i] == 0;
}


bool CHeapArr::fullheap(int i)
{
  return currentsize[i] == HEAPSIZE-1;
}

bool CHeapArr::inheap(AbstractSearchState *AbstractSearchState, int i)
{
  return (AbstractSearchState->heapind[i] != 0);
}


CKey CHeapArr::getkeyheap(AbstractSearchState *AbstractSearchState, int i)
{
  if (AbstractSearchState->heapind[i] == 0)
    heaperror("GetKey: AbstractSearchState is not in heap");

  return heap[i][AbstractSearchState->heapind[i]].key;
}

void CHeapArr::makeemptyheap(int j)
{
  int i;

  for (i=1; i<=currentsize[j]; ++i)
    heap[j][i].heapstate->heapind[j] = 0;
  currentsize[j] = 0;
}

void CHeapArr::makeheap(int j)
{
  int i;

  for (i = currentsize[j] / 2; i > 0; i--)
    {
      percolatedown(i, heap[j][i], j);
    }
}

void CHeapArr::growheap(int j)
{
  heapelement* newheap;
  int i;

  SBPL_PRINTF("growing heap[%d] size from %d ",j, allocated[j]);

  allocated[j] = 2*allocated[j];
  if(allocated[j] > HEAPSIZE)
	  allocated[j] = HEAPSIZE;

  SBPL_PRINTF("to %d\n", allocated[j]);

  newheap = new heapelement[allocated[j]];

  for (i=0; i<=currentsize[j]; ++i)
    newheap[i] = heap[j][i];

  delete [] heap[j];

  heap[j] = newheap;
}


void CHeapArr::sizecheck(int i)
{

  if (fullheap(i))
    heaperror("insertheap: heap is full");
  else if(currentsize[i] == allocated[i]-1)
  {
	growheap(i);
  }
}



void CHeapArr::insertheap(AbstractSearchState *AbstractSearchState, CKey key, int i)
{
  heapelement tmp;
  char strTemp[100];


  sizecheck(i);

  if (AbstractSearchState->heapind[i] != 0)
    {
      sprintf(strTemp, "insertheap: AbstractSearchState is already in heap");
      heaperror(strTemp);
   }
  tmp.heapstate = AbstractSearchState;
  tmp.key = key;
  percolateup(++currentsize[i], tmp, i); 
}

void CHeapArr::deleteheap(AbstractSearchState *AbstractSearchState, int i)
{
  if (AbstractSearchState->heapind[i] == 0)
    heaperror("deleteheap: AbstractSearchState is not in heap");
  percolateupordown(AbstractSearchState->heapind[i], heap[i][currentsize[i]--], i);
  AbstractSearchState->heapind[i] = 0;
}

void CHeapArr::updateheap(AbstractSearchState *AbstractSearchState, CKey NewKey, int i)
{
  if (AbstractSearchState->heapind[i] == 0)
    heaperror("Updateheap: AbstractSearchState is not in heap");
  if (heap[i][AbstractSearchState->heapind[i]].key != NewKey)
    {
      heap[i][AbstractSearchState->heapind[i]].key = NewKey;
      percolateupordown(AbstractSearchState->heapind[i], heap[i][AbstractSearchState->heapind[i]], i);
    }
}

void CHeapArr::insert_unsafe(AbstractSearchState *AbstractSearchState, CKey key, int i) {
	heapelement tmp;
	char strTemp[100];


	sizecheck(i);

	if (AbstractSearchState->heapind[i] != 0)
	{
      sprintf(strTemp, "insertheap: AbstractSearchState is already in heap");
      heaperror(strTemp);
    }
	tmp.heapstate = AbstractSearchState;
	tmp.key = key;


	++ currentsize[i];
	heap[i][currentsize[i]] = tmp;
	heap[i][currentsize[i]].heapstate->heapind[i] = currentsize[i];


}


void CHeapArr::deleteheap_unsafe(AbstractSearchState *AbstractSearchState, int i) {
	if (AbstractSearchState->heapind[i] == 0)
	 heaperror("deleteheap: AbstractSearchState is not in heap");
	
	heap[i][AbstractSearchState->heapind[i]] = heap[i][currentsize[i]];
	--currentsize[i];
	
	heap[i][AbstractSearchState->heapind[i]].heapstate->heapind[i] = AbstractSearchState->heapind[i];
	AbstractSearchState->heapind[i] = 0;

}

void CHeapArr::updateheap_unsafe(AbstractSearchState *AbstractSearchState, CKey NewKey, int i){

	if (AbstractSearchState->heapind[i] == 0)
		heaperror("Updateheap: AbstractSearchState is not in heap");
	if (heap[i][AbstractSearchState->heapind[i]].key != NewKey) {	
		heap[i][AbstractSearchState->heapind[i]].key = NewKey;
	}
}


AbstractSearchState* CHeapArr::getminheap(int i)
{
  if (currentsize[i] == 0)
    heaperror("GetMinheap: heap is empty");
  return heap[i][1].heapstate;
}

AbstractSearchState* CHeapArr::getminheap(CKey& ReturnKey, int i)
{
  if (currentsize[i] == 0)
    {
      heaperror("GetMinheap: heap is empty");
      ReturnKey = InfiniteKey();
    }
  ReturnKey = heap[i][1].key;
  return heap[i][1].heapstate;
}

CKey CHeapArr::getminkeyheap(int i)
{  
  CKey ReturnKey;
  if (currentsize[i] == 0)
    return InfiniteKey();
  ReturnKey = heap[i][1].key;
  return ReturnKey;
}

AbstractSearchState* CHeapArr::deleteminheap(int i)
{
  AbstractSearchState *AbstractSearchState;

  if (currentsize[i] == 0)
    heaperror("DeleteMin: heap is empty");

  AbstractSearchState = heap[i][1].heapstate;
  //printf("Before delete --%d\n", AbstractSearchState->heapindex1); 
  //assert(AbstractSearchState->heapindex1 == 1);
  AbstractSearchState->heapind[i] = 0;
  percolatedown(1, heap[i][currentsize[i]--],i);
  return AbstractSearchState;
}





