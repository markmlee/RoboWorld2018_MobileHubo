#ifndef DRINKSTOCK_H
#define DRINKSTOCK_H
#include "../../../share/Headers/UserSharedMemory.h"
#include <stdio.h>
#include <stdlib.h>
#define MAX_MENU    3

enum{
    NOEXIST = 0, EXIST, FAIL
};

typedef struct _Drink
{
    int     MENU;
    int     row;
    int     col;
    int     state;
    double  Ypos;
    struct _Drink   *prev;
    struct _Drink   *next;
}Drink;

class DrinkStock
{
public:
    DrinkStock();
    ~DrinkStock();

    Drink *cur;
    Drink *head;
private:
    int MenuStockNum[MAX_MENU];
    int ROW;
    int COL;
public:
    void setCOL(int _col);
    void ShowStock();
    int isNext(Drink *pDrink);
    int setCurDrink(int _row, int _col);
    int GetRemainMenuNum(int _menu);
    int AddDrinkFirst(int _menu, int _row);

};

#endif // DRINKSTOCK_H
