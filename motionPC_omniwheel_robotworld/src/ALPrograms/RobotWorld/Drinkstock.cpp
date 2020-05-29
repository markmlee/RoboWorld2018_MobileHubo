#include "Drinkstock.h"

DrinkStock::DrinkStock()
{
    head = (Drink*)malloc(sizeof(Drink));
    head->MENU = NULL;
    head->prev = NULL;
    cur = head;
    for(int i=0;i<MAX_MENU;i++)
        MenuStockNum[i] = 0;

}

DrinkStock::~DrinkStock()
{
    cur = head;
    Drink drink;
    while(cur->next != NULL){
        Drink *temp = cur;
        cur = cur->next;
        free(temp);
    }
    free(cur);
}

int DrinkStock::AddDrinkFirst(int _menu, int _row)
{
    ROW = _row+1;
    for(int i=0;i<COL;i++)
    {
        Drink *newDrink = (Drink*)malloc(sizeof(Drink));
        newDrink->MENU = _menu;
        newDrink->prev = cur;
        cur->next = newDrink;
        cur = newDrink;
        cur->state = EXIST;

        cur->row = _row;
        cur->col = i;
        MenuStockNum[_menu-1]++;
    }
    return true;
}

void DrinkStock::setCOL(int _col)
{
    COL = _col;
}

int DrinkStock::GetRemainMenuNum(int _menu)
{

    return MenuStockNum[_menu-1];
}

void DrinkStock::ShowStock()
{
    if(setCurDrink(0,0))
    {
        Drink drink;
        for(int i=0;i<ROW;i++)
        {
            for(int j=0;j<COL;j++)
            {
                printf("%d(%d, %d, %d) ",cur->MENU,cur->row, cur->col, cur->state);
                isNext(&drink);
            }
            printf("\n");
        }
    }
    printf("SPRITE : %d, VITAMINWATER : %d, SAMDASOO : %d\n",MenuStockNum[SPRITE-1],MenuStockNum[VITAMINWATER-1],MenuStockNum[SAMDASOO-1]);
}


int DrinkStock::isNext(Drink *pDrink)
{
   if(cur->next == NULL)
   {
       printf("no next\n");
       return false;
   }

   cur = cur->next;
   pDrink = cur;
   return true;
}

int DrinkStock::setCurDrink(int _row, int _col)
{

    Drink drink;
    cur = head;
    if(cur->next == NULL)
    {
        printf("no drink\n");
        return false;
    }

    while(isNext(&drink))
    {
        if(drink.col == _col && drink.row == _row)
        {
            return true;
        }
    }
    return false;
}
