#include "board.h"

void bsp_early_initialize(void){

}

void bsp_initialize(void){



}

void bsp_post_initialize(void){


}

void rt_hw_board_init()
{
    bsp_early_initialize();
}

