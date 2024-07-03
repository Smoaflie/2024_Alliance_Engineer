/*
    FLASH只能支持整页删除，算了下时间容易造成任务堵塞，
    为了让C板能上动态UI，没去研究达妙H7板自带的W25Q64，
    权衡后选择单独开个任务，并且只针对最后一个扇区进行操作
*/
#include "main.h"
#include "robot_def.h"
#include "bsp_flash.h"
#include "message_center.h"
extern osThreadId_t flashHandle;

static Subscriber_t *flash_cmd_sub;
static FLASH_Data_s flash_param;
static uint32_t flash_data_toWrite[8] = {0};
uint32_t flash_data_read[8];
static uint8_t data_cnt = 0;
void flashInit()
{
    flash_cmd_sub   = SubRegister("flash_cmd", sizeof(FLASH_Data_s));
    memcpy((void*)flash_data_read,(void*)ADDR_FLASH_SECTOR_7,32);
}
uint32_t SectorError = 0;
uint8_t write_flag = 0,erase_flag = 0;
static void flashRefresh()
{
    // flash_erase_address(ADDR_FLASH_SECTOR_7,1);
    static FLASH_EraseInitTypeDef flash_erase;

    flash_erase.Sector = FLASH_SECTOR_7;
    flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    flash_erase.NbSectors = 1;
    flash_erase.Banks = FLASH_BANK_1;
    HAL_FLASH_Unlock();
    if(HAL_FLASHEx_Erase(&flash_erase,&SectorError) == HAL_OK)
        write_flag = 1;
    HAL_FLASH_Lock();
}
void flashWrite(FLASH_Data_s flash_param)
{
    if(SubGetMessage(flash_cmd_sub, &flash_param) && flash_param.save_call){
        uint8_t flash_refresh_flag = 0;
        data_cnt = 0;
        if(erase_flag==0){
            for(int i = 0; i < sizeof(flash_param)/sizeof(Flash_write_param_t); i++){
                if(flash_param.flash_param[i].address==0)   continue;
                for(int j = 0; j < flash_param.flash_param[i].len; j++){
                    flash_data_toWrite[data_cnt] = *(flash_param.flash_param[i].data + j);
                    if(flash_data_toWrite[data_cnt] != *((uint32_t*)flash_param.flash_param[i].address + j))   flash_refresh_flag = 1;

                    data_cnt++;
                }
            }
            if(flash_refresh_flag)
                flashRefresh();
            if(write_flag){
                write_flag = 0;
                flash_write_single_address(ADDR_FLASH_SECTOR_7, flash_data_toWrite, 3);
            }
        }
    }
}