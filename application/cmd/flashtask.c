/*
    FLASH只能支持整页删除，算了下时间容易造成任务堵塞，
    为了让C板能上动态UI，没去研究达妙H7板自带的W25Q64，
    权衡后选择单独开个任务，并且只针对最后一个扇区进行操作
*/
#include "flashtask.h"

Flash_write_param_t get_arm_auto_mode_record();
Flash_write_param_t get_arm_encoder_data();
Flash_write_param_t get_UI_param();

uint32_t SectorError = 0;
uint8_t write_flag = 0;

static inline void flash_write(Flash_write_param_t *data){
    flash_write_single_address(data->address, data->data, data->len);
}
void flashRefresh()
{
    Flash_write_param_t arm_auto_mode_record = get_arm_auto_mode_record();
    Flash_write_param_t arm_encoder_record = get_arm_encoder_data();
    Flash_write_param_t UI_param_record =   get_UI_param();

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
    if(write_flag){
        write_flag = 0;
        flash_write(&arm_auto_mode_record);
        flash_write(&arm_encoder_record);
        flash_write(&UI_param_record);
    }
}