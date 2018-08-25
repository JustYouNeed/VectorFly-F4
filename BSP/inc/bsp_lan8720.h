# ifndef __BSP_LAN_H
# define __BSP_LAN_H

# include "bsp.h"
# include "bsp_malloc.h"

extern ETH_DMADESCTypeDef *DMArxDescTab;
extern ETH_DMADESCTypeDef *DMATxDescTab;
extern uint8_t *Rx_Buff;
extern uint8_t *Tx_Buff;
extern ETH_DMADESCTypeDef *DMATxDescToSet;
extern ETH_DMADESCTypeDef *DMARxDescToGet;
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_Infos;


# define LAN_RST		PDout(3)
# define LAN_PHY_ADDR		0x00

uint8_t bsp_LANConfig(void);
uint8_t bsp_LANGetSpeed(void);
uint8_t ETH_MACDMA_Config(void);
FrameTypeDef ETH_Rx_Packet(void);
uint8_t ETH_Tx_Packet(uint16_t length);
uint32_t ETH_GetCurTxBuff(void);
uint8_t ETH_Malloc(void);
void ETH_NVICConfig(void);
void ETH_Free(void);

# endif
