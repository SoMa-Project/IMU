/*******************************************************************************
* File Name: USB_VDD.h  
* Version 2.10
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_USB_VDD_H) /* Pins USB_VDD_H */
#define CY_PINS_USB_VDD_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "USB_VDD_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    USB_VDD_Write(uint8 value) ;
void    USB_VDD_SetDriveMode(uint8 mode) ;
uint8   USB_VDD_ReadDataReg(void) ;
uint8   USB_VDD_Read(void) ;
uint8   USB_VDD_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define USB_VDD_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define USB_VDD_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define USB_VDD_DM_RES_UP          PIN_DM_RES_UP
#define USB_VDD_DM_RES_DWN         PIN_DM_RES_DWN
#define USB_VDD_DM_OD_LO           PIN_DM_OD_LO
#define USB_VDD_DM_OD_HI           PIN_DM_OD_HI
#define USB_VDD_DM_STRONG          PIN_DM_STRONG
#define USB_VDD_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define USB_VDD_MASK               USB_VDD__MASK
#define USB_VDD_SHIFT              USB_VDD__SHIFT
#define USB_VDD_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define USB_VDD_PS                     (* (reg8 *) USB_VDD__PS)
/* Data Register */
#define USB_VDD_DR                     (* (reg8 *) USB_VDD__DR)
/* Port Number */
#define USB_VDD_PRT_NUM                (* (reg8 *) USB_VDD__PRT) 
/* Connect to Analog Globals */                                                  
#define USB_VDD_AG                     (* (reg8 *) USB_VDD__AG)                       
/* Analog MUX bux enable */
#define USB_VDD_AMUX                   (* (reg8 *) USB_VDD__AMUX) 
/* Bidirectional Enable */                                                        
#define USB_VDD_BIE                    (* (reg8 *) USB_VDD__BIE)
/* Bit-mask for Aliased Register Access */
#define USB_VDD_BIT_MASK               (* (reg8 *) USB_VDD__BIT_MASK)
/* Bypass Enable */
#define USB_VDD_BYP                    (* (reg8 *) USB_VDD__BYP)
/* Port wide control signals */                                                   
#define USB_VDD_CTL                    (* (reg8 *) USB_VDD__CTL)
/* Drive Modes */
#define USB_VDD_DM0                    (* (reg8 *) USB_VDD__DM0) 
#define USB_VDD_DM1                    (* (reg8 *) USB_VDD__DM1)
#define USB_VDD_DM2                    (* (reg8 *) USB_VDD__DM2) 
/* Input Buffer Disable Override */
#define USB_VDD_INP_DIS                (* (reg8 *) USB_VDD__INP_DIS)
/* LCD Common or Segment Drive */
#define USB_VDD_LCD_COM_SEG            (* (reg8 *) USB_VDD__LCD_COM_SEG)
/* Enable Segment LCD */
#define USB_VDD_LCD_EN                 (* (reg8 *) USB_VDD__LCD_EN)
/* Slew Rate Control */
#define USB_VDD_SLW                    (* (reg8 *) USB_VDD__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define USB_VDD_PRTDSI__CAPS_SEL       (* (reg8 *) USB_VDD__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define USB_VDD_PRTDSI__DBL_SYNC_IN    (* (reg8 *) USB_VDD__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define USB_VDD_PRTDSI__OE_SEL0        (* (reg8 *) USB_VDD__PRTDSI__OE_SEL0) 
#define USB_VDD_PRTDSI__OE_SEL1        (* (reg8 *) USB_VDD__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define USB_VDD_PRTDSI__OUT_SEL0       (* (reg8 *) USB_VDD__PRTDSI__OUT_SEL0) 
#define USB_VDD_PRTDSI__OUT_SEL1       (* (reg8 *) USB_VDD__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define USB_VDD_PRTDSI__SYNC_OUT       (* (reg8 *) USB_VDD__PRTDSI__SYNC_OUT) 


#if defined(USB_VDD__INTSTAT)  /* Interrupt Registers */

    #define USB_VDD_INTSTAT                (* (reg8 *) USB_VDD__INTSTAT)
    #define USB_VDD_SNAP                   (* (reg8 *) USB_VDD__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins USB_VDD_H */


/* [] END OF FILE */
