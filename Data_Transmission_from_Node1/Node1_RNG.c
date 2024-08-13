/**
 * 
 * 
 */

#include <stm32f4xx_hal.h>
#include "Node1_RNG.h"


/**
 * Init the random number generator (RNG) peripheral
 */
void rngInit( void )
{
	//start the peripheral clock
	__HAL_RCC_RNG_CLK_ENABLE();

	//enable the random number generator
	RNG->CR |= RNG_CR_RNGEN;
}

/**
 * NOTE:this function doesn't guarantee a new number every call
 * @param Min smallest number to generate
 * @param Max largest number to generate
 * @returns a pseudo random number from the dedicated RNG peripheral
 */
uint32_t StmRand( uint32_t Min, uint32_t Max )
{
	return RNG->DR %Max + Min;
}


