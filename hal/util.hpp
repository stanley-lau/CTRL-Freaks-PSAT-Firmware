#ifndef UTIL_HPP
#define UTIL_HPP

// Helper functions for registers passed as template arguments like `&P1OUT`

/// Clears the bits in the register that are set in the mask
#define CLEAR_BITS(reg, mask)    ((*(reg)) &= ~(mask))

/// Sets the bits in the register that are set in the mask
#define SET_BITS(reg, mask)      ((*(reg)) |= (mask))

/// Toggles the bits in the register that are set in the mask 
#define TOGGLE_BITS(reg, mask)   ((*(reg)) ^= (mask))

/// Returns true if any of the bits in the mask are set in the register
#define IS_SET(reg, mask)    ((*(reg)) & (mask))

/// Returns true if all the bits in the mask are set in the register
#define ALL_SET(reg, mask)    (((*(reg)) & (mask)) == (mask))

#endif
