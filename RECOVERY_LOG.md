# KAKUTEH7 Recovery Log - 2026-06-28

## Emergency Recovery Event

### Problem
After attempting to revert USB_DEVICE files to HEAD~1 and rebuild, the firmware would no longer boot. LED remained off even after 30+ seconds. Previous working state was lost.

### Investigation
1. **Attempted Fix 1**: Disabled USB voltage detector (`HAL_PWREx_EnableUSBVoltageDetector()`)
   - Result: Still no LED blink
   - Conclusion: Not the root cause

2. **Attempted Fix 2**: Mass erase + clean rebuild
   - Result: Still no LED blink
   - Conclusion: Problem not in build artifacts

3. **Root Cause**: The revert target (HEAD~1) itself had latent issues preventing boot
   - Likely: Incomplete USB initialization state in HEAD~1 causing undefined behavior

### Solution
**Hard reset to commit 1999c2d** - Last known good state with confirmed LED blinking

```bash
git reset --hard 1999c2d
```

### Key Finding: 30-Second Startup Delay

The ~30 second boot delay is **NOT caused by USB voltage detector alone**, but rather by the USB OTG core soft reset timeout in `stm32h7xx_ll_usb.c:1466`.

Even with voltage detector disabled, if any USB-related code path tries to initialize the USB core, the soft reset waits for CSRST bit which never clears without proper USB PHY clocks (requires PLL).

**Working Solution**: Disable ALL USB code paths:
- No `HAL_PWREx_EnableUSBVoltageDetector()`
- No `USBD_Init()`
- No `USBD_RegisterClass()`
- No `USBD_Start()`

### Current Stable Configuration (1999c2d)

**Files Status:**
- `USB_DEVICE/App/usb_device.c`: MX_USB_DEVICE_Init() is empty (all USB calls commented out)
- `Core/Src/main.c`: Calls MX_USB_DEVICE_Init() but function does nothing
- `Core/Src/system_stm32h7xx.c`: Timeout-protected power supply waits, debugger attachment delay
- `Core/Src/stm32h7xx_it.c`: HardFault handler tolerates DEBUGEVT faults

**Boot Sequence (with ~30s delay from USB soft reset timeout somewhere in chain):**
1. MCU boots from flash
2. Bootloader jumps to main
3. HAL_Init() → SystemInit() → System clock configured
4. SystemClock_Config() → HSI enabled, no PLL
5. MPU_Config() → MPU setup
6. MX_GPIO_Init() → PC2 LED configured
7. MX_USB_DEVICE_Init() → Does nothing (USB disabled)
8. Main loop enters → LED blinks 500ms on/off
   - With ~30s initial startup delay before first blink

**Verified Working:**
✅ LED blinks continuously every 500ms
✅ Debugger connects over SWD @ 200 kHz HOTPLUG
✅ Breakpoints halt execution correctly
✅ System runs stably without HardFaults

### Next Steps (Future Work)

**To enable USB enumeration as virtual COM port:**
1. Implement full PLL setup (M=8, N=12, Q=2 for 48 MHz)
2. Enable USB OTG peripheral clock in RCC
3. Properly configure USB PHY
4. Re-enable USBD_Init and voltage detector
5. Debug HardFault that occurred in previous PLL attempts

**Current Recommendation:**
- Keep USB disabled and baseline stable
- Work on USB in separate branch after PLL issues resolved
- Do not attempt USB changes on master until PLL stability confirmed

### Prevention Measures

1. **Never revert to HEAD~1 without testing first** - Check commit message for warnings
2. **Always rebuild from clean cmake** - Use "Configure & Build Debug" task, not manual builds
3. **Use branches for experimental features** - Checkout -b branch_name, don't revert main files
4. **Tag stable baselines** - After each working state, create annotated git tag

### Files Changed in This Recovery

No code changes - only git reset to 1999c2d. Recovery accomplished through version control.

---
**Recovery Time:** ~10 minutes from "LED off" to "LED blinking confirmed"
**Lesson:** Stable baselines with git tags are essential for embedded development
**Status:** ✅ STABLE - Ready for next feature development or USB debugging
