C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe --bin --output C:\Users\matt\Documents\workspace\theshop-genesis\code\level_1_1\bootloader\_build\mcg_bootloader C:\Users\matt\Documents\workspace\theshop-genesis\code\level_1_1\bootloader\_build\mcg_bootloader.axf
copy _build\mcg_bootloader\ER_IROM1 _build\mcg_bootloader.bin
"C:\Program Files (x86)\Nordic Semiconductor\Master Control Panel\3.9.0.6\nrf\nrf.exe" dfu genpkg mcg_bootloader.zip --bootloader C:\Users\matt\Documents\workspace\theshop-genesis\code\level_1_1\bootloader\_build\mcg_bootloader.bin --application-version 0x0100 --dev-revision 0xffff --dev-type 0xffff --sd-req 0xFFFE
copy mcg_bootloader.zip "C:\Users\matt\Google Drive\THE SHOP\Wearables\genesis\Firmware_Update\Bootloader\"