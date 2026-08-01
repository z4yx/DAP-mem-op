# Test command file for svf_gen.py
# Mix of writes, reads without verify, and reads with verify (Y)
#
# Format: <addr_hex>  <data_hex>  <W|R>  [Y [MASK]]  [WAIT<n>]
#   MASK (hex, optional) follows Y on a read line: only bits set to 1
#   in MASK are compared against data_hex; other bits are ignored.
#   WAIT<n> (optional) overrides the global --wait-cycles for this line.

0     100 TCK

# --- Write some registers / memory locations ---
1000  DEADBEEF  W WAIT1800
1004  CAFEBABE  W WAIT1800
1008  12345678  W WAIT1800
100C  000000FF  W  WAIT500

2000  16   MEM test_firmware.bin
2000  0    MEM test_firmware.bin

# --- Read back and verify ---
1000  DEADBEEF  R  Y 1 WAIT1800
1004  CAFEBABE  R  Y FFFFFFFF WAIT1800
100C  000000FF  R  Y FFFFFFFF WAIT1800

# --- Read without verification (TDO not checked) ---
1008  00000000  R
100C  00000000  R N

# --- Write to a different region ---
1A00  ABCD0001  W
1A04  ABCD0002  W
1A08  ABCD0003  W

# --- Read back and verify the second region ---
1A00  ABCD0001  R  Y FFFFFFFF
1A04  ABCD0002  R  Y FFFFFFFF
1A08  ABCD0003  R  Y FFFFFFFF

100C  87654321  W
100C  AAAAAAAA  W
100C  BBBBBBBB  W
29_0000100C  87654321  W
29_00001010  10101010  W
29_00001010  FF101010  W
29_00001014  FF101014  W

1014  FF101014  R
29_00001014  FF101014  R
29_00001010  FF101010  R
29_00001010  FF101010  R

# --- MEM binary download (length 0 = whole file) ---
0x80002000  0  MEM  test_firmware.bin

# --- MEM binary download (first 0x20 = 32 bytes only) ---
0x80003000  0x20  MEM  test_firmware.bin

