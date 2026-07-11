# Test command file for svf_gen.py
# Mix of writes, reads without verify, and reads with verify (Y)
#
# Format: <addr_hex>  <data_hex>  <W|R>  [Y]

# --- Write some registers / memory locations ---
1000  DEADBEEF  R Y
1004  CAFEBABE  R Y
1008  12345678  R Y
100C  000000FF  W

# --- Read back and verify ---
1000  DEADBEEF  R  Y
1004  CAFEBABE  R  Y
100C  000000FF  R  Y

# --- Read without verification (TDO not checked) ---
1008  00000000  R
100C  00000000  R N

# --- Write to a different region ---
1A00  ABCD0001  W
1A04  ABCD0002  W
1A08  ABCD0003  W

# --- Read back and verify the second region ---
1A00  ABCD0001  R  Y
1A04  ABCD0002  R  Y
1A08  ABCD0003  R  Y

