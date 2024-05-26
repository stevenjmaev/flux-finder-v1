# %%
MAX_VALUE = 65535
current = 0

single_px = 0xEF0000

max_data_len = 24 * 256 # 24 bits per px, 256 px's
# Note: bitrate is ~1MHz, so the max "framerate" is:
#  1MHz / (24*255) = 163 Hz.... That should be plenty fast

data = 0
for i in range(255, -1, -1):
    data |= (single_px) << (24 * i)

arr = []
i = True

current_bit = max_data_len
while current_bit > 0:
    if data & (1 << (current_bit - 1)): # it's a '1'
        width = 35 if not i else 15
    else: # it's a '0'
        width = 15 if not i else 35

    current = (current + width) % MAX_VALUE
    if not i:
        current_bit -= 1
    i = not i
    arr.append(current)

arr
# %%
print("{")

for i, a in enumerate(arr):
    print(f"{a}, ", end="")
    if i % 15 == 0 and i != 0:
        print("")

# %%
