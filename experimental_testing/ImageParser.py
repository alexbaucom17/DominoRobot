import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import skimage.transform as sktf
import numpy as np

# Configuration
image_name = 'MR.jpg'
desired_width = 300
desired_height = 300
dominoes = np.array(
            [('black', (0,0,0)),
             ('red',   (1,0,0)),
             ('blue',  (0,0,1)),
             ('green', (0,1,0)),
             ('white', (1,1,1))
             ])

# Load original image
img = mpimg.imread(image_name)

# Scaled image
img_scaled = sktf.resize(img, (desired_height, desired_width),anti_aliasing=False)

# Parse image into domino IDs by choosing 'closest' color
img_parsed_color = np.zeros((desired_width*desired_height, 3))
img_parsed_ids = np.zeros_like(img_parsed_color,dtype=np.int_)[:,0]
count = 0

for row in img_scaled:
    for px in row:
        best_id = -1
        best_score = 9999999
        for id,val in enumerate(dominoes):
            color = val[1]
            score = np.linalg.norm(px - color)
            if score < best_score:
                best_score = score
                best_id = id
        img_parsed_ids[count] = best_id
        img_parsed_color[count] = dominoes[best_id][1]
        count = count + 1

img_parsed_ids = img_parsed_ids.reshape((desired_height, desired_width))
img_parsed_color = img_parsed_color.reshape((desired_height, desired_width, 3))

# Output some metrics
print('Domino usage:')
print('Total number of dominoes: ' + str(img_parsed_ids.size))
print('Colors:')
unique_colors, counts = np.unique(img_parsed_ids, return_counts=True)
for i,id in enumerate(unique_colors):
    print('  ' + dominoes[id][0] + ': ' + str(counts[i]))

# Plot images
fig, axes = plt.subplots(nrows=1, ncols=3)
ax = axes.ravel()

ax[0].imshow(img)
ax[0].set_title('Original')
ax[1].imshow(img_scaled)
ax[1].set_title('Scaled')
ax[2].imshow(img_parsed_color)
ax[2].set_title('Dominoes')

plt.tight_layout()
plt.show()






